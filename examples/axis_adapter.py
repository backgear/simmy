#!/usr/bin/env python3
"""Host-side adapter for the single-axis characterization device
(docs/axis_device_protocol.md / characterization_rig firmware): speaks
the AXIS wire protocol over a real serial port and writes the CSV
examples/identify_axis.py's `capture` step expects.

NOT YET RUN AGAINST REAL HARDWARE. This has only been exercised against
the in-process loopback fixture in axis_fake_device.py (see
tests/test_axis_adapter.py) -- the protocol doc's own "Acceptance checks
before connecting a machine" section asks for exactly that before ever
opening a serial port. Treat every physical value in your machine config
(port, counts_per_unit, effort calibration, safe_action, travel limits)
as something a real bench setup must supply correctly, not a default to
trust; `axis_wire.open_serial`'s raw-mode handling is itself unverified
against a real USB-CDC device.

Reconciled against the REAL firmware (characterization_rig/src/
protocol.cpp), not just the prose doc, which describes a few manifest
fields this firmware does not actually have:
  - No per-channel command/encoder "scale, offset, unit" or encoder
    "motor/load/intermediate role" fields exist on the wire. The device
    only ever sees/returns raw integer counts and codes; ALL physical
    unit conversion and encoder-role assignment is this adapter's job,
    driven by --config (below), not a manifest field.
  - No canonical-JSON (RFC 8785) manifest serialization is replicated
    here: BEGIN's response already returns manifest_hash, and GET_RUN
    echoes it again, so this adapter just checks the two match rather
    than recomputing a bit-for-bit independent hash of its own request.
  - delta_rle_i32 (codec 2) is single-channel only in this firmware
    (ch!=1 is simply unsupported), and its first sample is 8 bytes (i64),
    not `channels` i64 values as the prose doc's generic description
    might suggest for a multi-channel stream.
  - "Required host artifact" (replay bundle) in the doc is explicitly
    "not yet implemented by the current validation-only prototype" --
    this adapter writes a <output>.manifest.json next to the CSV with
    enough raw material (manifest, hashes, segments, start ticks, the
    full uploaded command array) to build that later, but does not
    implement full continuous-run replay itself.
  - GET_BLOCK does not exist: this firmware never buffers a whole run.
    Every capture stream pushes itself off continuously, unsolicited, as
    soon as a small chunk is ready (DATA frames with sequence=0), reusing
    the USB CDC stack's own small queue instead of a second on-device
    buffer -- see run_buffer.h's file header. This adapter accumulates
    those pushed chunks via an `on_push` callback while it polls
    STATUS/KEEPALIVE waiting for COMPLETE/FAULT, tracking each stream's
    own push-sequence counter to catch a dropped/reordered chunk
    immediately, then cross-checks the fully-accumulated result against
    GET_RUN's authoritative sample_count/frames_pushed/sha256 once the
    run ends -- the same end-to-end integrity guarantee the old
    pull-after-COMPLETE design had, just assembled as data arrives
    instead of fetched afterward.
These are implementation facts, not a claim that the doc is wrong --
worth reconciling the doc text itself at some point, but that's a
separate decision from this file.

Usage:
    axis_adapter.py --plan PLAN.csv --output OUT.csv \\
        --config MACHINE.json [--port /dev/ttyACM0]

`examples/identify_axis.py capture --adapter ...` only ever invokes an
adapter as `adapter --plan ABS_PATH --output ABS_PATH` (no --config/
--port). To use this file unmodified as that --adapter target, set the
AXIS_CONFIG and AXIS_PORT environment variables instead of passing
--config/--port; both forms work identically otherwise.

Machine config JSON (keys with a default below may be omitted):
    output_mode              "step_dir" | "quadrature_command" | "analog_effort"  [required]
    counts_per_unit          float: command/encoder counts per plan position unit [required]
    command_period_ticks     int: device ticks per command sample              [required]
    safe_action              "disable_drive" | "zero_output" | "stop_pulses"    [required]
    encoder_motor            0 or 1: which encoder stream is motor_position     [required]
    encoder_load             0 or 1, or omitted for a single-body rig
    encoder_counts_per_unit  defaults to counts_per_unit if omitted
    effort_channel           0 or 1: analog channel carrying calibrated measured
                             effort, or omitted to refuse writing an `effort`
                             column at all -- never silently substitute command
    effort_scale, effort_offset   physical = raw_code * effort_scale + effort_offset  [0.0, 1.0 if effort_channel set]
    drive_interpretation     defaults to "effort" for analog_effort, else "position"
    analog_output_full_scale  int, analog_effort only                           [32767]
    capture_pre_roll_ticks, capture_post_roll_ticks                             [0, 0]
    encoder_sample_period_ticks, analog_sample_period_ticks
    watchdog_timeout_ticks, arming_timeout_ticks, connection_policy
    step_pulse_width_ticks, dir_setup_ticks, dir_hold_ticks,
    output_idle_level_high, enable_active_high, output_enable_used, quadrature_invert
See docs/axis_device_protocol.md for what each of these means on the wire.
"""
import argparse
import csv
import hashlib
import json
import os
import struct
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import axis_wire as wire  # noqa: E402


MEASURE_FIELDS = ['time', 'segment', 'command', 'effort', 'motor_position']


def read_plan(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, fields, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plan_segments(rows):
    """Contiguous (name, start, end) command-index ranges, half-open --
    the device only sees index ranges, never the plan's own per-row
    segment label."""
    segments = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]['segment'] != rows[start]['segment']:
            segments.append((rows[start]['segment'], start, i))
            start = i
    return segments


def plan_to_counts(rows, config):
    """Converts the plan's `target` (+ `excitation` for analog_effort)
    into the cumulative integer command stream this output_mode wants.
    Position-command drives cannot simply add an effort chirp to a pulse
    count (docs/axis_device_protocol.md, "Host adapter and existing
    fitter"): refuse rather than silently drop or misapply it."""
    mode = config['output_mode']
    scale = config['counts_per_unit']
    if mode == 'analog_effort':
        full_scale = config.get('analog_output_full_scale', 32767)
        counts = []
        for row in rows:
            code = int(round((float(row['target']) + float(row['excitation'])) * scale))
            if abs(code) > full_scale:
                raise ValueError(f'command code {code} exceeds analog_output_full_scale {full_scale}')
            counts.append(code)
        return counts
    if any(float(row.get('excitation', 0)) != 0 for row in rows):
        raise ValueError(
            f"output_mode {mode!r} is a position-command drive: it cannot inject the "
            "plan's `excitation` effort chirp through a pulse count. Use output_mode "
            "'analog_effort', a plan with no dynamic-segment excitation, or a supported "
            "torque injection facility.")
    return [int(round(float(row['target']) * scale)) for row in rows]


def build_manifest(config, rows):
    segments = plan_segments(rows)
    mode = config['output_mode']
    for required in ('output_mode', 'counts_per_unit', 'command_period_ticks', 'safe_action', 'encoder_motor'):
        if required not in config:
            raise ValueError(f'machine config missing required key {required!r}')
    manifest = {
        'drive_interpretation': config.get('drive_interpretation') or
            ('effort' if mode == 'analog_effort' else 'position'),
        'output_mode': mode,
        'command_sample_count': len(rows),
        'command_period_ticks': int(config['command_period_ticks']),
        'capture_pre_roll_ticks': int(config.get('capture_pre_roll_ticks', 0)),
        'capture_post_roll_ticks': int(config.get('capture_post_roll_ticks', 0)),
        'safe_action': config['safe_action'],
        'connection_policy': config.get('connection_policy', 'supervised'),
        'segments': [{'name': name, 'start': s, 'end': e} for name, s, e in segments],
    }
    passthrough = ('step_pulse_width_ticks', 'dir_setup_ticks', 'dir_hold_ticks',
                   'output_idle_level_high', 'enable_active_high', 'output_enable_used',
                   'quadrature_invert', 'analog_output_full_scale',
                   'encoder_sample_period_ticks', 'analog_sample_period_ticks',
                   'watchdog_timeout_ticks', 'arming_timeout_ticks')
    for key in passthrough:
        if key in config:
            manifest[key] = config[key]
    motor_idx, load_idx = config['encoder_motor'], config.get('encoder_load')
    invert = config.get('encoder_invert', [False, False])
    encoders = []
    for i in range(2):
        enabled = i in (motor_idx, load_idx)
        encoders.append({'enabled': enabled, 'decode_mult': config.get('encoder_decode_mult', 4),
                          'invert': invert[i] if enabled else False})
    manifest['encoders'] = encoders
    manifest['analog_enabled'] = config.get('effort_channel') is not None
    return manifest


def _interp(ticks, values, t):
    """Linear interpolation; holds the nearest end value outside range.
    `ticks` must be sorted. Plain stdlib bisect, no numpy -- a single-axis
    identification run's analog stream is a few thousand samples at most,
    not a scale where bisect's O(log n) lookup needs help from numpy."""
    import bisect
    if not ticks:
        return 0.0
    i = bisect.bisect_left(ticks, t)
    if i <= 0:
        return float(values[0])
    if i >= len(ticks):
        return float(values[-1])
    t0, t1 = ticks[i - 1], ticks[i]
    v0, v1 = values[i - 1], values[i]
    if t1 == t0:
        return float(v0)
    frac = (t - t0) / (t1 - t0)
    return v0 + frac * (v1 - v0)


def decode_to_measurements(rows, counts, config, run, streams):
    """Builds CSV rows on the motor encoder's own sample grid (sampling
    need not match the plan rate -- see docs/axis_identification.md),
    with `effort` linearly interpolated onto that grid from the analog
    stream's independent clock."""
    command_start = run['command_start_tick']
    period = int(config['command_period_ticks'])
    segments = plan_segments(rows)
    scale = config['counts_per_unit']
    motor_scale = config.get('encoder_counts_per_unit', scale)
    tick_hz = run.get('tick_hz', 1_000_000)

    motor = streams[config['encoder_motor']]
    load = streams.get(config.get('encoder_load')) if config.get('encoder_load') is not None else None
    effort_channel = config.get('effort_channel')
    analog = streams.get(wire_analog_stream_id(config)) if effort_channel is not None else None

    analog_ticks = analog_values = None
    if analog is not None:
        analog_ticks = [analog['first_sample_tick'] + i * analog['period_ticks']
                         for i in range(len(analog['samples']))]
        analog_values = [s[effort_channel] for s in analog['samples']]

    def segment_at(index):
        for name, start, end in segments:
            if start <= index < end:
                return name
        return 'unsegmented'

    out = []
    for i, frame in enumerate(motor['samples']):
        tick = motor['first_sample_tick'] + i * motor['period_ticks']
        t = (tick - command_start) / tick_hz
        if tick < command_start:
            segment = 'preroll'
            command_value = 0
        elif tick >= command_start + len(counts) * period:
            segment = 'postroll'
            command_value = counts[-1]
        else:
            index = min((tick - command_start) // period, len(counts) - 1)
            segment = segment_at(index)
            command_value = counts[index]
        row = {
            'time': t,
            'segment': segment,
            'command': command_value / scale,
            'motor_position': frame[0] / motor_scale,
        }
        if analog_ticks is not None:
            raw_effort = _interp(analog_ticks, analog_values, tick)
            row['effort'] = raw_effort * config.get('effort_scale', 1.0) + config.get('effort_offset', 0.0)
        else:
            row['effort'] = None
        if load is not None:
            load_scale = config.get('load_counts_per_unit', scale)
            load_index = min(i * motor['period_ticks'] // load['period_ticks'], len(load['samples']) - 1)
            row['load_position'] = load['samples'][load_index][0] / load_scale
        out.append(row)
    return out


def wire_analog_stream_id(config):
    return 2  # STREAM_ANALOG; fixed by this firmware's NUM_ENCODERS=2 layout


def enabled_stream_shapes(config):
    """stream_id -> {channels, codec}, matching capture_streams_service's
    fixed choice in protocol.cpp exactly (not negotiated on the wire):
    encoder streams (0/1) are single-channel raw_i64, the analog stream
    (2, fixed NUM_ANALOG_CHANNELS=2 on this firmware) is raw_i32."""
    motor_idx, load_idx = config['encoder_motor'], config.get('encoder_load')
    shapes = {}
    for i in (0, 1):
        if i in (motor_idx, load_idx):
            shapes[i] = dict(channels=1, codec=1)
    if config.get('effort_channel') is not None:
        shapes[wire_analog_stream_id(config)] = dict(channels=2, codec=3)
    return shapes


def capture(plan_path, output_path, config, read_fn, write_fn):
    rows = read_plan(plan_path)
    if not rows:
        raise ValueError('empty plan')
    if config.get('effort_channel') is None:
        raise ValueError(
            "machine config has no effort_channel configured: effort must be a calibrated "
            "measurement, never inferred from the command (docs/axis_device_protocol.md, "
            "'Host adapter and existing fitter'). Configure a real current/torque-sense "
            "analog channel before capturing.")

    shapes = enabled_stream_shapes(config)
    accum = {sid: {'samples': [], 'next_seq': 0} for sid in shapes}

    def handle_push(parsed):
        sid = parsed['stream_id']
        state = accum.get(sid)
        if state is None:
            return  # a stream we never enabled; shouldn't happen, ignore defensively
        if parsed['push_seq'] != state['next_seq']:
            raise RuntimeError(
                f"stream {sid}: push sequence gap -- expected chunk {state['next_seq']}, "
                f"got {parsed['push_seq']} (a chunk was lost, corrupted, or reordered in transit)")
        state['next_seq'] += 1
        shape = shapes[sid]
        state['samples'].extend(
            wire.decode_block(parsed['codec'], shape['channels'], parsed['sample_count'], parsed['body']))

    client = wire.AxisClient(write_fn, read_fn, on_push=handle_push)
    _, hello = client.request('HELLO')
    if config['output_mode'] not in hello['output_modes']:
        raise ValueError(f"device does not support output_mode {config['output_mode']!r}")
    tick_hz = hello['tick_hz']

    run_id = uuid.uuid4().hex
    manifest = build_manifest(config, rows)
    _, begin = client.request('BEGIN', run_id=run_id, manifest=manifest)
    manifest_hash = begin['manifest_hash']

    counts = plan_to_counts(rows, config)
    chunk_size = 64
    for offset in range(0, len(counts), chunk_size):
        chunk = counts[offset:offset + chunk_size]
        client.request('PUT_COMMAND', run_id=run_id, offset=offset, values=[str(v) for v in chunk])

    client.request('SEAL', run_id=run_id, sha256=wire.command_sha256(counts))
    client.request('ARM', run_id=run_id)
    _, start = client.request('START', run_id=run_id)

    # Every request() call drains and dispatches any pushed capture chunks
    # buffered since the last one (on_push above), so this STATUS/KEEPALIVE
    # poll loop -- otherwise unchanged from a pull-based design -- is also
    # what accumulates the whole capture as the run executes.
    deadline = time.monotonic() + 120  # generous; real single-axis experiments run in seconds
    status = None
    while True:
        _, status = client.request('STATUS')
        if status['state'] in ('COMPLETE', 'FAULT'):
            break
        if time.monotonic() > deadline:
            raise TimeoutError('device never reached COMPLETE/FAULT')
        client.request('KEEPALIVE', run_id=run_id)
        time.sleep(0.05)

    # The device flushes each stream's final, possibly-partial chunk on its
    # *next* protocol_poll() tick after reaching COMPLETE/FAULT, not before
    # -- one more round trip guarantees that's been dispatched to on_push
    # before GET_RUN's counts/hash are trusted as final.
    client.request('STATUS')

    _, run = client.request('GET_RUN', run_id=run_id)
    if run['manifest_hash'] != manifest_hash:
        raise ValueError('GET_RUN manifest_hash does not match BEGIN: device state is suspect')
    run['tick_hz'] = tick_hz
    run.update(start)

    fault_path = Path(str(output_path) + '.fault.json')
    if status['state'] == 'FAULT' or not run['capture_complete']:
        fault_path.write_text(json.dumps(dict(status=status, run=run), indent=2))
        client.request('CLEAR', run_id=run_id)
        raise RuntimeError(
            f"run faulted or incomplete ({run.get('fault_reason')}); diagnostic written to "
            f"{fault_path}, not {output_path}")

    streams = {}
    for stream in run['streams']:
        sid, channels, count = stream['stream_id'], stream['channels'], stream['sample_count']
        state = accum.get(sid)
        if state is None:
            raise ValueError(f'GET_RUN reports stream {sid} but no data was ever pushed for it')
        samples = state['samples']
        if len(samples) != count:
            raise ValueError(f'stream {sid}: received {len(samples)} pushed samples, '
                              f'GET_RUN declared {count}')
        if state['next_seq'] != stream['frames_pushed']:
            raise ValueError(f"stream {sid}: received {state['next_seq']} push chunks, "
                              f"GET_RUN declared {stream['frames_pushed']}")
        widen = channels == 1
        digest = wire.stream_sha256(samples, widen)
        if digest != stream['sha256']:
            raise ValueError(f'stream {sid}: received data does not match its declared sha256 '
                              '-- a chunk arrived corrupted despite passing its own frame CRC')
        streams[sid] = dict(stream, samples=samples)

    client.request('CLEAR', run_id=run_id)

    measured_rows = decode_to_measurements(rows, counts, config, run, streams)
    fields = list(MEASURE_FIELDS)
    if config.get('encoder_load') is not None:
        fields.append('load_position')
    write_csv(output_path, fields, measured_rows)
    Path(str(output_path) + '.manifest.json').write_text(json.dumps(
        dict(run_id=run_id, manifest=manifest, manifest_hash=manifest_hash, command_samples=counts,
             get_run={k: v for k, v in run.items() if k != 'tick_hz'}), indent=2))


def load_config(args):
    config_path = args.config or os.environ.get('AXIS_CONFIG')
    if not config_path:
        raise ValueError('no machine config given: pass --config or set AXIS_CONFIG')
    config = json.loads(Path(config_path).read_text())
    port = args.port or os.environ.get('AXIS_PORT') or config.get('port')
    if not port:
        raise ValueError('no serial port given: pass --port, set AXIS_PORT, or config["port"]')
    return config, port


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--port')
    args = parser.parse_args()
    try:
        from axis_motion_guard import require_motion_unpaused
        require_motion_unpaused()
        config, port = load_config(args)
        read_fn, write_fn, close_fn = wire.open_serial(port)
        try:
            capture(args.plan, args.output, config, read_fn, write_fn)
        finally:
            close_fn()
    except Exception as exc:  # noqa: BLE001 -- single top-level diagnostic exit, like identify_axis.py
        parser.exit(1, f'error: {exc}\n')


if __name__ == '__main__':
    main()
