#!/usr/bin/env python3
"""Zero-command AXIS bench recorder. Default is read-only; never enables drive.

--execute-unpowered explicitly permits BEGIN/ARM/START and fault injection.
Requires firmware honoring output_enable_used=false. No calibrated effort or
motion claims are made. Each run retains RX/TX bytes, frame arrival times,
decoded sample-major raw bytes, metadata and an integrity report (also on fault).
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import select
import struct
import time
import uuid
import termios
import tty

import axis_wire as wire


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


class Transport:
    """Single-threaded nonblocking pump: drains even with no request outstanding.

    Unlike the production FrameReader, rejects any corrupt frame instead of
    silently resynchronizing. RX bytes are preserved before validation.
    """
    def __init__(self, port, directory):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=False)
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.buf = bytearray()
        self.seq = 0
        self.responses = {}
        self.on_data = None
        self.rx = (directory / 'rx.bin').open('wb')
        self.tx = (directory / 'tx.bin').open('wb')
        self.frames = (directory / 'frames.jsonl').open('w')
        self.max_payload = 4096

    def pump(self):
        if not select.select([self.fd], [], [], .01)[0]:
            return
        chunk = os.read(self.fd, 16384)
        if not chunk:
            raise ConnectionError('serial connection closed')
        self.rx.write(chunk)
        self.rx.flush()
        self.buf.extend(chunk)
        while len(self.buf) >= 20:
            h = self.buf[:20]
            if h[:4] != b'AXIS' or h[4] != 1 or h[5] not in (2, 3, 4) or h[6:8] != b'\0\0':
                raise ValueError('invalid frame header (raw bytes retained)')
            seq, n, crc = struct.unpack_from('<III', h, 8)
            if n > self.max_payload:
                raise ValueError('oversized frame')
            if len(self.buf) < 20 + n:
                return
            payload = bytes(self.buf[20:20+n])
            del self.buf[:20+n]
            if wire.crc32(h[:16] + payload) != crc:
                raise ValueError('frame CRC mismatch')
            now = time.monotonic()
            record = dict(host_monotonic=now, kind=h[5], seq=seq, payload_bytes=n)
            if h[5] == wire.KIND_DATA:
                if seq != 0 or len(payload) < 24 or payload[21:24] != b'\0\0\0':
                    raise ValueError('invalid pushed DATA prefix')
                parsed = wire.parse_data_prefix(payload)
                record.update({k: v for k, v in parsed.items() if k != 'body'})
                if self.on_data:
                    self.on_data(parsed, now)
                else:
                    raise ValueError('unexpected capture data')
            else:
                record['json'] = json.loads(payload, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                if h[5] == wire.KIND_RESPONSE:
                    self.responses[seq] = record['json']
            self.frames.write(json.dumps(record) + '\n')
            self.frames.flush()

    def request(self, op, **fields):
        if op in ('ARM', 'START'):
            from axis_motion_guard import require_motion_unpaused
            require_motion_unpaused()
        self.seq += 1
        seq = self.seq
        packet = wire.pack_request(seq, op, **fields)
        if len(packet) - 20 > self.max_payload:
            raise ValueError('request exceeds negotiated maximum')
        self.tx.write(packet)
        self.tx.flush()
        view = memoryview(packet)
        write_deadline = time.monotonic() + 5
        while view:
            if time.monotonic() > write_deadline:
                raise TimeoutError('serial write timeout')
            if select.select([], [self.fd], [], .01)[1]:
                # Pace full-speed USB packets: an unpaced ~1KB BEGIN exposed
                # a core USB RX hard fault on this board. Keep the workaround
                # explicit; this does not certify arbitrary upload bursts.
                view = view[os.write(self.fd, view[:64]):]
                if view:
                    time.sleep(.002)
        deadline = time.monotonic() + 20
        while seq not in self.responses:
            if time.monotonic() > deadline:
                raise TimeoutError(f'{op} response lost; do not retry state-changing operation')
            self.pump()
        body = self.responses.pop(seq)
        if body.get('op') != op:
            raise RuntimeError(f'response op mismatch: {body}')
        if not body.get('ok'):
            raise wire.AxisDeviceError(op, body.get('error'), body.get('detail'))
        return body

    def close(self):
        os.close(self.fd)
        for f in (self.rx, self.tx, self.frames):
            f.close()


def decode_capture(codec, channels, count, body):
    """Strict lengths, bounded RLE expansion, and checked i64 arithmetic."""
    if count <= 0 or count > 1000000 or channels <= 0:
        raise ValueError('invalid capture dimensions')
    if codec in (1, 3):
        if len(body) != count * channels * (8 if codec == 1 else 4):
            raise ValueError('raw capture byte length mismatch')
        return wire.decode_block(codec, channels, count, body)
    if codec != 2 or channels != 1 or len(body) < 8 or len(body) % 8:
        raise ValueError('invalid RLE capture shape')
    value, = struct.unpack_from('<q', body)
    samples = [(value,)]
    for offset in range(8, len(body), 8):
        repeat, delta = struct.unpack_from('<Ii', body, offset)
        if repeat == 0 or repeat > count - len(samples):
            raise ValueError('invalid RLE repeat count')
        final = value + repeat * delta
        if not -(1 << 63) <= final < (1 << 63):
            raise ValueError('RLE i64 overflow')
        for _ in range(repeat):
            value += delta
            samples.append((value,))
    if len(samples) != count:
        raise ValueError('RLE count mismatch')
    return samples


class Stream:
    def __init__(self, path, channels, period, width):
        self.file = path.open('wb')
        self.channels, self.period, self.width = channels, period, width
        self.sha = hashlib.sha256()
        self.count = self.frames = 0
        self.first_tick = self.next_tick = None
        self.first_arrival = self.last_arrival = None
        self.max_arrival_gap = 0
        self.codec_counts = {}
        self.minimum = self.maximum = None

    def accept(self, p, arrival):
        if p['push_seq'] != self.frames or p['sample_count'] <= 0:
            raise ValueError('chunk sequence/count violation')
        if self.next_tick is not None and p['first_sample_tick'] != self.next_tick:
            raise ValueError('capture timestamp gap/overlap')
        if (self.width == 4 and p['codec'] != 3) or (self.width == 8 and p['codec'] not in (1, 2)):
            raise ValueError('unexpected stream codec')
        samples = decode_capture(p['codec'], self.channels, p['sample_count'], p['body'])
        fmt = '<' + ('q' if self.width == 8 else 'i') * self.channels
        for sample in samples:
            raw = struct.pack(fmt, *sample)
            self.file.write(raw)
            self.sha.update(raw)
            if self.minimum is None:
                self.minimum = list(sample)
                self.maximum = list(sample)
            else:
                self.minimum = [min(a, b) for a, b in zip(self.minimum, sample)]
                self.maximum = [max(a, b) for a, b in zip(self.maximum, sample)]
        if self.first_tick is None:
            self.first_tick = p['first_sample_tick']
            self.first_arrival = arrival
        if self.last_arrival is not None:
            self.max_arrival_gap = max(self.max_arrival_gap, arrival - self.last_arrival)
        self.last_arrival = arrival
        self.next_tick = p['first_sample_tick'] + p['sample_count'] * self.period
        self.count += len(samples)
        self.frames += 1
        self.codec_counts[p['codec']] = self.codec_counts.get(p['codec'], 0) + 1

    def report(self):
        return dict(sample_count=self.count, frames_pushed=self.frames, sha256=self.sha.hexdigest(),
                    first_tick=self.first_tick, next_tick=self.next_tick,
                    first_arrival=self.first_arrival, last_arrival=self.last_arrival,
                    max_arrival_gap_seconds=self.max_arrival_gap,
                    codecs=self.codec_counts, minimum=self.minimum, maximum=self.maximum)


def health(t):
    hello = t.request('HELLO')
    t.max_payload = min(4096, hello['max_payload'])
    statuses = []
    for _ in range(4):
        statuses.append(t.request('STATUS'))
        time.sleep(.05)
    save(t.directory / 'health.json', dict(hello=hello, statuses=statuses))
    if any(s['boot_id'] != hello['boot_id'] for s in statuses):
        raise ValueError('boot changed')
    ticks = [s['tick'] for s in statuses]
    if any(b <= a for a, b in zip(ticks, ticks[1:])):
        raise ValueError('clock did not advance monotonically')
    if statuses[-1]['state'] != 'IDLE' or statuses[-1]['output_enabled']:
        raise ValueError('device must be IDLE with output disabled; no automatic CLEAR')
    return hello


def record(t, hello, name, encoders, analog, period, duration, injection=None,
           command_values=None, post_roll_ticks=100000, travel_limit_counts=None,
           command_ceiling_code=3276, servo=None):
    directory = t.directory / name
    directory.mkdir()
    hz = hello['tick_hz']
    if type(post_roll_ticks) is not int or not 0 <= post_roll_ticks <= 10 * hz:
        raise ValueError('post-roll must be integer ticks in [0, 10 seconds]')
    if travel_limit_counts is not None and (
            type(travel_limit_counts) is not int or not 0 < travel_limit_counts < (1 << 31)):
        raise ValueError('travel limit must be a positive signed i32 count')
    run_id = uuid.uuid4().hex
    # CLI remains zero-only. Library callers may upload an explicitly authorized
    # bounded pulse train; the whole command buffer is sealed before ARM.
    if type(command_ceiling_code) is not int or not 0 < command_ceiling_code <= 32767:
        raise ValueError('explicit command ceiling must be an integer in 1..32767')
    if servo is not None:
        from axis_servo_config import validate_servo
        validate_servo(servo)
        if 'analog_position_servo' not in hello.get('output_modes', []):
            raise ValueError('installed firmware has no live position controller; no motion attempted')
        if analog or encoders != [0] or travel_limit_counts is None or command_ceiling_code != servo['torque_limit_code']:
            raise ValueError('servo requires encoder0, no ADC, travel guard, and matching effort ceiling')
    value_ceiling = 4096 if servo is not None else command_ceiling_code
    values = [0] * 100 if command_values is None else list(command_values)
    n = len(values)
    if (not 2 <= n <= min(600, hello.get('max_command_samples', 600))
            or any(type(v) is not int or abs(v) > value_ceiling for v in values)
            or values[0] != 0 or values[-1] != 0):
        raise ValueError(f'bench commands require 2..600 integers, |code| <= {command_ceiling_code}, zero endpoints')
    segments = []
    segment_start = 0
    for i in range(1, n + 1):
        if i == n or values[i] != values[segment_start]:
            segments.append(dict(name='pulse' if values[segment_start] else 'zero',
                                 start=segment_start, end=i))
            segment_start = i
    if len(segments) > 8:
        # Segments are analysis labels, not waveform compression. Preserve every
        # command sample while using one label for a multi-pulse experiment.
        segments = [dict(name='pulse_train', start=0, end=n)]
    command_period = math.ceil(duration * hz / n / 5) * 5
    if servo is not None:
        command_period = round(duration * hz / n / 1000) * 1000
        if command_period < 1000 or abs(command_period - duration * hz / n) > 1:
            raise ValueError('servo reference period must be an integer number of milliseconds')
    watchdog = int((2 if injection == 'watchdog' else 10) * hz)
    manifest = dict(output_mode='analog_effort', drive_interpretation='effort',
                    command_sample_count=n, command_period_ticks=command_period,
                    capture_pre_roll_ticks=100000, capture_post_roll_ticks=post_roll_ticks,
                    output_enable_used=False, enable_active_high=True,
                    analog_output_full_scale=32767, safe_action='disable_drive',
                    connection_policy='supervised', watchdog_timeout_ticks=watchdog,
                    arming_timeout_ticks=int(5 * hz),
                    encoders=[dict(enabled=i in encoders, decode_mult=4, invert=False,
                                   travel_limit_enabled=travel_limit_counts is not None and i in encoders,
                                   travel_limit_counts=travel_limit_counts or 0) for i in range(2)],
                    encoder_sample_period_ticks=period, analog_enabled=analog,
                    analog_sample_period_ticks=max(period, hello['analog_min_period_ticks']),
                    segments=segments)
    if servo is not None:
        manifest.update(output_mode='analog_position_servo', drive_interpretation='position',
                        analog_sample_period_ticks=1000)
        manifest.update({'servo_'+k:v for k,v in servo.items()})
    save(directory / 'plan.json', dict(run_id=run_id, manifest=manifest, commands=values,
                                      authorized_command_ceiling_code=command_ceiling_code,
                                      injection=injection))
    streams = {i: Stream(directory / f'stream-{i}.i64le', 1, period, 8) for i in encoders}
    if analog or servo is not None:
        streams[2] = Stream(directory / 'stream-2.i32le', 2, manifest['analog_sample_period_ticks'], 4)
    report = dict(name=name, injection=injection, errors=[])
    terminal = False
    start_host = None

    def on_data(p, now):
        if p['stream_id'] not in streams:
            raise ValueError('unexpected stream id')
        streams[p['stream_id']].accept(p, now)
    t.on_data = on_data
    try:
        begin = t.request('BEGIN', run_id=run_id, manifest=manifest)
        save(directory / 'begin.json', begin)
        for offset in range(0, n, 64):
            t.request('PUT_COMMAND', run_id=run_id, offset=offset,
                      values=[str(v) for v in values[offset:offset+64]])
        t.request('SEAL', run_id=run_id, sha256=wire.command_sha256(values))
        t.request('ARM', run_id=run_id)
        armed = t.request('STATUS')
        save(directory / 'armed.json', armed)
        if armed['output_enabled']:
            raise ValueError('ARM asserted output enable despite capture-only configuration')
        if armed['state'] != 'ARMED':
            raise ValueError(f"ARM did not remain ARMED: {armed}")
        start_host = time.monotonic()
        start = t.request('START', run_id=run_id)
        save(directory / 'start.json', start)
        next_poll = start_host
        injected = False
        deadline = start_host + duration * 4 + 30
        statuses = []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if injection and not injected and now - start_host >= .3:
                injected = True
                if injection == 'abort':
                    t.request('ABORT', run_id=run_id)
                elif injection.startswith('pause:'):
                    time.sleep(float(injection.split(':')[1]))
            if now >= next_poll:
                status = t.request('STATUS')
                statuses.append(dict(host_monotonic=time.monotonic(), **status))
                if status['boot_id'] != hello['boot_id'] or status['output_enabled']:
                    raise ValueError('boot changed or output enable unexpectedly asserted')
                if status['state'] in ('COMPLETE', 'FAULT'):
                    terminal = True
                    break
                if injection != 'watchdog':
                    try:
                        t.request('KEEPALIVE', run_id=run_id)
                    except wire.AxisDeviceError as exc:
                        # Completion may occur after STATUS but before KEEPALIVE.
                        if exc.code != 'STATE':
                            raise
                        status = t.request('STATUS')
                        statuses.append(dict(host_monotonic=time.monotonic(), **status))
                        if status['state'] not in ('COMPLETE', 'FAULT'):
                            raise
                        terminal = True
                        break
                next_poll = time.monotonic() + .02
            else:
                t.pump()
        if not terminal:
            raise TimeoutError('run did not terminate')
        save(directory / 'statuses.json', statuses)
        # Continue draining after termination; snapshot final metadata only after
        # all streams' final partial chunks have had a chance to flush.
        drain_end = time.monotonic() + .3
        while time.monotonic() < drain_end:
            t.pump()
        run = t.request('GET_RUN', run_id=run_id)
        save(directory / 'get_run.json', run)
        if len(statuses) >= 2:
            elapsed = statuses[-1]['host_monotonic'] - statuses[0]['host_monotonic']
            if elapsed > .5:
                ratio = (statuses[-1]['tick'] - statuses[0]['tick']) / hz / elapsed
                report['device_to_host_clock_ratio'] = ratio
                # A coarse bench check, not an aperture/jitter certification.
                if not injection and abs(ratio - 1) > .03:
                    report['errors'].append('device clock differs from host elapsed time by >3%')
        report['state'] = status['state']
        report['fault_reason'] = run.get('fault_reason')
        if run['manifest_hash'] != begin['manifest_hash']:
            report['errors'].append('manifest hash mismatch')
        if {s['stream_id'] for s in run['streams']} != set(streams):
            report['errors'].append('stream descriptor set mismatch')
        for desc in run['streams']:
            sid = desc['stream_id']
            if sid not in streams:
                continue
            actual = streams[sid].report()
            for key in ('sample_count', 'frames_pushed', 'sha256'):
                if actual[key] != desc[key]:
                    report['errors'].append(f'stream {sid}: {key} mismatch')
            if desc['channels'] != streams[sid].channels or desc['period_ticks'] != streams[sid].period:
                report['errors'].append(f'stream {sid}: descriptor mismatch')
            if actual['first_tick'] != start['capture_start_tick']:
                report['errors'].append(f'stream {sid}: wrong capture start tick')
            if status['state'] == 'COMPLETE':
                expected = (run['run_end_tick'] - start['capture_start_tick']) // streams[sid].period + 1
                if actual['sample_count'] != expected:
                    report['errors'].append(f'stream {sid}: capture duration/count mismatch')
        report['live_data_before_command_end'] = any(
            s.first_arrival is not None and s.first_arrival < start_host + duration for s in streams.values())
        if not injection:
            if status['state'] != 'COMPLETE' or not run['capture_complete']:
                report['errors'].append('normal run faulted/incomplete')
            if not report['live_data_before_command_end']:
                report['errors'].append('no live data during run')
        elif injection in ('watchdog', 'abort'):
            expected = 'WATCHDOG' if injection == 'watchdog' else 'HOST_ABORT'
            if status['state'] != 'FAULT' or run.get('fault_reason') != expected:
                report['errors'].append(f'expected FAULT/{expected}')
        elif injection and injection.startswith('pause:'):
            if status['state'] == 'FAULT' and run.get('fault_reason') != 'OVERFLOW':
                report['errors'].append('read pause fault was not OVERFLOW')
            elif status['state'] == 'COMPLETE' and not run['capture_complete']:
                report['errors'].append('read pause completed with incomplete capture')
        if report['errors']:
            raise ValueError('capture validation failed; preserving terminal state, no automatic CLEAR')
        t.request('CLEAR', run_id=run_id)
        after = t.request('STATUS')
        if after['state'] != 'IDLE' or after['output_enabled']:
            report['errors'].append('not safely IDLE after CLEAR')
    except Exception as exc:
        report['errors'].append(f'{type(exc).__name__}: {exc}')
        # Never retry START. Attempt ABORT on any uncertain active state, then
        # leave state/data intact for inspection rather than automatically CLEAR.
        try:
            # Keep recording frames but stop decoding an already-invalid stream
            # so further pushed DATA cannot prevent supervision/ABORT.
            t.on_data = lambda p, now: None
            after = t.request('STATUS')
            report['status_after_error'] = after
            if after['state'] in ('UPLOADING', 'READY', 'ARMED', 'RUNNING'):
                t.request('ABORT', run_id=run_id)
                after = t.request('STATUS')
                report['status_after_error'] = after
            if after['state'] in ('FAULT', 'COMPLETE'):
                save(directory / 'get_run_after_error.json', t.request('GET_RUN', run_id=run_id))
        except Exception as cleanup:
            report['cleanup_error'] = str(cleanup)
    finally:
        report['streams'] = {sid: s.report() for sid, s in streams.items()}
        report['passed'] = not report['errors']
        for s in streams.values():
            s.file.close()
        save(directory / 'report.json', report)
        t.on_data = None
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute-unpowered', action='store_true')
    parser.add_argument('--from-case', help='resume matrix at this named case (device must be IDLE)')
    parser.add_argument('--encoder-only', action='store_true', help='disable ADC for every test')
    args = parser.parse_args()
    t = Transport(args.port, args.output)
    try:
        hello = health(t)
        print(json.dumps(hello, indent=2), flush=True)
        if not args.execute_unpowered:
            return
        cases = [('smoke', [0, 1], True, 1000, 2, None),
                 ('long', [0, 1], True, 1000, 15, None)]
        for period in (1000, 500, 250):
            for label, enc, adc in [('enc0', [0], False), ('enc1', [1], False),
                                    ('encoders', [0, 1], False), ('adc', [], True),
                                    ('all', [0, 1], True)]:
                cases.append((f'{label}-{period}us', enc, adc, period, 3, None))
        for pause in (.01, .05, .2, 1., 3.):
            cases.append((f'pause-{pause}', [0, 1], True, 250, 3, f'pause:{pause}'))
        cases.extend([('watchdog', [0, 1], True, 1000, 5, 'watchdog'),
                      ('abort', [0, 1], True, 1000, 3, 'abort')])
        if args.encoder_only:
            cases = [(name, enc, False, period, duration, injection)
                     for name, enc, adc, period, duration, injection in cases
                     if enc and not name.startswith('all-')]
        if args.from_case:
            names = [case[0] for case in cases]
            if args.from_case not in names:
                raise ValueError(f'unknown case {args.from_case}')
            cases = cases[names.index(args.from_case):]
        reports = []
        for case in cases:
            print('RUN', case[0], flush=True)
            result = record(t, hello, *case)
            reports.append(result)
            save(args.output / 'summary.json', reports)
            print(json.dumps(result), flush=True)
            # Failures with final metadata are useful rate-limit measurements;
            # uncertain state or failed cleanup must stop the suite.
            if 'status_after_error' in result or 'cleanup_error' in result:
                break
        if any(not r['passed'] for r in reports):
            raise SystemExit(1)
    finally:
        t.close()


if __name__ == '__main__':
    main()
