"""An in-process stand-in for the real characterization_rig firmware,
implementing the same AXIS wire protocol (axis_wire.py) and state
machine as characterization_rig/src/protocol.cpp closely enough to drive
axis_adapter.py end-to-end with no physical board attached.

This is what the protocol doc's own "Acceptance checks before connecting
a machine" asks for: "exercise the complete upload -> run -> retrieve ->
decode -> fit -> plot path with a simulator or loopback fixture" before
ever opening a serial port. It is NOT a claim that real firmware behaves
identically in every corner case -- only that the wire framing, op
sequence, manifest defaults, and codecs match what protocol.cpp actually
does (see axis_adapter.py's docstring for the specific places checked).

The simulated axis is deliberately trivial (an encoder that reads back
the commanded cumulative pulse count after a fixed tick delay, plus one
analog channel reporting a synthetic effort proportional to commanded
velocity): this file's job is to exercise the ADAPTER's wire-level
plumbing, not to re-implement simmy's own mechanical models a second
time. Real dynamics belong in simmy; this is a protocol fixture.
"""
import json
import struct
import threading
import time

import axis_wire as wire

TICK_HZ = 1_000_000
TICKS_PER_SERVICE = 5
NUM_ENCODERS = 2
NUM_ANALOG_CHANNELS = 2
NUM_CAPTURE_STREAMS = NUM_ENCODERS + 1
STREAM_ANALOG = NUM_ENCODERS
MAX_COMMAND_SAMPLES = 600
PUSH_CHUNK_SAMPLES = 32  # matches protocol.cpp's capture_streams_service chunk size


class _CommandBuffer:
    def __init__(self):
        self.declared_count = 0
        self.values = []
        self.filled = []

    def reset(self, declared_count):
        self.declared_count = declared_count
        self.values = [0] * declared_count
        self.filled = [False] * declared_count

    def put(self, offset, values):
        if offset > self.declared_count or offset + len(values) > self.declared_count:
            return 'RANGE'
        if offset + len(values) > MAX_COMMAND_SAMPLES:
            return 'RANGE'
        for i, v in enumerate(values):
            if not (-(2**31) <= v <= 2**31 - 1):
                return 'RANGE'
            idx = offset + i
            if self.filled[idx] and self.values[idx] != v:
                return 'CONFLICT'
            self.values[idx] = v
            self.filled[idx] = True
        return None

    def is_complete(self):
        return self.declared_count > 0 and all(self.filled)


class _CaptureStream:
    """No on-device-style capacity any more (see run_buffer.h): this fake
    just accumulates the whole simulated run's samples in a plain list,
    then the fake device pushes them off in PUSH_CHUNK_SAMPLES-sized DATA
    frames afterward (see FakeDevice._push_stream_chunks), the same
    chunking the real firmware does continuously instead of all at once."""

    def __init__(self):
        self.samples = []  # list of tuples, length = channels
        self.enabled = False
        self.first_sample_tick = 0
        self.period_ticks = 1
        self.channels = 1
        self.next_push_seq = 0

    def reset(self, enabled, period_ticks, channels):
        self.samples = []
        self.enabled = enabled
        self.period_ticks = period_ticks
        self.channels = channels
        self.next_push_seq = 0

    def append(self, frame):
        self.samples.append(tuple(frame))
        return True


class FakeDevice:
    """Call `feed(bytes)` with whatever the host wrote, then drain
    `outgoing()` for bytes to send back -- mirrors protocol_feed's
    reassembly loop. `poll()` advances the simulated run to COMPLETE once
    its scheduled end tick has passed (the real device does this between
    host requests too; nothing here depends on wall-clock time)."""

    def __init__(self, effort_gain=0.5, encoder_delay_ticks=0):
        self._reader = wire.FrameReader(self._read_available)
        self._rx_queue = b''
        self._tx_queue = b''
        self.state = 'IDLE'
        self.boot_id = 1
        self.run_id = None
        self.manifest = None
        self.command_period_ticks = 0
        self.segments = []
        self.cmdbuf = _CommandBuffer()
        self.streams = [_CaptureStream() for _ in range(NUM_CAPTURE_STREAMS)]
        self.manifest_hash = None
        self.command_hash = None
        self.command_start_tick = None
        self.capture_start_tick = None
        self.run_end_tick = None
        self.fault_reason = None
        self.now_ticks = 1000
        self._effort_gain = effort_gain
        self._encoder_delay_ticks = encoder_delay_ticks
        self._simulated = False

    # -- transport shim: FrameReader wants a blocking-ish read_fn; we feed it manually --
    def _read_available(self, n):
        chunk, self._rx_queue = self._rx_queue[:n], self._rx_queue[n:]
        if not chunk:
            raise _NoMoreData()
        return chunk

    def feed(self, data):
        self._rx_queue += data
        while True:
            try:
                kind, seq, payload = self._reader.next_frame(scan_budget=len(self._rx_queue) + 1)
            except _NoMoreData:
                return
            except wire.FramingError:
                return
            if kind == wire.KIND_REQUEST:
                self._handle(seq, json.loads(payload.decode('utf-8')))

    def outgoing(self):
        data, self._tx_queue = self._tx_queue, b''
        return data

    def _send(self, kind, seq, payload):
        self._tx_queue += wire.pack_frame(kind, seq, payload)

    def _ok(self, seq, op, **fields):
        body = {'ok': True, 'op': op}
        body.update(fields)
        self._send(wire.KIND_RESPONSE, seq, json.dumps(body, allow_nan=False).encode('utf-8'))

    def _err(self, seq, op, code, detail=''):
        body = {'ok': False, 'op': op, 'error': code, 'detail': detail}
        self._send(wire.KIND_RESPONSE, seq, json.dumps(body).encode('utf-8'))

    # -- op handlers, mirroring protocol.cpp's field names/defaults -----------
    def _handle(self, seq, req):
        op = req.get('op')
        handler = getattr(self, f'_op_{op}', None) if op else None
        if handler is None:
            self._err(seq, op or '?', 'UNSUPPORTED', 'unknown op')
            return
        handler(seq, req)

    def _check_run_id(self, seq, op, req):
        rid = req.get('run_id')
        if not rid:
            self._err(seq, op, 'RUN_ID', 'missing run_id')
            return False
        if self.run_id is None or rid != self.run_id:
            self._err(seq, op, 'RUN_ID', 'run_id does not match the active run')
            return False
        return True

    def _op_HELLO(self, seq, req):
        self._ok(seq, 'HELLO', version=1, device_serial='FAKE0000', firmware_version='fake',
                  boot_id=self.boot_id, tick_hz=TICK_HZ, tick_width=64, max_payload=2048,
                  output_modes=['step_dir', 'quadrature_command', 'analog_effort'],
                  encoder_count=NUM_ENCODERS, encoder_decode_mults=[4],
                  encoder_counter_width_bits=64, analog_channels=NUM_ANALOG_CHANNELS,
                  analog_raw_bits=12, analog_simultaneous=False,
                  memory=dict(max_command_samples=MAX_COMMAND_SAMPLES,
                              capture_mode='push_streamed', push_chunk_samples=PUSH_CHUNK_SAMPLES),
                  codecs=['raw_i64', 'delta_rle_i32', 'raw_i32'],
                  service_tick_hz=TICK_HZ // TICKS_PER_SERVICE,
                  max_timing_error_ticks=TICKS_PER_SERVICE)

    def _op_STATUS(self, seq, req):
        self._advance()
        self._ok(seq, 'STATUS', boot_id=self.boot_id, state=self.state, run_id=self.run_id,
                  tick=self.now_ticks,
                  captured_sample_counts=[len(s.samples) for s in self.streams],
                  capture_complete=(self.state in ('COMPLETE', 'FAULT')),
                  output_enabled=(self.state == 'RUNNING'),
                  fault_reason=self.fault_reason if self.state == 'FAULT' else None)

    def _op_BEGIN(self, seq, req):
        if self.state != 'IDLE':
            self._err(seq, 'BEGIN', 'STATE', 'device is not IDLE')
            return
        rid = req.get('run_id')
        m = req.get('manifest')
        if not rid or not isinstance(m, dict):
            self._err(seq, 'BEGIN', 'RANGE', 'run_id/manifest missing')
            return
        if m.get('output_mode') not in ('step_dir', 'quadrature_command', 'analog_effort'):
            self._err(seq, 'BEGIN', 'RANGE', 'output_mode missing/invalid')
            return
        if 'command_sample_count' not in m or 'command_period_ticks' not in m:
            self._err(seq, 'BEGIN', 'RANGE', 'command_sample_count/command_period_ticks required')
            return
        if m.get('safe_action') not in ('disable_drive', 'zero_output', 'stop_pulses'):
            self._err(seq, 'BEGIN', 'RANGE', 'safe_action required')
            return
        count = int(m['command_sample_count'])
        if count <= 0 or count > MAX_COMMAND_SAMPLES:
            self._err(seq, 'BEGIN', 'RANGE', 'command_sample_count out of range')
            return

        self.manifest = m
        self.command_period_ticks = int(m['command_period_ticks'])
        self.segments = m.get('segments', [])
        self.run_id = rid
        self.cmdbuf.reset(count)

        encoder_period = int(m.get('encoder_sample_period_ticks', 64 * TICKS_PER_SERVICE))
        analog_period = int(m.get('analog_sample_period_ticks', 200 * TICKS_PER_SERVICE))
        encoders = m.get('encoders', [])
        for i in range(NUM_ENCODERS):
            enabled = i < len(encoders) and encoders[i].get('enabled', False)
            self.streams[i].reset(enabled, encoder_period, 1)
        analog_enabled = bool(m.get('analog_enabled', False))
        self.streams[STREAM_ANALOG].reset(analog_enabled, analog_period, NUM_ANALOG_CHANNELS)

        canonical = json.dumps(m, sort_keys=True, separators=(',', ':')).encode('utf-8')
        import hashlib
        self.manifest_hash = hashlib.sha256(canonical).hexdigest()
        self.state = 'UPLOADING'
        self._ok(seq, 'BEGIN', manifest_hash=self.manifest_hash)

    def _op_PUT_COMMAND(self, seq, req):
        if self.state != 'UPLOADING':
            self._err(seq, 'PUT_COMMAND', 'STATE', 'device is not UPLOADING')
            return
        if not self._check_run_id(seq, 'PUT_COMMAND', req):
            return
        offset = req.get('offset')
        values = req.get('values')
        if offset is None or not isinstance(values, list):
            self._err(seq, 'PUT_COMMAND', 'RANGE', 'offset/values missing')
            return
        try:
            parsed = [int(v) for v in values]  # firmware requires decimal strings; be strict like it is
        except (TypeError, ValueError):
            self._err(seq, 'PUT_COMMAND', 'RANGE', 'values must be signed decimal strings')
            return
        if not all(isinstance(v, str) for v in values):
            self._err(seq, 'PUT_COMMAND', 'RANGE', 'values must be signed decimal strings')
            return
        err = self.cmdbuf.put(int(offset), parsed)
        if err:
            self._err(seq, 'PUT_COMMAND', err)
            return
        self._ok(seq, 'PUT_COMMAND')

    def _op_SEAL(self, seq, req):
        if self.state != 'UPLOADING':
            self._err(seq, 'SEAL', 'STATE', 'device is not UPLOADING')
            return
        if not self._check_run_id(seq, 'SEAL', req):
            return
        given = req.get('sha256', '')
        if not self.cmdbuf.is_complete():
            self._err(seq, 'SEAL', 'INCOMPLETE', 'not all command samples were uploaded')
            return
        actual = wire.command_sha256(self.cmdbuf.values)
        if given.lower() != actual.lower():
            self._err(seq, 'SEAL', 'HASH', 'uploaded command data does not match the declared sha256')
            return
        self.command_hash = actual
        self.state = 'READY'
        self._ok(seq, 'SEAL')

    def _op_ARM(self, seq, req):
        if self.state != 'READY':
            self._err(seq, 'ARM', 'STATE', 'device is not READY')
            return
        if not self._check_run_id(seq, 'ARM', req):
            return
        self.state = 'ARMED'
        self._ok(seq, 'ARM')

    def _op_START(self, seq, req):
        if self.state != 'ARMED':
            self._err(seq, 'START', 'STATE', 'device is not ARMED')
            return
        if not self._check_run_id(seq, 'START', req):
            return
        pre = int(self.manifest.get('capture_pre_roll_ticks', 0))
        post = int(self.manifest.get('capture_post_roll_ticks', 0))
        count = self.cmdbuf.declared_count
        self.command_start_tick = self.now_ticks + pre + 1000
        self.capture_start_tick = self.command_start_tick - pre
        self.run_end_tick = self.command_start_tick + count * self.command_period_ticks + post
        self.state = 'RUNNING'
        self._simulate_run()
        self._ok(seq, 'START', command_start_tick=self.command_start_tick,
                  capture_start_tick=self.capture_start_tick)

    def _op_KEEPALIVE(self, seq, req):
        if self.state not in ('ARMED', 'RUNNING'):
            self._err(seq, 'KEEPALIVE', 'STATE', 'device is not ARMED or RUNNING')
            return
        if not self._check_run_id(seq, 'KEEPALIVE', req):
            return
        self._ok(seq, 'KEEPALIVE')

    def _op_ABORT(self, seq, req):
        if self.state not in ('UPLOADING', 'READY', 'ARMED', 'RUNNING'):
            self._err(seq, 'ABORT', 'STATE', 'no active run to abort')
            return
        self.state = 'FAULT'
        self.fault_reason = 'HOST_ABORT'
        self._ok(seq, 'ABORT')

    def _op_GET_RUN(self, seq, req):
        self._advance()
        if self.state not in ('COMPLETE', 'FAULT'):
            self._err(seq, 'GET_RUN', 'STATE', 'run data is only available after COMPLETE or FAULT')
            return
        if not self._check_run_id(seq, 'GET_RUN', req):
            return
        streams = []
        for i, s in enumerate(self.streams):
            if not s.enabled:
                continue
            widen = i < NUM_ENCODERS
            codecs = ['raw_i64', 'delta_rle_i32'] if widen else ['raw_i32']
            streams.append(dict(stream_id=i, role='encoder' if widen else 'analog',
                                 channels=s.channels, period_ticks=s.period_ticks,
                                 first_sample_tick=s.first_sample_tick, sample_count=len(s.samples),
                                 frames_pushed=s.next_push_seq,
                                 codecs=codecs, sha256=wire.stream_sha256(s.samples, widen)))
        self._ok(seq, 'GET_RUN', run_id=self.run_id, manifest_hash=self.manifest_hash,
                  command_hash=self.command_hash, command_start_tick=self.command_start_tick,
                  capture_start_tick=self.capture_start_tick, run_end_tick=self.run_end_tick,
                  capture_complete=(self.state == 'COMPLETE'),
                  fault_reason=self.fault_reason if self.state == 'FAULT' else None,
                  streams=streams, segments=self.segments)

    def _op_CLEAR(self, seq, req):
        self._advance()
        if self.state not in ('COMPLETE', 'FAULT'):
            self._err(seq, 'CLEAR', 'STATE', 'device is not COMPLETE or FAULT')
            return
        if not self._check_run_id(seq, 'CLEAR', req):
            return
        self.run_id = None
        self.state = 'IDLE'
        self._ok(seq, 'CLEAR')

    # -- synthetic axis: a step_dir/quadrature_command encoder that reads
    #    back the commanded cumulative count after a fixed delay, plus an
    #    analog channel reporting a synthetic effort proportional to
    #    commanded velocity. See the module docstring: this exists to test
    #    the adapter's wire plumbing, not to model real motor dynamics. --
    def _simulate_run(self):
        count = self.cmdbuf.declared_count
        values = self.cmdbuf.values
        encoders_cfg = self.manifest.get('encoders', [])
        for i in range(NUM_ENCODERS):
            s = self.streams[i]
            if not s.enabled:
                continue
            s.first_sample_tick = self.capture_start_tick
            delay_samples = self._encoder_delay_ticks // max(s.period_ticks, 1)
            tick = 0
            while tick <= self.run_end_tick - self.capture_start_tick:
                t = self.capture_start_tick + tick
                if t < self.command_start_tick:
                    count_now = 0
                else:
                    index = min((t - self.command_start_tick) // self.command_period_ticks, count - 1)
                    delayed_index = max(0, int(index) - delay_samples)
                    count_now = values[delayed_index]
                s.append((count_now,))
                tick += s.period_ticks
            self._push_stream_chunks(i, s)

        analog = self.streams[STREAM_ANALOG]
        if analog.enabled:
            analog.first_sample_tick = self.capture_start_tick
            tick = 0
            previous = 0
            while tick <= self.run_end_tick - self.capture_start_tick:
                t = self.capture_start_tick + tick
                if t < self.command_start_tick:
                    current = 0
                else:
                    index = min((t - self.command_start_tick) // self.command_period_ticks, count - 1)
                    current = values[int(index)]
                velocity = current - previous
                previous = current
                code = int(round(self._effort_gain * velocity))
                analog.append((code, 0))
                tick += analog.period_ticks
            self._push_stream_chunks(STREAM_ANALOG, analog)
        self.now_ticks = self.run_end_tick + 1
        self.state = 'COMPLETE'

    def _push_stream_chunks(self, stream_id, s):
        """Chunks s.samples (already fully computed -- this fake simulates
        a run synchronously, all at once, unlike the real device) into
        PUSH_CHUNK_SAMPLES-sized unsolicited DATA frames, exactly as
        capture_streams_service does continuously on real hardware,
        including a final possibly-partial chunk."""
        for start in range(0, len(s.samples), PUSH_CHUNK_SAMPLES):
            self._push_chunk(stream_id, s, start, s.samples[start:start + PUSH_CHUNK_SAMPLES])

    def _push_chunk(self, stream_id, s, start, chunk):
        """Sends one chunk and advances next_push_seq. Split out from
        _push_stream_chunks so a test can override just this method to
        simulate one lost/corrupted-in-transit chunk (seq still advances
        on the device side; the frame itself just never arrives)."""
        codec = 1 if s.channels == 1 else 3
        width = 8 if codec == 1 else 4
        body = wire.encode_block_raw(chunk, s.channels, width)
        first_tick = s.first_sample_tick + start * s.period_ticks
        seq = s.next_push_seq
        s.next_push_seq += 1
        prefix = struct.pack('<IIQIB3x', stream_id, seq, first_tick, len(chunk), codec)
        self._send(wire.KIND_DATA, 0, prefix + body)

    def _advance(self):
        pass  # the simplified fake runs to completion synchronously in START; nothing to poll


class _NoMoreData(Exception):
    pass


class LoopbackTransport:
    """Wires an AxisClient directly to a FakeDevice in-process, with no
    real socket/thread: request bytes go straight into feed(), and the
    device's queued response bytes are handed back to the next read()."""

    def __init__(self, device):
        self.device = device
        self._pending = b''  # outgoing() drains the whole queue at once; read(n) must not

    def write(self, data):
        self.device.feed(data)

    def read(self, n):
        for _ in range(1000):
            if not self._pending:
                self._pending = self.device.outgoing()
            if self._pending:
                chunk, self._pending = self._pending[:n], self._pending[n:]
                return chunk
            time.sleep(0)  # nothing queued yet; shouldn't happen since feed() replies synchronously
        raise TimeoutError('fake device produced no response')
