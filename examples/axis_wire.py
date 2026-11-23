"""Shared AXIS wire-protocol framing/codec helpers for the single-axis
characterization device. Matched against the REAL firmware in
characterization_rig/src/protocol.cpp -- not just the prose spec in
docs/axis_device_protocol.md, which describes a few things (per-channel
manifest scale/offset, encoder "roles") this firmware doesn't actually
implement; see axis_adapter.py's docstring for the reconciled list.

Stdlib only. Works over anything read_fn(n)/write_fn(bytes) can reach:
a raw serial file descriptor (axis_adapter.open_serial) or an in-process
loopback pipe (axis_fake_device.py, used by the test suite since no real
hardware exists to test against here).
"""
import json
import struct
import zlib
import hashlib

MAGIC = b'AXIS'
HEADER_LEN = 20
VERSION = 1
KIND_REQUEST, KIND_RESPONSE, KIND_EVENT, KIND_DATA = 1, 2, 3, 4


class FramingError(ValueError):
    """A frame was structurally invalid (bad magic/version/CRC) and no
    valid frame could be resynchronized onto within the scan budget."""


class AxisDeviceError(RuntimeError):
    """The device returned {"ok": false, ...} for a request."""
    def __init__(self, op, code, detail):
        super().__init__(f'{op}: {code} ({detail})')
        self.op, self.code, self.detail = op, code, detail


def crc32(data):
    return zlib.crc32(data) & 0xffffffff


def pack_frame(kind, seq, payload):
    header = MAGIC + bytes([VERSION, kind, 0, 0]) + struct.pack('<II', seq, len(payload))
    return header + struct.pack('<I', crc32(header + payload)) + payload


def pack_request(seq, op, **fields):
    body = {'op': op}
    body.update(fields)
    return pack_frame(KIND_REQUEST, seq, json.dumps(body, allow_nan=False).encode('utf-8'))


class FrameReader:
    """Accumulates bytes and yields parsed (kind, seq, payload) frames,
    mirroring protocol.cpp's try_parse_frames: on a bad magic position,
    version, flags, or CRC, drop one byte and keep scanning rather than
    treating any single corrupt byte as fatal."""

    def __init__(self, read_fn):
        self._read = read_fn  # read_fn(n) -> 1..n bytes; raises/returns b'' only on a closed connection
        self._buf = b''

    def _fill(self, want):
        while len(self._buf) < want:
            chunk = self._read(max(want - len(self._buf), 4096))
            if not chunk:
                raise ConnectionError('device connection closed while waiting for data')
            self._buf += chunk

    def next_frame(self, scan_budget=1 << 20):
        dropped = 0
        while dropped < scan_budget:
            self._fill(HEADER_LEN)
            at = self._buf.find(MAGIC)
            if at == -1:
                # Keep a 3-byte tail: "AXIS" may straddle the next read.
                dropped += max(len(self._buf) - 3, 0)
                self._buf = self._buf[-3:]
                continue
            if at > 0:
                dropped += at
                self._buf = self._buf[at:]
                continue
            version, kind = self._buf[4], self._buf[5]
            flags = struct.unpack_from('<H', self._buf, 6)[0]
            if version != VERSION or flags != 0 or not (1 <= kind <= 4):
                self._buf = self._buf[1:]
                dropped += 1
                continue
            seq, payload_len = struct.unpack_from('<II', self._buf, 8)
            self._fill(HEADER_LEN + payload_len)
            stored_crc = struct.unpack_from('<I', self._buf, 16)[0]
            payload = self._buf[HEADER_LEN:HEADER_LEN + payload_len]
            if crc32(self._buf[:16] + payload) != stored_crc:
                self._buf = self._buf[1:]
                dropped += 1
                continue
            self._buf = self._buf[HEADER_LEN + payload_len:]
            return kind, seq, payload
        raise FramingError(f'no valid frame found after dropping {dropped} bytes')


class AxisClient:
    """One-outstanding-request-at-a-time client, per the protocol doc
    ("The host uses one outstanding request at a time in v1"). Stray
    EVENT frames are ignored; a RESPONSE frame whose sequence doesn't
    match the current request is also ignored (a late reply to something
    we gave up on), not treated as ours.

    There is no pull-based block retrieval any more (GET_BLOCK was
    removed): every DATA frame is now the device pushing a just-captured
    capture chunk, unsolicited, with sequence=0 ("0 reserved for
    unsolicited events" in the wire doc). `on_push`, if given, is called
    with `parse_data_prefix(payload)` for each one, as soon as it's seen
    -- including ones that arrive interleaved while waiting for an
    unrelated request's response, which is the common case once a run is
    RUNNING and capture_streams_service is pushing continuously."""

    def __init__(self, write_fn, read_fn, on_push=None):
        self._write = write_fn
        self._reader = FrameReader(read_fn)
        self._seq = 0
        self._on_push = on_push

    def request(self, op, **fields):
        self._seq += 1
        seq = self._seq
        self._write(pack_request(seq, op, **fields))
        while True:
            kind, got_seq, payload = self._reader.next_frame()
            if kind == KIND_DATA and got_seq == 0:
                if self._on_push:
                    self._on_push(parse_data_prefix(payload))
                continue
            if kind == KIND_EVENT or got_seq != seq:
                continue
            body = json.loads(payload.decode('utf-8'))
            if not body.get('ok', False):
                raise AxisDeviceError(op, body.get('error'), body.get('detail'))
            return 'RESPONSE', body


def parse_data_prefix(payload):
    """The 24-byte DATA prefix (docs/axis_device_protocol.md, "Acquisition
    and lossless encoding"), plus the remaining encoded body. The second
    header field is still named `block_index` on the wire, but every DATA
    frame is now an unsolicited push (GET_BLOCK is gone): it's this
    stream's own monotonic push-sequence counter, starting at 0 -- a gap
    means a chunk was lost or reordered."""
    if len(payload) < 24:
        raise FramingError('DATA payload shorter than the 24-byte prefix')
    stream_id, push_seq = struct.unpack_from('<II', payload, 0)
    first_sample_tick, = struct.unpack_from('<Q', payload, 8)
    sample_count, = struct.unpack_from('<I', payload, 16)
    codec = payload[20]
    return dict(stream_id=stream_id, push_seq=push_seq,
                first_sample_tick=first_sample_tick, sample_count=sample_count,
                codec=codec, body=payload[24:])


CODEC_NAMES = {1: 'raw_i64', 2: 'delta_rle_i32', 3: 'raw_i32'}
CODEC_IDS = {name: num for num, name in CODEC_NAMES.items()}


def decode_block(codec, channels, sample_count, body):
    """Returns a list of `sample_count` tuples of length `channels`,
    matching capstream_encode_block's exact byte layout (run_buffer.cpp)."""
    if codec == 1:  # raw_i64: sample-major i64
        n = sample_count * channels
        values = struct.unpack_from(f'<{n}q', body, 0)
        return [values[i * channels:(i + 1) * channels] for i in range(sample_count)]
    if codec == 3:  # raw_i32: sample-major i32
        n = sample_count * channels
        values = struct.unpack_from(f'<{n}i', body, 0)
        return [values[i * channels:(i + 1) * channels] for i in range(sample_count)]
    if codec == 2:  # delta_rle_i32: single-channel only in this firmware
        if channels != 1:
            raise ValueError('delta_rle_i32 is single-channel only in this firmware')
        if sample_count == 0:
            return []
        first, = struct.unpack_from('<q', body, 0)
        samples = [first]
        offset = 8
        while len(samples) < sample_count:
            if offset + 8 > len(body):
                raise ValueError('delta_rle_i32: truncated record')
            repeat, delta = struct.unpack_from('<Ii', body, offset)
            offset += 8
            if repeat == 0:
                raise ValueError('delta_rle_i32: repeat must be positive')
            for _ in range(repeat):
                samples.append(samples[-1] + delta)
        if len(samples) != sample_count:
            raise ValueError('delta_rle_i32: repeat counts do not sum to sample_count-1')
        return [(v,) for v in samples]
    raise ValueError(f'unknown codec {codec!r}')


def encode_block_delta_rle(samples):
    """Inverse of decode_block's codec 2, for the fake device (and for
    anyone cross-checking against real firmware captures). `samples` is a
    flat list of single-channel i64-range ints."""
    out = struct.pack('<q', samples[0])
    i = 1
    while i < len(samples):
        delta = samples[i] - samples[i - 1]
        repeat = 1
        while i + repeat < len(samples) and samples[i + repeat] - samples[i + repeat - 1] == delta:
            repeat += 1
        out += struct.pack('<Ii', repeat, delta)
        i += repeat
    return out


def encode_block_raw(samples, channels, width):
    """`samples` is always a list of length-`channels` tuples (sample-major,
    channel-interleaved), even for single-channel streams."""
    fmt = {8: 'q', 4: 'i'}[width]
    flat = [v for frame in samples for v in frame]
    return struct.pack(f'<{len(flat)}{fmt}', *flat)


def command_sha256(values):
    """Matches cmdbuf_hash_canonical: SHA-256 over the declared-count
    command samples, each widened to a signed little-endian i64."""
    h = hashlib.sha256()
    for v in values:
        h.update(struct.pack('<q', v))
    return h.hexdigest()


def stream_sha256(samples, widen_to_i64):
    """Matches capstream_hash: SHA-256 over concatenated sample-major
    values at their declared element width (8 bytes if widen_to_i64,
    i.e. an encoder/raw_i64-capable stream, else 4 bytes for analog)."""
    h = hashlib.sha256()
    fmt = '<q' if widen_to_i64 else '<i'
    for frame in samples:
        for v in frame:
            h.update(struct.pack(fmt, v))
    return h.hexdigest()


def open_serial(port, baud=115200):
    """Opens a serial/USB-CDC character device in raw binary mode. No
    pyserial: termios/tty/os are stdlib and sufficient for a POSIX tty --
    this is the one part of the adapter that is NOT exercised by the
    loopback tests (there is no real character device here to open), so
    treat it as unverified until it's actually run against hardware.
    Returns (read_fn, write_fn, close_fn)."""
    import os
    import termios
    import tty

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f'B{baud}', termios.B115200)
    attrs[4] = attrs[5] = speed  # ispeed, ospeed; USB CDC virtual ports ignore the actual rate
    attrs[3] &= ~(termios.ECHO | termios.ICANON)  # belt-and-suspenders on top of setraw
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def read_fn(n):
        return os.read(fd, n)

    def write_fn(data):
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view):]

    def close_fn():
        os.close(fd)

    return read_fn, write_fn, close_fn
