# Axis characterization device contract — draft v1

This is the host/MCU boundary, **not firmware**, and not a claim that the current
Python adapter implements this wire protocol. Firmware is supplied separately.
Transport: USB CDC, binary framed messages. The MCU owns all realtime execution,
encoder counting, ADC timing, buffering, and shutdown. The host owns experiment
design, calibration metadata, decoding, fitting, and plots.

## Scope

One commanded axis; selectable step/direction, quadrature-command, or analog
output; up to four independently counted quadrature encoder inputs; optional
analog measurements and digital status inputs. A typical fit needs two encoders
(motor/load) and actual effort. Commanded pulse count is never measured position.

A run is finite and completely uploaded before execution. USB latency never
changes move timing. Capture can remain entirely in device memory until the run
finishes. Streaming during execution is optional, not required by v1. No firmware
or host may assume that digital compression always achieves a particular ratio.

Normative words MUST/SHOULD describe the proposed interoperability contract.
All example limits, rates, and amplitudes must come from machine configuration;
there are no universal safe defaults.

## Wire framing

All integers are little-endian. USB reads/writes may split or combine frames.
The receiver accumulates bytes and searches for this 20-byte header:

| Offset | Type | Field |
|---|---|---|
| 0 | 4 bytes | ASCII `AXIS` |
| 4 | u8 | version = 1 |
| 5 | u8 | kind: REQUEST=1, RESPONSE=2, EVENT=3, DATA=4 |
| 6 | u16 | flags = 0 in v1 |
| 8 | u32 | request sequence; 0 reserved for unsolicited events |
| 12 | u32 | payload length in bytes |
| 16 | u32 | CRC-32/ISO-HDLC of header bytes 0..15 followed by payload |

CRC is compatible with Python `zlib.crc32(header[:16] + payload) & 0xffffffff`.
No padding or trailing terminator. Reject unknown versions, nonzero flags, invalid
CRC, and oversized frames. The bootstrap maximum payload is 4096 bytes; HELLO may
negotiate a different maximum, never exceeding either peer's supported value.
After corruption, scan for the next plausible header and verify its CRC. Invalid
frames MUST NOT execute operations. An incomplete frame expires after a negotiated
receive timeout and is discarded.

REQUEST/RESPONSE/EVENT payloads are UTF-8 JSON objects, without NaN/Infinity.
A request is `{"op":"HELLO", ...}`. A normal response is
`{"ok":true,"op":"HELLO", ...}`; an error is
`{"ok":false,"op":"ARM","error":"LIMIT","detail":"..."}`.
RESPONSE/DATA echoes its request sequence. A request gets exactly one terminal
RESPONSE or DATA. Events are informational, never required to reconstruct state.
The host uses one outstanding request at a time in v1, monotonically increasing
sequences, and reconnects before sequence wrap. It MUST query status after a
lost response rather than blindly repeat a state-changing operation. Upload
chunks and data reads are idempotent; START is not a retry mechanism.

## Device state and identity

States: `IDLE → UPLOADING → READY → ARMED → RUNNING → COMPLETE`.
Any active state can enter `FAULT`; ABORT ends an armed/running experiment in
`FAULT` with reason `HOST_ABORT`. CLEAR returns COMPLETE/FAULT to IDLE only after
outputs have reached their configured safe condition. START is accepted only in
ARMED. Configuration/upload is forbidden in ARMED/RUNNING. No automatic restart.

HELLO returns:
- protocol version, device serial, firmware version, boot_id (changes every boot);
- device clock frequency `tick_hz` (u64), tick width (64), maximum payload;
- supported output modes and their electrical/configuration capabilities;
- encoder count, decode multipliers, counter width/extension behavior;
- analog channels, raw representation/scales, simultaneous vs multiplexed ADC,
  aperture offsets in ticks and supported sample periods;
- memory capacity, maximum command samples, acquisition rates, pulse/DAC limits;
- supported codecs (`raw_i64`, `delta_rle_i32`, `raw_i32`) and safety capabilities.

STATUS is authoritative: boot_id, state, run_id, current tick, executed command
index, captured sample counts, buffer use, fault reason/tick, output enabled, and
whether capture is complete. The clock MUST be monotonic within a boot; times
from different boot_id values cannot be joined silently.

## Operations and transfer sequence

1. **HELLO**, then **STATUS**. Verify identity and capabilities.
2. **BEGIN** with a host-generated unique run_id and immutable manifest (below).
   Accepted only in IDLE; reserves the experiment and enters UPLOADING.
3. **PUT_COMMAND**: JSON `{run_id, offset, values:[...]}`. Offset is a sample index;
   integer values are signed decimal **strings** to avoid JSON precision loss.
   Chunks may arrive out of order. Repeating identical samples is accepted;
   conflicting data is an error. No partially uploaded plan can execute.
4. **SEAL** with SHA-256 of the canonical command bytes: concatenated signed
   little-endian i64 samples. Device checks completeness, digest, memory budget,
   all command constraints, and safety configuration; enters READY or reports
   rejection. Manifest SHA-256 is computed over its RFC 8785 canonical JSON and
   retained too. Neither manifest nor command data may change after sealing.
5. **ARM** validates live interlocks and configured output idle state, latches
   encoder origins without moving the mechanism, and enters ARMED. It does not
   generate command pulses or enable torque unless explicitly specified by the
   safe enable policy in the manifest. Arming expires after the configured timeout.
6. **START** `{run_id}` schedules execution wholly on the device and returns
   the command start tick and capture start tick. The command start is sufficiently
   far in the future to acquire the configured pre-roll after accepting START.
   Capture includes configured post-roll. START is
   rejected unless sufficient resources and healthy interlocks remain available.
7. Poll **STATUS** or listen for state events. **KEEPALIVE** renews the configured
   watchdog lease. USB carries supervision, not sample timing. **ABORT** is always
   accepted for the current run and schedules the configured safe action.
8. **GET_RUN** returns immutable manifest, hashes, stream descriptors, segment
   boundaries, start/end ticks, faults, completeness, and block counts. Data can
   be retrieved after COMPLETE or FAULT. Partial/faulted data remains diagnostic;
   it MUST NOT be silently treated as a successful identification run.
9. **GET_BLOCK** `{run_id, stream_id, block_index}` returns one DATA frame, or an
   error if unavailable. Repeated reads return identical bytes. Host verifies
   all blocks, counts, and timing before converting to analysis CSV.
10. **CLEAR** `{run_id}` explicitly releases buffers. A disconnect MUST NOT erase
    completed data; reset/power loss may erase volatile data and changes boot_id.

Stable error codes: `STATE`, `RUN_ID`, `RANGE`, `UNSUPPORTED`, `INCOMPLETE`,
`HASH`, `CONFLICT`, `MEMORY`, `LIMIT`, `INTERLOCK`, `WATCHDOG`, `OVERFLOW`,
`CAPTURE_TIMING`, `HOST_ABORT`, `INTERNAL`. Error detail is human-readable, not
parsed for program logic. Faults include the first fault's device tick.

## Manifest: required semantics

The manifest MUST define:
- run_id, output mode, drive interpretation (`position`, `velocity`, `effort`),
  command sample count, command period in ticks, capture pre/post-roll;
- output electrical configuration, idle value, enable polarity/policy, step pulse
  width and direction setup/hold, or quadrature polarity/state order, or DAC
  channel and update/settling constraints;
- command integer scale/offset and unit: pulses/counts for digital output, DAC
  codes for analog. Analog physical meaning and drive interpretation are separate;
- selected encoders, polarity, x1/x2/x4 decode, counts per mechanical unit,
  motor/load/intermediate role, calibrated relative offsets and transmission ratios;
- selected analog channels, raw-to-physical scale/offset/unit, effort calibration
  or phase-current/rotor electrical-angle model reference;
- capture stream sample periods, channel order, selected codec/block sizes;
- segment names and half-open command-index ranges, including `validation`;
- permitted command range/rate, travel/velocity limits in encoder coordinates,
  current/effort limits if available, monitored faults/interlocks, watchdog timeout,
  arming timeout, and explicit safe action on completion/abort/fault/disconnect;
- required connection policy: `supervised` (watchdog required) or explicitly
  permitted `autonomous` (no USB heartbeat required; all local safeguards remain).

The device MUST reject unsupported or unenforceable requested safeguards instead
of acknowledging them and ignoring them. Calibration records identify their
source/version. A drive-current limit enforced externally may be recorded as
external, but MUST NOT be represented as device-enforced protection.

### Command timing

Each command sample defines one period relative to command start. Digital edges
belong to `(i*period, (i+1)*period]`; analog holds use
`[i*period, (i+1)*period)`. For digital outputs, sample `c[i]` is the cumulative
commanded count at that interval's **end**; the initial count is zero. The device emits exactly
`c[i]-c[i-1]` signed transitions, evenly spaced across the interval, with the last
transition at its end. For sample zero, use `c[-1]=0`. One step pulse rising edge
is one step/dir transition; one adjacent Gray-code A/B state change is one
quadrature-command transition. Reverse moves decrement the cumulative count.
The plan MUST allow pulse-width, direction setup/hold, and maximum edge-rate
requirements, including at interval boundaries and reversals. Device quantization
to timer ticks and its maximum timing error are declared by HELLO. No missing or
extra transitions are allowed; reject an unrepresentable plan at SEAL.

Analog codes use zero-order hold, applied at the interval's start. There is no
implicit interpolation. ADC sampling aperture is separate from DAC update time.
The host resamples trajectories into this representation; firmware does not
interpret high-level chirps, targets, or controller tuning requests.

Capture tick zero need not be command tick zero. GET_RUN reports both absolute
start ticks. Segments are mapped from command intervals to capture timestamps;
pre/post-roll gets its own segment label. The device SHOULD capture actual
command phase/count or DAC code in a separate stream where supported. Planned
command, physical output, measured motion, and measured effort remain distinct.

## Acquisition and lossless encoding

Encoders are counted continuously in hardware (or an equivalent verified method),
not by sampling A/B at the acquisition frequency. Sampled counts are latched at
specified acquisition ticks. Counts are extended to signed i64 without rollover
loss. HELLO specifies maximum supported input edge rates and synchronization
accuracy. Counter snapshot skew, ADC apertures, and analog frontend delays must
be characterized/reported; '100 kHz' alone is not a synchronization specification.

Each stream has a fixed sample period and ordered channels. The host can request
100 kHz if supported, independently of command rate. Device buffering MUST
prevent silent loss. SEAL checks a conservative storage budget including block
headers and worst-case codec expansion, or rejects the plan. Implementations may
fall back to raw encoding per block. Compression is not a capacity guarantee.
Any runtime overflow or missed acquisition tick faults the run and marks its
recording incomplete. Safety shutdown takes priority over capture.

DATA payload begins with this 24-byte prefix:

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | stream_id |
| 4 | u32 | block_index, starting at zero |
| 8 | u64 | first_sample_tick, absolute device clock |
| 16 | u32 | sample_count, positive |
| 20 | u8 | codec: 1=raw_i64, 2=delta_rle_i32, 3=raw_i32 |
| 21 | 3 bytes | reserved, zero |

Channel count/order and sample period come from GET_RUN. Consecutive sample ticks
are `first_sample_tick + k*period_ticks`. Blocks have no overlap or gaps within a
stream. Streams may have different periods and aperture offsets. A final block
may be shorter. Sum of block sample counts MUST equal GET_RUN's captured count.
Wire CRC protects every block transfer. GET_RUN also supplies SHA-256 per stream
over the concatenated **uncompressed sample-major little-endian raw values**,
so missing/reordered blocks or decoding errors are detectable after retrieval.

Codec 1: sample-major i64 values, exactly `sample_count * channels * 8` bytes.
Used for encoder counts and optionally actual digital command counters.

Codec 2: first sample is `channels` i64 absolute counts. Remaining bytes are
records of `repeat:u32` followed by `channels` signed i32 count deltas. Each record
applies the delta vector repeatedly to generate `repeat` successive samples.
`repeat` MUST be positive; sum of repeats MUST be `sample_count-1`. A single
constant region is one record with zero deltas; repeated constant-velocity count
increments also compress. Records never depend on a previous block. Integer
addition MUST be checked for i64 overflow. Deltas that do not fit i32 require a
raw block. This is RLE of sampled count increments, not sampled A/B logic levels:
all intermediate raw edges are not preserved, but every sampled count is exact.

Codec 3: sample-major signed i32 raw analog/status/DAC values, exactly
`sample_count * channels * 4` bytes. Per-channel scale/offset maps to physical
units; ADC settling/aperture are metadata, not hidden corrections. Digital flags
can be a bitmask in an i32 channel. Future codecs require negotiation.

## Safety contract

A hardware estop and appropriate external safeguards remain necessary. USB CDC,
CRC, and a software watchdog are not a safety-rated system. Safe actions MUST be
specified per machine: 'zero analog output', 'stop pulses', and 'disable drive'
are not interchangeable and may not stop an axis safely. Firmware MUST report
what it can implement; no generic uncontrolled immediate stop is assumed safe.
Fault handling is local and does not wait for a USB response or free buffer space.

## Host adapter and existing fitter

The executable adapter interface already used by `identify_axis.py` remains:

    adapter --plan ABS_PLAN_CSV --output ABS_MEASUREMENT_CSV

The adapter reads separate machine configuration, converts planned targets and
injected effort into an output the drive actually accepts, speaks this protocol,
and writes the decoded recording. Position-command drives cannot simply add an
effort chirp to a pulse count: use a suitable position excitation, or a supported
torque injection facility. Preserve what was actually commanded. Mapping a
position sweep through a closed drive identifies different dynamics from a direct
effort-input experiment; the fitter still requires actual effort measurements.

The output CSV preserves `time,segment,command,effort,motor_position` and optional
`load_position`/other channels. `time` is seconds relative to command start.
Encoder counts are converted into a common mechanical coordinate. Effort is
calibrated measured force/torque, not inferred from the command. The adapter MUST
reject incomplete runs for normal fitting, retain faulted raw recordings for
diagnostics, and preserve manifest/run metadata beside the CSV. If analog effort
needs resampling onto the position grid, apply documented filtering/alignment;
never silently substitute command for missing effort.

## Acceptance checks before connecting a machine

Firmware and host implementations should share golden vectors for framing/CRC,
partial USB reads, large signed counts, RLE reconstruction, stream hashing,
corrupt/missing/reordered blocks, command polarity/timing, and tick offsets.
Exercise interrupted uploads, duplicate chunks, lost START response, disconnects,
watchdog/limit trips, capture overflow, and reset during a run. Verify pulses and
sampling synchronization on a logic analyzer. Then exercise the complete upload →
run → retrieve → decode → fit → plot path with a simulator or loopback fixture.
The document defines that interface; an implementation and these interoperability
tests are still required before hardware characterization is automatic.
