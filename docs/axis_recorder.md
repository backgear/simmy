# Unpowered streaming validation recorder

`examples/axis_recorder.py` is a raw acquisition/transport validator, not a
motion planner or calibrated identification adapter. Default operation sends
only HELLO and STATUS. The execution option sends finite **zero-only analog
effort commands** and sets `output_enable_used=false`, with active-high enable,
supervised watchdog, and `safe_action=disable_drive`.

**Keep the drive unpowered.** Confirm the actual enable polarity before using
this configuration. Use firmware that honors `output_enable_used=false` in
`axis_hw_latch_origin`; the previous implementation asserted enable regardless
of that flag. Software status does not replace electrical verification.

```sh
# Directory must not already exist. Stdlib-only, POSIX serial transport.
python3 examples/axis_recorder.py --port /dev/serial/by-id/DEVICE \
    --output /tmp/axis-health

# Explicit permission to configure, ARM, START, ABORT, and inject faults:
python3 examples/axis_recorder.py --port /dev/serial/by-id/DEVICE \
    --output /tmp/axis-streaming --execute-unpowered
```

For the current encoder-only acceptance scope, add `--encoder-only` to the
execution command. It disables ADC in every manifest and omits ADC-only cases.
`--from-case NAME` resumes at a named case with the device already IDLE.
ADC timing is a documented deferred defect, not an approved measurement path.

## Test matrix

- HELLO and four STATUS snapshots: stable boot, advancing clock, IDLE,
  output disabled. Non-IDLE state stops execution without automatic CLEAR.
- All streams at 1 kHz, 2 seconds, then 15 seconds.
- Encoder 0, encoder 1, both encoders, ADC, all streams: each at 1, 2,
  and 4 kHz for 3 seconds. ADC has two multiplexed channels.
- Host read pauses of 10, 50, 200 ms, 1 second and 3 seconds at 4 kHz. OS/USB buffers can
  absorb these pauses; this is not guaranteed to induce device overflow.
- Withhold KEEPALIVE while continuously draining: expect WATCHDOG.
- Explicit ABORT while continuously draining: expect HOST_ABORT.

The transport drains continuously between requests; it does not sleep 50 ms
between drain opportunities. Requests remain single-outstanding. Upload writes
are paced in 64-byte pieces with 2 ms gaps. Firmware also has a USB RX NAK guard
for the queue-full path. Neither mechanism certifies arbitrary upload bursts.
Enumeration after reset is not guaranteed; a physical power cycle can still be
necessary.

## Artifacts

The root holds `health.json`, full `rx.bin`/`tx.bin` byte logs,
`frames.jsonl` with host monotonic frame-dispatch timestamps, and `summary.json`.
Each case holds the manifest/zero command array, BEGIN/START/ARM/status/final
metadata, `report.json`, and decoded sample-major little-endian `.i64le` encoder
or `.i32le` analog files. Raw evidence remains on fault or integrity failure.
Frame timestamps are host parser-dispatch times, not physical USB packet times.

Checks include frame CRC/header/size, DATA reserved fields, stream membership,
chunk sequence, exact codec lengths, bounded RLE expansion/i64 overflow,
chunk tick continuity, first capture tick, final sample/frame counts and SHA-256,
manifest hash consistency, live arrival, and output-disabled status throughout.
Fault-injection runs are diagnostic even when their test expectation passes;
they are never valid identification runs. Unexpected normal-run faults fail the
test. On an uncertain request result, START is never retried; ABORT is attempted
and state/data are left for inspection rather than automatically cleared.

Stationary counts compress exceptionally well: these tests do **not** certify
moving-axis bandwidth, physical timing, ADC aperture/skew, or safe electrical
output. Independent timing measurements and dynamic quadrature input are still
needed. In particular, firmware tick continuity alone cannot detect a service
ISR losing wall-clock time under load.

Unit tests:

```sh
python3 -m unittest discover -s tests -p test_axis_recorder.py
```
