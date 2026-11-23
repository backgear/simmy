"""Protocol-level tests for the axis_adapter.py host adapter, run against
the in-process axis_fake_device.py loopback double -- there is no real
characterization_rig board to test against here. This is exactly the
"simulator or loopback fixture" the protocol doc's own "Acceptance checks
before connecting a machine" section asks for before ever opening a
serial port; it is NOT a claim that real firmware has been exercised.
"""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples'))
import axis_wire as wire  # noqa: E402


def _load(name):
    root = Path(__file__).resolve().parents[1] / 'examples'
    spec = importlib.util.spec_from_file_location(name, root / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


adapter = _load('axis_adapter')
fake = _load('axis_fake_device')
identify = _load('identify_axis')


def make_plan_rows(counts_per_unit):
    """A tiny plan: a 'friction' out-and-back, then a smaller 'validation'
    move, matching identify_axis.py's own segment naming so the adapter's
    output is a drop-in fit for examples/identify_axis.py fit."""
    rows = []
    t = 0.0
    dt = 0.001
    for segment, distance in (('friction', 5.0), ('validation', 3.0)):
        for i in range(40):
            s = i / 39
            target = distance * (s if s < .5 else 1 - s) * 2
            rows.append(dict(time=f'{t:.6f}', segment=segment, target=f'{target:.6f}', excitation='0.0'))
            t += dt
    return rows


def write_plan_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time', 'segment', 'target', 'excitation'])
        writer.writeheader()
        writer.writerows(rows)


class RoundTripTests(unittest.TestCase):
    def _config(self, **overrides):
        config = dict(output_mode='step_dir', counts_per_unit=1000.0, command_period_ticks=1000,
                      safe_action='disable_drive', encoder_motor=0, effort_channel=0,
                      effort_scale=0.01, effort_offset=0.0)
        config.update(overrides)
        return config

    def test_full_round_trip_produces_fittable_csv(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            plan_path, out_path = root / 'plan.csv', root / 'out.csv'
            rows = make_plan_rows(1000.0)
            write_plan_csv(plan_path, rows)

            device = fake.FakeDevice(effort_gain=0.5)
            transport = fake.LoopbackTransport(device)
            adapter.capture(plan_path, out_path, self._config(), transport.read, transport.write)

            self.assertTrue(out_path.exists())
            measured = identify.read_measurements(out_path)  # raises on any contract violation
            self.assertGreater(len(measured), 10)
            self.assertEqual({r['segment'] for r in measured} - {'preroll', 'postroll'},
                              {'friction', 'validation'})
            # Encoder reads back the commanded count with no delay configured here,
            # so motor_position should track the commanded target closely.
            late = [r for r in measured if r['segment'] == 'friction'][-1]
            self.assertAlmostEqual(late['motor_position'], late['command'], delta=0.05)

            manifest_path = Path(str(out_path) + '.manifest.json')
            self.assertTrue(manifest_path.exists())
            manifest = json.loads(manifest_path.read_text())
            self.assertEqual(manifest['manifest']['output_mode'], 'step_dir')
            self.assertEqual(len(manifest['command_samples']), len(rows))

    def test_two_body_round_trip_has_load_position(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            plan_path, out_path = root / 'plan.csv', root / 'out.csv'
            write_plan_csv(plan_path, make_plan_rows(1000.0))
            device = fake.FakeDevice()
            transport = fake.LoopbackTransport(device)
            adapter.capture(plan_path, out_path, self._config(encoder_load=1), transport.read, transport.write)
            measured = identify.read_measurements(out_path)
            self.assertIn('load_position', measured[0])

    def test_effort_channel_required(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            plan_path, out_path = root / 'plan.csv', root / 'out.csv'
            write_plan_csv(plan_path, make_plan_rows(1000.0))
            config = self._config()
            del config['effort_channel']
            device = fake.FakeDevice()
            transport = fake.LoopbackTransport(device)
            with self.assertRaisesRegex(ValueError, 'effort_channel'):
                adapter.capture(plan_path, out_path, config, transport.read, transport.write)
            self.assertFalse(out_path.exists())

    def test_position_mode_rejects_excitation(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            plan_path, out_path = root / 'plan.csv', root / 'out.csv'
            rows = make_plan_rows(1000.0)
            rows[0]['excitation'] = '0.5'  # a stray chirp sample on a position-command drive
            write_plan_csv(plan_path, rows)
            device = fake.FakeDevice()
            transport = fake.LoopbackTransport(device)
            with self.assertRaisesRegex(ValueError, 'position-command drive'):
                adapter.capture(plan_path, out_path, self._config(), transport.read, transport.write)

    def test_dropped_push_chunk_is_detected_as_a_sequence_gap(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            plan_path, out_path = root / 'plan.csv', root / 'out.csv'
            write_plan_csv(plan_path, make_plan_rows(1000.0))
            device = fake.FakeDevice()
            # Simulate one lost/corrupted-in-transit chunk: the device still
            # advances its own push-sequence counter (it thinks it sent
            # chunk 0), but the frame itself never arrives -- exactly a
            # dropped/garbled USB transfer, not a device-side bug.
            original = device._push_chunk
            dropped = set()

            def flaky(stream_id, s, start, chunk):
                if stream_id not in dropped:
                    dropped.add(stream_id)
                    s.next_push_seq += 1  # pretend it was sent; never actually send it
                    return
                original(stream_id, s, start, chunk)
            device._push_chunk = flaky
            transport = fake.LoopbackTransport(device)
            with self.assertRaisesRegex(RuntimeError, 'push sequence gap'):
                adapter.capture(plan_path, out_path, self._config(), transport.read, transport.write)
            self.assertFalse(out_path.exists())

    def test_faulted_run_is_refused_not_written(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            plan_path, out_path = root / 'plan.csv', root / 'out.csv'
            write_plan_csv(plan_path, make_plan_rows(1000.0))
            device = fake.FakeDevice()
            device._simulate_run = lambda: (
                setattr(device, 'state', 'FAULT'), setattr(device, 'fault_reason', 'WATCHDOG'))
            transport = fake.LoopbackTransport(device)
            with self.assertRaisesRegex(RuntimeError, 'faulted or incomplete'):
                adapter.capture(plan_path, out_path, self._config(), transport.read, transport.write)
            self.assertFalse(out_path.exists())
            fault_path = Path(str(out_path) + '.fault.json')
            self.assertTrue(fault_path.exists())
            diagnostic = json.loads(fault_path.read_text())
            self.assertEqual(diagnostic['run']['fault_reason'], 'WATCHDOG')


class FramingTests(unittest.TestCase):
    def test_resyncs_past_corrupted_frame(self):
        good = wire.pack_frame(wire.KIND_RESPONSE, 1, b'{"ok":true,"op":"STATUS"}')
        corrupt = bytearray(wire.pack_frame(wire.KIND_RESPONSE, 2, b'{"ok":true,"op":"HELLO"}'))
        corrupt[-1] ^= 0xFF  # flip a payload byte so its CRC no longer matches
        stream = bytes(corrupt) + good

        reader = wire.FrameReader(_chunked_reader(stream))
        kind, seq, payload = reader.next_frame()
        self.assertEqual((kind, seq), (wire.KIND_RESPONSE, 1))
        self.assertEqual(json.loads(payload)['op'], 'STATUS')

    def test_detects_garbage_with_no_valid_frame(self):
        reader = wire.FrameReader(_chunked_reader(b'not a frame at all' * 20))
        with self.assertRaises(wire.FramingError):
            reader.next_frame(scan_budget=50)


def _chunked_reader(data, chunk=7):
    position = [0]

    def read(n):
        start = position[0]
        if start >= len(data):
            return b''
        end = min(start + min(n, chunk), len(data))
        position[0] = end
        return data[start:end]
    return read


class CodecTests(unittest.TestCase):
    def test_delta_rle_round_trip(self):
        samples = [0, 10, 20, 30, 30, 30, 25, 20, 15]
        body = wire.encode_block_delta_rle(samples)
        decoded = wire.decode_block(2, 1, len(samples), body)
        self.assertEqual([v for (v,) in decoded], samples)

    def test_raw_i64_and_i32_round_trip(self):
        frames = [(1, -2), (3, 4), (-5, 6)]
        for codec, width in ((1, 8), (3, 4)):
            body = wire.encode_block_raw(frames, 2, width)
            decoded = wire.decode_block(codec, 2, len(frames), body)
            self.assertEqual(decoded, frames)

    def test_stream_and_command_hashes_match_encode(self):
        samples = [(v,) for v in (0, 5, 5, 5, 10, 10, 3)]
        digest = wire.stream_sha256(samples, widen_to_i64=True)
        body = wire.encode_block_raw(samples, 1, 8)
        import hashlib
        import struct as struct_mod
        expected = hashlib.sha256(body).hexdigest()
        self.assertEqual(digest, expected)
        self.assertEqual(len(wire.command_sha256([1, -1, 2])), 64)


if __name__ == '__main__':
    unittest.main()
