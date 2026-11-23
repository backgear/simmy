"""Recorder integrity tests, independent of attached hardware."""
import hashlib
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples'))
import axis_recorder as recorder
import axis_wire as wire


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.stream = recorder.Stream(Path(self.tmp.name) / 'samples.bin', 1, 250, 8)

    def tearDown(self):
        self.stream.file.close()
        self.tmp.cleanup()

    def block(self, seq=0, tick=100, samples=None, codec=1):
        samples = samples if samples is not None else [(0,), (1,), (-1,)]
        body = (wire.encode_block_raw(samples, 1, 8) if codec == 1
                else wire.encode_block_delta_rle([s[0] for s in samples]))
        return dict(push_seq=seq, first_sample_tick=tick, sample_count=len(samples),
                    codec=codec, body=body)

    def test_raw_and_rle_hash_same_canonical_bytes(self):
        self.stream.accept(self.block(), 1.)
        self.stream.accept(self.block(seq=1, tick=850, codec=2), 1.1)
        expected = struct.pack('<qqq', 0, 1, -1) * 2
        self.assertEqual(self.stream.sha.hexdigest(), hashlib.sha256(expected).hexdigest())
        self.assertEqual(self.stream.count, 6)
        self.assertEqual(self.stream.next_tick, 1600)
        self.stream.file.flush()
        self.assertEqual((Path(self.tmp.name) / 'samples.bin').read_bytes(), expected)

    def test_gap_rejected(self):
        with self.assertRaisesRegex(ValueError, 'sequence'):
            self.stream.accept(self.block(seq=1), 1.)

    def test_duplicate_rejected(self):
        self.stream.accept(self.block(), 1.)
        with self.assertRaisesRegex(ValueError, 'sequence'):
            self.stream.accept(self.block(), 1.1)

    def test_timestamp_gap_and_overlap_rejected(self):
        self.stream.accept(self.block(), 1.)
        for tick in (600, 1100):
            with self.assertRaisesRegex(ValueError, 'timestamp'):
                self.stream.accept(self.block(seq=1, tick=tick), 1.1)

    def test_wrong_codec_rejected(self):
        with self.assertRaisesRegex(ValueError, 'codec'):
            self.stream.accept(self.block(codec=3), 1.)

    def test_truncated_body_rejected(self):
        p = self.block()
        p['body'] = p['body'][:-1]
        with self.assertRaises(ValueError):
            self.stream.accept(p, 1.)

    def test_extra_raw_bytes_rejected(self):
        with self.assertRaises(ValueError):
            recorder.decode_capture(1, 1, 1, struct.pack('<qq', 1, 2))

    def test_rle_overflow_rejected(self):
        with self.assertRaisesRegex(ValueError, 'overflow'):
            recorder.decode_capture(2, 1, 2, struct.pack('<qIi', (1 << 63)-1, 1, 1))

    def test_rle_huge_repeat_rejected_before_expansion(self):
        with self.assertRaisesRegex(ValueError, 'repeat'):
            recorder.decode_capture(2, 1, 2, struct.pack('<qIi', 0, 0xffffffff, 0))

    def test_rle_trailing_records_rejected(self):
        with self.assertRaises(ValueError):
            recorder.decode_capture(2, 1, 1, struct.pack('<qIi', 0, 1, 0))

    def test_analog_hash_uses_i32_even_for_one_channel(self):
        self.stream.file.close()
        self.stream = recorder.Stream(Path(self.tmp.name) / 'analog.bin', 1, 1000, 4)
        body = struct.pack('<ii', 10, -10)
        self.stream.accept(dict(push_seq=0, first_sample_tick=0, sample_count=2,
                                codec=3, body=body), 1.)
        self.assertEqual(self.stream.sha.hexdigest(), hashlib.sha256(body).hexdigest())


if __name__ == '__main__':
    unittest.main()
