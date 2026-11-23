from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_motion_guard import require_motion_unpaused
from axis_pc_tune import HardwareBackend
from axis_recorder import Transport

class MotionPauseTests(unittest.TestCase):
    def test_pause_requires_explicit_removal(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'pause.json'
            with patch('axis_motion_guard.PAUSE_FILE',p):
                require_motion_unpaused();p.write_text('{}')
                with self.assertRaisesRegex(RuntimeError,'paused'):require_motion_unpaused()
                p.unlink();require_motion_unpaused()
    def test_hardware_backend_blocks_before_serial_open(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'pause.json';p.write_text('{}')
            with patch('axis_motion_guard.PAUSE_FILE',p),patch('os.open') as opened:
                with self.assertRaisesRegex(RuntimeError,'paused'):HardwareBackend(Path(d),'/not-a-port','unused')
                opened.assert_not_called()
    def test_arm_start_block_before_transport_state_or_writes(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'pause.json';p.write_text('{}')
            ghost=Transport.__new__(Transport)
            with patch('axis_motion_guard.PAUSE_FILE',p):
                for op in ('ARM','START'):
                    with self.assertRaisesRegex(RuntimeError,'paused'):ghost.request(op)

if __name__=='__main__':unittest.main()
