import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_drive_fault_audit import suspicious_windows

class FaultSymptomScreenTests(unittest.TestCase):
    def test_sustained_high_effort_no_response(self):
        self.assertEqual(suspicious_windows(np.zeros(100),np.full(100,3276),np.full(100,256)),[0])
    def test_motion_is_not_a_no_response_window(self):
        self.assertEqual(suspicious_windows(np.arange(100),np.full(100,3276),np.full(100,256)),[])
    def test_short_or_low_effort_not_flagged(self):
        self.assertEqual(suspicious_windows(np.zeros(99),np.full(99,3276),np.full(99,256)),[])
        self.assertEqual(suspicious_windows(np.zeros(100),np.full(100,1637),np.full(100,256)),[])

if __name__=='__main__':unittest.main()
