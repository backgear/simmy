"""Local operator pause latch. No hardware access and no automatic recovery."""
from pathlib import Path
PAUSE_FILE=Path(__file__).resolve().parents[1]/'build/axis-validation/POWERED_TESTS_PAUSED.json'

def require_motion_unpaused():
    if PAUSE_FILE.exists():
        raise RuntimeError(f'Powered testing is paused by {PAUSE_FILE}; no ARM/START or automatic override. Operator recovery/authorization required.')
