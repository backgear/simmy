"""Shared PC-side bounds and normal position-workload definition; no plant/fitter."""
FIELDS=('position_kp_q8','velocity_kp_q8','velocity_ki_q8','torque_limit_code','error_limit_counts','velocity_clamp')
DEFAULT=dict(zip(FIELDS,(4,64000,1024,3276,1024,8)))
def validate_servo(c):
    if set(c)!=set(FIELDS) or any(type(c[k]) is not int for k in FIELDS):
        raise ValueError('servo configuration requires six integer fields')
    bounds=((1,256),(1,1048576),(0,1048576),(1,6553),(1,4096),(1,32))
    if any(not lo<=c[k]<=hi for k,(lo,hi) in zip(FIELDS,bounds)):
        raise ValueError('servo parameter outside supported range')
    return c

def workload(distance=256,hold_samples=25):
    """Application position requests, NOT effort probes or model-based excitation.
    One 20ms-reference schedule: hold origin, positive goal, origin, negative
    goal, origin. Controller supplies all effort from its independent feedback.
    """
    if type(distance) is not int or not 1<=distance<=512 or not 5<=hold_samples<=100:
        raise ValueError('workload outside bounded pilot envelope')
    return [0]*10 + [distance]*hold_samples + [0]*hold_samples + [-distance]*hold_samples + [0]*hold_samples
