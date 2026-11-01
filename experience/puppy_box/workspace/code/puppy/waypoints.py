import sys, json
sys.path.insert(0, '.')
import ik, plan, numpy as np

def grasp_plan(base, target, table_z, edge_x, wp=0.25, side='left', lift=0.12):
    """target: claw centre (between thumb and finger tips) on the puppy torso, pelvis frame.
    Returns ordered list of (name, arm dict, waist pitch)."""
    b = dict(base)
    for k in b:
        if 'waist' in k:
            b[k] = 0.0
    b['waist_pitch_joint'] = wp
    R = ik.claw_rotation(side)
    # palm offset from the claw centre (thumb/index tip midpoint) at the default open hand
    off = np.array([-0.03, 0.0, 0.10])
    palm_g = np.array(target) + off
    ag, pe, re = ik.solve_multi(side, b, palm_g, R)
    # pre-grasp: straight above, seeded from the grasp branch
    names = list(ag)
    ap, pe2, re2 = ik.solve(side, b, palm_g + [0, 0, lift], R, seed=[ag[n] for n in names])
    out = {'grasp': (ag, pe, re), 'pre': (ap, pe2, re2)}
    return b, out
