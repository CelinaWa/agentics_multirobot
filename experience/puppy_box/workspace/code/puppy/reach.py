"""Plan the left claw reach from the latest observation: pre-high, pre, grasp (arm joint dicts)."""
import json, sys
import numpy as np
sys.path.insert(0, '.')
import ik, plan

ARMN = ['left_%s_joint' % a for a in ik.ARM]


def descent(base, target, wp, side='left', heights=(0.16, 0.08, 0.04, 0.0)):
    b = dict(base)
    b['waist_pitch_joint'] = wp
    b['waist_yaw_joint'] = 0.0
    b['waist_roll_joint'] = 0.0
    R = ik.claw_rotation(side)
    off = np.array([-0.03, 0.0, 0.10])
    palm_g = np.array(target) + off
    ag, pe, re = ik.solve_multi(side, b, palm_g, R)
    sols = [(0.0, ag, pe, re)]
    seed = [ag[n] for n in ARMN]
    for h in sorted(heights)[1:]:
        a, pe, re = ik.solve(side, b, palm_g + [0, 0, h], R, seed=seed)
        seed = [a[n] for n in ARMN]
        sols.append((h, a, pe, re))
    return b, sols[::-1]   # high first


if __name__ == '__main__':
    target = [float(x) for x in sys.argv[1:4]]
    wp = float(sys.argv[4])
    table = float(sys.argv[5])
    o = json.load(open('../../output/obs/last.json'))
    base = dict(zip(o['proprio']['joint_names_canonical29'], o['proprio']['joint_positions_rad']))
    b, sols = descent(base, target, wp)
    out = []
    for h, a, pe, re in sols:
        j = dict(b); j.update(a)
        print(h, [round(a[n], 2) for n in ARMN], round(pe, 3), round(re, 2), ik.tips(j, 'left'),
              'clear', round(plan.clearance(j, 'left', table, 0.23), 3))
        out.append({'h': h, 'arm': {n: a[n] for n in ARMN}})
    json.dump({'target': target, 'wp': wp, 'sols': out}, open('../../output/reach_plan.json', 'w'))
