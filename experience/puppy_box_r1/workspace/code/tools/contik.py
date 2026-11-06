"""Continuous IK for the right arm with the backend's own model: each goal is solved seeded by the previous solution,
no random restarts, so consecutive joint vectors stay close (no wrist flips while holding).
Usage: PYTHONPATH=pylib python contik.py '<q0 json {joint: rad}>' '<goals json [[x,y,z],...]>' '[ax,ay,az]|null' '[fx,fy,fz]|null'"""
import sys, json
from pathlib import Path
import numpy as np
SK = Path('/home/junjieye/rax-runtime-r1/sessions/20261106T021942Z_puppy_box_r1_cdc45b/agent-workspace/.agents/skills/robot-interface/scripts')
sys.path.insert(0, str(SK))
from r1_kin import Kinematics, ARM

K = Kinematics(SK / 'r1_piper_kin.xml')


def plan(q0, goals, approach=None, fingers=None, weight=0.15):
    q = dict(q0)
    out = []
    for g in goals:
        K.set(q)
        res = K._ik('right', g, approach, dict(q), False, weight, 300, 0.05, fingers)
        qn, err, ang = res
        jump = max(abs(qn[n] - q.get(n, 0.0)) for n in ARM['right'])
        K.set({**q, **qn})
        p, R = K.site('right_grasp')
        out.append({'goal': g, 'q': {n: round(v, 4) for n, v in qn.items()}, 'err': round(err, 4),
                    'appr_deg': round(ang, 1), 'jump': round(jump, 3), 'approach_now': np.round(R[:, 2], 3).tolist(),
                    'fingers_now': np.round(R[:, 1], 3).tolist(), 'p': np.round(p, 3).tolist()})
        q.update(qn)
    return out


def fk(q):
    K.set(q)
    p, R = K.site('right_grasp')
    return p, R


if __name__ == '__main__':
    q0 = json.loads(sys.argv[1]); goals = json.loads(sys.argv[2])
    a = json.loads(sys.argv[3]) if len(sys.argv) > 3 else None
    f = json.loads(sys.argv[4]) if len(sys.argv) > 4 else None
    for r in plan(q0, goals, a, f):
        print(json.dumps(r))


OFF = 0.107   # backend grasp point = local 'right_grasp' site - OFF * approach (calibrated: home, pre-grasp, grasp)


def bfk(q):
    p, R = fk(q)
    return p - OFF * R[:, 2], R


def plan_b(q0, goals, approach=None, fingers=None, weight=0.15, inner=3):
    """Like plan() but goals are BACKEND grasp points."""
    q = dict(q0)
    out = []
    for g in goals:
        g = np.asarray(g, float)
        qn = dict(q)
        for _ in range(inner):
            K.set({**q, **qn})
            _, R = K.site('right_grasp')
            gl = g + OFF * R[:, 2]
            qn, err, ang = K._ik('right', gl, approach, {**q, **qn}, False, weight, 300, 0.05, fingers)
        jump = max(abs(qn[n] - q.get(n, 0.0)) for n in ARM['right'])
        pb, R = bfk({**q, **qn})
        out.append({'goal': g.round(3).tolist(), 'q': {n: round(v, 4) for n, v in qn.items()},
                    'err': round(float(np.linalg.norm(pb - g)), 4), 'appr_deg': round(ang, 1), 'jump': round(jump, 3),
                    'approach_now': np.round(R[:, 2], 3).tolist(), 'fingers_now': np.round(R[:, 1], 3).tolist(),
                    'p': np.round(pb, 3).tolist()})
        q.update(qn)
    return out
