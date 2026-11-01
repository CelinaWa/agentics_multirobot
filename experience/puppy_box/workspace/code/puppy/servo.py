"""Closed-loop arm positioning: IK with measured waist, command = IK + (last command - measured)."""
import json, sys, time
import numpy as np
sys.path.insert(0, '.')
import ik
from g1 import IDX

ARMN = lambda side: ['%s_%s_joint' % (side, a) for a in ik.ARM]


def solve_here(g, palm, side='left', R=None, seed=None):
    base = g.joints()
    R = ik.claw_rotation(side) if R is None else R
    names = ARMN(side)
    seed = seed or [base[n] for n in names]
    a, pe, re = ik.solve(side, base, np.array(palm), R, seed=seed)
    return a, pe, re


def servo(g, palm, side="left", iters=2, secs=1.5, gain=0.8, R=None, max_bias=0.6, hands=None, tol=0.015, max_jump=1.0):
    """Bring the palm link to `palm` (pelvis) with claw orientation. Returns measured palm and log."""
    log = []
    seed = None
    for it in range(iters + 1):
        g.observe()
        a, pe, re = solve_here(g, palm, side, R, seed)
        seed = [a[n] for n in ARMN(side)]
        meas = g.joints()
        cmd_last = g.base_row()
        tgt = {}
        for n in ARMN(side):
            bias = cmd_last[9 + IDX[n]] - meas[n]
            bias = float(np.clip(bias, -max_bias, max_bias))
            tgt[n] = a[n] + gain * bias
        cur = np.array(g.pose().point('%s_hand_palm_link' % side, 'pelvis')['position_m'])
        err = float(np.linalg.norm(cur - palm))
        log.append({'it': it, 'palm_meas': cur.round(3).tolist(), 'err': round(err, 3), 'ik_err': round(pe, 3)})
        if it > 0 and err < tol:
            break
        if it == iters:
            break
        jump = max(abs(tgt[n] - cmd_last[9 + IDX[n]]) for n in tgt)
        if pe > 0.02 or jump > max_jump:
            raise RuntimeError('servo refused: ik_err %.3f, joint jump %.2f rad (target %s)' % (pe, jump, list(palm)))
        r = g.move(tgt, secs=secs, hands=hands)
        time.sleep(0.8)
    return log


# typical (command - measured) of the free left arm in the claw pose, measured at the pre-grasp
FREE_BIAS = {'left_shoulder_pitch_joint': -0.15, 'left_shoulder_roll_joint': -0.26, 'left_shoulder_yaw_joint': 0.10,
             'left_elbow_joint': 0.27, 'left_wrist_roll_joint': 0.27, 'left_wrist_pitch_joint': -0.13,
             'left_wrist_yaw_joint': -0.18}


def relative(g, d, side='left', secs=2.0, hands=None, bias=None):
    """Move the palm by d (pelvis) keeping its orientation: IK from the measured pose + free-arm bias."""
    g.observe()
    base = g.joints()
    P = g.pose()
    r, t = ik.fk_point(base, '%s_hand_palm_link' % side)
    a, pe, re = ik.solve(side, base, np.array(t) + np.array(d), np.array(r), seed=[base[n] for n in ARMN(side)])
    bias = FREE_BIAS if bias is None else bias
    tgt = {n: a[n] + bias.get(n, 0.0) for n in ARMN(side)}
    g.move(tgt, secs=secs, hands=hands)
    time.sleep(0.8)
    g.observe()
    r2, t2 = ik.fk_point(g.joints(), '%s_hand_palm_link' % side)
    return np.round(t, 3).tolist(), np.round(t2, 3).tolist()


def current_bias(g, side='left'):
    g.observe()
    m = g.joints()
    row = g.base_row()
    return {n: row[9 + IDX[n]] - m[n] for n in ARMN(side)}
