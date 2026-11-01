"""Top-down 'palm-down' left-hand grasp built from joint poses SONIC tracks: wrist roll fixed (+1.3 left),
shoulder pitch/roll/yaw + elbow solved for the palm position, (cmd - meas) bias correction."""
import time
import numpy as np
from scipy.optimize import minimize
from sh import g1mod, ik
from g1 import IDX

CLIP = 0.9
PROX = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow']


# palm-frame directions of the fingers and thumb in the attempt-1 pose (fingers forward, slightly down;
# thumb down and to the outside), measured over the puppy neck in session 20261101T002411Z
FINGER_DIR = np.array([0.94, 0.14, -0.32])
THUMB_DIR = np.array([-0.22, 0.57, -0.80])


def dirs(j, side):
    t = ik.tips(j, side)
    p = np.array(t['palm'])
    f = np.array(t['index_tip']) - p
    th = np.array(t['thumb_tip']) - p
    return f / np.linalg.norm(f), th / np.linalg.norm(th)


def solve4(base, side, pos, wrist=(1.3, 0.0, 0.0), w_dir=0.0):
    """w_dir > 0 also frees wrist pitch/yaw (within +-0.7) to match FINGER_DIR / THUMB_DIR."""
    names = ['%s_%s_joint' % (side, j) for j in PROX]
    if w_dir:
        names = names + ['%s_wrist_pitch_joint' % side, '%s_wrist_yaw_joint' % side]
    wn = ['%s_wrist_%s_joint' % (side, j) for j in ('roll', 'pitch', 'yaw')]
    lo, hi = ik.limits(side)
    j0 = dict(base)
    j0.update(zip(wn, wrist))

    def err(x):
        j = dict(j0)
        j.update(zip(names, x))
        _, t = ik.fk_point(j, '%s_hand_palm_link' % side)
        e = float(np.sum((t - pos) ** 2)) * 100 + 0.001 * float(np.sum((x - x0) ** 2))
        if w_dir:
            f, th = dirs(j, side)
            sg = np.array([1, 1 if side == 'left' else -1, 1])
            e += w_dir * (float(np.sum((f - FINGER_DIR * sg) ** 2)) + float(np.sum((th - THUMB_DIR * sg) ** 2)))
        return e
    x0 = np.array([j0[n] for n in names])
    blo, bhi = list(lo[:4]), list(hi[:4])
    seeds = [x0, np.array([-1.0, 0.25, -0.2, 0.2]), np.array([-0.8, 0.3, 0.0, 0.4])]
    if w_dir:
        blo, bhi = blo + [-0.7, -0.7], bhi + [0.7, 0.7]
        seeds = [x0] + [np.r_[s, 0.0, 0.0] for s in seeds[1:]]
    best = None
    for seed in seeds:
        r = minimize(err, np.clip(seed, blo, bhi), bounds=list(zip(blo, bhi)), method='L-BFGS-B')
        if best is None or r.fun < best.fun:
            best = r
    j = dict(j0)
    j.update(zip(names, best.x))
    _, t = ik.fk_point(j, '%s_hand_palm_link' % side)
    out = dict(zip(wn, wrist))
    out.update(zip(names, best.x))
    return out, float(np.linalg.norm(t - pos))


def palm(g, side='left'):
    return np.array(g.pose().point('%s_hand_palm_link' % side, 'pelvis')['position_m'])


def go(g, side, pos, wrist=(1.3, 0.0, 0.0), iters=2, secs=2.0, gain=0.8, tol=0.015, bias=None, hands=None, w_dir=0.0):
    """Closed-loop palm positioning. bias=None: estimate (cmd - meas) each iteration (free space only);
    pass a fixed dict near contact."""
    log = []
    for it in range(iters + 1):
        g.observe()
        meas = g.joints()
        p = palm(g, side)
        err = float(np.linalg.norm(p - pos))
        log.append({'it': it, 'palm': p.round(3).tolist(), 'err': round(err, 3), 'tilt': g.obs.get('tilt_deg')})
        if it == iters or (it > 0 and err < tol):
            break
        a, pe = solve4(meas, side, np.array(pos), wrist, w_dir)
        last = g.base_row()
        if bias is None:
            b = {n: float(np.clip(last[9 + IDX[n]] - meas[n], -CLIP, CLIP)) for n in a}
        else:
            b = bias
        tgt = {n: a[n] + gain * b.get(n, 0.0) for n in a}
        log[-1]['ik_err'] = round(pe, 3)
        if pe > 0.03:
            log.append({'refused': 'ik_err %.3f' % pe})
            break
        g.move(tgt, secs=secs, hands=hands)
        time.sleep(0.5)
    return log
