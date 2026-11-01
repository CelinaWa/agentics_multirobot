"""Position-only closed-loop reach of one palm (IK with the measured waist, cmd = IK + gain*(cmd-meas))."""
import time
import numpy as np
from sh import g1mod, ik
from g1 import IDX

SEEDS = ([-0.8, 0.6, 0, 0, 0, 0, 0], [-1.2, 0.9, -0.3, 0.3, 0, 0, 0], [-0.5, 1.2, 0.5, -0.3, 0, 0, 0],
         [-1.5, 0.5, 0, 0, 0, 0, 0])


def names(side):
    return ['%s_%s_joint' % (side, a) for a in ik.ARM]


def best_ik(base, side, pos, R=None, extra_seeds=()):
    best = None
    seeds = [[base[n] for n in names(side)]] + list(extra_seeds) + [
        [s[0], s[1] * (1 if side == 'left' else -1)] + s[2:] for s in SEEDS]
    for seed in seeds:
        a, pe, re = ik.solve(side, base, np.array(pos), R, seed=seed)
        sc = pe + 0.02 * re
        if best is None or sc < best[0]:
            best = (sc, a, pe, re)
    return best[1], best[2], best[3]


def reach(g, side, pos, R=None, iters=2, secs=2.0, gain=0.8, max_jump=1.2, max_ik_err=0.03, hands=None):
    log = []
    for it in range(iters + 1):
        g.observe()
        meas = g.joints()
        a, pe, re = best_ik(meas, side, pos, R)
        palm = np.array(g.pose().point('%s_hand_palm_link' % side, 'pelvis')['position_m'])
        err = float(np.linalg.norm(palm - pos))
        log.append({'it': it, 'palm': palm.round(3).tolist(), 'err': round(err, 3), 'ik_err': round(pe, 3),
                    'rot_err': round(re, 2), 'tilt': g.obs.get('tilt_deg')})
        if it == iters or (it > 0 and err < 0.015):
            break
        last = g.base_row()
        tgt = {n: a[n] + gain * float(np.clip(last[9 + IDX[n]] - meas[n], -0.5, 0.5)) for n in names(side)}
        jump = max(abs(tgt[n] - last[9 + IDX[n]]) for n in tgt)
        if pe > max_ik_err or jump > max_jump:
            log.append({'refused': 'ik_err %.3f jump %.2f' % (pe, jump)})
            break
        g.move(tgt, secs=secs, hands=hands)
        time.sleep(0.6)
    return log
