"""Palm moves solved on the COMMANDED row (SONIC tracks the whole-body pose: when the waist under-tracks, the arm
compensates, so FK(commanded joints) ~ measured palm within ~1 cm; solving on the measured waist misses by ~10 cm).
python mv.py x y z [palm_yaw_deg] [secs]"""
import sys, os, json, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import controller as c
from reach2 import best


def move(ctx, pos, yaw_deg=30.0, secs=3.0, side='right', hands=None, max_err=0.01, max_jump=1.0, settle=0.5):
    row, meas, _ = c.pps.last_row(ctx)
    cmd = list(row[9:38])
    t, err, _ = c.pps.solve(cmd, side, pos, c.pps.palm_quat(yaw_deg, side))
    if err > max_err:
        err, t = best(cmd, pos, yaw_deg, side)
    jump = max(abs(t[k] - cmd[c.J.index(k)]) for k in t)
    if err > max_err or jump > max_jump:
        return {'ok': False, 'why': f'refused: IK err {err:.3f} jump {jump:.2f}', 'err': err, 'jump': jump}
    rep = c.pps.ramp(ctx, t, secs, hands=hands)
    time.sleep(settle)
    ob = ctx.value('g1_observe')
    j = ob['proprio']['joint_positions_rad']
    palm = c.pps.link_points(j, side)[f'{side}_palm']
    f = c.fingers(ob, side)
    return {'ok': True, 'cmd': list(pos), 'palm': palm, 'fingers': [round(v, 2) for v in f], 'tilt': ob['tilt_deg'],
            'frame': ob['camera']['frame_id'], 'step_id': ob['step_id'], 'waist': [round(j[2], 3), round(j[8], 3)],
            'track': rep['transition']['feedback'].get('max_joint_tracking_error_rad'), 'ob': ob}


if __name__ == '__main__':
    a = [float(v) for v in sys.argv[1:]]
    from api.runtime import Context
    with Context() as ctx:
        r = move(ctx, a[:3], a[3] if len(a) > 3 else 30.0, a[4] if len(a) > 4 else 3.0)
        ob = r.pop('ob', None)
        if ob:
            json.dump(ob, open(f"output/mv_{ob['step_id']}.json", 'w'))
        print(json.dumps(r, default=float))
