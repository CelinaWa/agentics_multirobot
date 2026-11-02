"""Deeper-press variant of controller3 step_grasp (session 996d97): after the first palm stall, press ONE more PRESS level
(3 cm) if the tilt stays < 7, close 4 s, report both finger base joints. Same state file (output/pb3_state.json).
python code/puppy_box/grasp2.py [extra_levels=1]"""
import sys, os, json, time
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-same-side/code')
import controller3 as c3
from api.runtime import Context

CLOSE_ONLY = len(sys.argv) > 1 and sys.argv[1] == 'close'   # close where the hand is (press_z given), no press
extra = 1 if CLOSE_ONLY or len(sys.argv) < 2 else int(sys.argv[1])
st = c3.load(); gx, gy = st['grasp']; top = st['top']
levels = list(c3.PRESS) + [c3.PRESS[-1] - 0.03]
hist, stalled, left = [], False, extra
with Context() as ctx:
    for d in ([] if CLOSE_ONLY else levels):
        z = top + d
        r = c3.move(ctx, (gx, gy, z), c3.GRASP_YAW, 2.5); r.pop('ob', None)
        hist.append([round(z, 3), r.get('palm'), r.get('tilt')])
        if stalled and not r['ok'] and 'refused' in str(r.get('why')):   # extra level out of reach: close where we stalled
            r, z = last_r, last_z
            break
        if not r['ok'] or r['tilt'] > 7:
            print(json.dumps({'ok': False, 'why': r.get('why', f"tilt {r.get('tilt')}"), 'hist': hist}, default=float)); sys.exit(1)
        last_r, last_z = r, z
        if stalled:                       # this was the extra level after the first stall
            left -= 1
            if left <= 0:
                break
        elif r['palm'][2] > z + 0.04:     # first stall: palm held up by the plush
            stalled = True
            if extra <= 0 or r['tilt'] > 6.0:
                break
    if CLOSE_ONLY:
        z = float(sys.argv[2]); r = {'palm': None}
    c3.c.pps.ramp(ctx, {}, 4.0, hands=(0.0, 1.0))
    time.sleep(1.0)
    ob, _ = c3.c.observe(ctx, 'pb3_grasp2')
    f = c3.c.fingers(ob, 'right')
    st['grasp_palm'] = r['palm']; st['press_z'] = z; c3.save(st)
    print(json.dumps({'ok': c3.grip(f), 'base_idx_mid': [round(f[5], 2), round(f[3], 2)], 'fingers': [round(v, 2) for v in f],
                      'hist': hist, 'frame': ob['camera']['frame_id']}, default=float))
