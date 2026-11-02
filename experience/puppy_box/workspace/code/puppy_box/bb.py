"""Box-behind run for session 8cef10 on top of the library skill pbb.py (one step per call):
  python code/puppy_box/bb.py setstate gx gy top far relx rely relz      # write output/pb3_state.json (box from layout2 rim)
  python code/puppy_box/bb.py hover|grasp|lift|carry|release|lean_release
Carry z 0.27: the rim is at table + 0.14 = +0.08 pelvis here (table -0.0625, legs straight after the walk)."""
import sys, os, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import pbb
pbb.CARRY_Z = 0.27
pbb.pb3.LIFT_Z = pbb.pb3.CARRY_Z = pbb.CARRY_Z
pbb.pb3.GRASP_YAW, pbb.pb3.RELEASE_YAW = float(os.environ.get('GY', '0')), -10.0
BOX = dict(x=[0.39, 0.68], y=[-0.40, -0.05], rim_z=0.0775, table_z=-0.0625, centre=[0.535, -0.225], y_inner=-0.05)
def grasp_deep(ctx, st):
    """Press to top-0.03 then top-0.06 (no early stop at first contact: 8cef10 run 1 stopped at top+0 with the palm 4 cm
    above the plush and the hook slipped at z 0.24); stop only if the palm stops descending or tilt > 7. Then close 3 s."""
    import time
    gx, gy = st['grasp']; top = st['top']; last = None; z = top
    for d in [float(v) for v in os.environ.get('DEPTHS', '0,-0.03,-0.06,-0.08').split(',')]:
        z = top + d
        r = pbb.move(ctx, (gx, gy, z), pbb.pb3.GRASP_YAW, 2.5); r.pop('ob', None)
        print(json.dumps({'cmd_z': z, 'palm': r.get('palm'), 'tilt': r.get('tilt')}), flush=True)
        if not r['ok'] or r['tilt'] > 7:
            pbb.report(False, r.get('why', f"tilt {r.get('tilt')}"))
        if r['palm'][2] <= top + float(os.environ.get('GATE', '0.005')) or r['tilt'] > 6.0:
            break
        if last is not None and r['palm'][2] > last - 0.005:
            break
        last = r['palm'][2]
    pbb.c.pps.ramp(ctx, {}, 3.0, hands=(0.0, 1.0))
    time.sleep(1.0)
    ob, _ = pbb.c.observe(ctx, 'bb_grasp')
    f = pbb.c.fingers(ob, 'right')
    st['grasp_palm'] = r['palm']; st['press_z'] = z; pbb.save(st)
    ok = min(f[3], f[5]) < 1.3 and max(f[3], f[5]) > 0.7
    pbb.report(ok, '' if ok else 'weak or empty close', palm=r['palm'], fingers=[round(v, 2) for v in f], frame=ob['camera']['frame_id'])


pbb.STEPS['grasp_deep'] = grasp_deep


if __name__ == '__main__':
    step = sys.argv[1]
    if step == 'setstate':
        gx, gy, top, far, rx, ry, rz = [float(v) for v in sys.argv[2:9]]
        st = dict(box=BOX, box_side='behind', grasp=[gx, gy], top=top, table_z=-0.0625, release=[rx, ry, rz], far=far,
                  waist_cmd=0.7, press_depth=-0.06)
        pbb.save(st); print(json.dumps(st)); sys.exit(0)
    st = pbb.load()
    from api.runtime import Context
    with Context() as ctx:
        try:
            pbb.STEPS[step](ctx, st)
        except pbb.Fail:
            sys.exit(1)
