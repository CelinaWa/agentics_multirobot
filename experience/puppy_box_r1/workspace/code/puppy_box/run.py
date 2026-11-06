"""puppy_box_r1 step controller (sim, box-behind layout). Run one step at a time and LOOK at the printed frame.
python run.py <step> [args]
  approach              waist/roll 0, head .5, back -.1 x3, turn 70, turn -70, look
  ready                 right hand to READY (0.30,-0.25,0.08) fingers horizontal, look
  fwd <cmd>             r1_walk forward cmd (arm keeps its pose), look
  look                  observe: hand, width, waist, paw sphere fits (x>.3), edge line
  pre <y>               waist yaw cmd -0.35, hand to (0.28, y, 0.01) open, look   (paw must be visible: x >= ~.38)
  grasp x y z           preview (ik <= .03, fingers horizontal), move there open, close to 0, report width
  lift                  arc lift (6 guarded steps to z .27) then waist-assisted raise to z .38 - NO PAUSE
  carry <cmd>           walk forward holding (width checked before/after), look
  release [dz]          lower dz (default .04), open, hand up/back
  retreat               open, hand up/back, home joints (waist 0), walk back -0.1
Every step prints JSON lines; stops (exit 2/3) on refusal or slip."""
import sys, json, time, math
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T033337Z_puppy_box_r1_7c03aa/agent-workspace')
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T033337Z_puppy_box_r1_7c03aa/agent-workspace/code/puppy_box')
from api.runtime import Context

ctx = Context()
ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
       'right_wrist_roll_joint']
WAIST = ['waist_roll_joint', 'waist_yaw_joint']
HOME = {'right_shoulder_pitch_joint': 0.35, 'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 0.87, 'right_wrist_roll_joint': 0.0, 'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}
READY = [0.30, -0.25, 0.08]
HELD_MIN = 0.02
ARC = [(0.010, 0.04, 0.1), (0.010, 0.04, 0.2), (0.010, 0.04, 0.3), (0.010, 0.04, 0.4), (0.010, 0.04, 0.5),
       (0.005, 0.04, 0.5)]           # (dx toward hip, dz, approach z) per step, from the grasp point
RAISE = [(-0.005, -0.01, 0.035), (-0.005, -0.01, 0.035), (-0.005, -0.0, 0.035)]   # waist-assisted, approach (1,0,.5)


def log(**kw):
    print(json.dumps({'t': time.strftime('%H:%M:%S'), **kw}, default=str), flush=True)


def status():
    return ctx.value('r1_status')


def width():
    return status()['proprio']['gripper_width_m']['right']


def look(tag=''):
    import scene, pawfit, edges
    o = ctx.value('r1_observe')
    f = o['camera']['frame_id']
    pr = o['proprio']
    out = {'frame': f, 'hand': pr['grasp_points_pelvis_m']['right'], 'width': pr['gripper_width_m']['right'],
           'waist_yaw': round(pr['joint_positions_rad'][5], 3), 'tilt': o['tilt_deg']}
    try:
        b = scene.brief(f)
        paws = []
        for pb in b['paw_blobs']:
            if pb['area'] > 150:
                ft = pawfit.fit(f, *pb['pix'])
                if ft and ft['centre'][0] > 0.25 and ft['centre'][2] > -0.08:
                    paws.append({'pix': pb['pix'], 'c': ft['centre'], 'n': ft['n']})
        out['paws'] = paws
        out['box'] = b.get('box')
        L = edges.lines(f, b['table_z'])['lines']
        out['edges'] = [(l['dist'], l['normal_yaw_deg'], l['n_in']) for l in L[:2]]
    except Exception as e:  # perception must not crash the step
        out['percep_error'] = repr(e)
    log(step='look', tag=tag, **out)
    return out


def guarded(pos, approach=(1, 0, 0), use_waist=False, secs=2.5, w=0.0, max_arm=0.45, max_waist=0.6):
    st = status()
    q0 = st['command']['upper_joints']
    rep = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(approach), fingers=[0, 1, 0],
                    gripper_width_m=w, preview_only=True, use_waist=use_waist)
    pv = rep.get('preview', rep)
    reach = rep.get('reachable', pv.get('reachable', False))
    fy = abs(pv['fingers_axis_achieved'][1])
    ja = max(abs(pv['joints'][n] - q0[n]) for n in ARM)
    jw = max([abs(pv['joints'][n] - q0[n]) for n in WAIST if n in pv['joints']] or [0])
    if not reach or fy < 0.85 or ja > max_arm or jw > max_waist:
        log(step='REFUSED', pos=pos, reach=reach, ik=pv.get('ik_error_m'), fy=round(fy, 3), arm_jump=round(ja, 3),
            waist_jump=round(jw, 3), joints=pv['joints'])
        return None
    r = ctx.value('r1_move_hand', arm='right', position=list(pos), approach=list(approach), fingers=[0, 1, 0],
                  gripper_width_m=w, secs=secs, use_waist=use_waist)
    log(step='moved', goal=[round(v, 3) for v in pos], hand=r['grasp_points_pelvis_m']['right'],
        width=r['gripper_width_m']['right'], tilt=r['tilt_deg'], ik=pv.get('ik_error_m'), steps=r.get('steps'))
    return r


def walk(**kw):
    r = ctx.value('r1_walk', **kw)
    log(step='walk', cmd=kw, **{k: r.get(k) for k in ('walked_m', 'pelvis', 'tilt_deg', 'fallen')})
    if r.get('fallen') or (r.get('tilt_deg') or 0) > 8:
        log(step='STOP', why='tilt/fall')
        sys.exit(2)
    return r


def main():
    step, a = sys.argv[1], [float(v) for v in sys.argv[2:]]
    if step == 'approach':
        ctx.value('r1_move_joints', joints={'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}, secs=1.0)
        ctx.value('r1_head', pitch=0.5, secs=1.0)
        for kw in [dict(forward_m=-0.1)] * 3 + [dict(turn_deg=70), dict(turn_deg=-70)]:
            walk(**kw)
        look('approach')
    elif step == 'ready':
        ctx.value('r1_head', pitch=0.5, secs=0.5)
        if guarded(READY, w=0.07, max_arm=2.5, max_waist=1.0) is None:
            sys.exit(2)
        look('ready')
    elif step == 'fwd':
        walk(forward_m=a[0])
        look('fwd')
    elif step == 'look':
        look('look')
    elif step == 'pre':
        ctx.value('r1_move_joints', joints={'waist_yaw_joint': -0.35}, secs=2.0)
        if guarded([0.28, a[0], 0.01], w=0.07, max_arm=1.0) is None:
            sys.exit(2)
        look('pre')
    elif step == 'align':
        if guarded(a[:3], w=0.07, secs=2.0) is None:
            sys.exit(2)
        look('align')
    elif step == 'grasp':
        r = guarded(a[:3], w=0.07, secs=2.5, max_arm=0.9)
        if r is None:
            sys.exit(2)
        g = ctx.value('r1_gripper', arm='right', width_m=0.0, secs=1.5)
        log(step='closed', gripper=g['gripper'], hand=g['grasp_points_pelvis_m']['right'],
            held=g['gripper']['measured_m'] >= HELD_MIN)
    elif step == 'lift':
        w0 = width()
        if w0 < HELD_MIN:
            log(step='NOT_HELD', width=w0)
            sys.exit(3)
        hand = status()['proprio']['grasp_points_pelvis_m']['right']
        x, y, z = hand
        for dx, dz, az in ARC:
            x, z = x + dx, z + dz
            r = guarded([x, y, z], (1, 0, az), secs=2.0)
            if r is None:
                sys.exit(2)
            if r['gripper_width_m']['right'] < HELD_MIN:
                log(step='SLIP', width=r['gripper_width_m']['right'])
                sys.exit(3)
        y = min(y, -0.19)
        for dx, dy, dz in RAISE:
            x, z = x + dx, z + dz
            r = guarded([x, y, z], (1, 0, 0.5), use_waist=True, secs=2.0)
            if r is None:
                sys.exit(2)
            if r['gripper_width_m']['right'] < HELD_MIN:
                log(step='SLIP', width=r['gripper_width_m']['right'])
                sys.exit(3)
        look('lifted')
    elif step == 'carryloop':
        # walk forward holding (no pauses) until the hand is past the box front wall or the edge gets close
        o = look('carry0')
        for i in range(int(a[0]) if a else 4):
            box_x = (o.get('box') or {}).get('x', [9, 9])[0]
            edge = min([e[0] for e in o.get('edges', []) if e[0] > 0.08] or [9])
            log(step='carry_check', hand_x=o['hand'][0], box_front=box_x, edge=edge, width=o['width'])
            if o['width'] < HELD_MIN:
                log(step='SLIP_IN_WALK', width=o['width'])
                sys.exit(3)
            if o['hand'][0] >= box_x + 0.07 or edge < 0.18:
                break
            walk(forward_m=0.15)
            o = look('carry%d' % (i + 1))
        box_x = (o.get('box') or {}).get('x', [9, 9])[0]
        if o['width'] >= HELD_MIN and o['hand'][0] < box_x + 0.03:
            log(step='HOLDING_SHORT', hand_x=o['hand'][0], box_front=box_x)
            sys.exit(4)
        if o['width'] >= HELD_MIN:
            g = ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
            log(step='opened', gripper=g['gripper'])
            time.sleep(1.0)
            look('released')
    elif step == 'carry':
        w0 = width()
        walk(forward_m=a[0])
        o = look('carry')
        if o['width'] < HELD_MIN:
            log(step='SLIP_IN_WALK', width=o['width'], w0=w0)
            sys.exit(3)
    elif step == 'release':
        dz = a[0] if a else 0.04
        x, y, z = status()['proprio']['grasp_points_pelvis_m']['right']
        guarded([x, y, z - dz], (1, 0, 0.5), use_waist=True, secs=2.0)
        g = ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        log(step='opened', gripper=g['gripper'])
        time.sleep(1.0)
        look('released')
    elif step == 'retreat':
        ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
        x, y, z = status()['proprio']['grasp_points_pelvis_m']['right']
        guarded([min(x, 0.42) - 0.05, -0.30, max(z, 0.20) + 0.03], (1, 0, 0.3), use_waist=True, secs=2.0,
                w=0.07, max_arm=2.0, max_waist=1.0)
        guarded([0.25, -0.32, 0.20], (1, 0, 0.3), use_waist=True, secs=2.0, w=0.07, max_arm=2.0, max_waist=1.0)
        ctx.value('r1_move_joints', joints=HOME, secs=2.5)
        walk(forward_m=-0.1)
        look('retreat')
    else:
        print(__doc__)


if __name__ == '__main__':
    main()
