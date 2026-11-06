"""v6 (session 727e14): perception for the backend primitives r1_leg_lift + r1_carry_release.

    python code/puppy_box/v6.py look [tag]            HOME, observe, paw sphere fit, leg axis, front edge, box rim
    python code/puppy_box/v6.py lift [depth] [lift_z] [use_axis] [shift]   r1_leg_lift from the stored look
    python code/puppy_box/v6.py carry [in_m] [lower_m] [max_steps] [step_m]  r1_carry_release, target from the box front wall
    python code/puppy_box/v6.py out [back_m]          open, up, out beside the body, walk back, HOME (v4 out)

State: output/v6_state.json. Exit codes: 0 ok, 2 STOP (perception / precondition), 3 dropped, 4 lift not held.
"""
import sys, json, math, time
from pathlib import Path
import numpy as np
WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS)); sys.path.insert(0, str(WS / 'code' / 'puppy_box'))
import chain
from chain import ctx, perceive, select, leg_axis, box_pose

STATE = WS / 'output' / 'v6_state.json'
LOG = WS / 'output' / 'v6.log'
BOX_FRONT_BEHIND_EDGE = 0.22   # PARAMS box_front_wall_behind_edge_m (sim .21-.25) - fallback when the rim is not seen
BODY_OFF_X = 0.06              # hanging body centroid ahead of the hand (2fbfb7 dead reckoning)

HOME = {'left_shoulder_pitch_joint': 0.35, 'left_shoulder_roll_joint': 0.18, 'left_shoulder_yaw_joint': 0,
        'left_elbow_joint': 0.87, 'left_wrist_roll_joint': 0, 'right_shoulder_pitch_joint': 0.35,
        'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0, 'right_elbow_joint': 0.87,
        'right_wrist_roll_joint': 0, 'waist_yaw_joint': 0, 'waist_roll_joint': 0}


def log(**kw):
    kw['t'] = round(time.time(), 1)
    s = json.dumps(kw, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    print(s, flush=True)
    with open(LOG, 'a') as f:
        f.write(s + '\n')


def load():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save(**kw):
    s = load(); s.update(kw); STATE.write_text(json.dumps(s, indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)))


def home(pitch=0.6):
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=0.5)
    ctx.value('r1_move_joints', joints=HOME, secs=1.0)
    ctx.value('r1_head', pitch=pitch, secs=1.0)


def look(a):
    tag = a[0] if a else 'look'
    home()
    if STATE.exists():
        STATE.unlink()
    o = perceive(tag)
    T, R = select(o)
    if T is None:
        log(step='STOP', why='no clean overhanging paw', paws=o['paws'], frame=o['frame']); sys.exit(2)
    ax = leg_axis(o['frame'], T['c'])
    if ax is None or ax['res'] > 0.004 or not (-10 < ax['yaw'] < 45) or min(e_[1] for e_ in ax['edge']) < -0.037:
        log(step='STOP', why='leg axis not measured reliably', axis=ax, paw=T, frame=o['frame']); sys.exit(2)
    yaw = math.radians(ax['yaw'])
    bp = box_pose(o['frame'], o['table_z'])
    e = o['edge']
    wall = None
    if bp is not None and e is not None and 0.15 < bp['x0'] - e['dist'] < 0.30:
        wall = bp['x0']
    wall_src = 'rim' if wall is not None else 'edge+%.2f' % BOX_FRONT_BEHIND_EDGE
    if wall is None and e is not None:
        wall = e['dist'] + BOX_FRONT_BEHIND_EDGE
    save(frame=o['frame'], paw=T['c'], axis_pt=ax['axis_at_paw'], leg_yaw=ax['yaw'],
         leg=[math.cos(yaw), math.sin(yaw)], edge=e, table_z=o['table_z'], box_pose=bp, wall=wall, wall_src=wall_src,
         overhang=T.get('overhang'), ref=R and R['c'])
    log(step='look', frame=o['frame'], paw=T['c'], n=T['n'], rms=T['rms'], overhang=T.get('overhang'),
        axis_pt=ax['axis_at_paw'], lat0=ax['lat0'], leg_yaw=ax['yaw'], res=ax['res'], edge=e, box_pose=bp, wall=wall,
        wall_src=wall_src, tilt=o['tilt'])


def lift(a):
    s = load()
    depth = a[0] if len(a) > 0 else 0.03
    lz = a[1] if len(a) > 1 else 0.265
    use_axis = (a[2] > 0) if len(a) > 2 else True
    shift = a[3] if len(a) > 3 else 0.0      # report the paw this far further along the leg (moves the tool's hip/arc)
    p = list(s['axis_pt'] if use_axis else s['paw'])
    p[2] = s['paw'][2]
    p[0] += shift * s['leg'][0]; p[1] += shift * s['leg'][1]
    import os
    dyaw = float(os.environ.get('YAW_OFFSET_DEG', '0'))   # rotate the measured leg direction (deg, + = left)
    if dyaw:
        y0 = math.atan2(s['leg'][1], s['leg'][0]) + math.radians(dyaw)
        s['leg'] = [math.cos(y0), math.sin(y0)]
    if os.environ.get('PAW_OVERRIDE'):            # sim diagnostic only: operator-stated paw centre
        p = [float(v) for v in os.environ['PAW_OVERRIDE'].split(',')]
    args = dict(paw_position=[round(float(v), 4) for v in p], leg_direction=[round(float(v), 4) for v in s['leg']],
                depth_m=depth, lift_z_m=lz)
    log(step='leg_lift_call', **args)
    r = ctx.value('r1_leg_lift', **args)
    log(step='leg_lift', reply=r)
    save(lift=r, lift_args=args)
    held = bool(r.get('held'))
    sys.exit(0 if held and not r.get('width_falling') else 4)


def carry(a):
    s = load()
    in_m = a[0] if len(a) > 0 else 0.08
    lower = a[1] if len(a) > 1 else 0.06
    steps = int(a[2]) if len(a) > 2 else 5
    step_m = a[3] if len(a) > 3 else 0.07
    st = ctx.value('r1_status')
    hand = st['proprio']['grasp_points_pelvis_m']['right']
    w = st['proprio']['gripper_width_m']['right']
    # the walk does not move anything in the pelvis frame except the scene: wall measured at look time (arm home);
    # the reach pulled the pelvis ~1-3 cm (ignored, inside the margin)
    tgt = s['wall'] + in_m - (hand[0] + BODY_OFF_X)
    tgt = round(min(0.40, max(0.05, tgt)), 3)
    log(step='carry_call', hand=hand, width=w, wall=s['wall'], target_forward_m=tgt, lower_m=lower, max_steps=steps, step_m=step_m)
    r = ctx.value('r1_carry_release', target_forward_m=tgt, lower_m=lower, max_steps=steps, step_m=step_m)
    log(step='carry', reply=r)
    save(carry=r, carry_target=tgt)
    o = ctx.value('r1_observe')
    log(step='after_release', frame=o['camera']['frame_id'], hand=o['proprio']['grasp_points_pelvis_m']['right'],
        width=o['proprio']['gripper_width_m']['right'], tilt=o['tilt_deg'])
    sys.exit(3 if r.get('dropped') else 0)


def out(a):
    sys.argv = ['v4.py', 'out'] + [str(v) for v in a]
    import v4
    v4.out(a)


if __name__ == '__main__':
    step = sys.argv[1]
    args = [float(v) for v in sys.argv[2:] if v.replace('.', '').replace('-', '').isdigit()]
    if step == 'look':
        look(sys.argv[2:])
    else:
        {'lift': lift, 'carry': carry, 'out': out}[step](args)
