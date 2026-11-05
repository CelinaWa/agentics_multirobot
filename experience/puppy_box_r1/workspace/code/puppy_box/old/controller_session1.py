"""puppy_box_r1 step-wise controller (draft, not yet run on the robot: motion was never armed).

Run one step at a time from the agent workspace root:
    python /tmp/claude-1001/ws/code/puppy_box/controller.py <step> [args]
Steps: look | approach | grasp | carry | place | retreat
Each step re-observes, checks its precondition, does one thing, checks the result and prints a JSON
report with the frame ids for the agent to look at. It never chains past a perception checkpoint."""
import json, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import percep
sys.path.insert(0, str(percep.WS))
from api.runtime import Context

HEAD_PITCH_TABLE = 0.5      # rad, looks down at the table front (robot-interface default)
EDGE_TARGET = (0.25, 0.35)  # m, front edge ahead of pelvis for the right-hand grasp (TASK.md)
STEP_FWD = 0.25             # m per walk call near the table (TASK.md: 0.2-0.3)
MAX_TILT = 8.0              # deg, stop and hold above this after any walk
HOME = {'left_shoulder_pitch_joint': 0.35, 'left_shoulder_roll_joint': 0.18, 'left_shoulder_yaw_joint': 0.0,
        'left_elbow_joint': 0.87, 'left_wrist_roll_joint': 0.0, 'right_shoulder_pitch_joint': 0.35,
        'right_shoulder_roll_joint': -0.18, 'right_shoulder_yaw_joint': 0.0, 'right_elbow_joint': 0.87,
        'right_wrist_roll_joint': 0.0, 'waist_roll_joint': 0.0, 'waist_yaw_joint': 0.0}


def observe(ctx):
    o = ctx.value('r1_observe')
    return o, percep.summary(o['camera']['frame_id'])


def look(ctx, pitch=HEAD_PITCH_TABLE):
    ctx.value('r1_head', pitch=pitch, secs=1.0)
    o, s = observe(ctx)
    return {'ok': True, 'measured': {'scene': s, 'tilt_deg': o['tilt_deg']}, 'frames': [o['camera']['directory']],
            'why': 'observed'}


def approach(ctx, max_walks=6):
    """Walk straight in STEP_FWD steps until the table front edge is EDGE_TARGET ahead of the pelvis."""
    log = []
    for i in range(max_walks):
        o, s = observe(ctx)
        tb = s['table']
        if not tb or tb.get('edge_x') is None:
            return {'ok': False, 'measured': {'log': log}, 'frames': [o['camera']['directory']],
                    'why': 'table not found; look at the frame'}
        ex = tb['edge_x']
        log.append({'edge_x': round(ex, 3), 'yaw': round(tb['edge_yaw_deg'], 1), 'view_limited': tb['view_limited']})
        if not tb['view_limited'] and EDGE_TARGET[0] <= ex <= EDGE_TARGET[1]:
            return {'ok': True, 'measured': {'edge_x': ex, 'log': log}, 'frames': [o['camera']['directory']],
                    'why': 'front edge in reach band'}
        if not tb['view_limited'] and ex < EDGE_TARGET[0]:
            fwd = -min(0.2, EDGE_TARGET[0] + 0.05 - ex)
        else:
            fwd = STEP_FWD if (tb['view_limited'] or ex - 0.30 > STEP_FWD) else max(0.05, ex - 0.30)
        w = ctx.value('r1_walk', forward_m=round(fwd, 3), turn_deg=0)
        log[-1]['walk'] = {'cmd': fwd, 'walked_m': w.get('walked_m'), 'tilt': w.get('tilt_deg')}
        if (w.get('tilt_deg') or 0) > MAX_TILT:
            ctx.value('r1_hold')
            return {'ok': False, 'measured': {'log': log}, 'why': f'tilt {w.get("tilt_deg")} after walk; held'}
    return {'ok': False, 'measured': {'log': log}, 'why': 'edge not in band after max walks'}


def grasp(ctx):
    r = ctx.value('r1_grasp_paw')
    o, s = observe(ctx)
    return {'ok': bool(r.get('held')), 'measured': {'grasp': r, 'grip_w': o['proprio']['gripper_width_m']},
            'frames': [o['camera']['directory']],
            'checkpoint': {'subagent': 'haiku', 'frames': [o['camera']['directory'] + '/cam_head.jpg'],
                           'question': 'Is the plush puppy hanging from the right gripper, lifted off the table?',
                           'pass_if': 'yes', 'on_fail': 'open, back off, re-observe, retry once with a small reposition'},
            'why': 'r1_grasp_paw reply'}


def place(ctx, side='right'):
    r = ctx.value('r1_place_in_box', side=side)
    o, s = observe(ctx)
    return {'ok': bool(r.get('in_box')), 'measured': {'place': r}, 'frames': [o['camera']['directory']],
            'checkpoint': {'subagent': 'haiku', 'frames': [o['camera']['directory'] + '/cam_head.jpg'],
                           'question': 'Is the plush puppy body inside the clear box (not on the rim, not beside)?',
                           'pass_if': 'yes', 'on_fail': 'stop; report'},
            'why': 'r1_place_in_box reply'}


def retreat(ctx):
    ctx.value('r1_gripper', arm='right', width_m=0.07, secs=1.0)
    ctx.value('r1_move_joints', joints=HOME, secs=3.0)
    w = ctx.value('r1_walk', forward_m=-0.2, turn_deg=0)
    o, s = observe(ctx)
    return {'ok': True, 'measured': {'walk': w, 'grip_w': o['proprio']['gripper_width_m'], 'tilt': o['tilt_deg']},
            'frames': [o['camera']['directory']], 'why': 'retreated'}


STEPS = {'look': look, 'approach': approach, 'grasp': grasp, 'place': place, 'retreat': retreat}

if __name__ == '__main__':
    step = sys.argv[1]
    kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    with Context() as ctx:
        try:
            res = STEPS[step](ctx, **kw)
        except Exception as e:                      # never leave the robot streaming on an error
            try:
                ctx.value('r1_hold')
            except Exception:
                pass
            res = {'ok': False, 'why': f'{type(e).__name__}: {e}'}
    out = Path(__file__).resolve().parents[2] / 'output' / f'{step}_{int(time.time())}.json'
    out.write_text(json.dumps(res, indent=1, default=str))
    print(json.dumps(res, indent=1, default=str)[:4000])
