"""After the approach: r1_grasp_paw, then without delay walk forward with the puppy held and raise the hand over the
box. Stops (holding) before the release so the agent can look. Prints a JSON log line per step.
Usage: python grasp_carry.py '{"walk": 0.2, "lift": [dx, dy, z], "skip_grasp": false}'"""
import sys, json, time
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace')
from api.runtime import Context
args = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
WALK = args.get('walk', 0.2)
LIFT = args.get('lift')            # absolute pelvis-frame hand goal after the walk, or None
t0 = time.time()
def log(step, **kw):
    print(json.dumps({'t': round(time.time() - t0, 1), 'step': step, **kw}, default=str), flush=True)
def width(ctx):
    s = ctx.value('r1_status'); p = s['proprio']
    return p['gripper_width_m']['right'], p['grasp_points_pelvis_m']['right'], s['tilt_deg']
with Context() as ctx:
    try:
        if not args.get('skip_grasp'):
            g = ctx.value('r1_grasp_paw'); log('grasp', **g)
            if not g.get('held') or g.get('side') != 'right':
                log('stop', why='not held by the right hand'); sys.exit(0)
        w, hp, tilt = width(ctx); log('after_grasp', width=w, hand=hp, tilt=tilt)
        if w < 0.01:
            log('stop', why='gripper closed: nothing held'); sys.exit(0)
        if LIFT:
            r = ctx.value('r1_move_hand', arm='right', position=LIFT, gripper_width_m=0.0, use_waist=True, secs=3)
            log('lift', reached=r.get('grasp_points_pelvis_m'), err=r.get('grasp_error_m'), tilt=r.get('tilt_deg'), steps=r.get('steps'))
            w, hp, tilt = width(ctx); log('after_lift', width=w, hand=hp, tilt=tilt)
        if WALK:
            r = ctx.value('r1_walk', forward_m=WALK, turn_deg=0); log('walk', **r)
            w, hp, tilt = width(ctx); log('after_walk', width=w, hand=hp, tilt=tilt)
            if w < 0.01:
                log('stop', why='lost during walk'); sys.exit(0)
        ctx.value('r1_head', pitch=0.6)
        o = ctx.value('r1_observe'); log('observe', frame=o['camera']['frame_id'], step_id=o['step_id'])
    except Exception as e:
        try: ctx.value('r1_hold')
        except Exception: pass
        log('error', why=str(e)[:600])
