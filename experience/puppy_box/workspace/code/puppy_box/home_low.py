"""End pose for a stance close to the table (edge <= 0.22 m): waist 0 with the right hand high, then both arms back and
DOWN beside the hips (shoulder pitch 0.4, elbow 0.3), hands open. The skill's home (sp 0.5 / el 0.6) holds the hands
forward at table height ~0.2 m ahead: at a 0.19-0.20 m stance they hit the table edge and the robot stepped 0.17 m by
itself (cab719 run 2). Usage: PYTHONPATH=. python code/puppy_box/home_low.py"""
import sys, os, json, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'skill-library', 'skills', 'plush-pick-place-box-same-side', 'code'))
import controller3 as ctl  # noqa: E402
from carry import waypoint  # noqa: E402
c = ctl.c


def home_low(ctx):
    R = lambda d: c.mirror('right', d); L = lambda d: c.mirror('left', d)
    both = lambda d: dict(R(d), **L(d))
    r = waypoint(ctx, (0.22, -0.22, 0.38), -10, 0.0, 4.0)        # waist back to 0 with the hand high
    if not r['ok']:
        c.pps.ramp(ctx, R({'shoulder_pitch': -0.8, 'elbow': 0.0}), 3.0)
        c.pps.ramp(ctx, {'waist_yaw_joint': 0.0}, 4.0)
    ctx.value('g1_hand_shape', side='right')
    c.pps.ramp(ctx, dict(R({'shoulder_pitch': 0.1, 'shoulder_roll': -0.2, 'shoulder_yaw': 0, 'elbow': -0.9, 'wrist_roll': 0,
                            'wrist_pitch': 0, 'wrist_yaw': 0}), waist_pitch_joint=0.0, waist_roll_joint=0.0), 4.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, both({'shoulder_pitch': 0.9}), 3.0, hands=(0.0, 0.0))            # both hands back, forearms up
    c.pps.ramp(ctx, dict(both({'elbow': 0.3, 'shoulder_yaw': 0, 'wrist_roll': 0, 'wrist_pitch': 0, 'wrist_yaw': 0}),
                         **R({'shoulder_roll': -0.2}), **L({'shoulder_roll': 0.2})), 3.0, hands=(0.0, 0.0))
    c.pps.ramp(ctx, both({'shoulder_pitch': 0.4}), 3.0, hands=(0.0, 0.0))
    time.sleep(2.0)
    ob, path = c.observe(ctx, 'home_low')
    f = c.fingers(ob, 'right') + c.fingers(ob, 'left')
    return dict(ok=ob['tilt_deg'] < 6 and max(f) < 0.5, tilt=ob['tilt_deg'], fingers_max=round(max(f), 2),
                frame=ob['camera']['frame_id'], json=path)


if __name__ == '__main__':
    from api.runtime import Context
    with Context() as ctx:
        print(json.dumps(home_low(ctx), default=float))
