"""Arms for the step-up walk (session 4ddb74): right arm straight down beside the hip (shoulder back first, then elbow),
left arm kept tucked but further back (sp 0.9, el -0.5) so neither hand passes over the puppy lying at the table edge."""
import sys, os, json, time
sys.path[:0] = [os.path.join(os.getcwd(), d) for d in ('code/puppy_box', '.')]
import controller as c
from api.runtime import Context
from api.params import param

if __name__ == '__main__':
    with Context() as ctx:
        R = lambda d: c.mirror('right', d); L = lambda d: c.mirror('left', d)
        c.pps.ramp(ctx, dict(R({'shoulder_pitch': param('walk.right_arm_sp', 0.7)}), **L({'shoulder_pitch': param('walk.left_arm_sp', 0.9), 'elbow': -0.5})), 3.0, hands=(0.0, 0.0))
        c.pps.ramp(ctx, R({'elbow': param('walk.right_arm_el', 0.6)}), 3.0, hands=(0.0, 0.0))
        time.sleep(1.0)
        ob, path = c.observe(ctx, 'walkpose')
        print(json.dumps({'tilt': ob['tilt_deg'], 'frame': ob['camera']['frame_id'], 'json': path}))
