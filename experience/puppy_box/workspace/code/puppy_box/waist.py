import sys, time, json
sys.path.insert(0,'code/puppy_box'); import cr
from api.runtime import Context
with Context() as ctx:
    cr.pb3.c.pps.ramp(ctx, {'waist_yaw_joint': float(sys.argv[1])}, 3.0)
    time.sleep(1.5)
    ob, path = cr.pb3.c.observe(ctx, 'cr_look')
    print(json.dumps({'tilt': ob['tilt_deg'], 'waist': ob['proprio']['joint_positions_rad'][2], 'path': path, 'frame': ob['camera']['frame_id']}))
