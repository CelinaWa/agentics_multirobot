"""Observe; print hand, waist, paw blobs + sphere fits. python look2.py"""
import sys, json
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace')
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace/code/puppy_box')
from api.runtime import Context
import scene, pawfit
ctx = Context(); o = ctx.value('r1_observe'); f = o['camera']['frame_id']; pr = o['proprio']
print(json.dumps({'frame': f, 'step': o['step_id'], 'hand': pr['grasp_points_pelvis_m']['right'], 'width': pr['gripper_width_m']['right'],
                  'waist_yaw': pr['joint_positions_rad'][5], 'tilt': o['tilt_deg']}))
b = scene.brief(f)
for pb in b['paw_blobs']:
    if pb['area'] > 150:
        u, v = pb['pix']
        print('paw', u, v, json.dumps(pawfit.fit(f, u, v)))
