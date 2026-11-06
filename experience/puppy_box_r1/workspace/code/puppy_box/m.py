"""Observe and print edge line + paw fits near given seed pixels (or auto paw blobs). python m.py [u,v ...]"""
import sys, json
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T043952Z_puppy_box_r1_cf6e88/agent-workspace')
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T043952Z_puppy_box_r1_cf6e88/agent-workspace/code/puppy_box')
from api.runtime import Context
import scene, edges, pawfit
ctx = Context()
o = ctx.value('r1_observe')
f = o['camera']['frame_id']
pr = o['proprio']
print(json.dumps({'frame': f, 'step': o['step_id'], 'tilt': o['tilt_deg'], 'grip': pr['gripper_width_m'],
                  'gp_right': pr['grasp_points_pelvis_m']['right'], 'waist_yaw': pr['joint_positions_rad'][5]}))
b = scene.brief(f)
print(json.dumps({k: b[k] for k in ('edge_x', 'table_z', 'paw_blobs', 'box')}))
print(json.dumps(edges.lines(f, b['table_z'])['lines'][:2]))
for pb in b['paw_blobs']:
    if pb['area'] > 300:
        u, v = pb['pix']
        print('paw', u, v, json.dumps(pawfit.fit(f, u, v)))
