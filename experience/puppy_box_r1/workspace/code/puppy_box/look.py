"""Observe (optional head pitch), save nothing new; print edges + paws + box for the frame. python look.py [pitch]"""
import sys, json
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T033337Z_puppy_box_r1_7c03aa/agent-workspace')
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T033337Z_puppy_box_r1_7c03aa/agent-workspace/code/puppy_box')
from api.runtime import Context
import edges, scene
ctx = Context()
if len(sys.argv) > 1:
    ctx.value('r1_head', pitch=float(sys.argv[1]), secs=1.0)
o = ctx.value('r1_observe')
fid = o['camera']['frame_id']
pr = o['proprio']
print(json.dumps({'frame': fid, 'step': o['step_id'], 'tilt': o['tilt_deg'], 'grip': pr['gripper_width_m'],
                  'gp_right': pr['grasp_points_pelvis_m']['right'], 'waist_yaw': pr['joint_positions_rad'][5]}))
b = scene.brief(fid)
print(json.dumps({k: b[k] for k in ('edge_x', 'table_z', 'paw_blobs', 'box')}))
print(json.dumps(edges.lines(fid, b['table_z'])))
