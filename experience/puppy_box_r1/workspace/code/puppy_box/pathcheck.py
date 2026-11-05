import sys, json, numpy as np
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T222431Z_puppy_box_r1_e0ac0c/agent-workspace/.agents/skills/robot-interface/scripts')
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T222431Z_puppy_box_r1_e0ac0c/agent-workspace')
from r1_kin import Kinematics
from api.runtime import Context
K = Kinematics('/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T222431Z_puppy_box_r1_e0ac0c/agent-workspace/.agents/skills/robot-interface/scripts/r1_piper_kin.xml')
ARM = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint','right_wrist_roll_joint']
def path(q0, q1, n=20):
    out=[]
    for t in np.linspace(0,1,n):
        q={k: q0[k]+(q1[k]-q0[k])*t for k in q1}
        K.set(q); p,_=K.site('right_grasp'); out.append(p.round(3).tolist())
    return out
if __name__=='__main__':
    with Context() as ctx:
        s=ctx.value('r1_status'); pr=s['proprio']
        cur={n:v for n,v in zip(pr['joint_names'],pr['joint_positions_rad'])}
    targets=json.loads(sys.argv[1])  # list of {joint:angle} dicts (arm only)
    q0={k:cur[k] for k in ARM+['waist_roll_joint','waist_yaw_joint']}
    for tq in targets:
        q1=dict(q0); q1.update(tq)
        pts=path(q0,q1)
        print('->', {k[6:-6]:round(v,2) for k,v in tq.items()})
        print('   ', pts[::4], 'min z where x>0.22:', min([p[2] for p in pts if p[0]>0.22] or [9]))
        q0=q1
