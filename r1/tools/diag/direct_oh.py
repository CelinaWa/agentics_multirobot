import sys, json, numpy as np, mujoco
sys.path.insert(0,"sim")
from r1_sim_server import Sim
from r1kin import Kinematics
from oh_grasp_core import oh_grasp
from pathlib import Path
sd=Path(f"tools/scratch/scene_test_ft/{sys.argv[1]}")
sim=Sim(sd/"scene_r1_dog_box.xml", next(sd.glob("layout_*.json")), Path("SONIC_R1_100k"))
kin=Kinematics(sd/"r1_piper_cam.xml"); m,d=sim.m,sim.d
gr=oh_grasp(sim, kin, reach_ahead=float(sys.argv[2]), do_overhang=True, lean=int(sys.argv[3]))
print("GRASP", json.dumps({k:gr.get(k) for k in ('side','held','clamped_width_m','lifted_m')}))
if not gr.get('held'): print(json.dumps({"IN_BOX":False})); sys.exit(0)
side=gr['side']; gs=d.site_xpos[m.site(f'{side}_grasp').id].copy()
boxb=m.body("box"); ba=m.jnt_qposadr[boxb.jntadr[0]]; boxq=d.qpos[ba:ba+7].copy()
tt=m.geom("table_top").id; top=d.geom_xpos[tt][2]+m.geom_size[tt][2]
sim.place('box',[gs[0],gs[1],top],boxq[3:7])
for _ in range(25): sim.tick()
pl=sim.place_in_box(side)
print(json.dumps({"IN_BOX":bool(pl.get('in_box'))}))
