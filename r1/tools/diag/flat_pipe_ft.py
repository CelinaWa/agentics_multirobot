"""Direct-Sim FLAT grasp attempt (no ZMQ server, no pre-walk; the 600-tick settle inside grasp_paw normalizes the
pose). Slide nearest paw to 0.27 ahead -> grasp_paw -> box under hand -> place_in_box. Reports IN_BOX."""
import sys, json, numpy as np, mujoco
sys.path.insert(0,"sim")
from r1_sim_server import Sim
from pathlib import Path
seed=sys.argv[1]
sd=Path(f"tools/scratch/scene_test_ft/{seed}")
sim=Sim(sd/"scene_r1_dog_box.xml", next(sd.glob("layout_*.json")), Path("SONIC_R1_100k"))
m,d=sim.m,sim.d; pel=m.body("pelvis").id
for _ in range(150): sim.tick()
paws={n:m.geom(f"dog_paw_{n}").id for n in range(4)}
near=min((d.geom_xpos[paws[n]].copy() for n in range(4)), key=lambda p:p[0])
sim.slide_table(-(near[0]-d.xpos[pel][0]-0.27)); mujoco.mj_forward(m,d)
for _ in range(20): sim.tick()
gr=sim.grasp_paw()
if not gr.get('held'):
    print(json.dumps({"seed":seed,"method":"flat","IN_BOX":False,"stage":"grasp","cw":gr.get('clamped_width_m')})); sys.exit(0)
side=gr['side']; gs=d.site_xpos[m.site(f'{side}_grasp').id].copy()
boxb=m.body("box"); ba=m.jnt_qposadr[boxb.jntadr[0]]; boxq=d.qpos[ba:ba+7].copy()
tt=m.geom("table_top").id; top=d.geom_xpos[tt][2]+m.geom_size[tt][2]
sim.place('box', [gs[0], gs[1], top], boxq[3:7])
for _ in range(25): sim.tick()
pl=sim.place_in_box(side)
print(json.dumps({"seed":seed,"method":"flat","IN_BOX":bool(pl.get('in_box')),"fallen":bool(pl.get('fallen'))}))
