"""Test the faithful edge_grasp primitive: edge_grasp(reach_ahead, lean, do_overhang) replicates full_pipe_oh's flow."""
import sys, json, numpy as np
sys.path.insert(0,"sim"); from r1_sim_server import Sim; from pathlib import Path
seed=sys.argv[1]; EA=float(sys.argv[2]); L=int(sys.argv[3])
sd=Path(f"tools/scratch/scene_test_ft/{seed}")
sim=Sim(sd/"scene_r1_dog_box.xml", next(sd.glob("layout_*.json")), Path("SONIC_R1_100k"))
m,d=sim.m,sim.d; tt=m.geom("table_top").id
gr=sim.edge_grasp(reach_ahead=EA, do_overhang=True)
if not gr.get('held'):
    print(json.dumps({"seed":seed,"IN_BOX":False,"stage":"grasp","cw":gr.get('clamped_width_m'),"side":gr.get('side')})); sys.exit(0)
side=gr['side']; gs=d.site_xpos[m.site(f'{side}_grasp').id].copy()
boxb=m.body("box"); ba=m.jnt_qposadr[boxb.jntadr[0]]; boxq=d.qpos[ba:ba+7].copy()
top=d.geom_xpos[tt][2]+m.geom_size[tt][2]
sim.place('box',[gs[0],gs[1],top],boxq[3:7])
for _ in range(25): sim.tick()
pl=sim.place_in_box(side)
print(json.dumps({"seed":seed,"IN_BOX":bool(pl.get('in_box')),"side":side,"fallen":bool(pl.get('fallen'))}))
