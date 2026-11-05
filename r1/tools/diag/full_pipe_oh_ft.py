"""Full pipeline WITH faithful operator overhang (push dog so frontmost paw ~6cm past the edge, hanging in
open space), then arm-choice width-feedback wedge grasp of the hanging paw -> lift -> box under -> drop.
Live SONIC policy. Operator-equivalent: table slide for reach, dog overhang push, box-under-hand slide."""
import sys, json, numpy as np, argparse
sys.path.insert(0,"sim")
from r1_sim_server import Sim
from r1kin import Kinematics
from pathlib import Path
import mujoco
ap=argparse.ArgumentParser()
ap.add_argument("--seed", default="seed22"); ap.add_argument("--edge_ahead", type=float, default=0.28)
ap.add_argument("--cmdopen", type=float, default=0.050); ap.add_argument("--wedge", type=float, default=0.060)
ap.add_argument("--liftz", type=float, default=0.22); ap.add_argument("--video", default=None)
ap.add_argument("--lean", type=int, default=0)
a=ap.parse_args()
sd=Path(f"tools/scratch/scene_test_ft/{a.seed}")
sim=Sim(sd/"scene_r1_dog_box.xml", next(sd.glob("layout_*.json")), Path("SONIC_R1_100k"), gt_path=a.video)
kin=Kinematics(sd/"r1_piper_cam.xml"); m,d=sim.m,sim.d
pel=m.body("pelvis").id; dog=m.body("dog").id; boxb=m.body("box")
PIN=[None]; BOX=[None]; PINQ=[None]
def pins(): return {b:(m.jnt_qposadr[m.body(b).jntadr[0]], m.jnt_dofadr[m.body(b).jntadr[0]]) for b in ("table","box")}
def run(n):
    for _ in range(n):
        sim.tick()
        if PIN[0]:
            for b,(qa,va) in PIN[0].items():
                tgt = BOX[0] if (b=="box" and BOX[0] is not None) else PINQ[0][b]
                d.qpos[qa:qa+7]=tgt; d.qvel[va:va+6]=0
    if sim.fallen: print("FELL"); sys.exit(1)
def to_pel(p): Rp=d.xmat[pel].reshape(3,3); return Rp.T@(np.asarray(p)-d.xpos[pel])
def gfk(side): kin.set(dict(zip(sim.names,d.qpos[sim.qadr]))); return kin.site(f"{side}_grasp")[0]
A=np.array([1,0,-0.05]); A/=np.linalg.norm(A); F=np.array([0,1.0,0])
def plan(side, goal_pel, grip):
    base=0 if side=="left" else 7; gidx=21 if side=="left" else 22; qi=base+3; idx=0 if side=="left" else 1
    q0=dict(zip(sim.names,d.qpos[sim.qadr])); q,pe,_=kin.ik(side,goal_pel,A,q0=q0,fingers=F)
    pts=kin.points({**q0,**q}); t=sim.row.copy()
    t[base:base+3]=pts[idx][0]; t[base+3:base+7]=pts[idx][1]
    if grip is not None: t[gidx]=grip
    return t,qi,base,pe
def ramp(t,qi,n):
    st=sim.row.copy()
    for k in range(n):
        s=(k+1)/n; s=s*s*(3-2*s); r=st+s*(t-st)
        qa,qb=st[qi:qi+4],t[qi:qi+4].copy()
        if qa@qb<0: qb=-qb
        qq=(1-s)*qa+s*qb; r[qi:qi+4]=qq/np.linalg.norm(qq)
        sim.stream([r]); run(1)
    run(10)
def go(side, goalw, secs, servo, grip, off=(0,0,0)):
    off=np.asarray(off,float)
    g=(to_pel(goalw())+off) if callable(goalw) else (to_pel(goalw)+off)
    t,qi,base,pe=plan(side,g,grip)
    if pe>0.12: return  # IK diverged/unreachable -> don't command garbage (prevents falls)
    ramp(t,qi,int(secs*50)); cmd=t.copy()
    for _ in range(servo):
        g=(to_pel(goalw())+off) if callable(goalw) else (to_pel(goalw)+off)
        e=np.asarray(g)-gfk(side)
        if np.linalg.norm(e)<0.005: break
        cmd[base:base+3]+=0.85*e; ramp(cmd,qi,12)
run(600)
# operator: slide table so edge is edge_ahead of pelvis (reach)
tt=m.geom("table_top").id; pelx=d.xpos[pel][0]
edge_x=d.geom_xpos[tt][0]-m.geom_size[tt][0]
sim.slide_table(-(edge_x - pelx - a.edge_ahead))
mujoco.mj_forward(m,d); run(20)
# operator overhang: push dog forward so frontmost paw ~6cm past edge
edge_x=d.geom_xpos[tt][0]-m.geom_size[tt][0]
paws={n:m.geom(f"dog_paw_{n}").id for n in range(4)}
front_paw_x=min(d.geom_xpos[paws[n]][0] for n in range(4))
dx=float(np.clip((edge_x-0.06)-front_paw_x, -0.25, 0.0))
jb=m.body("dog").jntadr[0]; da=m.jnt_qposadr[jb]
d.qpos[da]+=dx; d.qvel[m.jnt_dofadr[jb]:m.jnt_dofadr[jb]+6]=0
PINQ[0]={b:(d.qpos[m.jnt_qposadr[m.body(b).jntadr[0]]:m.jnt_qposadr[m.body(b).jntadr[0]]+7].copy()) for b in ("table","box")}
PIN[0]=pins(); mujoco.mj_forward(m,d); run(30)
# choose frontmost paw, arm by its side
nearpaw=min(paws, key=lambda n: d.geom_xpos[paws[n]][0]); pg=paws[nearpaw]
paww=lambda: d.geom_xpos[pg].copy(); p0=paww(); dog0=d.xpos[dog].copy()
pely=d.xpos[pel][1]; side="right" if p0[1]<pely else "left"; gidx=21 if side=="left" else 22
top=d.geom_xpos[tt][2]+m.geom_size[tt][2]
print(f"{a.seed} overhang dx={dx:.3f} paw{nearpaw}@{p0.round(3).tolist()} zAbove={(p0[2]-top)*100:.1f}cm side={side} edge_ahead={edge_x-pelx:.2f}")
# torso counter-lean (optional): pre-lean back + away from the reaching arm to keep balance through the reach
if a.lean:
    sim.row[14]-=0.030; sim.row[15]+=(0.025 if side=="right" else -0.025)
    run(80)
else:
    run(40)
# approach from front, slightly below paw height so lower jaw is under the hanging paw (past edge = no table)
go(side, lambda: paww(), 2.2, 5, a.cmdopen, off=np.array([-0.06,0,-0.005]))
import os as _os
if _os.environ.get("GDBG"):
    _wr=float(d.qpos[sim.qadr][sim.names.index(side+"_wrist_roll_joint")])
    _re=float(np.linalg.norm(to_pel(paww())-gfk(side)))
    print(f"[oh] side={side} reach={_re*100:.1f}cm gpos={gfk(side).round(3).tolist()} paw={to_pel(paww()).round(3).tolist()} wr={_wr:+.2f}", flush=True)
xoff=-0.06; grabbed=False
for step in range(10):
    xoff+=0.012; go(side, lambda: paww(), 0.6, 2, a.cmdopen, off=np.array([xoff,0,-0.005]))
    w=sim.gripper_width(side)
    if _os.environ.get("GDBG"):
        print(f"[ohc] xoff={xoff:+.3f} w={w:.3f} gpos={gfk(side).round(3).tolist()}", flush=True)
    cs=[(m.geom(d.contact[i].geom1).name or "",m.geom(d.contact[i].geom2).name or "") for i in range(d.ncon)]
    touch=sorted(set(x for pr in cs if (f"{side}_gripper" in pr[0]+pr[1]) for x in pr if x.startswith("dog")))
    if w>=a.wedge or any("paw" in t for t in touch): grabbed=True; break
if _os.environ.get("GDBG"):
    _cs=[(m.geom(d.contact[i].geom1).name or "",m.geom(d.contact[i].geom2).name or "") for i in range(d.ncon)]
    _tc=sorted(set(x for pr in _cs if (f"{side}_gripper" in pr[0]+pr[1]) for x in pr if x.startswith("dog")))
    print(f"[oh2] preclose reach={np.linalg.norm(to_pel(paww())-gfk(side))*100:.1f}cm gpos={gfk(side).round(3).tolist()} w={sim.gripper_width(side):.3f} xoff={xoff:.3f} contacts={_tc}", flush=True)
sim.row[gidx]=0.0; run(70); cw=sim.gripper_width(side)
if cw<0.01: print(json.dumps(dict(seed=a.seed, stage="grasp", IN_BOX=False, cw=round(cw,4)))); sys.exit(0)
heldpoint=paww().copy()
for dz in np.arange(0.04, a.liftz+1e-6, 0.04):
    go(side, lambda: heldpoint, 1.5, 2, 0.0, off=np.array([0,0,float(dz)]))
    if sim.gripper_width(side)<0.01: print(json.dumps(dict(seed=a.seed, stage="lift", IN_BOX=False))); sys.exit(0)
liftcm=(d.xpos[dog][2]-dog0[2])*100
gs=d.site_xpos[m.site(f'{side}_grasp').id].copy()
ba=m.jnt_qposadr[boxb.jntadr[0]]; boxq=d.qpos[ba:ba+7].copy(); boxq[0]=gs[0]; boxq[1]=gs[1]; boxq[2]=top; BOX[0]=boxq
mujoco.mj_forward(m,d); run(25)
boxtop=d.xpos[m.body('box').id][2]+0.14
go(side, lambda: np.r_[gs[:2], boxtop+0.03], 2.2, 3, 0.0)
sim.row[gidx]=0.074; run(90)
go(side, lambda: np.r_[gs[0]-0.05, gs[1], boxtop+0.15], 1.5, 1, 0.074); run(60)
bx=d.xpos[m.body('box').id]; dg=d.xpos[dog]
inside = abs(dg[0]-bx[0])<0.134 and abs(dg[1]-bx[1])<0.19 and dg[2] < bx[2]+0.16 and dg[2] > bx[2]-0.02
print(json.dumps(dict(seed=a.seed, side=side, lifted_cm=round(liftcm,1), dog=dg.round(3).tolist(),
      box=bx.round(3).tolist(), IN_BOX=bool(inside), fell=sim.fallen)))
if a.video: sim.save_gt()
