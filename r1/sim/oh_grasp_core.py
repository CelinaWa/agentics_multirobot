"""full_pipe_oh's EXACT overhang-grasp sequence, factored so the sim server's edge_grasp op and the standalone share
ONE code path (byte-identical go/plan/ramp on the same Sim) -- the only way to guarantee the server reproduces the
validated standalone grasp despite the open-loop chaos. Operates on a Sim instance `sim` + its Kinematics `kin`."""
import numpy as np, mujoco

def oh_grasp(sim, kin, cmdopen=0.05, wedge=0.06, liftz=0.22, reach_ahead=0.28, do_overhang=True, lean=0):
    m, d = sim.m, sim.d; pel = m.body("pelvis").id
    A = np.array([1.0, 0, -0.05]); A /= np.linalg.norm(A); F = np.array([0, 1.0, 0])
    def run(n):
        for _ in range(n):
            sim.tick()                      # pins table/box via sim.pinned (set after overhang below)
    def to_pel(p):
        Rp = d.xmat[pel].reshape(3, 3); return Rp.T @ (np.asarray(p) - d.xpos[pel])
    def gfk(side):
        kin.set(dict(zip(sim.names, d.qpos[sim.qadr]))); return kin.site(f"{side}_grasp")[0]
    def plan(side, goal_pel, grip):
        base = 0 if side == "left" else 7; gidx = 21 if side == "left" else 22; qi = base + 3; idx = 0 if side == "left" else 1
        q0 = dict(zip(sim.names, d.qpos[sim.qadr])); q, pe, _ = kin.ik(side, goal_pel, A, q0=q0, fingers=F)
        pts = kin.points({**q0, **q}); t = sim.row.copy()
        t[base:base + 3] = pts[idx][0]; t[base + 3:base + 7] = pts[idx][1]
        if grip is not None: t[gidx] = grip
        return t, qi, base, pe
    def ramp(t, qi, n):
        st = sim.row.copy()
        for k in range(n):
            s = (k + 1) / n; s = s * s * (3 - 2 * s); r = st + s * (t - st)
            qa, qb = st[qi:qi + 4], t[qi:qi + 4].copy()
            if qa @ qb < 0: qb = -qb
            qq = (1 - s) * qa + s * qb; r[qi:qi + 4] = qq / np.linalg.norm(qq)
            sim.stream([r]); run(1)
        run(10)
    def go(side, goalw, secs, servo, grip, off=(0, 0, 0)):
        off = np.asarray(off, float)
        g = (to_pel(goalw()) + off) if callable(goalw) else (to_pel(goalw) + off)
        t, qi, base, pe = plan(side, g, grip)
        if pe > 0.12: return            # IK diverged -> don't command garbage (prevents falls)
        ramp(t, qi, int(secs * 50)); cmd = t.copy()
        for _ in range(servo):
            g = (to_pel(goalw()) + off) if callable(goalw) else (to_pel(goalw) + off)
            e = np.asarray(g) - gfk(side)
            if np.linalg.norm(e) < 0.005: break
            cmd[base:base + 3] += 0.85 * e; ramp(cmd, qi, 12)
    run(600)
    tt = m.geom("table_top").id; pelx = d.xpos[pel][0]
    edge_x = d.geom_xpos[tt][0] - m.geom_size[tt][0]
    sim.slide_table(-(edge_x - pelx - reach_ahead)); mujoco.mj_forward(m, d); run(20)
    if do_overhang:
        edge_x = d.geom_xpos[tt][0] - m.geom_size[tt][0]
        paws = {n: m.geom(f"dog_paw_{n}").id for n in range(4)}
        front_paw_x = min(d.geom_xpos[paws[n]][0] for n in range(4))
        dx = float(np.clip((edge_x - 0.06) - front_paw_x, -0.25, 0.0))
        jb = m.body("dog").jntadr[0]; da = m.jnt_qposadr[jb]
        d.qpos[da] += dx; d.qvel[m.jnt_dofadr[jb]:m.jnt_dofadr[jb] + 6] = 0
        sim._pin("table", "box")             # pin BEFORE run(30) (full_pipe_oh_ft's PINQ timing: hold the table/box
        mujoco.mj_forward(m, d); run(30)     # at the slid+overhang position so the edge doesn't drift under the settle)
    else:
        sim._pin("table", "box")
    paws = {n: m.geom(f"dog_paw_{n}").id for n in range(4)}
    nearpaw = min(paws, key=lambda n: d.geom_xpos[paws[n]][0]); pg = paws[nearpaw]
    paww = lambda: d.geom_xpos[pg].copy(); p0 = paww(); dog0 = d.xpos[m.body("dog").id].copy()
    pely = d.xpos[pel][1]; side = "right" if p0[1] < pely else "left"; gidx = 21 if side == "left" else 22
    if lean:                                  # torso counter-lean (full_pipe_oh_ft --lean): balance through the reach
        sim.row[14] -= 0.030; sim.row[15] += (0.025 if side == "right" else -0.025); run(80)
    else:
        run(40)
    import os as _os
    if _os.environ.get("GDBG"):
        print(f"[st] side={side} pelvis={d.xpos[pel].round(3).tolist()} tilt={sim.tilt_deg():.2f} paw_pel={to_pel(paww()).round(3).tolist()} edge_ahead={(d.geom_xpos[tt][0]-m.geom_size[tt][0]-d.xpos[pel][0]):.3f}", flush=True)
    go(side, lambda: paww(), 2.2, 5, cmdopen, off=np.array([-0.06, 0, -0.005]))
    xoff = -0.06; grabbed = False
    for step in range(10):
        xoff += 0.012; go(side, lambda: paww(), 0.6, 2, cmdopen, off=np.array([xoff, 0, -0.005]))
        if sim.gripper_width(side) >= wedge: grabbed = True; break
    sim.row[gidx] = 0.0; run(70); cw = sim.gripper_width(side)
    if cw < 0.01:
        return {"side": side, "paw": int(nearpaw), "wedged": bool(grabbed), "held": False, "clamped_width_m": round(float(cw), 4), "lifted_m": 0.0}
    heldpoint = paww().copy(); held = True
    for dz in np.arange(0.04, liftz + 1e-6, 0.04):
        go(side, lambda: heldpoint, 1.5, 2, 0.0, off=np.array([0, 0, float(dz)]))
        if sim.gripper_width(side) < 0.01: held = False; break
    lifted = float(d.xpos[m.body("dog").id][2] - dog0[2])
    return {"side": side, "paw": int(nearpaw), "wedged": bool(grabbed), "held": bool(held), "clamped_width_m": round(float(cw), 4), "lifted_m": round(lifted, 3)}
