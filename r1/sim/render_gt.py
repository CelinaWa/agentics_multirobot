"""Render an R1 sim ground-truth recording (r1_sim_server --gt) to mp4: a side view of the right hand and an overview.
  render_gt.py <scene.xml> <gt.npz> <out.mp4> [--from S] [--to S] [--every N]"""
import argparse, numpy as np, mujoco, cv2
ap = argparse.ArgumentParser(); ap.add_argument("scene"); ap.add_argument("gt"); ap.add_argument("out")
ap.add_argument("--from", dest="t0", type=float, default=0); ap.add_argument("--to", dest="t1", type=float, default=1e9)
ap.add_argument("--every", type=int, default=1); ap.add_argument("--still", default=None, help="write one png at this time instead")
a = ap.parse_args()
m = mujoco.MjModel.from_xml_path(a.scene); d = mujoco.MjData(m)
fr = np.load(a.gt)["frames"]
r = mujoco.Renderer(m, 360, 480)
opt = mujoco.MjvOption()
dog = m.body("dog").id; pel = m.body("pelvis").id
def views():
    out = []
    for az, el, dist, look in ((0, -80, 1.0, None), (90, -25, 0.9, None)):
        cam = mujoco.MjvCamera(); cam.azimuth, cam.elevation, cam.distance = az, el, dist
        cam.lookat[:] = 0.4 * d.xpos[dog] + 0.6 * d.xpos[pel]; cam.lookat[2] = d.xpos[dog][2]
        r.update_scene(d, cam, opt); out.append(r.render()[:, :, ::-1])
    return np.hstack(out)
sel = [f for f in fr[::a.every] if a.t0 <= f[0] <= a.t1]
if a.still:
    f = min(fr, key=lambda f: abs(f[0] - float(a.still))); d.qpos[:] = f[1:1 + m.nq]; mujoco.mj_forward(m, d)
    cv2.imwrite(a.out, views()); raise SystemExit
vw = None
for f in sel:
    d.qpos[:] = f[1:1 + m.nq]; mujoco.mj_forward(m, d)
    img = views(); cv2.putText(img, f"t={f[0]:.1f}s", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    if vw is None: vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), 10, (img.shape[1], img.shape[0]))
    vw.write(img)
vw.release(); print("wrote", a.out, len(sel), "frames")
