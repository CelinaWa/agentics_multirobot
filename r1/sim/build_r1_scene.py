"""Build the R1 puppy_box scene: R1 (SONIC_R1 26 DOF) with a PiPER parallel gripper on each wrist and an RGB-D camera
in the head, in the same room/table/clear box/plush puppy as the G1 scene (3/eng/recon_dog/build_dog_scene.py, which
lays out the objects; this script swaps the robot in).

  build_r1_scene.py <out dir> [--seed N] [build_dog_scene.py options]

Writes <out dir>/r1_piper_cam.xml (the robot), <out dir>/scene_r1_dog_box.xml (the scene) and, with --seed, the layout
json next to it. Gripper geometry: cad/r1_piper (piper_on_r1_mount.urdf, collision boxes and meshes); the wrist->gripper
mount transform is not measured yet (the URDF says so), so the gripper base sits on the wrist roll axis just past the
wrist mesh, approach direction = wrist x, fingers opening along wrist y."""
import argparse, json, subprocess, sys
import xml.etree.ElementTree as ET
from pathlib import Path

R1 = Path(__file__).resolve().parents[1]
MJCF = R1 / "gr00t-wbc-r1/gear_sonic/data/assets/robot_description/mjcf/r1.xml"
URDF = Path("/home/junjieye/workspace/cad/r1_piper/description/piper_on_r1_mount.urdf")
PIPER_MESHES = R1 / "sim/assets/piper"
DOG_BUILDER = R1.parent / "3/eng/recon_dog/build_dog_scene.py"

MOUNT_X = 0.04            # gripper base at the wrist flange (wrist_roll mesh ends at x ~0.043; was 0.15 = 15cm gap, wrong)
FINGER_Z = 0.1358         # gripper base -> finger joint origin = fingertip plane (URDF piper_joint7)
STROKE = 0.035            # per finger: 0 closed .. 0.035 open (70 mm)
# head RGB-D camera: front of the head yaw link (mesh front at x ~0.08), optical axis along the link x
HEAD_CAM = dict(pos="0.085 0 0.01", xyaxes="0 -1 0 0 0 1", fovy="58")


def E(parent, tag, **kw):
    return ET.SubElement(parent, tag, {k.rstrip("_"): str(v) for k, v in kw.items()})


def boxes(link):
    out = []
    for c in link.findall("collision"):
        size = [float(v) for v in c.find("geometry/box").get("size").split()]
        if min(size) < 0.001:          # the URDF's zero-thickness face patches
            continue
        out.append((c.find("origin").get("xyz"), " ".join(f"{v / 2:.5f}" for v in size)))
    return out


def inertial(link):
    i = link.find("inertial")
    m = float(i.find("mass").get("value")); a = i.find("inertia").attrib
    return dict(pos=i.find("origin").get("xyz"), mass=f"{m:.4f}",
                fullinertia=" ".join(a[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")))


def add_gripper(wrist, side, links):
    base = E(wrist, "body", name=f"{side}_gripper_base", pos=f"{MOUNT_X} 0 0", quat="0.70710678 0 0.70710678 0")
    E(base, "inertial", **inertial(links["piper_gripper_base"]))
    E(base, "geom", type="mesh", mesh="piper_gripper_base", class_="gripper_visual")
    for k, (pos, size) in enumerate(boxes(links["piper_gripper_base"])):
        E(base, "geom", name=f"{side}_gripper_base_c{k}", type="box", pos=pos, size=size, class_="gripper_collision")
    E(base, "site", name=f"{side}_tcp", pos=f"0 0 {FINGER_Z}", size="0.006", rgba="1 0.2 0.2 0.6", group="5")
    E(base, "site", name=f"{side}_grasp", pos=f"0 0 {FINGER_Z - 0.02}", size="0.006", rgba="0.2 1 0.2 0.6", group="5")
    for n, quat, axis, rng in (("7", "0.70710678 0.70710678 0 0", "0 0 1", f"0 {STROKE}"),
                               ("8", "0 0 0.70710678 0.70710678", "0 0 -1", f"{-STROKE} 0")):
        f = E(base, "body", name=f"{side}_gripper_finger{n}", pos=f"0 0 {FINGER_Z}", quat=quat)
        E(f, "inertial", **inertial(links[f"piper_link{n}"]))
        E(f, "joint", name=f"{side}_gripper_slide{n}", type="slide", axis=axis, range=rng, limited="true",
          damping="2", armature="0.01")
        E(f, "geom", type="mesh", mesh=f"piper_link{n}", class_="gripper_visual")
        for k, (pos, size) in enumerate(boxes(links[f"piper_link{n}"])):
            E(f, "geom", name=f"{side}_gripper_finger{n}_c{k}", type="box", pos=pos, size=size, class_="gripper_pad")


def build_robot(out):
    root = ET.parse(MJCF).getroot()
    root.set("model", "r1_piper_cam")
    comp = root.find("compiler"); comp.set("meshdir", str((MJCF.parent / comp.get("meshdir")).resolve()))
    asset = root.find("asset")
    for name, f in (("piper_gripper_base", "gripper_base.STL"), ("piper_link7", "link7.STL"), ("piper_link8", "link8.STL")):
        E(asset, "mesh", name=name, file=str(PIPER_MESHES / f))
    default = root.find("default")
    if default is None:
        default = ET.Element("default"); root.insert(list(root).index(asset), default)
    vis = E(default, "default", class_="gripper_visual"); E(vis, "geom", contype="0", conaffinity="0", group="1", density="0", rgba="0.2 0.2 0.22 1")
    col = E(default, "default", class_="gripper_collision"); E(col, "geom", group="3", rgba="0.5 0.5 0.6 0.3", contype="1", conaffinity="1")
    pad = E(default, "default", class_="gripper_pad")
    E(pad, "geom", group="3", rgba="0.5 0.5 0.6 0.3", contype="1", conaffinity="1", condim="4", friction="1.5 0.02 0.002",
      solref="0.01 1", solimp="0.95 0.99 0.001")
    links = {l.get("name"): l for l in ET.parse(URDF).getroot().findall("link")}
    for side in ("left", "right"):
        add_gripper(root.find(f".//body[@name='{side}_wrist_roll_link']"), side, links)
    head = root.find(".//body[@name='head_yaw_link']")
    E(head, "camera", name="head_cam", **HEAD_CAM)
    act = root.find("actuator")
    # PiPER gripper: position servo per finger (the real one is position-commanded over CAN, ~10 N per URDF effort)
    for side in ("left", "right"):
        for n, sign in (("7", 1), ("8", -1)):
            E(act, "position", name=f"{side}_gripper_f{n}", joint=f"{side}_gripper_slide{n}", kp="400",
              ctrlrange=f"{min(0, sign * STROKE)} {max(0, sign * STROKE)}", forcerange="-12 12")
    path = out / "r1_piper_cam.xml"
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")
    # mesh-free copy for kinematics (the RAX r1sonic backend's FK/IK): bodies, joints, sites and the camera only
    for parent in root.iter():
        for g in list(parent.findall("geom")):
            if g.get("type") == "mesh":
                parent.remove(g)
    for mesh in list(asset.findall("mesh")):
        asset.remove(mesh)
    comp.attrib.pop("meshdir", None)
    (out / "r1_piper_kin.xml").write_text(ET.tostring(root, encoding="unicode") + "\n")
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    a, rest = ap.parse_known_args()
    a.out.mkdir(parents=True, exist_ok=True)
    robot = build_robot(a.out)
    seed = rest[rest.index("--seed") + 1] if "--seed" in rest else "nominal"
    layout = a.out / f"layout_{seed}.json"
    subprocess.run([sys.executable, "-I", str(DOG_BUILDER), str(a.out), "--out", "scene_r1_dog_box.xml",
                    "--layout-json", str(layout), *rest], check=True, stdout=subprocess.DEVNULL)
    scene_path = a.out / "scene_r1_dog_box.xml"
    scene = ET.parse(scene_path).getroot()
    scene.set("model", "r1 dog box scene")
    scene.find("include").set("file", robot.name)
    opt = scene.find("option")
    if opt is None:
        opt = E(scene, "option")
    opt.set("timestep", "0.005"); opt.set("integrator", "implicitfast")
    scene_path.write_text(ET.tostring(scene, encoding="unicode") + "\n")
    print(json.dumps({"scene": str(scene_path), "layout": json.loads(layout.read_text())}))


if __name__ == "__main__":
    main()
