"""Merge the lab's office (OmniRobot g1_sim, a code-built MuJoCo scene) into gear_sonic's G1 sim scene.

Writes, next to gear_sonic's robot model:
  g1_29dof_with_hand_cam.xml  the robot model plus a `head_cam` on torso_link at the URDF d435_link pose
                              (position and tilt from the RAX g1 model's `frame_transform('camera')`)
  scene_office.xml            gear_sonic's lighting, skybox and friction defaults, the robot, and the
                              office's materials and world bodies shifted so the robot stands where the
                              office's ego camera was (facing the chair and the desk)
Office physics defaults go on an `office` class, so the robot's contacts stay as in gear_sonic's scene;
the free objects' contacts (solref 8 ms) are softened to 20 ms because gear_sonic's sim steps at 5 ms,
and free objects lighter than MIN_MASS kg are made that heavy: with the stiff settings a touch of the
Dex3 fingers launched the 6 g paper and 24 g can tens of metres. The floor, rug and furniture keep the
office's stiff contacts: a soft floor left the robot standing tilted and split-legged.
Usage: build_office_scene.py <office.xml> <g1 model dir> [--shift X Y] [--move NAME X Y ...] [--render DIR]
--move places a world body at X Y in the robot's frame (after the shift), e.g. the chair out of the way.
"""
import argparse
import copy
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

MIN_MASS = 0.03
# Air drag stand-in for crumpled paper (a 30 g frictionless sphere otherwise flies metres off a finger
# touch): velocity damping on its free joint, terminal fall speed m g / d of about 1 m/s.
DAMPED_JOINTS = {"paper_free": "0.3"}
SOFT_CONTACT = {"solref": "0.02 1", "solimp": "0.9 0.95 0.001"}
HEAD_CAM = ('<camera name="head_cam" pos="0.05762 0.01753 0.42987" xyaxes="0 -1 0 0.73846 0 0.6743" '
            'fovy="58"/>')  # d435_link in torso_link: x forward tilted 47.6 deg down; MuJoCo cameras look along -z


def shifted(text, dx, dy, stride=3):
    values = [float(v) for v in text.split()]
    for i in range(0, len(values), stride):
        values[i] += dx
        values[i + 1] += dy
    return " ".join(f"{v:.6g}" for v in values)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("office")
    ap.add_argument("model_dir")
    ap.add_argument("--shift", nargs=2, type=float, default=(0.7, 0.2), metavar=("X", "Y"))
    ap.add_argument("--move", nargs=3, action="append", default=[], metavar=("NAME", "X", "Y"),
                    help="put this world body at X Y (robot frame, after the shift); repeatable")
    ap.add_argument("--render", default=None, help="directory for head_cam / overview PNGs of the built scene")
    args = ap.parse_args()
    model_dir = Path(args.model_dir)
    dx, dy = args.shift

    robot = (model_dir / "g1_29dof_with_hand.xml").read_text()
    robot_cam, n = re.subn(r'(<body name="torso_link"[^>]*>)', r"\1\n      " + HEAD_CAM, robot, count=1)
    assert n == 1, "torso_link body not found"
    (model_dir / "g1_29dof_with_hand_cam.xml").write_text(robot_cam)

    office = ET.parse(args.office).getroot()
    scene = ET.Element("mujoco", model="g1_43dof office scene")
    ET.SubElement(scene, "include", file="g1_29dof_with_hand_cam.xml")
    ET.SubElement(scene, "statistic", center="0 0 0.5", extent="2.0")
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3", specular="0 0 0")
    ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")
    ET.SubElement(visual, "global", azimuth="-130", elevation="-20")
    asset = ET.SubElement(scene, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="3072")
    for element in office.find("asset"):
        asset.append(copy.deepcopy(element))
    default = ET.SubElement(scene, "default")
    ET.SubElement(default, "geom", friction="1.0")
    office_class = ET.SubElement(default, "default", {"class": "office"})
    for element in office.find("default"):
        element = copy.deepcopy(element)
        office_class.append(element)
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", pos="0 0 1.5", dir="0 0 -1", directional="true")
    ET.SubElement(world, "site", name="com_marker", pos="0.1 0 0", size="0.05", rgba="1 0 0 0", type="sphere")  # invisible to head_cam
    names = []
    for element in office.find("worldbody"):
        element = copy.deepcopy(element)
        if "pos" in element.attrib:
            element.set("pos", shifted(element.get("pos"), dx, dy))
        elif "fromto" in element.attrib:
            element.set("fromto", shifted(element.get("fromto"), dx, dy))
        elif element.tag in ("body", "geom", "camera", "site"):
            element.set("pos", f"{dx:g} {dy:g} 0")
        if element.tag == "geom":
            element.set("class", "office")
        elif element.tag == "body":
            element.set("childclass", "office")
            for name, x, y in args.move:
                if element.get("name") == name:
                    z = element.get("pos", "0 0 0").split()[2]
                    element.set("pos", f"{float(x):g} {float(y):g} {z}")
                    print(f"moved {name} to {x} {y} {z}")
        if element.tag == "body" and element.find("freejoint") is not None:
            free = element.find("freejoint")
            if free.get("name") in DAMPED_JOINTS:  # MuJoCo has no damping on a freejoint: use a free joint
                free.tag = "joint"
                free.set("type", "free")
                free.set("damping", DAMPED_JOINTS[free.get("name")])
            for geom in element.iter("geom"):
                geom.attrib.update(SOFT_CONTACT)
                if "mass" in geom.attrib and float(geom.get("mass")) < MIN_MASS and geom.get("contype") != "0":
                    geom.set("mass", f"{MIN_MASS:g}")
        names.append(f'{element.tag}:{element.get("name", "?")}')
        world.append(element)
    ET.indent(scene, space="  ")
    out = model_dir / "scene_office.xml"
    out.write_text(ET.tostring(scene, encoding="unicode") + "\n")
    print(f"wrote {out} with {len(names)} office elements: {', '.join(names[:12])} ...")

    import mujoco
    import numpy as np
    model = mujoco.MjModel.from_xml_path(str(out))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base = mujoco.MjModel.from_xml_path(str(model_dir / "scene_43dof.xml"))
    print(f"office scene: nbody {model.nbody} (base {base.nbody}), ngeom {model.ngeom} (base {base.ngeom}), nq {model.nq} (base {base.nq})")
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_cam")
    assert cam >= 0, "head_cam missing"
    print("head_cam world position at the default pose:", np.round(data.cam_xpos[cam], 3), "fovy", model.cam_fovy[cam])
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    print("pelvis:", np.round(data.xpos[pelvis], 3))
    if args.render:
        from PIL import Image
        renderer = mujoco.Renderer(model, height=480, width=640)
        for name in ("head_cam", "overview"):
            renderer.update_scene(data, camera=name)
            Image.fromarray(renderer.render()).save(Path(args.render) / f"office_{name}.png")
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="head_cam")
        depth = renderer.render()
        print(f"head_cam depth: min {depth.min():.2f} m, median {np.median(depth):.2f} m")
        print("rendered", args.render)


if __name__ == "__main__":
    main()
