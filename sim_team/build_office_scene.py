"""Build the two-robot office scene: the G1 (29 DoF + Dex3 hands) and the R1 (PiPER grippers) in one MuJoCo world,
with the office from the reference videos (rug, desk, cabinet, drawer unit, mini fridge, lamp, curtain partition,
shelves, trash bin) and the open-top moving box on the floor.

  build_office_scene.py <out dir> [--box-x X --box-y Y --box-yaw DEG] [--table HEIGHT_M]

Writes <out dir>/office_g1_r1.xml (one self-contained MJCF, absolute mesh paths) and <out dir>/layout.json.
Dimensions are estimates from the videos (no tape measure yet): every number is in LAYOUT below, in metres, in the
room frame: x toward the far wall (the fridge wall), y to the left, origin at the middle of the rug's near edge.
The G1 comes first in the model and keeps its own names (the SONIC G1 simulator needs both); the R1's bodies,
joints and actuators are behind the prefix r1_."""
import argparse, json, math
from pathlib import Path
import mujoco

G1_XML = '/opt/sonic/gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand_cam.xml'   # with the head camera
R1_XML = '/home/junjieye/workspace/multirobot/r1/tools/scratch/scene_test_ft/seed21/r1_piper_cam.xml'

LAYOUT = {
    # x toward the back wall, y to the left, origin at the middle of the rug's near edge. The room is about 3.2 m wide:
    # a wall on the left, a black curtain partition on the right, the back wall ahead, open to the lab behind.
    'rug': {'x': [-0.25, 3.15], 'y': [-1.30, 0.80]},
    'back_wall_x': 4.0, 'left_wall_y': 1.6, 'curtain_y': -1.55,
    # along the back wall, from the left to the right: (wooden shelf in the corner), white panel, grey cabinet, drawer unit, fridge, lamp
    # pos = centre x, y; size = x, y, z extents; all on the floor
    'board': {'pos': [3.60, 1.28], 'size': [0.03, 0.90, 1.75], 'yaw_deg': 45.0},   # white panel leaning across the corner
    'cabinet': {'pos': [3.77, 0.50], 'size': [0.45, 0.90, 1.70]},          # tall grey two-door cabinet
    'drawer_unit': {'pos': [3.72, -0.19], 'size': [0.55, 0.42, 0.68]},     # black drawer unit
    'fridge': {'pos': [3.72, -0.68], 'size': [0.55, 0.50, 1.10]},          # two-door mini fridge, flower on top
    'lamp': {'pos': [3.72, -1.20], 'base_r': 0.14, 'height': 1.75},
    'tv': {'pos': [3.97, 0.60, 2.05], 'size': [0.04, 1.20, 0.68]},
    # against the left wall, from the back corner toward the lab: wooden shelf (two bays), wire shelf, pillar
    'trash_bin': {'pos': [3.25, 1.05], 'size': [0.26, 0.34, 0.32]},        # in front of the white panel, at the shelf's foot
    'wood_shelf': {'pos': [3.10, 1.40], 'size': [1.70, 0.40, 2.00]},      # runs into the back corner, beside the white panel
    'wire_shelf': {'pos': [1.72, 1.40], 'size': [1.05, 0.40, 1.80]},
    'pillar': {'pos': [0.90, 1.35], 'size': [0.60, 0.55, 2.60]},
    # the standing desk stands lengthwise toward the back wall, nearer the wooden shelf than the curtain
    'desk': {'pos': [1.50, -0.05], 'size': [1.50, 0.75, 0.76], 'top': 0.03},   # far end about 1.2 m short of the fridge
    'chair': {'pos': [1.50, 0.42], 'yaw_deg': -90.0},                      # visitor chair pushed in on the desk's left side, between desk and shelves
    # the moving box (Bankers Box SmoothMove, open top): outer length, width, height; wall thickness; total mass
    'box': {'size': [0.41, 0.31, 0.26], 'wall': 0.006, 'mass': 1.2},
    # the low table the box may start on (--table): a top just larger than the box, so the robots can stand close
    'table': {'size': [0.47, 0.40], 'top': 0.03},
    'goal': {'pos': [2.85, 0.70]},                     # on the floor beside the trash bin
}
GREY, DARK, WOOD, STEEL = [0.62, 0.64, 0.66, 1], [0.08, 0.08, 0.09, 1], [0.80, 0.62, 0.38, 1], [0.55, 0.57, 0.6, 1]


def quat_z(deg):
    h = math.radians(deg) / 2
    return [math.cos(h), 0, 0, math.sin(h)]


def box_geom(body, name, pos, size, rgba, **kw):
    return body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=[s / 2 for s in size], rgba=rgba, **kw)


def furniture(world, name, item, rgba):
    (x, y), (sx, sy, sz) = item['pos'], item['size']
    box_geom(world, name, [x, y, sz / 2], [sx, sy, sz], rgba)


def shelf(world, name, item, rgba, bays, boards):
    """An open shelf unit against the left wall: posts at the corners of each bay and boards at the given heights."""
    (x, y), (sx, sy, sz) = item['pos'], item['size']
    for k in range(bays + 1):
        for side, dy in (('f', -sy / 2 + 0.02), ('b', sy / 2 - 0.02)):
            box_geom(world, f'{name}_post_{k}{side}', [x - sx / 2 + k * sx / bays, y + dy, sz / 2], [0.04, 0.04, sz], rgba)
    for k, z in enumerate(boards):
        box_geom(world, f'{name}_board_{k}', [x, y, z], [sx, sy, 0.02], rgba)


def chair(world, item):
    """A visitor chair: seat, backrest, armrests and four legs, as one fixed body."""
    x, y = item['pos']
    body = world.add_body(name='chair', pos=[x, y, 0], quat=quat_z(item['yaw_deg']))
    fabric, frame = [0.28, 0.22, 0.22, 1], [0.6, 0.6, 0.62, 1]
    box_geom(body, 'chair_seat', [0, 0, 0.45], [0.48, 0.50, 0.07], fabric)
    box_geom(body, 'chair_back', [-0.25, 0, 0.74], [0.05, 0.48, 0.46], [0.12, 0.12, 0.13, 1])
    for k, (dx, dy) in enumerate(((0.21, 0.24), (0.21, -0.24), (-0.23, 0.24), (-0.23, -0.24))):
        box_geom(body, f'chair_leg_{k}', [dx, dy, 0.21], [0.025, 0.025, 0.42], frame)
    for k, dy in enumerate((0.26, -0.26)):
        box_geom(body, f'chair_arm_{k}', [0.0, dy, 0.66], [0.34, 0.04, 0.03], [0.12, 0.12, 0.13, 1])
        box_geom(body, f'chair_arm_post_{k}', [0.14, dy, 0.55], [0.025, 0.025, 0.20], frame)


def build(out, box_xy, box_yaw, table_height=0.0):
    L = LAYOUT
    # The G1's own model is the base of the scene, so it comes first and keeps its names and defaults: the SONIC G1
    # simulator addresses it by name and by position (root = qpos[0:7], its motors = the first actuators).
    s = mujoco.MjSpec.from_file(G1_XML)
    s.meshdir = str((Path(G1_XML).parent / s.meshdir).resolve())
    s.modelname = 'office_g1_r1'
    s.option.timestep = 0.005
    s.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    s.option.impratio = 10
    s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    s.visual.global_.offwidth, s.visual.global_.offheight = 1280, 960
    s.visual.global_.fovy = 62
    s.visual.headlight.ambient = [0.4, 0.4, 0.4]
    s.visual.headlight.diffuse = [0.55, 0.55, 0.55]
    w = s.worldbody
    # the robots stand at the two ends of the box, facing it: the G1 at end a, the R1 (attached last, behind r1_) at end b
    bl = L['box']['size'][0]
    c, sn = math.cos(math.radians(box_yaw)), math.sin(math.radians(box_yaw))
    stand = (L['table']['size'][0] / 2 + 0.27) if table_height > 0 else (bl / 2 + 0.42)
    poses = {'g1': ([box_xy[0] - stand * c, box_xy[1] - stand * sn, 0], box_yaw),
             'r1': ([box_xy[0] + stand * c, box_xy[1] + stand * sn, 0], box_yaw + 180)}
    pelvis = s.body('pelvis')                      # the G1 where it starts (start the simulator with the same --spawn)
    pelvis.pos = [poses['g1'][0][0], poses['g1'][0][1], pelvis.pos[2]]
    pelvis.quat = quat_z(poses['g1'][1])

    def attach(prefix, path, name):
        child = mujoco.MjSpec.from_file(path)
        meshdir = (Path(path).parent / child.meshdir).resolve()       # its meshes are in another folder than the G1's:
        for mesh in child.meshes:                                     # absolute paths survive the merge and the saved XML
            if mesh.file and not Path(mesh.file).is_absolute():
                mesh.file = str(meshdir / mesh.file)
        child.meshdir = ''
        pos, yaw = poses[name]
        s.attach(child, prefix=prefix, frame=w.add_frame(pos=pos, quat=quat_z(yaw)))
    w.add_light(pos=[1.8, 0, 3.0], dir=[0, 0, -1], diffuse=[0.5, 0.5, 0.5])
    w.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[8, 8, 0.1], rgba=[0.07, 0.07, 0.08, 1], friction=[1.0, 0.005, 0.0001])
    (x0, x1), (y0, y1) = L['rug']['x'], L['rug']['y']
    box_geom(w, 'rug', [(x0 + x1) / 2, (y0 + y1) / 2, 0.001], [x1 - x0, y1 - y0, 0.002], [0.72, 0.72, 0.72, 1], contype=0, conaffinity=0)
    box_geom(w, 'back_wall', [L['back_wall_x'] + 0.05, 0.0, 1.3], [0.1, 3.4, 2.6], [0.93, 0.90, 0.80, 1])
    box_geom(w, 'left_wall', [2.6, L['left_wall_y'] + 0.05, 1.3], [2.9, 0.1, 2.6], [0.93, 0.90, 0.80, 1])
    box_geom(w, 'curtain', [1.9, L['curtain_y'] - 0.03, 1.1], [4.2, 0.06, 2.2], DARK)          # black curtain partition
    furniture(w, 'cabinet', L['cabinet'], GREY)
    furniture(w, 'drawer_unit', L['drawer_unit'], DARK)
    furniture(w, 'fridge', L['fridge'], STEEL)
    (px, py), (psx, psy, psz) = L['board']['pos'], L['board']['size']
    box_geom(w, 'board', [px, py, psz / 2], [psx, psy, psz], [0.95, 0.95, 0.93, 1], quat=quat_z(L['board']['yaw_deg']))
    shelf(w, 'wood_shelf', L['wood_shelf'], WOOD, bays=2, boards=(0.12, 0.62, 1.12))
    shelf(w, 'wire_shelf', L['wire_shelf'], [0.15, 0.15, 0.16, 1], bays=1, boards=(0.15, 0.55, 0.95, 1.35, 1.78))
    furniture(w, 'pillar', L['pillar'], [0.93, 0.90, 0.80, 1])
    chair(w, L['chair'])
    box_geom(w, 'tv', L['tv']['pos'], L['tv']['size'], DARK, contype=0, conaffinity=0)
    fx, fy = L['fridge']['pos']; fz = L['fridge']['size'][2]
    box_geom(w, 'fridge_gap', [fx - L['fridge']['size'][0] / 2 - 0.001, fy, fz * 0.68], [0.002, L['fridge']['size'][1], 0.02], DARK, contype=0, conaffinity=0)
    w.add_geom(name='vase', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[fx, fy, fz + 0.07], size=[0.035, 0.07, 0], rgba=[0.95, 0.95, 0.95, 1], contype=0, conaffinity=0)
    w.add_geom(name='flower', type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[fx, fy, fz + 0.20], size=[0.06, 0, 0], rgba=[0.95, 0.5, 0.6, 1], contype=0, conaffinity=0)
    lx, ly = L['lamp']['pos']
    w.add_geom(name='lamp_base', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[lx, ly, 0.015], size=[L['lamp']['base_r'], 0.015, 0], rgba=GREY)
    w.add_geom(name='lamp_pole', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[lx, ly, L['lamp']['height'] / 2], size=[0.012, L['lamp']['height'] / 2, 0], rgba=GREY)
    w.add_geom(name='lamp_shade', type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[lx, ly, L['lamp']['height']], size=[0.13, 0.05, 0], rgba=[1, 0.95, 0.75, 1], contype=0, conaffinity=0)
    # trash bin: open top, thin walls, fixed
    (bx, by), (bsx, bsy, bsz) = L['trash_bin']['pos'], L['trash_bin']['size']
    for name, pos, size in (('bin_bottom', [bx, by, 0.005], [bsx, bsy, 0.01]),
                            ('bin_front', [bx, by - bsy / 2, bsz / 2], [bsx, 0.008, bsz]), ('bin_back', [bx, by + bsy / 2, bsz / 2], [bsx, 0.008, bsz]),
                            ('bin_near', [bx - bsx / 2, by, bsz / 2], [0.008, bsy, bsz]), ('bin_far', [bx + bsx / 2, by, bsz / 2], [0.008, bsy, bsz])):
        box_geom(w, name, pos, size, DARK)
    # desk: top on two legs
    (dx, dy), (dsx, dsy, dsz) = L['desk']['pos'], L['desk']['size']
    box_geom(w, 'desk_top', [dx, dy, dsz - L['desk']['top'] / 2], [dsx, dsy, L['desk']['top']], WOOD)
    for k, off in enumerate((-dsx / 2 + 0.2, dsx / 2 - 0.2)):          # two legs along the desk's length
        box_geom(w, f'desk_leg_{k}', [dx + off, dy, (dsz - L['desk']['top']) / 2], [0.06, 0.08, dsz - L['desk']['top']], GREY)
        box_geom(w, f'desk_foot_{k}', [dx + off, dy, 0.015], [0.08, dsy * 0.95, 0.03], GREY)
    # goal marker (visual only)
    gx, gy = L['goal']['pos']
    box_geom(w, 'goal', [gx, gy, 0.003], [L['box']['size'][0] + 0.1, L['box']['size'][1] + 0.1, 0.002], [0.2, 0.8, 0.3, 0.5], contype=0, conaffinity=0)
    # the moving box: free body, bottom + four walls, rim edges are what the robots pinch
    bl, bw, bh = L['box']['size']; t = L['box']['wall']; mass = L['box']['mass']
    if table_height > 0:      # a low table under the box: top on a centre column and a foot plate, fixed
        tl, tw = L['table']['size']; tt = L['table']['top']
        table = w.add_body(name='low_table', pos=[box_xy[0], box_xy[1], 0], quat=quat_z(box_yaw))
        box_geom(table, 'low_table_top', [0, 0, table_height - tt / 2], [tl, tw, tt], WOOD, friction=[0.9, 0.01, 0.001])
        box_geom(table, 'low_table_column', [0, 0, (table_height - tt) / 2], [0.10, 0.10, table_height - tt], GREY)
        box_geom(table, 'low_table_foot', [0, 0, 0.01], [0.30, 0.30, 0.02], GREY)
    box = w.add_body(name='box', pos=[box_xy[0], box_xy[1], table_height + 0.002], quat=quat_z(box_yaw))
    box.add_freejoint(name='box_free')
    card = dict(rgba=[0.78, 0.62, 0.42, 1], friction=[0.9, 0.01, 0.001], condim=4, solref=[0.01, 1])
    box_geom(box, 'box_bottom', [0, 0, t / 2], [bl, bw, t], mass=mass * 0.45, **card)       # contents ride on the bottom
    for name, pos, size in (('box_end_a', [-bl / 2 + t / 2, 0, bh / 2], [t, bw, bh]), ('box_end_b', [bl / 2 - t / 2, 0, bh / 2], [t, bw, bh]),
                            ('box_side_l', [0, bw / 2 - t / 2, bh / 2], [bl, t, bh]), ('box_side_r', [0, -bw / 2 + t / 2, bh / 2], [bl, t, bh])):
        box_geom(box, name, pos, size, mass=mass * 0.55 / 4, **card)
    for name, x in (('box_rim_a', -bl / 2 + t / 2), ('box_rim_b', bl / 2 - t / 2)):           # where each robot holds: middle of each end's rim
        box.add_site(name=name, pos=[x, 0, bh], size=[0.01, 0.01, 0.01], rgba=[1, 0.2, 0.2, 1])
    attach('r1_', R1_XML, 'r1')
    w.add_camera(name='overview', pos=[-1.5, -0.3, 2.7], mode=mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY, targetbody='box')   # third-person recording
    model = s.compile()
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    (out / 'office_g1_r1.xml').write_text(s.to_xml())
    layout = {**L, 'box_start': {'pos': list(box_xy), 'yaw_deg': box_yaw, 'z': table_height}, 'robots': {k: {'pos': v[0], 'yaw_deg': v[1]} for k, v in poses.items()}}
    (out / 'layout.json').write_text(json.dumps(layout, indent=1))
    print('bodies', model.nbody, 'joints', model.njnt, 'nq', model.nq, 'actuators', model.nu, 'geoms', model.ngeom)
    return model


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out'); ap.add_argument('--box-x', type=float, default=0.4); ap.add_argument('--box-y', type=float, default=-1.05)
    ap.add_argument('--box-yaw', type=float, default=0.0)
    ap.add_argument('--table', type=float, default=0.0, metavar='HEIGHT_M',
                    help='start the box on a low table of this height instead of on the floor (e.g. 0.45)')
    a = ap.parse_args()
    build(a.out, (a.box_x, a.box_y), a.box_yaw, a.table)
