"""Serve the sim G1's head camera over HomeBody's camera wire (HBCF) for the RAX g1sonic backend.

Subscribes to sim_headless.py's qpos stream, mirrors the pose in a private MjData of the same scene,
renders colour and depth from --camera with OSMesa, and streams the wire's frames (see
`services.g1sonic.camera.encode_frame`) to every TCP client at --hz. With --snapshot-dir the newest
head and overview images are also written as PNGs; with --record the overview camera is written
as an mp4 at --hz for the whole run (the third-person recording of a session).
"""
import argparse
import math
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import zmq  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from services.g1sonic.camera import encode_frame  # noqa: E402


def intrinsics(model, cam_id, width, height):
    fy = (height / 2.0) / math.tan(math.radians(model.cam_fovy[cam_id]) / 2.0)
    return [[fy, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--camera", default="head_cam")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--shadows", action="store_true", help="render shadows (slow under OSMesa)")
    ap.add_argument("--port", type=int, default=7790)
    ap.add_argument("--qpos-port", type=int, default=5591)
    ap.add_argument("--snapshot-dir", default=None)
    ap.add_argument("--depth-hide-group", type=int, default=4,
                    help="geom group left out of the DEPTH render only (clear plastic a RealSense cannot range); -1 keeps all")
    ap.add_argument("--record", default=None, help="mp4 path: record the scene's overview camera at --hz")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
    assert cam_id >= 0, f"no camera {args.camera!r} in {args.scene}"
    K = intrinsics(model, cam_id, args.width, args.height)
    colour = mujoco.Renderer(model, height=args.height, width=args.width)
    depth = mujoco.Renderer(model, height=args.height, width=args.width)
    depth.enable_depth_rendering()
    if 0 <= args.depth_hide_group < 6:
        depth._scene_option.geomgroup[args.depth_hide_group] = 0
        print(f"[sim_camera] depth render ignores geom group {args.depth_hide_group} (clear plastic)", flush=True)
    overview, writer = None, None
    has_overview = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview") >= 0
    if (args.snapshot_dir or args.record) and has_overview:
        overview = mujoco.Renderer(model, height=480, width=640)
    if not args.shadows:  # software (OSMesa) shadow maps cost ~0.37 s of a 0.47 s colour frame: 1 Hz instead of 4 Hz
        for renderer in (colour, overview):
            if renderer is not None:
                renderer._scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    if args.snapshot_dir:
        Path(args.snapshot_dir).mkdir(parents=True, exist_ok=True)
    if args.record and has_overview:
        Path(args.record).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), args.hz, (640, 480))
        print(f"[sim_camera] recording the overview camera to {args.record} at {args.hz:g} fps", flush=True)

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.connect(f"tcp://127.0.0.1:{args.qpos_port}")

    clients, lock = [], threading.Lock()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(8)

    def accept():
        while True:
            conn, peer = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with lock:
                clients.append(conn)
            print(f"[sim_camera] client {peer}", flush=True)

    threading.Thread(target=accept, daemon=True).start()
    print(f"[sim_camera] READY: {args.camera} {args.width}x{args.height} at {args.hz:g} Hz on tcp port {args.port}, "
          f"fx={K[0][0]:.1f}", flush=True)
    def finish(*_):
        raise SystemExit(0)

    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):  # tmux kill-session sends SIGHUP
        signal.signal(sig, finish)
    try:
        serve(args, model, data, sub, clients, lock, colour, depth, overview, writer, K)
    finally:
        if writer is not None:
            writer.release()  # writes the mp4's index; without it the file does not play
            print(f"[sim_camera] recording closed: {args.record}", flush=True)


def serve(args, model, data, sub, clients, lock, colour, depth, overview, writer, K):
    qpos, frames, period = None, 0, 1.0 / args.hz
    timing_acc = np.zeros(4)
    times_file = open(args.record + ".times", "w") if (writer is not None and args.record) else None  # wall time per video frame
    while True:
        started = time.time()
        try:
            while True:
                qpos = np.frombuffer(sub.recv(zmq.NOBLOCK), dtype=np.float64)
        except zmq.Again:
            pass
        if qpos is not None and qpos.size == model.nq:
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            t1 = time.time()
            colour.update_scene(data, camera=args.camera)
            rgb = colour.render()
            t2 = time.time()
            depth.update_scene(data, camera=args.camera)
            depth_m = depth.render()
            t3 = time.time()
            message = encode_frame(started, rgb, depth_m, K)
            t4 = time.time()
            timing_acc += [t1 - started, t2 - t1, t3 - t2, t4 - t3]
            with lock:
                for conn in list(clients):
                    try:
                        conn.sendall(message)
                    except OSError:
                        clients.remove(conn)
                        conn.close()
            frames += 1
            third_person = None
            if overview is not None and (writer is not None or frames % max(1, int(2 * args.hz)) == 0):
                overview.update_scene(data, camera="overview")
                third_person = np.ascontiguousarray(overview.render()[:, :, ::-1])
            if writer is not None and third_person is not None:
                if times_file is not None:
                    times_file.write(f"{started:.3f}\n")
                    times_file.flush()
                stamp_text = time.strftime("%H:%M:%S")
                cv2.putText(third_person, stamp_text, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                writer.write(third_person)
            if args.snapshot_dir and third_person is not None and frames % max(1, int(2 * args.hz)) == 0:
                cv2.imwrite(str(Path(args.snapshot_dir) / "head_cam.png"), np.ascontiguousarray(rgb[:, :, ::-1]))
                cv2.imwrite(str(Path(args.snapshot_dir) / "overview.png"), third_person)
            if frames % int(10 * args.hz) == 0:
                print(f"[sim_camera] {frames} frames, {len(clients)} clients, depth median {np.median(depth_m):.2f} m; "
                      f"mean s forward/colour/depth/encode {np.round(timing_acc / frames, 3).tolist()}", flush=True)
        time.sleep(max(0.0, period - (time.time() - started)))


if __name__ == "__main__":
    main()
