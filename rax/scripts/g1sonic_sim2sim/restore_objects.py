"""Sim operator aid: put a free object back where the scene started it when a contact glitch throws
it out of the room (outside --bounds) or below the floor. Watches sim_headless.py's qpos stream and
sends `place=` keys; logs every restore so the session report can mention it.

    $SONIC_ROOT/.venv_sim/bin/python restore_objects.py --scene <xml> [--objects can,paper,...]
"""
import argparse
import time

import mujoco
import numpy as np
import zmq

from sim_keys import send_keys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--objects", default="can,paper,plate,orange,plum,book")
    ap.add_argument("--bounds", default="-1.0,2.6,-1.7,2.0", help="xmin,xmax,ymin,ymax of the room (robot frame)")
    ap.add_argument("--qpos-port", type=int, default=5591)
    args = ap.parse_args()
    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    xmin, xmax, ymin, ymax = (float(v) for v in args.bounds.split(","))
    objects = {}
    for name in args.objects.split(","):
        body = model.body(name)
        adr = model.jnt_qposadr[body.jntadr[0]]
        objects[name] = (adr, data.qpos[adr:adr + 7].copy())  # start position and orientation
    sub = zmq.Context.instance().socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.connect(f"tcp://127.0.0.1:{args.qpos_port}")
    print(f"[restore] watching {list(objects)}", flush=True)
    while True:
        qpos = np.frombuffer(sub.recv(), dtype=np.float64)
        for name, (adr, home) in objects.items():
            x, y, z = qpos[adr:adr + 3]
            if not (xmin < x < xmax and ymin < y < ymax) or z < -0.1:
                print(f"[restore] {time.strftime('%H:%M:%S')} {name} at ({x:.1f}, {y:.1f}, {z:.2f}): back to {np.round(home[:3], 3).tolist()}",
                      flush=True)
                send_keys([f"place={name},{home[0]:.4f},{home[1]:.4f},{home[2] + 0.01:.4f},"
                           + ",".join(f"{v:.6f}" for v in home[3:7])], pause=0.2,
                          port=5590)
                time.sleep(1.0)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
