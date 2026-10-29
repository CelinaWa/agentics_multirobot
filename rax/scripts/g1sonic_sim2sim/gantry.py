"""The sim's gantry operator, in the order a real gantry is used: lower the hanging G1 until its feet
touch the floor (the band still carries the pelvis), wait for the deploy stack to enter CONTROL (its
first state message: the agent's first armed hold), and keep the band on until the backend has finished
its start (planner settle, quiet wait, the ease to the default stance), then release it: the robot
leaves the gantry already in its standing pose, as an operator does with the real robot. Releasing
earlier dropped it mid-start into whatever stance it had, and it sometimes slowly fell from there.
Run next to run_sim2sim.sh before a session starts; exits after the release.

    $SONIC_ROOT/.venv_sim/bin/python gantry.py [--state-port 5557] [--settle 8.0]
"""
import argparse
import time

import zmq

from sim_keys import send_keys  # "band=" and "release" repeat safely; "7"/"9" would not


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state-host", default="127.0.0.1")
    ap.add_argument("--state-port", type=int, default=5557)
    ap.add_argument("--settle", type=float, default=8.0, help="seconds of control on the ground before releasing")
    ap.add_argument("--fade", type=float, default=0.0,
                    help="ramp the band's stiffness to zero over this many seconds instead of an abrupt release")
    ap.add_argument("--lower", type=float, default=0.2,
                    help="metres to lower the band (0.2 puts the pelvis at its standing height; more loads the feet)")
    args = ap.parse_args()
    print(f"[gantry] lowering the robot {args.lower:g} m: feet on the floor, the band still holds the pelvis", flush=True)
    steps = [f"band={-0.1 * k:.2f}" for k in range(1, int(args.lower / 0.1 + 1e-6) + 1)]
    if abs(args.lower - 0.1 * len(steps)) > 1e-6:
        steps.append(f"band={-args.lower:.2f}")
    send_keys(steps, pause=1.0)
    sub = zmq.Context.instance().socket(zmq.SUB)
    sub.connect(f"tcp://{args.state_host}:{args.state_port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"g1_debug")
    print(f"[gantry] waiting for the stack's control to start (state on {args.state_port})", flush=True)
    sub.recv()
    print(f"[gantry] control is running; {args.settle:g} s to settle on the ground, then releasing", flush=True)
    time.sleep(args.settle)
    if args.fade > 0:
        send_keys([f"fade={args.fade:g}"])
        time.sleep(args.fade + 0.5)
    else:
        send_keys(["release"])
    print("[gantry] released; the robot stands on its own", flush=True)


if __name__ == "__main__":
    main()
