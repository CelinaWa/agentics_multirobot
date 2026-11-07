"""Drive the R1 robot service by hand for the first-day checks (README section 8.4), no agent.

Talks to the same backend and the same gates the agent uses, so every number you measure here is the
number the agent will get. One call per invocation; the result is printed as JSON.

    export RAX_ARTIFACT_ROOT=$HOME/rax-runtime PYTHONPATH=$PWD        # from rax/
    python scripts/r1_check.py status
    python scripts/r1_check.py head 0.5           # pitch rad (0.5 = look down), then  head 0
    python scripts/r1_check.py gripper right 0.07 # open;  gripper right 0.0  closes (bridge: < 0.035 = close)
    python scripts/r1_check.py hand right 0.30 -0.21 -0.02     # pelvis frame, metres; add --waist, --secs 3
    python scripts/r1_check.py walk 0.14           # forward metres;  walk 0 30  = turn 30 deg in place
    python scripts/r1_check.py hold                # stop streaming, hold the standing pose
    python scripts/r1_check.py observe             # needs the bridge started with --camera-port/--camera-intrinsics

The bridge address comes from ~/r1_real_options.json (stack_host, stack_port, head_pitch), like a session;
--options points at another file (the deploy-sim test uses port 5594). Motions are armed for --minutes
(default 10) in a scratch session directory: the backend refuses to move otherwise, exactly as in a session.
Somebody must be at the e-stop for every call that moves the robot."""
import argparse
import json
import sys
import tempfile
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services import arming                                   # noqa: E402
from services.r1sonic import tools as T                       # noqa: E402
from services.r1sonic.backend import R1SonicBackend           # noqa: E402

MOVING = {T.MOVE_HAND, T.MOVE_JOINTS, T.GRIPPER, T.HEAD, T.WALK}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--options', default=str(Path.home() / 'r1_real_options.json'),
                    help='backend options JSON (stack_host, stack_port, head_pitch)')
    ap.add_argument('--minutes', type=float, default=10.0, help='arming window for a moving call')
    ap.add_argument('--secs', type=float, default=None, help='motion duration where the tool takes one')
    ap.add_argument('--waist', action='store_true', help='hand: allow the waist to help the reach')
    ap.add_argument('--root', default=None, help='session directory (default: a fresh temp dir)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status')
    sub.add_parser('observe')
    sub.add_parser('hold')
    p = sub.add_parser('head'); p.add_argument('pitch', type=float)
    p = sub.add_parser('gripper'); p.add_argument('arm', choices=('left', 'right')); p.add_argument('width_m', type=float)
    p = sub.add_parser('hand'); p.add_argument('arm', choices=('left', 'right'))
    p.add_argument('x', type=float); p.add_argument('y', type=float); p.add_argument('z', type=float)
    p.add_argument('--gripper', type=float, default=None, help='gripper width to hold during the move')
    p = sub.add_parser('walk'); p.add_argument('forward_m', type=float); p.add_argument('turn_deg', type=float, nargs='?', default=0.0)
    a = ap.parse_args()

    options = json.loads(Path(a.options).read_text())
    root = Path(a.root) if a.root else Path(tempfile.mkdtemp(prefix='r1_check_'))
    (root / 'operator').mkdir(parents=True, exist_ok=True)

    if a.cmd == 'status':
        name, args = T.STATUS, {}
    elif a.cmd == 'observe':
        name, args = T.OBSERVE, {}
    elif a.cmd == 'hold':
        name, args = T.HOLD, {}
    elif a.cmd == 'head':
        name, args = T.HEAD, {'pitch': a.pitch}
    elif a.cmd == 'gripper':
        name, args = T.GRIPPER, {'arm': a.arm, 'width_m': a.width_m}
    elif a.cmd == 'hand':
        name, args = T.MOVE_HAND, {'arm': a.arm, 'position': [a.x, a.y, a.z], 'use_waist': a.waist}
        if a.gripper is not None:
            args['gripper_width_m'] = a.gripper
    else:
        name, args = T.WALK, {'forward_m': a.forward_m, 'turn_deg': a.turn_deg}
    if a.secs is not None and name in (T.MOVE_HAND, T.GRIPPER, T.HEAD):
        args['secs'] = a.secs

    if name in MOVING:
        arming.write(root, True, minutes=a.minutes, by='r1_check.py')
        print(f'[r1_check] ARMED for {a.minutes:g} min -> {name} {json.dumps(args)}  (e-stop in hand?)', file=sys.stderr)
    be = R1SonicBackend(types.SimpleNamespace(backend_options=options, root=root, observation_profile='rgb'))
    try:
        result = be.call(name, args)
    except Exception as e:                                     # a rejected or failed call is the result here
        result = {'error': type(e).__name__, 'message': str(e)}
    print(json.dumps(result, indent=1, default=str))
    print(f'[r1_check] session dir: {root}', file=sys.stderr)


if __name__ == '__main__':
    main()
