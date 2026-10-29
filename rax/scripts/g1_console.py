#!/usr/bin/env python3
"""One-keyboard operator console for the G1 under SONIC + RAX.

    RAX_PYTHON scripts/g1_console.py [--session DIR]      (the robot service's python; newest session by default)

Keys (single press, no Enter):
    s      START: the stack's control starts and the robot stands under the planner, then holds (you work the
           gantry). The agent is still blocked until you arm.
    space  ARM / DISARM toggle = start, pause and resume the agent's motion. Disarm stops a running motion within
           one tick; the robot holds its last reference. Arm lets the agent's next call through.
    r      reply "done" to the agent's oldest open request (after a scene reset or a pause, if it asked)
    o      STOP: the stack's stop (every joint damped, the robot goes limp: it must be hanging) and disarm.
           After it, relaunch the deploy before the next START.
    q      quit this console (changes nothing on the robot)

The console only writes files in the session's operator/ directory: armed.json (services.arming, what `g1 arm`
writes) and stack.json (start / stop, read by the robot service's poll_operator). It shows the status the
service writes back (stack_status.json). The deploy terminal's own O and the hardware e-stop stay the outermost
stops.
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from services import arming  # noqa: E402
from services.controller import operator  # noqa: E402
from services.storage_root import artifact_root  # noqa: E402


def newest_session():
    roots = sorted((p.parent for p in (artifact_root()/'sessions').glob('*/operator-input.json')),
                   key=lambda p: (p/'operator-input.json').stat().st_mtime)
    return roots[-1] if roots else None


def send_stack(root, cmd):
    path = Path(root)/'operator'/'stack.json'
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps({'cmd': cmd, 'nonce': uuid.uuid4().hex, 'at': time.time(), 'by': 'g1-console'}))
    tmp.chmod(0o600)
    os.replace(tmp, path)


def read_status(root):
    try:
        return json.loads((Path(root)/'operator'/'stack_status.json').read_text())
    except (OSError, ValueError):
        return None


def do_start(root):
    send_stack(root, 'start')
    return 'START sent: the robot stands (holding its standing reference), then the hold takes over'


def do_stop(root):
    arming.write(root, False, by='g1-console', reason='console stop')
    send_stack(root, 'stop')
    return 'STOP sent: every joint damped, session disarmed; relaunch the deploy before the next START'


def do_toggle_arm(root):
    if arming.read(root)['armed']:
        arming.write(root, False, by='g1-console', reason='paused by the operator console')
        return 'PAUSED (disarmed): a running motion stops within one tick, the robot holds'
    arming.write(root, True, by='g1-console')
    return 'ARMED: the agent may move the robot'


def do_reply(root, text='done'):
    pending = sorted(operator.pending(root), key=lambda r: r['created'])
    if not pending:
        return 'no open request to answer'
    r = pending[0]
    operator.reply(root, r['id'], text if r['kind'] != 'reset' else 'Scene reset done.', by='g1-console')
    return f"answered {r['kind']} request {r['id']}"


def render(root, status, message, now):
    lines = [f"G1 console   session {Path(root).name}   {time.strftime('%H:%M:%S')}", '']
    armed = arming.read(root)
    lines.append(f"agent motion : {'ARMED (agent may move)' if armed['armed'] else 'disarmed / paused  (' + armed.get('reason', '') + ')'}")
    if status is None:
        lines.append('robot service: no status yet (is the robot service running for this session?)')
    else:
        age = now - status.get('at', 0)
        stale = '  [STALE %.0f s]' % age if age > 3 else ''
        lines.append(f"robot service: control {status.get('control_mode')}  primed {status.get('primed')}  "
                     f"executing {status.get('executing')}{stale}")
        st = status.get('state')
        if st is None:
            lines.append('stack state  : none (control not started, or the deploy is not running)')
        else:
            lines.append(f"stack state  : tilt {st['tilt_deg']:5.1f} deg   state age {st['age_s']:.2f} s   index {st['index']}")
        if status.get('fault'):
            lines.append(f"FAULT        : {status['fault']}")
        if status.get('camera_error'):
            lines.append(f"camera       : {status['camera_error'][:90]}")
        lines.append('events:')
        for e in status.get('events', [])[-6:]:
            lines.append(f"   {e.get('at', 0):9.1f}  {e.get('what', '')[:110]}")
    pending = sorted(operator.pending(root), key=lambda r: r['created'])
    lines.append('')
    lines.append(f"agent requests: {len(pending)} open")
    for r in pending[:3]:
        lines.append(f"   {r['kind'].upper():5s} {time.strftime('%H:%M:%S', time.localtime(r['created']))}  {r['message'][:100]}")
    lines += ['', f"> {message}", '',
              '[s] START (stand)   [space] ARM / PAUSE toggle   [r] reply done   [o] STOP (damp)   [q] quit console']
    sys.stdout.write('\x1b[2J\x1b[H' + '\r\n'.join(lines) + '\r\n')
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--session', type=Path, help='session root (default: the newest session)')
    args = ap.parse_args()
    root = args.session or newest_session()
    if root is None:
        sys.exit('no session (scripts/configure_session.py first)')
    if not sys.stdin.isatty():
        sys.exit('needs a terminal')
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    message = 'ready'
    try:
        tty.setcbreak(fd)
        while True:
            now = time.time()
            render(root, read_status(root), message, now)
            ready, _, _ = select.select([fd], [], [], 0.25)
            if not ready:
                continue
            key = os.read(fd, 1).decode(errors='ignore')
            if key == 'q':
                break
            elif key == 's':
                message = do_start(root)
            elif key == ' ':
                message = do_toggle_arm(root)
            elif key == 'r':
                message = do_reply(root)
            elif key == 'o':
                message = do_stop(root)
            elif key:
                message = f'unknown key {key!r}'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print('console closed; the robot keeps its current state (armed: '
              f"{arming.read(root)['armed']})")


if __name__ == '__main__':
    main()
