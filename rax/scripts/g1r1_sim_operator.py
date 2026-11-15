"""The operator of a g1r1 session, played by a script when the robots are the office simulation (sim_team/): it does
what a person does beside the real robots. It arms the session, lowers the G1 on its gantry, and answers the agent's requests. A reset request puts the box back where it started (the robots stay where they are);
after a G1 fall it starts the G1's stack again and hangs the G1 back on its gantry at its start;
any other request gets a plain answer that no further help exists here. Run on the host beside the launcher:

  PYTHONPATH=rax $RAX_PYTHON rax/scripts/g1r1_sim_operator.py --session <session root> [--minutes 120]
"""
import argparse, json, os, re, shutil, subprocess, time
from pathlib import Path
import zmq
from services import arming
from services.controller import operator
from services.controller.frontend import robot_clock
import attempt_review    # (this script's own folder is on the path)
import can_task
import human_view

SIM = Path(__file__).resolve().parents[2] / 'sim_team'      # the office simulation (sim_team/team_sim.sh)
# several simulations side by side (sim_team/team_sim.sh): TEAM_INSTANCE=n shifts every port by 100 n and uses logs<n>/
INSTANCE = int(os.environ.get('TEAM_INSTANCE', '0') or 0)
OFF = 100 * INSTANCE
KEYS, STATE = 5790 + OFF, 5797 + OFF
P_R1, P_QPOS, P_TOUCH, P_POSE = 5791 + OFF, 5792 + OFF, 5793 + OFF, 5796 + OFF
LOGS = 'logs' + (str(INSTANCE) if INSTANCE else '')
CONTAINER = 'sonic-sim2sim'


def log(*parts):
    print(time.strftime('%H:%M:%S'), '[operator]', *parts, flush=True)


def standing(ctx):
    """Both robots on their feet, from the simulation itself: the R1's own stack says so, the G1's pelvis is up."""
    import numpy as np
    try:
        r1 = ctx.socket(zmq.REQ); r1.setsockopt(zmq.LINGER, 0); r1.setsockopt(zmq.RCVTIMEO, 3000); r1.connect(f'tcp://127.0.0.1:{P_R1}')
        r1.send(json.dumps({'op': 'state'}).encode()); state = json.loads(r1.recv_multipart()[0]); r1.close()
        sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect(f'tcp://127.0.0.1:{P_QPOS}')
        qpos = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
        return (not state.get('fallen')) and state.get('tilt_deg', 90) < 25 and qpos[2] > 0.35
    except Exception:
        return False


def box_height(ctx):
    import numpy as np
    try:
        sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect(f'tcp://127.0.0.1:{P_QPOS}')
        q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
        return float(q[52])
    except Exception:
        return -1.0


_TOUCH = {}


def hands_on_box(ctx):
    """{'g1' / 'r1': {'left', 'right': bool}}: which hands touch the box, from the simulation's contacts; None without it."""
    try:
        if 'sock' not in _TOUCH:
            sock = ctx.socket(zmq.SUB); sock.setsockopt(zmq.SUBSCRIBE, b''); sock.setsockopt(zmq.CONFLATE, 1)
            sock.connect(f'tcp://127.0.0.1:{P_TOUCH}'); _TOUCH['sock'] = sock
            time.sleep(0.2)
        message = json.loads(_TOUCH['sock'].recv_string(zmq.NOBLOCK))
        _TOUCH['last'] = message
    except zmq.Again:
        message = _TOUCH.get('last')
    except Exception:
        return None
    if not message or time.time() - message.get('t', 0) > 2.0:
        return None
    return message.get('hands_on_box')


def truth(ctx, layout):
    """Where the box and the robots are in the simulation itself, against the task's success condition."""
    import math
    import numpy as np
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect(f'tcp://127.0.0.1:{P_QPOS}')
    try:
        q = np.frombuffer(sub.recv(), dtype=np.float64)
    finally:
        sub.close()
    if layout.get('task') == 'can_bin':
        hands_on_box(ctx)
        return can_task.truth(q, layout, (_TOUCH.get('last') or {}).get('hands_on'))
    tilt = lambda w, x, y, z: math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    gx, gy = layout['goal']['pos'][:2]
    (bx, by) = layout['box_start']['pos'][:2]
    distance, box_tilt = math.hypot(q[50] - gx, q[51] - gy), tilt(*q[53:57])
    up = q[2] > 0.5 and tilt(*q[3:7]) < 25 and q[59] > 0.45 and tilt(*q[60:64]) < 25
    on_start = layout['box_start'].get('z', 0.0)
    w, x, y, z = q[53:57]
    hands = hands_on_box(ctx)
    return {**({'hands_on_box': hands, 'both_robots_on_box': bool(all(hands['g1'].values()) and all(hands['r1'].values()))}
               if hands else {}),
            'box': [round(float(v), 3) for v in q[50:53]], 'box_tilt_deg': round(box_tilt, 1),
            'box_yaw_deg': round(math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), 1),
            'box_from_start_m': round(math.hypot(q[50] - bx, q[51] - by), 3),
            'box_to_goal_m': round(distance, 3), 'box_lifted': bool(q[52] > on_start + 0.03),
            'g1': [round(float(v), 3) for v in q[0:3]], 'g1_tilt_deg': round(tilt(*q[3:7]), 1),
            'r1': [round(float(v), 3) for v in q[57:60]], 'r1_tilt_deg': round(tilt(*q[60:64]), 1),
            'robots_standing': bool(up),
            'success': bool(distance <= 0.35 and box_tilt < 20 and q[52] < 0.05 and up)}


def end_state(ctx, layout):
    """The ground truth in one line (for the operator's log)."""
    try:
        t = truth(ctx, layout)
    except Exception as error:
        return f'no simulation state ({error})'
    return (f"box at ({t['box'][0]:.2f}, {t['box'][1]:.2f}, {t['box'][2]:.2f}), {t['box_to_goal_m']:.2f} m from the spot, "
            f"tilt {t['box_tilt_deg']:.0f} deg; robots {'standing' if t['robots_standing'] else 'NOT both standing'}: "
            f"{'SUCCESS' if t['success'] else 'not a success'}")


def truth_words(ctx, layout):
    """Where the box was, in words for the agent's reply [sim-only]: S28 recorded two boxes leaning 30-46 deg across
    the table edge as 'landed upright on the floor', judged from a robot camera."""
    try:
        t = truth(ctx, layout)
    except Exception:
        return ''
    if layout.get('task') == 'can_bin':
        return can_task.words(t)
    z, tilt, table = t['box'][2], t['box_tilt_deg'], layout['box_start'].get('z', 0.0)
    if tilt >= 60:
        where = f'on its side or upside down (tilt {tilt:.0f} deg)'
    elif tilt >= 10:
        where = f'tipped {tilt:.0f} deg, its base {z:.2f} m high (leaning on something)'
    elif z < 0.03:
        where = 'upright on the floor'
    elif abs(z - table) < 0.03:
        where = 'upright on the table top'
    else:
        where = f'upright, its base {z:.2f} m high (held?)'
    return (f" [sim-only] Ground truth before this reset (label your attempt with it, not with what a camera "
            f"suggested): the box was {where}, at ({t['box'][0]:.2f}, {t['box'][1]:.2f}), {t['box_from_start_m']:.2f} m "
            f"from its start and {t['box_to_goal_m']:.2f} m from the goal spot; robots "
            f"{'both standing' if t['robots_standing'] else 'NOT both standing'}.")


def record(session, ctx, layout, event, **fields):
    """One ground-truth row in <session>/stream/ground_truth.jsonl: the agent reads it (it cannot write there) to label
    its attempts with what really happened, and the export carries it to later sessions. [sim-only]"""
    row = {'t': round(time.time(), 1), 'time': time.strftime('%H:%M:%S'), 'event': event, **fields}
    try:
        row.update(truth(ctx, layout))
    except Exception as error:
        row['error'] = str(error)
    snap = SIM / LOGS / 'snap' / 'overview.png'      # the room's third-person camera, newest frame (sim_camera_server)
    if event in ('before_reset', 'g1_stack_stopped', 'operator_end') and snap.exists():
        replay = session / 'stream' / 'replay'
        try:
            replay.mkdir(parents=True, exist_ok=True)
            name = f"{time.strftime('%H%M%S')}_{event}{'_' + str(fields['reset']) if 'reset' in fields else ''}.png"
            shutil.copyfile(snap, replay / name)
            row['third_person'] = f'runtime/stream/replay/{name}'
        except OSError:
            pass
    # --reply human: the agent's record (stream/) carries what a person beside the robots could report; the full row
    # goes to operator/, which the agent cannot read
    writes = [(truth_path(session), row)]
    if REPLY == 'human':
        writes.append((session / 'stream' / 'ground_truth.jsonl', human_view.observed(row, fields.get('closer_than_before'))))
    for path, content in writes:
        try:
            path.parent.mkdir(exist_ok=True)
            with path.open('a') as out:
                out.write(json.dumps(content) + '\n')
        except OSError as error:
            log(f'ground truth not written: {error}')
    return row


REPLY = 'human'


def truth_path(session):
    return session / ('operator' if REPLY == 'human' else 'stream') / 'ground_truth.jsonl'


def write_attempt_review(session, number, since, until, samples, layout, frame):
    """stream/reviews/attempt_<n>.md for the agent (runtime/stream/reviews/ in its workspace); its path, or None."""
    try:
        rows = [json.loads(line) for line in truth_path(session).read_text().splitlines() if line]
    except (OSError, ValueError):
        rows = []
    actions = [f"{r['time']} {r['event'].replace('_', ' ')}" + (f" ({r['restart']})" if r.get('restart') else '')
               for r in rows if since <= r.get('t', 0) < until and r.get('event') not in ('before_reset', 'after_reset')]
    try:
        attempt_review.write_review(session / ('operator' if REPLY == 'human' else 'stream') / 'reviews' / f'attempt_{number}.md',
                                    number, since, until, samples, layout, session / 'mcp.jsonl', actions, frame)
        return None if REPLY == 'human' else f'runtime/stream/reviews/attempt_{number}.md'
    except Exception as error:
        log(f'attempt review not written: {error}')
        return None


def qpos_now(ctx):
    import numpy as np
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect(f'tcp://127.0.0.1:{P_QPOS}')
    try:
        return np.frombuffer(sub.recv(), dtype=np.float64)
    finally:
        sub.close()


def deploy_alive():
    """The G1 deploy stack runs in the container (team_sim.sh): after a fall the backend tells it to stop and it exits."""
    try:
        return subprocess.run(['docker', 'exec', CONTAINER, 'pgrep', '-f', f'^/.*/g1_deploy_onnx_ref .*--zmq-port {P_POSE} '],
                              capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return True     # cannot tell: do nothing


def restart_g1(keys, layout):
    """What a person does after a G1 fall: start its stack again and hang the robot back on its gantry at its start.
    The lockstep simulation is paused while no stack runs, so without this a fall ends the session. Returns whether
    the new stack came up."""
    log = SIM / LOGS / 'deploy.log'
    began = time.time()
    subprocess.run(['docker', 'exec', '-e', f'TEAM_INSTANCE={INSTANCE}', '-e', f"TEAM_SCENE_DIR={os.environ.get('TEAM_SCENE_DIR', '')}",
                    '-e', f"TEAM_GPU={os.environ.get('TEAM_GPU', '')}", CONTAINER, 'bash', str(SIM / 'team_sim.sh'), 'restart-deploy'],
                   capture_output=True, timeout=60)
    # the G1 on its gantry first: that also lets the paused lockstep physics run again, and the new stack waits for
    # the robot state the running physics publishes before it can come up
    g1 = layout['robots']['g1']
    keys.send_string(f"g1_home={g1['pos'][0]},{g1['pos'][1]}"); time.sleep(1.0)
    end = time.time() + 120
    while time.time() < end:
        time.sleep(2.0)
        fresh = log.exists() and log.stat().st_mtime > began     # only the new stack's log counts
        text = log.read_text(errors='replace') if fresh else ''
        if 'Init Done' in text:
            break
        if 'CUDA initialization failure' in text or 'Program exiting' in text:
            return False
    else:
        return False
    for key in ('band=-0.10', 'band=-0.20'):
        keys.send_string(key); time.sleep(1.0)
    return True


def place_box(ctx, keys, layout, attempts=2):
    """Put the low table and the box back at their start and check where the box came to rest (ground truth)."""
    import math
    if layout.get('task') == 'can_bin':
        return can_task.place(keys, layout, qpos_now, ctx, attempts)
    (bx, by), yaw = layout['box_start']['pos'], layout['box_start']['yaw_deg']
    bz = layout['box_start'].get('z', 0.0) + 0.01      # on the floor, or on the low table it starts on
    h = math.radians(yaw) / 2
    off = tilt = None
    for _ in range(attempts):
        if layout['box_start'].get('z', 0.0) > 0:      # the low table first, then the box on it
            keys.send_string(f'place=low_table,{bx},{by},0.0,{math.cos(h)},0,0,{math.sin(h)}'); time.sleep(1.0)
        keys.send_string(f'place=box,{bx},{by},{bz},{math.cos(h)},0,0,{math.sin(h)}'); time.sleep(2.0)
        try:
            q = qpos_now(ctx)
        except zmq.Again:
            return None
        w, x, y, z = q[53:57]
        off = math.hypot(q[50] - bx, q[51] - by)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
        turned = abs(math.degrees(2 * math.atan2(z, w)) - yaw)
        if off < 0.02 and tilt < 3 and min(turned, 360 - turned) < 3:
            return None
    return (f' Note: after placing it twice the box rests {off * 100:.0f} cm from its start, tilted {tilt:.0f} deg '
            '(something touches it there: an arm or a robot).')


R1_AT = re.compile(r'r1_at\s*=\s*\(?\s*(-?\d*\.?\d+)\s*,\s*(-?\d*\.?\d+)\s*,\s*(-?\d*\.?\d+)')


G1_AT = re.compile(r'g1_at\s*=\s*\(?\s*(-?\d*\.?\d+)\s*,\s*(-?\d*\.?\d+)\s*,\s*(-?\d*\.?\d+)')


ROBOTS_STAY = re.compile(r'robots?\s*=\s*stay|(leave|keep)\b.{0,40}\brobots?\b.{0,40}(where|in place|as they are|there)', re.I | re.S)


def place_refusal(x, y, layout, other=None):
    """Why a person could not stand a robot at (x, y), or '': the place must be on the open floor (0.25 m clear of every
    piece of furniture and wall in the layout), 0.33 m from where the task's object starts, and 0.6 m from the other
    robot's place. Anywhere else in the room is allowed (two sessions had every 'at the object' practice stance refused
    by a rule tying stances to the start places, and walked across the room for each try)."""
    import math
    clear = 0.25
    if not (-1.5 <= x <= layout.get('back_wall_x', 1e9) - clear and layout.get('curtain_y', -1e9) + clear <= y <= layout.get('left_wall_y', 1e9) - clear):
        return 'it is outside the room or against a wall'
    things = {k: v for k, v in layout.items() if isinstance(v, dict) and 'pos' in v and k not in ('box_start', 'can_start', 'bin_start', 'tv', 'goal')}
    for name, thing in things.items():
        px, py = thing['pos'][:2]
        if 'size' in thing:
            hx, hy = thing['size'][0] / 2, thing['size'][1] / 2
            if thing.get('yaw_deg'):
                hx = hy = math.hypot(hx, hy)
        else:
            hx = hy = float(thing.get('base_r', 0.3))
        if abs(x - px) < hx + clear and abs(y - py) < hy + clear:
            return f'it is in or within {clear:g} m of the {name.replace("_", " ")}'
    bx, by = layout['box_start']['pos'][:2]
    if math.hypot(x - bx, y - by) < 0.33:
        return 'it is within 0.33 m of where the object starts'
    if other is not None and math.hypot(x - other[0], y - other[1]) < 0.6:
        return 'it is within 0.6 m of the other robot\'s place'
    return ''


def _pose_asked(pattern, name, other_name, message, layout):
    m = pattern.search(message)
    if not m:
        return None, ''
    x, y, yaw = (float(v) for v in m.groups())
    om = (R1_AT if pattern is G1_AT else G1_AT).search(message)
    other = [float(v) for v in om.groups()][:2] if om else layout['robots'][other_name]['pos'][:2]
    why = place_refusal(x, y, layout, other)
    if why:
        return None, (f' I did not put the {name.upper()} at {name}_at={x:g},{y:g},{yaw:g}: {why}. This reset moved NO robot: both are '
                      'where they were.')
    return (x, y, yaw), ''


def g1_pose_asked(message, layout):
    """`g1_at=x,y,yaw_deg` in a reset request: a person sets the G1 there, or (None, reason) if nobody could."""
    return _pose_asked(G1_AT, 'g1', 'r1', message, layout)


def r1_pose_asked(message, layout):
    """`r1_at=x,y,yaw_deg` in a reset request (the team frame, which is the room's here), likewise."""
    return _pose_asked(R1_AT, 'r1', 'g1', message, layout)


def box_aside(layout):
    """The place key that sets the box out of the robots' way: where the scene parks it if it has no part in the task
    (out of sight: a can-task session took the box parked behind the curtain for the bin), else beside the room."""
    x, y, z = (layout.get('parked') or {}).get('box') or (-1.6, -3.2, 0.15)
    return f'place=box,{x},{y},{max(z, 0.15) if "parked" not in layout else z},1,0,0,0'


def tripped_since(session, since):
    """The reason of a disarm by the robot service (a runaway or fall guard) written at or after `since`, else None."""
    try:
        value = json.loads(arming.arm_file(session).read_text())
    except (OSError, ValueError):
        return None
    if value.get('armed') is not True and (value.get('since') or 0) >= since and value.get('by') != 'sim operator':
        return value.get('reason') or 'disarmed by the robot service'
    return None


def stand_r1_up(ctx, keys, layout):
    """What a person does when the R1 lies on the floor: lift it, stand it at its start, let its controller start again
    from there. Returns True if it stands afterwards (S40: the R1 fell during a reset and nobody stood it up; the
    session sat disarmed for 36 min with 80 min of robot time left)."""
    import math
    r1 = layout['robots']['r1']
    h = math.radians(r1['yaw_deg']) / 2
    for _ in range(2):
        keys.send_string(f"place=r1_pelvis,{r1['pos'][0]},{r1['pos'][1]},0.80,{math.cos(h)},0,0,{math.sin(h)}"); time.sleep(0.4)
        try:
            for _ in range(2):
                sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000)
                sock.connect(f'tcp://127.0.0.1:{P_R1}')
                sock.send(json.dumps({'op': 'reset'}).encode()); sock.recv(); sock.close()
                time.sleep(1.5)
            sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000)
            sock.connect(f'tcp://127.0.0.1:{P_R1}')
            sock.send(json.dumps({'op': 'state'}).encode()); state = json.loads(sock.recv_multipart()[0]); sock.close()
        except zmq.ZMQError:
            return False
        if not state.get('fallen') and state.get('tilt_deg', 90) < 20:
            return True
    return False


def r1_is_down(ctx):
    try:
        sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000)
        sock.connect(f'tcp://127.0.0.1:{P_R1}')
        sock.send(json.dumps({'op': 'state'}).encode()); state = json.loads(sock.recv_multipart()[0]); sock.close()
        return bool(state.get('fallen')) or state.get('tilt_deg', 0) > 25
    except zmq.ZMQError:
        return False


def replace_robots(ctx, keys, layout, r1_pose=None, g1_pose=None):
    """Carry both standing robots back to where they started (a person would walk them there), each turned to its
    start heading (S29: the G1 kept its heading, which drifted ~2-3 deg over attempts, so it saw the box 2.5 cm off its
    axis and its prep refused); the R1's controller's reference frame restarts from there. With r1_pose / g1_pose
    (x, y, yaw_deg) a robot goes there instead (practice from a chosen stance)."""
    import math
    import numpy as np
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect(f'tcp://127.0.0.1:{P_QPOS}')
    qpos = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
    g1 = layout['robots']['g1'] if g1_pose is None else {'pos': g1_pose[:2], 'yaw_deg': g1_pose[2]}
    r1 = layout['robots']['r1'] if r1_pose is None else {'pos': r1_pose[:2], 'yaw_deg': r1_pose[2]}
    # The G1 is set down ONCE and its service told at once (it then takes the pose it has as its reference), and then
    # left alone until it stands quiet. Measured: one placement with the service told within 10 ms leaves it within
    # 0.14 m and 27 deg after a 90 deg turn, standing; every way of placing it more than once in a few seconds (set
    # down, tell 0.3 s later, set down again; held at the place for a second; turned in 3 deg steps) had it stepping
    # under a pelvis that was being snapped back, and it stumbled 0.4-0.9 m off or fell (2 of 10). What is left over
    # is corrected by the next single placement, which is small.
    session = getattr(replace_robots, 'session', None)
    flag = Path(session) / 'operator' / 'g1_moved' if session is not None else None

    def told():
        if flag is None:
            time.sleep(0.1)
            return False
        try:
            flag.write_text(str(time.time()))
            for _ in range(1000):                      # (the service stands the robot under the planner, then holds it afresh)
                if not flag.exists():
                    return True
                time.sleep(0.01)
            flag.unlink(missing_ok=True)
        except OSError:
            pass
        return False

    def quat_turned(q0, turn):                          # q_new = q_turn(z) * q: roll and pitch kept
        w, x, y, z = q0
        c, s = math.cos(turn / 2), math.sin(turn / 2)
        return (c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)

    w, x, y, z = qpos[3:7]
    turn = math.radians(g1.get('yaw_deg', 0.0)) - math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    q = quat_turned(qpos[3:7], turn)
    keys.send_string(f"place=pelvis,{g1['pos'][0]},{g1['pos'][1]},{qpos[2]},{q[0]},{q[1]},{q[2]},{q[3]}")
    replace_robots.g1_took_the_pose = told()
    time.sleep(0.5)
    h = math.radians(r1['yaw_deg']) / 2
    r1z = qpos[57 + 2]                                      # the R1's root follows the G1 (50) and the box (7) in qpos
    # the controller's reference goes to the asked pose first, then the robot: with the reference left at the old
    # heading the policy turned the robot back before the reset (turns of 60 deg or more ended 40-90 deg off)
    sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000); sock.connect(f'tcp://127.0.0.1:{P_R1}')
    sock.send(json.dumps({'op': 'reset', 'pose': [r1['pos'][0], r1['pos'][1], r1['yaw_deg']]}).encode()); sock.recv()
    keys.send_string(f"place=r1_pelvis,{r1['pos'][0]},{r1['pos'][1]},{r1z},{math.cos(h)},0,0,{math.sin(h)}")
    time.sleep(0.5)     # the R1's controller mirrors the world once a tick: its frame restarts from the pose it has seen
    sock.send(json.dumps({'op': 'reset'}).encode()); sock.recv(); sock.close()
    time.sleep(3.0)


def robot_poses(ctx):
    """{'g1' / 'r1': {'x', 'y', 'yaw_deg', 'tilt_deg'}} from the simulation, or None."""
    import math
    import numpy as np
    try:
        sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect(f'tcp://127.0.0.1:{P_QPOS}')
        q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
    except zmq.ZMQError:
        return None
    out = {}
    for name, at in (('g1', 0), ('r1', 57)):
        w, x, y, z = q[at + 3:at + 7]
        out[name] = {'x': round(float(q[at]), 3), 'y': round(float(q[at + 1]), 3),
                     'yaw_deg': round(math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), 1),
                     'tilt_deg': round(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))), 1)}
    return out


def placement_errors(poses, targets, pos_tol=0.05, yaw_tol=4.0):
    """What is off after a reset: [(robot, metres off, degrees off)] beyond the tolerances; targets {robot: (x, y, yaw)}."""
    import math
    off = []
    for name, (x, y, yaw) in targets.items():
        p = poses[name]
        d = math.hypot(p['x'] - x, p['y'] - y)
        a = abs((p['yaw_deg'] - yaw + 180) % 360 - 180)
        if d > pos_tol or a > yaw_tol or p['tilt_deg'] > 15:
            off.append((name, round(d, 3), round(a, 1)))
    return off


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--minutes', type=float, default=120)
    ap.add_argument('--max-resets', type=int, default=12)
    ap.add_argument('--allow-table-removal', action='store_true',
                    help='a person may take the low table away while the box is held clear of it (off by default: '
                         'getting round obstacles is the task)')
    ap.add_argument('--stall-attempts', type=int, default=8,
                    help='say so in the reset reply each time this many attempts in a row brought no task progress')
    ap.add_argument('--max-g1-restarts', type=int, default=6,
                    help='how often the operator starts the G1 stack again after it stopped (a fall)')
    ap.add_argument('--reply', choices=('human', 'full'), default='human',
                    help='human: replies and the record the agent reads carry only what a person beside the robots '
                         'could report (no measured coordinates, distances or contact flags); full: the simulation\'s '
                         'ground truth, as before')
    a = ap.parse_args()
    global REPLY
    REPLY = a.reply
    layout = json.loads((Path(os.environ.get('TEAM_SCENE_DIR') or SIM / 'scene') / 'layout.json').read_text())
    replace_robots.session = a.session
    ctx = zmq.Context.instance()
    keys = ctx.socket(zmq.PUB); keys.bind(f'tcp://127.0.0.1:{KEYS}'); time.sleep(0.6)
    arming.write(a.session, True, minutes=a.minutes, by='sim operator')
    log(f'armed for {a.minutes:g} min; lowering the G1 on the gantry')
    for key in ('band=-0.10', 'band=-0.20'):
        keys.send_string(key); time.sleep(1.0)
    state = ctx.socket(zmq.SUB); state.connect(f'tcp://127.0.0.1:{STATE}'); state.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
    released, control_at, resets, done = False, None, 0, set()
    attempt_since, samples = time.time(), []    # the ground truth of the attempt under way, for its review
    reset_times = []                            # when each reset was asked for: how long an attempt takes lately
    # the task's progress, kept by the harness from ground truth: the closest the object came to the goal while carried
    # (S41: 22 of 25 carries ended further from the goal and were recorded as 11/18 'ok' by the agent's own labels)
    progress = {'best_m': None, 'best_attempt': None, 'attempts': 0, 'since_best': 0}
    deadline = time.time() + a.minutes * 60
    restarts, next_check, notice = 0, time.time() + 30, ''
    while time.time() < deadline:
        if released and time.time() > next_check:
            next_check = time.time() + 10
            if not deploy_alive() and restarts < a.max_g1_restarts:
                restarts += 1
                log(f'the G1 deploy stack has stopped (a fall?): restarting it ({restarts}/{a.max_g1_restarts})')
                record(a.session, ctx, layout, 'g1_stack_stopped', restart=restarts)
                if restart_g1(keys, layout):
                    released, control_at = False, None
                    state.close()       # its queue holds the old stack's ticks: listen afresh for the new stack's
                    state = ctx.socket(zmq.SUB); state.connect(f'tcp://127.0.0.1:{STATE}'); state.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
                    arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                    notice = (f' The G1\'s stack had stopped at {time.strftime("%H:%M")} (after a fall or a stop): I started it '
                              'again and hung the G1 on its gantry at its starting place, in its standing pose; the session '
                              'is armed. Take control with g1_hold; I release the band 8 s after its stack is in control. '
                              'The box and the R1 were not moved.')
                    log('the G1 hangs on its gantry at its start; the new stack is up')
                    record(a.session, ctx, layout, 'g1_restarted', restart=restarts)
                else:
                    notice = (' The G1\'s stack stopped and did not start again (no GPU?): the G1 cannot move for the rest '
                              'of this session.')
                    log('the G1 deploy stack did not come up again')
        if not released:
            if control_at is None and state.poll(0):
                control_at = time.time(); log('the G1 stack is in control; releasing the band in 8 s')
            if control_at is not None and time.time() - control_at > 8.0:
                keys.send_string('release'); released = True; log('band released: the G1 stands on its own')
                time.sleep(3.0)     # the G1 sways while it is lowered and released and can brush the box: square it up
                misplaced = place_box(ctx, keys, layout)
                record(a.session, ctx, layout, 'g1_released', box_misplaced=misplaced)
        for request in operator.pending(a.session):
            if request['id'] in done:
                continue
            done.add(request['id'])
            state_first = ''
            log(f"request [{request['kind']}]: {request['message'][:300]}")
            if request['kind'] == 'reset' and resets < a.max_resets:
                log('before the reset: ' + end_state(ctx, layout))
                reset_started, before = time.time(), ('' if REPLY == 'human' else truth_words(ctx, layout))
                reset_times.append(reset_started)
                resets += 1
                summary = None
                try:
                    summary = attempt_review.attempt_summary(samples, layout, a.session / 'mcp.jsonl')
                except Exception as error:
                    log(f'attempt summary failed: {error}')
                # the task's rule, where the simulation reports hand contacts: off its support, the box has both hands of
                # each robot on it (2 s of slack for the sampling); an end state at the goal reached otherwise is not a success
                carried_by_both = (summary is None or summary.get('in_air_without_both_robots_s') is None
                                   or summary['in_air_without_both_robots_s'] <= 2.0)
                task_roles = None
                if layout.get('task') == 'can_bin':     # this task's rule: one robot on the bin, the other on the can
                    task_roles = can_task.roles(samples)
                    carried_by_both = task_roles['as_the_task_asks']
                    if REPLY != 'human':
                        before += can_task.role_words(task_roles)
                # (whether this attempt carried the object closer than any before: the same test the progress count uses)
                reached_now = (summary or {}).get('closest_to_goal_in_air_m')
                best_before = progress['best_m'] if progress['best_m'] is not None else (summary or {}).get('start_to_goal_m')
                closer = bool(reached_now is not None and best_before is not None and reached_now < best_before - 0.02)
                row = record(a.session, ctx, layout, 'before_reset', reset=resets, request=request['message'][:600],
                             attempt=summary, carried_by_both=carried_by_both, roles=task_roles,
                             closer_than_before=closer if summary else None)
                if REPLY == 'human':
                    before += human_view.words(human_view.observed(row, closer if summary else None))
                if row.get('success') and task_roles is not None:
                    before += (' SUCCESS: the can is in the bin, the bin stands, and it was done as the task asks.' if carried_by_both else
                               ' The can is in the bin, but not as the task asks (see above): not a success.')
                elif row.get('success') and not carried_by_both:
                    before += (' The box is at the goal, but it was off its support without both hands of each robot on it'
                               + ('' if REPLY == 'human' else f" for {summary['in_air_without_both_robots_s']:g} s") + ': not a success.')
                elif row.get('success'):
                    before += ' SUCCESS: the box stands at the goal and was carried by both robots.'
                review = write_attempt_review(a.session, resets, attempt_since, reset_started, samples, layout,
                                              row.get('third_person'))
                moved = ''
                r1_stood, targets, placement = '', None, []
                if r1_is_down(ctx):
                    keys.send_string(box_aside(layout)); time.sleep(0.5)     # out of the way first
                    if stand_r1_up(ctx, keys, layout):
                        arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                        r1_stood = (' The R1 was lying on the floor: I stood it up at its start (its arms as they were, '
                                    'its controller started again from there) and armed the session again.')
                        log('the R1 was down: stood up at its start, armed again')
                    else:
                        r1_stood = ' The R1 is lying on the floor and I could not stand it up.'
                        log('the R1 is down and did not stand up')
                r1_pose, refused = r1_pose_asked(request['message'], layout)
                g1_pose, g1_refused = g1_pose_asked(request['message'], layout)
                refused += g1_refused
                # the robots go back to their starts on every reset unless the request says to leave them (the word
                # 'robot' anywhere used to decide it: 'leave the robots where they are' carried both to the start, and
                # a request naming only 'the R1' moved nothing); a refused place moves nothing
                stay = bool(ROBOTS_STAY.search(request['message'])) and not (r1_pose or g1_pose)
                if not stay and not refused and standing(ctx):
                    # as a person would: pick the box up and set it aside, carry the robots, then put the box back.
                    # Robots carried while the box stays put land in it (S28: the G1 set down on the box at its feet,
                    # or with its arms out around it, stepped 0.43 m away, disarmed and fell); a box put back before
                    # the robots is knocked by their arms as they arrive
                    keys.send_string(box_aside(layout)); time.sleep(0.5)
                    replace_robots(ctx, keys, layout, r1_pose, g1_pose)
                    # a reset is done when the robots ARE where they were put, heading included: measure, and put them
                    # there again if not (S41: all 49 replies said 'at its start heading' while the R1 was 6-27 deg and
                    # the G1 10-28 deg off after turning runs; 20 of 52 lifts were refused for it)
                    targets = {'g1': (*(g1_pose[:2] if g1_pose else layout['robots']['g1']['pos'][:2]),
                                      g1_pose[2] if g1_pose else layout['robots']['g1'].get('yaw_deg', 0.0)),
                               'r1': (*(r1_pose[:2] if r1_pose else layout['robots']['r1']['pos'][:2]),
                                      r1_pose[2] if r1_pose else layout['robots']['r1'].get('yaw_deg', 0.0))}
                    for again in range(3):
                        poses = robot_poses(ctx)
                        placement = placement_errors(poses, targets) if poses else []
                        if not placement:
                            break
                        log(f'after placing the robots, off: {placement}; the G1 service took the pose: '
                            f"{getattr(replace_robots, 'g1_took_the_pose', None)}; placing them again ({again + 1})")
                        replace_robots(ctx, keys, layout, r1_pose, g1_pose)
                    where = lambda name, pose: (f'the {name} stands at ({pose[0]:g}, {pose[1]:g}), heading {pose[2]:g} deg, '
                                                'as you asked' if pose else f'the {name} is back at its start, at its start heading')
                    moved = (' T' + where('G1', g1_pose)[1:] + '; ' + where('R1', r1_pose) + ';'
                             + ' both standing as they stood (arms and legs as you left them). Their poses in the team frame '
                               'are stale: set them again.')
                misplaced = place_box(ctx, keys, layout)
                time.sleep(2.0)     # a guard tripped by the placement shows within a second or two
                if moved:
                    # say where the robots really ended up (S28: three replies said "back at their starting places"
                    # while the G1, set down into the box, had been thrown 0.6-3 m away)
                    try:
                        t = truth(ctx, layout)
                        aims = {'G1': ((g1_pose[:2] if g1_pose else layout['robots']['g1']['pos'][:2]), t['g1'], t['g1_tilt_deg']),
                                'R1': ((r1_pose[:2] if r1_pose else layout['robots']['r1']['pos'][:2]), t['r1'], t['r1_tilt_deg'])}
                        off = [f"the {name} is NOT where I put it" + ('' if REPLY == 'human' else f": it is at ({at[0]:.2f}, {at[1]:.2f}), "
                               f"{((at[0] - aim[0]) ** 2 + (at[1] - aim[1]) ** 2) ** 0.5:.2f} m away, tilt {tl:.0f} deg")
                               for name, (aim, at, tl) in aims.items()
                               if ((at[0] - aim[0]) ** 2 + (at[1] - aim[1]) ** 2) ** 0.5 > 0.05 or tl > 15]
                        if off:
                            moved += ' CHECK: ' + '; '.join(off) + ' (my placement went wrong; ask for another reset).'
                            log('robot placement off: ' + '; '.join(off))
                    except Exception:
                        pass
                tripped, caused = tripped_since(a.session, reset_started), ''
                if tripped is not None:
                    # the agent must not take the operator's doing for a result of its own commands (S28: three G1
                    # runaways during resets were blamed on the agent's last motion and became rules in its notes)
                    caused = (f' While I carried the robots, a robot\'s guard tripped ("{tripped[:160]}"): MY PLACEMENT '
                              'caused that, not your commands; do not draw conclusions about your motions from it.')
                    if standing(ctx) and deploy_alive():
                        arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                        caused += ' Both robots stand; I armed the session again.'
                    log('a guard tripped during the reset: ' + tripped[:160])
                if summary:
                    progress['attempts'] += 1
                    reached = summary.get('closest_to_goal_in_air_m')
                    start_goal = summary.get('start_to_goal_m')
                    if progress['best_m'] is None and isinstance(start_goal, (int, float)):
                        progress['best_m'] = round(float(start_goal), 2)      # the start: no progress yet
                    if reached is not None and progress['best_m'] is not None and reached < progress['best_m'] - 0.02:
                        progress.update(best_m=reached, best_attempt=resets, since_best=0)
                    elif reached is not None or progress['best_attempt'] is not None:
                        # (attempts before the object was ever carried do not count toward a stall: the measure has
                        # nothing to say yet; a from-scratch session was told 'stalled' at its 8th reset, before any lift)
                        progress['since_best'] += 1
                if summary and REPLY != 'human':   # S30 read 6 of 37 reviews and labelled runs from the end position alone
                    before += (' The attempt in numbers (label it with these): '
                               + attempt_review.summary_words(summary) + '.')
                text = (f'Reset {resets}/{a.max_resets}:' + before + ' Now the box is back at its starting place, upright.'
                        + (moved or ' The robots were not moved (ask for the robots too if you want them put back).')
                        + refused + (misplaced or '') + caused + r1_stood + ' Observe before acting.')
                # the state after the reset, measured, first in the reply and machine-readable (a wrapper that keeps only
                # part of a reply still gets it; S41: 5 'not where I put it' warnings, 1 reached the agent)
                poses = robot_poses(ctx)
                report = {'reset': resets, 'state_ok': None, 'robots': poses, 'warnings': [],
                         'task_progress': {'closest_to_goal_while_carried_m': progress['best_m'], 'in_attempt': progress['best_attempt'],
                                           'this_attempt_m': (summary or {}).get('closest_to_goal_in_air_m'),
                                           'attempts_since_it_improved': progress['since_best'], 'attempts': progress['attempts']}}
                try:
                    t_now = truth(ctx, layout)
                    report['box'] = {'x': t_now['box'][0], 'y': t_now['box'][1], 'base_z': t_now['box'][2], 'tilt_deg': t_now['box_tilt_deg'],
                                    'yaw_deg': t_now.get('box_yaw_deg')}
                    report['robots_standing'] = t_now['robots_standing']
                    if layout.get('task') == 'can_bin':
                        report['can'] = report.pop('box')
                        report['can']['in_bin'] = t_now['can_in_bin']
                        report['bin'] = {'x': t_now['bin'][0], 'y': t_now['bin'][1], 'tilt_deg': t_now['bin_tilt_deg']}
                except Exception:
                    pass
                if targets and poses:
                    placement = placement_errors(poses, targets)
                    report['robots_asked'] = {k: {'x': v[0], 'y': v[1], 'yaw_deg': v[2]} for k, v in targets.items()}
                    for robot, off_m, off_deg in placement:
                        report['warnings'].append(f'{robot} is not standing where it was to be put, after 3 tries' if REPLY == 'human' else
                                                  f'{robot} is {off_m} m and {off_deg} deg from where it was to be put, after 3 tries')
                if misplaced:
                    report['warnings'].append('the box did not come to rest at its start')
                if tripped is not None:
                    report['warnings'].append('a guard tripped during the reset (the operator caused it)')
                if 'could not stand it up' in r1_stood:
                    report['warnings'].append('the R1 is down')
                report['state_ok'] = bool(not report['warnings'] and report.get('robots_standing', True))
                said = report
                if REPLY == 'human':     # the places the robots were put (as on floor marks), not measured poses
                    said = {'reset': resets, 'state_ok': report['state_ok'], 'robots_standing': report.get('robots_standing'),
                            'warnings': report['warnings'], 'robots_put_at': report.get('robots_asked'),
                            'task_progress': {'attempts': progress['attempts'], 'attempts_since_it_improved': progress['since_best']}}
                    report = {**report, 'robots_put_at': report.get('robots_asked')}
                state_first = 'STATE ' + json.dumps(said, separators=(',', ':')) + ' '
                if progress['since_best'] and progress['since_best'] % a.stall_attempts == 0:
                    # a stall is a fact the harness can see; what to do about it is the agent's (strategy-review skill).
                    # First in the reply: mid-text in a 4 kB reply it was acted on once in four
                    state_first = (f"TASK PROGRESS STALLED: {progress['since_best']} attempts in a row have not brought the task "
                                    'closer to its goal than the best so far ('
                                    + ('' if REPLY == 'human' else f"{progress['best_m']} m from it, ")
                                    + (f"attempt {progress['best_attempt']}" if progress['best_attempt'] else 'the start') + '). '
                                    'The strategy-review skill asks for a fresh review now, from what was observed, before more of the same. '
                                   + state_first)
                log('after the reset: ' + end_state(ctx, layout))
                record(a.session, ctx, layout, 'after_reset', reset=resets, r1_at=list(r1_pose) if r1_pose else None,
                       guard_tripped_by_reset=tripped, state=report)
                if review:
                    text += f' The review of the attempt before this reset: `{review}`.'
                attempt_since, samples = time.time(), []
            elif request['kind'] == 'reset':
                text = 'No resets are left in this session.'
            elif (a.allow_table_removal and layout['box_start'].get('z', 0.0) > 0 and 'table' in request['message'].lower()
                  and any(w in request['message'].lower() for w in ('remove', 'take', 'away', 'pull'))):
                # a person beside the robots pulls the low table out from under the lifted box and carries it off
                where = end_state(ctx, layout)
                lifted = box_height(ctx) > layout['box_start']['z'] + 0.015
                if lifted:
                    keys.send_string('place=low_table,-1.6,-2.6,0.0,1,0,0,0'); time.sleep(1.0)
                    text = ('I pulled the low table out from under the box and carried it out of the room. It stays '
                            'away until the next reset, which puts it back with the box on it.')
                else:
                    text = ('The box is resting on the table: I take the table away only while the robots hold the box '
                            'clear of it (at least 1.5 cm above the top). Lift it, hold it, then ask again.')
                log(f'table request, box lifted: {lifted}; {where}')
            elif not arming.read(a.session, holds=False)['armed'] and standing(ctx):
                # a person beside the robots looks, sees both on their feet, and arms again
                arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                text = ('I looked: both robots are on their feet. The session is armed again. Check each robot\'s status '
                        'and observe before the next motion; say in your notes what set the fault off.')
            elif r1_is_down(ctx):
                # a person beside the robots stands a fallen R1 up again (S40: a help request about it got 'no help')
                if stand_r1_up(ctx, keys, layout):
                    arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                    text = ('The R1 was lying on the floor: I stood it up at its start (its arms as they were, its controller '
                            'started again from there). The session is armed again. Observe before the next motion; the box '
                            'is where it was (ask for a reset if you want it back).')
                else:
                    text = 'The R1 is lying on the floor and I could not stand it up: go on with what the G1 can do.'
                log('help: the R1 was down; ' + text[:60])
            else:
                text = ('No operator help is available in this simulation beyond putting the box back at its start '
                        '(request kind reset). Nobody can move a robot, hold the box, or move the furniture: getting round it is part of the task. Continue with '
                        'what the robots can do, and write down what they cannot.')
            text, notice = text + notice, ''
            recent = [b - c for c, b in zip(reset_times[-6:], reset_times[-5:]) if b - c > 45]  # S33: 'about 0 min'
            if len(recent) >= 2:
                try:
                    (a.session / 'operator' / 'pace.json').write_text(
                        json.dumps({'attempt_seconds': sorted(recent)[len(recent) // 2]}))
                except OSError:
                    pass
            clock = robot_clock(a.session)
            if clock:   # first, where a loop that logs only the head of a reply still shows it (S29-S32)
                head = (f"Robot time left: {clock['robot_minutes_left']:g} min: robot work goes on until "
                        f"{clock['robot_work_ends_at']}; the hand-over comes after that, on its own time.")
                if clock.get('last_attempt_start_that_fits') and clock['robot_minutes_left'] > 0:
                    head += (f" Your attempts lately take about {clock['attempt_minutes_lately']:g} min from reset to "
                             f"reset: one started before {clock['last_attempt_start_that_fits']} still fits (status gives "
                             "last_attempt_start_that_fits).")
                text = head + ' ' + text
            if layout.get('task') == 'can_bin':
                text = can_task.reword(text)
            text, state_first = state_first + text, ''
            operator.reply(a.session, request['id'], text, done=True, by='sim operator')
            log('replied:', text[:120])
        try:
            samples.append({'t': round(time.time(), 1), **truth(ctx, layout)})
        except Exception:
            pass
        time.sleep(1.0)
    log('session time is over')
    record(a.session, ctx, layout, 'operator_end', resets=resets, g1_restarts=restarts)


if __name__ == '__main__':
    # an operator that dies leaves every later request unanswered (S42: one exception in a reset, 40 min of robot time
    # lost waiting): start again after an error, a few times; the request it died on is answered by the new run
    import traceback
    for attempt in range(6):
        try:
            main()
            break
        except (KeyboardInterrupt, SystemExit):
            raise
        except Exception:
            log('operator error, starting again:\n' + traceback.format_exc()[-1500:])
            time.sleep(2.0)
