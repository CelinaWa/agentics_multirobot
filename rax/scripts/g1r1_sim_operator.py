"""The operator of a g1r1 session, played by a script when the robots are the office simulation (sim_team/): it does
what a person does beside the real robots. It arms the session, lowers the G1 on its gantry, and answers the agent's requests. A reset request puts the box back where it started (the robots stay where they are);
after a G1 fall it starts the G1's stack again and hangs the G1 back on its gantry at its start;
any other request gets a plain answer that no further help exists here. Run on the host beside the launcher:

  PYTHONPATH=rax $RAX_PYTHON rax/scripts/g1r1_sim_operator.py --session <session root> [--minutes 120]
"""
import argparse, json, re, shutil, subprocess, time
from pathlib import Path
import zmq
from services import arming
from services.controller import operator
from services.controller.frontend import robot_clock
import attempt_review    # (this script's own folder is on the path)

SIM = Path(__file__).resolve().parents[2] / 'sim_team'      # the office simulation (sim_team/team_sim.sh)
KEYS, STATE = 5790, 5797
CONTAINER = 'sonic-sim2sim'


def log(*parts):
    print(time.strftime('%H:%M:%S'), '[operator]', *parts, flush=True)


def standing(ctx):
    """Both robots on their feet, from the simulation itself: the R1's own stack says so, the G1's pelvis is up."""
    import numpy as np
    try:
        r1 = ctx.socket(zmq.REQ); r1.setsockopt(zmq.LINGER, 0); r1.setsockopt(zmq.RCVTIMEO, 3000); r1.connect('tcp://127.0.0.1:5791')
        r1.send(json.dumps({'op': 'state'}).encode()); state = json.loads(r1.recv_multipart()[0]); r1.close()
        sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
        qpos = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
        return (not state.get('fallen')) and state.get('tilt_deg', 90) < 25 and qpos[2] > 0.35
    except Exception:
        return False


def box_height(ctx):
    import numpy as np
    try:
        sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
        q = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
        return float(q[52])
    except Exception:
        return -1.0


def truth(ctx, layout):
    """Where the box and the robots are in the simulation itself, against the task's success condition."""
    import math
    import numpy as np
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    try:
        q = np.frombuffer(sub.recv(), dtype=np.float64)
    finally:
        sub.close()
    tilt = lambda w, x, y, z: math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    gx, gy = layout['goal']['pos'][:2]
    (bx, by) = layout['box_start']['pos'][:2]
    distance, box_tilt = math.hypot(q[50] - gx, q[51] - gy), tilt(*q[53:57])
    up = q[2] > 0.5 and tilt(*q[3:7]) < 25 and q[59] > 0.45 and tilt(*q[60:64]) < 25
    on_start = layout['box_start'].get('z', 0.0)
    w, x, y, z = q[53:57]
    return {'box': [round(float(v), 3) for v in q[50:53]], 'box_tilt_deg': round(box_tilt, 1),
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
    snap = SIM / 'logs' / 'snap' / 'overview.png'      # the room's third-person camera, newest frame (sim_camera_server)
    if event in ('before_reset', 'g1_stack_stopped', 'operator_end') and snap.exists():
        replay = session / 'stream' / 'replay'
        try:
            replay.mkdir(parents=True, exist_ok=True)
            name = f"{time.strftime('%H%M%S')}_{event}{'_' + str(fields['reset']) if 'reset' in fields else ''}.png"
            shutil.copyfile(snap, replay / name)
            row['third_person'] = f'runtime/stream/replay/{name}'
        except OSError:
            pass
    path = session / 'stream' / 'ground_truth.jsonl'
    try:
        path.parent.mkdir(exist_ok=True)
        with path.open('a') as out:
            out.write(json.dumps(row) + '\n')
    except OSError as error:
        log(f'ground truth not written: {error}')
    return row


def write_attempt_review(session, number, since, until, samples, layout, frame):
    """stream/reviews/attempt_<n>.md for the agent (runtime/stream/reviews/ in its workspace); its path, or None."""
    try:
        rows = [json.loads(line) for line in (session / 'stream' / 'ground_truth.jsonl').read_text().splitlines() if line]
    except (OSError, ValueError):
        rows = []
    actions = [f"{r['time']} {r['event'].replace('_', ' ')}" + (f" ({r['restart']})" if r.get('restart') else '')
               for r in rows if since <= r.get('t', 0) < until and r.get('event') not in ('before_reset', 'after_reset')]
    try:
        attempt_review.write_review(session / 'stream' / 'reviews' / f'attempt_{number}.md', number, since, until,
                                    samples, layout, session / 'mcp.jsonl', actions, frame)
        return f'runtime/stream/reviews/attempt_{number}.md'
    except Exception as error:
        log(f'attempt review not written: {error}')
        return None


def qpos_now(ctx):
    import numpy as np
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    try:
        return np.frombuffer(sub.recv(), dtype=np.float64)
    finally:
        sub.close()


def deploy_alive():
    """The G1 deploy stack runs in the container (team_sim.sh): after a fall the backend tells it to stop and it exits."""
    try:
        return subprocess.run(['docker', 'exec', CONTAINER, 'pgrep', '-f', '^/.*/g1_deploy_onnx_ref '],
                              capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return True     # cannot tell: do nothing


def restart_g1(keys, layout):
    """What a person does after a G1 fall: start its stack again and hang the robot back on its gantry at its start.
    The lockstep simulation is paused while no stack runs, so without this a fall ends the session. Returns whether
    the new stack came up."""
    log = SIM / 'logs' / 'deploy.log'
    began = time.time()
    subprocess.run(['docker', 'exec', CONTAINER, 'bash', str(SIM / 'team_sim.sh'), 'restart-deploy'],
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


def g1_pose_asked(message, layout):
    """`g1_at=x,y,yaw_deg` in a reset request: a person sets the G1 there. Refused (None, reason) unless within
    0.2 m and 20 deg of the G1's start and at least 0.33 m from the box centre."""
    import math
    m = G1_AT.search(message)
    if not m:
        return None, ''
    x, y, yaw = (float(v) for v in m.groups())
    (bx, by), (sx, sy) = layout['box_start']['pos'][:2], layout['robots']['g1']['pos'][:2]
    if (math.hypot(x - bx, y - by) < 0.33 or math.hypot(x - sx, y - sy) > 0.2
            or abs(yaw - layout['robots']['g1'].get('yaw_deg', 0.0)) > 20):
        return None, (f' I did not set the G1 at g1_at={x:g},{y:g},{yaw:g}: it must be within 0.2 m and 20 deg of its '
                      'start and at least 0.33 m from the box centre.')
    return (x, y, yaw), ''


def r1_pose_asked(message, layout):
    """`r1_at=x,y,yaw_deg` in a reset request (the team frame, which is the room's here): a person carries the R1 to that
    stance for practice. Refused (None, reason) unless it is clear of the box and the low table and near the start."""
    import math
    m = R1_AT.search(message)
    if not m:
        return None, ''
    x, y, yaw = (float(v) for v in m.groups())
    (bx, by), (sx, sy) = layout['box_start']['pos'][:2], layout['robots']['r1']['pos'][:2]
    if math.hypot(x - bx, y - by) < 0.33 or math.hypot(x - sx, y - sy) > 1.0:
        return None, (f' I did not carry the R1 to r1_at={x:g},{y:g},{yaw:g}: a stance must be at least 0.33 m from the '
                      'box centre and within 1.0 m of the R1\'s start.')
    return (x, y, yaw), ''


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
                sock.connect('tcp://127.0.0.1:5791')
                sock.send(json.dumps({'op': 'reset'}).encode()); sock.recv(); sock.close()
                time.sleep(1.5)
            sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000)
            sock.connect('tcp://127.0.0.1:5791')
            sock.send(json.dumps({'op': 'state'}).encode()); state = json.loads(sock.recv_multipart()[0]); sock.close()
        except zmq.ZMQError:
            return False
        if not state.get('fallen') and state.get('tilt_deg', 90) < 20:
            return True
    return False


def r1_is_down(ctx):
    try:
        sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000)
        sock.connect('tcp://127.0.0.1:5791')
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
    sub = ctx.socket(zmq.SUB); sub.setsockopt(zmq.SUBSCRIBE, b''); sub.setsockopt(zmq.RCVTIMEO, 3000); sub.connect('tcp://127.0.0.1:5792')
    qpos = np.frombuffer(sub.recv(), dtype=np.float64); sub.close()
    g1 = layout['robots']['g1'] if g1_pose is None else {'pos': g1_pose[:2], 'yaw_deg': g1_pose[2]}
    r1 = layout['robots']['r1'] if r1_pose is None else {'pos': r1_pose[:2], 'yaw_deg': r1_pose[2]}
    w, x, y, z = qpos[3:7]                               # keep the G1's roll and pitch, set its yaw
    turn = math.radians(g1.get('yaw_deg', 0.0)) - math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = math.cos(turn / 2), math.sin(turn / 2)       # q_new = q_turn(z) * q
    q = (c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)
    keys.send_string(f"place=pelvis,{g1['pos'][0]},{g1['pos'][1]},{qpos[2]},{q[0]},{q[1]},{q[2]},{q[3]}"); time.sleep(0.3)
    h = math.radians(r1['yaw_deg']) / 2
    r1z = qpos[57 + 2]                                      # the R1's root follows the G1 (50) and the box (7) in qpos
    keys.send_string(f"place=r1_pelvis,{r1['pos'][0]},{r1['pos'][1]},{r1z},{math.cos(h)},0,0,{math.sin(h)}"); time.sleep(0.1)
    sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 3000); sock.connect('tcp://127.0.0.1:5791')
    sock.send(json.dumps({'op': 'reset'}).encode()); sock.recv(); sock.close()
    time.sleep(3.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--minutes', type=float, default=120)
    ap.add_argument('--max-resets', type=int, default=12)
    ap.add_argument('--allow-table-removal', action='store_true',
                    help='a person may take the low table away while the box is held clear of it (off by default: '
                         'getting round obstacles is the task)')
    ap.add_argument('--max-g1-restarts', type=int, default=6,
                    help='how often the operator starts the G1 stack again after it stopped (a fall)')
    a = ap.parse_args()
    layout = json.loads((SIM / 'scene' / 'layout.json').read_text())
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
            log(f"request [{request['kind']}]: {request['message'][:300]}")
            if request['kind'] == 'reset' and resets < a.max_resets:
                log('before the reset: ' + end_state(ctx, layout))
                reset_started, before = time.time(), truth_words(ctx, layout)
                reset_times.append(reset_started)
                resets += 1
                summary = None
                try:
                    summary = attempt_review.attempt_summary(samples, layout, a.session / 'mcp.jsonl')
                except Exception as error:
                    log(f'attempt summary failed: {error}')
                row = record(a.session, ctx, layout, 'before_reset', reset=resets, request=request['message'][:600],
                             attempt=summary)
                review = write_attempt_review(a.session, resets, attempt_since, reset_started, samples, layout,
                                              row.get('third_person'))
                moved = ''
                r1_stood = ''
                if r1_is_down(ctx):
                    keys.send_string('place=box,-1.6,-3.2,0.15,1,0,0,0'); time.sleep(0.5)     # out of the way first
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
                if ('robot' in request['message'].lower() or r1_pose or g1_pose) and standing(ctx):
                    # as a person would: pick the box up and set it aside, carry the robots, then put the box back.
                    # Robots carried while the box stays put land in it (S28: the G1 set down on the box at its feet,
                    # or with its arms out around it, stepped 0.43 m away, disarmed and fell); a box put back before
                    # the robots is knocked by their arms as they arrive
                    keys.send_string('place=box,-1.6,-3.2,0.15,1,0,0,0'); time.sleep(0.5)
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
                        off = [f"the {name} is NOT where I put it: it is at ({at[0]:.2f}, {at[1]:.2f}), "
                               f"{((at[0] - aim[0]) ** 2 + (at[1] - aim[1]) ** 2) ** 0.5:.2f} m away, tilt {tl:.0f} deg"
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
                    caused = (f' While I carried the robots, the G1\'s guard tripped ("{tripped[:160]}"): MY PLACEMENT '
                              'caused that, not your commands; do not draw conclusions about your motions from it.')
                    if standing(ctx) and deploy_alive():
                        arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                        caused += ' Both robots stand; I armed the session again.'
                    log('a guard tripped during the reset: ' + tripped[:160])
                if summary:   # S30 read 6 of 37 reviews and labelled runs from the end position alone
                    before += (' The attempt in numbers (label it with these): '
                               + attempt_review.summary_words(summary) + '.')
                text = (f'Reset {resets}/{a.max_resets}:' + before + ' Now the box is back at its starting place, upright.'
                        + (moved or ' The robots were not moved (ask for the robots too if you want them put back).')
                        + refused + (misplaced or '') + caused + r1_stood + ' Observe before acting.')
                log('after the reset: ' + end_state(ctx, layout))
                record(a.session, ctx, layout, 'after_reset', reset=resets, r1_at=list(r1_pose) if r1_pose else None,
                       guard_tripped_by_reset=tripped)
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
    main()
