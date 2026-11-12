"""The operator of a g1r1 session, played by a script when the robots are the office simulation (sim_team/): it does
what a person does beside the real robots. It arms the session, lowers the G1 on its gantry, and answers the agent's requests. A reset request puts the box back where it started (the robots stay where they are);
any other request gets a plain answer that no further help exists here. Run on the host beside the launcher:

  PYTHONPATH=rax $RAX_PYTHON rax/scripts/g1r1_sim_operator.py --session <session root> [--minutes 120]
"""
import argparse, json, time
from pathlib import Path
import zmq
from services import arming
from services.controller import operator

SIM = Path(__file__).resolve().parents[2] / 'sim_team'      # the office simulation (sim_team/team_sim.sh)
KEYS, STATE = 5790, 5797


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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--minutes', type=float, default=120)
    ap.add_argument('--max-resets', type=int, default=12)
    a = ap.parse_args()
    layout = json.loads((SIM / 'scene' / 'layout.json').read_text())
    (bx, by), yaw = layout['box_start']['pos'], layout['box_start']['yaw_deg']
    bz = layout['box_start'].get('z', 0.0) + 0.01      # on the floor, or on the low table it starts on
    ctx = zmq.Context.instance()
    keys = ctx.socket(zmq.PUB); keys.bind(f'tcp://127.0.0.1:{KEYS}'); time.sleep(0.6)
    arming.write(a.session, True, minutes=a.minutes, by='sim operator')
    log(f'armed for {a.minutes:g} min; lowering the G1 on the gantry')
    for key in ('band=-0.10', 'band=-0.20'):
        keys.send_string(key); time.sleep(1.0)
    state = ctx.socket(zmq.SUB); state.connect(f'tcp://127.0.0.1:{STATE}'); state.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
    released, control_at, resets, done = False, None, 0, set()
    deadline = time.time() + a.minutes * 60
    while time.time() < deadline:
        if not released:
            if control_at is None and state.poll(0):
                control_at = time.time(); log('the G1 stack is in control; releasing the band in 8 s')
            if control_at is not None and time.time() - control_at > 8.0:
                keys.send_string('release'); released = True; log('band released: the G1 stands on its own')
        for request in operator.pending(a.session):
            if request['id'] in done:
                continue
            done.add(request['id'])
            log(f"request [{request['kind']}]: {request['message'][:300]}")
            if request['kind'] == 'reset' and resets < a.max_resets:
                import math
                h = math.radians(yaw) / 2
                keys.send_string(f'place=box,{bx},{by},{bz},{math.cos(h)},0,0,{math.sin(h)}'); time.sleep(2.0)
                resets += 1
                text = (f'Reset {resets}/{a.max_resets}: the box is back at its starting place, upright. '
                        'The robots were not moved: they stand where you left them. Observe before acting.')
            elif request['kind'] == 'reset':
                text = 'No resets are left in this session.'
            elif not arming.read(a.session, holds=False)['armed'] and standing(ctx):
                # a person beside the robots looks, sees both on their feet, and arms again
                arming.write(a.session, True, minutes=max(1.0, (deadline - time.time()) / 60), by='sim operator')
                text = ('I looked: both robots are on their feet. The session is armed again. Check each robot\'s status '
                        'and observe before the next motion; say in your notes what set the fault off.')
            else:
                text = ('No operator help is available in this simulation beyond putting the box back at its start '
                        '(request kind reset). Nobody can move a robot, hold the box, or change the scene. Continue with '
                        'what the robots can do, and write down what they cannot.')
            operator.reply(a.session, request['id'], text, done=True, by='sim operator')
            log('replied:', text[:120])
        time.sleep(1.0)
    log('session time is over')


if __name__ == '__main__':
    main()
