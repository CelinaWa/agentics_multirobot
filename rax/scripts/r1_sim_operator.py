"""Scripted operator for R1 sim sessions: answers the agent's request_operator calls within seconds (they expire in
a few minutes), the way a human operator at the robot would. It only does what the task allows the operator to do:
arm motion; reset the scene (puppy + box back, robot back at its start pose) and present the hind-paw overhang;
position the overhang on request; restore + re-arm after a fall (counts as a reset); other help -> a factual reply,
no scene change. Run beside a session:  r1_sim_operator.py --session <root> --port 5691 [--max-resets 5]"""
import argparse, glob, json, os, sys, time
import zmq
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from services import arming                       # noqa: E402
from services.controller import operator          # noqa: E402

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', required=True); ap.add_argument('--port', type=int, default=5691)
    ap.add_argument('--max-resets', type=int, default=5); ap.add_argument('--overhang-cm', type=float, default=6.0)
    ap.add_argument('--arm-minutes', type=int, default=150); ap.add_argument('--poll', type=float, default=3.0)
    a = ap.parse_args(); root = a.session; ctx = zmq.Context()

    def req(op, **f):
        s = ctx.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 30000); s.setsockopt(zmq.LINGER, 0)
        s.connect(f'tcp://127.0.0.1:{a.port}'); s.send(json.dumps({'op': op, **f}).encode())
        r = json.loads(s.recv_multipart()[0]); s.close(); return r
    def log(*p): print(time.strftime('%H:%M:%S'), *p, flush=True)
    def arm(): arming.write(root, True, minutes=a.arm_minutes, by='scripted operator (sim)')
    MATE = {0: 1, 1: 0, 2: 3, 3: 2}   # the two legs of one end of the puppy (lying on its side they stack ~5 cm apart)
    def place_overhang():
        g = req('gt'); edge_x = g['table']['pos'][0] - 0.300; front = min(p[0] for p in g['dog_paws'])
        dx = max(-0.25, min(0.0, (edge_x - a.overhang_cm / 100) - front)); d = g['dog']
        req('place', body='dog', pos=[d['pos'][0] + dx, d['pos'][1], d['pos'][2]], quat=d['quat']); time.sleep(1.5); return edge_x
    def separate_pair(min_gap=0.035):
        """Present a clean leg (task: 'legs clustered -> operator reposition to present a clean leg'): arrange the
        overhanging paw's pair-mate limb (swing+splay, held by the spring rest like a plush leg stays where it is
        put), then re-place the puppy for the overhang; judge after settling. Target: mate paw >= min_gap behind."""
        p = req('gt')['dog_paws']; front = min(range(4), key=lambda n: p[n][0]); mate = MATE[front]
        def arrange(sw, sp):
            req('dog_joint', name=f'dog_limb_{mate}_swing', value=sw); req('dog_joint', name=f'dog_limb_{mate}_splay', value=sp)
            time.sleep(1.5); place_overhang(); time.sleep(2.0)
            q = req('gt')['dog_paws']; return (q[mate][0] - q[front][0]), (min(range(4), key=lambda n: q[n][0]) == front)
        best = (0.0, 0.0, 0.0)
        for sw, sp in ((1.1, 1.1), (1.1, -1.1), (-1.1, 1.1), (-1.1, -1.1)):
            gap, ok = arrange(sw, sp)
            if ok and gap > best[0]: best = (gap, sw, sp)
        if best[0] >= min_gap: arrange(best[1], best[2])
        else: arrange(0.0, 0.0)
        log('pair separation: paw %d / mate %d gap %.1f cm (swing %+.1f splay %+.1f)' % (front, mate, best[0] * 100, best[1], best[2]))
    def overhang():
        edge_x = place_overhang(); separate_pair()
        g = req('gt'); return (edge_x - min(p[0] for p in g['dog_paws'])) * 100, g
    def scene(g):
        st = req('state')
        return ('robot %.2f m behind the front edge (tilt %.1f deg, %s), puppy on the table, box at its place, motion %s'
                % (g['table_edge_ahead_m'], st.get('tilt_deg', 0), 'FALLEN' if st.get('fallen') else 'standing',
                   'armed' if arming.read(root)['armed'] else 'NOT armed'))
    def reset():
        req('reset_scene'); req('reset'); time.sleep(3.5); return overhang()

    arm(); log('armed for', a.arm_minutes, 'min; max resets', a.max_resets)
    resets = 0; seen = set()
    while True:
        folder = os.path.join(root, 'operator', 'requests')
        for p in sorted(glob.glob(os.path.join(folder, '*.json')), key=os.path.getmtime):
            if p.endswith('.reply.json'): continue
            try: r = json.load(open(p))
            except Exception: continue
            rid = r.get('id')
            if not rid or rid in seen or r.get('status') != 'pending' or os.path.exists(os.path.join(folder, f'{rid}.reply.json')):
                continue
            kind = r.get('kind', ''); text = str(r.get('message') or r.get('text') or '').lower()
            try:
                if any(w in text for w in ('fall', 'fell', ' down', 'tilt', 'disarm')):
                    resets += 1; hang, g = reset(); arm()
                    reply = (f'Operator: the robot is restored and RE-ARMED (used reset {resets}/{a.max_resets}). {scene(g)}; '
                             f'the frontmost hind paw hangs {hang:.1f} cm past the edge. Re-check status and observe before moving.')
                elif kind == 'reset' or 'reset' in text:
                    if resets >= a.max_resets:
                        reply = (f'Operator: the reset budget ({a.max_resets}) is used up - no further resets; the scene stays as it is. '
                                 'Finish safely and write what you learned to memory/ (RUNS.md, TIPS.md, PARAMS.json).')
                    else:
                        resets += 1; hang, g = reset(); arm()
                        reply = (f'Operator: reset {resets}/{a.max_resets} done. {scene(g)}; the puppy\'s frontmost hind paw hangs '
                                 f'{hang:.1f} cm past the front edge in open space; grippers open.')
                elif any(w in text for w in ('overhang', 'hind leg', 'hind paw', 'over the edge', 'past the edge', 'position the puppy')):
                    hang, g = overhang()
                    reply = (f'Operator: done - the puppy\'s frontmost hind paw now hangs {hang:.1f} cm past the table\'s front edge '
                             f'in open space; nothing else moved. {scene(g)}.')
                elif 'arm' in text:
                    arm(); g = req('gt'); reply = f'Operator: motion is ARMED; nothing in the scene was moved. {scene(g)}.'
                else:
                    g = req('gt')
                    reply = (f'Operator: no scene change made (I only arm, reset, and position the hind-paw overhang). {scene(g)}. '
                             'Backward walking works but overshoots (~1.0-1.25x the command); re-observe after every walk.')
                operator.reply(root, rid, reply, done=True, by='scripted operator (sim)'); seen.add(rid)
                log(kind, rid, '->', reply[:120])
            except Exception as exc:  # noqa: BLE001  (keep serving)
                log('ERROR', kind, rid, repr(exc))
        time.sleep(a.poll)

if __name__ == '__main__':
    main()
