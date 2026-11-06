"""Scripted operator for R1 sim sessions: answers the agent's request_operator calls within seconds (they expire in
a few minutes), the way a human operator at the robot would. It only does what the task allows the operator to do:
arm motion; reset the scene (puppy + box back, robot back at its start pose) and present the hind-paw overhang;
position the overhang on request; restore + re-arm after a fall (counts as a reset); other help -> a factual reply,
no scene change. Run beside a session:  r1_sim_operator.py --session <root> --port 5691 [--max-resets 5]"""
import numpy as np
from scipy.spatial.transform import Rotation
import re
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
    ap.add_argument('--notice', default='', help='engineering notice appended to every reply (e.g. a simulator update)')
    ap.add_argument('--resets-used', type=int, default=0, help='resets already granted before this daemon started (restart)')
    ap.add_argument('--max-stances', type=int, default=20, help='sim-only "grasp stance" placements granted (0 disables)')
    ap.add_argument('--stances-used', type=int, default=0, help='stance placements already granted before this daemon started')
    ap.add_argument('--stance-jitter', type=float, default=0.0, help='random +-m offset of the grasp stance (x and y), so practice covers walk-in error')
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
        dx = max(-0.25, min(0.25, (edge_x - a.overhang_cm / 100) - front)); d = g['dog']   # either direction: a dragged puppy goes back too
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
    def box_beside(side):
        """Task layout variant: the box beside the puppy (left or right of it on the table, a few cm gap) instead of
        behind it. Sides are the ROBOT's left (+y) / right (-y)."""
        g = req('gt'); d = g['dog']['pos']; tx = g['table']['pos'][0]
        sign = 1.0 if side == 'left' else -1.0
        head = str(g.get('layout', {}).get('head', 'right'))   # the puppy's head side (robot's left/right): nose adds ~0.23 m
        extent = 0.13 + (0.23 if head == side else 0.07)     # torso half-length + nose or tail beyond it
        by = d[1] + sign * (extent + 0.04 + 0.195)            # ... + gap + box half-width (long side along y)
        bx = min(max(d[0], tx - 0.300 + 0.14 + 0.02), tx + 0.300 - 0.14 - 0.02)   # fully on the table
        req('place', body='box', pos=[bx, by, g['table_top_z'] + 0.0005], quat=[1, 0, 0, 0]); time.sleep(2.0)
        g = req('gt'); return g, (by - d[1]), head
    STANCE = (0.31, -0.21)   # the proven pivot/shaft-grasp stance: hanging hind paw here in the pelvis frame, heading square
    def stance():
        """Sim-only accelerator: a fresh scene with the robot already square at the grasp stance (the scene is slid under
        the robot, which is what a teleport of the robot amounts to), so grasp/lift/carry/place can be practised."""
        hang, g = reset()
        front = min(range(4), key=lambda n: g['dog_paws'][n][0])
        o = np.asarray(g['pelvis']['pos']); q = np.asarray(g['pelvis']['quat'])[[1, 2, 3, 0]]
        yaw = Rotation.from_quat(q).as_euler('zyx')[0]; R = Rotation.from_euler('z', yaw).as_matrix()
        paw = R.T @ (np.asarray(g['dog_paws'][front]) - o)
        jx, jy = (np.random.uniform(-a.stance_jitter, a.stance_jitter, 2) if a.stance_jitter > 0 else (0.0, 0.0))
        d = R @ np.array([STANCE[0] + jx - paw[0], STANCE[1] + jy - paw[1], 0.0])
        req('slide_table', dx=float(d[0]), dy=float(d[1])); time.sleep(3.0)
        g = req('gt'); paw = R.T @ (np.asarray(g['dog_paws'][front]) - np.asarray(g['pelvis']['pos']))
        return hang, g, paw

    arm(); log('armed for', a.arm_minutes, 'min; max resets', a.max_resets)
    resets = a.resets_used; stances = a.stances_used; seen = set()
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
                elif (any(w in text for w in ('inside the box', 'in the box', 'relative to the box', 'count as', 'success criterion', 'is the puppy',
                                              'question', 'do not move', "don't move", 'where is', 'where are', 'tell me'))
                      and not any(w in text for w in ('please reset', 'reset the scene', 'place me'))):
                    # QUESTIONS are answered from ground truth and never move anything
                    g = req('gt'); d = np.asarray(g['dog']['pos']); b = np.asarray(g['box']['pos']); rel = d - b
                    o = np.asarray(g['pelvis']['pos']); q = np.asarray(g['pelvis']['quat'])[[1, 2, 3, 0]]
                    Rm = Rotation.from_euler('z', Rotation.from_quat(q).as_euler('zyx')[0]).as_matrix()
                    paws = [Rm.T @ (np.asarray(p) - o) for p in g['dog_paws']]; front = min(range(4), key=lambda n: paws[n][0])
                    hip = Rm.T @ (np.asarray(g['dog_hips'][front]) - o); ax = hip - paws[front]
                    inside = abs(rel[0]) < 0.134 and abs(rel[1]) < 0.189 and 0.0 < rel[2] < 0.145
                    where = ('INSIDE the box (torso centre within the walls and below the rim)' if inside else
                             'NOT inside: ' + ('its torso centre is above the rim height (resting on or across the rim does not count)' if (abs(rel[0]) < 0.134 and abs(rel[1]) < 0.189) else 'its torso centre is outside the box footprint'))
                    edge_x = g['table']['pos'][0] - 0.300
                    # what a HUMAN operator could say by looking (no coordinates: the real world has no ground truth)
                    fwd = rel[0]; side = rel[1]
                    rough = ('in the box' if inside else
                             ('on top of the box rim' if (abs(fwd) < 0.134 and abs(side) < 0.189) else
                              f"on the table about {abs(fwd + 0.137)*100:.0f} cm {'in front of' if fwd < 0 else 'behind'} the box"
                              + (f", {abs(side)*100:.0f} cm to your {'left' if side > 0 else 'right'} of its centre" if abs(side) > 0.08 else '')))
                    hang = (edge_x - (o + Rm @ paws[front])[0]) * 100
                    paw_words = ('a hind paw hangs about %.0f cm past the front edge' % hang if hang > 1 else 'no paw hangs past the front edge')
                    reply = (f'Operator (looking, nothing moved): the puppy is {where}; it lies {rough}; {paw_words}. {scene(g)}.')
                    g = req('gt'); d = np.asarray(g['dog']['pos']); b = np.asarray(g['box']['pos']); rel = d - b
                    inside = abs(rel[0]) < 0.134 and abs(rel[1]) < 0.189 and 0.0 < rel[2] < 0.145
                    where = ('INSIDE the box (torso centre within the walls and below the rim)' if inside else
                             'NOT inside: ' + ('its torso centre is above the rim height (resting on or across the rim does not count)' if (abs(rel[0]) < 0.134 and abs(rel[1]) < 0.189) else 'its torso centre is outside the box footprint'))
                    reply = (f'Operator: the puppy is {where}. Torso centre relative to the box centre: {rel[0]*100:+.0f} cm along your forward axis, '
                             f'{rel[1]*100:+.0f} cm to your left, {rel[2]*100:+.0f} cm above the box floor (walls are 14 cm high, interior 27 x 38 cm). '
                             f'Nothing was moved. {scene(g)}.')
                elif kind == 'reset' or 'reset' in text:
                    if resets >= a.max_resets:
                        reply = (f'Operator: the reset budget ({a.max_resets}) is used up - no further resets; the scene stays as it is. '
                                 'Finish safely and write what you learned to memory/ (RUNS.md, TIPS.md, PARAMS.json).')
                    else:
                        resets += 1; hang, g = reset(); arm()
                        reply = (f'Operator: reset {resets}/{a.max_resets} done. {scene(g)}; the puppy\'s frontmost hind paw hangs '
                                 f'{hang:.1f} cm past the front edge in open space; grippers open.')
                elif (('stance' in text and a.max_stances > 0)
                      or re.search(r'(put|place|move|set|position|bring)\s+(the\s+)?box|box\s+(beside|next to|on my (left|right)|to (the|my) (left|right))', text)
                      or any(w in text for w in ('overhang', 'hind leg', 'hind paw', 'over the edge', 'past the edge', 'position the puppy'))):
                    # composable scene services: a fresh grasp stance, the box beside the puppy, a hind-leg overhang
                    want_stance = 'stance' in text and a.max_stances > 0
                    want_box = re.search(r'(put|place|move|set|position|bring)\s+(the\s+)?box|box\s+(beside|next to|on my (left|right)|to (the|my) (left|right))', text) is not None
                    want_overhang = any(w in text for w in ('overhang', 'hind leg', 'hind paw', 'over the edge', 'past the edge', 'position the puppy'))
                    parts = []
                    if want_stance:
                        if stances >= a.max_stances:
                            parts.append(f'the stance budget ({a.max_stances}) is used up (ask for a reset or walk in yourself)')
                        else:
                            stances += 1; hang, g, paw = stance(); arm()
                            parts.append(f'sim-only accelerator, stance {stances}/{a.max_stances} (not counted as a reset): fresh scene, robot '
                                         f'standing square to the table at the proven grasp stance - the hanging hind paw is at '
                                         f'({paw[0]:.2f}, {paw[1]:.2f}, {paw[2]:.2f}) in your pelvis frame, {hang:.1f} cm past the edge, front edge '
                                         f'{g["table_edge_ahead_m"]:.2f} m ahead; arms home, grippers open, armed')
                    if want_box:
                        side = 'left' if ('left' in text and 'right' not in text) else 'right'
                        g, dy, head = box_beside(side); arm()
                        parts.append(f'the box now stands on the table beside the puppy, on your {side} (its centre {abs(dy)*100:.0f} cm to the '
                                     f'{side} of the puppy, ~4 cm beyond its {"nose" if head == side else "tail"}, same depth); the puppy was not '
                                     f'moved; this is one of the task\'s layouts (box left / right / behind)')
                    if want_overhang and not want_stance:
                        hang, g = overhang(); arm()
                        parts.append(f'the puppy\'s frontmost hind paw now hangs {hang:.1f} cm past the table\'s front edge in open space; '
                                     f'nothing else was moved')
                    g = req('gt')
                    reply = 'Operator: ' + '; '.join(parts) + f'. {scene(g)}.'
                elif 'arm' in text:
                    arm(); g = req('gt'); reply = f'Operator: motion is ARMED; nothing in the scene was moved. {scene(g)}.'
                else:
                    g = req('gt')
                    reply = (f'Operator: no scene change made (I only arm, reset, and position the hind-paw overhang). {scene(g)}. '
                             'Backward walking works but overshoots (~1.0-1.25x the command); re-observe after every walk.')
                if a.notice: reply = reply + ' NOTICE FROM ENGINEERING: ' + a.notice
                operator.reply(root, rid, reply, done=True, by='scripted operator (sim)'); seen.add(rid)
                log(kind, rid, '->', reply[:120])
            except Exception as exc:  # noqa: BLE001  (keep serving)
                log('ERROR', kind, rid, repr(exc))
        time.sleep(a.poll)

if __name__ == '__main__':
    main()
