"""Ground truth and resets of the can-into-the-bin task for the simulation's operator [sim-only].

One robot takes the trash bin and brings it to the other; the other takes the drink can from the desk and puts it in
the bin. The operator's generic machinery (attempt numbers, progress, replies) follows "the object" = the can; this
module adds what is particular: where the bin is, whether the can is inside it, and which robot handled which.
"""
import json
import math
import time

CAN_HALF_HEIGHT = 0.078


def _tilt(w, x, y, z):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _yaw(w, x, y, z):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def truth(q, layout, hands):
    """The operator's truth row for this task; `hands` = {'can' / 'bin': {robot: {side: bool}}} or None."""
    a = layout['qpos']
    can, bin_ = q[a['can']:a['can'] + 7], q[a['bin']:a['bin'] + 7]
    can_tilt, bin_tilt = _tilt(*can[3:7]), _tilt(*bin_[3:7])
    sx, sy, sz = layout['bin_start']['size']
    c, s = math.cos(-_yaw(*bin_[3:7])), math.sin(-_yaw(*bin_[3:7]))
    dx, dy = can[0] - bin_[0], can[1] - bin_[1]
    lx, ly = c * dx - s * dy, s * dx + c * dy                       # the can in the bin's own frame
    inside = bool(abs(lx) < sx / 2 and abs(ly) < sy / 2 and bin_[2] - 0.02 < can[2] < bin_[2] + sz and bin_tilt < 30)
    (cx, cy), (bx, by) = layout['can_start']['pos'], layout['bin_start']['pos']
    up = q[2] > 0.5 and _tilt(*q[3:7]) < 25 and q[a['r1'] + 2] > 0.45 and _tilt(*q[a['r1'] + 3:a['r1'] + 7]) < 25
    row = {'box': [round(float(can[0]), 3), round(float(can[1]), 3), round(float(can[2] - CAN_HALF_HEIGHT * math.cos(math.radians(min(can_tilt, 90)))), 3)],
           'box_tilt_deg': round(can_tilt, 1), 'box_yaw_deg': round(math.degrees(_yaw(*can[3:7])), 1),
           'box_from_start_m': round(math.hypot(can[0] - cx, can[1] - cy), 3),
           'box_to_goal_m': round(math.hypot(dx, dy), 3),               # the can's distance to the bin, wherever the bin is
           'box_lifted': bool(can[2] - CAN_HALF_HEIGHT > layout['can_start']['z'] + 0.03),
           'bin': [round(float(v), 3) for v in bin_[:3]], 'bin_tilt_deg': round(bin_tilt, 1),
           'bin_from_start_m': round(math.hypot(bin_[0] - bx, bin_[1] - by), 3), 'can_in_bin': inside,
           'g1': [round(float(v), 3) for v in q[0:3]], 'g1_tilt_deg': round(_tilt(*q[3:7]), 1),
           'r1': [round(float(v), 3) for v in q[a['r1']:a['r1'] + 3]], 'r1_tilt_deg': round(_tilt(*q[a['r1'] + 3:a['r1'] + 7]), 1),
           'robots_standing': bool(up), 'success': bool(inside and bin_tilt < 20 and up)}
    if hands:
        row['hands_on'] = {k: hands[k] for k in ('can', 'bin') if k in hands}
    return row


def place(keys, layout, qpos_now, ctx, attempts=2):
    """The bin back at its start and the can back on the desk, upright; a note if either did not come to rest there."""
    (bx, by), (cx, cy), cz = layout['bin_start']['pos'], layout['can_start']['pos'], layout['can_start']['z']
    a, note = layout['qpos'], None
    for name, pos in layout.get('parked', {}).items():
        keys.send_string(f'place={name},{pos[0]},{pos[1]},{pos[2]},1,0,0,0'); time.sleep(0.2)
    for _ in range(attempts):
        keys.send_string(f'place=bin,{bx},{by},0.002,1,0,0,0'); time.sleep(0.8)
        keys.send_string(f'place=can,{cx},{cy},{cz + CAN_HALF_HEIGHT + 0.002},1,0,0,0'); time.sleep(1.5)
        q = qpos_now(ctx)
        can, bin_ = q[a['can']:a['can'] + 7], q[a['bin']:a['bin'] + 7]
        off_can, off_bin = math.hypot(can[0] - cx, can[1] - cy), math.hypot(bin_[0] - bx, bin_[1] - by)
        if off_can < 0.03 and _tilt(*can[3:7]) < 5 and off_bin < 0.06 and _tilt(*bin_[3:7]) < 5:
            return None
        note = (f' Note: after placing them twice the can rests {off_can * 100:.0f} cm from its start (tilt {_tilt(*can[3:7]):.0f} deg) '
                f'and the bin {off_bin * 100:.0f} cm from its start (tilt {_tilt(*bin_[3:7]):.0f} deg): something touches one of them.')
    return note


def words(t):
    """Where the can and the bin are, for the reply."""
    can, bin_ = t['box'], t['bin']
    where = ('inside the bin' if t['can_in_bin'] else
             f"on its side (tilt {t['box_tilt_deg']:.0f} deg), base {can[2]:.2f} m high" if t['box_tilt_deg'] >= 45 else
             f'upright, its base {can[2]:.2f} m above the floor')
    return (f" [sim-only] Ground truth before this reset (label your attempt with it, not with what a camera suggested): the can "
            f"was {where}, at ({can[0]:.2f}, {can[1]:.2f}), {t['box_from_start_m']:.2f} m from its start and {t['box_to_goal_m']:.2f} m "
            f"from the bin's centre; the bin was at ({bin_[0]:.2f}, {bin_[1]:.2f}), {t['bin_from_start_m']:.2f} m from its start, "
            f"tilted {t['bin_tilt_deg']:.0f} deg; robots {'both standing' if t['robots_standing'] else 'NOT both standing'}.")


def roles(samples):
    """Who handled what, and how the bin travelled, from the sampled ground truth: which robot's hands were on the bin and
    on the can while each moved; how far the bin moved lifted off the floor and how far while standing on it (pushed or
    dragged); and, at the end, how far the bin stands from the robot that handled the can."""
    movers = {'bin': set(), 'can': set()}
    carried = dragged = 0.0
    for before, now in zip(samples, samples[1:]):
        hands = now.get('hands_on') or {}
        moved = {'bin': math.dist(now['bin'][:3], before['bin'][:3]) > 0.01, 'can': math.dist(now['box'][:3], before['box'][:3]) > 0.01}
        for thing in ('bin', 'can'):
            for robot, sides in (hands.get(thing) or {}).items():
                if moved[thing] and any(sides.values()):
                    movers[thing].add(robot)
        step = math.dist(now['bin'][:2], before['bin'][:2])
        if step > 0.01:
            if min(now['bin'][2], before['bin'][2]) > 0.02:
                carried += step
            else:
                dragged += step
    one_each = len(movers['bin']) == 1 and len(movers['can']) == 1 and movers['bin'] != movers['can']
    beside = None
    if samples and len(movers['can']) == 1:
        last, robot = samples[-1], next(iter(movers['can']))
        beside = round(math.dist(last['bin'][:2], last[robot][:2]), 2)
    out = {'moved_the_bin': sorted(movers['bin']), 'moved_the_can': sorted(movers['can']), 'one_robot_each': one_each,
           'bin_carried_m': round(carried, 2), 'bin_moved_on_the_floor_m': round(dragged, 2),
           'bin_from_start_m': samples[-1].get('bin_from_start_m') if samples else None, 'bin_to_can_robot_m': beside}
    # the task: the bin is carried (not pushed or dragged) to the robot that handles the can
    out['bin_was_carried'] = bool(carried >= 0.5 and dragged <= 0.15)
    out['bin_brought_to_the_can_robot'] = bool(out['bin_was_carried'] and (out['bin_from_start_m'] or 0) >= 1.0
                                               and beside is not None and beside <= 0.9)
    out['as_the_task_asks'] = bool(one_each and out['bin_brought_to_the_can_robot'])
    return out


def role_words(r):
    name = lambda robots: ' and '.join(x.upper() for x in robots) if robots else 'no robot'
    text = (f" Hands on the bin while it moved: {name(r['moved_the_bin'])}; hands on the can while it moved: {name(r['moved_the_can'])}."
            f" The bin moved {r['bin_carried_m']:.2f} m lifted off the floor and {r['bin_moved_on_the_floor_m']:.2f} m standing on it"
            f" (pushed or dragged); it ended {r['bin_from_start_m']} m from its start"
            + (f" and {r['bin_to_can_robot_m']:.2f} m from the robot that handled the can." if r['bin_to_can_robot_m'] is not None else '.'))
    if not r['as_the_task_asks']:
        missing = [what for ok, what in ((r['one_robot_each'], 'one robot on the bin and the other on the can'),
                                         (r['bin_was_carried'], 'the bin carried, not pushed or dragged'),
                                         (r['bin_brought_to_the_can_robot'], 'the bin brought to the side of the robot with the can')) if not ok]
        text += ' Not as the task asks: ' + '; '.join(missing) + '.'
    return text


def reword(text):
    """The operator's generic sentences say 'box' for the object."""
    return (text.replace('Now the box is back at its starting place, upright.', 'Now the can is back on the desk, upright, and the bin at its start.')
                .replace('the box', 'the can').replace('The box', 'The can'))
