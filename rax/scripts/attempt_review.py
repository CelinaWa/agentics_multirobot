"""A review of each attempt for the agent, written by the simulation's operator at every reset [sim-only].

The agent sees its own tool replies and its cameras, not the world: in S28 it labelled two boxes leaning across the
table edge as "landed upright on the floor" and blamed three faults the operator's resets had caused on its own last
command. Upstream RAX reviews every closed episode against ground truth for the same reason. This review needs no
model: the agent's calls from the tool log next to the ground truth the operator sampled during the attempt, with the
moments that matter (lift, highest point, furthest travel, tip, drop, robots off their feet) on one clock.
"""
import json
import math
import time
from pathlib import Path

QUIET = {'r1_status', 'g1_status', 'team_status', 'status', 'g1_observe', 'r1_observe', 'team_observe', 'r1_head',
         'g1_pixel_to_position', 'r1_pixel_to_position', 'team_wait'}   # perception and polling: counted, not listed
MAX_LINES = 120


def _clock(t):
    return time.strftime('%H:%M:%S', time.localtime(t))


def _short(value, limit=110):
    text = json.dumps(value, separators=(',', ':'), default=str)
    return text if len(text) <= limit else text[:limit] + '...'


def _arguments(arguments):
    arguments = dict(arguments or {})
    if isinstance(arguments.get('actions'), list):
        arguments['actions'] = f"<{len(arguments['actions'])} rows>"
    if isinstance(arguments.get('calls'), list):
        arguments['calls'] = [c.get('tool') for c in arguments['calls'] if isinstance(c, dict)]
    return arguments


def condensed_trace(mcp_path, since, until):
    """The agent's motion and operator calls in [since, until): one line each, with errors; polling counted."""
    try:
        events = [json.loads(line) for line in Path(mcp_path).read_text().splitlines() if line.strip()]
    except (OSError, ValueError):
        return ['(no tool log)']
    requests, lines, quiet = {}, [], 0
    for event in events:
        t = event.get('time', 0)
        if not since <= t < until:
            continue
        if event.get('event') == 'mcp_request':
            if event.get('tool') in QUIET:
                quiet += 1
                continue
            requests[event.get('call_id')] = event
        elif event.get('call_id') in requests:
            request = requests.pop(event['call_id'])
            took = t - request['time']
            if event.get('event') == 'mcp_error':
                outcome = f"ERROR {event.get('type')}: {str(event.get('reason'))[:120]}"
            else:
                outcome = _outcome(event.get('result'))
            lines.append(f"{_clock(request['time'])} +{took:4.1f}s {request.get('tool')} "
                         f"{_short(_arguments(request.get('arguments')))}" + (f" -> {outcome}" if outcome else ''))
    for request in requests.values():
        lines.append(f"{_clock(request['time'])} {request.get('tool')} {_short(_arguments(request.get('arguments')))}"
                     ' -> (no reply in this window)')
    if len(lines) > MAX_LINES:
        head = MAX_LINES // 4
        lines = lines[:head] + [f'... {len(lines) - MAX_LINES} calls omitted ...'] + lines[-(MAX_LINES - head):]
    return lines + [f'({quiet} status, observe and look calls not listed)']


def _outcome(result):
    """A tool reply's few words that matter for attribution: refusals, faults, tilt."""
    try:
        value = json.loads(result) if isinstance(result, str) else (result or {})
    except ValueError:
        return ''
    if not isinstance(value, dict):
        return ''
    bits = []
    for key in ('status', 'reason', 'fault', 'error'):
        if value.get(key) not in (None, '', 'ok'):
            bits.append(f'{key}={_short(value[key], 90)}')
    if 'tilt_max_deg' in value:
        bits.append(f"tilt_max {value['tilt_max_deg']}")
    return ' '.join(bits)


def truth_events(samples, layout):
    """The moments that matter in a list of ground-truth samples [{'t', 'box', 'box_tilt_deg', 'g1', 'g1_tilt_deg',
    'r1', 'r1_tilt_deg', 'robots_standing', ...}] (the operator's truth() rows), in time order."""
    if not samples:
        return ['(no ground truth sampled in this attempt)']
    table = layout['box_start'].get('z', 0.0)
    sx, sy = layout['box_start']['pos'][:2]
    lines, state = [], {}

    def note(t, text):
        lines.append(f'{_clock(t)} {text}')

    first = samples[0]
    note(first['t'], f"start: box at ({first['box'][0]:.2f}, {first['box'][1]:.2f}) base z {first['box'][2]:.2f}; "
                     f"G1 ({first['g1'][0]:.2f}, {first['g1'][1]:.2f}); R1 ({first['r1'][0]:.2f}, {first['r1'][1]:.2f})")
    top, far = first, first
    for s in samples:
        z, tilt = s['box'][2], s['box_tilt_deg']
        travel = math.hypot(s['box'][0] - sx, s['box'][1] - sy)
        lifted = tilt < 20 and (z > table + 0.015 or 0.02 < z < table - 0.01)   # past the edge, lowered: still held
        if lifted and not state.get('lifted'):
            note(s['t'], f'box lifted off its support (base z {z:.2f}, tilt {tilt:.0f})')
        if not lifted and state.get('lifted'):
            note(s['t'], f'box down again: base z {z:.2f}, tilt {tilt:.0f}, {travel:.2f} m from its start')
        state['lifted'] = lifted
        tipped = tilt >= 15
        if tipped and not state.get('tipped'):
            note(s['t'], f'box tipped {tilt:.0f} deg (base z {z:.2f})')
        if not tipped and state.get('tipped'):
            note(s['t'], f'box level again (tilt {tilt:.0f})')
        state['tipped'] = tipped
        off_table = z < 0.05           # its base on the floor (a box leaning on the table edge stays higher)
        if off_table and not state.get('off_table'):
            note(s['t'], f'box on the floor (base z {z:.2f}, tilt {tilt:.0f}), {travel:.2f} m from its start')
        state['off_table'] = off_table
        for robot in ('g1', 'r1'):
            down = s[f'{robot}_tilt_deg'] > 25
            if down and not state.get(robot):
                note(s['t'], f'{robot.upper()} tilted {s[robot + "_tilt_deg"]:.0f} deg (falling or fallen)')
            state[robot] = down
        if z > top['box'][2]:
            top = s
        if travel > math.hypot(far['box'][0] - sx, far['box'][1] - sy):
            far = s
    last = samples[-1]
    g1_moved = math.hypot(last['g1'][0] - first['g1'][0], last['g1'][1] - first['g1'][1])
    r1_moved = math.hypot(last['r1'][0] - first['r1'][0], last['r1'][1] - first['r1'][1])
    lines.append(f"highest: box base z {top['box'][2]:.2f} at {_clock(top['t'])} (table top {table:.2f})")
    lines.append(f"furthest: {math.hypot(far['box'][0] - sx, far['box'][1] - sy):.2f} m from the start at "
                 f"{_clock(far['t'])}, ({far['box'][0]:.2f}, {far['box'][1]:.2f}) base z {far['box'][2]:.2f} "
                 f"tilt {far['box_tilt_deg']:.0f}")
    lines.append(f"end: box at ({last['box'][0]:.2f}, {last['box'][1]:.2f}) base z {last['box'][2]:.2f} tilt "
                 f"{last['box_tilt_deg']:.0f}, {last.get('box_to_goal_m', float('nan')):.2f} m from the goal; G1 moved "
                 f"{g1_moved:.2f} m, R1 {r1_moved:.2f} m over the attempt")
    return lines


def last_motion_call(mcp_path, before, since=0.0):
    """(clock, tool) of the last motion or team call that started in [since, before), or None."""
    try:
        events = [json.loads(line) for line in Path(mcp_path).read_text().splitlines() if line.strip()]
    except (OSError, ValueError):
        return None
    calls = [e for e in events if e.get('event') == 'mcp_request' and e.get('tool') not in QUIET
             and since <= e.get('time', 0) < before]
    return (_clock(calls[-1]['time']), calls[-1].get('tool')) if calls else None


def attempt_summary(samples, layout, mcp_path=None):
    """The attempt in numbers, from the ground truth sampled during it [sim-only]: seconds the box was in the air,
    the furthest it travelled from its start while in the air, where it left the air (the release, or a drop) and
    how far it fell, where it ended (on the table, leaning on its edge, on the floor, tipped, still held), how much
    closer to the goal, and whether it was disturbed after resting level, with the call that began just before.
    S30 labelled its runs from the end position alone; S31 released at a fixed travel estimate without the travel
    at which a release cleared the table; both are answered here."""
    if not samples:
        return None
    table = layout['box_start'].get('z', 0.0)
    sx, sy = layout['box_start']['pos'][:2]
    travel = lambda s: math.hypot(s['box'][0] - sx, s['box'][1] - sy)
    # in the air: above the table top, or (past the table) between the floor and the table height, level enough not
    # to be leaning on an edge - S32's boxes lowered past the edge were counted as "left the air" at table height
    lifted = lambda s: s['box_tilt_deg'] < 20 and (s['box'][2] > table + 0.015 or 0.02 < s['box'][2] < table - 0.01)
    resting = lambda a, b: (b['t'] - a['t'] >= 1.5 and a['box_tilt_deg'] < 10 and b['box_tilt_deg'] < 10
                            and abs(a['box'][2] - b['box'][2]) < 0.005 and travel_gap(a, b) < 0.01
                            and (abs(b['box'][2] - table) < 0.006 or b['box'][2] < 0.02))
    travel_gap = lambda a, b: math.hypot(a['box'][0] - b['box'][0], a['box'][1] - b['box'][1])
    air, air_max, left_air, rest, disturbed, previous = 0.0, 0.0, None, None, None, None
    for i, s in enumerate(samples):
        if lifted(s):
            air_max = max(air_max, travel(s))
            if previous is not None and lifted(previous):
                air += s['t'] - previous['t']
            rest = None
        elif previous is not None and lifted(previous):
            later = [x for x in samples[i:] if x['t'] <= s['t'] + 2.5]
            low = min(later, key=lambda x: x['box'][2])
            left_air = {'at': _clock(previous['t']), 'from_start_m': round(travel(previous), 2),
                        'fell_m': round(max(0.0, previous['box'][2] - low['box'][2]), 2),
                        'tilt_after_deg': round(max(x['box_tilt_deg'] for x in later))}
            rest = None
        if rest is None and not lifted(s) and previous is not None and not lifted(previous):
            back = [x for x in samples[:i] if s['t'] - x['t'] >= 1.5 and not lifted(x)]
            if back and resting(back[-1], s):
                rest = s
        elif rest is not None and disturbed is None and (
                travel_gap(s, rest) > 0.03 or s['box_tilt_deg'] > rest['box_tilt_deg'] + 15):
            call = last_motion_call(mcp_path, s['t'], rest['t'] - 5) if mcp_path else None
            disturbed = {'at': _clock(s['t']), 'tilt_deg': round(s['box_tilt_deg']), 'moved_m': round(travel_gap(s, rest), 2),
                         'after_call': f'{call[1]} at {call[0]}' if call else None}
        previous = s
    first, last = samples[0], samples[-1]
    z, tilt = last['box'][2], last['box_tilt_deg']
    if lifted(last):
        end = 'in the air (held)'
    elif z < 0.05:
        end = 'on the floor, upright' if tilt < 10 else f'on the floor, tipped {tilt:.0f} deg'
    elif abs(z - table) < 0.02 and tilt < 10:
        end = 'on the table, level'
    else:
        end = f'leaning or tipped (base z {z:.2f}, tilt {tilt:.0f} deg), not level on the table or the floor'
    closer = (first.get('box_to_goal_m') - last.get('box_to_goal_m')
              if isinstance(first.get('box_to_goal_m'), (int, float)) and isinstance(last.get('box_to_goal_m'), (int, float)) else None)
    return {'in_air_s': round(air, 1), 'in_air_max_from_start_m': round(air_max, 2), 'left_the_air': left_air,
            'end_from_start_m': round(travel(last), 2), 'end': end, 'end_tilt_deg': round(tilt),
            'closer_to_goal_m': None if closer is None else round(closer, 2),
            'disturbed_after_set_down': disturbed}


def summary_words(summary):
    """One sentence of attempt_summary for the reset reply."""
    if not summary:
        return ''
    text = (f"in the air {summary['in_air_s']:g} s, furthest {summary['in_air_max_from_start_m']:.2f} m from its start "
            f"while in the air")
    left = summary.get('left_the_air')
    if left:
        text += (f"; it left the air (set down, released or dropped) at {left['at']}, {left['from_start_m']:.2f} m from its start, "
                 f"fell {left['fell_m']:.2f} m, tilt up to {left['tilt_after_deg']} deg within 2.5 s")
    text += f"; it ended {summary['end']}, {summary['end_from_start_m']:.2f} m from its start"
    if summary.get('closer_to_goal_m') is not None:
        c = summary['closer_to_goal_m']
        text += f" ({abs(c):.2f} m {'closer to' if c >= 0 else 'further from'} the goal than at the start)"
    d = summary.get('disturbed_after_set_down')
    if d:
        text += (f"; AFTER it rested level it moved {d['moved_m']:.2f} m / tilted to {d['tilt_deg']} deg at {d['at']}"
                 + (f", after your {d['after_call']}" if d.get('after_call') else ''))
    return text


def write_review(path, number, since, until, samples, layout, mcp_path, operator_lines=(), frame=None):
    """Write the review of the attempt between two resets (or the session start) to `path`."""
    text = [f'# Attempt {number}: {_clock(since)} to {_clock(until)} [sim-only]', '',
            'Written by the simulation operator from its ground truth (sampled about once a second) and your tool',
            'log. Use it to label the attempt and to check your causes: a fault or a drop belongs to whatever',
            'happened just before it on this clock, which is not always your last command.', '',
            f'**In numbers:** {summary_words(attempt_summary(samples, layout, mcp_path)) or "no samples"}.', '',
            '## Ground truth', ''] + [f'- {line}' for line in truth_events(samples, layout)]
    if operator_lines:
        text += ['', '## Operator actions', ''] + [f'- {line}' for line in operator_lines]
    text += ['', '## Your calls (motion, team, operator)', '', '```'] + condensed_trace(mcp_path, since, until) + ['```']
    if frame:
        text += ['', f'Third-person frame at the end: `{frame}`']
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text('\n'.join(text) + '\n')
    return path
