"""Every try of every stage, in one machine-readable log, and the funnel it adds up to across sessions.

A task is a graph of stages (`memory/STAGES.json`, yours to write and revise): each stage has the robots it needs,
the state it starts from, the check that says it succeeded, and the stages it comes after:

    {"stages": [
      {"id": "r1_grip", "robots": ["r1"], "after": ["r1_stance"],
       "start": "R1 at its stance, box on the table", "check": "pinch width 20-45 mm, box moved < 1 cm"},
      {"id": "joint_lift", "robots": ["g1", "r1"], "after": ["r1_grip", "g1_grip"],
       "start": "both holding", "check": "box 3+ cm above the table, level within 5 deg, 4 s"}]}

Record each try of a stage as it ends, whether it ran inside a whole attempt (`chain`), on its own from its start
state (`practice`), or as a one-robot stand-in for a joint stage (`proxy`):

    from api.attempts import record
    record('r1_grip', ok=False, mode='practice', cause='finger landed on the wall, box pushed 2 cm',
           params={'approach': [0.15, 0, -1], 'y': 0.12}, controller='code/s28/grip.py')

`python -m api.attempts` prints the funnel: tries and successes per stage, this session and all sessions, by mode,
with the most common causes of failure, and the operator's ground truth per session (resets, how far the box
really got, robot restarts) where the simulation provides it ([sim-only]: runtime/stream/ground_truth.jsonl for
this session, memory/ground_truth/ for earlier ones). `--json` gives the same as data.
"""
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[1]
LOG = WORKSPACE/'memory'/'attempts.jsonl'
STAGES = WORKSPACE/'memory'/'STAGES.json'
MODES = ('chain', 'practice', 'proxy')


def session_id():
    """This session's name: the session root holds the workspace."""
    return WORKSPACE.parent.name if WORKSPACE.name == 'agent-workspace' else WORKSPACE.name


def record(stage, ok, *, mode='chain', cause=None, params=None, controller=None, reset=None, note=None):
    """Append one try of `stage` (ok: did its check pass) to memory/attempts.jsonl and return the row."""
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}, not {mode!r}')
    row = {'t': round(time.time(), 1), 'session': session_id(), 'stage': str(stage), 'ok': bool(ok), 'mode': mode}
    for key, value in (('cause', cause), ('params', params), ('controller', controller), ('reset', reset),
                       ('note', note)):
        if value is not None:
            row[key] = value
    LOG.parent.mkdir(exist_ok=True)
    with LOG.open('a') as out:
        out.write(json.dumps(row, default=str) + '\n')
    return row


def rows(path=LOG):
    try:
        lines = path.read_text().splitlines()
    except FileNotFoundError:
        return []
    out = []
    for line in lines:
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return out


def stage_order():
    """Stage ids in the order STAGES.json lists them (stages only seen in the log come after)."""
    try:
        return [s['id'] for s in json.loads(STAGES.read_text()).get('stages', [])]
    except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError):
        return []


def funnel(log=None):
    log = rows() if log is None else log
    me = session_id()
    order = stage_order()
    order += sorted({r['stage'] for r in log} - set(order))
    table = []
    for stage in order:
        mine = [r for r in log if r['stage'] == stage]
        entry = {'stage': stage}
        for scope, subset in (('session', [r for r in mine if r.get('session') == me]), ('all', mine)):
            entry[scope] = {mode: [sum(r['ok'] for r in subset if r['mode'] == mode),
                                   sum(1 for r in subset if r['mode'] == mode)] for mode in MODES}
        causes = Counter(r['cause'] for r in mine if not r['ok'] and r.get('cause'))
        entry['causes'] = causes.most_common(4)
        entry['sessions'] = len({r.get('session') for r in mine})
        table.append(entry)
    return table


def truth_rows():
    """The operator's ground truth, by session: earlier sessions' from memory/ground_truth/, this one's live."""
    by_session = {}
    for path in sorted((WORKSPACE/'memory'/'ground_truth').glob('*.jsonl')):
        by_session[path.stem] = rows(path)
    live = WORKSPACE/'runtime'/'stream'/'ground_truth.jsonl'
    if live.exists():
        by_session[session_id()] = rows(live)
    return by_session


def truth_summary():
    out = []
    for session, events in truth_rows().items():
        before = [e for e in events if e.get('event') in ('before_reset', 'operator_end') and 'box' in e]
        out.append({
            'session': session,
            'resets': sum(1 for e in events if e.get('event') == 'before_reset'),
            'g1_restarts': sum(1 for e in events if e.get('event') == 'g1_stack_stopped'),
            'box_left_start_m_max': max((e['box_from_start_m'] for e in before), default=None),
            'box_to_goal_m_min': min((e['box_to_goal_m'] for e in before), default=None),
            'successes': sum(1 for e in before if e.get('success')),
        })
    return out


def _cell(pair):
    ok, n = pair
    return f'{ok}/{n}' if n else '-'


def main(argv):
    table, truth = funnel(), truth_summary()
    if '--json' in argv:
        print(json.dumps({'funnel': table, 'truth': truth}, indent=1))
        return
    print(f'Funnel ({LOG.relative_to(WORKSPACE)}; this session = {session_id()})')
    print(f'{"stage":24} {"chain":>9} {"practice":>9} {"proxy":>9} | {"all chain":>9} {"all prac":>9} {"all proxy":>9} sessions')
    for e in table:
        s, a = e['session'], e['all']
        print(f'{e["stage"][:24]:24} {_cell(s["chain"]):>9} {_cell(s["practice"]):>9} {_cell(s["proxy"]):>9} | '
              f'{_cell(a["chain"]):>9} {_cell(a["practice"]):>9} {_cell(a["proxy"]):>9} {e["sessions"]:>8}')
        for cause, count in e['causes']:
            print(f'{"":26}{count} x {cause[:100]}')
    if truth:
        print('\nGround truth by session [sim-only]: resets, G1 restarts, furthest the box got from its start, '
              'closest to the goal, successes')
        for t in truth:
            print(f'  {t["session"][:44]:44} resets {t["resets"]:>3}  restarts {t["g1_restarts"]}  '
                  f'box moved {t["box_left_start_m_max"]} m  to goal {t["box_to_goal_m_min"]} m  '
                  f'successes {t["successes"]}')


if __name__ == '__main__':
    main(sys.argv[1:])
