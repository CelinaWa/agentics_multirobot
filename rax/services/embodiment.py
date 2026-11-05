"""Experience across robots (operator side): what one robot learned, made usable by another.

A session can inherit the experience of a session on another robot (configure_session.py --experience; the robot
is read from the source's configuration or export stamp). The task layer transfers: where things are and how to see
them, what the goal looks like, what failed and why, the order of the steps, the checks. The body layer does not:
joint angles, hand shapes, tool names, gaits, reach numbers of the other body. prepare_cross() keeps the first in
place and sets the second aside, so the new robot starts from the other robot's task knowledge without acting on its
body numbers:

- memory/TIPS.md, handoff/README.md: each row is classified by the body vocabulary of the source robot (its tools,
  joints and end effector). Rows with no body term stay; rows with body terms and task terms stay with a
  `[body:<robot>]` mark (the mechanism may differ); rows with only body terms go to
  memory/history/<session>/BODY_<robot>.md.
- memory/PARAMS.json: entries classified the same way (name + note); body entries go aside
  (memory/history/<session>/PARAMS_<robot>.json), task/scene entries stay as priors to re-measure.
- code/: the source robot's code moves to code/from_<robot>/ (reference: perception modules often run unchanged, motion
  modules call the other robot's tools), with the source's skill library (from an export's bundle) next to it.
- memory/EMBODIMENT.md: what came over, what was set aside, how to use it.
"""
from __future__ import annotations

import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from services.sim_to_real import TAGGED, PARAMS, _rows, _is_row

# Words that tie a row to one body: its tools, joints, end effector and gait. Matched case-insensitively on word
# boundaries (a prefix ends with '_' or is a whole word).
BODY_TERMS = {
    'g1sonic': ('g1_', 'dex3', 'finger', 'fingers', 'fingertip', 'fingertips', 'thumb', 'palm', 'palms', 'fist',
                'index_', 'middle_', 'thumb_', 'hand_shape', 'pinch shape', 'pre-curled', 'semantic40', 'crouch',
                'lean', 'leaned', 'leaning', 'walk', 'walks', 'walked', 'walking', 'stride', 'planner', 'odometry',
                'shoulder', 'elbow', 'wrist', 'waist', 'knee', 'ankle', 'hip', 'sp ', 'el ', 'ik', 'fk',
                'controller3', 'g1_model', 'sonic'),
    'r1sonic': ('r1_', 'piper', 'gripper', 'grippers', 'finger', 'fingers', 'jaw', 'jaws', 'shoulder', 'elbow',
                'wrist', 'waist', 'teleop', 'r1_kin', 'ik', 'fk', 'sonic'),
}
TASK_TERMS = ('puppy', 'plush', 'dog', 'box', 'rim', 'wall', 'table', 'edge', 'depth', 'camera', 'image', 'colour',
              'color', 'mask', 'pixel', 'blob', 'layout', 'release', 'drop', 'place', 'inside', 'operator', 'reset',
              'observe', 're-observe', 'perceive', 'perception', 'success', 'verify', 'head', 'tail', 'legs', 'body')
REFERENCE = 'from_{robot}'


def source_robot(source):
    """The robot an agent-workspace, session root or export's workspace/ ran on, or None."""
    source = Path(source)
    for folder in (source, source.parent):
        for name in ('configuration.json', 'operator-input.json'):
            path = folder/name
            if path.is_file():
                try:
                    robot = json.loads(path.read_text()).get('robot')
                except (OSError, ValueError):
                    robot = None
                if robot:
                    return robot
        path = folder/'SOURCE.txt'
        if path.is_file():
            for line in path.read_text().splitlines():
                if line.startswith('robot:'):
                    return line.split(':', 1)[1].strip() or None
    return None


def _pattern(terms):
    parts = []
    for t in terms:
        e = re.escape(t.strip())
        parts.append(rf'\b{e}' if t.endswith(('_', ' ')) else rf'\b{e}\b')
    return re.compile('|'.join(parts), re.IGNORECASE)


def classify(text, robot):
    """'task', 'mixed' or 'body' for one row of the source robot's experience."""
    body = _pattern(BODY_TERMS.get(robot, (robot,))).findall(text)
    if not body:
        return 'task'
    return 'mixed' if _pattern(TASK_TERMS).search(text) else 'body'


def split_rows(text, robot):
    """(kept text with mixed rows marked, body rows set aside, counts)."""
    kept, aside, counts = [], [], {'task': 0, 'mixed': 0, 'body': 0}
    for row in _rows(text):
        if not _is_row(row):
            kept.append(row)
            continue
        kind = classify(row, robot)
        counts[kind] += 1
        if kind == 'body':
            aside.append(row)
        elif kind == 'mixed':
            stripped = row.lstrip()
            lead = row[:len(row) - len(stripped)]
            bullet = re.match(r'([-*]\s+|\|\s*)', stripped)
            cut = bullet.end() if bullet else 0
            kept.append(f'{lead}{stripped[:cut]}[body:{robot}] {stripped[cut:]}')
        else:
            kept.append(row)
    return ''.join(kept), aside, counts


def _library_files(bundle, destination):
    """Extract the main branch of a skill-library bundle (files only, no git) into destination."""
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(['git', 'clone', '-q', '--bare', str(bundle), f'{tmp}/lib.git'], check=True)
        archive = subprocess.run(['git', '--git-dir', f'{tmp}/lib.git', 'archive', 'main'], check=True,
                                 capture_output=True).stdout
        destination.mkdir(parents=True, exist_ok=True)
        subprocess.run(['tar', '-x', '-C', str(destination)], input=archive, check=True)


def prepare_cross(workspace, session, source, source_robot_name, target_robot):
    """Keep the task layer of experience inherited from another robot and set its body layer aside."""
    workspace = Path(workspace)
    robot = source_robot_name
    history = workspace/'memory'/'history'/session
    done = {'from_robot': robot, 'to_robot': target_robot, 'rows': {}, 'params_kept': [], 'params_aside': [],
            'code': None, 'library': None}
    aside_rows = []
    for name in TAGGED:
        path = workspace/name
        if path.is_file():
            kept, aside, counts = split_rows(path.read_text(), robot)
            path.write_text(kept)
            done['rows'][name] = counts
            if aside:
                aside_rows += [f'From {name}:\n'] + aside
    if aside_rows:
        history.mkdir(parents=True, exist_ok=True)
        (history/f'BODY_{robot}.md').write_text(
            f'# Rows about the {robot} body\n\nSet aside when a {target_robot} session inherited this experience: '
            f'they name the other robot\'s tools, joints, hand or gait. Read one only for the reason behind a '
            f'mechanism; its numbers do not apply to this robot.\n\n' + ''.join(aside_rows))
    params_path = workspace/PARAMS
    if params_path.is_file():
        params = json.loads(params_path.read_text())
        keep, aside = {}, {}
        for name, entry in params.items():
            text = f"{name.replace('.', ' ').replace('_', ' ')} {entry.get('note', '')}"
            (aside if classify(text, robot) != 'task' else keep)[name] = entry
        params_path.write_text(json.dumps(keep, indent=1) + '\n')
        if aside:
            history.mkdir(parents=True, exist_ok=True)
            (history/f'PARAMS_{robot}.json').write_text(json.dumps(aside, indent=1) + '\n')
        done['params_kept'], done['params_aside'] = sorted(keep), sorted(aside)
    code = workspace/'code'
    reference = code/REFERENCE.format(robot=robot)
    moved = []
    if code.is_dir():
        for path in sorted(code.iterdir()):
            if path.name.startswith('from_'):
                continue
            reference.mkdir(parents=True, exist_ok=True)
            shutil.move(str(path), reference/path.name)
            moved.append(path.name)
    done['code'] = moved
    source = Path(source)
    for folder in (source, source.parent):
        bundle = folder/f'skill-library-{robot}.bundle'
        if bundle.is_file():
            try:
                _library_files(bundle, reference/'skill-library')
                done['library'] = str(reference/'skill-library')
            except (OSError, subprocess.CalledProcessError) as exc:
                done['library'] = f'not extracted: {exc}'
            break
    perception = []
    if reference.is_dir():
        tools = _pattern([f'{robot.removesuffix("sonic")}_', 'ctx.value', 'ctx.call', 'controller'])
        for path in sorted(reference.rglob('*.py')):
            try:
                text = path.read_text(errors='replace')
            except OSError:
                continue
            if not tools.search(text):
                perception.append(str(path.relative_to(workspace)))
    done['robot_free_code'] = perception
    _write_note(workspace, session, done, target_robot)
    return done


def _write_note(workspace, session, done, target):
    robot = done['from_robot']
    rows = done['rows']
    lines = [f'# Experience from another robot: {robot} -> {target}', '',
             f'This session inherited the experience of `{session}`, which ran on **{robot}**. This session runs on '
             f'**{target}**: a different body, different tools, a different hand. The task is the same.', '',
             'What came over and how to use it ([inherited-experience](../.agents/skills/inherited-experience/SKILL.md)):', '',
             '- **Task layer, kept in place** (`memory/TIPS.md`, `handoff/README.md`): how the scene looks, how to find '
             'the objects and the box, what success looks like and how to check it, which steps the task needs and in '
             'what order, what failed and why. Use it as your plan until a measurement here contradicts it.',
             f'- **Rows marked `[body:{robot}]`**: a task lesson learned through the other body. The lesson may hold; the '
             'mechanism and every number in it (angles, heights, distances, hand shapes) belong to the other robot. '
             'Re-derive them for this body.',
             f'- **Body layer, set aside** (`memory/history/{session}/BODY_{robot}.md`'
             + (f', `PARAMS_{robot}.json`' if done['params_aside'] else '') + '): the other robot\'s joints, hand, gait '
             'and tool use. Do not act on it; open it only for the reason behind a mechanism.',
             f'- **Its code** (`code/from_{robot}/`): reference, not your controller. It calls the other robot\'s tools. '
             'Modules that call no robot tool (perception: finding the puppy, the box, the table from images and '
             'depth) may run here unchanged once their camera inputs match this robot\'s observations; read before '
             'reuse.', '']
    if done.get('library') and not str(done['library']).startswith('not'):
        lines += [f'- **Its skill library** (`code/from_{robot}/skill-library/`): the other robot\'s task skills, '
                  'files only. Your own library is this robot\'s and starts from what you merge into it.', '']
    lines += ['Counts:', '']
    for name, c in rows.items():
        lines.append(f'- `{name}`: {c["task"]} task rows kept, {c["mixed"]} kept marked `[body:{robot}]`, '
                     f'{c["body"]} body rows set aside')
    lines.append(f'- PARAMS: {len(done["params_kept"])} kept (priors to re-measure here), '
                 f'{len(done["params_aside"])} set aside')
    if done['robot_free_code']:
        lines += ['', f'Code files that call no robot tool ({len(done["robot_free_code"])}), candidates for reuse:', '']
        lines += [f'- `{p}`' for p in done['robot_free_code'][:40]]
    lines.append('')
    (workspace/'memory'/'EMBODIMENT.md').write_text('\n'.join(lines))
