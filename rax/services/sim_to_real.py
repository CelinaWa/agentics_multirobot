"""One path for experience between simulation and the real robot (operator side).

Every session has an environment (Configuration.environment: 'real' or 'sim'). What a session
learns is written in one format whichever it ran in (see the sim-to-real agent skill):

- memory/TIPS.md and handoff/README.md rows carry a kind tag:
  [strategy]  what to do and why; transfers as a rule until contradicted
  [param]     a measured number; transfers as a prior to measure again (its value lives in PARAMS.json)
  [sim-only]  an artifact of the simulation (its operator, its contact model, its scene builder); dropped
              when the experience reaches the real robot
- memory/PARAMS.json holds each measured number per environment ({name: {unit, note, sim: {...}, real: {...}}}),
  read and written by the agent's code through api/params.py, so one controller runs in both.

On inheritance into a session of another environment, prepare() moves [sim-only] rows aside and writes
memory/TRANSFER.md: where the experience came from, which numbers are unverified here, what to measure
first. lint() reports the same for an export, before it leaves the sim host.
"""
from __future__ import annotations

from datetime import date
import json
from pathlib import Path
import re

TAGS = ('strategy', 'param', 'sim-only')
TAGGED = ('memory/TIPS.md', 'handoff/README.md')
PARAMS = 'memory/PARAMS.json'
TRANSFER = 'memory/TRANSFER.md'
NUMBER = re.compile(r'[-+]?\d+(?:\.\d+)?\s*(?:m|cm|mm|rad|deg|°|s|Hz|N|Nm|kg|%)(?![A-Za-z])')
TAG = re.compile(r'\[(strategy|param|sim-only)\]')


def source_environment(source):
    """'real', 'sim' or 'unknown' for an agent-workspace, a session root or an export's workspace/."""
    source = Path(source)
    for folder in (source, source.parent):
        for name in ('configuration.json', 'operator-input.json'):
            path = folder/name
            if path.is_file():
                try:
                    return json.loads(path.read_text()).get('environment', 'unknown')
                except (OSError, ValueError):
                    pass
        path = folder/'SOURCE.txt'
        if path.is_file():
            for line in path.read_text().splitlines():
                if line.startswith('environment:'):
                    return line.split(':', 1)[1].strip() or 'unknown'
    return 'unknown'


def _rows(text):
    """Split markdown into rows: a bullet or table line with its indented continuation lines."""
    rows, current = [], []
    for line in text.splitlines(keepends=True):
        starts = line.lstrip().startswith(('- ', '* ', '|')) and not line.startswith((' ', '\t'))
        if starts or not line.startswith((' ', '\t')) or not current:
            if current:
                rows.append(''.join(current))
            current = [line]
        else:
            current.append(line)
    if current:
        rows.append(''.join(current))
    return rows


def _is_row(row):
    head = row.lstrip()
    return head.startswith(('- ', '* ')) or (head.startswith('|') and not re.match(r'\|[\s:|-]+\|?\s*$', head))


def split_sim_only(text):
    """(text without [sim-only] rows, the removed rows)."""
    kept, removed = [], []
    for row in _rows(text):
        (removed if _is_row(row) and '[sim-only]' in row else kept).append(row)
    return ''.join(kept), removed


def load_params(workspace):
    path = Path(workspace)/PARAMS
    try:
        value = json.loads(path.read_text()) if path.is_file() else {}
    except ValueError:
        return {}
    return value if isinstance(value, dict) else {}


def unverified(params, environment):
    """Names measured elsewhere but not yet in this environment."""
    return sorted(name for name, entry in params.items()
                  if isinstance(entry, dict) and environment not in entry
                  and any(e in entry for e in ('sim', 'real')))


def lint(workspace):
    """Counts of tagged rows and the rows that state numbers without a tag; what an export carries."""
    workspace = Path(workspace)
    report = {'files': {}, 'untagged_numbers': []}
    for name in TAGGED:
        path = workspace/name
        if not path.is_file():
            continue
        counts = dict.fromkeys(TAGS, 0)
        counts['untagged'] = 0
        for row in filter(_is_row, _rows(path.read_text())):
            tags = TAG.findall(row)
            for tag in set(tags):
                counts[tag] += 1
            if not tags:
                counts['untagged'] += 1
                if NUMBER.search(row):
                    report['untagged_numbers'].append(f'{name}: {row.strip().splitlines()[0][:160]}')
        report['files'][name] = counts
    params = load_params(workspace)
    report['params'] = {env: sorted(n for n, e in params.items() if isinstance(e, dict) and env in e)
                        for env in ('sim', 'real')}
    return report


def format_lint(report):
    lines = []
    for name, counts in report['files'].items():
        lines.append(f'{name}: ' + ', '.join(f'{k} {v}' for k, v in counts.items()))
    lines.append(f"PARAMS.json: {len(report['params']['sim'])} measured in sim, "
                 f"{len(report['params']['real'])} on the real robot")
    if report['untagged_numbers']:
        lines.append(f"{len(report['untagged_numbers'])} untagged rows state numbers (tag them [param] or [sim-only]):")
        lines += [f'  {row}' for row in report['untagged_numbers'][:40]]
    return '\n'.join(lines)


def prepare(workspace, session, source, target):
    """Adapt freshly inherited experience to this session's environment; returns what was done."""
    workspace = Path(workspace)
    done = {'from': source, 'to': target, 'moved_sim_only': 0, 'unverified_params': []}
    old = workspace/TRANSFER
    if old.is_file():  # the earlier session's own transfer note is its history
        aside = workspace/'memory'/'history'/session/'TRANSFER.md'
        aside.parent.mkdir(parents=True, exist_ok=True)
        old.replace(aside)
    # unknown: a session from before the environment field; on the real robot it is read as simulation
    if source == target or (source == 'unknown' and target == 'sim'):
        return done
    moved = []
    if target == 'real':
        for name in TAGGED:
            path = workspace/name
            if path.is_file():
                kept, removed = split_sim_only(path.read_text())
                if removed:
                    path.write_text(kept)
                    moved += [f'From {name}:\n'] + removed
        if moved:
            aside = workspace/'memory'/'history'/session/'SIM_ONLY.md'
            aside.parent.mkdir(parents=True, exist_ok=True)
            aside.write_text('# Rows the earlier session marked [sim-only]\n\nArtifacts of the simulation; not '
                             'guidance for the real robot. Kept for reference only.\n\n' + ''.join(moved))
    done['moved_sim_only'] = sum(1 for row in moved if not row.startswith('From '))
    params = load_params(workspace)
    done['unverified_params'] = unverified(params, target)
    lines = [f'# Transfer: {source} -> {target}', '',
             f'Written by the launcher on {date.today().isoformat()} when this session inherited the experience of '
             f'`{session}`, which ran in **{source}**. This session runs on **{target}**.', '']
    if target == 'real':
        lines += ['What came over and how to use it ([sim-to-real](../.agents/skills/sim-to-real/SKILL.md)):', '',
                  '- `[strategy]` rows: your plan. Follow them until a measurement here contradicts one.',
                  '- `[param]` rows and every number in PARAMS.json without a `real` entry: priors. Measure each '
                  'before a committal action depends on it, cheapest check first, and record it with '
                  '`api.params.record()` so the next real session starts from it.',
                  '- Untagged rows: treat any number in them as a `[param]`.',
                  f'- {done["moved_sim_only"]} `[sim-only]` rows were moved to '
                  f'`memory/history/{session}/SIM_ONLY.md`; do not act on them.', '']
    else:
        lines += ['The experience comes from the real robot. Its `real` PARAMS entries are the ground the simulation '
                  'should match: when the simulation disagrees with one, record the sim value next to it and say so '
                  'in the handoff; do not overwrite the real entry.', '']
    if done['unverified_params']:
        lines += [f'Numbers measured in {source} only ({len(done["unverified_params"])}), to verify here:', '']
        for name in done['unverified_params']:
            entry = params[name]
            layer = entry.get(source) or next(entry[e] for e in ('sim', 'real') if e in entry)
            unit = entry.get('unit', '')
            lines.append(f'- [ ] `{name}` = {layer.get("value")} {unit}'
                         + (f' ({entry["note"]})' if entry.get('note') else '')
                         + (f'; measured by: {layer["how"]}' if layer.get('how') else ''))
        lines.append('')
    (workspace/TRANSFER).write_text('\n'.join(lines))
    return done


if __name__ == '__main__':  # python3 services/sim_to_real.py lint <workspace>
    import sys
    if len(sys.argv) != 3 or sys.argv[1] != 'lint':
        sys.exit('usage: sim_to_real.py lint <agent-workspace or exported workspace/>')
    print(format_lint(lint(sys.argv[2])))
