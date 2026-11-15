"""Usage-accounted inheritance: move what the last session neither opened nor ran out of the experience a new session
inherits, into `cold/` (kept, listed in cold/INDEX.md, one `mv` away).

Memory, code and notes only grew: every session appends, nobody can tell what was ever used again, and whoever wrote
a note is the one judging whether to keep it. Here the harness decides from the record, not from anyone's judgement: a
file under memory/ or code/ stays if the session's transcripts or tool log name it, or if a file that stays imports or
names it; everything else goes cold. The few files the framework itself reads always stay.

  python retire_unused.py <session root> <exported workspace> [--dry-run]
"""
import argparse
import json
import re
import shutil
from pathlib import Path

KEEP = {'memory/CURRENT.md', 'memory/STRATEGY.md', 'memory/STAGES.json', 'memory/attempts.jsonl', 'memory/TOOL_REQUESTS.md'}
KEEP_DIRS = ('memory/ground_truth/',)
IMPORT = re.compile(r'^\s*(?:from\s+([\w\.]+)\s+import|import\s+([\w\.]+))', re.M)


def usage_text(session):
    """Everything the session's agent read, wrote or ran, as one string: its transcripts and its tool log."""
    parts = []
    for path in list((session / 'claude-home').rglob('*.jsonl')) + [session / 'mcp.jsonl']:
        try:
            parts.append(path.read_text(errors='replace'))
        except OSError:
            pass
    return '\n'.join(parts)


def used_files(workspace, text):
    files = [p for root in ('memory', 'code') for p in (workspace / root).rglob('*') if p.is_file()]
    rel = {p: p.relative_to(workspace).as_posix() for p in files}
    used = {p for p in files if rel[p] in KEEP or rel[p].startswith(KEEP_DIRS) or rel[p] in text}
    by_stem = {}
    for p in files:
        if p.suffix == '.py':
            by_stem.setdefault(p.stem, []).append(p)
    frontier = list(used)
    while frontier:                       # what a used file imports or names is used too
        path = frontier.pop()
        if path.suffix not in ('.py', '.sh', '.json', '.md') or path.stat().st_size > 2_000_000:
            continue
        try:
            body = path.read_text(errors='replace')
        except OSError:
            continue
        named = set()
        if path.suffix == '.py':
            for a, b in IMPORT.findall(body):
                named.update(by_stem.get((a or b).split('.')[-1], []))
        if path.suffix in ('.py', '.sh'):
            for other in files:
                if other not in used and (rel[other] in body or (other.suffix in ('.py', '.sh', '.json') and other.name in body)):
                    named.add(other)
        for other in named - used:
            used.add(other); frontier.append(other)
    return files, used


def retire(session, workspace, dry_run=False):
    session, workspace = Path(session), Path(workspace)
    files, used = used_files(workspace, usage_text(session))
    cold = workspace / 'cold'
    moved, kept_bytes, moved_bytes, by_dir = 0, 0, 0, {}
    for path in files:
        size = path.stat().st_size
        if path in used:
            kept_bytes += size
            continue
        rel = path.relative_to(workspace)
        moved += 1; moved_bytes += size
        top = '/'.join(rel.parts[:2]) if len(rel.parts) > 2 else rel.parts[0]
        entry = by_dir.setdefault(top, [0, 0]); entry[0] += 1; entry[1] += size
        if not dry_run:
            target = cold / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(path), str(target))
    if not dry_run and moved:
        lines = [f'# cold: inherited files that session {session.name} neither opened nor ran', '',
                 'The harness moved them here to keep what you inherit to what is in use. Nothing is lost: move a file back',
                 'to the same path without `cold/` if you need it (`mv cold/memory/X memory/X`).', '',
                 '| where | files | kB |', '|---|---|---|']
        previous = cold / 'INDEX.md'
        lines += [f'| {name} | {n} | {size // 1024} |' for name, (n, size) in sorted(by_dir.items())]
        if previous.exists():
            lines += ['', '## earlier', ''] + previous.read_text().splitlines()[1:40]
        previous.write_text('\n'.join(lines) + '\n')
        for root in ('memory', 'code'):      # empty directories left behind
            for d in sorted((p for p in (workspace / root).rglob('*') if p.is_dir()), reverse=True):
                if not any(d.iterdir()):
                    d.rmdir()
    return {'files': len(files), 'kept': len(files) - moved, 'kept_kb': kept_bytes // 1024, 'moved': moved, 'moved_kb': moved_bytes // 1024}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('session'); ap.add_argument('workspace'); ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    print(json.dumps(retire(a.session, a.workspace, a.dry_run)))
