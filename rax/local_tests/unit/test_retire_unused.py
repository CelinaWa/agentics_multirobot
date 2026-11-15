import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import retire_unused  # noqa: E402


def test_unused_files_go_cold_and_what_used_code_imports_stays(tmp_path):
    session, ws = tmp_path / 'session', tmp_path / 'ws'
    (session / 'claude-home' / 'p').mkdir(parents=True)
    (session / 'claude-home' / 'p' / 't.jsonl').write_text(json.dumps({'cmd': 'python code/s2/run.py; cat memory/TIPS.md'}))
    for rel, body in (('memory/CURRENT.md', 'x'), ('memory/TIPS.md', 'x'), ('memory/history/a/PROGRESS.md', 'old'),
                      ('memory/attempts.jsonl', '{}'), ('code/s2/run.py', 'import helper\n'), ('code/s1/helper.py', 'x = 1\n'),
                      ('code/s1/dead.py', 'x = 2\n')):
        path = ws / rel; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(body)
    out = retire_unused.retire(session, ws)
    assert out['moved'] == 2
    assert (ws / 'code/s1/helper.py').exists() and (ws / 'memory/TIPS.md').exists() and (ws / 'memory/CURRENT.md').exists()
    assert (ws / 'cold/code/s1/dead.py').exists() and (ws / 'cold/memory/history/a/PROGRESS.md').exists()
    assert 'neither opened nor ran' in (ws / 'cold/INDEX.md').read_text()
