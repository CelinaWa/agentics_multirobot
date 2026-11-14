"""The agent's attempt log and funnel (auto_research_agent/api/attempts.py)."""
import importlib.util
import json
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2]/'auto_research_agent'/'api'/'attempts.py'


def load(workspace):
    (workspace/'api').mkdir(parents=True)
    (workspace/'api'/'attempts.py').write_text(SOURCE.read_text())
    spec = importlib.util.spec_from_file_location('attempts_under_test', workspace/'api'/'attempts.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_funnel_counts_tries_by_stage_mode_and_session(tmp_path):
    root = tmp_path/'20261114T000000Z_task_abc'
    attempts = load(root/'agent-workspace')
    memory = root/'agent-workspace'/'memory'
    memory.mkdir()
    (memory/'STAGES.json').write_text(json.dumps({'stages': [{'id': 'grip'}, {'id': 'lift'}]}))
    (memory/'attempts.jsonl').write_text(json.dumps({'session': 'older', 'stage': 'grip', 'ok': True, 'mode': 'chain'}) + '\n')
    attempts.record('grip', False, mode='practice', cause='pushed the box')
    attempts.record('grip', True, mode='practice', params={'y': 0.12})
    attempts.record('lift', False, cause='slipped')
    table = {e['stage']: e for e in attempts.funnel()}
    assert list(table) == ['grip', 'lift']
    assert table['grip']['session']['practice'] == [1, 2] and table['grip']['all']['chain'] == [1, 1]
    assert table['grip']['session']['chain'] == [0, 0] and table['grip']['sessions'] == 2
    assert table['lift']['causes'] == [('slipped', 1)]


def test_ground_truth_is_summarised_per_session(tmp_path):
    root = tmp_path/'20261114T000000Z_task_abc'
    attempts = load(root/'agent-workspace')
    (root/'agent-workspace'/'memory'/'ground_truth').mkdir(parents=True)
    rows = [{'event': 'before_reset', 'box': [0, 0, 0], 'box_from_start_m': 0.5, 'box_to_goal_m': 3.0, 'success': False},
            {'event': 'g1_stack_stopped', 'box': [0, 0, 0], 'box_from_start_m': 0.1, 'box_to_goal_m': 3.2},
            {'event': 'operator_end', 'box': [0, 0, 0], 'box_from_start_m': 0.2, 'box_to_goal_m': 2.9, 'success': False}]
    (root/'agent-workspace'/'memory'/'ground_truth'/'older.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (summary,) = attempts.truth_summary()
    assert summary == {'session': 'older', 'resets': 1, 'g1_restarts': 1, 'box_left_start_m_max': 0.5,
                       'box_to_goal_m_min': 2.9, 'successes': 0}


def test_a_stage_outside_the_graph_is_refused_and_a_row_carries_its_check(tmp_path):
    import pytest
    root = tmp_path/'20261114T000000Z_task_abc'
    attempts = load(root/'agent-workspace')
    memory = root/'agent-workspace'/'memory'
    memory.mkdir()
    (memory/'STAGES.json').write_text(json.dumps({'stages': [{'id': 'grip', 'check': 'pinch 20-45 mm'}]}))
    with pytest.raises(ValueError, match='not in memory/STAGES.json'):
        attempts.record('walk_hold', True)
    assert attempts.record('grip', True)['check'] == 'pinch 20-45 mm'
