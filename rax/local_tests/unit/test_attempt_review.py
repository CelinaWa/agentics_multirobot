import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import attempt_review  # noqa: E402

LAYOUT = {'box_start': {'pos': [-0.15, -0.35, 0.45], 'z': 0.45}}


def sample(t, box, tilt=0.0, g1=(-0.57, -0.35, 0.78), g1_tilt=1.0):
    return {'t': t, 'box': list(box), 'box_tilt_deg': tilt, 'g1': list(g1), 'g1_tilt_deg': g1_tilt,
            'r1': [0.25, 0.04, 0.73], 'r1_tilt_deg': 1.0, 'box_to_goal_m': 3.0, 'robots_standing': g1_tilt < 25}


def test_the_moments_of_a_lift_that_ends_leaning_on_the_table_edge():
    samples = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.16, -0.35, 0.49)),
               sample(102, (-0.40, -0.30, 0.50)), sample(103, (-0.50, -0.28, 0.32), tilt=46)]
    lines = attempt_review.truth_events(samples, LAYOUT)
    text = '\n'.join(lines)
    assert 'box lifted off its support' in text
    assert 'box tipped 46 deg' in text
    assert 'box down again' in text
    assert 'on the floor' not in text        # base z .32 leaning on the edge, not on the floor
    assert 'furthest: 0.36 m' in text


def test_a_box_set_on_the_floor():
    samples = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.6, -0.35, 0.0))]
    assert any('box on the floor (base z 0.00, tilt 0)' in l for l in attempt_review.truth_events(samples, LAYOUT))


def test_a_robot_thrown_during_the_attempt_is_reported():
    samples = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.15, -0.35, 0.45), g1=(-3.4, -1.7, 0.23), g1_tilt=80)]
    assert any('G1 tilted 80 deg' in line for line in attempt_review.truth_events(samples, LAYOUT))


def test_the_trace_lists_motion_and_errors_and_counts_polling(tmp_path):
    log = tmp_path / 'mcp.jsonl'
    events = [
        {'time': 10.0, 'event': 'mcp_request', 'call_id': 'a', 'tool': 'g1_status', 'arguments': {}},
        {'time': 10.1, 'event': 'mcp_result', 'call_id': 'a', 'tool': 'g1_status', 'result': '{}'},
        {'time': 11.0, 'event': 'mcp_request', 'call_id': 'b', 'tool': 'g1_step', 'arguments': {'actions': [[0] * 40] * 5}},
        {'time': 13.0, 'event': 'mcp_result', 'call_id': 'b', 'tool': 'g1_step', 'result': json.dumps({'tilt_max_deg': 2.1})},
        {'time': 14.0, 'event': 'mcp_request', 'call_id': 'c', 'tool': 'r1_move_hand', 'arguments': {'arm': 'left'}},
        {'time': 14.2, 'event': 'mcp_error', 'call_id': 'c', 'type': 'RejectedBeforeAction', 'reason': 'unreachable'},
        {'time': 99.0, 'event': 'mcp_request', 'call_id': 'd', 'tool': 'g1_step', 'arguments': {}},
    ]
    log.write_text('\n'.join(json.dumps(e) for e in events))
    lines = attempt_review.condensed_trace(log, 0, 50)
    assert any('g1_step' in l and '<5 rows>' in l and 'tilt_max 2.1' in l for l in lines)
    assert any('r1_move_hand' in l and 'ERROR RejectedBeforeAction: unreachable' in l for l in lines)
    assert not any(l.startswith('00') and 'g1_status' in l for l in lines[:-1])
    assert lines[-1] == '(1 status, observe and look calls not listed)'
    assert len(lines) == 3


def test_write_review(tmp_path):
    path = attempt_review.write_review(tmp_path / 'reviews' / 'attempt_3.md', 3, 100, 104,
                                       [sample(100, (-0.15, -0.35, 0.45))], LAYOUT, tmp_path / 'none.jsonl',
                                       ['20:31:57 g1 stack stopped (1)'], 'runtime/stream/replay/x.png')
    text = Path(path).read_text()
    assert text.startswith('# Attempt 3') and '## Operator actions' in text and 'g1 stack stopped' in text
    assert '(no tool log)' in text and 'runtime/stream/replay/x.png' in text


def test_the_summary_gives_the_in_air_maximum_not_the_end_position():
    samples = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.35, -0.35, 0.50)), sample(102, (-0.55, -0.35, 0.49)),
               sample(103, (-0.50, -0.35, 0.33), tilt=46)]
    s = attempt_review.attempt_summary(samples, LAYOUT)
    assert s['in_air_max_from_start_m'] == 0.4 and s['in_air_s'] == 1.0 and s['end_from_start_m'] == 0.35
    assert s['end'].startswith('leaning or tipped') and s['disturbed_after_set_down'] is None
    assert s['left_the_air']['from_start_m'] == 0.4 and s['left_the_air']['fell_m'] == 0.16
    assert 'furthest 0.40 m from its start while in the air' in attempt_review.summary_words(s)
    assert 'released or dropped from base z 0.49' in attempt_review.summary_words(s)


def test_a_held_box_dipping_to_the_table_is_not_a_set_down():
    held = [sample(100 + i, (-0.15 - 0.03 * i, -0.35, 0.46 if i % 2 else 0.48)) for i in range(8)]
    s = attempt_review.attempt_summary(held + [sample(108, (-0.5, -0.35, 0.3), tilt=60)], LAYOUT)
    assert s['disturbed_after_set_down'] is None and s['left_the_air'] is not None


def test_a_box_knocked_after_it_was_set_down_names_the_call_before(tmp_path):
    log = tmp_path / 'mcp.jsonl'
    log.write_text('\n'.join(json.dumps(e) for e in [
        {'time': 101.5, 'event': 'mcp_request', 'call_id': 'a', 'tool': 'r1_status'},
        {'time': 102.5, 'event': 'mcp_request', 'call_id': 'b', 'tool': 'r1_move_hand', 'arguments': {}}]))
    samples = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.30, -0.35, 0.50)), sample(102, (-0.30, -0.35, 0.45)),
               sample(103, (-0.30, -0.35, 0.45)), sample(104, (-0.30, -0.35, 0.45)),
               sample(105, (-0.30, -0.35, 0.40), tilt=47)]
    s = attempt_review.attempt_summary(samples, LAYOUT, log)
    d = s['disturbed_after_set_down']
    assert d['tilt_deg'] == 47 and d['after_call'].startswith('r1_move_hand')
    assert 'AFTER it rested level' in attempt_review.summary_words(s)


def test_a_box_on_the_table_untouched():
    s = attempt_review.attempt_summary([sample(100, (-0.15, -0.35, 0.45))] * 3, LAYOUT)
    assert s['end'] == 'on the table, level' and s['in_air_s'] == 0


def test_a_box_lowered_past_the_table_edge_stays_in_the_air_until_the_floor():
    zs = [0.48, 0.40, 0.30, 0.20, 0.10, 0.03, 0.0, 0.0]
    samples = [sample(100, (-0.15, -0.35, 0.45))] + [sample(101 + i, (-0.40 - 0.03 * min(i, 3), -0.35, z)) for i, z in enumerate(zs)]
    s = attempt_review.attempt_summary(samples, LAYOUT)
    assert s['in_air_max_from_start_m'] == 0.34 and s['left_the_air']['fell_m'] == 0.03
    assert s['end'] == 'on the floor, upright' and s['fell_to_floor'] is None


def test_a_collapse_is_a_fall_not_a_held_carry_and_an_edge_release_is_a_fall():
    collapse = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.5, -0.35, 0.48)), sample(102, (-0.7, -0.35, 0.16), tilt=56),
                sample(103, (-0.9, -0.35, 0.06), tilt=15), sample(104, (-0.9, -0.35, 0.06), tilt=15)]
    s = attempt_review.attempt_summary(collapse, LAYOUT)
    assert s['in_air_max_from_start_m'] == 0.35 and s['end'].startswith('leaning')
    edge = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.5, -0.35, 0.48)), sample(102, (-0.6, -0.35, 0.46), tilt=22),
            sample(105, (-0.6, -0.35, 0.46), tilt=22), sample(106, (-0.7, -0.35, 0.0))]
    s = attempt_review.attempt_summary(edge, LAYOUT)
    assert s['fell_to_floor']['from_z'] == 0.46 and 'FELL to the floor' in attempt_review.summary_words(s)


def test_a_box_never_lifted_but_knocked_off_is_an_incident():
    knocked = [sample(100, (-0.15, -0.35, 0.45)), sample(101, (-0.15, -0.35, 0.45)), sample(103, (0.3, -0.5, 0.64), tilt=86),
               sample(105, (0.41, -0.55, 0.0))]
    s = attempt_review.attempt_summary(knocked, LAYOUT)
    assert s['moved_without_lift']['moved_m'] > 0.5 and 'NEVER LIFTED' in attempt_review.summary_words(s)


def test_the_turn_in_the_air_is_measured_from_the_box_yaw():
    turn = [dict(sample(100 + i, (-0.15, -0.35, 0.48)), box_yaw_deg=yaw) for i, yaw in enumerate([0, 5, 12, 179, -178])]
    assert attempt_review.attempt_summary(turn, LAYOUT)['turned_in_air_deg'] == 179


def test_a_nudge_before_any_lift_is_not_a_disturbance_and_a_skim_counts_its_touchdowns():
    nudged = [sample(100, (-0.15, -0.35, 0.45)), sample(102, (-0.15, -0.35, 0.45)), sample(103, (-0.15, -0.31, 0.45))]
    assert attempt_review.attempt_summary(nudged, LAYOUT)['disturbed_after_set_down'] is None
    skim = [sample(100 + i, (-0.15 - 0.05 * i, -0.35, 0.48 if i % 2 else 0.45)) for i in range(1, 8)]
    s = attempt_review.attempt_summary(skim, LAYOUT)
    assert s['came_down'] == 3 and 'came down 3 times' in attempt_review.summary_words(s)
