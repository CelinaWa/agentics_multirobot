"""What a person standing beside the robots could report, from the simulation's ground truth. The operator's replies
and the record the agent can read (stream/ground_truth.jsonl) carry only this when the operator runs with
--reply human: judgements a person makes by eye (where the object ended up, whether it left its support, fell, was
held by which hands, whether the task's rule and success condition were met, whether the robots stand) and the
places the robots were put back to. No measured coordinates, distances or contact flags: on real robots nobody has
them, so what an agent learns from them does not carry over. The full rows stay in <session>/operator/, which the
agent cannot read."""


def _about(seconds):
    if seconds is None or seconds < 0.5:
        return None
    return 'a few seconds' if seconds < 4 else f'about {int(round(seconds / 5.0)) * 5 or 5} s'


def observed(row, closer_than_before=None):
    """The human-visible part of one ground-truth row."""
    out = {k: row[k] for k in ('t', 'time', 'event', 'reset', 'request', 'third_person', 'restart') if k in row}
    if 'robots_standing' in row:
        out['robots_standing'] = row['robots_standing']
    if row.get('event') == 'after_reset':
        state = row.get('state') or {}
        out['state'] = {k: state[k] for k in ('reset', 'state_ok', 'robots_standing', 'warnings', 'robots_put_at') if k in state}
        progress = state.get('task_progress') or {}
        out['state']['task_progress'] = {k: progress.get(k) for k in ('attempts', 'attempts_since_it_improved')}
        if 'robot_motion_min' in state:
            out['state']['robot_motion_min'] = state['robot_motion_min']
        return out
    if 'box' not in row:
        return out
    can_task = 'can_in_bin' in row
    name = 'can' if can_task else 'box'
    tilt, z = row.get('box_tilt_deg', 0.0), row['box'][2]
    pose = 'lying on its side' if tilt >= 60 else 'tipped' if tilt >= 15 else 'upright'
    place = ('inside the bin' if row.get('can_in_bin') else 'held up, off any support' if row.get('box_lifted') else
             'on the floor' if z < 0.05 else 'where it started' if row.get('box_from_start_m', 1.0) < 0.05 else 'resting on something above the floor')
    seen = {'object': name, 'ended': f'{pose}, {place}', 'success': bool(row.get('success')) and row.get('carried_by_both', True) is not False}
    attempt = row.get('attempt')
    if attempt:
        seen['left_its_support'] = bool((attempt.get('in_air_s') or 0) >= 0.5)
        seen['off_its_support_for'] = _about(attempt.get('in_air_s'))
        seen['fell_to_the_floor'] = attempt.get('fell_to_floor') is not None
        seen['pushed_or_knocked_without_being_lifted'] = attempt.get('moved_without_lift') is not None
        both, without = attempt.get('in_air_with_both_robots_s'), attempt.get('in_air_without_both_robots_s')
        if seen['left_its_support'] and both is not None and without is not None and not can_task:
            seen['all_four_hands_on_it_while_off_its_support'] = ('all the time' if without <= 2.0 else 'never' if both <= 1.0
                                                                  else 'most of the time' if both >= 2 * without else 'part of the time')
        if closer_than_before is not None and not can_task:
            seen['carried_closer_to_the_goal_than_in_any_attempt_before'] = bool(closer_than_before)
    roles = row.get('roles')
    if roles:
        carried, floor = roles.get('bin_carried_m', 0.0), roles.get('bin_moved_on_the_floor_m', 0.0)
        seen['bin'] = {'upright': row.get('bin_tilt_deg', 0.0) < 20,
                       'moved': ('not moved' if carried + floor < 0.05 else 'moved only a little' if carried + floor < 0.3 else
                                 'carried off the floor' if carried >= 0.1 and floor <= 0.15 else
                                 'partly carried, partly pushed or dragged along the floor' if carried >= 0.1 else
                                 'pushed or dragged along the floor' if floor > 0.15 else 'not moved'),
                       'hands_on_it_while_it_moved': roles.get('moved_the_bin'),
                       'ended_beside_the_robot_with_the_can': bool(roles.get('bin_brought_to_the_can_robot'))}
        seen['hands_on_the_can_while_it_moved'] = roles.get('moved_the_can')
        seen['done_as_the_task_asks'] = bool(roles.get('as_the_task_asks'))
    out['seen'] = seen
    return out


def words(view):
    """The same, as the operator would say it."""
    seen = view.get('seen')
    if not seen:
        return ''
    name = seen['object']
    text = f" What I saw before this reset: the {name} ended {seen['ended']}"
    if 'left_its_support' in seen:
        text += (f"; it was off its support for {seen['off_its_support_for']}" if seen['left_its_support'] else '; it never left its support')
        if seen.get('fell_to_the_floor'):
            text += '; it fell to the floor'
        if seen.get('pushed_or_knocked_without_being_lifted'):
            text += '; it was pushed or knocked along without being lifted'
        rule = seen.get('all_four_hands_on_it_while_off_its_support')
        if rule:
            text += f'; all four hands were on it {rule} while it was off its support'
        closer = seen.get('carried_closer_to_the_goal_than_in_any_attempt_before')
        if closer is not None:
            text += '; it was carried closer to the goal than in any attempt before' if closer else '; it was not carried closer to the goal than before'
    bin_ = seen.get('bin')
    if bin_:
        who = lambda robots: ' and '.join(r.upper() for r in robots) if robots else 'no robot'
        text += (f". The bin was {bin_['moved']} (hands on it while it moved: {who(bin_['hands_on_it_while_it_moved'])}), "
                 f"{'upright' if bin_['upright'] else 'NOT upright'}, "
                 f"{'beside' if bin_['ended_beside_the_robot_with_the_can'] else 'not beside'} the robot with the can; "
                 f"hands on the can while it moved: {who(seen.get('hands_on_the_can_while_it_moved'))}")
    text += f". Robots {'both standing' if view.get('robots_standing', True) else 'NOT both standing'}."
    return text
