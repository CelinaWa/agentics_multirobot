"""MCP tool schemas and request validation of the team tools (services/team/backend.py)."""
from __future__ import annotations

import math

STATUS, OBSERVE = 'team_status', 'team_observe'
START, WAIT, PARALLEL, STOP = 'team_start', 'team_wait', 'team_parallel', 'team_stop'
SET_POSE = 'team_set_pose'
TEAM_TOOLS = (STATUS, OBSERVE, START, WAIT, PARALLEL, STOP, SET_POSE)
READ_TOOLS = (STATUS, OBSERVE)
MUTATING = frozenset({START, PARALLEL})
MAX_WAIT_S = 600.0


class Refused(ValueError):
    """A request refused before anything moved."""


def refuse_unless(ok, message):
    if not ok:
        raise Refused(message)


def tool_definitions(robots, example):
    """robots: the team's robot names; example: (tool, tool) of two different robots, for the descriptions."""
    names = ', '.join(robots)
    calls = {'type': 'array', 'minItems': 1, 'maxItems': len(robots), 'items': {
        'type': 'object', 'required': ['tool'], 'additionalProperties': False, 'properties': {
            'tool': {'type': 'string', 'description': "one robot's own tool, e.g. " + ' or '.join(example)},
            'arguments': {'type': 'object', 'description': "that tool's arguments (default {})"}}},
        'description': 'at most one call per robot'}
    group = {
        'calls': calls,
        'coupled': {'type': 'boolean', 'default': False, 'description': (
            'true when these robots work on one object within reach of each other (a shared carry, a handover): '
            'the body-separation check is skipped between them, and stop_all_on_failure defaults to true')},
        'stop_all_on_failure': {'type': 'boolean', 'description': (
            'when one call fails or stops early, stop the other calls of this group where they are (each robot holds). '
            'Default: the value of coupled')},
    }
    return [
        {'name': STATUS, 'description': (
            f'The team at a glance ({names}): arming, what each robot is doing (idle, or the call it is running), each '
            "robot's fault and last events, the team frame (where each robot stands, with its uncertainty, and the "
            "body-to-body gaps) and the calls still running. Each robot's full state is in its own status tool. "
            'Moves nothing.'),
         'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False}},
        {'name': OBSERVE, 'description': (
            "Every idle robot's own observation in one reply, in the order of image_order, with the team frame. A robot "
            'that is running a call is reported as busy instead. Moves nothing.'),
         'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False}},
        {'name': PARALLEL, 'description': (
            f'Run one call on each of several robots at the same time and wait until all have ended: {names} each take '
            'one of their own tools. Every call is validated first (arguments, arming, reach, body separation); if any '
            'is refused, nothing moves. The calls start together. Returns each call\'s own result and images. Use it '
            'for independent subtasks that should not wait for each other, and with coupled=true for the steps of a '
            'shared task that must start together.'),
         'inputSchema': {'type': 'object', 'required': ['calls'], 'additionalProperties': False, 'properties': group}},
        {'name': START, 'description': (
            'Like team_parallel, but returns at once with a job id per call; the calls keep running. A robot that is '
            "running a call refuses other calls until it ends; the other robots stay free, so you can work with one "
            'robot while another walks. Collect the results with team_wait.'),
         'inputSchema': {'type': 'object', 'required': ['calls'], 'additionalProperties': False, 'properties': group}},
        {'name': WAIT, 'description': (
            'Wait for started calls to end (all running ones, or the given job ids) for at most timeout_s, and return '
            'each one\'s state. A result and its images are returned once, by the first team_wait after the call ended.'),
         'inputSchema': {'type': 'object', 'additionalProperties': False, 'properties': {
             'jobs': {'type': 'array', 'items': {'type': 'string'}, 'maxItems': 16},
             'timeout_s': {'type': 'number', 'minimum': 0, 'maximum': MAX_WAIT_S, 'default': 60}}}},
        {'name': STOP, 'description': (
            'Stop every running call where it is: each robot holds its last command, as after an operator disarm, and the '
            'session stays armed. A call that cannot be interrupted (a gait or a grasp primitive that runs inside the '
            "robot's own stack) ends on its own first; the reply says which. Use it when one robot's result makes "
            "another robot's running call wrong."),
         'inputSchema': {'type': 'object', 'additionalProperties': False, 'properties': {
             'reason': {'type': 'string', 'maxLength': 300}}}},
        {'name': SET_POSE, 'description': (
            'Correct where one robot stands in the team frame (metres, degrees; x forward and y left of the frame the '
            'operator chose) from something you measured: a landmark both robots see, one robot seen by the other, a '
            'known contact. The frame is otherwise dead reckoning and drifts with every walk. Give the uncertainty '
            'of your measurement. Moves nothing.'),
         'inputSchema': {'type': 'object', 'required': ['robot', 'x_m', 'y_m', 'yaw_deg'], 'additionalProperties': False,
                         'properties': {
             'robot': {'type': 'string', 'enum': list(robots)},
             'x_m': {'type': 'number'}, 'y_m': {'type': 'number'}, 'yaw_deg': {'type': 'number'},
             'uncertainty_m': {'type': 'number', 'minimum': 0.0, 'maximum': 2.0, 'default': 0.05},
             'source': {'type': 'string', 'maxLength': 200, 'description': 'how you measured it'}}}},
    ]


def _only(arguments, allowed):
    unknown = set(arguments) - set(allowed)
    refuse_unless(not unknown, f'unknown arguments: {sorted(unknown)}')


def _number(value, name, low=-math.inf, high=math.inf):
    refuse_unless(type(value) in (int, float) and math.isfinite(value), f'{name} must be a finite number')
    refuse_unless(low <= value <= high, f'{name} must be in [{low:g}, {high:g}]')


def validate_request(name, arguments, robots):
    refuse_unless(isinstance(arguments, dict), 'arguments must be an object')
    if name in (STATUS, OBSERVE):
        _only(arguments, ())
    elif name in (START, PARALLEL):
        _only(arguments, ('calls', 'coupled', 'stop_all_on_failure'))
        calls = arguments.get('calls')
        refuse_unless(isinstance(calls, list) and 1 <= len(calls) <= len(robots),
                      f'calls must list 1..{len(robots)} calls, at most one per robot')
        for call in calls:
            refuse_unless(isinstance(call, dict) and isinstance(call.get('tool'), str)
                          and not set(call) - {'tool', 'arguments'}, 'each call is {tool, arguments}')
            refuse_unless(isinstance(call.get('arguments', {}), dict), 'a call\'s arguments must be an object')
        for key in ('coupled', 'stop_all_on_failure'):
            refuse_unless(type(arguments.get(key, False)) is bool, f'{key} must be a boolean')
    elif name == WAIT:
        _only(arguments, ('jobs', 'timeout_s'))
        jobs = arguments.get('jobs', [])
        refuse_unless(isinstance(jobs, list) and len(jobs) <= 16 and all(isinstance(j, str) for j in jobs),
                      'jobs must be a list of job ids')
        _number(arguments.get('timeout_s', 60), 'timeout_s', 0.0, MAX_WAIT_S)
    elif name == STOP:
        _only(arguments, ('reason',))
        refuse_unless(isinstance(arguments.get('reason', ''), str) and len(arguments.get('reason', '')) <= 300,
                      'reason must be text of at most 300 characters')
    elif name == SET_POSE:
        _only(arguments, ('robot', 'x_m', 'y_m', 'yaw_deg', 'uncertainty_m', 'source'))
        refuse_unless(arguments.get('robot') in robots, f'robot must be one of {list(robots)}')
        _number(arguments.get('x_m'), 'x_m', -100.0, 100.0)
        _number(arguments.get('y_m'), 'y_m', -100.0, 100.0)
        _number(arguments.get('yaw_deg'), 'yaw_deg', -360.0, 360.0)
        _number(arguments.get('uncertainty_m', 0.05), 'uncertainty_m', 0.0, 2.0)
        refuse_unless(isinstance(arguments.get('source', ''), str) and len(arguments.get('source', '')) <= 200,
                      'source must be text of at most 200 characters')
    else:
        raise Refused(f'Unknown tool: {name}')
