"""What the team backend needs to know about each kind of robot it can hold: which of its tools read, hold and move
the base, and how a base motion is commanded and reported. Everything else about a robot stays in its own backend."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable


@dataclass(frozen=True)
class Kind:
    backend: str                    # services.backends registry key
    status: str
    observe: str
    hold: str
    radius_m: float                 # body radius in the plane with the arms at the sides
    # tool -> the base motion its arguments command, as an upper bound: (forward m, left m, turn deg)
    base_motion: Callable = field(default=lambda tool, arguments: None)
    # (tool, arguments, result value) -> (forward m, left m, turn deg, measured?, extra uncertainty m) or None
    base_result: Callable = field(default=lambda tool, arguments, value: None)


def _g1_motion(tool, arguments):
    if tool == 'g1_walk':
        return float(arguments.get('forward_m', 0.0)), 0.0, float(arguments.get('turn_deg', 0.0))
    return None


def _g1_result(tool, arguments, value):
    if tool != 'g1_walk':
        return None
    feedback = (value.get('transition') or {}).get('feedback') or {} if isinstance(value, dict) else {}
    odometry = feedback.get('odometry_m')
    turn = feedback.get('measured_turn_deg')
    if isinstance(odometry, dict) and turn is not None:
        return float(odometry.get('forward', 0.0)), float(odometry.get('left', 0.0)), float(turn), True, 0.0
    forward, left, commanded_turn = _g1_motion(tool, arguments)
    return forward, left, commanded_turn, False, 0.0


def _r1_motion(tool, arguments):
    if tool == 'r1_walk':
        return float(arguments.get('forward_m', 0.0)), 0.0, float(arguments.get('turn_deg', 0.0))
    if tool == 'r1_carry_release':   # steps forward with the hand held, up to max_steps of step_m
        reach = max(float(arguments.get('target_forward_m', 0.0)),
                    float(arguments.get('step_m', 0.07)) * int(arguments.get('max_steps', 5)))
        return reach + float(arguments.get('advance_m', 0.0)), 0.0, 0.0
    return None


def _r1_result(tool, arguments, value):
    if not isinstance(value, dict):
        return None
    if tool == 'r1_walk':
        walked = value.get('walked_m')
        turned = value.get('turned_deg', value.get('turn_deg'))
        forward, _, commanded_turn = _r1_motion(tool, arguments)
        measured = walked is not None
        # the side drift's sign is the bridge's own: it widens the uncertainty instead of moving the entry
        return (float(walked if measured else forward), 0.0, float(turned if turned is not None else commanded_turn),
                measured, abs(float(value.get('lateral_m') or 0.0)))
    if tool == 'r1_carry_release':
        walked = value.get('walked_total_m')
        if walked is not None:
            return float(walked), 0.0, 0.0, True, 0.0
        return _r1_motion(tool, arguments) + (False, 0.0)
    return None


KINDS = {
    'g1sonic': Kind('g1sonic', 'g1_status', 'g1_observe', 'g1_hold', 0.35, _g1_motion, _g1_result),
    'r1sonic': Kind('r1sonic', 'r1_status', 'r1_observe', 'r1_hold', 0.30, _r1_motion, _r1_result),
    'mock': Kind('mock', 'mock_observe', 'mock_observe', 'mock_observe', 0.2),
}
