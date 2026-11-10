"""Operator motion arming: a file in the session root that only the operator writes.

The agent's sandbox cannot write outside its workspace, so it cannot arm itself. The
backend re-reads the file before and during every motion; disarming stops a running
motion within one servo period.
"""
import json
import os
from pathlib import Path
import threading
import time

# Holds the robot service places on itself (services/team): while one is set, read() reports the session as not
# armed, so every running motion stops within one tick and holds, the way it does for an operator disarm. A hold never
# arms anything and never touches the operator's file; it lives in this process and ends with release().
_HOLDS = {}
_HOLDS_LOCK = threading.Lock()


def arm_file(session_root):
    return Path(session_root)/'operator'/'armed.json'


def hold(session_root, key, reason):
    """Stop and refuse motion in this process until release(session_root, key)."""
    with _HOLDS_LOCK:
        _HOLDS.setdefault(str(Path(session_root)), {})[key] = str(reason)


def release(session_root, key):
    with _HOLDS_LOCK:
        reasons = _HOLDS.get(str(Path(session_root)), {})
        reasons.pop(key, None)
        if not reasons:
            _HOLDS.pop(str(Path(session_root)), None)


def held(session_root):
    """{key: reason} of the holds now set on this session by this process."""
    with _HOLDS_LOCK:
        return dict(_HOLDS.get(str(Path(session_root)), {}))


def read(session_root, now=None, holds=True):
    """{'armed': bool, ...}; expired or unreadable arming counts as disarmed. holds=False reads the operator's file
    alone, without this process's own holds."""
    state = _read_file(session_root, now)
    if holds and state['armed']:
        reasons = held(session_root)
        if reasons:
            return {'armed': False, 'held': True,
                    'reason': 'held by the robot service: ' + '; '.join(reasons.values())}
    return state


def _read_file(session_root, now=None):
    path = arm_file(session_root)
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError):
        return {'armed': False, 'reason': 'not armed by the operator'}
    now = time.time() if now is None else now
    until = value.get('until')
    if value.get('armed') is not True:
        return {'armed': False, 'reason': value.get('reason') or 'disarmed by the operator'}
    if until is not None and now >= until:
        return {'armed': False, 'reason': 'operator arming expired'}
    return {'armed': True, 'until': until, 'by': value.get('by'), 'since': value.get('since')}


def write(session_root, armed, *, minutes=None, by=None, reason=None):
    path = arm_file(session_root)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    now = time.time()
    value = {'armed': bool(armed), 'since': now, 'by': by or os.environ.get('USER'),
             'until': now + 60 * minutes if armed and minutes else None,
             'reason': reason}
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value))
    tmp.chmod(0o600)
    os.replace(tmp, path)
    return value
