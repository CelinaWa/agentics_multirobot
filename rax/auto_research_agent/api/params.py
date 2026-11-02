"""Measured numbers, kept per environment, so one controller runs in simulation and on the real robot.

`memory/PARAMS.json` maps a name to its measurements in each environment:

    {"reach.forward_m": {"unit": "m", "note": "empty right hand ahead of the pelvis at table height",
                         "sim":  {"value": 0.26, "measured": "2026-11-02", "how": "3 hovers, depth"},
                         "real": {"value": 0.24, "measured": "2026-11-05", "how": "hover over tape mark"}}}

This session's environment is RAX_ENVIRONMENT ('real' or 'sim'). In code:

    from api.params import param, record
    reach = param('reach.forward_m', default=0.25)     # this environment's value, else the other's (a prior)
    record('reach.forward_m', 0.24, unit='m', how='hover over tape mark')   # into this environment's entry

`param()` falls back to the other environment's value and says so on stderr (once per name): on the real
robot that is a sim number to verify. `param(..., verified=True)` refuses that fallback, for numbers a
committal action depends on. `python -m api.params` lists every name and whether it holds here.
"""
from datetime import date
import json
import os
from pathlib import Path
import sys

PATH = Path(__file__).resolve().parents[1]/'memory'/'PARAMS.json'
ENVIRONMENT = os.environ.get('RAX_ENVIRONMENT', 'real')
OTHER = {'real': 'sim', 'sim': 'real'}[ENVIRONMENT]
_warned = set()


class Unverified(LookupError):
    pass


def load():
    try:
        return json.loads(PATH.read_text())
    except FileNotFoundError:
        return {}


def param(name, default=None, *, verified=False):
    entry = load().get(name, {})
    if ENVIRONMENT in entry:
        return entry[ENVIRONMENT]['value']
    if OTHER in entry and not verified:
        if name not in _warned:
            _warned.add(name)
            print(f'[params] {name}: using the {OTHER} value {entry[OTHER]["value"]} {entry.get("unit", "")}; '
                  f'not measured on {ENVIRONMENT} yet', file=sys.stderr)
        return entry[OTHER]['value']
    if default is not None and not verified:
        return default
    raise Unverified(f'{name} has no {ENVIRONMENT} measurement' + (f' ({OTHER}: {entry[OTHER]["value"]})'
                                                                   if OTHER in entry else ''))


def record(name, value, *, unit=None, how=None, note=None):
    """Store this environment's measurement of name (the other environment's entry is kept)."""
    data = load()
    entry = data.setdefault(name, {})
    if unit is not None:
        entry['unit'] = unit
    if note is not None:
        entry['note'] = note
    entry[ENVIRONMENT] = {'value': value, 'measured': date.today().isoformat(), **({'how': how} if how else {})}
    PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = PATH.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')
    tmp.replace(PATH)
    return entry


def main():
    data = load()
    if not data:
        print(f'no {PATH.relative_to(PATH.parents[1])} yet')
    for name in sorted(data):
        entry = data[name]
        here = entry.get(ENVIRONMENT, {}).get('value')
        there = entry.get(OTHER, {}).get('value')
        state = 'measured here' if here is not None else f'{OTHER} only: verify'
        print(f'{name:40s} {ENVIRONMENT}={here!s:>10} {OTHER}={there!s:>10} {entry.get("unit", ""):5s} {state}')


if __name__ == '__main__':
    main()
