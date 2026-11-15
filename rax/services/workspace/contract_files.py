"""MCP_CONTRACT.json and MCP_SESSION.md: the session's tool list in the workspace."""
import json
from pathlib import Path


def offered_tools(setup, backend_options=None):
    """The tools this session really offers: the robot's list, less what the operator hid, plus or minus the interfaces
    its backend options switch (a from-scratch session found the list naming a hidden tool, missing two offered ones
    and calling a simulation 'real hardware')."""
    tools = set(setup.tools)
    groups = [backend_options or {}] + [v for v in (backend_options or {}).values() if isinstance(v, dict)]
    switched = {'hands_walk': 'g1_hands_walk', 'planner_stream': 'g1_planner', 'gait_stream': 'r1_gait'}
    for group in groups:
        tools -= set(group.get('hidden_tools') or ())
    for option, tool in switched.items():
        robot = tool.split('_')[0]
        if not any(tool.startswith(name.split('_')[0] + '_') or name.startswith(robot) for name in setup.tools):
            continue                                   # another robot's interface
        if any(group.get(option) for group in groups):
            tools.add(tool)
        else:
            tools.discard(tool)
    if any(name.startswith('g1_') for name in tools):          # what cannot run on this host is not listed either
        from services.g1sonic.tools import unavailable_here
        tools -= unavailable_here()
    return tuple(sorted(tools))


def describe(setup, tools=None) -> str:
    if setup.camera_geometry:
        geometry = 'RGB images, depth and camera calibration are supplied.'
    else:
        geometry = 'Only RGB images and measured robot state are supplied.'
    listing = ('Tools: ' + ', '.join(f'`{name}`' for name in (tools or setup.tools))
               + '. The live tool schemas give arguments and limits. `status` reports the session budgets '
               'and the live robot; `request_operator` asks the human operator for help or a scene reset.')
    return (f'# Session tools\n\nRobot: {setup.context()["robot"]["name"]} ({'simulation' if setup.environment == 'sim' else 'real hardware'}). '
            f'Observation profile: {setup.observation_profile}.\n\n{geometry}\n\n{listing}\n')


def manifest(setup, tools=None):
    return {'robot': setup.robot, 'observation_profile': setup.observation_profile,
            'subagents': setup.subagents, 'tools': list(tools or setup.tools)}


def write_contract(workspace, setup, backend_options=None):
    workspace = Path(workspace)
    tools = offered_tools(setup, backend_options) if backend_options is not None else None
    (workspace / 'MCP_CONTRACT.json').write_text(json.dumps(manifest(setup, tools), indent=2) + '\n')
    (workspace / 'MCP_SESSION.md').write_text(describe(setup, tools))
