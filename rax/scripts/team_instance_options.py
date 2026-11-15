"""Backend options for another simulation instance (sim_team/team_sim.sh TEAM_INSTANCE=n): every port shifted by 100 n.

  python team_instance_options.py <options.json> <n> > <options for instance n>.json
"""
import json
import sys

PORTS = {'g1': ('pose_port', 'state_port', 'camera_port'), 'r1': ('stack_port',)}

options = json.loads(open(sys.argv[1]).read())
shift = 100 * int(sys.argv[2])
for robot, keys in PORTS.items():
    for key in keys:
        if key in options.get(robot, {}):
            options[robot][key] += shift
print(json.dumps(options, indent=2))
