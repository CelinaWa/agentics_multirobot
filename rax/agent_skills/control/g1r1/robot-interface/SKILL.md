---
name: robot-interface
description: The Unitree G1 and the Unitree R1 as one team under one agent - the team tools (calls on both robots at once, waiting, the shared stop, the team frame and its separation check), what they guarantee and what they do not, and where each robot's own interface and motion skills are. Read before composing any robot action.
---

# Robot interface: the G1 + R1 team

You control two robots through one robot service: a Unitree G1 (tools `g1_*`) and a Unitree R1 (tools `r1_*`).
Each keeps its own backend, limits and guards. The team tools (`team_*`) add what two robots need: calls that run on
both at once, one stop, and a shared frame.

## Each robot's own skills

Everything about one body is in that robot's own skills. Read a robot's `robot-interface` before you move it; its
numbers do not apply to the other robot.

- G1: [robot-interface](../../robots/g1/skills/robot-interface/SKILL.md) and the other skills in
  `.agents/robots/g1/skills/` (walking and reaching on its controller, its planner, its perception).
- R1: [robot-interface](../../robots/r1/skills/robot-interface/SKILL.md) and the other skills in
  `.agents/robots/r1/skills/`.

Where a shared skill says "the robot", it means the robot you are commanding at that moment.

**What each robot already learned.** `.agents/robots/g1/skill-library/` and `.agents/robots/r1/skill-library/` hold the
skills each robot learned in its own single-robot sessions (files only; absent when that robot has none yet). Their
code calls that robot's own tools, which this session has, so a verified skill runs here unchanged for that robot.
Read their `INDEX.md` before you practise a subtask one robot does alone. Skills you learn here, for one robot or for
both, go into this session's own `skill-library/`
([continual-learning](../continual-learning/SKILL.md)); say in each which robot its code commands.

## Team tools

| Tool | What it does |
|---|---|
| `team_status` | arming, what each robot is running, each robot's fault and last events, the team frame, running jobs |
| `team_observe` | every idle robot's own observation in one reply (`image_order` names the images), with the team frame |
| `team_parallel` | one call per robot, started together; returns when all have ended, with each result |
| `team_start` | the same, but returns at once with job ids; the calls keep running |
| `team_wait` | wait for started calls (all, or given job ids) up to `timeout_s`; results and images are returned once |
| `team_stop` | stop every running call where it is; each robot holds; the session stays armed |
| `team_set_pose` | correct one robot's pose in the team frame from a measurement |

A robot runs one call at a time. While it runs a started call, every other call for that robot is refused (`... is
running ...`); the other robot stays free. So there are three ways to use two robots:

1. **One after the other.** Call `g1_*` and `r1_*` tools directly, as with one robot.
2. **Both at once, then look.** `team_parallel` with one call each. Use it when neither robot should wait for the
   other, and for the steps of a shared task that must start together.
3. **One works while the other is busy.** `team_start` a long call (a walk), work with the other robot directly, then
   `team_wait`.

```python
reply = ctx.value("team_parallel", calls=[
    {"tool": "g1_walk", "arguments": {"forward_m": 0.5}},
    {"tool": "r1_move_joints", "arguments": {"joints": {"waist_yaw_joint": 0.3}, "secs": 2.0}},
])
for job in reply["jobs"]:          # one entry per call: state done | failed, result or error
    print(job["robot"], job["tool"], job["state"])
```

**A group is checked whole before anything moves.** Arguments, arming, each robot's own limits and the body
separation are checked for every call first; if one is refused, none starts and the error names the call.

**A group's calls start together and then run on their own clocks.** Nothing makes one robot follow the other during
the motion. Give calls that must stay in step the same duration, keep them short, and check the result before the
next step.

**`stop_all_on_failure`.** With it, when one call of the group raises, or ends early on its robot's own stop (a
result with `stopped`, or a fall), the other calls stop where they are and their robots hold. It is the default for
`coupled` groups. Without it the other calls run to their end.

## Stopping

- `team_stop` and a failed `stop_all_on_failure` group stop running motions within one control tick, the way an
  operator disarm does, and each robot holds its last command. The session stays armed; the stopped calls report
  that they were stopped.
- Calls that run inside a robot's own stack (a built-in gait such as `r1_walk`) do not stop midway. `team_stop`
  reports them as `still_running`; no robot starts a motion until they have ended.
- The operator arms and disarms both robots together. A fault that disarms one robot (a fall, a robot stepping by
  itself) stops the other. After such a disarm make no motion with either robot: `request_operator` (kind `help`).
- Each robot's own hold (`g1_hold`, `r1_hold`) works when that robot is idle.

## The team frame

Neither robot measures where it is in the room, and neither sees the other. The team frame is a ledger of each
robot's pose (x forward, y left, yaw; metres and degrees) in one frame the operator chose:

- it starts from the poses the operator measured (`team_status` shows `null` for a robot that has none);
- every walk moves the walking robot's entry by what that robot reported (the G1's leg odometry and measured turn;
  the R1's measured distance and turn), and widens its `uncertainty_m`;
- `team_set_pose` replaces an entry with your own measurement: a landmark both robots see, one robot in the other's
  camera, a known contact with the table.

`gap_m` is the body-to-body distance after both radii and both uncertainties. A walk whose path would leave less than
`min_gap_m` is refused before it starts. Treat this as a planning check, not as protection:

- it knows bodies with the arms at the sides; **it does not know arms.** Before one robot reaches into space the
  other may occupy, look (`team_observe`) and keep one robot still while the other moves there;
- it does not see steps a robot takes on its own while reaching or balancing; re-measure after such a step;
- the uncertainty grows with every walk, and a large one makes the check refuse walks that are safe. Re-measure
  (`team_set_pose`) instead of working around the refusal.

`coupled: true` on a group says these robots are meant to work within reach of each other on one object: the
separation check is skipped between them for that group, and the group stops together on a failure.

## What the team tools do not do

- **No control loop across the robots.** There is no tool that makes one robot follow the other's motion or the
  force on a shared object. A coupled group is a set of steps that start together. Plan shared work as short,
  slow, quasi-static phases with a check after each
  ([multi-robot-task](../multi-robot-task/SKILL.md)).
- **No shared perception.** Each observation is one robot's camera in that robot's own frame. To relate the two,
  use the team frame and a landmark both see.
{% if simulated %}
- **Simulation.** Ask the operator whether both robots run in one simulated world. If each runs in its own
  simulation, they share no objects and cannot touch: only what each does alone, and the order of the steps, means
  anything there.
{% endif %}
