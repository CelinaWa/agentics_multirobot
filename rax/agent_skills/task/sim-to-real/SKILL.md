---
name: sim-to-real
description: How experience is written so it moves between simulation and the real {{ robot.name }} — kind tags on tips, measured numbers per environment in memory/PARAMS.json via api/params.py, what an operator may tell you, and the first-session protocol after a change of environment. Read at the start of every session that inherits experience, and before writing TIPS.md or the handoff.
---

# Sim to real: one way of writing what you learn

This session runs on **{{ environment }}**{% if simulated %} (a simulation of the {{ robot.name }}){% else %} (the physical {{ robot.name }}){% endif %}.
Sessions in simulation and on the real robot use the same tools, the same memory files and the same code,
so experience flows both ways: simulation explores, the real robot confirms and corrects, and the
simulation is re-tuned to what the real robot measured. That only works if what you write says which
kind of knowledge it is.

## Three kinds of knowledge, three tags

Every row you add to `memory/TIPS.md` and `handoff/README.md` starts with one tag:

| Tag | What it is | Example | On the other environment |
|---|---|---|---|
| `[strategy]` | what to do, in what order, and why it works; failure signatures | "grasp the head-side half; a pinch at the reach limit closes to a fist" | a rule until a measurement contradicts it |
| `[param]` | a measured number the plan depends on; its value goes in PARAMS.json | "hands reach 0.26 m ahead of the pelvis at table height (`reach.forward_m`)" | a prior: measure again before relying on it |
| `[sim-only]` | true only because of the simulation: its scripted operator, contact model, scene builder, resets | "the operator's reset puts the puppy back at its spawn" | removed from a real session's memory |

A strategy row may cite numbers as evidence ("4 of 4 closed to a fist"); a number the controller
uses is a `[param]` and has a PARAMS.json name. When unsure between `[strategy]` and `[sim-only]`, ask:
would a person standing next to the real robot see the same thing? If it depends on how the simulator
models friction, softness, stiffness or the operator, it is `[sim-only]` or at most a `[param]`.
Keep `where: sim` / `where: real` and `held on <sim|real> <date>` as the
[inherited-experience](../inherited-experience/SKILL.md) skill says.

## Numbers live in memory/PARAMS.json, read through api/params.py

Do not hard-code measured numbers in `code/`. Read them by name:

```python
from api.params import param, record
reach = param('reach.forward_m', default=0.25)
edge = measure_table_edge()            # your perception
record('table.edge_from_pelvis_m', edge, unit='m', how='depth, hands tucked')
```

`param()` returns this environment's value; without one it returns the other environment's value and
prints that it did, so a sim number used on the real robot is always visible in the log. For a number
a committal action depends on (a grasp height, a release point, a walk distance), call
`param(name, verified=True)`: it raises until the number was measured here. Names: dotted, unit last
(`grasp.close_height_m`, `walk.forward_per_m_cmd`, `hand.dex3_close_ms`). `python -m api.params` lists
them all and which still need measuring here.

## What an operator may tell you

The real operator can arm, reset, hold the gantry, put objects back roughly where they were and say
what they see. They cannot read positions to the centimetre, see contact forces or know the robot's
pose. The simulation's operator is held to the same contract: if a reply in simulation contains a
measurement a person could not make, do not build a strategy on it and tag what depends on it
`[sim-only]`. Plan every step so that your own observation provides the numbers.

## Starting on a new environment

`memory/TRANSFER.md` exists when this session's experience came from the other environment. Then:

1. Read `memory/TRANSFER.md`, `handoff/README.md`, the `[strategy]` rows of `memory/TIPS.md`.
2. List the `[param]` names the first subtask needs (`python -m api.params`). For each, find the
   cheapest measurement that does not commit: observe (depth, frames), then a hover or an empty-hand
   reach, then a slow partial motion. Record each with `record()` and tick it in TRANSFER.md.
3. Only then run the subtask; when a strategy fails here, write what differed under its row, do not
   delete it.
{% if not simulated %}
4. Robot-level numbers come first, they are shared by every task: how far the hand reaches at table
   height, whether the hand holds the object when closed and lifted 5 cm, how far a short walk really
   moves the robot (`feedback.odometry_m` against a mark on the floor the operator can confirm), how
   your edge and object detection reads with the real camera. Record them under `robot.` / `walk.` /
   `hand.` names so the next task inherits them too.
{% else %}
4. In simulation, explore for strategies that do not depend on a lucky number: vary what the scene
   randomizes, keep the controller reading its numbers from PARAMS.json, and record the spread you saw
   (`note`: "0.24-0.28 over 6 seeds") so the real session knows how hard to check.
{% endif %}

## Before the session ends

- Every row you added this session carries a tag; every number your code uses has a PARAMS.json
  name with this environment's entry.
- The handoff names, per subtask, which `[param]` the next environment must measure first.
