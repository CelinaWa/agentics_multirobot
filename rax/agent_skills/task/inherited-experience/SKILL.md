---
name: inherited-experience
description: Use what an earlier session on this task left in this workspace (handoff/, memory/, code/ and the skill library), typically from simulation or from a differently laid-out scene; read when handoff/README.md or memory/history/ exists at the start.
---

# Inherited experience

The operator started this session from an earlier one on the same task (`--experience`). That
session may have run **in simulation** on a model of this robot, or on this robot in another
layout. Its knowledge is in your workspace; its scene, calibration and numbers are not yours.

| What you find | Written by the earlier session as | Use it as |
|---|---|---|
| `handoff/README.md` | how to plan each subtask in a differently laid-out scene | your first plan, subtask by subtask |
| `memory/TIPS.md` | condition → action → measured result rows | candidate rules, each to be confirmed here |
| `memory/RUNS.md`, `memory/INDEX.md`, `memory/skills/` | its attempts and distilled procedures | evidence and procedures; keep adding to the same files |
| `memory/history/<session>/PROGRESS.md` | its running state | history only: nothing in it is finished work of yours |
| `code/` | controllers it wrote | code to read and adapt; it was tuned to that scene |
| `skill-library/` | skills merged by earlier sessions (`INDEX.md`, status `draft` / `verified`) | per [continual-learning](../continual-learning/SKILL.md): check preconditions, run, confirm |

## Order of work at the start

1. `TASK.md`, then `memory/TRANSFER.md` if present, then `handoff/README.md`, then `memory/TIPS.md` and `memory/INDEX.md`, then `skill-library/INDEX.md`.
   Do not read `memory/history/` unless a tip or the handoff refers to it.
2. Observe before planning. For every inherited position, distance or height, find the
   corresponding feature in your own observation and measure it; the inherited number is a
   prior, your measurement is the value.
3. Start from the step the handoff recommends next, not from the beginning, unless your
   first observation contradicts its preconditions.

## What transfers and what does not

Transfers (treat as rules until contradicted): which approach directions failed and why,
contact and grasp strategies, the order of subtasks, how to recognise a failed grasp, what
the operator was needed for.

Does not transfer (measure again): absolute coordinates, pixel positions, table and object
heights, joint tracking offsets, hand stiffness and how much a soft object compresses,
walking distances per command. A simulated hand and a real hand close differently; a
simulated table is where the model put it.

The earlier session tagged its rows `[strategy]`, `[param]` or `[sim-only]` and kept its numbers in
`memory/PARAMS.json` ([sim-to-real](../sim-to-real/SKILL.md)). When it ran in the other environment,
the launcher already moved its `[sim-only]` rows to `memory/history/<session>/SIM_ONLY.md` and listed
the numbers to measure in `memory/TRANSFER.md`; work that list first. Untagged rows predate the tags:
read their numbers as `[param]`.

## Bookkeeping

- When an inherited tip holds here, append to its row in `memory/TIPS.md`: `held on <sim|real> <date>`.
  When it does not, write what differed on a new line under it; do not delete it.
- Give new rows a kind tag and `where: sim` or `where: real` so the next session knows the source.
- A number you measured here goes into `memory/PARAMS.json` with `api.params.record()`; the
  other environment's entry stays next to it.
- A library skill from simulation keeps status `draft` until it has run unattended here;
  the merge tool enforces this, your evidence rows make it possible.
- Before the session ends, update `handoff/README.md` for the next scene as
  [transfer-handoff](../transfer-handoff/SKILL.md) says, including which inherited
  items held and which did not.
