---
name: transfer-handoff
description: Write handoff/README.md before the session ends, describing how to plan the motion for each subtask and each object manipulation, as notes for a later session on this task in a scene laid out differently. Read in the last part of every session and whenever a subtask first works.
---

# Transfer handoff

`handoff/README.md` holds notes for a later session on this same task in a scene laid out
differently (objects elsewhere, another table, another room), which starts from `handoff/`, the
skill library and `memory/`, not from your frames or logs. Write in it how to plan the motion for
each subtask and each object manipulation of this task, so that session can plan them in the new
scene instead of working them out again. The skill library
([continual-learning](../continual-learning/SKILL.md)) keeps the executable procedures; the
handoff keeps the reasoning around them: how to decide, what to measure, where it went wrong.

## What to write

Start with the subtasks in order, one line each, with what marks each one complete in
the current observation and the `TASK.md` condition it serves.

Then give each subtask a section, with one entry per object manipulation in it (for
example reach, grasp, lift, carry, place, push or release):

- **Object and goal**: which object, what must change, and how to see that it did.
- **Locating it**: how to find the object and the target in a fresh observation (which
  camera, which visible feature, which skill or tool), and what to locate again after
  each move.
- **Motion plan**: the ordered moves, each relative to something observed (for
  example "\<distance> above \<observed feature>, \<orientation>"), with the arm, the tool
  that plans or executes it (`{{ robot.move }}` or `{{ robot.move_joints }}` frames), the
  stance and heading, the hand command, the approach and retreat directions, and the
  step sizes that worked.
- **Parameters**: each value with its unit, what it is measured from, the range you
  tried and which value worked.
- **Checks and failures**: how to confirm the manipulation worked, the failures you
  saw, how to recognize each and what fixed it; the incidents (near-falls, stops) and
  where they happened.
- **Code**: the function under `code/` or the skill library entry that performs it and how to
  call it, if any.
- **The operator**: what you needed them for (resets, confirmations, measurements) and what a
  scene has to look like before you start.

Write what you verified and mark anything untested. Give positions relative to
observed features, never absolute coordinates from one scene. Keep it as text; copy
at most a few decisive images into `handoff/`.

Update `handoff/README.md` when you learn something new, and keep it short: text, plus
at most a few images.
