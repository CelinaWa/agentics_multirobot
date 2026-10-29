---
name: engineer-the-controller
description: Engineer the controller so it runs on the robot without you fixing it step by step - a main loop that never crashes or hangs, steps with preconditions and verified results, progress measures, stall detectors, timeouts and an escalation ladder, explicit state, and offline and fault tests. Read when structuring controller.py and before running a version on the robot.
---

# Engineer the controller: steps, escalation, state and tests

Once a procedure works by hand, your code under `code/<project>/` runs it while you and the
operator watch. Every step it handles alone is time you keep; every step that needs you is one the
next session has to work out again. Most failed runs come from engineering faults, not from the
task: a crash in a branch that never ran, a debug switch left on, a controller that kept trying in
a way that could not succeed, bookkeeping that a retry never reset.

## The main loop

1. **Never crash and never hang.** Handle each step inside `try`/`except`: a tool error raises
   `RuntimeError`. On an exception, log the traceback, send a safe frame (hold the last one), re-perceive
   and continue or return. `main` returning ends its run where the robot stands, so it returns only in a
   pose that is safe to hold: arms in, nothing mid-step, nothing pressed against a surface.
2. **Read every reply.** `transition.feedback` (tracking error, tilt), the measured joints and the
   refusal reasons decide the next step; a refusal, a stop or a disarm means stop calling motion tools
   and report (`request_operator`), never retry blindly.
3. **Respect the limits.** At most 100 frames per call; every call ends in a pose to hold; start every
   call from the measured joints.
4. **Log one line per decision** to stdout or a file under `output/`: the controller version, the
   step, the key measurements, the action, and the escalation rung if any.
5. **Keep each decision short;** precompute what you can at start-up. A robot standing still with an
   arm out while you think is a robot drifting.
6. **Keep decisions apart from I/O:** a function from (observation, state) to (action, state), so an
   offline replay can call it on archived observations
   ([evidence-and-regression](../evidence-and-regression/SKILL.md)).

## Steps

Write the controller as a pipeline of named steps, one per phase. Each step:

- **checks its preconditions from fresh observations**, never from what an earlier step believed;
- **acts, then verifies** the result with a measurement;
- **returns `{ok, measured, why}`:** whether it succeeded, the quantities it measured, and on
  failure the reason;
- **returns instead of improvising** when the situation lies outside the range in which it was
  tested (a target farther, more rotated or in a configuration never seen): it escalates rather
  than extrapolates;
- **can be the entry point:** the pipeline resumes at any step whose preconditions hold, so a
  recovery re-enters the step that restores a lost condition.

**Tunables are named constants** at the top of one module, each annotated with the range in which
it was tested and the evidence, for example `STOP_DISTANCE_M = 0.42  # tested 0.38-0.46, 7/8,
attempts 3-11`. A value needed outside its tested range is a reason to escalate.

## Progress, stalls, timeouts and escalation

For every step:

1. **A progress measure from perception:** the quantity that must improve, measured from the scene
   (a distance, a height, an angle, a gap) and logged every decision. Never let dead reckoning
   count commanded motion as progress while the scene does not move: this robot's walks and turns
   deliver less than commanded, and its pelvis shifts when an arm reaches.
2. **A stall detector:** compare the motion you predicted (from your measured primitive table,
   [{{ robot.tracking_skill }}](../{{ robot.tracking_skill }}/SKILL.md)) with the motion you measured. Measured
   motion well below the prediction for two or three actions means the robot is blocked or slipping.
3. **A timeout** in time and actions, from the step's measured duration in successful attempts
   (two to three times the median).
4. **An escalation ladder** of actions that each change something: re-perceive and re-plan; change
   the approach geometry (side, offset, angle, stance, posture); back off until the target is fully
   in view; switch primitive or method; return to an earlier step that restores the preconditions;
   and at the top of the ladder, hold and `request_operator`. Never repeat the identical action
   that just failed, and never let a timeout lead back into the step it guarded.
5. **A refused action needs an alternative.** When a tool or a safety rule of your own refuses an
   action, choose another action that changes the situation; waiting for the same action to become
   allowed is not one.

Across steps, count entries into each step and revisits of similar states; when a cycle repeats
(three times), force the next rung or another branch. Keep a watchdog on what perception reports
(the robot's position relative to the target and the objects): if it has not changed for K
decisions, force a different branch, whatever the state machine believes. Re-entering a step can
reset its counters; this watchdog cannot.

## State

1. **Write the transition table** in `code/<project>/` and at the top of the main file: every step,
   its exits (success, failure, timeout) and the next step for each. No step may wait for something
   that can no longer happen.
2. **Entering a step starts a new attempt:** reset its histories, counters, references and timers
   on entry. Keep what must survive (how often the ladder ran) in separately named variables, so
   that re-entry cannot reset the counters meant to end it.
3. **Flags that relax a check expire** after a time, a step change or an event; one left set for
   the rest of the run lets an estimate drift unchecked.
4. **While anything is held, check in every step that it still is,** with two independent signals
   (a perceived object that moves with the hand, and a body signal such as the arm's tracking error
   under load or fingers that stopped short of the commanded closure), and go to a recovery step
   when it is not. A single signal is easily fooled.
5. **No dead values.** Every computed quantity is used in a decision or removed; a logged but unused
   measurement is a missing check.

## Tests

- **Transitions offline:** feed recorded observations and synthetic perception results to the
  decision function and assert the next step and action for every exit, timeouts and recoveries
  included.
- **Liveness offline:** freeze the world (the same observation for 50 decisions) and assert that the
  controller leaves every step within its timeout and never repeats one action forever.
- **Faults, by hand:** switches (an environment variable or a flag file, off by default) that
  simulate a missed detection, a lost object, a failed check or a blocked path, and attempts where
  the operator deliberately creates them (hides the object, moves it after the grasp). Run every
  escalation rung and recovery at least once and count the outcomes.

## Before the session ends

- **Switches** default to what the next session runs: no environment variable and no file edited by
  hand outside `code/`.
- **Run what you keep.** The version in `code/<project>/` is the one that ran last on the robot; know
  which changes never ran live, and say so in its README and in the skill library entry
  ([continual-learning](../continual-learning/SKILL.md)).
- **A checklist** in `code/<project>/` helps whoever runs it next: for each step its preconditions,
  progress measure, stall detection, timeout and escalation rungs; for each irreversible action its
  preconditions and where the code checks them; each with the function that implements it.
