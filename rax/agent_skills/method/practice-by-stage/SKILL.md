---
name: practice-by-stage
description: Turn the task into a graph of stages, find the stage that loses the most attempts, and practise that stage from its own start state (alone, as a one-robot stand-in, or with both robots for a joint stage) until it is reliable, with scripted unattended repetitions and every try recorded. Read before planning a session and whenever whole attempts keep failing at the same stage.
---

# Practise the stage that fails, from where it starts

A whole attempt that fails at stage 3 spends the time of stages 1 and 2 to buy one try of stage 3. When one stage
fails most of the time, whole attempts mostly re-run the stages that already work: twenty attempts give twenty tries
of the easy stages and a handful of the one that matters. Practise the failing stage itself, many times, then put the
chain back together.

## 1. The stage graph (`memory/STAGES.json`)

Write the task as stages, each with the robots it needs, its start state, its success check, and what it comes after
(the format is in `api/attempts.py`). A stage is the smallest piece with a check you can measure. Typical kinds:

- **One robot, independent**: one robot's own piece that does not touch the other's (an approach, a grip on its end).
- **One robot, after the other**: it needs the other's result but runs alone (a hand-over, placing after the other
  holds).
- **Joint**: both act at once on a shared object or each other (a lift of a box both hold, carrying it in step).
  It cannot be practised by one robot; it can be practised from its own start state.

Revise the graph when a stage turns out to hide two failure modes; split it.

## 2. Record every try (`api/attempts.py`)

`record(stage, ok, mode=..., cause=..., params=...)` after every try of every stage, inside whole attempts too
(`chain`), on its own (`practice`) or as a stand-in (`proxy`). `python -m api.attempts` is the funnel across all
sessions, with the operator's ground truth per session where the simulation provides it. A stage's rate counts only
tries that reached it; label failures by the first stage that failed. `practice` means the try started from that
stage's own start state; every stage inside a run from a reset is `chain`, even when the run was a practice run.
Label the outcome with what really happened: where the simulation's operator reports ground truth (its reset reply,
`runtime/stream/ground_truth.jsonl`) [sim-only], use it over your camera's impression, and note where they disagreed;
that disagreement is a perception error the real robot will have too.

## 3. Choose what to practise

From the funnel: the stage with the most lost tries per try that reached it, weighted by how much of the chain
depends on it, is the bottleneck. Practise it until its rate is good enough for the chain (rule of thumb: the chain's
success is roughly the product of its stages' rates; five stages at 0.9 give 0.6, one stage at 0.3 caps everything at
0.3) or until you have evidence that the stage needs a different method or a different tool.

## 4. Practise from the stage's start state

Ask the operator to set the scene up at the stage's start state (`request_operator` kind `reset`, saying exactly what
state: where the object, where each robot). What the operator can set up differs:

- In simulation the operator's resets are cheap and exact: `TASK.md` (the operator section) says what it can set up
  (objects back at their start, robots carried to their starting places or to a stance you name). Use as many as the
  practice needs.
- With real robots a person sets the scene up by hand, which is slower and less exact: fewer repetitions, and each one
  logged with how the start state really looked.

Ways to practise each kind of stage:

- **One-robot stage**: that robot alone, the other parked clear. Vary the start state on purpose (object offset,
  stance error of the size your approach really produces) so the practised skill covers what the chain will hand it.
- **Proxy for a joint stage**: one robot does its share against a fixed stand-in for the other (for a two-robot box
  lift: the R1 lifts its end a few centimetres while the far end stays on the table, and must keep its grip). A proxy
  that passes is necessary, not sufficient: confirm in the joint stage.
- **Joint stage**: both robots, from the joint stage's start state (both already holding, if that is how it starts).
  This is the only way to learn coordination (timing, who leads, how the load shifts); do not replace it with
  one-robot practice, but do not reach it through a failing upstream stage either: set up its start state.

## 5. Repeat without the model in the loop

Write the stage as a script under `code/` that sets up (or asks for) its start state, runs the stage, checks the result
from fresh measurements, records it with `api.attempts.record(..., mode='practice')`, and puts things back for the next
repetition. Run it for N repetitions (a loop with a stop on any safety signal: a fall warning, a runaway, a tilt, a
lost grip under load). Look at the funnel and at the frames of failures, not at every repetition. Change one thing
between batches.

## 6. Put the chain back together

When the bottleneck stage is reliable, run whole attempts again: the funnel then shows the next bottleneck, or the
seams between stages (a hand-over state the practice start state did not match). A stage that keeps failing for a
reason the tools cannot fix (a walk that cannot place the robot to the accuracy the stage needs) goes to
`memory/TOOL_REQUESTS.md` with the numbers: that is a finding, not a failure to try harder.
