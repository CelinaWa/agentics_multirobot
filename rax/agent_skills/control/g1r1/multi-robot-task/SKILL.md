---
name: multi-robot-task
description: How to do one task with two robots - splitting it into subtasks, naming how the subtasks depend on each other (independent, sequential, shared space, coupled), exploring each kind, chaining them into the whole task, and reading a failure by subtask, by robot and by cause. Read before planning the task and after every whole-task attempt.
---

# One task, two robots

A task for two robots fails in more places than a task for one: in a robot's own motion, where one robot hands the
scene to the other, and where the two must agree in time or space. The method below keeps those apart, so each can be
practised alone and a failure of the whole task can be traced to one of them.

## 1. Split the task and write the subtasks down

Keep one table in `memory/SUBTASKS.md` and update it after every attempt. One row per subtask:

| Field | What to write |
|---|---|
| id, robot(s) | a short name; which robot does it, or both |
| depends on | the subtasks that must have succeeded first, and the **kind** of dependence (below) |
| starts from | what must be true in the scene and of the robot(s) before it starts, as something you can measure |
| success check | the measured quantity and threshold that says it worked (not "looks done") |
| explored from | the range of starting states you have actually tried it from |
| ends in | the states it actually ended in when it succeeded (measured, with their spread) |
| count | successes / attempts{% if simulated %}, in simulation{% endif %}; and how each failure showed |

"Starts from", "ends in" and the success check are what make subtasks chainable: a subtask can follow another only
if what the first **ends in** lies inside what the second was **explored from**.

Give each subtask to the robot that does it more reliably, measured, not assumed. Where you cannot tell, try the
subtask with each robot before fixing the split. The split itself is something to explore: a different order, a
different robot, or one robot steadying an object while the other works on it may be far more reliable than the
first plan.

## 2. The four kinds of dependence

| Kind | Meaning | How to run it |
|---|---|---|
| **independent** | the subtasks touch different objects and different space | at the same time: `team_parallel`, or `team_start` and work with the other robot |
| **sequential** | one subtask leaves the scene the way the next needs it | one after the other, with the handover check between them |
| **shared space** | different objects, but the robots' bodies or arms pass through the same space | one robot in that space at a time; the other parked clear of it and still |
| **coupled** | both robots hold or load the same object at the same time | `coupled` groups, in short phases that start together, with a check after each |

## 3. Explore each kind in its own way

**Independent.** Both robots work from the start: while you practise one subtask with one robot, the other robot
practises its own (`team_start` and carry on, or `team_parallel`). A robot that stands still is robot time not used;
the reset reply's `robot_motion_min` shows how the time has been shared. If the counts drop when both work at once,
the subtasks were not independent (a shared table that shakes, a shadow in the other camera, a body in the way).

**Sequential.** While practising the first subtask, record what it ends in. Practise the second from those states,
not from a tidy state you set up by hand. Write the handover check (the second subtask's "starts from", measured
after the first) and run it every time; a failed check is a finding about the first subtask, and the second need not
be attempted.

**Shared space.** Decide who is in the space when, and where the other waits. Keep the team frame current
([robot-interface](../robot-interface/SKILL.md)): correct it from a landmark before a robot walks toward the other,
and look before an arm reaches where the other robot may be.

**Coupled.** Agree the interface before any motion: where each robot holds the object, at what height, who leads,
and the order and duration of the phases. Then:

1. Practise each robot's part alone against a stand-in for the other: the object resting on a support at the
   other end, or the operator holding it (`request_operator`). Vary the stand-in (height, timing, a few centimetres
   of offset), or each robot learns a partner that does not exist.
2. Put the two parts together and compare with the stand-in counts. A small drop: keep practising the parts apart.
   A large drop: the parts depend on each other more than the interface says; practise together from here on.
3. Run every coupled step as a `coupled` group, so a failure of one robot stops the other, and check the object
   after each phase (is it still held at both ends, how far has it tilted or slipped) before the next.

The team tools start a group's calls together but do not keep the robots in step during a motion, and neither robot
follows the force on the object. So give coupled phases that must stay in step the same duration, and check the object after each.

## 4. Chain the subtasks into the whole task

When each subtask is reliable alone, run the whole task, with the success check after every subtask and the
handover check before the next. A failed check stops the attempt at that subtask: retry the subtask when its failure
leaves the scene in a state it was explored from, otherwise ask for a reset. A retry that is checked costs little
and recovers a lot (a subtask that succeeds half the time succeeds seven times in eight within three tries), but
only if the failure is detected and the scene is still usable; a subtask that fails nine times in ten has to be
fixed, not retried.

Account for every whole-task attempt in the funnel ([prioritize-by-funnel](../prioritize-by-funnel/SKILL.md)), with
the subtasks as its stages.

## 5. Read each failure three ways

For the first failed subtask of an attempt, record:

- **Which subtask**, and whether its handover check had passed.
- **Which cause:**

  | Cause | Sign | What to change |
  |---|---|---|
  | the subtask itself | it started inside what it was explored from and still failed | practise that subtask |
  | handover | the subtask before succeeded by its own check, but ended outside what this one was explored from | widen what this one is explored from, tighten where the earlier one ends, add a step between them, or merge the two into one subtask |
  | coordination | wrong order or timing, a robot in the other's way, a refusal for separation, both waiting | the order, who waits where, the team frame, the group's phases |
  | the plan | the split or the choice of robot cannot work as designed | another split, another robot, another order |
  | perception | the check or the measurement was wrong, not the motion | the measurement; add a second view or the other robot's view |
{% if simulated %}
  | simulation | it depends on something the simulation does not model the way the room does | note it for the real robots ([sim-to-real](../sim-to-real/SKILL.md)) |
{% endif %}

- **Which robot**, for a coupled subtask. Repeat the failed phase with one robot replaced by the stand-in: if the
  failure stays, it is the remaining robot's part; if it goes, it is the other's, or the interface.

Fix what loses the most attempts first. After changing a subtask, rerun the whole task, not only that subtask
([evidence-and-regression](../evidence-and-regression/SKILL.md)): a change that helps one subtask often moves where
it ends, and the next subtask pays for it.
