---
name: in-context-action-learning
{% if demonstration_kind == 'terminal_state' %}
description: Infer the task goal on the {{ robot.name }} from the operator's terminal-state {{ 'image' if one_demonstration_image else 'images' }} of the completed task. Use when visual examples should guide the current task.
{% elif demonstration_kind == 'completion_sequence' %}
description: Infer the task goal and useful strategy on the {{ robot.name }} from the operator's image sequence of a person or robot completing the task. Use when visual examples should guide the current task.
{% else %}
description: Infer the task goal and useful strategy on the {{ robot.name }} from a supplied image sequence of a person or robot completing the task, or from a supplied terminal-state image. Use when visual examples should guide the current task.
{% endif %}
---

# In-context action learning

{% if demonstration_kind == 'terminal_state' %}
{% if one_demonstration_image %}
The operator supplied one image of the completed task, under `runtime/demonstrations/`. It
contains no actions, robot state, poses, calibration, or control timing. Use it for visual
intent, never as executable motion data.
{% else %}
The operator supplied images of the completed task, under `runtime/demonstrations/`. They
contain no actions, robot state, poses, calibration, or control timing. Use them for visual
intent, never as executable motion data.
{% endif %}
{% elif demonstration_kind == 'completion_sequence' %}
The operator supplied an ordered image sequence of a person or robot completing the task,
under `runtime/demonstrations/`. It contains no actions, robot state, poses, calibration, or
control timing. Use it for visual intent and strategy, never as executable motion data.
{% else %}
Supplied demonstrations under `runtime/demonstrations/` contain images only: either an ordered
sequence of a person or robot completing the task, or one terminal-state image. They contain
no actions, robot state, poses, calibration, or control timing. Use them for visual intent
and strategy, never as executable motion data.
{% endif %}

The demonstration was taken in a scene that may differ from the one in front of the robot now
(objects elsewhere, another viewpoint, a person's hand instead of the robot's). Use it mainly to
understand what the task requires and the intended final relationships, not to copy exact positions
or reproduce the image pixel-for-pixel. Satisfy every condition in `TASK.md` using the current
scene and observations.
{% if demonstration_kind != 'terminal_state' %}
Use a sequence to understand the task phases, but do not imitate every demonstrated action. A
person's hand can do what the Dex3 hand cannot; choose simpler or more convenient actions when
they achieve the same required outcome.
{% endif %}

## Retrieve

- Retrieve matching tips from this task or learned experience for similar actions, then inspect
  the supplied {{ 'image' if one_demonstration_image else 'images' }} under `runtime/demonstrations/`.
{% if demonstration_kind != 'terminal_state' %}
- View the images directly. Preserve a sequence's supplied order and identify visible phases,
  contacts, object relations, and the final effect. Do not infer exact timing or interpolate
  missing phases.
{% endif %}
{% if demonstration_kind != 'completion_sequence' %}
- For a terminal image, infer only the desired visible end relationship; devise the intermediate
  robot behavior independently.
{% endif %}

## Adapt

- Transfer semantic structure, such as approach side, contact order, grasp region, relative
  placement, and completion appearance. A human hand or a different robot does not define the
  current arm, pose, or trajectory.
- Ground all geometry again from current observations. Use
  [object-localization](../object-localization/SKILL.md) and the motion tools in
  [robot-interface](../robot-interface/SKILL.md) when needed; never estimate metric targets or
  action magnitudes from demonstration pixels alone.
- Construct new actions with the robot tools, as the live tool schemas define them.

## Update from experience

- Associate the chosen visual strategy with its observed effect; do not repeat an ineffective
  interpretation from the same conditions.
- When starting a new attempt, retrieve the failed attempt's decisive before/action/after evidence
  and revise the plan.
- As you build the controller, turn relevant evidence into executable logic that reacquires
  current geometry rather than assuming the demonstration's scene is identical.
- Keep this working set compact. Promote only reusable, evidence-backed results through the
  experience-memory skill, and record demonstration-derived tips under `demonstration_provenance`
  in the experience-memory format.
