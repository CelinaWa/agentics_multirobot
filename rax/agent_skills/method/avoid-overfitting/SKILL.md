---
name: avoid-overfitting
description: Warning signs that code or notes fit the scene you happened to see (encoded poses, scene-specific fallbacks, constants tuned on one attempt, replayed trajectories) and what transferred instead. Read when reviewing your controller and before relying on a constant.
---

# Avoid overfitting to the scene you saw

The real scene changes between attempts and between sessions: the operator puts objects back by
hand, never to the millimetre; the table is bumped; another day the room is laid out differently.
Code that works only where the objects happened to be during one attempt does nothing for you at
the next, however well it worked before.

## Warning signs in code and notes

- **Encoded poses.** Hard-coded positions, distances, headings, pixel coordinates or joint targets
  that record where an object was in one attempt.
- **Fallbacks that assume a scene.** "If the target is not found, walk at heading X", or "after
  step Y the target is straight ahead". When perception fails, a search
  ([object-localization](../object-localization/SKILL.md)) transfers; a remembered pose does not.
- **Constants tuned on one attempt** and never checked across the range TASK.md allows (the object
  anywhere on the table, the bin anywhere in its corner).
- **Replayed trajectories.** A recorded motion replayed as it is only fits the scene it was
  recorded in. As a primitive whose effect you measure and re-aim, it can transfer.
- **Inherited assumptions.** Code or notes from earlier attempts carry assumptions about where
  things are; listing each phase's assumptions and checking them against TASK.md exposes them.

## What has worked

- Perceiving the actual pose in each attempt, acting relative to that measurement, and re-measuring
  after each move.
- Writing down, for each constant, what it encodes and over which range it has to hold; quantities
  measured online generalize better than fixed numbers.
- Walking through the extremes TASK.md allows (nearest, farthest, most rotated, objects at opposite
  ends of their regions), asking what each phase does there, and asking the operator to set up the
  extremes that reasoning cannot settle.
