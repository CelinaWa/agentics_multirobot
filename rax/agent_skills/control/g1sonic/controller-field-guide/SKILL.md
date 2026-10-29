---
name: controller-field-guide
description: Interface facts and practical motion limits of the frozen SONIC controller through semantic40, with references on locomotion, upper body, hands and motion primitives. Read when designing walking, turning, reaching or contact, and when a motion surprises you.
---

# Controller field guide: motion, balance and perception

See [locomotion](references/locomotion.md), [upper body](references/upper-body.md) and
[primitives](references/primitives.md) for the frozen SONIC controller through semantic40.

{{ block("g1sonic.field-guide-legend") }}

## Interface

- **Root translation comes from the legs.** Root xy `[0:2]` does not directly translate the body;
  use dynamic stepping leg references. Do not rely on root height `[2]` alone to change posture.
- **Heading is the rot6d block** `[3:9]`, relative to the initial heading; a pure yaw ψ is
  `[cos ψ, −sin ψ, sin ψ, cos ψ, 0, 0]`. The feedback reports clipped or projected commands.
- **An observation every 0.2 s** is enough to inspect an action's progress, but too coarse for
  resolving each tick of a step cycle.

## Body signals and balance

- **Tracking error** is joint RMSE in radians. Standing, walking and contact have different
  normal ranges, and walking and contact overlap. Measure the baseline for the motion you use
  before treating a rise as contact.
- **Tilt can rise rapidly.** Check both its magnitude and its trend; turns also produce tilt,
  so a fixed low threshold can interrupt normal movement.
- **Fall risks** include a blocked foot while the body moves on, turning or lowering while
  tilted, a split stance, alternating reversed turn steps, backward pelvis pitch at gait start,
  and low obstacles.

## Measured so far

Numbers from the MuJoCo sim2sim loop (gear_sonic sim, the lab office scene, 2026-10-20), each a
1.0 to 1.2 s ramp from standing followed by a hold. The real robot will differ: re-measure each
before relying on it, and keep your own table in the skill library.

| Motion (reference) | Measured | Note |
| --- | --- | --- |
| right shoulder pitch −0.8 rad | −0.70 to −0.75 rad reached | arms track to about 90 % |
| waist yaw +0.4 rad | +0.17 to +0.21 rad | the waist tracks to about half; turn the heading instead |
| heading +30° (rot6d ramp over 1.2 s) | +19° to +27°, by stepping | coarse and variable: measure the heading after every turn |
| standing hold | `joint_tracking_rmse_rad` 0.13 to 0.25 | the controller's own PD target sits off the measured joints; this is the baseline, not contact |
| standing hold | `reference_error_rmse_rad` 0.02 to 0.14 | your reference against the measured joints |
| any of the above | tilt 1° to 4°, no fall | the drop from the stand took tilt to 7° once and recovered |

## Perception and timing

- **Dead reckoning drifts.** Use visible structure to re-localize. The camera's yaw differs
  from the body's measured yaw by the waist yaw.
- **Hands, knees and feet appear in the lower image.** Account for them in object detection.
- **Allow objects to settle after reset.** Startup and the first action may also need more wall
  time than later actions.
{% if camera_geometry %}
- **Re-measure near contact.** Base motion and hand occlusion can make a remembered object pose
  stale even when the original depth estimate was accurate. Very close objects may be only partly
  visible; re-perceive after closing the hand to check whether the object moved.
{% endif %}
