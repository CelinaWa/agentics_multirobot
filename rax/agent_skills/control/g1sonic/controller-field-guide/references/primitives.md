# Field guide, primitives: reusable motion segments

Use [the field guide](../SKILL.md) to check the controller's response before composing motions.

## The library pattern

Represent each primitive as a function from parameters to 50 Hz reference frames. Keep its tested
parameter range, entry and exit conditions, measured effect and duration beside the code. Budget
time for approach, contact, recovery and settling before composing a complete sequence.

## Retargeting contact

- **Reuse your own commanded contacts.** Your action records contain
  `transition.steps[*].executed_action`: retarget a contact that held by inverse kinematics at
  keyframes, adding corrections to preserve its pre-load.
- **Check pre-load and hand timing separately.** The commanded-minus-measured offset describes
  how far the references press past a surface. The hand-command profile sets when closure starts,
  its speed and its level. Compare both with a contact that held when diagnosing a failure.

## Graded closure

Send the closing fraction directly: a hand value between 0 and 1 puts every finger joint that
fraction of the way to the fist. Ramp it over several rows to close slowly, stop where the measured
finger joints stop following (contact), and check the object's motion as you tune the profile;
close a little further than the touch to keep a grip during a lift.

## Locomotion segments

A segment of your own that produces repeatable motion can become a primitive:

- blend from the measured pose into the segment and back to a stand;
- use a yaw-only rot6d ramp toward the target heading for a walk; retain roll and pitch only
  when the motion needs them and the resulting balance has been checked;
- identify matching loop frames for walks of different lengths;
- check the model's roll and yaw sign convention before mirroring leg joints;
- compare clips with matching timing before blending their amplitudes;
- keep actions short enough to observe and correct between them;
- for obstacle crossings, verify the landing foot supports the weight before lifting the
  trailing foot, and measure a usable approach window.
