# Field guide, locomotion: walking, turning and obstacles

Use [the field guide](../SKILL.md) with current observations to characterize each motion.

## Walking

- **Use `g1_walk` to travel.** It runs SONIC's own locomotion planner (the gait a teleoperator's joystick
  drives): give a forward distance, a turn (at most 45 deg per call; a 60 deg turn overshot to 72 and ended mid-step) and a slow-walk speed, and the robot
  steps there and stands. It walks forward only: turn first to go elsewhere. Measured in sim (2026-10-20, after the turn
  fix): 45 deg commands turned 38-41 deg; forward 0.4 m at 0.2 m/s covered about 0.2 m, 0.3 m at 0.3 m/s
  covered 0.37 m: prefer 0.3 m/s and re-localize. Eight walks in a row with tilt under 6.2 deg. Distances are commanded, not measured: re-localize against the scene after every walk. Measured
  in sim (2026-10-20): 0.6 m forward and a 45 deg turn executed with tilt under 5 deg; a 45 deg command
  turned about 33 deg.
- **Joint-reference gaits are for fine positioning.** Through `g1_step` you can also step by streaming
  leg joint references (see your skill library); use that for short corrections near furniture.
- **Walking needs dynamic stepping leg references.**
  Build a repeatable walk cycle from your commanded frames and measured motion.
- **Small commands may not produce small steps.** Quasi-static or bent-knee motions may leave the
  body in place. Characterize the smallest dynamic stride, end the last walk within tolerance,
  and use the arms or torso for the remaining correction.
- **Starts and stops have transients.** Blend from the measured pose into the cycle and back to
  a stand. Allow for coasting and for the old direction to persist briefly after a reversal.
- **Asymmetric cycles can crab sideways.** Check forward and lateral movement against the scene.
- **Pelvis pitch affects balance at gait start.** Check whether a yaw-only root orientation is
  sufficient before carrying roll and pitch from a recorded gait into a new motion.

## Turning

- **Heading is a closed-loop target.** Allow for lag, saturation and a minimum effective command;
  measure the resulting heading rather than adding commanded angles.
- **Turning in place depends on rate and amplitude.** A small, slow rot6d ramp may twist the hips
  without making the feet step. Characterize a range before choosing a turning primitive.
- **Turning can translate the body.** Re-localize afterwards, and allow clearance for the body's
  sideways swing while turning during a walk.
- **Sidesteps and backward walking need their own characterization.** Check direction, distance
  and balance before using them for precise positioning.

## Obstacles

- **Check foot clearance and weight transfer.** Raised steps and low obstacles can block a foot
  or destabilize a gait. Prefer a clear route when one exists; characterize an unavoidable
  crossing separately before composing it into a longer controller.
