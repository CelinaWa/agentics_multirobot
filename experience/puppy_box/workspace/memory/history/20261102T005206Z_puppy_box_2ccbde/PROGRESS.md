# PROGRESS puppy_box (session 20261102T005206Z, sim rehearsal) - ENDED, task NOT achieved

## Final state
- Robot standing still, both hands open (default shape restored), arms back by the body (sp ~0.6-0.77, el 0.6),
  waist 0, tilt ~1.9 deg. Box untouched on the table. PUPPY ON THE FLOOR (operator confirmed; needs a reset).
- Resets used: 2 of 5. Operator help requests: 3.

## Layout (measured this session)
- Table top z ~ -0.045 (pelvis), front edge ~x 0.15-0.17. Clear box x 0.36-0.65, y ~-0.02..0.40, rim z ~+0.10
  (sim depth sees a plane at rim height; ray/plane hits of the two faint rectangles agree; percep.draw_box overlay).
- Puppy start: right side, torso ~(0.34,-0.21); after each operator reset ~(0.23,-0.23) (closer, near the edge).

## What worked / failed (details: memory/RUNS.md, skill-library plush-pinch-grasp)
- Default open shape pinch: 2/2 slip at 3 cm lift.
- PRE-CURLED pinch (open [0,±0.4,0,±0.5,0,±0.5,0]), palm pressed to stall: lifts the puppy (4/5 grasps), but every
  transport dropped it (fast lift, arm extending across body, arm at reach limit x 0.41-0.44, lean+reach).
- Reach: right arm cannot reach into the box (it is on the left); left arm palm-down reaches x <= ~0.44 at z 0.26,
  only 4-8 cm past the box front wall; box centre is at x ~0.50. The puppy hangs with legs toward the robot, so
  its legs stay outside the front wall at the reach limit.
- Operator replies were canned: "The dog is INSIDE the box" twice while the camera showed it on the table in front
  of the box - do not trust that sentence; "dog on the FLOOR" was consistent with the image.

## Next session should
- Solve REACH first: a stance closer to the box (ask the operator whether walking is allowed; the first g1_walk
  does a warm-up turn that steps 0.1-0.2 m forward - table edge only 0.15 m away), or place the puppy over the
  box's near half with a release right after the body clears the wall, then nudge from above.
- Keep carries short: grasp the puppy where it lies closest to the box; move in 3 s steps; stop/release on grip creep.
