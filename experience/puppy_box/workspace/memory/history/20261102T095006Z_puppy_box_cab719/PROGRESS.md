# PROGRESS - session 20261102T095006Z_puppy_box_cab719 (sim)

Layout: puppy centre (head LEFT, legs to robot), clear box RIGHT (x .23-.54, y -.17..-.56 at run 1) -> library
plush-pick-place-box-same-side (now v4, merged 94691ea).

## Results: 2 placements in 4 attempts (operator confirmed INSIDE for runs 1 and 3)
- run 1 SUCCESS: stance (3 walks + 2 crouch restores, edge .278 -> .221), skill steps with looks at hover and lift.
- run 2 FAIL: chained without looks; hover over the neck, grip lost on the lift (z .22). Then skill home (sp .5 el .6) at a
  0.20 stance -> robot stepped 0.17 m by itself, disarmed + auto re-armed (I noticed late: check g1_status events after every step).
- run 3 SUCCESS: robot re-squared by in-place turns (heading had crept +12 deg left), box out of view -> inferred from run-1
  offset, grasp from a tucked view, release (.37,-.40) by IK reach, carry finished by mv.move after a grip flag, home_low.
- run 4 STOPPED safely: box out of view again -> bogus box -> hover refused; lean fallback refused; home.
## End state (10:35): robot standing still, arms down beside hips, hands open, tilt ~6 (steady), not touching the table; puppy ON THE TABLE (not in the box), box upright. Resets used: 3 of 5. Runs 4-5 stopped safely (box detection).
Resets used: 3 of 5.
## Code
- code/puppy_box/run_cab.py (stance + library chain), code/puppy_box/home_low.py, code/puppy_box/edgefit.py
- library: skill-library/skills/plush-pick-place-box-same-side/code/{run.py,stance.py,controller3.py}
## Open issues
- perceive's box detection has no plausibility check; when the box is out of view it returns garbage (hover then refuses).
- base yaw creeps left with arm work/crouch; robot also drifted ~8 cm left relative to the puppy after the self-step.
- left-hand contact never confirmed by the operator (generic replies).
