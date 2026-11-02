# PROGRESS - session 20261102T115009Z_puppy_box_eed312 (sim)

Layout: puppy ahead-right (head RIGHT, out of view; legs to the robot; body x .27-.40, y -.12..-.37+, crest z +.037 pelvis,
table z -.042..-.047), clear box BEHIND it, rotated ~11 deg with the robot heading: near wall rim from (0.436,-0.074) to
(0.382,-0.358) (x_wall(y) ~ 0.436 + (y+0.074)*0.19), far rim ~0.73. Library skill: plush-pick-place-box-behind (run_bb.py).

Stance: start edge .304; run.py stance (3 walks) -> .254; 2 more walks (0.07, 0.10) moved 0 (odo -0.02). Work at edge ~.26.

## Attempt 1 (run 1)
- prep, lean 0.1, perceive (grasp .25/-.21 far-.105, yaw -10), approach, hover: OK.
- grasp_deep light press (0,-0.015,-0.03 gate .012): palm stalled at top+.027, close weak (middle base 0.0, index .55).
- retry grasp DEPTHS 0,-.03,-.05: palm stuck at .052, TILT 7.44 -> aborted, raised. Cause (depth profile): palm at x .273 was over
  the near slope; pre-curled fingers sat on the crest (x .32-.34) and blocked the descent.
- APPROACH 2 (worked): palm over the CREST x .335 cmd (measured .324), y = body centre -.25, DEPTHS .01,-.01 GATE .02:
  close middle/index base .87/.86 (held). Lift to .27 held (middle .85, index crept 1.35).
- carry to (0.47,-0.25,0.26): palm .435 (= near wall .426). Gated lean release: palm .467 -> opened. FAILED: puppy fell back
  in front of the box, legs up. Body half-width 6.5 cm: palm needs >= wall + 0.07.
- Next: release at y ~ -0.33 where the slanted near wall is at ~0.387 (palm .46 = 7-8 cm inside). Reset 1 requested.

## Reset 1 (run 2): 3 failed closes, no lift (12:02-12:09)
- run_eed.py crest grasp (palm cmd .343, wall .42 at grasp y): palm stalled at z .087 -> fingertips on the box RIM (opened, raised).
- yaw -40 at the crest: close "held" (.96/1.24) but FK put the finger links 5-7 cm PAST the wall at rim height -> opened, no lift.
- deliberate drag (close at near slope, slide back 3 cm): puppy moved RIGHT, not back.
- asked the operator (3 approaches failed) -> "approved, continue". Reset 2.

## Reset 2 (run 3): 3 closes, no lift, BOX DRAGGED (12:10-12:18)
- palm .313 (wall-.103): fingers flat on top (base .32/.30). palm .333 (wall-.093) yaw -25.6: weak (.66/.68), palm z .081.
- palm cmd .331 (wall-.095) yaw -20, deeper press: "held" 1.08/1.06 but FK: finger BASE links exactly on the rim (x-wall 0, z-rim 0).
  Opened. IK raise refused (branch jump 1.64/1.20). My joint-space raise (elbow +0.35, then a clipped Jacobian step) moved the palm
  13 cm BACK and down: the open fingers over the rim DRAGGED THE BOX ~20 cm toward the robot; then home_safe swept it further and it
  tipped at the table's front edge. No fall (tilt <= 7.2). Operator reset 3 (box + puppy back at start).
- LESSON: with the puppy's back 3-4 cm from the box wall, a fingers-forward pinch has no safe window: palm <= wall-0.10 -> fingers on
  the crest (no wrap); palm >= wall-0.09 -> finger bases on the rim. The palm drifts +1.3 cm forward while pressing. Never move the
  hand back/down while any finger link is past the wall: raise first in Cartesian space (and check the IK solution's FK path).

## State at 12:18: robot home (both arms tucked, hands open), scene reset 3 = start layout. Resets used 3/5.

## Reset 3 (run 4, 12:20-12:24): roll close + crest grasp, no lift
- asked the operator to move the puppy 5 cm from the box: generic "continue", not moved.
- roll close at palm wall-0.125 (.296): weak (base .26/.20); after it wall read .441 (pelvis slid 2 cm), crest .354 -> gap +2 cm.
- crest grasp cmd .341 (wall-0.10), meas .351: weak (.54/.44), palm z .079 (high). Opened, raised straight up (Cartesian, ok), home.
- FINAL: robot home (arms tucked, hands open, tilt .8), box upright in place, puppy on the table. Task NOT solved: 0/4 runs,
  1 lift in 11 closes. Resets used 3/5. Library merged (plush-pick-place-box-behind v4).
