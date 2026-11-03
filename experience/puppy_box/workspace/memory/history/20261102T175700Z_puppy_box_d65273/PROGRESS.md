# PROGRESS - session 20261102T175700Z_puppy_box_d65273 (sim), 17:57-20:57 UTC
Task: puppy_box, right hand only. ONE program: code/puppy_box/run.py (now v17.2, code commit f8db6cd).
## State at 20:02
- Development: 11 runs (+retries). Placed (operator INSIDE): runs 1, 5 (manual centre nudge), 7, 8, 11 = 5.
- EVALUATION (memory/EVAL.md, commit 41edb62 = v14.1): 3/8 placed (box right 1/2, behind 2/3, left 0/3); 0 falls, box never
  knocked over in the eval. Funnel: lift held 5/10 attempts (2 IK branch jumps at the lateral reach, 3 slips), release 3/5.
- Post-eval: v15 (lift inward fallback, set-down for a plush on the hand, side release >= near wall + .13), v16 (Cartesian retreat
  before home: run 10's home dragged the box + plush off the table), v17 (box-left lateral grasp yaw -45: run 11 placed).
- Incidents: run 2 (side-look swept an edge puppy off: LOOK el .6), run 3 (waist .9 oscillation, box displaced: waist <= .7),
  run 6 (reset put the plush between tucked hands, the look move dragged it off: home ends in the look pose + hand-clearance gate),
  run 10 (home sweep dragged the box off: retreat). No robot falls; tilt <= 6.4 deg.
- Run 12 (v17): lateral yaw -45 empty fist at waist .7/.15, abort dragged the plush off the edge -> v17.2 abort hand-empty check.
- Resets used 19 of 20 (incl. the final tidy reset).
## Lessons
- Measured-palm aim at hover and per press level; press on while the palm rests above the plush top.
- Look at the hover: the paw-midpoint target sits 1-2 cm toward the head 9/9 times (shift -.015 toward the tail; param .015 now).
- Lift slips (3/10) near the front edge send the plush to the FLOOR: unsolved.
- Untested in v15-v17: lift inward fallback, set-down release recovery, x-nudge for the near-wall margin (none triggered in runs 10-11).
## Next
- Run v17 on new layouts (esp. box left lateral, slips), then re-evaluate; consider lifting over the table interior (slip = on the table).
- 20:01 Library merged (main f58349f, any-layout v5). Robot standing in the look pose (arms down, hands open, tilt 2.9); puppy and box re-laid on the table by the operator. Session work done.
