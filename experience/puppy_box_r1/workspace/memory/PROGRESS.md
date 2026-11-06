# PROGRESS - session 20261106T112411Z_puppy_box_r1_3d54de (sim, 7200 s, started ~03:23 local, ends ~05:23)

## Current step (05:01) - session wrap-up done
Resets 10/10 used. Stance placements 19/20 (s15-s19 all failed: lift aborts, s17 carry stall; regrasp rule OFF after s18). Library merged 4x:
r1-walkin-leg-chain-lift-box v2 (draft) + r1-leg-lowtilt-lift-advance-carry v4.2 -> main 83707ce. handoff/README.md,
TIPS rows, INCIDENTS rows, PARAMS.json entries written. Final: robot backed off, arms home, grippers open (puppy NOT in the box at the end: s19 failed).

## Controller (final)
`bash code/puppy_box/full.sh <tag>` = home.py -> walkin5.py (.35/.375) -> cl.py -> one in-place retry after a safe
lift abort / weak-grip put-back / grasp miss. Then ask the operator "Question only, please do not move anything: is
the puppy inside the box?". From a stance: `python code/puppy_box/cl.py <tag>`; sim batches:
`python code/puppy_box/stance_batch.py <n0> <n> [key=value]`.
cl.py v6: look (lowtilt v4.2 waist look) -> grasp depth .045 (<= .405 - paw x) -> regrasp 1.5 cm deeper while close
< .039 (max 2) -> chain lift (1.2 s/step, yaw -> 2 deg, abort_fall .006 steps 0-5, waist retry for refused low
steps) -> top gate .031 -> r1_carry_release(total = wall + .09 - (hand x + .035), walk <= edge - .16, lower .12) ->
v4.out retreat with a walk-back recovery (hand up, HOME, walk -.15).

## Results (memory/RUNS.md, session 3d54de section)
- Reset walk-ins (10): INSIDE R6, R8. First five (R1-R5, ctl.py/bl.py lifts): 0 inside. Final chain (R6-R10): 2/5.
- Stance placements: INSIDE s1 (bl.py) and s14 (cl.py v6, dropped past the wall); rim s4; others lift aborts / drop.
- Lift is the weak stage: chain lift 8/26 attempts (reset walk-in scenes 3/4, stance placements 4/19), r1_leg_lift 2/7, lowtilt 0/1. Carry + release with the final
  target 2/3 inside (R8, s14; s17 carry stalled and dropped).
- Falls 0, puppy off the table 0. One stuck retreat (s14) recovered by hand (hand up, HOME, walk back).

## Lessons
- Close width >= .039 lifted 6/13, < .039 0/13 (chain lift): weak predictor; regrasping to widen lifted 1/5.
- r1_carry_release counts advance_m inside target_forward_m; torso lands at hand x + .02..+.05.
- Walks drift 0-5 cm either side; each waist look shifts the pelvis 2-5 cm; far paw (> .37) = IK dead end for the
  lift, close paw (< .30) = grasp misses.
