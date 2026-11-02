# PROGRESS (session 20261102T041502Z_puppy_box_6a0bfc, sim, 3600 s)
## State (04:56): run 1 SUCCESS, runs 2-3 FAIL (grip lost on the lift, puppy on the FLOOR at the end, operator says unreachable).
## Earlier (04:44): TASK DONE once (run 1). Puppy INSIDE the box (operator confirmed + own frame 20261102044313dd4917),
robot standing, arms home, hands open, tilt 0.8-2.5. Remaining: notes, skill library merge, handoff.

## Layout (box BEHIND the puppy, new type)
- Start: robot had slid back (operator disarmed "stance slide", re-armed ~3 min later and moved puppy+box ~8 cm closer).
  Edge 0.27 -> puppy x .29-.43 (head right), box floor x .49-.77 y -.01..-.41: out of reach.
- Operator allowed g1_walk 0.12 (then two more on request). Each 0.12 m walk (carry=true, arms lowered) advanced only
  ~3 cm (first one incl. warm-up turns). After walks the legs straighten (pelvis +2.5 cm); g1_crouch(0, forward .05,
  arms_home=False) restores the stand but slides back ~1.7 cm. Final stance: edge 0.196 (legs straight).
- Lean 0.1 + arm prep slid the pelvis back another ~4 cm (box near wall 0.39 -> 0.437).

## Run 1 (code/puppy_box/pbb.py + lean_release.py)
prep -> lean 0.1 -> hand up-left (waypoint (0.22,0.06,0.32) waist 0) -> perceive (box_behind colour mask, size prior)
-> state set by hand: grasp (0.28,-0.21) yaw 0, press top-0.03, waist cmd 0.7 -> hover -> grasp (idx 1.02, mid 0.91)
-> lift to 0.22 (mid 0.85 held, idx 1.35) -> carry to cmd (0.495,-0.20,0.21): palm only 0.453 (4 cm short at the
reach limit), paws 4 cm outside the near wall -> raised to cmd (0.51,-0.21,0.30): palm 0.461 -> lean_release.py 0.2
0.48 0.54 -0.21 0.28: palm 0.488 (tilt 3.2-3.6) -> opened -> raised -> unlean -> home. Puppy fell fully inside.

## Lessons
- Reach is THE constraint for box-behind: release must be checked on the commanded row with random-seed IK
  (reach2.best now adds 20 random seeds; the 6 fixed seeds gave 2-5 cm false misses).
- At the reach limit commanded-row FK over-predicts by ~4 cm (cmd 0.495 -> 0.453): plan release cmd >= near wall + 0.10.
- A gated leaned release (lean 0.2 ramp, release only if measured palm >= wall + 0.03, raise + unlean at once) worked.
## 04:48 library merged: plush-pick-place-box-behind v1 draft (library main 1540a99). handoff/README.md updated.

## Runs 2-3 (after resets): grip lost on the lift both times (1/3 overall). The grasp at far-0.10..0.11 / top-0.03 is
the reach-limited shallow version: palm stalls on the top (2.4 cm above in run 2), base finger joints 0.13-0.42 = weak
hook; even a good wrap (run 3 close 2, = run 1 values) slipped at z .07. Next: get the robot CLOSER (stance 0.15) so the
deep press (top-0.06) at far-0.07 is reachable, or a different grasp; check base joints >= 0.7 before lifting.
Final state: robot home, hands open, box in place, puppy on the floor (operator informed).
- [operator-scripted] 04:56 session end: run1 success, runs 2-3 fail; library main 3c66e82
