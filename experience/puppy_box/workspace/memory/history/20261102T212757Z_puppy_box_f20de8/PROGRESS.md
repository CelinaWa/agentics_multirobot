# PROGRESS - session 20261102T212757Z_puppy_box_f20de8 (sim), started 21:28 UTC, 3.5 h budget (ends ~01:00)
Task: fix run.py (v17.2 f8db6cd) per TASK.md: box-LEFT lateral grasp, home's left TUCK hooking the box, box-BEHIND far-carry
finger loss, resets from the LOOK pose; test on resets; EVAL >= 8 trials.

## State (23:41)
- EVAL DONE (memory/EVAL.md block 4, code 1fa815b = v18.2): 4/8 clean program successes, 5/8 placed; no falls/box pushes/floor.
- code/ HEAD e5e5be0 = v18.4 (v18.3 post-eval release step-down + params read from PARAMS). Dev runs 1-8 in memory/RUNS.md.
- Skill library merged: plush-pick-place-box-any-layout v6, main f76e034 (controller = v18.3).
- TIPS.md, handoff/README.md (f20de8 section), PARAMS.json (6 new/updated entries) written.
- Resets used: 13 of 24 (5 dev + 8 eval). Robot standing in LOOK, puppy INSIDE the box (eval 8).

## v18 changes (all in code/puppy_box/run.py)
1. step_home: no TUCK; guard/sp .8 -> LOOK (FK links x <= .203). 7 homes beside near walls .206-.221: box untouched.
2. box LEFT: plan_left() grasp waist with IK margin (roll .25-.35), across-body yaw, separate release waist/yaw; carry: up, swing(), carry.
3. mv(): sync only on palm error > 4 cm.  4. carry at rim + max(.21, hang+.05), hang floor .22.  5. riding plush continues the carry.
6. v18.3: release steps down while the measured palm is > release z + .04 (untested).

## Update (00:00)
- Post-eval dev runs 9-11: 3/3 placed (operator INSIDE). Code HEAD 80b8524 = v18.8: carry height reachable at the release (v18.5),
  dense-run box extents (v18.6), to_look returns a blocked arm to guard (v18.7/18.8, the drawer unit beside the robot), hand-clearance
  gate ignores the raised hand's pixels. Library main 4035a67 (controller v18.8). Resets used 16 of 24.
- Robot standing, BOTH arms in guard (LOOK blocked at the end of dev 11), hands open, puppy INSIDE the box (dev 11), tilt 1.5.

## Next (if time)
- behind far carry: pick a carry height reachable at the release xy before the carry (no lowered + synced last step); test on resets.
- after any code change: copy code/puppy_box/run.py to skill-library/.../code/controller.py, commit, merge again.
