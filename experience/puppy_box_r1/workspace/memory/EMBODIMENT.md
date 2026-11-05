# Experience from another robot: g1sonic -> r1sonic

This session inherited the experience of `puppy_box`, which ran on **g1sonic**. This session runs on **r1sonic**: a different body, different tools, a different hand. The task is the same.

What came over and how to use it ([inherited-experience](../.agents/skills/inherited-experience/SKILL.md)):

- **Task layer, kept in place** (`memory/TIPS.md`, `handoff/README.md`): how the scene looks, how to find the objects and the box, what success looks like and how to check it, which steps the task needs and in what order, what failed and why. Use it as your plan until a measurement here contradicts it.
- **Rows marked `[body:g1sonic]`**: a task lesson learned through the other body. The lesson may hold; the mechanism and every number in it (angles, heights, distances, hand shapes) belong to the other robot. Re-derive them for this body.
- **Body layer, set aside** (`memory/history/puppy_box/BODY_g1sonic.md`): the other robot's joints, hand, gait and tool use. Do not act on it; open it only for the reason behind a mechanism.
- **Its code** (`code/from_g1sonic/`): reference, not your controller. It calls the other robot's tools. Modules that call no robot tool (perception: finding the puppy, the box, the table from images and depth) may run here unchanged once their camera inputs match this robot's observations; read before reuse.

- **Its skill library** (`code/from_g1sonic/skill-library/`): the other robot's task skills, files only. Your own library is this robot's and starts from what you merge into it.

Counts:

- PARAMS: 0 kept (priors to re-measure here), 0 set aside

Code files that call no robot tool (48), candidates for reuse:

- `code/from_g1sonic/skill-library/skills/hand-claw-pregrasp/code/palm_down.py`
- `code/from_g1sonic/skill-library/skills/hand-claw-pregrasp/code/run.py`
- `code/from_g1sonic/skill-library/skills/hand-claw-pregrasp/code/servo.py`
- `code/from_g1sonic/skill-library/skills/palm-pose-servo/code/ik.py`
- `code/from_g1sonic/skill-library/skills/place-reach-check/code/run.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/bb_8cef10.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/goto2.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/meas.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/nudge.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/openhand.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/raise.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/reachck.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/reachgrid.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/rimhit.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/e578/tuck_measure.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/fk.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/hang.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/layout2.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/pb4/reach.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/pbb.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/reach_rs.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/run.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/up.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-behind/code/via.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/boxdet.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd/cr.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd/layout2.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd/plan_cr.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/headleft_0878bd/pup.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/lr.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/percep.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/run.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/x258/corners.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/x258/open_up.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/x258/reachq.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/x258/reachq2.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-cross-right-hand/code/x258/regrasp.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-one-hand/code/boxdet.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-one-hand/code/percep.py`
- `code/from_g1sonic/skill-library/skills/plush-pick-place-box-same-side/code/da90/open_fwd.py`
