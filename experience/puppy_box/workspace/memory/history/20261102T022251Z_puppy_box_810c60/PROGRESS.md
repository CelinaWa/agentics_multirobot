# PROGRESS (session 20261102T022251Z_puppy_box_810c60, sim)
- 00:00 Read TASK/handoff/TIPS/skills. Reusing plush-pick-place-box-one-hand (code/puppy_box/controller.py).
  Rule this session: RIGHT hand only, left tucked (sp .45 el -.5). Box side unknown yet.
- 02:27 Layout: puppy CENTRE-LEFT (head left, legs to robot, body x 0.24-0.34 y 0.0-0.17 top ~0.09 pelvis, step 615),
  box RIGHT (inner wall y -0.12, centre y ~-0.31, rim z ~0.135). Puppy read 7 cm closer after arm moves (robot settled).
  Right hand raised safely: sp 0.9 -> el -0.9 -> sp 0.1 (fingers stay x<0.22, behind the box near wall).
  Plan: waist yaw +0.2 measured, palm yaw 30, grasp (0.27,0.08,0.06); release (0.38,-0.27,0.29) at waist 0 (IK ok).
  code/puppy_box/reach2.py = multi-seed IK reach check (single-seed solve gets stuck in local minima).
- 02:30 KEY: SONIC compensates under-tracked waist with the arm: FK(COMMANDED row incl. cmd waist) ~ measured palm
  (+-1.5 cm). Solve IK on the commanded row (code/puppy_box/mv.py move()), NOT on measured waist (10 cm off).
  Waist yaw cmd 0.7 -> meas 0.24-0.30.
- Attempt 1 grasp: palm yaw 30, (0.27,0.04) pressed cmd z .06 -> palm stalled .098; close -> idx .54 mid .77 (held);
  lift 4 cm steps 2.5 s: idx crept .63->.84->1.52 at z .22, lost at .26-.29. Puppy fell back on the table.
- Attempt 2: puppy shifted +4 cm y. palm yaw 15, (0.265,0.11) cmd z .08/.05/.02 -> palm .11/.095/.081 (waist crept .31->.375,
  palm drifted to rear third). Close -> idx .96 mid 1.02. Lift 3 s steps to z .274 held (idx .81-.87, mid 1.0->1.31).
  Carry waypoint 1 (waist cmd .7->.45 + arm, 3.5 s): grip LOST (fist), puppy on its back near front edge. Reset 1 requested.
  Lesson: grip survives slow vertical lift but not a waist swing while hanging. Next: smaller waist cmd for the
  grasp, carry by arm-only slow small steps, waist unchanged.
- 02:37 Reset 1 done (puppy body y -.05..19, far edge .335). Box (2 views agree): x .24-.53, y -.125..-.515, rim z .13.
  Approach 3: waist cmd 0.45 fixed for grasp AND carry (IK ok for grasp (.265,.07,.02) y15 and release (.37,-.27,.27) y-20),
  arm-only slow carry. code/puppy_box/carry.py waypoint() changes waist+arm together.
- 02:40 ATTEMPT 3 SUCCESS (by hand, operator confirmed "dog INSIDE the box", own camera agrees, frame 202611020240164042ec).
  waist cmd .45 (meas .17-.22) fixed; hover (.265,.07,.22) -> shift y .045 (looked mid-body); press cmd z .15/.11/.08/.05/.02
  -> palm .087 (stall); close idx .83 mid 1.01; lift 3 s 4 cm steps to .28 (idx .6-.81); carry arm-only 5 s steps
  (.30,-.02,.30)y10 (.32,-.10,.30)y0 (.34,-.18,.30)y-10 (.38,-.31,.29)y-20 (.39,-.35,.29)y-25 -> palm (.369,-.337,.324);
  open; raise (.33,-.30,.38); waist 0 hand (.22,-.22,.38); home R(sp.1 el-.9 wr0)->sp.9->el.6, L sp.9->el.6->both sp.5.
  Next: controller code/puppy_box/pb3.py + reset test.
- 02:56 Controller code/puppy_box/pb3.py: run 7 SUCCESS step by step (operator confirmed). Runs 4-6 failures fixed (pinch shape
  not set after home; lift first step too large; free-space jump limits). Resets used: 4 of 5.
  Hazard: robot drifts back from the table over the session (operator: 0.175 -> 0.399 m); home step showed tilt 11.6 deg once.
  Remaining: skill library (plush-pick-place-box-one-hand v2), handoff, TIPS.
- 02:57 Skill plush-pick-place-box-same-side v1 merged (library main ecaabb0). handoff/README.md + TIPS updated.
  Final scene: puppy INSIDE the box (operator confirmed after run 7), robot home, hands open. Resets used 4/5.
- 02:59 Second merge (one-hand skill note), library main fc5f7d0. Session end state: puppy in box, robot home. Done.
