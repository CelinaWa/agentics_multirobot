# PROGRESS - session 20261102T122445Z_puppy_box_ad5d0f (sim), 60 min wall clock

## State at the end
- Task NOT solved. Robot standing in the safe end pose (both arms tucked sp ~0.45 / el -0.5, waist 0, hands open, tilt 2.9);
  box upright in place; puppy on the table (shoved a few cm toward the robot/right by two presses). No resets used, no falls, no incidents.
- Skill library merged (main 5e9d23a): plush-pick-place-box-behind evidence + code/prep_side.py, up.py, fk.py.

## Layout (this session)
- Box directly BEHIND the puppy: near wall x 0.445 (pelvis, after crouch fwd 0.2), y -0.35..+0.06, rim z = table + 0.14 = +0.07.
- Puppy head RIGHT, legs to the robot, body y -0.04..-0.27, head ~-0.33, crest x 0.348, far side ~0.40, depth top +0.005
  (real top ~+0.04), gap to the wall ~5 cm. Table z -0.0675..-0.0725 pelvis.

## What happened
1. stance (same-side stance.py, EDGE_GOAL 0.175): edge 0.278 -> 0.268 -> 0.271 -> 0.265; one more 0.10 walk: odometry -0.004.
   Walking does NOT advance in this sim. Operator reply generic ("step up yourself").
2. g1_crouch(0, forward_rad 0.1) then 0.2 (arms_home False): edge 0.245 -> 0.228 (~2 cm each), tilt <= 2.9. Safe.
3. code/pb_ad5/prep_side.py (hand raised at the side) -> run_eed lean 0.1 -> perceive (release 0.47 out of envelope: wall 0.445).
4. Hover yaw 0: finger links 0.11 ahead of the palm frame (x 0.48) = past the wall -> re-hover yaw -40 (realised ~-27) -> -65 (~-40).
5. Attempt 1: palm cmd (0.32,-0.12), DEPTHS .01,-.01: palm stalled at z 0.076 (above the plush), close -> middle 1.52 index 0.89,
   finger gate warning (index 1.5 cm past the wall at rim height); raised straight up: empty fist. Box unmoved.
6. Attempt 2: palm cmd (0.30,-0.12), DEPTHS -.01,-.03 (-.05 IK-refused), palm 0.053, tilt 5.4; close -> 1.52/0.51; 3 cm raise: not lifted.
7. Stopped for the time budget; hand opened, raised straight up (up.py), home_safe.py. Notes + skill merge.

## Lessons
- The press is unreachable from >= 0.22 m: the root cause in every box-behind failure. Fix the stance first (operator), or don't grasp.
- index_0 0.5-0.9 with middle_0 at the fist is the index blocked by the thumb (pinch shape), not a hold.
- crouch(0, forward_rad 0.1-0.2) is a safe ~2 cm/0.1 stance gain when walking does not advance (tilt < 3).

## Next concrete step (new session)
Measure the stance; if edge > 0.21 after one walk + crouch(0, fwd 0.2), ask the operator to PLACE the robot ~0.17 m from the edge before
any grasp; then run_eed (prep_side instead of prep), yaw -65 cmd, palm cmd = crest - 0.03, deep press with the gate palm <= depth top.

## UPDATE 12:48 (only ~24 min had elapsed at the "end" above - I misread the clock; work continued)
- Attempt 3 (puppy shoved closer, pinch yaw -65, deep press): palm stalled z .047 -> empty. Attempt 4 (yaw -10, palm on a STALE far-side
  estimate): fingers at the wall/rim height, opened + raised straight up. Operator: generic. Reset 1 used (robot had crept to edge 0.205).
- Attempt 5 WORKED UP TO THE RELEASE: depth profile far side .383 @ y -.18, wall .428; palm measured (0.286,-0.146) (cmd .255/-.17 at
  waist .7/lean .1, yaw 0), light press DEPTHS 0,-.015,-.03 GATE .012 -> base joints 0/0, thumb .3 (fingers lying on the back) = 'weak'
  by the gate, BUT a 3 cm raise with the hand closed rolled the puppy into the hand (paws up) -> base .59-.95 / 1.33-1.39, lift in 4 cm
  steps to z .25 held, carry x .30->.48 cmd in 5 s steps held (palm .459), lean_release.py 0.2 0.47 0.50 -0.17 0.29 opened at palm .481.
  Puppy ended ON THE FLOOR (operator) - where it fell is not visible (turned waist hides the right side). Next: reset 2, same grasp,
  release lower (palm z ~.22) and look/depth before opening.

## FINAL (13:02, ~38 of 60 min): TASK SOLVED once - operator: "The dog is INSIDE the box"
- Controller: code/pb_ad5/run_ad5.py (library copy skill-library/skills/plush-pick-place-box-behind/code/run_ad5.py, v5, merged main 2052498).
  Run: `PYTHONPATH=.:code/puppy_box python code/pb_ad5/run_ad5.py prep,perceive,hover` -> LOOK -> `... grasp,lift` -> LOOK -> `... carry,release,home`.
- Runs: run 1 (attempt 5 by hand, reset 1) carried + released, puppy on the FLOOR (curled hand moved back after the open);
  run 2 (reset 2) grip lost in the lift; run 3 (reset 3) INSIDE. Resets used 3 of 5. No falls, max tilt 5.6, box nudged ~3 cm once (run 2).
- End state: robot standing, both arms tucked (sp .45 el -.5), hands open (fingers <= 0.01), waist 0, tilt 2.7; puppy in the box; box upright.
- Heading crept ~11 deg left over the session (quaternion z .005 -> .095) with arm work; edge 0.278 -> ~0.20 by itself.
- Unresolved: left-hand contact not confirmed by the operator (FK: left hand y >= +0.1, z >= +0.12, the puppy y <= -0.02).
  Grasp reliability 2 lifts / 3 closes with this method; release 1/2 (the one with release_up.py). Hover from a puppy closer than far .33 fails
  (palm-down pose not trackable near the body: fingers point up): reset rather than grasp there.
