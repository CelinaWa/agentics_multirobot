# Handoff - puppy_box_r1 on r1sonic (sim), session 20261106T094654Z_puppy_box_r1_5600c1

Outcome: 6 placements confirmed INSIDE by the operator (j2, j4v, R2, j13, j14, j21) in 25 unattended attempts on
JITTERED grasp stances (+-3 cm) and 4 reset walk-ins (R2 = the FIRST walk-in placement on this task). j15a: robot stuck
against the table after a carry (operator reset). j21 used the stance gate (paw x >= .32, y <= -.18) and placed.
FINAL STATE: puppy INSIDE (j21: +1 fwd, -9 left, +12 above the floor), robot .44 m behind the edge, arms home,
grippers open, tilt .85.
Falls 0. Object incidents 2 (puppy off the table: j3 operator overhang request after a put-back; j12 my pull-back).
Resets 5/10, stance placements 20/30.
Library: **r1-leg-lowtilt-lift-advance-carry v4.1 (verified)** merged (main f3b9bcd). Older handoff: README_90b103.md.

## THE lesson: the lift depends on the STANCE
Lift completed (horizontal fingers, my paw fit): paw x < .32 -> 1/9; x >= .32 and y <= -.18 -> 4/5; x >= .32 but
y >= -.16 (walk-ins that drifted left) -> 1/4. So get the paw to x .32-.38, y -.19..-.24 BEFORE the grasp. Backward
walks to fix x drift the robot 5-11 cm right (paw y up) - j20 fixed x and failed on y. Next: a lateral correction that
does not drift (e.g. a small turn far from the table, or approach from further left so the right-drift lands at -.20).

## Run it
    # from an operator grasp stance (sim) or after the walk-in:
    python code/puppy_box/ctl.py <tag> tries=2         # v4: look (+waist look) -> grasp -> level lift -> carry
                                                       # precondition -> advance -> r1_carry_release -> retreat (~80 s)
    # full walk-in from a reset (robot .62 m back, paw overhanging):
    python code/puppy_box/home.py 0.6; python code/puppy_box/walkin3.py 0.39 0.35 7; python code/puppy_box/ctl.py R<n>
    # ground truth (sim): request_operator(help) "Question only, please do not move anything: is the puppy inside the box?"
Library copy: skill-library/skills/r1-leg-lowtilt-lift-advance-carry/code/{run.py, ctl.py}.

## How to plan each subtask in a differently laid-out scene
1. Walk-in (R1-R3, 4-5 walk calls, ~25 s): straight forward walks re-selecting the overhanging paw each time.
   The paw drops out of view at x < ~.33 (head .6); walkin3 then steps back -0.05 which walks -.11..-.18 AND drifts
   the robot 9-11 cm RIGHT (paw y -.25 -> -.14..-.16). Final stances: paw (.316,-.155), (.362,-.156), (.38,-.235).
   Better: stop the walk-in early (paw x <= .39-.40, still visible) and grasp from there - x .35-.38 works (R2 at
   .375, j13 .349, j4v .351). Do NOT walk back to fix a close stance: use the waist look (below).
2. Look: paw-ball sphere fit + leg axis (library chain.py). v4: accept a cut paw (n >= 300); if no paw and the edge
   is < .375 ahead, look_waist(): waist yaw -.35, perceive, waist 0, perceive, map the paw by the shared paw nearest it
   (translation only). Wrong rotation from a yawed-view edge fit threw the paw 4 cm once (j6v, fixed).
3. Grasp: horizontal fingers across the leg, approach = measured leg yaw, G = paw + .03 along the leg, pregrasp 8 cm
   short, servo on, close to 0 -> .035-.038 typical. Regrasp 1.5 cm deeper ONCE only if > .045; never chase a narrow
   width with deeper regrasps (the leg gets thicker; regrasps pushed the paw: j11, R3).
4. Lift (servo off, level approach, 2 s/step, library PROFILE): THE weak stage, ~50%. Signature of a failure: width
   rises 2-4 mm at steps 0-2, then falls at hand z .10-.17 (body leaving the table). Abort = retrace and open low,
   no drop (8/8 safe this session). Not separable by leg yaw (11-17), paw x (.30-.38), close width, lift speed (3 s/step
   failed too). Vertical straddle (fingers 0,0,1) held the lift steadily but loses the leg SUDDENLY (3/4 that got high).
   Ideas not tried: move the hand toward the body's centre of mass while lifting (perceive the torso blob), or a
   box-beside layout to avoid the carry walk.
5. Carry (v4.1): target <= edge distance - .22 (j15a: a .203 m carry from an edge .39 ahead left the torso against the
   table, the retreat crashed and the robot could not walk back -> operator reset), equal steps <= .10.
   Carry precondition (v4): only carry if the end-of-lift width >= .031 and <= 3 mm below its peak; otherwise put back.
   r1_carry_release(target = wall + .10 - (hand x + .06), lower .06, step .10): walks overshoot (.144/.145 for .10)
   but entries >= .0337 held every time; release lands the torso -6..+2 cm of the box centre.
6. Retreat: library v4.out (open, up, back, out (.14,-.40,.14), walk back .2, HOME). After a slip with the hand up:
   free_arm (open, back to x .18 at height, out (.12,-.40,.10), HOME) - never straight down onto the puppy.
7. Operator: never ask for an overhang after a put-back (pushed the puppy off the table, and the reply still said
   "puppy on the table"); never drag a paw toward the edge. Since 02:20 the operator gives no coordinates.

## Files
memory/PROGRESS.md, memory/RUNS.md (5600c1 table j1..j14, R1..R3), memory/TIPS.md (5600c1 rows), memory/INCIDENTS.md,
memory/PARAMS.json; code/puppy_box/{ctl.py, lowtilt_v4.py, look_waist.py, look_only.py, dry_v.py, ov2.py, walkin3.py,
home.py}; results output/skill_runs.jsonl; frames runtime/frames/<id>/ (ids in RUNS.md).
