# Handoff — puppy_box_r1 on r1sonic (sim), session 20261106T021942Z_puppy_box_r1_cdc45b

Outcome: NOT solved (0 placements). 5 attempts, resets 4/10, falls 0. NEW: a reliable whole-puppy LIFT (3/3 to hand
z .36-.40) by a horizontal-finger pivot pinch of the clean overhanging FRONT paw + a servo-OFF arc lift. The carry to
the box (behind the puppy, ~.3 m away) is the open problem: r1_walk while holding dropped it 3/3.
Older handoffs: README_f6b331.md (shaft grasp, waist raise), README_c4f6c3.md (turn-arc model - now STALE), e0ac0c, 08803f.
Library: r1-paw-pivot-pinch-servo-off-lift (draft, merged main 9c0f1a6). Controller: code/puppy_box/frontpaw.py.

## What the scene/sim does now (verify first)
- A `reset` puts the puppy, the box AND THE ROBOT back (robot .62 m from the edge, square, head pitch 0 -> set .5).
- Layout after resets 2-4 (identical): puppy head RIGHT, box BEHIND it (front wall edge+.22, rim table+.145), lower
  FRONT paw F at pelvis (.55,-.25) overhanging 7.4 cm, nearest paw 12 cm away; leg F +11.6 deg vs table normal,
  rising 17 deg. The very first layout of a session (before any reset) can differ (hind paw overhang, crowded pairs).
- r1_walk: straight walks keep distance but drift 0-.5 m right per metre; turns move the robot .4-1.3 m
  unpredictably. From the reset start one straight .15-.2 walk is enough (F ends at (.32-.40,-.15..-.25)).

## How to plan each subtask in any layout
1. Stance: choose the overhanging paw with no other paw within 9 cm (select_paw in the skill); approach it with
   straight walks only, so it ends at pelvis x .33-.40, y -.15..-.25; check the grasp IK by preview (<= .02 good).
   Approach yaw = the leg direction in the pelvis frame (inward/positive yaw is easy for the right arm; outward
   -30..-60 needs y <= -.30).
2. Localize: sphere-fit the paw (n >= 800, rms <= 1 mm); measure the offset to a well-fitted neighbour paw at the start
   pose. After every arm move re-observe; if the target is hidden use neighbour + offset (pelvis frame shifts 1-3 cm).
3. Grasp: horizontal fingers perpendicular to the leg, waypoint 12 cm behind at z .12, pregrasp 4.5 cm short,
   re-observe, advance to 4-5 cm past the ball centre, close to 0 (.031-.048 = held). Back out along the leg before
   any lateral correction if a jaw touches a paw (full-open width < .069 or > .071).
4. Lift: 9 arc steps servo=False (code: skill lift()), width every step; stop at z >= .33 if the jump guard refuses.
5. Carry (OPEN): with the box behind, the hand must reach edge+.30..+.37 at z >= .33; the arm reaches only x .50
   holding, so the pelvis must be within ~.15-.2 of the edge. Walking while holding failed 3/3. Ideas not yet tried:
   (a) lift, swing/set the puppy down on the table nearer the box, release, walk (empty-handed), re-grasp, lift in;
   (b) walk to the close stance BEFORE the grasp and grasp a paw that does not overhang (top-down impossible: needs a
       new grasp); (c) use_waist reach (+5-10 cm) right after the lift, no pause (the grip creeps when waiting);
   (d) ask the operator whether a box-beside layout is allowed (then no carry walk is needed).
   UPDATE att 7: a .04 walk IMMEDIATELY after the lift held (width .054 -> .037), but the pinch then crept out
   within ~15 s standing still. If you try walking: one script, lift -> walk .04 x N -> extend (use_waist) ->
   release, no pauses between calls, width check after each; expect slips (1/4 so far).
6. Release/retreat: open, hand up/out (.28,-.32,.25), HOME joints, walk back .2 — worked every time, no contacts.

## Files
code/puppy_box/frontpaw.py (step controller: look | walk <m> | pre | grasp [past] | lift | retreat; state in
output/frontpaw_state.json), nav.py (walk + scene report), safe_move.py (previewed guarded move), lift_h2.py
(servo-off arc lift), lift_c.py (V-finger lift, failed), yawreach*.py (IK maps for yawed approaches), legdir.py.
memory/PROGRESS.md (attempt log), RUNS.md, TIPS.md (tagged rows), INCIDENTS.md, PARAMS.json (sim values recorded).
