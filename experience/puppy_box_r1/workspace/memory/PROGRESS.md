# PROGRESS - puppy_box_r1, session 20261106T043952Z_puppy_box_r1_cf6e88 (sim), started 2026-11-06 04:40Z

Budget: 7200 s wall clock, $10 Gemini. Resets used 0/10 (first 5 transferable), stance placements 0/20, falls 0.
Inherited: code/puppy_box/chain.py (look | pre | go 0 <past> <dz> | retreat <back>), library skill
r1-leg-axis-grasp-short-carry (draft). Prior funnel (7c03aa): lift 12/12, 2 walks held 6/12, 3rd walk 0/3,
1 assisted placement (release on rim + 2 pushes).

## Plan
1. Stance baseline with chain.py (verify the sim still behaves).
2. Box-beside layout (TASK.md suggestion) to avoid the carry walk.
3. Full chains with walking in (>= 2 of the first 5 resets).

## Log
### st1 (stance, box behind) 04:43: inherited chain.py unchanged: grasp .0397, lift to z .40 (.031), 2 walks .047+.067
  held (.0285) but only .114 of .195 needed -> released: puppy on the TABLE in front of the box's right corner.
### box-beside request: operator moved the box 44 cm LEFT of the puppy centre (same depth) but did NOT reset the puppy;
  a following "overhang" request said "18.5 cm past the edge" -> puppy fell OFF the table. Geometry: box-left is
  worse for the right-hand front-paw grasp (box centre ~.45 m left of the hand). Dropped.
### st2 (stance) 04:50: grasp .0397, lift_only -> width .0306, hand (.454,-.237,.426). HEAD-UP LOOK (pitch .1) shows
  the hanging body: centroid (.534,-.146,.39), x .48-.65, zmin .125 vs box front wall .59 -> 5-6 cm short.
  Outward yaw rotation needs an arm branch change past ~-10 deg (joint jumps > 1 rad). Yaw -10 at the real hand
  position: servo-off move jumped the hand 6 cm -> SLIP (.004). Puppy off the table. LESSON: no arm moves under load
  after the lift except tested ones.
### NEW PLAN (close stance): measure the paw at the stance (x .31), walk ~.1 closer EMPTY-HANDED, re-localize the
  invisible paw from the static box rim (chain.box_pose / box_transform), grasp+lift as usual -> body should already
  hang over the opening; head-up look; release. IK preview OK for paw x .21-.27 (ik < .003).
### st3 (stance) 04:54-04:58: close-stance trial. look (paw .317,-.208), closer .1 -> walked .04 (box_pose bug:
  back wall at rim height -> gated by prior), closer .08 -> walked .083: paw (.197,-.19) arm home. pre: the reach
  moved the scene 3.2 cm closer + 4.2 deg yaw + 6.7 cm in y (box-measured) -> paw (.158,-.236): waypoint/pregrasp IK
  .02-.04 = too close. Walk back -.05 -> -.135 (overshoot) + 5 cm right drift. Abandoned (no contact).
  LESSON [param]: target paw at x ~.24-.26 with the arm home (the reach brings it ~3 cm closer); walk total ~.06.
### st4 (stance) 04:58-05:02: closer walks: .06 -> .044-.048, .04 -> -.061 (BACKWARD), .08 -> .053; each drifted the paw
  +1.5..4 cm left (y -.208 -> -.155). box re-localization accurate (ref paw predicted within 3-5 mm). pre/go at paw
  (.27,-.185): grasp .039, lift refused at z .33 (y -.17: IK), width .048 -> .056 twisting; head-up: body held level in
  front of the camera (x .28-.52) far from the box -> released on the table: puppy SAT UP leaning on the box front rim.
  Push on its back (approach yaw 30 needed for reach) yawed the waist ~30 deg; puppy ended off the table (not seen).
  LESSONS: [param] small walk cmds (< .06) are random (even backward); closer-stance gains ~3-5 cm only at the cost of
  y drift; keep the grasp at y <= -.19 or the lift stops at z .33. Close-stance idea SHELVED.
### st5 (stance) 05:03-05:06: grasp .0395, lift_only .0297 hand (.453,-.245,.429); head-up: body x .46-.60 (front .59).
  walk .07 -> .095, w .0286; hand sagged to (.434,-.320,.380); body moved RIGHT: x .64-.79, y -.25..-.40 (box right
  wall ~-.26) = beside the box's right wall, deep enough in x. r1_place_in_box(side right) -> in_box FALSE: dog world
  (.885,-.186,.807), box (.875,.088,.703) (27 cm right of the box centre). ONE walk is enough in x; y is the problem.
  NOTE: st2's slip was MY error (re-based hold_pos 8 cm from the real command -> 7 cm jump). Relative moves are OK.
  NEXT: after walk 1, correct y with a slow waist yaw (+.1..+.2 rad) or a relative hand move, then release.
### st6 (stance) 05:07-05:09: grasp .0384 lift .0297; walk .098 (.0285): body hung LOW in front of the box right part
  (front wall blocked it; hand sagged to z .38); waist yaw +.25 while holding: width unchanged (.0282), hand moved
  (+.05,+.06,-.03) -> released: puppy on the table right of the box front corner.
### st7 (stance) 05:10-05:12: grasp WIDE .0436 -> .049; waist +.3 right after the lift: body HIGH (z .37+), left
  (y -.07..+.22), centroid x .42 (behind the hand!); walk .07 -> SLIP; puppy fell against the box front-left wall
  (depth: x .38-.44, z .14-.25, in FRONT of the wall .50; the image looks "inside" through the clear wall!).
  Push from the front at y -.05..0 not reachable (ik .04-.15).
  TALLY: lift 7/7; walk-1 survival 3/4 (st5, st6 held; st7 wide grip slipped; st1 2/2); placement 0/5.
### R1 = reset 2/10 (operator counts the mis-routed request as reset 1) 05:15-05:18: walk-in .136 + .11 (edge .372);
  target paw out of view at the bottom-right -> waist -.3 look, leg axis 12 deg measured manually, waist 0, pre/go:
  grasp .039, lift .034 (clean) z .44; head-up: body hung LOW (legs up, bottom z .04) against the box front wall;
  waist +.3 kept the grip but did not lift the body; released -> on its back in front of the box. FAIL.
  NOTE: a reset request containing "overhang" is routed as an overhang request: ask "reset the scene" plainly.
### st8 05:20-05:23: HIGH RAISE (chain go ... 1 1: after the LIFT 4 steps to cmd z .505, approach z 1.0, ik <= .006):
  hand actual z .464 (4 cm sag); body lay ON THE FOREARM (z .47-.55, x .25-.52) - supported, not hanging; grip wide
  .053 -> walk .07 -> SLIP; puppy sat up in front of the box (head over the rim). Push at y -.17 z .16 to x .536
  (approach (1,.15,-.2) start, ik .017): no tip-over.
  TALLY (session): lift 9/9; walk-1 held 2/4 (clean .03 grips 2/2, wide .05 grips 0/2); placement 0/10.
### R3 = reset 3/10 05:24-05:27: walk .2 -> .121 (8 cm LEFT drift: paw off-image right), turn -10 -> 10 deg (+5 deg
  over relative to the table) and -.024, walk .15 -> .118, walk .08 -> .121 (overshoot: paws below the view); box-rim
  re-localization gave y errors of 5-12 cm here (puppy at rim height in front of the box contaminates the right rim);
  back -.06 -> -.07 -> paw (.341,-.195) visible, leg yaw 24 (robust slice fit). grasp .0425 -> SLIP at lift step 9 ->
  puppy sat up against the box front rim. FAIL.
### st9 05:29-05:30: high raise again: closed .0384 -> .053 after the raise (approach rotation twists the leg);
  body on the forearm z .50-.54; walk .07 -> SLIP. High raise + walk: 0/2. Robot stepped back .1, arms home.
### st10 box-right request: box centre 60 cm right of the puppy (y -.5..-.8): out of reach -> dropped.
### st11 05:35-05:38: close .036 (clean) but lost at the end of the lift (hold step, now checked: exit 3); puppy sat
  up against the box front rim (4th time). Hind-leg tip-in: the two hind legs lie stacked (z .05/.01 along y at x .39);
  vertical jaws hit the lower leg (miss); top-down unreachable. Robot retreated .12, arms home.

## FINAL STATE (05:39, ~68 min used)
- Task NOT achieved this session: 0/10 placements. Puppy on the table sitting against the box front (last scene), box
  upright, nothing on the floor in the final view; robot standing ~.5 m from the edge, arms home, grippers open,
  tilt 0.9 deg, not touching anything.
- Resets used 3/10 (2 full walk-in chains), stance placements 11/25, falls 0.
- Library: r1-leg-axis-grasp-short-carry v2 (draft) merged -> main 877b319 (evidence + code + findings).
- Controller: code/puppy_box/chain.py (look | closer <cmd> | reloc | pre | go 0 .06 -.004 [1 [1]] | retreat) and
  code/puppy_box/carry.py (look | yaw | move | waist | walk | release | push | out), code/puppy_box/home.py.
- Handoff: handoff/README.md (previous: README_7c03aa.md). Tips: memory/TIPS.md (cf6e88 rows). PARAMS: 5 sim rows.
- NEXT: the carry. Most promising untested: waist yaw +.15..+.2 BEFORE the single walk with a clean grip, then a
  RELATIVE raise (+.03..+.05) to undo the sag, head-up look, release only when depth shows the body beyond the wall.
### st12 05:40-05:42 (v3: code/puppy_box/v3.sh): grasp .0403, lift .0468; waist +.15 moved the hand 1.6 cm RIGHT
  (measured .096); walk .073 held; relative raise refused (z .46 IK; steep approach = .8 rad jump); body low beside the
  right wall -> released beside the box. Robot stepped back .14, arms home, grippers open (final).
FINAL TALLY: 11 attempts (9 stance + 2 walk-in resets): grasp 11/11 closed on the leg, lift >= .40 9/11, walk-1 held
  3/7, placement 0/11. Resets 3/10, stance placements 12/25, falls 0. Session time used ~66 of 120 min.
