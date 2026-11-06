# PROGRESS — puppy_box_r1, session 20261106T021942Z_puppy_box_r1_cdc45b (sim)
Started 02:19Z. Budget 7200 s, $10 Gemini. Resets 0/10 (first 5 count as "transferable").
Inherited: 5 sim sessions, 0 placements. Best so far (f6b331): leg-shaft grasp held, arc lift to z .365; slips were a
sim artifact (fixed 18:13). Walk now self-corrects. Code paths repointed (sed) to this workspace.

## Plan
1. Observe the start layout (edge, puppy, box side). 2. Approach with straight walks; stance so the overhanging
paw is at x .36-.39, y -.20..-.29 with the arm at READY. 3. Shaft grasp (run.py), arc lift, raise.
4. Walk holding (short steps) until the box interior is under the hand; lower, open. 5. Retreat.

## Log
### 18:20-18:40 attempt 1 (start layout): approach chase
- Start: edge .623 square; head RIGHT, box BEHIND (x .84-1.13, y -.33..+.06); operator overhang (29 s): hind paw A
  (.546,-.010) 7.7 cm past the edge, hind B (.592,-.134), front C (.548,-.192,-.022 low), front D (.566,-.256).
- Odometry (r1_walk 'pelvis') = world frame aligned with the table normal (paw A re-measured at (.669-.677,.168-.173)
  from 6 different poses). Edge world x .758. Paw A world (.669,.170); C (.674,-.012); D (.693,-.076).
- NEW WALK (this sim): straight walks accurate in distance (.244/.25, .458/.45, .483/.45, .385/.45, .391/.45) but
  drift to the robot's RIGHT .33-.5 m per metre forward (effective walking direction = heading - 18..-27 deg);
  backward walks drift little (.03-.1). TURNS ARE CHAOTIC: +45 -> 0.42 m right; -45 -> .06 right; -90 -> .6-.96 LEFT
  (+.25 fwd); +90 -> 1.23 right + .52 fwd (!); +15 (10 deg) -> .18 right; +11 (6.6) -> .02; +50 -> .34 right.
  => do big lateral moves by walking with a heading; never big turns near the table.
- Leg A (paw->hip) yaw ~ -21..-27 deg vs table normal (shaft points, frame 202611051837317a573b).
- IK preview grid (yawreach.py): approach yaw -30..-45 reachable at x .30-.36, y -.30..-.38 (ik <= .015);
  approach 0 at y -.15..-.22.
- 18:41-18:59 stance at pelvis world (.27,.12) heading ~30 (walks: .334 @ bearing 41, .148 @ bearing 10, turn -12 ->
  -13.6 deg/.07 right). Paw C (front lower) came into reach at pelvis (.29-.34,-.29..-.31), leg C yaw -3 deg vs table.
  C's neighbours B (+4.1 deeper, +5.9 left, +3.3 up) and D (+1.7, -5.9 right, +2.2 up) block horizontal fingers.
- PERCEPTION: every arm/waist move shifted the pelvis-frame scene 1-2.5 cm (and up to 2 deg). Waist yaw is NOT
  tracked with the arm out (cmd -.45 -> -.32/-.25/-.12; cmd -.6 -> -.22; cmd -.25 -> +.004). Head .6 + waist-yawed
  frames give z +1..2.4 cm vs head .5. Fix used: C = B + (-.061,-.038,-.032) (offset constant to 3 mm over 4 frames);
  B is visible when C is not. Hand tracking near the body (x<.30) ends 1-3 cm RIGHT and .6-1.5 cm HIGH of the goal
  (y servo saturates .046-.049): shoulder roll at its limit.
- GRASP (vertical fingers (0,0,1), approach along the leg (.84,-.54,0), grasp point 1.8 cm past ball centre, leg 1.9
  cm left of jaw centre): width .0322 = HELD. Arc lift (lift_c.py): z +.035 w .0315, +.07 w .0322, +.10 (approach
  z .1) -> SLIP .0069. Puppy rolled on the table (leg up). Vertical fingers clamp the leg's pitch -> torque -> slip
  (same as e0ac0c vertical ball pinch). => use HORIZONTAL fingers (pivot) for any lift.
- Full-open width reads .07 (max) even when a plate presses on a paw: open to .065 if contact sensing is wanted.
### Attempt 2 (reset 1, 19:00-19:06): reset ALSO returns the ROBOT to the start pose (edge .62 ahead, square).
- walk .2 from the start -> .257 straight (no drift this time). Paws below the view (x<.36); B visible at the image
  bottom; D = B + (-.026,-.122,-.007), C = B + (-.044,-.058,-.032) (start-frame offsets). D's leg yaw ~0 (3-D shaft
  points at constant y; it only LOOKS diagonal in the image: perspective).
- H fingers (-.174,.985,0), approach yaw +10 (achieved +5..+6, pitch -9..-11): pregrasp (.268,-.268,.022) ok (5 mm);
  D was 1.85 cm left of the axis -> moved; hand ended (.325,-.250,.021), width .0714 before closing (contact),
  closed .0483 (= ball, not shaft). Lift (lift_h.py): z +.03 w .0457 -> .0405; step 2: hand JUMPED 3.3 cm left
  (servo) -> slip .0041. Puppy back on the table.
### Attempt 3 (reset 2, 19:06-19:12) — LIFT SUCCEEDED, carry walk dropped it
- Reset layout DIFFERS from the first one: hind paws (.571,-.002) & (.55,-.066,-.017) overhang; front upper (.594,-.124);
  front lower F (.546,-.249,.021) overhangs 7.4 cm, CLEAN (neighbours >= 12 cm left, snout ~8 cm right), leg yaw
  +11.6 deg vs table, pitch +17 (= f6b331's post-update layout). Offset F - frontupper = (-.048,-.125,.005).
- walk .15 -> .196 (turned right 5.8 deg); walk .06 -> .023 fwd but 9.3 cm RIGHT -> F at (.358,-.152,.03): IK for
  H fingers with approach yaw +16.5 (along the leg) only .005-.009 (inward approach is easy for the right arm).
- pregrasp (.32,-.164,.039) ok (7 mm); F re-derived from the front-upper paw (visible at waist 0); advance -> hand
  (.3815,-.1295,.027) (2.2 cm left/1.3 low of cmd) ~3.7 cm past F; close -> .0451 (ball pinch).
- lift_h2.py (servo=False, constant approach first 10 cm, then tilt .1->.4; +1.2/1.2/1/.8/.5 cm along per 3.5-4 cm
  up): width .044 -> .056, hand to (.451,-.165,.363): WHOLE PUPPY LIFTED, hanging rigidly 9 cm BEHIND and 16 cm LEFT
  of the hand, lowest point z .27 (rim .12). Frame 202611051910037168b4.
- Reach while holding: x <= .50 at z .36 (ik .027), .55 -> .064. Extended to (.484,-.196,.347) servo off: held .0477.
- walk .1 holding -> .122 -> DROPPED (.0041). Puppy on the table right of the box front. Frame 20261105191136a00ce8.
### Attempt 4 (reset 3, 19:13-19:16): same layout (deterministic). walk .2 -> .146 (heading +2.5): F fully visible (.395,-.203)
- pregrasp (.345,-.212,.037) ok; F re-fit with the arm out (.390,-.2005,.030) == ref-paw estimate; advance 8.7 cm ->
  hand (.426,-.201,.056) 3.6 cm past F; close .0407; lift_h2 -> hand (.467,-.246,.402), width .041 -> .049. LIFTED.
- walk .05 holding -> DROPPED (.004). Puppy on the table in front of the box.
### Attempt 5 (reset 4, 19:18-19:23): controller code/puppy_box/frontpaw.py look/walk/pre (bugs fixed on the way:
  fit threshold n>=800, edge-normal sign, geometric-rule re-localization, stale-edge fallback, edge filter |yaw|<35 &
  dist>.2). pre put the hand 4 mm from the goal but the reach shifted the scene 3.3 cm -> jaw touching F's ball (.0689)
  -> manual back-off / re-align / deep advance (5 cm past) -> close .0435 -> lift to z .365 (step 9 refused by the jump
  guard) with width sliding .0435 -> .031 (onto the shaft) -> waited ~60 s (width crept to .0276) -> walk .05 overshot
  .141 -> DROPPED. Retreat ok.

## FINAL STATE (19:23, session ~63 min used)
- TASK NOT ACHIEVED (0 placements). Resets 4/10. Falls 0. Robot standing ~.44 m from the edge, arms home, grippers
  open, tilt .2; puppy on the table in front of the box (not in it); box upright; nothing on the floor.
- NEW and reliable: whole-puppy lift via clean front paw F pivot pinch + servo-off arc (3/3 lifts to z .36-.40).
- UNSOLVED: transport to the box (box BEHIND the puppy, needs ~.3 m) - r1_walk while holding drops it 3/3; arm reach
  holding is x <= .50 at z .36.
- Skill library: r1-paw-pivot-pinch-servo-off-lift (draft) merged -> main 9c0f1a6.
- Next: (1) a carry that does not walk while holding (e.g. lift, set the puppy down on the table nearer the box,
  release, walk, re-grasp; or use_waist reach + walk BEFORE the grasp so the box is within .5 m after the lift);
  (2) if walking, walk immediately after the lift (no pause: the width creeps) and test r1_walk 0.03; (3) ask whether
  the box may be placed beside the puppy (box-beside layout needs no carry walk).
### Attempt 6 (reset 5, 19:26-19:28): frontpaw look/walk(.194)/pre ok; deep grasp 6 cm past F: hand ended 1.8 cm low,
  width .0681 before closing (jaw pressing outside of a paw), closed .0072 = MISS; paws pushed. Retracted.
### Attempt 7 (reset 6, 19:29-19:31): walk .2 -> .236 but drifted 12 cm LEFT (F out of view, IK .03 refused); walk .06
  -> .013 fwd + 6.8 cm RIGHT -> F (.328,-.231) visible; pre ok (laterally centred .1 cm); grasp 5 cm past -> .049;
  lift_h2 -> hand (.426,-.244,.369) width .054 (4th servo-off lift success); IMMEDIATE walk .04 -> .039 walked,
  width .0536 -> .037 STILL HELD (first walk-while-holding that held); ~15 s later (next script call) width .004 =
  crept out while standing. Retreat ok.
## FINAL STATE (19:31, ~79 min used): robot standing ~.73 m from the edge, arms home, grippers open, tilt 1.2; puppy
  on the table in front of the box (head over the front edge, stable), box upright, nothing on the floor.
  Resets 6/10. Falls 0. TASK NOT ACHIEVED. Lifts 4/4 (servo-off); walk-while-holding 1/4 short step survived; the
  ball pinch creeps out within ~15 s even standing still.
