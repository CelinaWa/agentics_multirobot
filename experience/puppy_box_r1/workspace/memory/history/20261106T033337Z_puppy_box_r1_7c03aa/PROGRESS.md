# PROGRESS — puppy_box_r1, session 20261106T033337Z_puppy_box_r1_7c03aa (sim)
Started 03:33Z (sim clock 19:33 local). Budget 7200 s, $10 Gemini. Resets 0/10, stance placements 0/20.
Inherited: 6 sim sessions, 0 placements. Lift works (pivot pinch, servo-off arc). Carry by walking dropped 3/4
with BALL pinch (.041-.048). TASK.md now: deep SHAFT grasp (~3 cm up the leg past the ball, width .031-.035) held
150 s still and through 3 short walks. New sim accelerator: help "place me at the grasp stance".

## Plan
1. Observe start scene. 2. Practise at grasp stance: deep shaft grasp (fingers 0,1,0; approach 1,0,0), arc lift,
short walks holding until box under the hand, lower, open. 3. Full chain with walking in on resets.

## Log
### Stance 1 (19:35-19:41): paw (.322,-.202,.024) overhang 7.5, leg yaw ~12, edge .397 sq, table z -.041,
  box x .626-.906 y -.284..+.114. Pregrasp ok; advance along leg to 5 cm past -> hand ended 1.8 cm HIGH (contact,
  width .0711); lowered, closed .0402; re-grasp 1.5 cm deeper .0416; ~30 s idle (writing script) -> crept to .0107.
  Puppy rotated on the table. LESSON: chain grasp->lift in ONE script.
- GEOMETRY (r1_piper_kin.xml): finger pads span grasp-5.1 cm .. grasp+2.0 cm along the approach, 5.6 cm wide.
  Approaching along the leg the 5 cm ball is ALWAYS between the pads (shaft-only needs grasp > ball+7.6 cm, leg ~7).
  So widths .040-.045 = ball equator at the pads' back edge. Side approach (yaw >= 60) not reachable (IK .06-.085).
### Stance 2 (19:48-19:49): chain.py look/pre/go 0.18 0.05. Target (.327,-.196,.029); ref offset relocalization ok
  (ref moved 1.5 cm). Advance -> closed .0437; servo-off lift 8 steps -> hand (.434,-.254,.316), width .0437 -> .0278
  (slid onto the shaft). Hanging body (cream pts): centroid = hand + (.037,+.144,-.12), lowest pt hand z - .27.
  walk .18 (walked .158) -> DROPPED (.0278 -> .0041). Puppy out of view (probably floor in front of the table).
  Physics: COM 14 cm sideways along the finger axis -> torque about the approach axis resisted only by the friction
  couple (~18 N x width ~ .5 N m) -> marginal; walking jerks drop it.
### Stance 3 (19:51): r1_head after the stance placement RE-SENT the stale arm command (arm up to z .35, gripper 0):
  the arm swung through the paw area, tilt 4.1, scene shifted .2 m. RULE: after a stance placement/reset, first
  r1_move_joints HOME + r1_gripper open BEFORE any other motion tool (or make sure the stored command is HOME).
### Stance 4 (19:53-19:55): ENGINEERING NOTICE: paw ball is soft, jaws clamp the leg core; width .031-.035 = clean
  shaft on the leg axis; .040-.058 = diagonal/two legs. Leg axis measured by x-slices of the leg band (right edge):
  yaw 19.5 deg (not 12), axis 4 mm left of the paw centre, level (pitch ~0). Grasp along it: closed .039 -> .030
  during the lift (clean shaft). Lift to z .396; walks .082 + .065 HELD (w .030 -> .026); hand sagged 6 cm during
  walks -> body zmin .084 < rim .116 (pressed on the box front wall); rim detector picked the arm -> extra walk ->
  SLIP. Fixes in chain.py: HOLD_Z .44 re-raised after each walk, dead-reckoned box x, gated rim.
### Stance 5 (19:59-20:00): front-edge detector replaced (min-x of table points per y bin, robust line). Leg axis
  slices: yaw 19.7/20.1/22.4/20.3 deg in 4 stance frames, axis 1-3 mm right of the paw centre. Grasp IK: cmd yaw 20.3
  achieves only 17.0 (pitch -8, ik .016) and the hand landed 1.6 cm left -> closed .0373 (diagonal), lift ok, slipped
  at the first walk (w .0263). Hanging puppy is mostly ABOVE the camera view at head .6 -> body offset from constants.
### Stance 6 (20:03-20:04): choose_approach (cmd yaw +4, pitch -15 -> achieved 22.0 = leg 22.3, ik .008): CLEAN
  GRASP .0341; lift .034->.040->.0297 stable; walk1 .09 held (.0284); re-raise refused (z .455 ik .031); walk2 .00
  (.0261), walk3 .064 -> SLIP. Each walk costs ~1-1.5 mm of width and the hand sags/drifts right ~2-3 cm per walk.
### Stance 7 (20:06): leg yaw 23.8; closed .0376 -> grew to .047-.051 during lift (diagonal?) but held; walks .046,
  .067 held; re-raise ok; 3rd walk .067 -> SLIP. => 3rd walk fails 3/3 (st4, st6, st7); 2 walks survive 3/3.
  Change: MAX_WALKS 2 (cmd .08), then waist reach (z .43 -> .40) and release regardless.
### Stance 8 (20:09): yaw 20.8, approach ach 20.2 ik .006, hand 4 mm off; closed .0363 -> .0277 during the lift ->
  SLIP on the 1st walk (w .0269). Pattern: end-of-lift width ~.030 (st4, st6) or ~.05 (st7) survived 2 walks; < .028
  (st5, st8) failed at once. Next: past .06 (deeper), lift straight to ~.43.
### Stance 9 (20:11): past .06: closed .0373 -> .0298 (clean) at lift end z .41; walks .069 (.0284), .022 (.0259);
  re-raise move -> SLIP (.004). Waist reach solution jumped .87 rad (refused). Tally with end-of-lift width >= .0285:
  2 walks survived 4/4 (st4, st6, st7, st9); 3rd walk 0/3; arm moves under load after walks also slip.
  Next: lift straight to ~.44, 2 walks of cmd .10, no re-raise unless z < .40, reach without waist first.
### Stance 10 (20:14): wide grip .048-.053 (diagonal?) survived 2 walks (.066+.056); reach did not move the hand
  (x .43, arm cannot extend at z .40); released -> puppy on the TABLE in front of the box. Body hangs ~5 cm BEHIND
  the hand: the 13 cm sideways offset lies along the finger axis (yawed 22 deg). Fix: rotate approach yaw to ~2 deg
  during the lift (IK ok at the hold pose), no reach, release right after 2 walks (cmd .12).
### Stance 11 (20:19): yaw rotation 21.7 -> 2 deg during the lift: grip .0297 at the end, body centroid +.07 AHEAD of
  the hand (was +.04); walk cmd .12 (walked .081) -> SLIP at once. Back to cmd .07, try MAX_WALKS 3.
### Stance 12 (20:20): shifted layout (box .2 m right, paw z -.01 variant): wrong paw selected (leg edge at the slice
  window boundary) -> stopped at 'go' (reference not found). Added the boundary check. No contact.
### Stance 13 (20:22): yaw-rotated lift, wide grip .039 -> .052; walk .099 held, walk 2 (.053) -> SLIP.
  2-walk survival so far 5/10 runs (st4,6,7,9,10), 3rd walk 0/3. Back to MAX_WALKS 2 + immediate release.
### Stance 14 (20:25): grip .036 -> .0279 (weak); hold re-raise with yaw-2 approach jumped the hand 7 cm right; SLIP on
  walk 1. STOP stance experiments (time). Carry tally: 2 walks survived 5/12, 3rd walk 0/3, placements 0.
### Reset 1 (20:26-20:35) WHOLE CHAIN, walked in: walks .135, .061, .069 (overshoot: paws below view), -.05 -> -.129,
  .07 -> .088 -> edge .381; target paw hidden -> localized from the reference paw + offset (previous frame).
  pre/go: grasp .0425, lift to z .34 (IK limit at y -.17), walk .123 held, re-raise, walk .022 -> DROPPED onto the
  box FRONT RIM. Push-in: closed gripper, (1,0,0), z .16: push 1 (x .36->.52 at y -.14) moved it over the opening but
  it lay across the rims; walked closer (~5 cm), hit the table edge from below once (tilt 5.5, freed by HOME),
  push 2 via waypoint (.20,-.28,.16) -> (.36,-.20,.16) -> (.52,-.20,.16): puppy UPRIGHT INSIDE the box (tail end on
  the floor, head/front paws over the rim). Retreat, walk back: frame 20261105203524734959.

## FINAL STATE (20:35, ~70 min used)
- Puppy body inside the box (upright, head + front paws above the rim) - judged from the image of frame
  20261105203524734959 (depth sees the front wall, not through it). Box upright, nothing on the floor. Robot standing
  ~.45 m from the edge, arms home, grippers open, tilt 1.7. => success conditions appear met, ASSISTED by 2 pushes.
- Resets 1/10, stance placements 14/20, falls 0.
- Library: r1-leg-axis-grasp-short-carry (draft) merged -> main 41105ce.
- Controller: code/puppy_box/chain.py (look | pre | go 0 <past> <dz> | retreat <back>).
