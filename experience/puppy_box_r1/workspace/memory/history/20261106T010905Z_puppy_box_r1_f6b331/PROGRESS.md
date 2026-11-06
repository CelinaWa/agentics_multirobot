# PROGRESS — puppy_box_r1, session 20261106T010905Z_puppy_box_r1_f6b331 (sim)
Workspace WRITABLE this time. Inherited code paths repointed (sed) to this workspace.
New in TASK.md: SHAFT grasp (grasp point ~3 cm up the leg from the paw ball toward the hip, approach (1,0,0),
fingers (0,1,0), close to 0 -> held .031-.040), ARC lift (+1.5 cm fwd per 4 cm up first 10 cm, then +1/4,
approach (1,0,0)->(1,0,.5), steps <= 4 cm), walking while holding OK with the deep grasp (3 walks .05-.08).

## Attempt 1 (initial scene, 17:10)
- Start frame 202611051710086ba406: edge .634 square, head RIGHT, box BEHIND (x .85-1.14, y -.27..+.13).
- help overhang (29 s): frame 2026110517112365e969: overhanging hind paw (.556,+.048,.003), other hind (.604,-.063),
  front paws (.558,-.135,-.011) (also overhangs), (.576,-.185). Box y -.266..+.132.
- Plan (walkplan model): back -.1, back -.1, turn 70, turn -60 -> pelvis world (.238,.251,9deg); target paw at (.30,-.20).
- Walk log (odometry 'pelvis' = world, consistent with edges to ~1 cm): back -.1 -> .20 / .219 (+-3 cm y);
  turn 60 -> 44.5deg actual, moved (.269,.07) [model ok]; fwd .15 -> .18 at heading 45; turn -40 -> -29..-31deg actual,
  moved .26; back -.1 -> .217; fwd .15 -> .10. => this session turn ratio ~0.74-0.75 x cmd (model had 0.87).
- Ended heading +15.8, edge .49, paw (.369,-.144,-.023) [waist -.35]. Leg (paw->hip) measured from shaft surface
  points: pelvis yaw -32deg (= -13deg vs table normal), pitch +24deg up. Approaches along the leg (-15..-32deg) are
  unreachable at y -.16 (shoulder roll at limit) except with use_waist (waist yaw +.7, roll .47 = limit -> not used).
  (1,0,0) ik .031 but 30deg off the leg. Lesson: the stance needs heading <= 0 (leg within ~15deg of +x) AND paw at
  y -.20..-.25; leftward shift + right heading is not reachable near the table -> reset 1 to re-approach.
## Attempt 2 (reset 1, 17:20) — approach only, no contact
- Reset layout = c4f6c3 layout: edge .626 square; overhang hind paw (.546,-.023,-.002), other hind (.597,-.133).
  Leg (paw->hip) measured at waist 0 from >= .45 m: yaw -9..-13 deg vs table normal, rising ~14 deg; horizontal-ish.
  !! Close-range waist-yawed views (paw x < .34) gave leg yaw -32..-44 = ARTEFACT. Measure the leg at waist 0, x >= .40.
- Walks: back .236, back .234 (odometry not reset by the operator reset), turn 70 -> +58.4deg (ratio .83), moved (.333,.065);
  turn -70 -> -67.4deg (ratio .96), moved (.441,.231) -> paw (.276,-.231) heading -8.5 (too close to see);
  back -.1 -> .207 => paw (.451,-.222) waist 0, visible, leg yaw -6 (robot frame);
  fwd .15 -> .087 (+.047 drift right) => paw (.388,-.175) [GOOD stance: preview grasp (.418,-.178,-.008) ik .024];
  fwd .1 -> .11 => paw ~(.29,-.15) (invisible) ; back -.1 .218 ; fwd .15 .064 => paw (.426,-.117);
  turn 20 -> only 7.6deg and drifted .08 RIGHT => paw (.37,-.067). Small turns (<= 20) are useless (mostly drift).
- IK at waist 0: grasp x >= .445 at y -.12 is NOT reachable with fingers horizontal (ik .032-.04, wrist flips).
  With use_waist the backend wants waist yaw .58-.76 + roll .44-.47 (limit) -> not used.
## Attempt 3 (reset 2, 17:27) — approach ok, reach aborted (paw drifted left in the pelvis frame), no contact
- Deterministic: back x3 (.237,.236,.226), turn 70 (57.3deg, (.339,.088)), turn -70 (moved .439) -> paw (.513,-.232)
  heading -5.2 edge .572; fwd .15 (.086) -> paw (.461,-.202) heading -9.2 [frame 2026110517275593f4fc, best ref view];
  fwd .1 (.103) -> paw (.389,-.165) heading -13.7 (paw at image bottom) [frame A 20261105172803865b10].
  Forward walks rotate the robot right ~4deg each and drift it right 3-5 cm.
- Pregrasp (.34,-.166,.012) fingers (0,1,0): measured waist yaw +.17 (cmd 0); then the scene was SHIFTED +.057..+.107 m
  in y (and -.02..-.03 x) vs frame A in every later frame (landmarks: box corner, tail; ICP). Paw then ~(.36,-.08..-.10)
  = too far left (y > -.12: shoulder roll at limit, IK flips fingers vertical). Not caused by the current arm pose
  (persists with the arm home). Unknown whether A was a post-walk transient -> TEST: observe twice after a walk.
- Camera cannot see the paw at grasp range (x < .36-.40) in any waist config; hand is never in view near the paw.
- code/puppy_box/reloc.py (ICP re-localization): coarse x/y grid init needed (plain ICP stuck at a wrong minimum);
  the yaw grid version took 8.5 min = TOO SLOW, do not use the yaw grid. topdown.py renders a top view (debugging).
## Attempt 4 (reset 3, 17:49) — FIRST HELD SHAFT GRASP + full arc lift, then slip while holding still
- approach.py (back x3, turn 70, turn -70): paw (.526,-.235) heading -4; settle check: 2 frames 6 s apart differ .5 cm
  (=> walking does NOT cause the big shift; reaching across (y > -.17) with waist 0 does: waist yaws +.17..+.23).
- Arm READY (.30,-.25,.08) before walking (shift only -.02 x): fwd .15 -> .077, fwd .1 -> .048 => paw (.413,-.234)
  [frame 20261105175004ec2c58, leg yaw -6]; fwd .1 -> .065 => paw invisible (x ~.35).
- waist cmd -.35 -> paw partially visible (.339,-.215) [partial sphere fit = BIASED]; pregrasp (.27,-.218,0);
  advance to (.361,-.215,0) + close -> .009 = MISS (paw really at ~.39,-.17: pushed ~3 cm).
- Re-look with the hand at pregrasp (.28,-.22,.01), waist cmd -.35 (measured -.10): paw FULLY visible (.383,-.171)
  -> align (.31,-.178,.016) -> paw (.39,-.167,.007), leg yaw -15 -> grasp (.405,-.173,.014) (ik .0276; hand ended
  (.42,-.161,.037)) -> close -> width .0351 HELD.
- Arc lift lift.py 6 steps (dx +1, dz +4, approach z .1->.5): width .0313 constant, hand (.477,-.163,.275), puppy
  hanging head-down, lowest point z ~.05-.07 (hang ~22 cm below the hand; rim z .11 => hand must be >= .35).
  Without waist the hand cannot go higher/further (x .50 z .30 ik .044); with use_waist z .37-.40 at x .45-.48 is
  reachable (waist roll .14-.2).
- I then spent ~2 min planning with the arm still: width went .0313 -> .004 = SLIPPED WHILE STATIONARY; puppy on its
  back on the table in front of the box. => chain lift -> raise -> walk without pauses (code/puppy_box/run.py).
## Attempt 5 (reset 4, 17:58) — run.py steps; grasp missed twice (paw pushed), no lift
- approach -> paw (.527,-.193) (4 cm more left than att 4: turn-arc noise +-2-4 cm); ready; fwd .15 (.071) -> (.46,-.197);
  fwd .1 (.082) -> (.376,-.218); pre -.21 -> paw (.383,-.204) visible, leg yaw -8.6.
- grasp (.40,-.207,.018) in ONE 12 cm advance from (.278,-.209): width .0727 before closing (= contact), closed .0227,
  then .0097 within 5 s; paw pushed 5.6 cm (to .439). LESSON: always align ~8 cm short first (att 4), single long
  advance deviates and pushes the ball; jaw clearance around the 5 cm ball is only +-1 cm.
- After the push: paw (.365,-.148) -> pre/align at y -.15: every leftward reach yaws the waist +.06..+.19 and the paw
  shifts +1..+4 cm left in the pelvis frame -> grasp (.398,-.135) missed (.004). Paw y must be <= -.17 at pregrasp.
## Attempt 6 (reset 5, 18:02) — grasp HELD .0315, lift+raise to z .365 OK, SLIP on forward extension
- approach -> paw (.509,-.248); ready; fwd .15 -> paw (.459,-.308) (robot drifted LEFT + turned left 4deg this time);
  fwd .1 (.13) -> paw invisible; pregrasp (.24,-.28,.02) waist -.35; waist -.6 -> paw fully visible (.333,-.293),
  leg yaw -12.5; waist back -.35, align (.272,-.297,.015); fast ICP (reloc.py, x/y grid only, 18 s) -> paw
  (.326,-.283); grasp (.346,-.286,.016) ik .0278 -> width .0315 HELD (z .006 was ik .0312 refused).
- run.py lift: 6 arc steps + 3 waist-assisted raise steps, hand (.39,-.27,.365), width .0316 -> .0306 (creeping).
- Then extension (.43,-.26,.35) with use_waist (waist yaw/roll moved): SLIP .004 -> puppy fell (hand just in front
  of the edge -> probably floor). Waist motion under load = slip (also att 4's stationary slip).
## Attempt 7 (reset 6, 18:07) — stance ok, grasp too shallow (.0145), stopped for time
- approach -> (.547,-.251); ready; fwd .15 -> (.464,-.273); fwd .1 -> (.371,-.281); pre (.26,-.28) + waist -.6 look
  -> paw (.369,-.266) leg yaw -13.6; waist -.35, align (.29,-.268) -> paw visible (.377,-.265,0) with the arm out.
- 2 cm-deep grasp (.394,-.27,.014) ik .0396 refused; (.385,-.265,.03) ik .0293 -> width .0145 (weak) -> released,
  retreat (up/out -> HOME -> back -.1). FINAL 18:09: robot standing .63 m from the edge, arms home, grippers open,
  tilt 1.0; puppy on the table, box upright; nothing moved by the robot in this attempt.

## FINAL STATE / NEXT STEPS (18:20)
- TASK NOT ACHIEVED. Resets 6/10. Falls 0. Best: grasp + lift + raise to z .365 (att 6).
- Skill library: r1-leg-shaft-grasp-lift (draft) merged -> main aa203c6.
- Handoff: handoff/README.md (this session), older one moved to handoff/README_c4f6c3.md.
- Next: (1) after the raise, walk immediately (run.py carryloop) - no extension, no pause; (2) or test grip
  endurance at z .36 for 10-30 s first; (3) stance so the paw is at x .36-.38, y -.20..-.25 at pregrasp (deep grasp
  reachable).
## 18:14 ENGINEERING NOTICE (operator, reset 7): simulator updated at 18:13:
  (1) held puppy no longer creeps out of a closed gripper on pauses / waist moves (att 4/6 slips = sim artifact, fixed);
  (2) r1_walk now waits until still and corrects distance + heading (residual < 3 cm / 6 deg; reply has turned_deg);
      old walk statistics are STALE; rightward drift per call remains. Reset budget continues (7/10 used).
## Attempt 8 (reset 7, new sim, 18:14-18:18) — stance chase only, no contact
- New layout after the sim update: overhang hind paw (.551,-.068) with the other hind paw only 6.5 cm LEFT (no room
  for the left jaw); lower FRONT paw (.548,-.25) overhangs too, leg yaw +11 (both legs now point +11..+13 deg).
- New r1_walk (self-correcting) observations: turn -90 -> heading ok, moved .15; fwd .1 -> .104; turn 100 -> moved
  .578 (!), heading only ~+88; turn 18 -> +10 deg; turn 20 -> ~+16; fwd .35 -> .32/.379 (paw drifted ~+.2 left in
  the robot frame over 2 walks); turn 12 -> +6; turn +90 / fwd .15 / turn -90 side-step moved the robot RIGHT, not left.
  => small turns still under-turn; turn pairs still displace a lot. Re-model the new walk before planning stances.
- FINAL (18:18): robot standing ~.6 m from the table edge, heading ~24 deg, arms home, grippers open, tilt 1.3;
  puppy on the table near the box (not in it), box upright, nothing on the floor in view. Resets 7/10. Falls 0.
