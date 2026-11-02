# Tips (operator-curated 2026-11-01; kind tags curated for the sim -> real export 2026-11-02, see the sim-to-real skill)
- [strategy] Lean <= 0.2 rad, own ramp, never hold a leaned reach (one fall, 20261101T011122Z).
- [strategy] Re-observe after any motion: the pelvis drifts back with arm work; the first frame after start is off by 5-7 cm.
- [strategy] Tuck the idle arm (shoulder pitch +0.45, elbow -0.5) before any waist turn or sideways arm move.
- [sim-only] The puppy is soft now: earlier "this grasp fails" notes were on a rigid model; test again.
- [param] Dex3 opening is ~12 cm; fingers stopping short of the fist = holding something.
- [strategy] (20261102) Pre-curled pinch open shape [0,±0.4,0,±0.5,0,±0.5,0] + palm pressed into the plush to stall lifts the
  soft puppy; default open shape does not. Grip creeps ~0.02 rad/move: carry short and slow, never at reach limit.
- [param] (20261102) Grip check: |index_0| or |middle_0| < 1.3 after closing = held; stop lifting the moment it fails.
- [sim-only] (20261102) g1_model.py `project` was ~80 px off the sim camera; depth deprojection (percep.py) and hand FK agree.
- [param] (20261102) A commanded waist yaw keeps settling during the next arm ramp: IK on the measured waist then misses by
  5-9 cm. Waist yaw tracked 0.07-0.2 of 0.35-0.5 commands.
- [sim-only] (20261102) One operator reply said "dog INSIDE the box" while it lay in front: that was a bug in the sim operator, fixed after that session. Still confirm placement with your own camera before reporting done.
- [strategy] (20261102b) Check reach BEFORE trying grasp variants: plush-pinch-grasp `reach_check` at palm z = top - 0.015,
  6-8 cm in front of the far edge. Resting on the top (not pressed in) = fist every time (4/4).
- [strategy] (20261102b) The controller's start easing can move the robot back ~8 cm (edge 0.27-0.28 instead of 0.15-0.20) and
  drag anything the start-pose hands touch: if the first frame shows a hand on the puppy, ask the operator first.
  [operator 2026-11-02: the sim operator now places the objects only after the robot has settled, and the robot
  may still settle 0.22-0.30 m from the edge: re-measure the edge after start.]
- [param] (20261102b) Waist yaw: command -0.4..-0.6 -> measured -0.14..-0.27; it creeps for seconds. Set the waist, then
  settle (command := measured), THEN put the arm over the object. Never change the waist with the arm over it.
- [param] (20261102b) Short g1_walk commands (<= 0.12 m) barely advance (5 cm total for 0.28 m commanded).
- [strategy] (20261102b) A puppy whose legs overhang the front edge falls off at every slip/press (5/5): ask for it further in.
  [operator 2026-11-02: part of this was a sim bug: the puppy crept toward the edge on its own (floppy legs +
  soft contact), fixed after that session. Still true that a press near the edge can push it off.]
- [strategy] (20261102c) Cross-body carry works: box-side arm far back (sp .8 el -.3), waist yaw cmd .5 -> .8 (meas .41-.60, creeps
  for seconds), pitch .15, wait 3 s, IK on measured, ramp, re-solve on measured, ramp again. 3/3 in the box.
- [sim-only] (20261102c) Sim depth DOES see the clear box's rim top (+0.146 above the table); the pale low-saturation colour
  mask finds it either way (code/puppy_box/boxdet.py).
- [param] (20261102c) The hanging puppy (pinched mid-body, palm down) hangs legs-toward-the-robot, its body ~5 cm behind the
  palm: aim the palm 7-10 cm inside the near wall.
- [sim-only] (20261102c) Operator resets put the puppy out of reach (x 0.46-0.74, legs away) 2 of 5 times: perceive must check x.
- [strategy] [operator 2026-11-02, from the sim ground truth] Run 4 (unattended run.py) did NOT find the puppy out of reach
  after a bad reset: the reset put it 0.31 m ahead (in reach); the controller's prep step then unfolded the right
  arm forward in ~2 s, the hand swept up through the puppy and pushed it 20 cm away (to 0.52-0.6 m ahead), and the
  pelvis rocked back 8 cm. Fix prep before reusing it: raise the hand above the puppy's top (z >= top + 0.08)
  BEFORE moving it forward, slow ramps, and observe before and after prep; never unfold an arm through the space
  above the puppy.
- [strategy] [operator 2026-11-02, from the sim ground truth] In the 02:38 success the LEFT fingertips (index, middle) brushed
  the puppy for ~5 s during the lift: the left arm was raised in front of the body, not tucked. Rule now: the left
  hand must never touch the puppy (such an attempt does not count). Tuck it back and up (shoulder pitch +0.45, elbow
  -0.5) before the first motion and check it is still there before every grasp and lift.
- [strategy] (20261102d) Solve arm IK on the COMMANDED row incl. the commanded waist: SONIC compensates an under-tracked waist with the
  arm, so FK(commanded joints) ~ measured palm (+-1.5 cm); IK on the measured waist missed by ~10 cm. (code/puppy_box/mv.py)
- [param] (20261102d) Waist yaw cmd 0.45 -> meas 0.17-0.22, cmd 0.7 -> 0.24-0.37: set it once before the grasp, keep it to the release.
- [param] (20261102d) Grip survives 4 cm / 3 s lift steps and 5 s arm-only carry steps; lost on a 10 cm lift step and on a waist swing.
- [strategy] (20261102d) g1_hand_shape with no shapes (home) restores the flat default: set the pinch shape again before every grasp.
- [param] (20261102d) Robot drifted 0.175 -> 0.399 m back from the table over ~25 min of arm work; re-measure each run.
- [strategy] (20261102d) Don't run offline tests that write the live state file (pb3_state.json) between steps.
- [sim-only] [operator 03:18, answering your question] In the 03:14-03:16 success your LEFT hand never touched the puppy (sim ground truth: right-hand contacts only). Clean right-hand-only success.
- [param] (20261102e) Low table (z -0.05 pelvis): pressed grasp out of reach at waist <= 0.55; waist cmd 0.7 (meas .30-.37) or lean 0.1
  (tilt 2.5, pelvis slid 1.2 cm) made it reachable. Check reach for the pressed pose on the commanded row BEFORE hovering.
- [param] (20261102e) Grasp y = blob centre - 3 cm from the head (not 6): the press/close pushes the plush toward its head; 6 cm
  closed on the rear third 2/2 and the plush hung head-down; 3 cm closed mid-body 2/2 and both went into the box.
- [strategy] (20261102e) Shallow press (palm stall top+1.7 cm) slid along the body during the lift: deep press matters.
- [param] (20261102f) Lateral reach limit (plush mid-body |y| >= 0.30 on the right, right hand): commanded-row FK is OFF by +4.5..+8.7 cm
  in z (palm above its command) and under contact the arm deflects ~5 cm back/right; the palm only reaches the plush top -> fist
  4/4. Check the MEASURED free-space palm at top-0.03 before pressing; if not reachable, ask for the plush closer. Do not press:
  2 of 4 presses pushed it off the front edge.
- [strategy] (20261102f) A LEFT waist yaw command helps the right hand reach to the right (swings the right shoulder forward); waist roll
  +0.15..0.2 lowers the right shoulder, standing tilt 1.5-3.3 deg in its own 4 s ramp.
- [strategy] (20261102f) boxdet.locate_box can be 0.2 m off for a box on the left; use plane_hit on the 4 floor + 4 rim corners in the colour image.
- [sim-only] (20261102f) The first two frames after start were taken BEFORE the operator placed the objects (empty-ish scene, edge 0.30):
  wait until tilt settles (~1 deg) and observe again before measuring the layout.
- [sim-only] (20261102f) The scripted operator answers help requests generically ("approved, go ahead", distance to the dog); it
  does not reposition objects on request; resets put the puppy back roughly at its start (0.46-0.52 m from the pelvis).
- [param] (20261102f) Right hand into a box on the LEFT: the palm stalls at y ~+0.04 pelvis even with waist cmd 1.0 (meas 0.49); commanded
  targets beyond that are IK-feasible but not tracked. Hover the empty hand at the release point first and measure.
- [param] (20261102f) pb3 grasp/lift with only ONE finger stalled (middle 0.97, index fist) held through lift + carry (middle 0.8-1.05).
- [strategy] (20261102g) Before ANY motion, measure edge/puppy/box and run reach2.best for grasp AND release: here (edge 0.26, box
  behind the puppy at x 0.56-0.84) both were out of reach by 5-29 cm. The operator then found the robot had slid
  ("stance slide") and disarmed. Edge > 0.22 at start = suspect a slide: ask before moving.  [outdated 2026-11-02: setup changed, see the operator note on stance and walking]
- [param] (20261102g) g1_walk 0.12 m carry=true advanced only ~3 cm each (3 walks); the legs straighten after a walk (pelvis
  +2.5 cm, table 3 cm lower in the pelvis frame). g1_crouch(0, forward_rad .05, arms_home=False) restores the stand but
  slides ~1.7 cm back.
- [strategy] (20261102g) Box BEHIND the puppy: release at the far reach limit. cmd (0.495, z .21) -> palm 0.453; raising the hand
  (z .30) barely helps (0.461); lean 0.1->0.2 in its own ramp gave palm 0.488 (tilt 3.6). Gate the release on the
  MEASURED palm >= near wall + 0.03 and unlean right after: puppy fell fully inside (1/1).
- [strategy] (20261102g) reach2.best: add random seeds near the reach limit (fixed seeds gave 2-5 cm false misses).
- [param] (20261102h) Two operator-approved walks (0.12, 0.15 m, carry=true) from edge 0.32 gave +4 cm and +0 cm: near a table,  [outdated 2026-11-02: setup changed, see the operator note on stance and walking]
  g1_walk does not fix the stance. Compute the release reach (box near wall + 0.075) right after start; if > ~0.50 m
  pelvis-frame, the box-behind layout is infeasible: say so early and do not touch the puppy.
- [strategy] (20261102h) pb3 prep with the puppy straight ahead (x .34-.42, y -.14..+.13) bumped it (rotated ~40 deg toward the edge):
  the right arm's forward swing (sp .9 -> .1 with elbow -.9) passes through the space just in front of/above the puppy's
  near side. Raise the hand at the side (y <= -0.30) before swinging forward when the puppy is near y -0.15.
- [param] [operator 2026-11-02, from the sim ground truth] Stance and walking: the robot starts ~0.3 m behind the table edge so
  its hanging hands (they reach ~0.26 m ahead of the pelvis at table height) clear the edge. g1_walk now stops on leg
  odometry: forward walks of 0.1 / 0.2 / 0.3 m moved 0.10 / 0.22 / 0.28 m (GT), carrying 0.15 -> 0.12; the reply's
  feedback.odometry_m says how far it went. Before this, short walks barely moved the robot. To step up: tuck both
  arms back and up, g1_walk forward with carry=true by (measured edge distance - 0.20 m), measure the edge again.
- [strategy] [operator 2026-11-02, from the sim ground truth] Your table_edge() (1st percentile of x at table height) reads your
  own hanging hands as the edge: standing, they hang at table height 0.20-0.26 m ahead of the pelvis. Seed 27: it said
  0.20 m, the edge was 0.33 m; with the arms tucked it said 0.23 for a true 0.25. Tuck the arms before measuring, or
  ignore points within ~10 cm of your hands (forward kinematics) and take the median over y-bins of each bin's nearest
  table point (the table spans the view; the hands cover a few bins). On the real robot the same will happen.
- [strategy] (20261102i, 89afad) Cross-body (box LEFT, plush RIGHT, right hand): waist yaw cmd 1.1 + roll +0.15 fixed; then a palm-yaw
  swing 15 -> 75 at the release point moved the hanging body from the box's right wall to the interior. 1/1 in the box.
- [strategy] (20261102i) reach2.best random seeds can return an overhead branch (shoulder pitch -2.97). FK-check the joint path
  (lr.path_min_z) and the jump before any ramp of an IK result; use a raised via-config for jumps > 1.6 rad.
- [sim-only] (20261102i) The first frame after start was tilted (4-5 deg) and read the edge 0.20 while the operator said 0.33.
- [param] (20261102j, 170401) g1_walk straightens the legs (pelvis .78 -> .80): the pressed grasp went out of reach on a low table.
  g1_crouch(0, forward_rad .05, arms_home=False) restores .78 (slid 2.7 cm back); re-perceive, then hover reached at waist .7.
- [strategy] (20261102j) Same-side layout (box right, head left): pb3 chain 2/2, one fully unattended (code/puppy_box/run_pb3.py left).

## Session 8cef10 (sim, 2026-11-02)
- [param] g1_walk in this session did not stop on odometry: carry=true walks of 0.09/0.09/0.12/0.15 m advanced 5.6/0/0/1 cm (the first
  includes the warm-up). Do not count on stepping up; ask the operator to place the robot early if the edge is > 0.22 m.
- [strategy] pb3 step_grasp breaks out of the press at the first step whose palm is > 4 cm above the command, i.e. it can stop at command
  top+0 with the palm resting ON the plush (fingers then close flat, base joints < 0.7) -> slip on the lift. Require palm z <= top before closing.
- [param] At edge ~0.24, lean 0.1, waist yaw cmd 0.7, palm x 0.25-0.30 the palm stopped at top+0.017..0.04 even with command top-0.06 and
  tilt rose to 6 deg: not enough reach/force to press in from this stance. Two slips at palm z 0.23-0.24.
- [strategy] Pelvis heading drifts several degrees with waist-yaw arm work (+7 deg here): box corners move 5-8 cm in the pelvis frame; re-perceive.
- [strategy] Safe end pose at the table: both arms tucked (sp 0.45, el -0.5), waist 0 first with the hand high (code/puppy_box/home_safe.py).
- [strategy] Plush pinch that wrapped (8cef10 run 4): fingers perpendicular to the body axis (PCA of body points, full view), palm
  ~0.10 m short of the far side so the pre-curled fingertips land on the body's back (not the table), LIGHT press (command top..top-0.03,
  stop when palm <= top+0.012): base joints 1.00/0.92, held through a 20 cm lift. A deeper press at far-0.07 left the base joints at 0 (tips on the table).
- [param] From edge ~0.24 with lean 0.1 / yaw cmd 0.7 the loaded palm stops at x ~0.39 at y -0.3 (release 0.49 refused): a box near wall at 0.41 is out of reach.
- [strategy] Box behind at the reach limit (8cef10 run 5 success): carry straight forward at the grasp y to cmd 0.465 (measured 0.438 = wall+0.03),
  then the gated lean release (lean 0.2, measured palm 0.483) - open, hand back/up, unlean. Same sequence failed in run 6 when the lift weakened the
  grip (base joints 0.65/0.60): require base joints >= 0.8 at the END of the lift before carrying, else lower and re-grasp.
- [param] A g1_walk turn of -10 deg (forward 0) turned -7.9 deg with no forward step in this session.

## cab719 (2026-11-02, sim)
- [strategy] Never chain grasp after hover without looking at the hover frame: the one chained run closed over the neck and dropped it.
- [strategy] End pose at a close stance (edge <= 0.22): arms DOWN beside the hips (sp 0.4, el 0.3), not sp 0.5/el 0.6 (hands at
  table height 0.2 m ahead -> hit the edge -> robot stepped 0.17 m by itself and was disarmed).
- [strategy] Read g1_status events after every step: a disarm + auto re-arm can happen mid-script without any tool error.
- [param] g1_walk(forward 0, turn T) turns in place (odometry 0), measured 30-60 % of T; crouch restore adds +2-4 deg left.
- [param] crouch(0, forward 0.05) slides the pelvis ~2.5 cm back; forward 0.08 kept the walk's advance (edge .236 -> .221).
- [strategy] Edge line fit: a +angle (left side farther) means the robot faces LEFT of the table normal; turn right (negative).
- [strategy] Waist yaw to the right (cmd -0.5) barely moved (meas ~0) and the arms compensated: the tucked left arm swung forward.

## Session 0878bd (sim)
- [strategy] Box on the left, puppy on the right: look with BOTH arms tucked and the waist turned right (cmd -0.7); a raised right
  arm hides the scene. Measure the box from a waist-0 view (the turned view clips it).
- [param] Waist roll 0.25 + yaw 0.45: tilt 5.4 standing, 7.5 when pressing far right-low; roll 0.15: 1.5 standing, <= 6.8 pressing.
- [param] Depth under-reads the plush top by ~3.5 cm (palm stalls at depth top + 0.035).
- [strategy] A stall rule "palm > cmd + 2.5 cm" fires falsely on a step that also turns the wrist yaw: also require the palm to stop descending.
- [param] Held soft plush can let the fingers creep to 1.30-1.40 rad (empty fist 1.52-1.57): grip threshold 1.45 on the carry.
- [strategy] When the hand is left at a low pressed command (pressing into the plush), FK path checks fail at their start point:
  first set the arm command to the measured joints, then raise straight up in small moves, then go home.
- [strategy] (2a8660) Head-right puppy on the right: perceive with BOTH arms tucked and waist -0.7 (whole puppy visible) and take
  the PCA centre; the raised-arm pb3 view cut the head off and put the grasp 10 cm toward the tail (2 empty fists).
- [param] (2a8660, sim) Finger yaw vs lift hold: diagonal (-57.6) held to z .36; across a y-lying body (-12, +5) slipped at z .18-.24.
  Forcing -45 made the palm ride 4 cm above its command at hover (empty fist). Gate the close on palm z near the plush top.
- [param] (2a8660, sim) Waist 0.9/0.15 + arm over y -.30 -> tilt 6.24; 0.7/0.15 reached grasp and release (.43,.10,.30), tilt <= 4.7.
- [strategy] (2a8660) Carry the plush at palm z .35 and release at .30: the body hangs ~15 cm; palm-yaw swing 15->50->75 moved it
  from the right wall onto the interior (depth 100 %), then open at once.
- [strategy] (eed312, sim) Box BEHIND a puppy whose back is 3-4 cm from the near wall: the Dex3 finger BASES sit ~8 cm ahead of the palm
  frame (FK with hand joints: M.fk(j29, hand=...)), so a fingers-forward pinch puts them on the rim once palm > wall-0.09, and at
  palm < wall-0.10 the fingers lie on the crest (no wrap). Check finger links vs the wall by FK BEFORE closing and before lifting.
- [strategy] (eed312, sim) Never raise a hand whose fingers may be over the box rim with joint-space guesses or by moving back: the
  open fingers dragged the box 20 cm. Raise straight up in Cartesian steps; if IK refuses (branch jump), ask the operator.
- [param] (eed312, sim) Crest grasp that lifted: palm cmd .335 (meas .324) = body crest x, wall-0.10, DEPTHS .01,-.01 GATE .02,
  base .87/.86, lift to .27 held (index crept to 1.35, middle .85).
- [param] (eed312, sim) Gated lean release opened at palm .467 vs wall .426 (+4 cm): body fell back out. Need palm >= wall + 0.07;
  release where the slanted near wall is closest to the robot (run_eed.py picks that y).
- [param] (eed312) g1_walk carry=true at edge .25-.30 advanced 0 cm for 0.07/0.10 m commands (odometry -0.02); stance.py's 3 walks .304->.254.

## ad5d0f (sim, 2026-11-02)
- [param] g1_walk carry=true at edge 0.27 advanced ~0 (four walks, odometry -0.004 on the last). [param] g1_crouch(0, forward_rad 0.1
  then 0.2, arms_home False) brought the edge 0.265 -> 0.245 -> 0.228 with tilt <= 2.9: ~2 cm per 0.1 rad, not 6.
- [strategy] Pinch shape: index_0 0.5-0.9 while middle_0 is at the fist (1.52) = index blocked by the thumb, NOT a hold (2/2 empty).
  Count a hold only if both base joints stop short (< ~1.4), and confirm with a 3 cm raise + look before carrying.
- [param] Right hand, waist yaw cmd 0.7 / lean 0.1: finger links sit 0.11-0.12 m ahead of the palm frame with yaw 0; wrist yaw cmd -65
  realised ~-40 (links 0.06-0.09 ahead). Check finger-link FK (code/pb_ad5/fk.py) vs a box wall before descending.
- [strategy] code/pb_ad5/prep_side.py raises the right hand at the side (shoulder roll -0.45 before the elbow and the forward swing):
  passed a puppy lying straight ahead without touching it.
- [strategy] (ad5d0f) Across-body light press often ends with the finger base joints at 0 (fingers lying on the plush's back); do NOT
  give up: a 3 cm raise with the hand still closed rolled the plush into the hand (paws up) 2 of 3 times, then it held through a 25 cm
  lift and a 20 cm carry (base .6-.95 / 1.33-1.39).
- [strategy] (ad5d0f) Release with the FLAT default hand shape (g1_hand_shape with no shapes) and raise straight UP after a 2 s wait;
  the pinch 'open' shape stays curled, and moving the hand back 1 s after opening put the puppy on the floor (1/1). Flat + up: inside (1/1).
- [param] (ad5d0f, sim) At waist cmd 0.7 / lean 0.1 the measured palm sits ~+0.01..+0.05 x, +0.02 y from the commanded-row target;
  with a box wall 4-5 cm behind the plush, palm = far - 0.096 keeps the open finger links 2-3 cm short of the wall.
- [param] (ad5d0f) The robot crept ~7 cm closer to the table and ~11 deg left over 35 min of arm work (edge .278 -> .198).
- [strategy] (996d97) Same-side layout (puppy centre-left head left, box right): library same-side v5 `run_v5.py` placed 3/3. Look at the
  hover frame; shift the grasp 3 cm toward the tail (all 3 hovers sat over the front third).
- [param] (996d97, sim) The palm-down pinch press stalls at palm z .038-.045 (depth top +.021-.028), tilt 5.7-6.9; a deeper command
  either tips over tilt 7 or is IK-refused. Closing at that stall gives finger bases .79-.90 = enough with a 3 cm / 4 s lift (3/3);
  a 4 cm / 3 s lift slipped at z .115 (1/1).
- [param] (996d97, sim) A lifted plush often hangs from the MIDDLE finger alone (base 0.7-1.15, index at the fist 1.5+); fingers
  1.31-1.37 still held it. Empty fist reads ~1.53: use 1.45 as the grip-lost threshold during the carry.
- [param] (996d97) Over ~25 min of arm work the robot drifted back (far edge .36 -> .41) and the heading crept ~9 deg left;
  g1_walk 0.05 carry=true moved -0.6 cm. At ~10 deg the box left view B and perceive returned garbage (hover refused safely).
- [strategy] (996d97) pb3/controller3 perceive raises the right arm into the view: when the puppy lies close (far edge < ~0.35)
  the arm hides most of it (blob 4 cm wide). Sanity-check puppy width >= 0.25 m and box near wall 0.18-0.40 before hover.
