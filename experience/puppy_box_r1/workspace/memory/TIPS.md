# TIPS — puppy_box_r1 (r1sonic sim, session 20261105T194732Z) — tags: [strategy] [param] [sim-only]

- [strategy] Never send request_operator kind=help: no reply (240-300 s timeouts); the scene appeared to shift
  ~0.12 m (operator later said it was the robot drifting, not a slide). kind=reset is answered in 150-200 s and the
  reset pre-positions the puppy with a hind paw 6 cm over the edge (stated by the operator).
- [strategy] Backward r1_walk does not work (0, -0.11, +0.09 fwd, +0.05 fwd for -0.25..-0.6). Never overshoot the
  approach; approach in 0.15-0.2 m commands and re-measure. If too close, the only fix is a reset.
- [param] forward cmd 0.2 -> 0.02-0.10 m near the table, 0.25 -> 0.15-0.35 m; turns: 10deg cmd ~ deadband (1.5deg),
  15 -> 14, 17 -> 11.5, 25 -> ~15-20; 90deg turns drift 0.25-0.3 m along an arc. Walk resets head pitch to 0.
- [strategy] Square stance straight in front of the start layout is best: hind paws were already right of centre.
  Lateral moves by turn-walk-turn overshoot forward; prefer a heading change (turn) far from the table.
- [param] r1_grasp_paw worked with the hanging hind paw at x .37-.38, y -.03 (side=right, clamp .034, lift .028)
  but the grip slipped within ~1 min with the arm held still. Lift/carry IMMEDIATELY and watch the width.
- [param] Right-hand reach (preview, use_waist): x<=0.60 at y -0.2 (z .25-.35), <=0.55 at y -0.3, <=0.50 at y -0.4;
  without waist x<=0.50. Box-behind layout: near wall is edge+0.22 -> with edge 0.3 ahead, near wall at 0.52 and
  centre at 0.66 = at/over the reach limit. Box-behind needs a plan for carrying the hanging puppy ~0.2 m forward.
- [strategy] Whole-body coupling: an extended arm with waist roll/yaw shifted the robot 6-12 cm sideways relative to the
  scene and the measured waist yaw overshot the command 2x. Re-measure targets with the arm already out.
- [strategy] Going HOME from a hand above the table drove the hand into the table top (tilt 7deg). Route: up, back in
  front of the edge, then down.
- [sim-only] Depth sees through the clear box walls except the rim; use rim pixels (top edge) for box geometry.
- [strategy] INCIDENT: r1_grasp_paw made the robot FALL (tilt 81) on its 2nd use (paw ~0.38-0.39 ahead, slightly
  right). 1 success / 1 fall. Before calling it: stand with paw at 0.30-0.35 per its docstring, let the robot settle
  (observe twice, tilt < 1), and be ready that a fall ends the session (operator did not answer the fall report).
- [strategy] OPERATOR FACT: r1_grasp_paw is a FLAT table-top grasp primitive - wrong for a leg hanging past the edge
  (it knocked the puppy off twice and tipped the robot once). Use r1_move_hand in front of the hanging paw with
  fingers (0,0,1) and approach ~(1,0,0.15), then r1_gripper width 0 (TASK.md recipe).
- [strategy] A reset is answered in 50-260 s; help requests usually time out (the fall report was answered after
  a second request, 463 s).

# Session 20261105T213131Z_puppy_box_r1_08803f (r1sonic sim, where: sim) — new and re-checked rows
- [strategy] held on sim 2026-11-05: help request "position hind leg to overhang" is answered in ~3 s now; reset in
  ~7-8 s and the reset itself pre-positions the overhang (all 4 legs stick out horizontally ~6 cm, paws at table+5 cm).
- [param] where: sim. Walk this backend: fwd cmd 0.2 -> 0.045-0.142 (mean ~0.09), 0.15 -> 0.063-0.203, 0.12 -> 0.065-0.122,
  0.1 -> 0.062-0.122, 0.3 -> 0.10/0.25; lateral drift 0-0.12 m per call, mostly to the RIGHT. Backward WORKS now:
  -0.2 -> 0.228/0.263, -0.12 -> 0.218 (overshoots). Turns: 12 -> ~6.6deg, 15 -> ~3.8deg (+0.10 m right drift),
  28 -> ~20deg, 30 -> ~21deg (+0.12 fwd). Walk now keeps head pitch. (old row "backward walk does not work": no longer)
- [strategy] where: sim. r1_walk 'pelvis' odometry is consistent with perception (table edge at the same world x in
  every frame, +-1 cm) -> keep a world frame from it; IMU yaw does NOT reflect turns.
- [strategy] where: sim. Approach: two 0.2 cmds then <=0.12 cmds, re-measuring the front paw each time; stop with the
  target paw ~0.27-0.35 ahead. Head pitch 0.6 sees paws down to ~0.30 ahead; closer, use visible leg points + 6-7 cm.
- [strategy] where: sim. Paw-ball pinch WORKS 4/4: r1_move_hand pre-grasp 8 cm short, approach (1,0,0.1), fingers
  (0,0,1), open; final approach (1,0,-0.1) with the ball centre 0.5-2 cm past the grasp point; r1_gripper 0 -> width
  0.032-0.035. Right hand comfortable for paws at y -0.15..-0.20; y > -0.05 needs approach (0.8,0.6,0.05) and every
  reach yaws the waist (measured, command 0) and shifts the pelvis 1-6 cm -> re-measure after each arm move.
- [strategy] where: sim. The pinch does NOT survive a real lift: slipped at z 0.29 (IK jump), 0.29, 0.19, 0.12 (puppy
  pressed on the box wall); held to 0.20 once. Cause: the ball is pinched with the leg leaving over the fingertip edge,
  so the hanging weight pulls it out (friction-limited). Needs a stopper grasp (pads across the LEG, ball outside the
  pads) - not reachable in the overhang layout (side approach ik .04-.08).
- [strategy] where: sim. While holding, NEVER let IK restart: r1_move_hand without approach picked a new arm
  configuration (wrist +0.7 rad) and dropped the puppy. Use code/tools/contik.py (continuous IK seeded by the last
  solution, backend point = local site - 0.107*approach) and r1_move_joints, 2-3 s per 4-5 cm step.
- [strategy] where: sim. r1_grasp_paw: 7 calls over two sessions -> held 2, FELL 2 (tilt 81, 63; the 2nd at a stance
  1-2 cm from a success, waist yaw 1.5 after), knocked the puppy off 3. DO NOT CALL IT. When it held (clamp 0.057,
  lift 6 cm) the hold survived a 0.2 m walk.
- [strategy] where: sim. After r1_grasp_paw the backend's stored gripper command stays 0.07: the next r1_move_joints
  re-sent it and OPENED the gripper. After any primitive, call r1_gripper(width 0) before other motions.
- [strategy] where: sim. Retreat route: hand straight back at its height to x ~0.2, then HOME joints. HOME directly
  from a hand over/near the table hit the edge twice (oblique stance; edge 0.13 ahead) and once swept the puppy off.
- [param] where: sim. Box: 39 cm along y x 28 cm deep, rim z = table + 0.145 (pelvis ~+0.10..0.12); front wall at
  edge + 0.21..0.25. Depth sees the rim/top plane; puppy top (table+0.12) contaminates rim-height masks near the wall.

# Session 20261105T222431Z_puppy_box_r1_e0ac0c (r1sonic sim, where: sim) — new rows
- [strategy] where: sim. The agent workspace was READ-ONLY again (bwrap ro mount; Write/Read tools denied, single-file
  `cat` works). Work in /tmp/claude-1001/ws_<id>; repoint hard-coded paths (sed) first; skill-library/.git is ro too.
- [strategy] where: sim. VERTICAL-finger pinch of a paw ball is TORQUE-limited, not just friction-limited: the leg
  (~8 cm, stiff hip) is a lever that twists the ball out between horizontal pads. This session it slipped at the
  first 3-4 cm of lift twice (deep pinch w .030 incl. leg; shallow pinch w .028). Do not plan a full lift on it.
- [strategy] where: sim. HORIZONTAL-finger "pivot" pinch (fingers (0,1,0), approach (1,0,0), pads left/right of the
  ball) lets the ball rotate about the pinch axis -> hanging load is shear only. 1 try: w .0317 at grasp, held to hand
  z .086 (w .031 -> .0255 -> .0218 creeping), then lost by my IK flip (not a slip). UNTESTED above z .09: test first.
  It needs one finger between the paws of a pair (centres ~5 cm apart, balls ~5 cm wide -> ~0 gap): works only when
  the pair is splayed (>= 6 cm) or for a paw whose neighbour sits >= 3 cm further back/forward.
- [strategy] where: sim. Backend IK picks the wrist branch from its seed: from a vertical-finger pose, a goal with
  fingers (0,1,0) ends at wrist roll -1.87 (limit) = fingers still vertical. First r1_move_joints the wrist roll to
  ~+0.1..+0.25 (code/puppy_box/pivot_pinch.py SEED), then move_hand. Even then a far goal (x>=.40 at y-.10, or
  x .35 y -.08 z .01) can flip back to the vertical branch -> PREVIEW EVERY MOVE and refuse finger-axis changes or
  joint jumps > 0.35 rad (pivot_pinch.safe_move does this). A flip while holding = drop (happened once).
- [param] where: sim. Right-hand reach with fingers horizontal/vertical, approach (1,0,0): good at y -0.15..-0.20,
  x .30-.39 (servo ~0); at y > -0.08 the shoulder roll sits at its limit (0.177), the y servo saturates (~0.047)
  and the hand ends 1.5-3.5 cm RIGHT of the goal; the waist yaws +0.12..+0.36 rad (command 0) and the scene shifts
  +1.5..3 cm in y per reach. Keep the target paw at y <= -0.12 BEFORE reaching.
- [param] where: sim. Reaching to x ~0.40 moves the pelvis back ~2 cm: a paw measured with the arm home is ~2 cm
  further after the reach -> re-measure with the arm extended (pre-grasp), then the final move.
- [param] where: sim. Head pitch 0.6 sees table-height points only at x >= ~0.34-0.35 (arm home); with the arm out
  the waist yaw turns the camera LEFT and paws at y -0.15..-0.20 drop to the image's bottom-right corner or out.
  Stand so the target paw is at x 0.38-0.42 if you want to watch the grasp.
- [param] where: sim. Camera pose with a yawed waist gives z biased +1.5..+2 cm (table z -0.013 vs -0.033): use z
  from an arm-home (waist ~0) frame.
- [strategy] where: sim. r1_move_joints waist_yaw 0.3 (to face a target left of the right shoulder) made the backend
  IK WORSE (straight approach 10 cm error at y +0.015) -> do not use the waist to extend the right arm leftwards.
- [param] where: sim. Walks this session: fwd 0.2 -> .061 .091 .115 .103 .045 .127 .11; 0.12 -> .098 .053 .051;
  0.15 -> .084; 0.1 -> .035 .025 .088; back -0.1 -> .183, -0.08 -> .226. Every forward walk drifted the robot RIGHT
  2-6 cm (scene +y); turn 20 cmd -> edge yaw change ~10-13 deg plus 0.03-0.13 m forward and up to 0.17 m sideways.
- [strategy] where: sim. The pair of front (or hind) legs is the obstacle for every side grasp; the operator's
  overhang reset keeps them parallel ~5 cm apart. My failed grasps splayed / pushed the legs (and the paws ended on
  the table) - re-measure after every contact.
- [strategy] where: sim. PIVOT PINCH VERIFIED ONCE TO z .27: fingers (0,1,0), approach (1,0,0), grasp at the paw-ball
  centre (lower front paw after the pair was splayed ~3.5 cm), width .0315; lift with approach (1,0,0) to z ~.11
  then (1,0,0.3)..(1,0,0.4), x drifting back .38-.40, y moving right -.10 -> -.15: width stayed .031 the whole way.
  1/3 lifts reached >= .20 (others: slipped at .06-.08 when the neighbour paw sat only 2.5 cm behind; lost to my IK
  flip at .09). The puppy rolls onto its back while its body stays on the table.
- [strategy] where: sim. Carrying by walking with the pivot pinch FAILED 1/1 (dropped during a 0.19 m walk with the
  hand at z .27; the body was dragged on the table). Next: shorter walks (<= 0.1) with re-checks, or lower the hand
  to ~.15 before walking, or grasp from a stance already close enough for the box (box front wall = edge + .21).
- [strategy] where: sim. Splaying the pair: an OPEN gripper advanced onto the right paw of the pair (wedge) pushed it
  ~3.5 cm back and it stayed; closed-gripper pushes gave only ~2.5 cm (sprang back). Need >= 3 cm of x offset
  between the pair before the pivot pinch on the nearer paw.
- [strategy] where: sim. After a drop the puppy lies on its back at the edge with legs up, right where the arm swings
  down on HOME (hit it twice, tilt 5-7 deg, hand stuck at (0.22,-0.30,-0.02)). Free the arm by moving the hand UP
  (z .12), then OUT to y -0.40 (x .14, z .06), then HOME.

# Session 20261105T232821Z_puppy_box_r1_c4f6c3 (r1sonic sim, where: sim) — new rows
- [strategy] where: sim. Workspace READ-ONLY again (skill-library/.git too): skill_library_merge cannot see commits
  made in a copy -> merged:false. Ask the operator for a writable workspace, or hand the patch over.
- [strategy] where: sim. help "position a hind leg to overhang" now answers in ~30 s; every reset gives the SAME layout
  (head right, box behind, overhanging hind paw ~(0.55,-0.02) from the start, other hind paw 11 cm right/5 cm back).
- [param] where: sim. r1_walk TURNS ARE ARCS, not in place: left turns pivot about (0.05,+0.31) m (robot frame), right
  turns about (0.02,-0.52) m; actual angle 0.86-0.87 x cmd for 45-100 (cmd 25 -> 14deg, 15 -> deadband/odd).
  A 90deg turn moves the pelvis 0.33-0.53 m. Back cmd -0.1 -> 0.12-0.24 m. Fwd 0.1 -> 0.003-0.1 (deadband), 0.15 ->
  0.06-0.15. Every call drifts 0-0.06 m right. NEVER combine forward_m and turn_deg (-0.25/-45 moved 0.96 m).
- [strategy] where: sim. Use r1_walk reply 'pelvis' odometry as the world frame (x = table normal from a square start)
  + the edge heading (code/puppy_box/edges.py or skill r1-walk-approach) and re-plan after every call (walkplan.py).
- [strategy] where: sim. Do the big lateral shifts with turn pairs (+60/-50) from >= 0.8 m; plan for the turn arc.
- [strategy] where: sim. Paw at x < 0.34 is invisible at head 0.6: r1_move_joints waist_yaw -0.35 -> visible; the
  pelvis counter-rotates (IMU yaw +3..6 deg) so MEASURE AND GRASP IN THE SAME waist configuration.
- [param] where: sim. Pivot pinch at low z (-0.03..0.0): IK err .019-.027 at x .30-.38, y -.17..-.25 (approach (1,0,0));
  x .39+ or y < -.30 unreachable. Grasp point 0.5 cm short of the ball centre if the full depth is > 0.03.
- [strategy] where: sim. Pivot pinch HOLDS THE FULL HANGING PUPPY statically (z .285, width .031 constant) but is
  friction-only: dropped by (a) an approach change (1,0,.6)->(1,0,.8) + 3 cm servo jump, (b) r1_walk 0.089 m while
  holding. => do the place WITHOUT walking, or switch to a stopper grasp before walking.
- [strategy] where: sim. After a drop the puppy may land on its back with legs up next to the box: legs up = natural
  stopper candidates, but the leg-1 shaft grasp was blocked 2x (hand stopped 2 cm high, z servo maxed -0.05).
- [strategy] where: sim. HOME from a hand at (0.22,-0.30,0.12) next to the puppy hit something (tilt 8.3 deg): ALWAYS
  route up -> out (0.12,-0.40,0.06) -> HOME (worked) when anything is near the hand.
- [strategy] where: sim. Box-behind layout: from the closest grasp stance (paw x .28) the box interior starts at x .68;
  right-hand IK with waist reaches x .54 -> the carry needs ~0.15 m of walking, which the friction pinch does not
  survive (0/2), nor does lowering the puppy onto the table (0/1). Next: a stopper grasp, or ask whether a
  box-beside layout / operator help is allowed.
