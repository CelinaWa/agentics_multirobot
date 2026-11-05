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
