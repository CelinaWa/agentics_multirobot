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
