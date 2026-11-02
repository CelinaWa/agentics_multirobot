# PROGRESS (session 20261102T091205Z_puppy_box_8cef10, sim, 3600 s)
- start: read handoff, library INDEX, last session (ce59d5 fell in pb3 home's last ramp near table). Plan: hold, tuck, observe, layout, pick library skill.
- 09:13-09:15 Layout: box BEHIND puppy (slightly right, rotated ~14 deg: rim corners (0.44,-0.01),(0.36,-0.39),(0.65,-0.46),(0.73,-0.08) pelvis),
  puppy head right, legs to robot, far edge 0.36-0.37, body y -0.06..-0.31, top +0.03; table z -0.0625 after walk (legs straight).
  Edge 0.295 at start (arms tucked). Walks carry=true: 0.09 -> +5.6 cm (incl. warm-up), 0.09 -> 0, 0.12 -> 0, 0.15 -> ~1 cm. Odometry ~0:
  this backend has NO odometry-stopped walking; short walks barely step. Stance stayed at edge ~0.24.
- IK (commanded row): deep press (0.30,-0.17,-0.033) + release (0.465,-0.22,0.26) reachable only at lean 0.1 + yaw cmd 0.5-0.7.
- RUN 1 (09:19-09:22): prep, hand-out, lean 0.1 (tilt .65), via waypoints (hover direct from the hand-out pose refused: jump 2.71),
  hover yaw .7, shifted y -0.18 -> -0.20 after looking. pb3 grasp stopped at command top+0 (stall rule palm > z+0.04) with the palm 4 cm
  above the plush top: fingers base mid .69 / idx .38 -> slipped at palm z 0.24, puppy back on table ~5 cm closer.
- RUN 2 (09:23-09:24): grasp_deep (bb.py: press to top-0.06 regardless): palm stuck at 0.045 (top .028), tilt 6.0, base .55/.45 -> slipped at z .23.
- Pelvis heading drifted +7 deg left over the arm work (13.7 -> 21 deg): re-perceive the box every time (corners moved 5-8 cm in the pelvis frame).
- 09:25 home_safe.py (tucked both arms, waist 0, hands open): tilt 0.8. Stopped grasping (time). Puppy on table, box upright, NOT solved.
- Next idea: the palm never gets deeper than ~top+0.02 at x 0.25-0.30 with yaw .7/lean .1 -> the pinch closes over the top only.
  Try grasping nearer the robot (x = far-0.10, palm can press deeper) with fingers yawed ~-60 (toward the head) so the fingers wrap the
  body's thickness, or ask the operator to place the robot ~0.18 m from the edge (walking did not advance).
- (correction: at 09:25 only ~14 min of the session had passed; continued.) 09:27 reset 1 (operator).
- RUN 3 (09:29-09:31): fingers yawed -27 (perpendicular to the body axis from PCA of body points), palm far-0.07, press gate palm<=top+0.005:
  palm reached top (0.028), but on closing the finger BASE joints stayed 0 (fingertips blocked on the table beyond the far side). Opened, raised.
  Operator asked (3 approaches failed) -> "approved, continue".
- RUN 4 (09:32-09:35): waist -0.25 for a full view; body centre (0.308,-0.278), axis 72 deg -> yaw -18; palm = far-0.096 (fingertips land on the
  body's back), LIGHT press (0, -0.015, -0.03; gate palm <= top+0.012): close base mid 1.00 / idx 0.92 = GOOD WRAP; lift to 0.27 held (0.94/1.35).
  Carry: first 8 cm step tripped the grip check (1.51/1.50), I continued (paws still visible), second step to (0.42,-0.295) -> fingers at fist,
  palm measured (0.39,-0.31,0.295) = at the near wall (~0.41); release point (0.49,-0.31) refused (IK err 0.031). Puppy fell in front of the box,
  on the table, flipped. home_safe tilt 0.76. END: robot standing, arms tucked, hands open, waist 0; puppy on the table outside the box; box upright.
- Lessons: (1) grasp that works here = palm far-0.096 + light press + fingers perpendicular to the body; (2) the box-behind release at ~0.49 is
  out of reach from edge 0.24 (palm stops at ~0.39-0.41): the stance decides it; (3) the grip creeps on the forward carry: carry less far / release earlier.
- (clock: status said 1464 s at 09:36; continued.) Turn -10 deg walk: squared heading, 0 cm forward. Reset 2 (operator did not move robot/box).
- RUN 5 (09:38-09:41) SUCCESS (operator: "The dog is INSIDE the box"): prep, hand aside (0.14,-0.33,0.30), PCA yaw -4, palm (0.261,-0.225)=far-0.096,
  lean .1, staged approach, hover, light press -> base 1.03/0.89, lift .26, carry via (0.37,gy,.27)->(0.465,-0.22,0.26) measured 0.438 (wall .41),
  lean_release (lean .2): palm .483, opened, hand back/up, unlean; home_safe tilt 1.5. Frames: carry 20261102094025bf8d71, open 202611020941017c7215, home 202611020941374c1550.
- Controller code/puppy_box/run_bb.py (chains prep..home; perceive = layout2 rim corners + PCA puppy). Offline perceive test on run-5 frame matched within 1.5 cm.
- RUN 6 (09:43-09:46, reset 3) controller, looked at hover: grasp landed (0.292,-0.199) [2 cm further forward], base .92/.84 then .65/.60 during the lift,
  carry palm .441, lean_release opened at .486 but the open frame shows the puppy hanging by one end on the ROBOT side -> fell to the FLOOR (operator).
  hang.py gate (body x from depth) gave 0.447 vs 0.455 for runs 5/6 at the carry end: does not separate them. Stopped at 09:48 (time). Robot home, tilt 1.5.
- Tally: 1/6 attempts placed (run 5). Grasp+lift held 3/3 with the light-press perpendicular grasp (runs 4-6); release past the wall 1/3.
- 09:52 Skill library merged (box-behind v3, run_bb.py, evidence runs 1-6). Handoff updated. Robot home: arms tucked, hands open, waist 0. Puppy on the FLOOR after run 6 (operator informed).
