# PROGRESS — puppy_box_r1 (session 20261105T222431Z_puppy_box_r1_e0ac0c, sim)
NOTE: agent workspace READ-ONLY again (bwrap ro mount; Read/Write tools denied). Working copy: /tmp/claude-1001/ws_e0ac0c
(code/, memory/, handoff/, kin/ = backend r1_kin.py+xml, pylib -> ws_08803f/pylib with mujoco). Single-file `cat` works.
## State (t=11 min)
- t0 frame 20261105142718a8eaff (head 0.5): edge 0.63, table z -0.045; puppy head RIGHT, paws AT the edge (not overhanging);
  hind paws y +.03/-.02, front -.145/-.196; box behind (x .85-1.14, y -.29..+.11).
- walks 0.2 -> .061, .091, .115; robot drifted ~0.17 m RIGHT (scene moved +y) although odometry y ~0.
- frame 20261105143229a4ddc9: edge .432, front paws (ball top) R (.426,-.015,.026) L (.408,.036,.010); right-front leg top
  z .022-.030 from x .426 to .50 (leg as tall as the paw ball, ~3.3 cm); ball ~5 cm wide, pair spacing ~5 cm (touching).
## Key insight (planning)
- Previous pinch = fingers vertical, ball near the fingertips -> the hanging body's lever (leg ~8 cm) twists the ball
  out (friction torque 2*mu*N*r small). Plan: DEEP pinch - fingertips ~4 cm past the ball centre along the leg so the
  pads clamp ball + leg shaft (leg as tall as the ball) -> the torque is resisted over ~7 cm.
- Stopper across the leg does not fit: pads 5.6 cm wide vs ~6 cm leg shaft (ball to body).
## Attempt 1 (22:35-22:50Z, initial scene + operator overhang), NO grasp made
- Walks drift: robot drifted ~0.17 m right on the first 3 forward walks; a 20deg turn moved the pelvis 0.17 m right.
- Reaching the right hand to y > -0.05 (front paws ended near y 0): every reach yaws the waist (+0.15..0.33 rad measured,
  command 0) and the scene shifts +1.5..3 cm in y per move; servo y correction saturates at ~0.047 -> hand 1.5-3.5 cm
  right of the leg. Chased 3 times, never got the leg between the pads. Right shoulder roll sits at its limit (0.177).
- waist_yaw command 0.3 makes the backend IK WORSE (straight approach 10 cm error) -> do not use.
- Camera z is biased +1.5..2 cm when the waist is yawed (table z -0.013 vs -0.033).
- Lesson: the target paw must be at y <= -0.08 (pelvis) BEFORE the first reach (it shifts +3-5 cm left over the reaches).
## Attempt 2 (reset 1, 22:49Z-) layout: head RIGHT, box BEHIND, overhang by reset
- Approach: walk .2->.103, .2->.045, .2->.127; front paws then at y -.15/-.20 (square, edge .425) = good for the
  straight approach. Pre-grasp at y -0.19 needs no servo (0.5 cm error); x>=0.40 targets shift the pelvis back ~2 cm.
- Vertical-finger pinches: (a) deep (grasp pt 3 cm past ball centre) held w .030 -> slipped at the FIRST 4 cm lift;
  (b) shallow pinch missed once (ball 2 cm further than measured: pelvis moves back as the arm extends -> measure
  with the arm extended), then held w .028 -> slipped at z .035. Vertical pinch = torque-limited.
- HORIZONTAL-finger pinch (fingers (0,1,0), approach (1,0,0)) on the LOWER front paw (legs had splayed, gap >= 2 cm):
  needs wrist roll ~+0.2 (set it first with r1_move_joints; IK from a vertical-finger seed hits the -1.87 limit).
  Closed -> w .0317; lift z .048 (w .031), z .086 (w .0255 -> .0218 creeping). Then MY ERROR: move_hand to
  (0.405,-0.097,0.125) without preview -> IK flipped the wrist to fingers-vertical (roll -0.27 -> -1.87) -> dropped.
  RULE: while holding, preview every step and reject wrist-roll jumps > 0.3 rad (or use contik + move_joints).
## Attempt 3 (reset 2, 23:06Z) and 4 (reset 3, 23:13Z) - pivot pinch with code/puppy_box/pivot_pinch.py
- Reset 2: wedge-push with the open gripper splayed the front pair (upper paw ~3.5 cm behind the lower). Pivot pinch
  on the LOWER front paw (grasp (0.372,-0.094,0.002) -> hand (0.377,-0.095,0.002)): w .0315 HELD.
  Lift (approach (1,0,0) to z .11, then (1,0,0.3)/(1,0,0.4)): z .039/.069/.106/.136/.175/.20/.24/.27, width
  .0313-.0314 STABLE the whole way (frames 20261105151018a8a63e z .20, 202611051510454d88cd z .27). One step
  (0.409,-0.098,0.146) with approach (1,0,0) was REFUSED by safe_move (IK branch flip) - the guard works.
  The puppy rolled onto its back, body still on the table, front end lifted.
  Then r1_walk 0.2 holding (walked .186) -> DROPPED (w .004); puppy on its back at the edge in front of the box.
  HOME afterwards hit the puppy twice (tilt 5-6.7, hand stuck at (0.22,-0.30,-0.02)); freed by moving the hand up
  to (0.22,-0.30,0.12), out to (0.14,-0.40,0.06), then HOME (clean); walk back -0.1 -> .185.
- Reset 3: closed-gripper pushes on the upper front paw (to x .415 and .44) gave only ~2.5 cm splay (springs back);
  pivot pinch on the lower front w .0309 -> lift with approach (1,0,0.3) from the start -> w .0279 at z .045 ->
  SLIPPED at z ~.08. (Right finger probably rubbing the upper paw that sat only 2.5 cm behind.)
- Pivot-pinch funnel this session: grasp 3/3 held (.031-.032); lift >= .20: 1/3 (and that one to .27, stable);
  carry-walk 0/1. Vertical pinch: grasp 2/2, lift >= .05: 0/2.
## FINAL (23:20Z, ~56 min of 120 used; resets 4/10; falls 0)
- TASK NOT ACHIEVED. Final state (frame 20261105151957f2d67a): robot standing ~0.65 m from the edge, tilt 1.2, arms
  home, both grippers 0.07; puppy on the table at the front edge in front of the box; box upright; nothing on the floor.
- Main result: the PIVOT pinch (fingers horizontal) is load-bearing - lifted the puppy's front end to hand z .27 with
  a constant width .031 (1/3 lifts; needs the neighbour paw >= 3 cm behind). Carry by walking dropped it (0/1).
- Deliverables in /tmp/claude-1001/ws_e0ac0c (workspace read-only): code/puppy_box/pivot_pinch.py (+obs.py, px.py),
  memory/*, handoff/README.md, skill-library commit b8569a9 (r1-paw-pivot-pinch draft) - skill_library_merge returned
  merged:false "no committed changes" because it reads the read-only workspace clone.
- Next: reproduce the splay (open-gripper wedge) and the lift; carry with <= 0.1 m walks + width checks; or grasp from
  a stance where the box is reachable after the lift.
