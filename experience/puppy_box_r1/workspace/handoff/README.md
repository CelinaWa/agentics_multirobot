# Handoff — puppy_box_r1 on r1sonic (sim), session 20261105T232821Z_puppy_box_r1_c4f6c3

Outcome: NOT solved (0 placements; 6 attempts; resets 5/10; falls 0; one 8.3 deg tilt from a HOME move).
Workspace was READ-ONLY again: everything is in /tmp/claude-1001/ws_c4f6c3 (code/, memory/, handoff/, skill-library
commit 6e5f23d + patch output/0001-*.patch). skill_library_merge -> merged:false (reads the read-only clone).
Older handoffs: handoff/README_e0ac0c.md, README_08803f.md (perception, retreat routes, r1_grasp_paw = fall hazard).

## Layout (identical on every reset this session)
Start square, edge .62-.63 ahead; puppy head RIGHT; box BEHIND the puppy (front wall = edge + .20, rim = table + .145,
box spans y paw+.09 .. paw-.31); operator overhang: hind paw ~8 cm past the edge at (.548,-.017,+.004) from the start,
other hind paw 11 cm right / 5 cm back (clean for the pivot pinch).

## Per subtask: how to plan it in any layout
1. Stance (skill r1-walk-approach): goal = the target paw at pelvis (0.31,-0.21) [range x .30-.38, y -.17..-.25],
   heading 0..+10 deg (leg along +x). Walks are ARCS: left turns pivot ~0.31 m to the left, right turns ~0.52 m to the
   right; a 90 deg turn moves 0.33-0.53 m; every call drifts 0-6 cm right; fwd 0.1 is a deadband. Use odometry
   ('pelvis' in r1_walk replies) + edge heading and re-plan after each call (code/puppy_box/walkplan.py, plan4.py).
   Proven sequence from this start: back -.1, back -.1, turn +60, turn -50, fwd .1-.15 x1-3 (7 calls).
2. Look: the paw at x < .34 is out of view at head .6 -> waist_yaw -0.35 (r1_move_joints), sphere-fit the paw
   (code/puppy_box/pawfit.py, rms .3 mm). Measure and grasp in THE SAME waist configuration (pelvis counter-rotates).
3. Grasp (skill r1-paw-pivot-pinch): pregrasp x-.08, approach (1,0,0), fingers (0,1,0), open .07; re-measure with
   the arm out; advance to the ball centre (0.5 cm short if ik > .03); close -> width .031-.035 = held (2/2 in the
   proven configuration; a +10 deg rotated approach missed 1/1).
4. Lift (code/puppy_box/lift.py = guarded steps): approach (1,0,0) to z .10, then forward+up over the table edge
   ((.39,.125)->(.43,.18)->(.43,.265), approach tilt .2-.5). Holds the full hanging weight statically (z .285).
   Do NOT tilt the approach beyond ~.6 under load and keep steps <= 4 cm (a (1,0,.8) step dropped it).
5. Carry to the box: THE BLOCKER. Walking while holding the friction pinch dropped it 2/2 (0.19 m and 0.089 m).
   With the box behind the puppy the box interior is 0.25-0.30 m beyond arm reach from the grasp stance.
   Next options, in order:
   a) No-walk place: TESTED att 6 - from the closest stance (paw x .28) the box interior starts at x .68, waist IK
      reaches only x .54 (.58 flips, .62 unreachable) => impossible in the box-behind layout. Lowering the held
      puppy onto the table (to walk with low load) also slipped (width .0255 -> .004).
   b) Stopper grasp before walking: pads across a leg shaft with the ball outside (load on the ball, not friction).
      After a drop the puppy lay on its back legs-up against the box: a shaft grasp with approach (0.97,0.24,0)
      previewed ik .004, but the hand stopped 2 cm above the shaft twice (z servo saturates at .05).
   c) If the box is beside the puppy (left/right layouts), the carry is lateral and in arm reach: try that first.
6. Retreat: open, hand UP then OUT to (0.12,-0.40,0.06) then HOME joints, then walk back 0.1. Never HOME directly
   from a low hand near the puppy/edge (tilt 8.3 deg this session).

## Files (in /tmp/claude-1001/ws_c4f6c3)
code/puppy_box: walkplan.py (model + MPC search, `python walkplan.py x y psi tx ty tpsi edge_x`), plan4.py (depth-4),
edges.py (table edge lines), look.py (observe + scene + edges), pawfit.py (sphere fit), preview.py, sweep_angle.py
(IK reach sweep per approach angle), lift.py (guarded holding moves), pivot_pinch.py/scene.py/percep.py (older).
memory/{PROGRESS,TIPS,RUNS,INCIDENTS}.md, memory/PARAMS.json (walk.* and pivot_pinch.* rows added).
