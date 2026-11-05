# Handoff — puppy_box_r1 on r1sonic (sim), session 20261105T222431Z_puppy_box_r1_e0ac0c

Outcome: NOT solved (0 placements; 4 resets used of 10; no falls). Best: pivot pinch lifted the puppy's front to z .27. The agent workspace was READ-ONLY again: all
deliverables are in /tmp/claude-1001/ws_e0ac0c (code/, memory/, handoff/, skill-library/ commit). Previous handoff
kept as handoff/README_08803f.md (still valid: perception, approach, retreat, r1_grasp_paw = fall hazard).

## The blocking problem and what this session learned about it
Lifting the puppy needs a grasp that survives the full hanging weight. Evidence so far (3 sessions, sim):
- Vertical-finger pinch of the paw ball (pads above/below): grasps 6/6, but the leg is a stiff ~8 cm lever that
  twists the ball out -> slipped at hand z .035-.29 (this session at the first 3-4 cm, twice). Do not plan on it.
- Horizontal-finger PIVOT pinch (fingers (0,1,0), approach (1,0,0), pads left/right of the ball): the ball can rotate
  about the pinch axis so the hanging load is shear only. Grasp 3/3 (w .031-.032); lift to z .27 with the width
  STABLE at .031 in 1/3 (reset 2; waypoints in memory/PARAMS.json pivot_pinch_lift_waypoints_pelvis_m); the other two
  slipped at .06-.09 (neighbour paw too close behind / my IK flip). THIS IS THE LEAD. Carry-walk with it: 0/1 (dropped
  during a 0.19 m walk at z .27).
- The obstacle for the pivot pinch: the neighbouring paw of the pair (centres ~5 cm apart, balls ~5 cm wide).
  One finger must go between them. It worked only after earlier contacts had splayed the pair. Options to test:
  (a) wedge the open finger tip between the two balls from the front (push the neighbour aside 1-2 cm);
  (b) pick the paw of a pair whose neighbour is >= 3 cm further back or forward;
  (c) ask the operator for the overhang with "only one leg hanging" (untested wording).

## Plan for the next session
1. Reset, head 0.5, `code/puppy_box/pivot_pinch.py look`. Approach straight with 0.2 then <= 0.12 walks; expect
   2-6 cm RIGHT drift per walk; stop with the target front paw at x 0.38-0.42 (visible at head 0.6) and y -0.12..-0.20.
   If the paws end at y > -0.10, back off >= 0.2 and fix the heading far from the table (turn 20 cmd ~ 10-13 deg).
2. `seed` (wrist roll for horizontal fingers) -> `pregrasp X-0.08 Y Z` -> `look` (re-measure with the arm out; the
   pelvis moves ~2 cm back) -> `grasp X Y Z` (ball centre, z from an arm-home frame) -> width .025-.05 = held.
3. `lift 0.04 0.01` repeatedly (safe_move refuses IK flips) to z ~.25, moving x back over the hip as you rise so
   the leg hangs vertically. Watch the width; if it creeps below ~.02, lower and re-grasp deeper.
4. If it holds: walk holding in SHORT steps (<= 0.1, check the width after each; the 0.19 m walk dropped it) so the
   box front wall (edge+.21) comes within x ~.45,
   move the hand over the interior at z >= .22, lower 5 cm, open, retreat (straight back, then HOME).
5. If the pivot pinch also fails under load: the remaining options are a stopper grasp across a leg (needs a side
   approach the 5-DOF arm cannot do at table height from a square stance) or r1_grasp_paw (2 falls in 7 calls).

## Skill library
skill-library commit b8569a9 on session/20261105T222431Z_puppy_box_r1_e0ac0c in /tmp/claude-1001/ws_e0ac0c/skill-library:
new draft r1-paw-pivot-pinch (code/run.py primitives seed/pregrasp/grasp/lift/release with the IK-flip guard) and new
evidence for r1-paw-pinch-grasp. NOT merged (skill_library_merge reads the read-only workspace clone) - copy it over.

## Files
code/puppy_box/pivot_pinch.py (new: look/seed/pregrasp/grasp/lift/release, every move previewed + branch/jump
check), obs.py (observe+scene), px.py (pixels of a saved frame -> pelvis points), scene.py, percep.py;
code/tools/sweep.py (local IK reach sweep per approach); kin/ (backend r1_kin.py + xml copy).
memory/{PROGRESS,RUNS,TIPS,INCIDENTS}.md, memory/PARAMS.json.
