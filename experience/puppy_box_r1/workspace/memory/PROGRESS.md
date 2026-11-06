# PROGRESS — puppy_box_r1 (session 20261105T232821Z_puppy_box_r1_c4f6c3, sim)
Workspace READ-ONLY again -> working copy /tmp/claude-1001/ws_c4f6c3 (code/ memory/ handoff/ skill-library/ kin/ pylib->ws_08803f).
Read tool denied everywhere; images only via direct r1_observe. Frames: agent-workspace/runtime/frames/<id>/ (cat works).
New this session: operator help "hind leg overhang" answered in 27 s, presents ONE hind leg ~8 cm past the edge, the
other hind paw ~11 cm away (clean for the pivot pinch). Front paws also overhang but as a tight pair.
New tool code/puppy_box/edges.py (RANSAC table edges: distance + normal yaw = robot heading vs table) + look.py.
## Attempt 1 (initial scene) — approach failed, no contact
- t0 frame 2026110515294214302e: edge .63, head RIGHT, box BEHIND puppy (x .84-1.14, y -.28..+.12), hind paw (overhang)
  at (.555,+.03,+.01) -> needs ~0.17 m shift LEFT for the right hand.
- Turn-walk plan FAILED: in-place turns cmd 45 -> 39.3deg (+0.165 m), cmd 50 -> 40deg (+0.224 m), cmd -45 -> ~35deg
  (+0.23 m): every pure turn moves the robot ~0.2 m FORWARD along its heading. Ended 0.18 m from the edge at heading
  44deg with the hanging paw beside the right hip (robot (-.11,-.26)) - no safe move out (back = arm into leg,
  fwd/turn = body into table). -> reset 1.
- LESSON: never do lateral re-positioning by turns near the table; each turn ~ +0.2 m forward. Budget it from >=0.8 m.
## Attempt 2 (reset 1, 15:38) — approach failed again, no contact
- Same layout (shifted 5 cm right): hanging hind paw centre (.555,-.024), other hind (.58,-.135), front pair overhangs too.
- Walk data: turn 25 -> 14.2deg (.089 m), fwd .08 -> .149 (+6 cm right drift), back -.1 -> .225 (+4-5 cm right),
  turn 33 -> 26deg (.077 m, edge 8 cm closer, ~6-8 cm right).
- DRIFT MODEL [sim]: EVERY walk/turn call drifts the robot ~0.05-0.08 m to its RIGHT (per call, not per metre);
  turns also move it ~0.002-0.005 m/deg toward the table (X). Lateral shifts LEFT are expensive.
- IK sweep (horizontal fingers, z .01; code/puppy_box/sweep_angle.py): approach 0deg OK x .33-.38 y -.10..-.25;
  -15deg OK y -.20..-.30 (y -.15 flips to vertical); -30deg OK y -.25..-.30; -45 only y -.30 x .33. Downward-tilted
  approaches at x .44 err .02-.046. => the leg must point within ~0-20deg of the robot x axis (square-ish stance) with
  the paw at y -.10..-.25; an approach 40deg off the leg puts the right finger on the leg.
- New tools: pawfit.py (sphere fit r .025 to the brown blob near a seed pixel; rms .3 mm), preview.py, edges.py, look.py.
## Attempt 3 (reset 2, 15:45-15:55) — GRASP + full hanging LIFT to z .285, then SLIP (puppy fell to the floor)
- Approach (11 walk calls): back -.1 (.237, no drift), turn 95 -> ~84deg moved (+.34,+.35) [world=odometry, x=table normal],
  combo fwd -.25 + turn -45 -> moved .96 m (!! never combine), turn 60 (.23), walk .2/.1/.15 (.10/.003/.085), turn -100
  -> moved .53 m to 0.21 from the edge, back -.1 x2 (.117/.165), fwd .1 (.097).
  Odometry pelvis (r1_walk reply) is consistent with perception (edge world x .747 +-1 cm) -> use it.
- Turn model fitted: turns pivot about a centre ~0.25-0.39 m to the inside of the turn (car-like arc), x ~0..0.17.
- Paw not visible at x<.34 (head .6 max): r1_move_joints waist_yaw -.35..-.45 makes it visible (hand swings BACK, safe).
  Waist-yawed frames: edge yaw differs by ~3.7deg vs waist-0 frame (camera/pelvis coupling); measure in the SAME
  waist config you will grasp in. Grasped WITH the waist at -0.35 cmd (-0.16..-0.25 measured).
- Pivot pinch: paw (0.31,-0.213,-0.029) (sphere fit), pregrasp x-.08, grasp (0.313,-0.212,-0.028) approach (1,0,0)
  fingers (0,1,0): IK err .022 (low z), servo ok -> close -> width .032 HELD (1/1).
- Lift (code/puppy_box/lift.py guarded): approach (1,0,0) to z .123 (w .0311), then (1,0,.3/.4/.5/.6) to z .285 at
  x .29: w .0311 STABLE, puppy FULLY HANGING head-down in front of the table edge (hand 12 cm before the edge).
  Next step z .325 with approach (1,0,.8): servo correction 3 cm in y + approach change -> SLIP (w .004), puppy fell
  to the FLOOR. Lesson: under full load keep the approach constant, small steps, no big servo jumps; keep the hand
  over the table (x >= edge) so a drop lands on the table.
## Attempt 4 (reset 3, 15:58-16:02) — grasp MISSED, puppy knocked to the floor
- Approach: back x2, turn 90 (-> 77.7deg, (+.34,+.15)), back (Y -.25), turn -90 (moved .50 m! to .25 from the edge),
  back, fwd .1 -> paw (.32,-.27) at waist 0. RIGHT TURNS pivot about a centre ~(0.02,-0.52) (2 fits), left ~(.05,.31).
- Waist -0.35 look: pelvis counter-rotates (IMU yaw +5.9deg) -> paw appeared at (.354,-.306): pinch unreachable (ik .043).
- turn -15 -> heading -16..-20deg and moved BACK ~0.1 (unexpected). fwd .12 -> .095. Paw (.38,-.162) visible at waist 0.
- Pinch with approach rotated +10deg (leg at +20deg): pregrasp ok (ik .019), paw re-measured with arm out (.369,-.167),
  advance to (.372,-.166,-.026) -> close -> width .004 = MISS; afterwards the puppy was gone from the table (floor).
  Cause unknown (no view of the contact). Lesson: only grasp in the proven configuration (heading ~0, approach (1,0,0),
  paw at (.31,-.21) measured in the waist -0.35 config).
## Attempt 5 (reset 4, 16:03-16:15) — grasp + lift ok, DROPPED by walking while holding
- Approach with the MPC planner (walkplan.py/plan4.py, turn-arc model): back, back, turn 60 (46.6deg), turn -50 (6.9deg,
  edge .528), fwd .1/.1/.15 -> paw (.381,-.169,-.004) in the waist -.35 config; pregrasp; re-measured (.389,-.183,-.006);
  grasp at (.383,-.183,-.004) (full depth ik .031 > limit) -> width .0344 HELD.
- Lift (1,0,0) to z .09 (w .0313), then forward+up (.39,.125)->(.43,.18)->(.43,.265) approach (1,0,.2..0.5): w .0307,
  puppy hanging over the table top. Box front wall then at x .68 (edge .48, heading 10.5deg).
- r1_walk 0.15 (0.089 m) holding -> width .0041: DROPPED; the puppy landed on its back, legs up, against the box front.
- Tried a stopper grasp on an upward leg shaft (approach (0.97,0.24,0), no waist ik .004 / with waist exact): the hand
  stopped 2 cm high twice (z servo maxed) -> width .004; a ball pinch also missed (pelvis moved with the waist).
- HOME from beside the puppy -> tilt 8.25deg (hand caught); freed by up -> out -> HOME; stepped back. Robot safe.
## FINAL (16:22, 54 min used of 120; resets 4/10; falls 0)
- TASK NOT ACHIEVED (0 placements). Final state: robot standing ~0.37 m from the edge, arms home, grippers open;
  puppy on its back on the table in front of the box; box upright; nothing on the floor.
- Skill library: commit 6e5f23d (new r1-walk-approach draft; r1-paw-pivot-pinch v2) in /tmp/claude-1001/ws_c4f6c3/
  skill-library, patch in output/; skill_library_merge -> merged:false (reads the read-only workspace clone).
## Attempt 6 (reset 5, 16:19-16:23) — test of the no-walk place: box OUT OF REACH, then slip on lowering
- Same approach sequence (back, back, turn 60, turn -50, fwd .15/.15/.2) -> close stance: paw (.28,-.202,.011) at waist
  -.35; pregrasp (.20) -> grasp (.283) -> width .0345 held (3/3 in this configuration over the session).
- Lift (1,0,0) to z .10, forward+up to (.42,-.217,.26): width .0315 stable.
- Box front-left bottom corner at (.663,-.164) -> interior x >= .68. Waist IK preview: (.54,-.24,.28) ok (waist roll
  .47 = limit), (.58) flips branch, (.62) unreachable => the no-walk place is ~0.12-0.15 m short in this layout.
- Lowering to z .16 (approach (1,0,.3)) to rest the body on the table before a walk -> width .0255 -> .004 SLIP
  (load direction change when the body touches the table). Puppy on its back in front of the box again.
- Final (16:23): robot ~0.6 m back from the edge, arms home, grippers open, tilt 1.3; puppy on the table; box upright;
  nothing on the floor. Resets used 5/10.
