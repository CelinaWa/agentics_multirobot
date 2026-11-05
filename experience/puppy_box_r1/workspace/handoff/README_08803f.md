# Handoff — puppy_box_r1 on r1sonic (sim), session 20261105T213131Z_puppy_box_r1_08803f

Outcome: NOT solved (0 placements, 7 attempts, 5 resets + 1 operator restore after a fall). Final state safe: robot
standing 0.26 m back from the table, arms home, grippers open; puppy on the table in front of the box; box upright.
The agent workspace was READ-ONLY this session: everything is in /tmp/claude-1001/ws_08803f (code/, memory/, handoff/,
skill_draft/, pylib/ with mujoco). Copy it into the workspace first. Frames: agent-workspace/runtime/frames/<id>/.

## What works (use it)
1. Perception: code/puppy_box/scene.py <frame> -> table edge/height, paw blobs (dark spheres, k-means split by expected
   area), box rim. Front paws: controller.front_paws() (area filter + merge), verified on 5 frames (<=1.5 cm).
2. Approach: straight walks, 0.2 then <=0.12 commands, re-measure each time (walks are 30-100 % of the command and
   drift right up to 0.12 m). Backward walks work (overshoot ~0.25). Turn only >= 0.5 m from the edge.
3. Pinch grasp of a paw ball, 4/4: pre-grasp 8 cm short (approach (1,0,0.1), fingers (0,0,1), open), re-measure the
   paw (each arm move shifts the pelvis 1-6 cm and yaws the waist), final approach (1,0,-0.1) with the ball 0.5-2 cm
   past the grasp point, r1_gripper 0 -> width 0.032-0.035 = held. Keep the target paw at y -0.15..-0.20.
4. Joint-continuous motion while holding: code/tools/contik.py plan_b (backend point = local site - 0.107*approach),
   executed with r1_move_joints 2-3 s per 4-5 cm. Never r1_move_hand without an approach while holding (IK restart).
5. Carrying by walking works: 0.2 m forward walks with a held puppy kept the grip (2/2).
6. Retreat: hand straight back at height to x 0.2, then HOME joints, then walk back 0.2.

## What blocks success (start here)
- The paw-ball pinch is friction-limited: it slips once the puppy's weight hangs from it (z 0.12-0.29), so the puppy
  never cleared the 14 cm rim. Lifting from the overhang layout needs a STOPPER grasp: pads across the LEG with the
  ball outside the pads (then the ball, not friction, carries the load). In the overhang layout the legs point at the
  robot, so that needs a sideways approach - not reachable from a square stance (ik .04-.08). Untested options:
  (a) stance turned ~45-60deg left of the table normal + approach (0.8,0.6,0) in the robot frame (= ~90deg to the
  leg in the world) with the grasp point ~edge-1 cm; check the 14.5 cm housing against the table edge first;
  (b) after a slip the puppy twice ended with legs pointing up/left - a vertical leg is a natural stopper for a
  front approach with fingers along y, but the two legs were 4.5 cm apart (fingers 2.65 cm thick do not fit between);
  (c) the snout points sideways with the nose ball as stopper, but its underside is only ~3 cm above the table.
- r1_grasp_paw is a FALL HAZARD (2 falls, 3 knock-offs in 7 calls over two sessions). Do not call it. If someone
  must: call r1_gripper(0) right after it (the stored gripper command stays 0.07 and the next motion opens it).
- Geometry: box front wall is edge+0.21..0.25 behind the paws; right-hand reach x <= ~0.50 at z 0.2 -> the carry
  needs ~0.2 m of walking with the puppy held, ending with the edge ~0.15-0.2 ahead (no body contact seen at 0.13).

## Plan for the next session (same or different layout)
1. Observe from the start (head 0.5); find edge, puppy (head side, paw blobs) and box rim. Ask the operator for the
   overhang only if the reset did not already do it (it did, every time this session).
2. Pick the paw in front of the right shoulder after a straight approach (head-right layout: a front paw; head-left:
   a hind paw). If the box is not behind that paw, choose the heading at >= 0.5 m from the edge (turns drift).
3. Grasp with the pinch (above), then try a stopper re-grasp or a different hold before lifting above z ~0.12.
4. Lift/carry with contik joint steps, walk 0.15-0.2 holding, measure the rim, move over the interior at z >= 0.22
   (housing clears the rim), lower 5 cm, open, retreat. Verify "in box" from a view looking into the box.

## Files
code/puppy_box/controller.py (steps: look approach pregrasp grasp lift carry box place retreat; offline-tested
perception and planning, not run end-to-end), code/puppy_box/scene.py, percep.py, code/tools/{contik,reach,pathcheck}.py,
memory/{PROGRESS,RUNS,TIPS,INCIDENTS}.md, memory/PARAMS.json, skill_draft/ (library skill, not merged: read-only).
