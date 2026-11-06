# Sim evidence: R1 puppy-box placements (2026-11-05)

Third-person recordings of the measured full chain run through the agent's own tool path (`r1/tools/diag/carry_chain_test.py`
against the R1 MuJoCo sim with the no-slip solver): shaft grasp along the leg axis, arc lift to hand z 0.265, 7 cm steps,
release over the box. Batch of 4: placements in trials 1 and 3 (both ground-truth confirmed), trial 2 held two steps then
slipped, trial 4 slipped on the lift. Across the three measured batches tonight: 7 placements in 12 runs.

- `puppy_box_r1_sim_placement_3_static_cam.mp4`: a placement recorded with a STATIC camera (the table stays put, the
  robot's three 7 cm carry steps are visible as steps; 38 s, real time). Prefer this one.
- `puppy_box_r1_sim_placement_1.mp4`, `puppy_box_r1_sim_placement_2.mp4`: two placements, real-time speed (~40 s each),
  camera following the pelvis (so the scene shifts in frame when the robot steps),
  starting after the sim-only stance placement (the scene is slid under the robot to skip the walk-in, which looks like a
  jump in a third-person view). The standing sway is this reimplementation's balance jitter, larger than the real stack's.

Agent placements through the RAX loop (sessions 7 and 9) have no third-person video: their head-camera frames and
ground-truth files (`placement_gt*.txt`) are in the session directories and the exported experience.
