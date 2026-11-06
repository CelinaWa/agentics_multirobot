# Sim evidence: R1 puppy-box placements (2026-11-05)

Third-person recordings of the measured full chain run through the agent's own tool path (`r1/tools/diag/carry_chain_test.py`
against the R1 MuJoCo sim with the no-slip solver): shaft grasp along the leg axis, arc lift to hand z 0.265, 7 cm steps,
release over the box. Batch of 4: placements in trials 1 and 3 (both ground-truth confirmed), trial 2 held two steps then
slipped, trial 4 slipped on the lift. Across the three measured batches tonight: 7 placements in 12 runs.

- `puppy_box_r1_sim_placement_1.mp4`, `puppy_box_r1_sim_placement_2.mp4`: the two placements (10 fps, ~25 s each).

Agent placements through the RAX loop (sessions 7 and 9) have no third-person video: their head-camera frames and
ground-truth files (`placement_gt*.txt`) are in the session directories and the exported experience.
