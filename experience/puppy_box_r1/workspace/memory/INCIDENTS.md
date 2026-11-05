# Incidents (cumulative, r1sonic sim, puppy_box_r1)
| # | session | situation | what happened | cluster / hazard zone | rule now |
|---|---|---|---|---|---|
| 1 | 1d45ef | HOME joints from hand over table | hand onto table, tilt 7 | arm-home-near-table | retreat back at height first |
| 2 | 1d45ef | r1_grasp_paw, paw .38-.39 ahead | FELL tilt 81 | r1_grasp_paw | never call |
| 3 | 1d45ef | r1_grasp_paw, paw ~.25 | puppy knocked off | r1_grasp_paw | never call |
| 4 | 08803f | HOME joints, heading 24deg left, hand (.27,-.02,.04) | hand hit edge (closer on the right), tilt 7.9, 1 step | arm-home-near-table | retreat back at height first |
| 5 | 08803f | HOME joints, edge .13 ahead, puppy in front | hand swept puppy off table, tilt 7.3, 1 step | arm-home-near-table | retreat back at height first |
| 6 | 08803f | r1_grasp_paw, paws ~.30 | held false, tilt 12, puppy off | r1_grasp_paw | never call |
| 7 | 08803f | r1_grasp_paw, paws ~.26 (= a success stance) | FELL tilt 63, disarmed, operator restore | r1_grasp_paw | never call |
| 8 | e0ac0c | move_hand while holding, no preview, goal (0.405,-0.097,0.125) | IK flipped wrist roll -0.27 -> -1.87 (fingers horizontal -> vertical), puppy dropped on the table | ik-branch-flip | preview every move while holding; refuse finger-axis change / joint jump > 0.35 (pivot_pinch.safe_move) |
| 9 | e0ac0c | move_hand open gripper to (0.35,-0.082,0.012) fingers (0,1,0) | IK flipped to the vertical branch, hand 2.7 cm low, a paw forced the fingers open (.0759) and the puppy was dragged/rotated on retreat | ik-branch-flip | same |
