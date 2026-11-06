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

# Session 20261105T232821Z_puppy_box_r1_c4f6c3 (sim)
| time | situation | what happened | tilt | fix |
|---|---|---|---|---|
| 15:36 | 3 big turns near the table | ended 0.18 m from the edge at 44deg, hanging leg beside the right hip | 1.3 | reset (no safe walk) |
| 15:49 | turn -100 near table | moved 0.53 m, edge 0.21 m ahead | 1.6 | back -0.1 x2 |
| 15:54 | full-load lift step with approach change | slip, puppy fell to the floor | 0.8 | reset |
| 16:01 | rotated-approach grasp | miss, puppy ended off the table | 0.4 | reset |
| 16:14 | HOME joints with the hand at (0.22,-0.30,0.12) beside the puppy/edge | hand stuck at (0.16,-0.35,-0.05) | 8.25 | hold; up -> out (0.12,-0.40,0.06) -> HOME; back 0.1 |
Hazard zone: right hand low (z < 0.12) within 0.1 m of the table edge or the puppy -> never use HOME joints directly.

# Session 20261106T010905Z_puppy_box_r1_f6b331 (sim) — no falls, no tilts > 3 deg
| time | situation | what happened | fix / rule |
|---|---|---|---|
| 17:51 / 17:59 | long or misaligned advance to the paw | fingers pushed the paw 3-6 cm (width > .071 before close) | align 7-9 cm short, y err < 5 mm, advance <= 9 cm |
| 17:54-17:56 | held puppy at z .275, arm still ~2 min while planning | grip crept and let go (.0313 -> .004), puppy on table | never pause while holding |
| 18:05 | held at z .365, waist-assisted forward extension | slip -> puppy dropped near the edge (floor?) | no waist change under load; walk straight after the raise |
Hazard zone: holding the puppy (any pause or waist motion) - the grip is friction-limited and creeps.
