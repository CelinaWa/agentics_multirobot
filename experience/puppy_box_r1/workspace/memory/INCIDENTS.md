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

# Session 20261106T021942Z_puppy_box_r1_cdc45b (sim) — no falls, no tilts > 2.3 deg
| time | situation | what happened | fix / rule |
|---|---|---|---|
| 18:28-18:31 | large turns (+90 / -90) at 1-1.5 m from the table | robot displaced 1.0-1.3 m | no turns > 45; none near the table |
| 18:48 | V-finger pregrasp 1 cm high | lower jaw inside C's ball top (width .0706 at full open) | back out along the leg before any correction |
| 18:59 | V-finger lift step 3 | slip, puppy rolled on the table | use H fingers (pivot) for lifts |
| 19:05 | servo-ON lift step 2 | hand jumped 3.3 cm sideways -> slip | servo=False during lifts |
| 19:11 / 19:15 / 19:22 | r1_walk .1/.05/.05 holding the puppy at z .36-.40 | dropped on the table (box untouched) | no walking while holding |
Hazard zone: holding the puppy + any r1_walk.

# Session 20261106T033337Z_puppy_box_r1_7c03aa (sim) - no falls, max tilt 5.5
| time | situation | what happened | tilt | fix / rule |
|---|---|---|---|---|
| 19:51 | first r1_head after an operator stance placement | backend re-sent the STALE raised-arm command (gripper 0): arm swung through the paw area, scene shifted | 4.1 | after any placement/reset send HOME joints + gripper open first |
| 19:57 | HOME joints after walking .19 toward the table with the hand at (.28,-.30,.30) | hand stuck at the table edge (.23,-.34,-.03) | 4.9 | hold; hand up/out (.14,-.40,.08); walk back; HOME |
| 20:33 | move_hand from HOME to (.40,-.21,.16) with the edge only ~.34 ahead | hand came up under the table edge, stuck at (.33,-.25,-.02), grasp error .20 | 5.5 | hold; HOME joints (down/back) freed it; route via (.20,-.28,.16) behind the edge |
Hazard zone: right hand below table level within ~.1 m of the edge with the edge < .40 ahead (after walking in).

# Session 20261106T060138Z_puppy_box_r1_2fbfb7 (sim) - no falls, max tilt 6.65
| time | situation | what happened | tilt | fix / rule |
|---|---|---|---|---|
| 22:36 | v4 out after st9: walk back -0.2 went only .031, then HOME | hand driven onto the table edge (.26,-.29,.00), gripper blocked .049 | 6.65 | hold; open; hand up (.24,-.31,.16) -> out (.14,-.40,.12); walk -0.2; HOME. out() now repeats the walk back and refuses HOME if short |
| 23:07 | st17 release at hand z .356 after the LOWER move was refused (jump .49) | puppy ended on the FLOOR (operator) | 1.4 | if LOWER is refused, retry it as a smaller step / skip release and report; never release from a thin (.028) grip high above the rim |
Hazard zones: (1) arm HOME within .25 m of the table edge; (2) releasing a swinging body high above the box.

## Session 727e14 (sim)
- st3: after a carry DROP (hand at (.43,-.23,.26), robot .078 closer), `home.py` (direct HOME joints) caught the arm on
  the table/puppy: tilt 7.25, hand stuck at (.356,-.263,.081), waist yawed .18. Freed by `v6.py out 0.2` (up, back,
  out (.14,-.40,.14), walk back .198, HOME). Rule: after any carry, ONLY `v6.py out`, never home.py.
- st4: a QUESTION mentioning "the hanging hind paw" was routed as an overhang request (answer "paw now hangs 6.1 cm
  past the edge"), and the puppy was then gone from the table (off the table, frame 20261105234950f210a1). Never put
  "hanging"/"overhang"/"position" words in a question; questions only as "Question only ... is the puppy inside the box?"

## Session 90b103 (sim)
| time | situation | what happened | tilt | fix / rule |
|---|---|---|---|---|
| s12 | lift abort at the TOP step (width .0361 -> .0317 at cmd z .252) | direct put-back to G refused (joint jump .73), back_off opened the gripper at z ~.22 | 0.9 | puppy fell off the table. Abort = retrace the lift path downward, open only low; refuse -> keep holding and ask |
| s6 | puppy bridged across the box top after a carry drop; 2 closed-gripper push sweeps (approach (.8,.6,.15), z .20/.155) | puppy rode up onto the forearm (+20 cm above the box floor), robot .10 m from the edge, waist yawed -.24 | 2.9 | never push a bridged puppy with the arm; withdraw sideways (y -.30) and down, walk back, HOME |
| W0 | lateral walk-in by turn pairs (+45/-45) | -45 reported -6.8 deg but walked .356 m toward the table (edge .33 ahead, robot 38 deg off) | 1.5 | turns move the robot forward ~.2-.35 m per 45 deg; do turns only >= .6 m from the edge, re-observe after each |
