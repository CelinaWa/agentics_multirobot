# RUNS — puppy_box_r1, r1sonic sim, session 20261105T194732Z_puppy_box_r1_1d45ef (all by hand, no unattended runs)

| # | time (Z) | layout | what I did | outcome | where I looked / decided |
|---|---|---|---|---|---|
| 1 | 20:09-20:16 | puppy head R, hind paws LEFT (+.23..+.28), box behind-left | L90 / walk .25 / R90 to shift left, then backward walks | ended 0.09 m from edge (turn drift .25-.3/90deg, backward walk moves fwd); no grasp | frames 20261105120933fcd017 -> 20261105121506736b62; reset |
| 2 | 20:19-20:33 | puppy head R, hind paws -.01..-.06, box behind | 2 help requests for overhang (no reply), robot drifted .12 m (scene looked slid), 3 small turns | paws ended on the LEFT; no grasp | 2026110512194273af53 .. 20261105123203a111dc; reset |
| 3 | 20:35-20:44 | same, operator pre-positioned hind paw 6 cm over the edge | straight approach to paw x .37; r1_grasp_paw -> held (right, .034) | leg slipped within ~50 s; puppy rolled onto its back in front of the box; manual upright-leg grasp failed (body shifts 6-12 cm per reach); HOME drove hand onto table (tilt 7) - recovered | 20261105123646ddfe8f (held), 202611051237328d4c7a (slipped), 2026110512444046426a (end) |
| 4 | 20:54-20:56 | same (reset 4) | approach to paw x .38-.39; scripted grasp_carry.py -> r1_grasp_paw | ROBOT FELL (tilt 81, disarmed); puppy to floor; operator restored after 2nd help request | 2026110513063419ba79 |
| 5 | 21:14-21:16 | same (operator restore) | approach to edge ~.29 (paw ~.25); r1_grasp_paw | held=false, robot yawed 10deg, puppy knocked off the table | 202611051315087e64e0, 2026110513161251bd98 |
| 6 | 21:17-21:18 | same (extra reset granted) | approach for a manual hanging-leg grip: 0.25 cmd -> .104, then 0.25 cmd -> .334 (overshoot) | stopped: edge ~0.18 m ahead, paw (~0.1 m) out of camera view, no backward walk; no grasp attempted | 202611051318006bf360 (end: robot standing, arms home, grippers open; puppy + box on table) |

Funnel: approach OK 4/6, grasp held 1/3 (r1_grasp_paw), carry 0, place 0. Falls 1. Puppy off table 2.
