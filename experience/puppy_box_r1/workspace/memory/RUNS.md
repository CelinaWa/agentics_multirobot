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

# RUNS — session 20261105T213131Z_puppy_box_r1_08803f (r1sonic sim). All by hand (tool calls), no unattended runs.
| # | time (Z) | layout | what I did | outcome | where I looked / decided |
|---|---|---|---|---|---|
| 1 | 21:34-21:50 | head R, box behind (front wall edge+.21), overhang by operator | approach drifted R .12; 2 turns to heading 24deg; pinch upper front paw w/ diagonal approach (chased body shift) -> HOME hit table edge (incident); balance step; pinch at (.42,-.18) held .035; lift to .15 | move_hand w/o approach -> IK flip -> SLIP, puppy off table | 202611051342530bb2d0, 20261105134841411503 (held), 20261105134957094653 |
| 2 | 21:51-21:57 | same (reset 1) | straight approach; pinch lower front paw held .033; continuous-IK lift held to .20 | slip at .29; puppy on table (legs left) | 20261105135241f57590, 202611051355095cbc3e |
| 3 | 22:02-22:04 | same (reset 2) | pinch upper front paw held .032; slow lift | slip at .19; puppy off table | 2026110514030809f910, 202611051404013b3c62 |
| 4 | 22:05-22:08 | same (reset 3) | r1_grasp_paw held (.057, lift .062); walk .15 holding (.203, held) | my move_joints re-sent stale gripper 0.07 -> dropped in front of box; HOME swept puppy off (incident) | 202611051406052f337b, 2026110514061814109b, 202611051406574736b5 |
| 5 | 22:08-22:09 | same (reset 4) | r1_grasp_paw at paws ~.30 | held false, tilt 12, puppy knocked off | 2026110514091257db2d |
| 6 | 22:10-22:12 | same (reset 5) | r1_grasp_paw at paws ~.26 (stance ~= run 4) | ROBOT FELL (tilt 63), disarmed; operator restored | 20261105141213b7b9ba |
| 7 | 22:13-22:16 | same (operator restore) | blind pinch (paws below view, leg points +6 cm) held .032; forward+up lift to .07 held; walk .08 holding held | raising with puppy against box wall -> slip at .12; puppy on table; retreat + step back | 20261105141422e65f26, 20261105141520a0aab2, 20261105141627ff7de3 (final) |

Funnel (this session): approach 7/7; grasp: pinch 4/4, r1_grasp_paw 1/3 (+1 fall); lift above rim+housing (z>=.20): 1/5;
carry walk 2/2; place 0. Falls 1 (r1_grasp_paw). Puppy off table 4. Final state: robot standing, arms home, grippers open,
0.26 m back from the table; puppy on the table in front of the box; box upright. TASK NOT ACHIEVED.

# RUNS — session 20261105T222431Z_puppy_box_r1_e0ac0c (r1sonic sim). All by hand.
| # | time (Z) | layout | what I did | outcome | where I looked / decided |
|---|---|---|---|---|---|
| 1 | 22:27-22:49 | initial: head R, box behind, paws at the edge; operator overhang | 4 walks (drift .17 R), front paws ended y~0; 3 diagonal reaches chasing the paw (waist yaw, servo saturation); waist-yaw trick; 2 turns | no grasp; reset | 20261105142718a8eaff, 20261105143625abec0d, 2026110514403257f622, 202611051446295d36b4 |
| 2 | 22:49-23:10 | reset 1: same layout | straight approach, front paws y -.15/-.20; deep vertical pinch held .030 -> slipped at z+.04; shallow vertical pinch held .028 -> slipped at z .035; HORIZONTAL pivot pinch on the (splayed) lower front paw held .0317 -> to z .086 (w .022) -> my move_hand without preview flipped the wrist -> dropped; 2 more horizontal tries missed (blind, legs moved) | no lift above .09; puppy on table, legs splayed | 202611051450179e4b74, 2026110514514182e984, 20261105145933... (lift), 202611051459323c3e57 (held z .086), 20261105150430b0f15a (end) |
| 3 | 23:06-23:13 | reset 2 | walks (0.079,.031,.01,.048,.131 - the robot barely progressed for 4 calls); wedge-push splayed the pair; PIVOT pinch lower front held .0315; lift to z .27 width stable .031; walk .2 holding -> dropped | puppy on its back at the front edge, in front of the box; HOME hit the puppy (tilt 6.7), freed via up/out path | 20261105151018a8a63e (z .20), 202611051510454d88cd (z .27), 20261105151056a4ae40 (after walk, dropped), 2026110515124495263d |
| 4 | 23:13-23:17 | reset 3 | walks .09/.098; 2 closed-gripper pushes on the upper front paw (splay only ~2.5 cm); pivot pinch lower front held .0309; lift with (1,0,0.3) from the start | slipped at z ~.08; arm home, robot standing ~.45 from the edge | 20261105151431ee3d1a, 20261105151558f0a824 |
| 5 | 23:17-23:20 | reset 4 | walks .092/.108; closed-gripper pushes on the upper front paw (back-right to (.40,-.24); straight to .445/.46 refused as unreachable) -> pair offset only 2.3 cm, so no pinch tried; arm home, walk back -0.15 (.231) | final state (frame 20261105151957f2d67a): robot standing ~0.65 m from the edge, arms home, grippers open, tilt 1.2; puppy on the table at the edge, box upright, nothing on the floor | 20261105151842f8657c, 202611051519336c2528 |

Funnel (this session, 4 resets used of 10, 0 falls): approach 5/5; grasp: vertical pinch 2/2, pivot pinch 3/3; lift >= .20: vertical 0/2, pivot 1/3 (to .27, stable); carry-walk 0/1; place 0. TASK NOT ACHIEVED.
