# RUNS — puppy_box

## Session 20261031T234111Z (simulation rehearsal; layout: puppy left of centre, box far left/back)
All manual-with-code (code/puppy helpers); controller steps run one at a time with my checks.

| # | sim time | what | outcome | where I looked / decided |
|---|---|---|---|---|
| 1 | ~400-560 s | 45-deg claw (left), lean 0.25, servo to torso (0.44,0.06); servo iterated under contact | FAIL: hand drifted to the neck (y+7 cm), close stalled, 4 cm lift -> fist | frames 20261031235028949f5f (hand on puppy), 202610312351057d9e7c (after lift) |
| 2 | ~560-700 s | 45-deg claw, lean 0.35, converged 2 cm above torso (0.48,0.0), descended with fixed bias | FAIL: hand slid +5 cm y / -3 cm x on contact, tips 5 cm above table, lift -> fist | 202610312352449086ff (pre), 20261031235300a4a1b6 (contact), 202610312353206e3392 (lift) |
| - | ~760 s | moved hand sideways after opening (thumb_1 -0.4 shape) | puppy dragged ~20 cm left toward the box | 20261031235425ff7ff5 |
| - | ~790 s | servo to (0.30,0.02,0.25) picked a bad IK branch | left hand swung across the body, tilt 7 deg; g1_hold, reversed to last safe targets | 20261031235453e782d5; IK guard added |
| 3 | ~880-990 s | 30-deg deep claw, thumb opened -0.4, torso at (0.365,0.24) | FAIL before close: palm stalled on top, plush squirted left out of view | 202610312356229a9856, 20261031235658a6f1bf |
| reset 1 | ~1000 s | operator reset | puppy back at start | 20261031235913078f65 |
| - | ~1100 s | box measured from rim points: interior 16-19 cm beyond IK reach; asked operator (3x, generic replies) | no walking (table edge 0.23 m ahead at pelvis height) | 202611010001040b7dc2 |
| 4 | ~1300-1400 s | claw yawed -1.57 over all four legs (0.285,0.035) | FAIL: thumb pinned by contact (never closed), lift -> fist | 20261101000422f970a9, 20261101000438c2edf1 |
| - | ~1460 s | retract home | standing, arms home, hands open, box untouched | 202611010005407f5ba9 |
| 6 | ~1850-1900 s | RIGHT hand, 30-deg claw yawed 0.7 across the tail (mid (0.465,-0.045)), tips to table+1 cm | PARTIAL: held through a 5 cm lift (thumb_1 -0.39, index_0 1.08 stalled), slipped at ~13 cm; puppy fell back rotated near the front edge | 202611010012006d5be8 (pre), 202611010012352fb7fb (held), 2026110100125141aa5f (slipped) |
| reset 2 | ~1980 s | operator reset after the tail slip | puppy back | - |
| - | ~2000 s | retract_home fixed (shoulder 0.75 before elbow) verified on right arm | OK | 20261101001416606524 |
| 5 | ~1700 s | library skill hand-claw-pregrasp run.py unattended from home (perception -> pre-grasp -> retract) | OK: err 1.3 cm, tilt 1.1 deg; retract tilt 1.9 | 20261101000737e08116, 202611010007584800ae |

Funnel: perceive 5/5, pre-grasp 5/5, descend-without-pushing 2/4, close-stalls 3/3, holds-on-lift 1/4 (tail, 5 cm only), carry/place 0 (box unreachable).

---
## Previous session (20261031T211606Z) — kept for reference

No controller was run (shell sandbox unavailable: no Python). All attempts were manual g1_step frames.

| # | time | what | outcome | where I looked / decided |
|---|---|---|---|---|
| 1 | 21:20-21:29 | right palm-down top grasp, close at palm ~5 cm above body | FAIL: index full fist, lift -> all fingers to fist, plush slipped | frames 20261031212823..., hand_state after close and after 3 cm lift |
| - | 21:29-21:36 | re-approach with lean 0.3 to reach thumb over torso centre | puppy pushed to x~0.56 by down-pointing thumb; left hand hit table edge during lean | frames 20261031213220..., 2026103121351.. ; aborted |
| reset 1 | 21:37 | operator reset | puppy back but rotated (head front-left, legs to robot) | frame 20261031213712... |
| 2 | 21:38-21:44 | lean 0.22, shoulder yaw/waist yaw to centre over torso, close with thumb on torso top | FAIL: same finger pattern, lift -> full fist | frames 20261031214338..., 202610312144026... |
| - | 21:45-21:46 | open hand, raise, swing arm home (2 calls), un-lean | safe standing, arms home, hands open, box untouched | frame 20261031214621... |

## Session 20261101T002411Z (sim)
| # | what | frames | outcome |
|---|---|---|---|
| 1 | survey: waist roll +0.25/+0.42 and waist yaw cmd 1.3 to find the box (left of puppy) | 2026110100265412a973, 20261101002915355b1c | box floor x 0.30-0.61, y 0.53-0.92 |
| 2 | reach check (place-reach-check main, unattended) | 20261101004231cd4d1c | not reachable, 8 cm short |
| 3 | claw tilt 0.52 over head via waypoint + servo | 20261101003620c73061 | servo diverged (SONIC does not track elbow 1.4/wrist roll 1.9); unwound |
| 4 | palm-down left hand (topgrasp.go) over neck, close, lift 5 cm | 20261101004013a4a55c, 20261101004033efb09f | stalled on top, fist on lift: FAIL |
| 5 | retract_home both arms | 202611010041282ef48b | standing, arms home, hands open, puppy on table |
| 6 | palm-down + direction cost, thumb low in front of the head, close | 202611010046343d179d, 202611010046449d4216 | full fist = missed: FAIL |
| 7 | retract_home both arms | 20261101004719af7572 | standing, arms home, hands open, puppy on table |

## Session 20261101T004902Z (sim) — box LEFT of puppy, in view; puppy head toward the box
| # | what | frames | outcome |
|---|---|---|---|
| 0 | start: console start failed -> request_operator(help) armed; g1_hold | 20261101004938c428c9 | box at table front edge OVER the left hand (seen through the box floor) |
| 1 | right leg-wrap, palm-down yaw -45 (planner pre-grasp, own IK descent), palm z 0.035, thumb y -0.16 | 2026110100560218695d, 20261101005808977c53, 20261101005822492a2c | FAIL: stalled then full fist on 4 cm lift; front leg was at larger y (thumb on/over it) |
| 2 | re-localized (puppy 6 cm further), left arm back, lean 0.21->0.25->0.30->0.42, compensated y +6 cm, close | 20261101005944d8fe4a, 202611010101360ef07a, 2026110101015451dd5d | FAIL: full fist on lift. BOX FELL TO THE FLOOR between 005944 and 010010 (robot drifted back ~19 cm) |
| reset 1 | reset -> only the puppy restored; help -> generic reply; waist-roll look confirms table empty | 20261101010312b15d12, 202611010103332a7be8 | task impossible for the rest of the session |
| 3 | no lean, leg under fingers (d 0.093), palm meas z 0.055 | 20261101010518529a22, 202611010105359c29ee | FAIL: middle/index stalled, lift 3 cm -> fist; reach floor z~0.04 without lean |
| 4 | library palm-pose-servo run.py unattended: unfold -> main -> retract_home | 2026110101083568bd11, 20261101010848f4a5d1 | OK err 0.8/1.6 cm, tilt 1.6, retract tilt 3.3 |
Funnel (this session): perceive 4/4, pre-grasp 4/4, descend to wrap height 0/3 (reach floor / y offset), holds-on-lift 0/3, place 0 (box lost).

## Session 20261101T011122Z
- Run 1 (manual): unfold OK (frame 202611010114239e3185). Lean 0.3 + pre-grasp reach in one 4 s ramp (frame
  20261101011727cb1aa5: hand over the front legs, tilt 1.7, pelvis slid back 8 cm). Robot FELL ~14 s later while holding.
  Stage reached: pre-grasp (no contact). Outcome: failure (fall); scripted operator could not re-stand the robot.

## Session 20261101T012043Z (sim) - puppy LEFT, box RIGHT, 0.67 m apart
| # | what | frames | outcome |
|---|---|---|---|
| 0 | start, observe, deproject | 20261101012119267277, 20261101012313203e78 | puppy (0.51,0.31), box x 0.29-0.57 y -0.17..-0.56 |
| 1 | span check (model, both arms) | 20261101012729d9d2d2 | no arm spans (23 cm short each) -> no grasp attempted |
| 2 | left unfold -> hover (0.32,0.25,0.18) yaw 0 -> retract_home | 20261101012624b6bd7b, 202611010126376d3a5f, 2026110101265788ed90 | OK err 2.5 cm, tilt 1.5/3.0; robot crept 4 cm forward |
