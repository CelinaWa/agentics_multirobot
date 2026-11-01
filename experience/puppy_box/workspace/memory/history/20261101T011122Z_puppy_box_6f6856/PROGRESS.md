# PROGRESS — puppy_box session 20261101T011122Z (sim rehearsal, 5th session)

STATUS (final, ~36 min of 60): ROBOT FELL while holding a leaned reach pose. Disarmed (tilt 64 deg, pelvis height est
0.215 m). No motion since. Operator asked (help); scripted reply "approved, go ahead" but status still disarmed.

## Scene (this layout)
- Box LEFT, flat on the table (depth over it ~table+0.14-0.16, NOT sloped): x 0.33-0.63, y 0.05-0.33 (first frame).
- Puppy right of the box, head LEFT (toward box), legs toward robot. First frame was off again (~7 cm in y).
- After the first motion (frame 202611010114239e3185, table z -0.053): front leg pair = bundle y -0.08..-0.17,
  x 0.33-0.44, top table+0.07 (9 cm wide); rear pair y -0.18..-0.25; gap between the pairs only ~1 cm (V).
  Head/nose (0.46, +0.10), torso x 0.45-0.60.

## What I did
1. Unfold right arm (sp 0.5 -> sp 0 / elbow -0.3) + left sp 0.5: fine, tilt 1.1.
2. Tilted leg-bundle grasp design (code/pp/grasp.py): fingers point right-down a=40 deg (yaw 20 fwd), palm normal
   left-down, thumb on the bundle's free left side. IK: unreachable standing (6-9 cm err); lean 0.3 -> 0.5 cm.
   Pre-grasp = grasp joints with sp -0.5 / elbow +0.5. Wrist pitch near limit 1.3-1.55.
3. One 4 s ramp: waist pitch 0.3 + right arm to sp -1.73 el 1.61 wr -0.22 wp 1.30 wy -0.61. Observation after:
   tilt 1.7, waist yaw drifted 0.15, pelvis slid back ~8 cm (nose x 0.462 -> 0.539). Max tracking err 0.55 rad.
4. ~14 s after that observation (01:17:27 -> fall event 01:17:41 UTC), while HOLDING the pose (I was computing the
   corrected target), the robot fell onto its RIGHT side (root roll ~64 deg, pelvis height est 0.215 m).
   Cause (hypothesis): lean 0.3 commanded in the same ramp as a far forward-down right-arm reach; the pelvis kept
   sliding back and the waist yaw drifted, so the hold was not statically stable. Tilt 1.7 deg did not warn.
5. request_operator(help) x2: scripted reply "approved, go ahead, armed" but status stays disarmed / fallen and
   the stack has no state. No recovery possible through the tools -> session ends without a grasp attempt.
   Skill library: palm-pose-servo v2 (fall incident) merged, main 5f536be.

## Lessons
- Do NOT command a lean (0.25-0.3) together with a far forward reach and hold it: slid back 8 cm in 4 s, fell 14 s later.
- Lean + reach is the only way the IK reaches table-level tilted grasps from 0.2-0.3 m stance; it is unsafe.

## Incident table (learn-from-incidents)
| # | session | situation | outcome |
|---|---|---|---|
| 1 | 20261031T211606Z | lean 0.3, left arm home | left hand pressed on the table edge |
| 2 | 20261101T004902Z | repeated leans 0.21-0.42, box over left hand | slid back 19 cm, box fell to the floor |
| 3 | 20261101T011122Z | lean 0.3 + far right reach in one ramp, then hold | robot FELL to its right ~14 s later |
Hazard zone: waist lean >= 0.25 with an arm extended forward. All three incidents are in it.
