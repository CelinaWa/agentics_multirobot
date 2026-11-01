# Handoff — puppy_box (G1 + Dex3 under SONIC), session 20261031T211606Z

Result: NOT solved. Two top-down right-hand grasps failed; no pick, no place. Robot left standing safely, box untouched.
Shell (Python) was unavailable, so everything below is from manual g1_step frames; no controller exists yet.

## How to plan each subtask in a differently laid-out scene

### 1. Perceive
- g1_observe; g1_pixel_to_position on puppy points (head, torso hump, tail, paws). Use the PELVIS frame for arm planning
  (the level frame differs by the pelvis pitch, a few cm in z at 0.5 m).
- Box: depth sees through it. Find the rim lines in the colour image and ray-cast them to z = table + 0.14
  (camera pitch ~43-48 deg down, f = 433 px, cx 320, cy 240; camera at ~(0.02..0.06, 0.03, 0.47) pelvis).

### 2. Get the arm out from the table safely
- Home right hand is at the table front edge. Sequence: shoulder pitch -> +0.45, elbow -> -0.5 (forearm up),
  shoulder pitch -> -0.5 with elbow -0.3 (hand high), then extend.
- If leaning (waist pitch), first move the OTHER arm back (shoulder pitch 0.5) or it lands on the table edge.

### 3. Reach (planar model that matched the camera within ~3 cm)
- Right shoulder S = (0.004 + 0.234 sin p, -0.138, 0.044 + 0.234 cos p) for waist pitch p. Upper arm 0.18 m, elbow to
  hand-back 0.27 m. Upper-arm angle a = p + q1; forearm angle below horizontal = a + elbow; shoulder yaw swings the
  forearm sideways (+ = inward).
- Reach limit ~0.45 m from the shoulder: without lean the hand-back x tops out ~0.40-0.44; with p=0.2-0.3 ~0.47.
- Command offsets to expect (cmd - measured): shoulder pitch -0.1, elbow +0.3 (more under load), shoulder yaw -0.15..-0.4,
  waist yaw: does not track at all. Iterate: command, g1_observe, measure the hand with pixel_to_position, correct.

### 4. Grasp (the unsolved part)
- What failed (2/2): wrist roll -1.3 (palm down) + default open hand. The thumb then points straight down 10 cm and
  pushes/rolls the soft puppy before the palm arrives; closing pinches only the top of a 20 cm-wide body; index goes to
  full fist, lift -> slips.
- Recommended next: (a) set g1_hand_shape for the right hand so the open pose keeps the thumb out of the way
  (e.g. thumb_1 ~0.4 as in the robot-interface example, fingers pre-curled 0.5) and test the open shape in the air first,
  checking in the image where the thumb points; (b) aim at a narrower part (neck + front legs bundle ~10 cm) or grasp the
  torso from the robot side with the hand vertical (wrist roll ~0) so thumb/fingers straddle it; (c) descend vertically,
  never move laterally while the thumb is below the body top; (d) success check = middle/thumb stall short of the fist and
  stay stalled during a 3 cm lift.

### 5. Carry and place (never reached)
- Box centre ~(0.51, 0.33) in the start layout: ~0.5-0.6 m from the right shoulder, beyond the right arm's reach;
  near-right inner corner ~(0.46, 0.22) is reachable with lean 0.2-0.3 + shoulder yaw inward (+0.3..0.4 measured).
  Release only when the hand is >=8 cm inside the walls; lift so the puppy bottom clears the rim (rim z ~0.09 pelvis).

### 6. End pose
- Open hand at the raised pose, swing shoulder pitch back to +0.4 with elbow 0 (forearm level, clears the table), then
  elbow 0.6, wrist roll 0, waist pitch 0. Worked (step 858-943), but once the hand ended on the table front edge
  (elbow stuck 0.28 of 0.6): use right shoulder pitch 0.65 (left 0.5) for the final pose so both hands are clear.
- Checked: pre-curled open shape [0,0.4,0,0.5,0,0.5,0] does not move the thumb out of the way noticeably (air test).

Evidence frames: runtime/frames/<frame_id>/ — see memory/RUNS.md for the ids.
