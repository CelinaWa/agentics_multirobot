# PROGRESS puppy_box (session 20261102T012011Z, sim rehearsal, 3600 s)
- Step: start. Reusing code/puppy_box (percep, servo, mv, lift) + library plush-pinch-grasp / palm-pose-servo.
- Layout: to measure (differs from last session).
## Attempt log (details in RUNS.md)
- Start: the first frame showed the left hand touching the puppy; the controller's start easing moved the robot
  back (table edge 0.27-0.28 m ahead, not 0.15-0.20) and the puppy fell to the floor. Reset 1.
- Layout: table z -0.077 (pelvis), edge x 0.27-0.28. Box x 0.37-0.65, y -0.46..-0.08 (ray/plane fit, rim z +0.063).
  Puppy left, head left (+y), torso x 0.31-0.45, y 0.06-0.23 -> drifted to y ~0.20 with my touches.
- Grasps (left, pre-curled pinch): 4 closes, all fist (index/middle 1.5): palm resting on top (0.04-0.075) does not
  pinch; with fingers yawed 0 and palm x 0.36-0.38 the fingertips (x 0.49, z 0.06) are past the far edge in air while
  the thumb tip (6 cm below palm) touches first. Palm x 0.32 pressed to z 0.01: thumb stalled only.
  Repeated presses pushed the puppy toward the robot and off the front edge -> floor. Reset 2.
- Waist: cuRobo use_waist chose pitch 0.31 (reduced to 0.18); waist yaw tracks ~50% (-0.44 cmd -> -0.21).
  Servo works once the waist command == measured (servo2.py, unclipped bias): 1 cm in 3 iters.
- Runs 5-7 (deep-curl shape) held at close (middle ~0.89, index ~1.08) but released at <=3 cm of lift or drag:
  a top pinch on the flat 20 cm-wide side of the torso holds only skin. Run 6: hooking the far edge from x 0.385
  rolled the puppy off the front edge during the descent. Resets used 4/5 (by 2026-11-02 01:36).
- ROOT CAUSE (my judgement): after start easing the robot stands 0.27-0.28 m from the edge (task: 0.15-0.20), the
  torso's back edge is at x 0.45 and the known-working hook pinch (palm x ~0.38 pressed to z ~0.025) is out of reach
  (IK residual 0.02-0.03 even with lean 0.2). Proposed: walk forward ~0.08 m with carry=true, both arms tucked.
- Walks (approved by operator): first walk = warm-up turn; 0.02/0.06/0.08/0.12 m commands (carry=true, arms tucked)
  moved the edge only 0.30 -> 0.25 m. Planner standing posture has straighter knees: table z -0.077 -> -0.097.
- Run 8 (reset 5, last): waist yaw cmd -0.6 tracked -0.22; re-commanding it to the measured value while the arm was
  over the puppy swept the forearm across it -> floor.

## FINAL (session ended ~2026-11-02 01:45): task NOT achieved
- Robot standing still, waist 0, both arms back by the body (palms x ~0.01-0.03, z -0.055), hands open (default
  shapes), tilt < 1 deg, ~6 deg heading change from the walks. Box untouched and upright. PUPPY ON THE FLOOR.
- Resets used 5/5. Skill library: plush-pinch-grasp v2 merged (reach_check, settle_waist, deep-curl shape, failures).
- Controller: none end to end; CLI helpers code/puppy_box/{servo2.py (unclipped servo), mv.py, layout.py, obs.py}.
