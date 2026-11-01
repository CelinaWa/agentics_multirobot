# Handoff — puppy_box (G1 + Dex3 under SONIC), session 20261031T234111Z

Result: NOT solved (2nd session). No pick, no place. Robot left standing, arms home, hands open, box untouched.
Previous session's handoff: handoff/README_session_20261031T211606Z.md. Notes: memory/PROGRESS.md, RUNS.md, TIPS.md.
Library skill: `hand-claw-pregrasp` (draft) - perception + pre-grasp servo + unfold/retract, verified 1x unattended.

## Before anything: check the stance
- Measure the box first (perception.rim_points at table+0.144, plus the faint rim in the colour image). If the box
  interior (8 cm inside the walls) is more than ~0.5 m from the shoulder of the hand you will use, the place is
  impossible from here (this session: interior centre (0.62, 0.64) pelvis, IK missed by 16-19 cm at lean 0.35).
  Ask the operator to move the box or the robot BEFORE grasping; do not walk at a table whose edge is 0.2-0.3 m
  ahead at pelvis height (turns drift 0.1-0.2 m forward). The scripted operator answers "approved, go ahead" to
  every question: it does not move things on request.

## How to plan each subtask in a new layout
1. Perceive (code/puppy/scene.py or the skill's perception.py): own deprojection with the observation's
   intrinsics and camera pose; table = 30th percentile of z; object = points above table+0.03; per-y slices of
   points above table+0.08; the largest contiguous blob is the torso (neck dip separates the head). Re-measure after
   every lean or arm move: the pelvis frame pitches and the body slides back (table z went -0.12 -> -0.06).
2. Arm out of home: shoulder pitch 0.6 (back) -> elbow -0.4 -> park (sp 0.3, roll 0.4, elbow -0.8) -> target.
   Before leaning > 0.2 put the other arm back (sp 0.6, elbow 0.1) or its fingers catch the table edge.
3. Reach: skill main()/servo(): IK on the model with measured waist + (cmd - meas) bias iterations, 1-2 cm.
   Waist yaw is useless (does not track). Reach ~0.45-0.5 m from the shoulder; lean 0.35 adds ~8 cm.
4. Grasp (UNSOLVED). What failed on the 11.5 cm tall, ~15 cm wide plush torso with the left Dex3:
   45-deg claw x2, 30-deg deep claw with thumb opened -0.4, claw yawed across the legs. Fingers stall on close
   but go to a fist on a 5 cm lift; palm lands on the top before the tips pass the widest point; on contact the
   hand slides 4-5 cm sideways and pushes the plush (once 20 cm).
   Best so far: the TAIL with the hand on the tail's side (here the RIGHT hand: claw tilt 0.52, yaw 0.7 so it
   closes across the tail, palm 1.3 cm converged, tips to table+1 cm, close fully): fingers stall on the tail and
   it HOLDS a 5 cm lift but slips at ~13 cm as the whole weight comes on. Next: grip closer to the rump (tail
   base), close in two steps (close, press 1 cm down, close again), or lift slower; then lift so the hanging body
   (~30 cm) clears the rim. Other ideas: (b) a two-hand squeeze of the torso from both sides
   (palms facing, wrist roll ~0) if the rules allow two hands; (c) the head/neck with a pre-curled hand shape.
   Success check: middle_0/index_0 stall well short of -1.57 (left) AND stay there during a 3-5 cm lift.
5. Carry/place (never reached): lift only to clear the rim (box rim = table+0.14; hanging length below the hand
   counts), keep the arm close, release above the box centre >= 8 cm inside the walls.
6. End pose: skill retract_home (open, park, shoulder 0.6, elbow 0.6, shoulder 0.5, waist 0); verified twice,
   hands end behind the table edge.
