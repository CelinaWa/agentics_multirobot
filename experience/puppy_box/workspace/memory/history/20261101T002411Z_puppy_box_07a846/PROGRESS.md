# PROGRESS — puppy_box (session 20261101T002411Z, sim rehearsal)

## t~7 min (00:31)
- Stack was not started at launch; request_operator(help) -> armed. g1_hold started control.
- Scene: puppy head LEFT, torso ~(0.46, 0.13) pelvis, top table+0.11; table z ~-0.06 (pelvis frame).
- Box LEFT of puppy, seen only with waist yaw cmd 1.3 (meas 0.61; arms/legs contort, undone): floor corners
  (0.29,0.905),(0.609,0.572),(0.342,0.527) -> box x 0.30-0.61, y 0.53-0.92, centre ~(0.47,0.72). Frame 20261101002915355b1c.
- Waist yaw tracks ~45% here (0.5->0.18, 0.9->0.41, 1.3->0.61). Waist roll tracks (0.25->0.23, 0.42->0.40);
  +roll moves the camera view LEFT on the table; -roll moves the left shoulder toward +y.
- IK (left palm, position only, lean pitch 0.4 roll -0.45): x 0.40 y 0.55 z 0.18 misses by 2-3 cm, y 0.60 by 6 cm.
  => box interior likely out of reach. Operator asked to move box/robot: scripted reply "approved, go ahead" only.

## t~22 min (00:46): status
- Box interior confirmed out of reach (place-reach-check run: best 8 cm short at lean pitch 0.4 / roll 0.45).
  Walking judged unsafe (pelvis at table height, table edge ~0.28 m ahead, turns drift 0.1-0.2 m forward) and not
  instructed by the operator (scripted reply only). => full task infeasible from this stance.
- Grasp attempt 1 (palm-down left hand over neck/head, code/pb/topgrasp.py): fingers stalled on top, fist on 5 cm
  lift -> failed. Robot then retracted: standing, arms home, hands open, puppy on table (frame 202611010041282ef48b).
- Claw (tilt 0.52) IK branch did not track (elbow cmd 2.0 meas 0.35) - servo wound up; unwound by commanding measured.
- g1_free_space_move: unavailable (cuRobo list index error).
- Library merged: place-reach-check (draft) + hand-claw-pregrasp v2 (palm_down.py), main f1b000e.

## FINAL (00:49, ~25 min of 60 used): task NOT completed — place infeasible from this stance
- Grasp attempt 2 (palm-down, w_dir, thumb low in front of the head): full fist = missed (hand closed beside the head).
- Robot: standing, waist 0, both arms home (sp ~0.6-0.7, elbow ~0.5), hands open, no contact; puppy on the table
  (dragged ~10 cm toward the robot by attempt 1), box untouched. Frame 20261101004719af7572.
- Library main 4c9c0a2: place-reach-check (draft, new), hand-claw-pregrasp v2 (palm_down.py + evidence).
- Code: code/pb/ (look.py, percept.py, reach.py, claw.py, topgrasp.py). No end-to-end controller: blocked by reach.
- Next: get a stance from which the box is reachable (operator places robot/box), then a grasp that gets the
  fingertips below the plush's equator (side approach, palm vertical) — see handoff/README.md.
