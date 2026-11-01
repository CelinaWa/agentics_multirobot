# PROGRESS — puppy_box (session 20261031T234111Z)

## Status (t=750 s of 3600)
- Robot armed, leaning waist pitch 0.35 (meas ~0.32), right arm back (RSP 0.6, RE 0.1), left arm in claw pose above puppy.
- Grasp attempts: 2 failed with the 45-deg "claw" (fingers forward-down, thumb back-down, closing along x). No resets used yet.
- Puppy: lying on side, head LEFT (+y), legs to robot; torso centre ~ (0.51, 0.0) pelvis after the body slid back; top ~ table+0.115.
- Box: LEFT of puppy (+y), label wall seen at y~0.40-0.47, x 0.53-0.65 (rim z ~ table+0.14). Needs a proper look.

## Code (code/puppy/)
- g1.py: G1 helper (observe, move(targets, secs, hands) ramps from last commanded row; clips to limits).
- ik.py: model IK (scipy) for palm pose in pelvis frame; claw_rotation(side, tilt).
- servo.py: servo() IK + (cmd - meas) bias iteration; relative(d, bias) moves palm by d with a fixed bias.
- scene.py / locate.py: depth -> pelvis cloud (own deprojection with obs intrinsics + camera pose; matches pixel_to_position).

## Findings
- Waist yaw does NOT track (cmd 0.4 -> meas 0.06); it drifts -0.17..-0.22 when leaning with left arm out.
- Leaning 0.25 with right arm home put the right fingers on the table edge (wrist pitch -0.89): set RSP 0.6, RE 0.1 first.
- Left arm free tracking bias (cmd - meas) in the claw pose: SP -0.3, SR -0.4, SY +0.3, EL +0.5, WR +0.35, WP -0.25.
- Pelvis frame drifts: table z went -0.12 -> -0.06 over the session (body pitch / slide back). Re-measure before each reach.
- FK of the hand agrees with depth on the hand to ~2 cm.
- Contact: during the last 5 cm of descent the hand slides +4..5 cm in y and -3 cm in x (compliance).
- Grasp 1 (neck, servo diverged under contact) and grasp 2 (tail half of torso, tips 5 cm above table): fingers stall on close,
  but go to fist on a 5 cm lift -> plush slips out (only the top was pinched).

## t~1100 s: reach blocker
- After reset 1 (puppy back at ~(0.40,0.05), torso x 0.33-0.47, y -0.07..0.23, top table+0.115).
- Box measured from rim points in depth (z = table+0.144): right wall y~0.44-0.49 running x 0.52 -> 0.75; near wall x~0.48.
  Interior centre ~(0.62, 0.64) pelvis. IK (lean 0.35): nearest interior point 8 cm inside walls is 16-19 cm out of reach.
- Walking judged unsafe: table edge ~0.23 m ahead of pelvis at pelvis height; turns drift 0.1-0.2 m forward.
- Grasp lesson: from above, the claw (thumb/finger tips 8.4 cm below palm at 30 deg tilt, 12 cm at 45 deg) cannot reach
  below the widest point of the 11.5 cm tall torso: the palm hits the top first and the plush slides sideways (+y).
  Next grasp idea: narrow parts (tail ~3.5 cm, or the leg bundle) with the 30-deg claw closing along x.
- 2026-11-01: IK-guard added to servo.py after a bad IK branch swung the left hand across the body (tilt 7 deg, recovered).

## FINAL (t~1800 s of 3600): task NOT completed
- Robot standing still, both arms home (shoulder pitch L/R ~0.6/0.6, elbows ~0.5), hands open, no contact,
  puppy on the table at its reset pose, box untouched (frame 202611010007584800ae).
- 3 grasp approaches failed (rule limit reached; operator informed). Box interior out of reach from this stance.
- Library: hand-claw-pregrasp (draft) merged to main (eb72549). Controller pieces: code/puppy/ (+ run_pregrasp.py).
- Next: a grasp that works on an 11.5 cm tall plush with the small Dex3 (ideas in handoff/README.md), and a stance
  from which the box is reachable (ask the operator to place the robot / box before starting).

## Addendum (t~2000 s)
- Tail grasp with the right hand (claw tilt 0.52, yaw 0.7 across the tail, tips table+1 cm) HELD a 5 cm lift and
  slipped at ~13 cm. Most promising grasp so far. Reset 2 used. retract_home fixed (shoulder 0.75) and verified.
- Final state: standing, arms home (shoulder pitch ~0.6, elbow ~0.5), hands open, no contact, puppy reset, box untouched.
- Library main d3225c8 (hand-claw-pregrasp draft).
