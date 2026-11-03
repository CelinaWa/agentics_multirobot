# PROGRESS - session 20261102T152531Z_puppy_box_da90c5 (sim, 3600 s)
Layout (start frame 20261102152607fca9bf): puppy centre-left, head LEFT, legs to robot; clear box RIGHT. = same-side layout.
Plan: library plush-pick-place-box-same-side v5 run_v5.py (stance, prep,perceive,hover,shift -> LOOK -> grasp2,lift2 -> LOOK -> carry,release,home).
## Log
- stance: edge .302 -> .24 after 3 walks (odo ~1 cm each) + 3 crouches (fwd .1/.2/.2, ~3 cm only the first .2). Wrists crept to yaw +-.7 over the crouches -> code/da90/retuck.py. Operator: no re-placement.
- run 1 SUCCESS (operator INSIDE, ~15 min in): skill v5 hover found no reach at edge .24-.25 / table z -.07 (puppy top +.004, far .42-.43);
  its lean fallback slid the pelvis back 1.7 cm and refused -> unlean. Walk 0.2 m moved 0. New code/da90/ctl.py: restore (crouch fwd .1),
  prep (c3 prep+perceive; perceive's own range gate refuses gx > .36 but saves state), waist yaw .9 + roll .15 (tilt 5.9, meas yaw .55),
  hover palm far-.10 (DY -.03 after LOOK: hand was over the shoulders), grasp light press (stalled palm .031, top -.002) close: bases .71/.83,
  raise 3 cm (.67/.75), lift to palm .348 (middle 1.14 held, index fist), run_v5 carry (palm .386,-.368,.353 = wall+.09), release, home.
- run 2 FAIL: waist .9/.15 tilt 7.2-7.6 after the robot drifted 2 cm back; 2 empty fists; puppy rotated -> reset 2. regain.py (crouch fwd .2) brought the box 2.3 cm closer.
- run 3 SUCCESS (operator INSIDE): WAIST_YAW=0.7 (tilt 5.4). Default now 0.7 (PARAMS). ~40 min used. Next: library skill v6 + handoff + merge.
- library merged 0f6d388 (v6); handoff + TIPS written. End state: standing, arms beside hips, hands open, puppy INSIDE (run 3).
