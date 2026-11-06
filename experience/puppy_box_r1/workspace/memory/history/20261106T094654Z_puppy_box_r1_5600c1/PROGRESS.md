# PROGRESS - session 20261106T094654Z_puppy_box_r1_5600c1 (sim, 7200 s budget)

## Current step (FINAL, ~4600 s of 7200)
FINAL STATE (j21): puppy INSIDE (operator), robot .44 m behind the edge, standing (tilt .85), arms home, grippers open.
6 inside / 25 attempts (j2, j4v, R2 walk-in, j13, j14, j21). Library v4.1 merged (main f3b9bcd).
KEY FINDING: lift completes 1/9 with paw x < .32, 5/6 with x >= .32 and y <= -.18 (j21 = prospective confirmation).
Resets 5/10, stance placements 21/30. Next session: make the walk-in land at paw x .32-.38, y -.19..-.24 (no back steps).
Handoff written (handoff/README.md). Next session: improve the LIFT (~50%): try moving the hand toward the body's
centre of mass while lifting, or the box-beside layout; walk-in: stop at paw x <= .39 (visible) instead of stepping back.

## (start of session) Start: run library skill r1-leg-lowtilt-lift-advance-carry v3 (python code/puppy_box/run_skill.py <tag>) from
JITTERED operator grasp stances (+-3 cm this session), count per stance offset in memory/RUNS.md, fix weakest stage.
Then >= 1-2 reset walk-ins (python code/puppy_box/home.py 0.6; python code/puppy_box/walkin3.py 0.38 0.35 6).

## Inherited (verify): see handoff/README.md (90b103): 7/11 placements at exact stance, 0/4 lifts at walk-in stances.

## Runs this session (details in memory/RUNS.md) - at ~57 min
Placements: j2 (stance, horizontal), j4v (stance, vertical, dropped past the wall), R2 (WALK-IN reset 2, ctl.py v4),
j13, j14 (stance, ctl.py v4 final). Total 5 inside / 17 attempts; final config 3/3.
Later failures: j12 lift abort + pull-back -> puppy OFF the table (pull-back disabled); R3 (walk-in) ball gate refused.
Failures: lift aborts j1 j3 j10 j7v j8v R1 (safe), lift-top sudden loss j9v (puppy on the table), carry drops j6v (on rim),
j10b (in front of box), grasp gate j11 (ball_w .033 too strict), grasp miss j5v (robot drifted after walk-back).
Incident: j3 overhang re-request after a put-back pushed the puppy OFF THE TABLE.
Code: code/puppy_box/lowtilt_v4.py (skill v4 copy), code/puppy_box/ctl.py (in-place retries), look_waist.py,
look_only.py, dry_v.py, ov2.py.  Run: python code/puppy_box/ctl.py <tag> tries=3   (from a grasp stance)
Walk-in: python code/puppy_box/home.py 0.6; python code/puppy_box/walkin3.py 0.39 0.35 7; then ctl.py.
Vertical straddle (fingers 0,0,1): lift widths steady but SUDDEN losses (carry 0/2 held, top-of-lift loss 1) -> dropped.
