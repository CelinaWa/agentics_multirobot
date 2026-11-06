# PROGRESS - session 20261106T080133Z_puppy_box_r1_90b103 (sim), 2 h wall clock

## Current step (t ~ 77 min sim clock) - DONE, final state: puppy INSIDE (s17), robot .42 m back, arms home, open
Controller = library skill r1-leg-lowtilt-lift-advance-carry v3 (verified, merged, depth .03 default). Placements:
s7, s9, s11, s13, s14, s16, s17 (operator ground truth INSIDE; s11, s13, s14, s16, s17 fully unattended skill runs).
Walk-ins from resets reach a stance (straight walks, walkin3.py) but the lift failed 0/4 there.

## Run it (operator grasp stance)
    python code/puppy_box/run_skill.py <tag>      # skill main(): HOME, look, grasp, low-tilt lift, advance, carry,
                                                  # lower, release, retreat (walk back, HOME); ~80 s
    request_operator(help): "Question only, please do not move anything: is the puppy inside the box?"
Reset walk-in: python code/puppy_box/home.py 0.6; python code/puppy_box/walkin3.py 0.345 0.32 7; then run_skill.

## Confirmed results
- r1_leg_lift aborts 4/5 (width +3-4 mm at lift step 2, then falls), also with the engineering inputs. Its approach
  tilts to .5. Own servo-off lift with a LEVEL approach (z component <= .15): lift held 6/9 at the operator stance.
- 6 cm level advance at the top -> carry .16 m (2 steps) -> released inside 5/5 of the completed lifts with the advance
  (s7, s9, s11, s13, s14; s14 dropped on step 2 after passing the wall and still landed inside).
- Abort safety: retrace the lift path downward, open low (s12 opened high -> puppy off the table, fixed).
- Walk-in: straight walks + re-select the overhanging paw each time reaches x .34-.37 in 4-5 calls; lateral error
  3-5 cm and 3-6 deg remain, and at those stances the lift aborted 4/4 (R1, R1b, R2, R3).

## Failed approaches / lessons
- Lateral moves by turn pairs (W0), tracking a paw across walks (R1 jumped to another paw), r1_leg_lift with any input
  tried, pushing a bridged puppy with the arm (s6), releasing after a 3-step carry (s6).

## Funnel (operator stance, final recipe with the top advance: s7-s17)
stance 11 -> grasp 11/11 -> lift 7/11 (s8, s10, s12, s15 aborted; depth .02 5/9, .03 2/2) -> carry 7/7 -> inside 7/7.

## Next concrete step (next session)
Make the walk-in stance match the operator stance (paw (.32,-.20), square), or make the lift robust at paw x .34-.37 /
y -.15..-.18 (e.g. test the lift profile at those stances with servo-on pregrasp re-measure, or the waist to bring the
shoulder closer). See handoff/README.md.

## Budget
resets 3/10 (R1, R2, R3 = walk-ins), stance placements 17/30, falls 0, objects off the table 1 (s12).
