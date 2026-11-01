# Handoff: puppy_box (G1 + Dex3 under SONIC), session 20261101T011122Z (sim rehearsal, 5th session)

Result: NOT solved. The ROBOT FELL during the first pre-grasp reach. The scripted operator could not re-stand it, so
the session ended after ~36 min. Earlier handoffs: README_session_20261101T004902Z.md (4th), README_session_20261101T002411Z.md,
README_session_20261031T234111Z.md, README_session_20261031T211606Z.md.
Library: palm-pose-servo v2 (fall incident added) merged, main 5f536be. Session code: code/pp/ (robot.py helpers,
grasp.py tilted bundle-grasp geometry, scene.py height map), output/m*.py (the motions that were sent).

## Most important: a fall ends the session
- Lean 0.3 commanded in the SAME 4 s ramp as a far forward-down right-arm reach (palm ~0.43 m ahead, wrist pitch
  1.3) -> tilt 1.7 deg right after, the pelvis slid back 8 cm and waist yaw drifted +0.15 -> robot fell onto its RIGHT
  side ~14 s later while holding. Three sessions now show that leaning with an arm out is the hazard zone (slid back
  19 cm, box knocked to the floor; now a fall).
- Rule for next time: lean in its own ramp, at most ~0.2, observe twice a few seconds apart (tilt AND pelvis drift
  against a landmark such as the puppy's nose) before reaching. Keep the reach short and never leave a leaned reach
  held while you compute: plan the next targets BEFORE the reach, or retract at once.

## Order of work in a new layout
1. Start: "console start failed" -> request_operator(help) -> armed -> g1_hold. The first frame is wrong again
   (here ~7 cm off in y): make one small motion (unfold the arm) and re-measure before planning.
2. Check that the box is flat (depth over it ~ table+0.14-0.16 and FLAT here; sloped = resting on a hand).
3. Measure with `python3 code/pp/scene.py <frame_id>`: ASCII height map, x rows, 1.5 cm columns from y +0.6
   downward. Deproject chosen pixels with code/pbox/scene.py cloud(). Leg bundles show as 2-3 (4-7 cm), the torso
   as 5-6, the box as 7 (rim/label height).

## Grasp (still UNSOLVED after 5 sessions)
- Sim puppy: rigid capsules. Here the front leg pair = one ~9 cm wide, 7 cm tall bundle (y -0.08..-0.17), rear pair
  right beside it (y -0.18..-0.25), ~1 cm V-gap between them. Dex3 open span thumb-index ~19.5 cm in plane, closed gap
  ~8 cm: the object must sit ~6 cm along the fingers and ~4.5 cm off the palm (model `g1_model.py hand`).
- Tried design (not executed to contact): code/pp/grasp.py pose(c, a=40, yaw=20). Fingers point right-down at 40
  deg, the palm normal points left-down, the thumb goes to the free left side of the front bundle, and the fingers
  cross the V-gap and land on the rear legs (17.5 cm long fingers cannot fit in a 1 cm gap). IK needs waist lean
  >= 0.3 for it (6-9 cm short standing): that is what led to the fall. Only use it with a smaller lean, the
  stance closer to the table, or after asking the operator to place the puppy nearer (x <= 0.35).
- Untested ideas: g1_hand_shape with a pre-curled "open" shape (span ~10 cm) so the hand fits one bundle without
  touching its neighbour; grasp the rear bundle from its free right side.

## End pose that works (earlier sessions)
palm-pose-servo `retract_home(ctx, 'right')`.
