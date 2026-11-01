# TIPS — G1 SONIC, table-top reaching (puppy_box)

- Condition: standing 0.2 m from a 0.73 m table, right hand at home sits AT the table front edge (x~0.21, z~-0.065 pelvis).
  Tip: first command right shoulder pitch +0.45 (arm back), then elbow -> -0.5 (forearm up), then shoulder pitch forward;
  the hand never crosses the table edge below the top. Evidence: steps 0-120, frames 20261031212117.., 202610312123...
- Condition: palm-down right hand (wrist roll -1.3). Tip: the thumb hangs ~10 cm below the hand back; treat the
  thumb tip as the lowest point for clearance; lift >=5 cm before any lateral move or it drags the object.
- Condition: leaning waist pitch 0.2-0.3 with left arm at home (shoulder pitch 0.2). Tip: left hand presses the table edge;
  set left shoulder pitch 0.5 first.
- Condition: right arm extended forward ~0.45 m. Tip: commanded-vs-measured offsets are large (elbow +0.3, yaw up to -0.4,
  waist yaw ignored); correct from measured joints + camera each call; always g1_observe after a step before measuring.
- Condition: grasp closure check. Tip: index reaching 1.53/1.70 (full) = missed; if middle_0/thumb_1 stall but go to full
  on a 3 cm lift, nothing is held.

## Session 20261031T234111Z additions
- Condition: any arm reach with SONIC. Tip: solve IK on the robot model with the MEASURED waist, then command
  IK + 0.8*(last cmd - measured) for 2-3 iterations (servo.py); converges to 0.5-2 cm in free space.
  Never iterate this under contact (wind-up of 1+ rad) - descend with `relative()` and the pre-contact bias.
- Condition: commanding waist yaw. Tip: it does not track (0.4 -> 0.06) and the arms swing instead; it drifts
  -0.15..-0.2 when leaning with an arm out. Never use it to look around.
- Condition: before any IK move. Tip: refuse IK residual > 2 cm or joint jump > 1-1.6 rad; a far target once
  picked a branch that swung the left hand across the body (tilt 7 deg).
- Condition: plush 11.5 cm tall lying on its side. Tip: a top-down claw cannot reach below its widest point
  (tips 8.4 cm below the palm at 30 deg tilt); the palm lands on top and the plush slides sideways.
- Condition: clear box. Tip: depth returns the rim and label at rim height (table+0.144): threshold the cloud
  at that height to find the walls (perception.rim_points).
- Condition: moving an open hand sideways near the plush. Tip: lift so all tips are >=5 cm above the plush top
  first; a sideways move dragged it 20 cm.
- Condition: thin plush part (tail ~3.5 cm) near the hand's side. Tip: claw tilt 0.52 closing across it, tips to
  table+1 cm, full close -> fingers stall (right thumb_1 ~-0.4, index_0 ~1.1) and hold a 5 cm lift; slips by 13 cm.
  Evidence: frames 202611010012352fb7fb (held), 2026110100125141aa5f (slipped).
- Condition: retracting an arm to home at a table 0.23 m ahead. Tip: shoulder pitch 0.75 before straightening the
  elbow to 0.6, else the right fingers land on the table front edge (elbow stuck ~0.2).

## Session 20261101T002411Z additions (sim)
- Condition: place target beside the object, possibly outside the view. Tip: run library place-reach-check
  BEFORE grasping (waist roll +0.4 looks left; IK over leans). A box 0.53+ m to the side was 8 cm out of reach.
- Condition: extended arm, any waist yaw command. Tip: yaw tracks ~45 % and swings the palm ~15 cm; command 1.3
  only as a brief look (contorts arms/hips), then re-measure the scene (pelvis frame shifted, table z -0.06 -> 0.0).
- Condition: palm positioning in free space. Tip: palm_down.go (wrist roll fixed 1.3, solve 4 proximal joints)
  converges 1-2 cm when the bias clip is 0.9; clip 0.4 stalls at ~6 cm.

## Session 20261101T004902Z additions (sim)
- Condition: first observation right after start. Tip: it placed the scene ~14 cm too close (pelvis frame not
  settled); re-observe after the first motion before planning on positions. Evidence: 20261101004938c428c9 vs 2026110100560218695d.
- Condition: an object (clear box) at the table front edge with a hand visible THROUGH it / depth showing a sloped
  surface above rim height. Tip: it is resting on the robot's hand; do not lean or move that arm back before
  asking the operator; the box fell to the floor when the robot slid back. Evidence: library palm-pose-servo box_fall_montage.jpg.
- Condition: waist leans with an arm forward. Tip: the pelvis slides back a few cm per lean (19 cm cumulative,
  operator distance 0.48 -> 0.67 m) and waist yaw drifts +0.15; re-localize after every lean, prefer one lean.
- Condition: g1_free_space_move available. Tip: palm-down yaw -45 (wxyz [0.6533,-0.6533,0.2706,-0.2706]) plans for the
  right hand in front of the right hip; +-90 / +45 / palm z < 0.05 refuse. Execute from Python: the reply echoes
  every row (>25k tokens).
- Condition: plush with rigid capsule legs (~6.5 cm). Tip: a wrap needs the leg under the PALM centre (~6.5 cm
  along the fingers) and the palm <= 2 cm above the leg top; right palm-down cannot get below z~0.04 at x 0.30
  without a lean. Leg under the fingertips -> stall then fist on lift (3/3 fails).
- Condition: scripted operator. Tip: `reset` restores only the puppy; a box on the floor stays there.

## Session 20261101T011122Z additions (sim)
- Condition: waist lean 0.3 commanded together with a far forward-down arm reach (palm ~0.43 m ahead). Tip: the robot
  fell sideways ~14 s later while holding, though the tilt was 1.7 deg right after. Lean in its own ramp first, at most ~0.2,
  watch the tilt and pelvis drift for a few seconds, and never leave a leaned reach held while you compute.
  Evidence: frame 20261101011727cb1aa5, library palm-pose-servo v2.
- Condition: the scripted operator after a fall. Tip: it answers "armed, continue" but cannot re-stand the robot;
  a fall ends the session. Treat a fall as unrecoverable.
- Condition: legs of the sim puppy. Tip: the front and rear leg pairs are bundles ~9 cm wide, 7 cm tall, with a ~1 cm
  V-gap between the pairs. A single-leg wrap has no room for the thumb or the fingers.

## Session 20261101T012043Z additions (sim)
- Condition: a new layout, before any grasp. Tip: run place-reach-check span mode (one arm for grasp AND place);
  here puppy and box were 0.67 m apart and neither arm spanned them (23 cm). Evidence: library place-reach-check v2.
- Condition: scripted operator asked to move objects or allow walking. Tip: it repeats a generic "approved, go
  ahead"; nothing moves and it is not an explicit walking permission.
- Condition: left palm-down hand, fingers across legs lying along x (yaw -60..-90). Tip: unreachable at x >= 0.33
  without lean; fingers-forward (yaw 0) reaches z 0.05 at x 0.33.
