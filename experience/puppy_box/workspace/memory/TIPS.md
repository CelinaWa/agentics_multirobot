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
