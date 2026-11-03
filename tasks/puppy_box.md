Grasp the puppy doll lying on the table in front of you and place it into the open clear plastic box next to it, then return to a safe standing pose. No walking is needed: everything is within reach from where you stand.

## Scene

<!-- OPERATOR: nothing here needs measuring. Set the scene up by eye as below; the robot measures every distance itself. -->

- The robot stands at a wooden table, facing it squarely, roughly 0.3 m back from the front edge so its hanging hands start clear of it. The table top is roughly at the robot's pelvis height (its exact height is yours to measure: record it as `scene.table_height_m`). Tuck both arms, measure the edge, then ask the operator to slide the table (with the puppy and the box on it) up to you until its edge is about 0.2 m ahead of your pelvis; tell them "closer" or "further" from your own measurement until it is.
- The object: one plush puppy doll, cream coloured, about 30-40 cm nose to tail, soft and floppy, lying on its side on the table within reach, its legs toward the robot. Where it lies and which way its head points differ between tries: find it in an observation first.
- The box: an open-top clear plastic storage box, roughly 40 × 30 cm and 15 cm tall, standing on the table next to the puppy: on its left, on its right, or behind it (the puppy between you and the box), a few cm to about 10 cm from it. Its walls are clear: the colour image shows them faintly and the depth image may look through them to the table. Measure its rim from your own observations.
- People: an operator at the stop and one at the safety harness. The straps stay attached and slack.

## Goal and success conditions

1. The puppy's body is inside the box (seen in an observation looking into the box, or confirmed by the operator). A leg or the head hanging over the rim is acceptable; a puppy balanced on the rim or lying beside the box is not.
2. The robot ends standing still, both arms back near the body, hands open, not touching the table or the box.
3. The box has not been knocked over or pushed off the table, and nothing has fallen to the floor.

Report success only when all three hold.

## Rules for this session

What the robot cannot do (tool limits, not choices):
- It cannot step backward or sideways: `g1_walk` only moves forward along the heading plus a turn. This task needs no walking at all; do not walk unless the operator tells you to.
- It cannot reach the floor: if the puppy falls off the table, hold and ask the operator to reset the scene.

How to work (from the harness skills; they apply anyway):
- Explore by hand first (manual-robot-control). Only after a full successful grasp-lift-place by hand should you automate it under `code/`.
- Every motion small and slow; observe after each one and trust the measured result, not the commanded target.
- Keep the arms clear of the table edge on the way down: approach from above the table surface, not from in front of it.

Limits set for this session (operator's rules):
- One puppy. If the same grasp has failed twice (missed, pushed it away, dropped it), stop and `request_operator` with what you saw.
- Use only the right hand, wherever the puppy and the box are (the demonstrations used the right hand). Before moving it, tuck the left arm back and up (shoulder pitch about +0.45, elbow about −0.5) and keep it there for the whole task. The left hand must not touch the puppy at any point: an attempt where it does, even a brush, does not count. Check the left arm is still tucked before every grasp and lift.
- Lift no higher than needed to clear the box wall, carry the puppy close to the body, and release only when the hand is above the inside of the box.
- Never push the box: if a hand touches a box wall, back off upward first.

## Known from the demonstrations and from simulation (hints, not rules: verify on the real robot)

- The demonstrations (teleoperation, 29 episodes) grasp the puppy from above with one hand, fingers closing around the soft body at its middle, lift it 10–20 cm, swing the arm sideways over the box and open the hand above the box's centre. The whole thing takes about 20 s by hand.
- A plush body compresses: close the hand fully, then check it has stopped on the doll (hand joints stop moving before the full fist) before lifting. If the fingers reach the full fist, nothing is in the hand.
- The puppy is longer than the hand: grasp the middle of the body, not a leg or the head, or it hangs and catches on the rim.
- Simulation on this robot: the pelvis moves back 5–10 cm whenever an arm reaches far forward while leaning, so the hand lands short of the target; aim slightly past the first estimate and re-observe. Do not use `g1_crouch` to gain reach at a table. A waist lean is the hazard zone: six sim sessions (2026-10-31/11-01) saw the pelvis slide back 8-19 cm and the robot fall to its side once (lean 0.3 held with an arm stretched forward). Lean at most 0.2 rad, in its own ramp, observe twice a few seconds apart before reaching, never hold a leaned reach while you think (retract or plan the next rows first), and stand closer to the table instead of leaning further.
- Simulation on this robot, reach: standing upright the palm reaches about 0.32-0.36 m ahead of the pelvis at table height. To reach further sideways, turn the waist (yaw, up to about ±0.5 rad) instead of leaning. A waist turn swings the whole upper body, so the idle hand sweeps sideways across the table unless it is tucked first.
- Wrist roll 0 means palm facing inward (toward the body centre). For a top-down grasp the right hand needs the palm down (roll of about −1.3; the left hand mirrors this). Check the palm direction in an observation before descending.
- Release above the box centre, not near a wall: a soft body dropped near the rim folds over it.
- Simulation session 2026-10-31 (two failed top-down grasps): with the right wrist rolled to −1.3 (palm down) the Dex3 thumb points straight down, about 10 cm below the back of the hand. It touches the plush first and rolls it away before the palm arrives, and closing then pinches only the top of the 20 cm wide body. Treat the thumb tip as the lowest point of the hand; descend vertically only; consider a vertical hand (wrist roll near 0) that straddles the torso with thumb and fingers, or the narrower neck-and-front-legs bundle.
- Simulation, start position: the standing hands hang about 13 cm ahead of the pelvis at table height. Started with the hips closer than about 0.28 m from the edge, the fingers rest on the table and the start pushed the robot 10-16 cm back. Start about 0.3 m back, tuck both arms (shoulder pitch +0.45, elbow -0.5), then ask the operator to slide the table up. Do not walk at the table: in simulation a short `g1_walk` (0.12 m) either stepped in place (8 of 12) or overshot by 0.25-0.36 m.
- Simulation, measuring the table edge from depth: the standing hands hang at table height 0.20-0.26 m ahead of the pelvis, so "the nearest point at table height" is your own hand (sim: read 0.20 m for a true 0.33 m). Tuck the arms before measuring, or ignore points near your hands.
- The right hand at its home pose sits at the table's front edge when the hips are 0.2 m from it: move the arm back (shoulder pitch +0.45) and the forearm up (elbow −0.5) before swinging it forward over the table, and bring the other arm back (shoulder pitch 0.5) before leaning.
- Commanded minus measured joint offsets under load were large in simulation (elbow +0.3 rad, shoulder yaw up to −0.4, waist yaw not tracking at all): plan from the measured joints and the camera after every step, not from the command.

## Ask the operator (`request_operator`) when

- the puppy or the box cannot be found in two different views,
- the same grasp failed twice,
- anything is on the floor,
- a hand is in contact with the box or the table and you cannot see how,
- you are unsure whether the straps or the room allow the next motion.
