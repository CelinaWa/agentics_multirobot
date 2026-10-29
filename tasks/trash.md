Pick up the drink can standing on the desk in front of you and drop it into the trash bin, then return to a safe standing pose.

## Scene

- The robot starts standing about 0.5 m in front of a desk (desk top roughly at the robot's hip height, 0.72–0.78 m).
- One empty aluminium drink can (330 ml: about 6.6 cm diameter, 12 cm tall) stands upright on the desk, within arm's reach (0.3–0.55 m ahead, up to 0.25 m to either side). Nothing else that matters is on the desk.
- An open-top trash bin (opening about 25 × 25 cm, rim about 0.6 m above the floor) stands on the floor, roughly 1–1.5 m behind the robot on its left side. <!-- operator: edit the bin's position before the session if the room is different -->
- People may be near the robot: an operator at the stop and one at the safety harness. The straps stay attached and slack.

## Goal and success conditions

1. The can is inside the bin (visible in an observation looking down into the bin, or the bin is confirmed by the operator), not on the floor, not on the desk, not in the hand.
2. The robot ends standing still, both arms back near the body, hands open, at least 0.5 m from the desk and the bin, facing away from both.
3. The desk and the bin have not been pushed or knocked over.

Report success only when all three hold. A clean tool return or a can that merely moved is not success.

## Rules for this session

- Explore by hand first (manual-robot-control). Only after a full successful pick-carry-drop by hand should you automate it under `code/`.
- One can, one attempt chain. If the same grasp has failed twice on this can (missed, pushed it, knocked it over), stop and `request_operator` with what you saw; do not keep pushing the can further away.
- If the can falls to the floor, do not try to pick it from the floor. Hold, and ask the operator to reset the scene.
- Never walk with an arm extended or with the hand open around the can. Carry the can high and close to the body.
- Do not approach the bin closer than about 0.3 m with the feet. Reach over the opening from there.
- Before any turn or walk, look where you are going (active perception); the bin is behind you and people may be beside you.
- Keep every motion small and slow; the first time you try anything on this robot is also the first time this robot does it.

## Known from simulation on this robot (treat as hints, verify on the real robot)

- A side grasp works: fingers closing around the can's body at about half its height, approaching from the side at the can's height, not from above.
- After lifting, pull the arm in and observe again before walking: a can that was only pushed over can look lifted while the arm hides it. Check that the can is no longer on the desk and that there are depth points where the hand holds it.
- The pelvis moves back 5–10 cm whenever an arm reaches far forward while leaning. Expect the hand to land short; re-observe after the reach instead of trusting the commanded target.
- Walking commands are conservative: a 1 m walk came out at roughly 0.8 m and a 30° turn at about 22° in simulation. Walk in short legs, re-localize the bin after each leg.
- Approach the bin on the carrying hand's side. Release above the centre of the opening at about hip height, not near a wall of the bin: a can released 3 cm from the wall bounced out.
- Short carrying walks near the bin can march in place without advancing. If two legs did not advance, lean toward the bin instead of walking again, or ask the operator.

## Ask the operator (`request_operator`) when

- the can or the bin cannot be found in two different views,
- the same grasp failed twice,
- anything is on the floor,
- a person is in the path and stays there,
- you are unsure whether the straps or the room allow the next motion.
