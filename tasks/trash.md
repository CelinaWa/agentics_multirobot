Pick up the drink can standing on the desk in front of you and drop it into the trash bin, then return to a safe standing pose.

## Scene

<!-- OPERATOR: fill in or delete each line before the session. Do not leave guesses: the agent takes this file as fact.
     Anything left out, the agent must find by observing (it can look around by turning). -->

- The robot starts standing in front of a desk, about <distance> m from its edge.
- The object: one <description, e.g. empty 330 ml aluminium drink can, about 6.6 cm diameter, 12 cm tall>, standing upright on the desk.
- The trash bin: open top, opening about <width> × <depth> cm, rim about <height> m above the floor, standing on the floor <direction and distance from the robot's start, e.g. "about 1.5 m behind the robot, to its left">.
- People: an operator at the stop and one at the safety harness. The straps stay attached and slack.

## Goal and success conditions

1. The can is inside the bin (seen in an observation looking into the bin, or confirmed by the operator), not on the floor, not on the desk, not in the hand.
2. The robot ends standing still, both arms back near the body, hands open, clear of the desk and the bin.
3. The desk and the bin have not been pushed or knocked over.

Report success only when all three hold. A clean tool return or a can that merely moved is not success.

## Rules for this session

What the robot cannot do (tool limits, not choices):
- It cannot step backward or sideways: `g1_walk` only moves forward along the heading plus a turn (the planner does not step backward). To get away from something, turn first.
- It cannot kneel or reach the floor: `g1_crouch` lowers the pelvis by at most 0.32 m and the hands reach no lower than about
  0.35 m above the floor. If the can falls to the floor, hold and ask the operator to reset the scene.

How to work (from the harness skills; they apply anyway):
- Explore by hand first (manual-robot-control). Only after a full successful pick-carry-drop by hand should you automate it under `code/`.
- Every motion small and slow; observe after each one and trust the measured result, not the commanded target.
- Before any turn or walk, look where you are going; the bin may be behind you and people beside you.

Limits set for this session (operator's rules):
- One can. If the same grasp has failed twice on it (missed, pushed it, knocked it over), stop and `request_operator` with what you saw; do not keep pushing the can further away.
- Never walk with an arm extended or with an open hand around the can. Carry the can high and close to the body.
- Keep the feet at least about 0.3 m from the bin and the desk; reach over from there.

## Known from simulation on this robot (hints, not rules: verify on the real robot)

- A side grasp worked: fingers closing around the can's body at about half its height, approaching at the can's height, not from above.
- After lifting, pull the arm in and observe again before walking: a can that was only pushed over can look lifted while the arm hides it.
- The pelvis moves back 5–10 cm whenever an arm reaches far forward while leaning; the hand lands short of the target.
- Walks are conservative (a 1 m command gave about 0.8 m, a 30° turn about 22°). Walk in short legs and re-localize the bin after each.
- Approach the bin on the carrying hand's side and release above the centre of the opening, not near its wall (a can released 3 cm from the wall bounced out).
- Short carrying walks near the bin sometimes marched in place. If two legs did not advance, lean toward the bin instead, or ask the operator.
- Do not use `g1_crouch` to gain reach at a desk: leaning forward there pitched the pelvis back 4–8° and slid it back 2–3 cm, no reach gained. Waist pitch about 0.3 rad through `g1_step` rows (with the taller walk stance) is what brought the can within reach.
- Short walks of 0.08–0.12 m toward the desk often march in place (0–3.5 cm). Only the first walk after the warm-up advanced the full command. Plan the approach so one walk does it.
- Seat the can against the palm before closing. Closing with a 3–4 cm gap pulls the can into the hand and holds through turns in place, but the can slipped out during two 0.5 m carry walks with the arm held out to the side.
- Carry with the elbow tucked and the hand close to the chest, and read the hand joints after every walk: index and middle proximal joints drifting from about −0.85 toward −1.57 (full fist) means the can is slipping or gone.
- Left wrist roll 0 means palm facing inward (toward the body centre), thumb inward: that is the side-grasp orientation for a can approached from its left. Roll +1.33 is palm down. The right hand mirrors this.
- Carry turns in place: a 45° command measured about 37°, 40° about 33°; four calls gave 138°.

## Ask the operator (`request_operator`) when

- the can or the bin cannot be found in two different views,
- the same grasp failed twice,
- anything is on the floor,
- a person is in the path and stays there,
- you are unsure whether the straps or the room allow the next motion.
