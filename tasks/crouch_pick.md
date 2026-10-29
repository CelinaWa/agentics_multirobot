Pick up the drink can from the low surface in front of you (a chair seat or a low shelf, below your standing hand height), lift it, hold it up for the operator to see, and put it back down on the same surface, then stand back up.

## Scene

<!-- OPERATOR: fill in or delete each line before the session. Do not leave guesses: the agent takes this file as fact.
     Anything left out, the agent must find by observing (it can look around by turning). -->

- The robot starts standing about <distance> m in front of a <chair / low shelf>, facing it.
- The surface: <description, e.g. a chair seat about 0.48 m above the floor, 0.4 m deep>. Nothing else on it.
- The object: one <description, e.g. empty 330 ml aluminium drink can, 6.6 cm diameter, 12 cm tall>, standing upright on the surface, <where on it, e.g. near the front edge, centred>.
- People: an operator at the stop and one at the safety harness. The straps stay attached and slack.

## Goal and success conditions

1. The can left the surface in the hand (observed: a gap under it, or the operator confirms), rose at least 10 cm, and is set back down standing upright on the same surface, not on the floor.
2. The robot ends standing at full height (`g1_crouch` with pelvis_drop_m 0), still, both arms back near the body, hands open.
3. The chair or shelf has not been pushed over, and the robot did not fall or need the straps.

Report success only when all three hold.

## Rules for this session

What the robot cannot do (tool limits, not choices):
- It cannot step backward or sideways: `g1_walk` only moves forward along the heading plus a turn. To get away from something, turn first.
- It cannot kneel or reach the floor. `g1_crouch` lowers the pelvis by up to 0.32 m (about 0.45 m above the floor at the deepest) and leans the torso up to 0.5 rad; even so the hands reach no lower than about 0.35 m above the floor. If the can falls to the floor, hold and ask the operator.
- `g1_crouch` is validated in simulation only (ground truth: commands of 0.1 / 0.2 / 0.3 m gave 0.08 / 0.21 / 0.29 m, tilt under 7°, the body slid 3-19 cm backward and stayed about 9 cm back after rising). On the real robot, start with 0.1 m and read `transition.feedback` and the operator before going deeper.

How to work (from the harness skills; they apply anyway):
- Explore by hand first (manual-robot-control). Only after one full crouch-reach-lift-place-stand by hand should you automate it under `code/`.
- Crouch, observe, then reach: never crouch with an arm already extended, and never walk while crouched. Stand up (`g1_crouch` 0, arms_home true) before any walk or turn.
- Every motion small and slow; observe after each one and trust the measured result, not the commanded target. The pelvis moves whenever an arm reaches far forward.

Limits set for this session (operator's rules):
- Crouch depth at most <depth, e.g. 0.2> m this session; lean at most <lean, e.g. 0.3> rad.
- One can. If the same grasp has failed twice (missed, pushed it, knocked it over), stop and `request_operator` with what you saw.
- Keep the feet at least about 0.2 m from the chair or shelf; the body slides backward while crouching, so re-observe the distance after every crouch.

## Known from simulation on this robot (hints, not rules: verify on the real robot)

- A side grasp worked on the can: fingers closing around the body at about half its height, approaching at the can's height, not from above.
- Crouching 0.2 m puts the pelvis about 0.57 m above the floor; a hand reaching forward and down from there comfortably works a surface at 0.45-0.55 m. Lean adds reach forward but costs balance margin.
- Standing up quickly from a lean with an arm out fell in simulation; the tool takes the arms home first (arms_home) for that reason. Do not override it while holding something far from the body.
- After standing up the body is about 9 cm further from the surface than before the crouch.

## Ask the operator (`request_operator`) when

- the can or the surface cannot be found in two different views,
- the same grasp failed twice,
- anything is on the floor,
- `transition.feedback` shows a tilt over 8° or a rising tracking error while crouched,
- you are unsure whether the straps or the room allow the next motion.
