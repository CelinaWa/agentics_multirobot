Two robots carry a box across an office: a Unitree G1 (Dex3 hands) and a Unitree R1 (PiPER grippers). Both robots
are in the same room with the box, so they can touch the same box and each other.

## The scene

A room about 3.2 m wide and 4 m deep. Looking in from the open side: a wall on the left with a wire shelf and a wooden
shelf; the back wall with, from the left corner, a white panel, a tall grey cabinet, a black drawer unit, a mini
fridge with a flower on top, and a floor lamp; a black curtain on the right. A grey rug covers most of the floor. A
wooden standing desk stands in the middle of the rug, lengthwise toward the back wall, with a chair pushed in on its
left side. A black trash bin stands on the floor at the foot of the wooden shelf, in front of the white panel.

The box is an open-top cardboard moving box, about 41 x 31 x 26 cm, about 1.2 kg, with thin plain walls (6 mm):
no handles and no hand holes. It starts on a low table (about 0.45 m high, its top just larger than the box) at the open
side of the room, in front of the desk's near end with about half a metre between the R1's back and the desk and
open floor on both sides, between the robots: the G1 stands at one short end facing it, the R1 at the other short
end facing it (and facing the G1). The rim of the box is about 0.71 m above the floor there.

## The task

The two robots lift the box off the low table and carry it together, each holding one end, by way of the far end of
the desk (between the desk and the fridge) to the floor beside the trash bin, and put it down there. The box must be
carried: off the floor and off the table, held by the robots, for the whole way. Pushing or sliding it along the
floor, or dragging it, does not count. How each robot holds its end is yours to decide (the rim of a wall, the side
walls, the bottom edge once the box is raised), and so is how the two keep in step.

Success: the box stands upright on the floor (tilted less than 20 degrees) with its centre within 0.35 m of the spot
on the floor just in front of the trash bin, both robots are standing, and neither the bin, the chair nor anything else
was knocked over. A green patch on the floor marks the spot.

## What is known, and what is not

- Nobody has done this before. Earlier sessions on this task left their experience in `memory/` and the skill
  library; finding out which parts are feasible and making them reliable is the result, as much as doing the whole
  task.
- On the low table the rim is within both robots' reach while they stand. The put-down is on the floor: to lower
  the box that far both robots have to bend their legs while holding it (the G1 with its own frames through `g1_step`,
  the R1 through the leg reference of `r1_step`).
- The team tools start a group of calls together but do not keep the robots in step during a motion, and neither
  robot follows the force on the box. If both carry it, walking together is the least certain part.
- Each robot's balance controller is a fixed learned policy; a pull through the box is a disturbance to it.
- Neither robot places itself precisely by walking: short walks overshoot or do nothing, and turns travel.

Use the session: when one way of doing a step fails, record what you measured and try another way; one failed
attempt is a data point, not the end. Spend the robot time where the task is lost: a stage that fails most of the
time is worth practising on its own, from its own start state, before more whole attempts. Stop early only when going
on would put a robot at risk.

Report where it stands: the funnel by stage (tries and successes), which robot held the box how, how far it got
toward the spot.

## The operator

The operator can put the box back at its starting place on the low table (`request_operator` kind `reset`), and, when
you ask for the robots too, carry both standing robots back to their starting places. For practice, a reset can put the
R1 at a stance you name instead of its start: write `r1_at=x,y,yaw_deg` (team frame) in the reset message; the stance
must be at least 0.33 m from the box centre and within 1 m of the R1's start. Resets are cheap here (up to 200 a
session). Do not count on other help: nobody holds the box for you. If the G1 falls, the operator starts its stack
again and hangs it on its gantry at its start (a minute or two; it says so in its next reply): protect the robots
anyway, a fall in a real lab costs far more.
