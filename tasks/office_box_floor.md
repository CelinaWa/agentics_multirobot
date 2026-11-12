Two robots carry a box across an office: a Unitree G1 (Dex3 hands) and a Unitree R1 (PiPER grippers). Both robots
are in the same room with the box, so they can touch the same box and each other.

## The scene

A room about 3.2 m wide and 4 m deep. Looking in from the open side: a wall on the left with a wire shelf and a wooden
shelf; the back wall with, from the left corner, a white panel, a tall grey cabinet, a black drawer unit, a mini
fridge with a flower on top, and a floor lamp; a black curtain on the right. A grey rug covers most of the floor. A
wooden standing desk stands in the middle of the rug, lengthwise toward the back wall, with a chair pushed in on its
left side. A black trash bin stands on the floor at the foot of the wooden shelf, in front of the white panel.

The box is an open-top cardboard moving box, about 41 x 31 x 26 cm, about 1.2 kg, with thin plain walls (6 mm):
no handles and no hand holes. It starts on the floor near the open side of the room, on the right, between the
robots: the G1 stands at one short end facing it, the R1 at the other short end facing it (and facing the G1), each
about 0.6 m from its centre.

## The task

Get the box from where it starts to the floor beside the trash bin, by way of the far end of the desk (between the
desk and the fridge), and leave it standing there. How is yours to decide: which robot does what, how the box is held
(the rim of a wall, the sides, from below), whether both robots carry it or one
carries and the other helps. Use what works and can be repeated.

Success: the box stands upright on the floor (tilted less than 20 degrees) with its centre within 0.35 m of the spot
on the floor just in front of the trash bin, both robots are standing, and neither the bin, the chair nor anything else
was knocked over. A green patch on the floor marks the spot.

## What is known, and what is not

- Nobody has done this before. This session is the first exploration: finding out which parts are feasible is the
  result, as much as doing the whole task.
- The box does not have to be carried: it may travel on the floor, pushed by a walking robot. In an earlier session
  the G1 pushed it upright about 0.3 m per straight walk (3 of 4), and fell once when it turned while in contact
  beside an obstacle on the floor.
- The team tools start a group of calls together but do not keep the robots in step during a motion, and neither
  robot follows the force on the box. If both carry it, walking together is the least certain part.
- Each robot's balance controller is a fixed learned policy; a pull through the box is a disturbance to it.
- Neither robot places itself precisely by walking: short walks overshoot or do nothing, and turns travel.

Use the session: when one way of doing a step fails, record what you measured and try another way; one failed
attempt is a data point, not the end. Stop early only when going on would put a robot at risk.

Report where it stops: which robot held the box how, how many times of how many, how far it got toward the spot.

## The operator

The operator can put the box back at its starting place on the floor (`request_operator` kind `reset`), and, when
you ask for the robots too, carry both standing robots back to their starting places. Do not count on other help:
nobody holds the box for you. A robot that falls ends the session: protect the robots before the task.
