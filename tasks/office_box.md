Two robots carry a box across an office: a Unitree G1 (Dex3 hands) and a Unitree R1 (PiPER grippers). Both robots
are in the same room with the box, so they can touch the same box and each other.

## The scene

A room about 3.2 m wide and 4 m deep. Looking in from the open side: a wall on the left with a wire shelf and a wooden
shelf; the back wall with, from the left corner, a white panel, a tall grey cabinet, a black drawer unit, a mini
fridge with a flower on top, and a floor lamp; a black curtain on the right. A grey rug covers most of the floor. A
wooden standing desk stands in the middle of the rug, lengthwise toward the back wall, with a chair pushed in on its
left side. A black trash bin stands on the floor at the foot of the wooden shelf, in front of the white panel.

The box is an open-top cardboard moving box, about 41 x 31 x 26 cm, about 1.2 kg, with thin walls: a robot can pinch
the rim of a wall. It starts on the floor near the open side of the room, on the right, between the robots: the G1
stands at one short end facing it, the R1 at the other short end facing it (and facing the G1).

## The task

Carry the box together, each robot holding one end by the rim (the R1 with its grippers, the G1 with its Dex3 hands),
from where it starts, past the far end of the desk (between the desk and the fridge), to the floor beside the trash
bin, and put it down there.

Success: the box stands upright on the floor (tilted less than 20 degrees) with its centre within 0.35 m of the spot
on the floor just in front of the trash bin, both robots are standing, and neither the bin, the chair nor anything else
was knocked over. A green patch on the floor marks the spot.

## What is known, and what is not

- Nobody has done this before. This session is the first exploration: finding out which parts are feasible is the
  result, as much as doing the whole task.
- Standing with straight legs, neither robot's hands reach the rim of a box on the floor (the rim is 0.26 m high).
  The G1 can crouch and lean (`g1_crouch`). The R1 has no crouch: its hands reach down to about 0.75 m in front of it.
  How the box gets from the floor into both robots' hands is yours to find; the G1 lifting it first to a height the
  R1 can reach is one candidate. If no way exists with these robots, establish that with evidence and say so.
- The team tools start a group of calls together but do not keep the robots in step during a motion, and neither
  robot follows the force on the box. Walking while both hold the box is the least certain part.
- Each robot's balance controller is a fixed learned policy; a pull through the box is a disturbance to it.

Work in this order and report where it stops: (1) each robot alone gets a hold on a rim of the box at a height it can
reach; (2) both hold the box at once and it stays level off the floor; (3) both move with it a short distance; (4) the
route and the put-down.

## The operator

The operator can put the box back at its starting place on the floor (`request_operator` kind `reset`); the robots are
not moved by a reset. Do not count on other help: nobody holds the box or moves a robot for you. A robot that falls
ends the session: protect the robots before the task.
