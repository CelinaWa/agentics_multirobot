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

The two robots lift the box off the low table and carry it together to the floor in front of the mini fridge, and put
it down there. A green patch on the floor marks the spot.

The box must be carried by both robots: whenever it is off the table and off the floor, at least one hand of each
robot is touching it. How the hands touch it is yours to decide. Pushing or sliding the box along the floor, or
dragging it, does not count.

Success: the box stands upright on the floor (tilted less than 20 degrees) with its centre within 0.35 m of the green
patch, both robots are standing, nothing else in the room was knocked over, and the box was carried as the rule above
says the whole way.

Nobody has done this before, and you start with no earlier experience of it.

Report where it stands: how far the box got toward the spot while carried by both robots, what each stage's tries and
successes were, and how each robot held the box.

## The operator

The operator can put the box back at its starting place on the low table (`request_operator` kind `reset`), and, when
you ask for the robots too, carry both standing robots back to their starting places and headings. For practice, a
reset can put a robot somewhere else instead: write `r1_at=x,y,yaw_deg` (team frame; at least 0.33 m from the box
centre and within 1 m of the R1's start) or `g1_at=x,y,yaw_deg` (within 0.2 m and 20 deg of the G1's start) in the
reset message. Resets are cheap here (up to 200 a session). Each reset reply begins with the measured state of the
scene and carries the ground truth of the attempt before it. Nobody holds the box or moves the furniture for you. If
a robot falls, the operator stands it up again at its start (a minute or two; its next reply says so): protect the
robots anyway, a fall in a real lab costs far more.
