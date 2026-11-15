Two robots, a Unitree G1 and a Unitree R1, are in a room with a box. The box is on a low table between them.

## The task

The two robots lift the box and carry it together to the floor in front of the mini fridge, and put it down there. A
green patch on the floor marks the spot.

The box must be carried by both robots: whenever it is off the table and off the floor, both hands of each robot are
touching it (four hands in all). How the hands touch it is yours to decide. Pushing or sliding the box along the floor, or
dragging it, does not count.

Success: the box stands upright on the floor on the green patch (its centre within 0.35 m of the patch's centre,
tilted less than 20 degrees), both robots are standing, nothing else in the room was knocked over, and the box was
carried as the rule above says the whole way.

Nobody has done this before, and you start with no earlier experience of it: the room, the box and what each robot
can do are yours to find out.

Report where it stands: how far the box got toward the spot while carried by both robots, each stage's tries and
successes, and how each robot held the box.

## The operator

The operator can put the box back at its starting place on the low table (`request_operator` kind `reset`), and, when
you ask for the robots too, carry both standing robots back to their starting places and headings. For practice, a
reset can put a robot somewhere else instead: write `r1_at=x,y,yaw_deg` or `g1_at=x,y,yaw_deg` (team frame) in the
reset message; the reply says if a place is refused. Resets are cheap here (up to 200 a session). Each reset reply
begins with the measured state of the scene and carries the ground truth of the attempt before it. Nobody holds the
box or moves the furniture for you. If a robot falls, the operator stands it up again at its start; protect the
robots anyway, a fall in a real lab costs far more.
