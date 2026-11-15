Two robots, a Unitree G1 and a Unitree R1, are in an office. A can of Celsius drink stands on the office desk, and a
trash bin stands on the floor elsewhere in the room.

## The task

One robot takes the trash bin and carries it to the side of the other robot. The other robot takes the can from the
desk and puts or throws it into the bin. Which robot does which, and how each holds its object, is yours to decide.

The bin must be carried: lifted off the floor while it is moved. Pushing or dragging it along the floor does not
count. And the bin comes to the robot with the can: that robot does not go to where the bin stands.

Success: the can is inside the bin, the bin stands upright beside the robot that handled the can, both robots are
standing, nothing else in the room was knocked over, one robot handled the bin while the other handled the can, and
the bin was carried there as the rule above says.

Nobody has done this before, and you start with no earlier experience of it: the room, the can, the bin and what each
robot can do are yours to find out.

Report where it stands: each stage's tries and successes, which robot did what, and how close the can came to being in
the bin.

## The operator

The operator can put the can back on the desk and the bin back at its starting place (`request_operator` kind
`reset`), and, when you ask for the robots too, carry both standing robots back to their starting places and
headings. For practice, a reset can put a robot somewhere else instead: write `r1_at=x,y,yaw_deg` or
`g1_at=x,y,yaw_deg` (team frame) in the reset message; the reply says if a place is refused. Resets are cheap here (up
to 200 a session). Each reset reply begins with the measured state of the scene and carries the ground truth of the
attempt before it. Nobody holds or moves anything else for you. If a robot falls, the operator stands it up again at
its start; protect the robots anyway, a fall in a real lab costs far more.
