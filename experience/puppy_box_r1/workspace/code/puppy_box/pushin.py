"""Push-in finish for a puppy balanced on the box front rim with its legs up toward the robot (s6, 90b103).
python code/puppy_box/pushin.py <y> <z> <x_end>  closed gripper, approach yawed LEFT (0.8,0.6,0.15) (no waist: the
level approach (1,0,0) at y > -.10, z .20 is out of reach, ik .05-.15): back/up (.28,y-.07,.26) -> down (.28,y,z)
-> slow forward (x_end,y+.02,z) 3 s -> back (.24,y-.07,z+.08)."""
import sys; sys.path.insert(0, '.'); sys.path.insert(0, 'code/puppy_box')
from chain import guarded, ctx, log
y, z, xe = (float(v) for v in sys.argv[1:4])
ctx.value('r1_gripper', arm='right', width_m=0.0, secs=0.5)
A, F = (0.8, 0.6, 0.15), (-0.6, 0.8, 0.0)
for p, secs in (([0.30, y - 0.07, 0.27], 2.0), ([0.28, y, z], 1.5), ([xe, y + 0.02, z], 3.0)):
    r = guarded(p, A, F, w=0.0, secs=secs, max_jump=1.6)
    if r is None:
        log(step='STOP', why='refused', goal=p); sys.exit(2)
r = guarded([0.26, y - 0.07, z + 0.08], A, F, w=0.0, secs=2.0, max_jump=1.6)
o = ctx.value('r1_observe'); log(step='pushed', frame=o['camera']['frame_id'], tilt=o['tilt_deg'])
