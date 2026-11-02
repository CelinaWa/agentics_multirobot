# PROGRESS - session 20261102T045636Z_puppy_box_37fe31 (sim)
STATUS: task NOT completed - stance out of reach; robot in safe end pose (arms down beside hips, hands open, standing,
tilt 0.8), puppy on the table (diagonal, nose at the front edge), box untouched. 1 of 5 resets used.
Layout: box BEHIND the puppy (near wall 0.49-0.53 m, centre ~0.62-0.66, y -0.21..0.17), puppy far edge 0.42, head left.
Robot 0.32 m from the edge after the start easing; operator allowed 2 walks (0.12, 0.15, carry=true, arms down):
edge 0.28 after them (+4 cm then +0). Operator refused further walks / repositioning ("work from where you stand").
Reach (reach2.best on the commanded row): palm max x ~0.50 (z .24, lean .2, yaw .5) / ~0.53 (lean .15, yaw .7);
measured palm is ~4 cm short of the model at the limit. Release needs ~0.575-0.59. Rear-grasp/head-down drop also infeasible.
Incidents: (1) start easing dragged the puppy to the floor (no command of mine); (2) pb3 prep (sp .9 -> el -.9 -> sp .1,
wrist roll -1.3) bumped the puppy: it rotated ~40 deg. Fix: run prep only with arms clear of the puppy's footprint.
New code: code/puppy_box/arms_down.py (arms beside hips before a walk), code/puppy_box/reach_probe.py (empty-hand reach probe).
