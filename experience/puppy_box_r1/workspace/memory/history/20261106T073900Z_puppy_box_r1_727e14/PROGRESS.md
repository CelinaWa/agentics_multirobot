# PROGRESS - session 20261106T073900Z_puppy_box_r1_727e14 (sim), 2 h wall clock

## Current step
Start (~10 min): read handoff (2fbfb7: 3 placements/21 stance runs with own chain), TASK (NEW backend primitives
r1_leg_lift + r1_carry_release). Plan: v6.py = my perception (chain.py look: paw sphere fit, leg axis, edge, box rim)
-> r1_leg_lift(paw axis point, leg dir) -> r1_carry_release(target from the box front wall) -> v4 out -> operator
ground truth. Repeat on grasp stances, count per stage; >= 2 reset walk-ins (walkin.py v3 + v6).

## Run it
    python code/puppy_box/v6.py look <tag>; python code/puppy_box/v6.py lift [depth .03] [lift_z .265]
    python code/puppy_box/v6.py carry [in_m .08] [lower .06] [max_steps 5]; python code/puppy_box/v6.py out 0.2

## Budget
resets 0/10, stance placements 0/20, falls 0.
