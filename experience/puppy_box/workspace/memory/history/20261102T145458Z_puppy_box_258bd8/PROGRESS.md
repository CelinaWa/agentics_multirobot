# PROGRESS - session 20261102T145458Z_puppy_box_258bd8 (sim, 1 h wall clock)
Layout (first frame): box ahead-LEFT (table-aligned, clear), puppy RIGHT, head LEFT (snout toward box right wall), legs to robot.
= like 0878bd (head left) / 2a8660 layouts. Plan: library cross-right-hand v4 run_x2a.py (code/puppy_box/run_x2a.py).
Steps: stance,look,plan -> LOOK cr_plan.jpg -> prep,waist,over,hover,grasp,lift -> LOOK -> carry,swing(/adjust) -> LOOK -> release,home

## Status 15:15 (~20 min used)
Stance: 3 stance walks + 1 walk 0.10 (odometry -0.006) -> no advance; edge (arms tucked, after crouch fwd .08) 0.27 at y 0, 0.22 at y -0.2 (table rotated ~12 deg to the robot). table z level -0.0375.
Box (colour corners, code/p258/corners.py): near-right (.31,-.114) near-left (.396,.293) far-left (.682,.218) far-right (.604,-.177); BOX=0.33,0.62,-0.11,0.28 passed.
Puppy (start/reset): head left y -.17..-.29 (x .30-.39), front legs y -.32..-.36, mid-body y -.40 x .26-.37, top +.03 (depth), body to y -.52.
Fixes: percep.table_z TABLE_Z env override (histogram locked on the tucked left arm -> 0.11); run_x2a WAIST / GC env overrides.
Attempts (all right hand, left tucked, no falls, box untouched):
 1. waist .7/.15, PCA centre (front legs) y -.338, yaw -30: palm stalled z .088 (5 cm above top), empty fist; pushed puppy 6 cm to the edge -> reset 1.
 2. waist .5/.25 (offline IK: roll is what reaches y -.40), mid-body (.29,-.39) yaw -30: stall .071, CLOSED .80/.90 base -> lift held to palm z .12, slipped at .15-.18; puppy flipped on its back, diagonal.
 3. belly of flipped puppy, yaw -55 (-69 no IK at z .30): one finger (middle .95, index fist) -> lost at first lift step.
 -> rule: 3 approaches failed -> request_operator.

## Status 15:23 (~33 min used) - STOPPED attempts, documenting
 4. reset 2; as 2 with SINK .03 + lift 2 cm/4 s: one finger -> lost. Puppy pushed away -> reset 3.
 5. as 2 + lean .1: one finger at the stall; in-place deeper re-press -> empty fist. Robot home, arms tucked, hands open; puppy on table; box untouched.
Resets used 3/5. Conclusion: the grasp at ~0.40 m lateral is reach-limited (palm 4-5 cm above command). Next idea (untested): ask the
operator early to place the robot ~10-15 cm further right (or the puppy left) before any press; or a stance squared to the table.
Remaining: library v5 note + merge, handoff section.
## Final (15:27): library merged (main 1ecd6d9, cross-right-hand v5), handoff section added, PARAMS recorded. Robot standing still, arms tucked, hands open, tilt 0.6.
