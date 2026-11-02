# PROGRESS (session 20261102T062330Z_puppy_box_89afad, sim, 3600 s)
## State (06:59): runs 1 AND 2 SUCCESS (2/2, operator confirmed); robot home, hands open, tilt 1.2. Skill merged (library main 4c7c35c).
## Run 1 (06:52): SUCCESS (operator: "dog INSIDE the box"); robot home, hands open, tilt 1.
Layout: puppy RIGHT (head right, legs to robot, y -0.37..-0.09, far edge 0.42, top +0.042 pelvis), clear box LEFT
(x 0.28-0.585, y 0.025-0.43 from colour corners via percep.plane_hit; boxdet was 0.2 m off in y again). Table z -0.057.
Start 0.33 m from edge (operator); walks 0.13 (odo .094) + 0.07 (odo .042), arms tucked, carry=true.
## What worked (run 1, right hand only, left tucked sp .45 el -.5 -> measured sp 0.9-1.3 i.e. further back)
1. pb3 prep, pb3 perceive right (box from it is WRONG: use lr.py box with plane_hit corners).
2. Fixed waist yaw cmd 1.1 + roll 0.15 (meas yaw .5-.6, roll .13-.17), own 4 s ramp.
3. Arm to above the grasp via a raised via-config (IK branch sp .68 / yaw -1.48 at waist 1.1); grasp cmd (0.365,-0.23)
   = head-side of mid-body, +1.5 cm x for the ~4 cm short landing. pb3 grasp: stall palm z .053, fingers ~1.0. pb3 lift ok.
4. pb3 carry with RELEASE_YAW=15 to cmd (0.38,0.20,0.27): measured palm (0.394,0.129,0.284). Depth: body centred y 0.01
   (on the right wall!) -> rotate palm yaw 15 -> 50 -> 75 (CCW): body swung forward-left over the interior (all y>0.12).
5. open, raise straight up at yaw 75, pb3 home. In the box.
## Failures / lessons
- Empty-hand probe: waist cmd .9 -> palm y .069 (cmd .155); cmd 1.1 -> y .09 (cmd .20). Carrying the puppy got .129.
- reach2.best random seeds returned an overhead branch (sp -2.97) -> arm went overhead; recovered by staged ramps
  with waist->0. ALWAYS check jump and FK path min-z before ramping an IK result.

## Run 2: chained over..swing via skill run.py; waist 0.9 (1.1 gave tilt 6.5). Swing depth gate under-counts (occlusion): look instead.
## Next: try run.py unattended end-to-end on a different layout (head left / box elsewhere); improve the swing gate (use colour mask below the palm).
