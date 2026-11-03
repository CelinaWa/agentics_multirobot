# PROGRESS — session 20261102T140446Z_puppy_box_e578ed (sim)

Layout: box directly BEHIND the puppy; puppy head RIGHT, legs to the robot, far side .33-.35, top +.017-.021 (depth), mid-body y ~-.125..-.135
(head hidden behind the tucked right hand in the waist-0 view). Box near wall x .38-.39 (colour floor/rim lines via code/e578/rimhit.py), y -.33..+.07.
Stance: edge .285 (arms tucked) -> walk .09 (odo .032, warm-up) -> crouch(0,.08) -> edge .23 -> walk .05 (odo .034) -> crouch -> edge ~.19.
Reach (commanded-row IK, code/e578/reachgrid.py): release x .50 z .24-.28 needs waist yaw cmd 0.7 (waist 0 reaches only x ~.43).
Waist cmd 0.7 measured .17-.25.

Attempts (right hand only, left tucked palm ~(0, .24, .15) throughout):
1. run_f0 grasp (fingers forward, palm stalled top+.027), base .66/.72, lift: palm jumped off command (IK branch flip, jump up to 1.6 allowed) -> z .40;
   grip lost on carry step 1. Puppy fell on the table. Reset 1.
2. code/e578/ctl.py (jump limit 0.5 on lift/carry, deeper press levels): hover cmd flipped branch (palm 14 cm off) -> synced to measured, ramped back to
   attempt-1 joints (jgo.py), approached via attempt-1 waypoints (goto.py). Press to palm .027, close: middle base .90; closed raise ok (.54);
   lift slipped at palm z .13. Puppy on the FLOOR. Reset 2 requested.
3. (reset 2) yaw +30 fingers, press gate top+0, diagonal lift: slipped at z .14 (table).
4. yaw +30 again, straight 2 cm lift: slipped at z .10 (table). Closed raise pulled the palm 3 cm back each time.
5. Approach C: yaw 30, palm 2 cm further forward, raise from the MEASURED palm: slipped at the first lift step. -> 3 approaches failed:
   asked the operator (help); answer: continue.
6. (reset 3) fingers forward yaw 0 (attempt-1 recipe) + press gate top+0 + measured-palm raise + straight 2 cm lift: HELD to palm .217;
   carry z .24 cmd to x .47 (palm .449), nudges (max jump .8) to cmd .505 (palm .465 = wall+.075), near paws then past the wall;
   opened -> operator: INSIDE the box. home_safe: arms tucked, waist 0, hands open, tilt 1.05.
RESULT: SUCCESS once (attempt 6); repeat test (7, reset 4) slipped at lift-off. Resets used 4 of 5. End: robot home, puppy on the table, box in place. Skill library merged (df756bc): plush-pick-place-box-behind v7.
Controller: code/e578/ctl.py (see its docstring); run order in handoff/README.md (session e578ed section).
Tools: code/e578/{tuck_measure,rimhit,reachck,reachgrid,goto,meas,armcmp,sync,jgo,openhand,raise,carry2,ctl}.py
