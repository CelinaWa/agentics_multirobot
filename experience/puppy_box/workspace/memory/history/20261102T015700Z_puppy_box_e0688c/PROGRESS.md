# PROGRESS (session 20261102T015700Z_puppy_box_e0688c, sim)

## Layout (this session)
- Box LEFT: rim (ray/plane on colour corners) x 0.26-0.56, y 0.05-0.44, rim z +0.09 pelvis; table top z -0.05 (level).
- Puppy RIGHT, head right, tail toward the box, legs toward robot: x 0.16-0.32, y -0.10..-0.29(+ out of view), top 0.064.
- Table edge 0.18-0.20 after the reset (0.25-0.29 right after start).

## Attempt 1: SUCCESS by hand (02:00-02:07)
0. Start: the puppy fell to the floor during controller start/settle (no motion sent by me). Reset 1 (of 5).
1. Tuck left (sp .45 el -.5), unfold right (sp .5 -> sp 0 el -.3), right shoulder roll -0.6 to see.
2. plush-pinch-grasp grasp(right, xy=(0.25,-0.26)) [palm x = far edge 0.318-0.07, y = mid-body]:
   no palm stall, descended to z 0.04 cmd 0.01; close -> index 0.90 middle 1.07 = held. bias (0.0136,0.012,-0.0096).
3. lift steps (0.25,-0.26,.08)->(.12)->(.25,-.25,.16)->(.26,-.23,.20)->(.28,-.18,.24)->(.31,-.12,.27): grip held (index 0.5-0.6).
4. Left arm further back (sp .8 sr .2 el -.3) to clear the carry path.
5. Waist yaw cmd 0.5 (meas 0.285) then cmd 0.8 (meas 0.41, creeps to 0.47). Palm (0.33,0.14,0.24).
6. Waist pitch 0.15 (meas 0.14), tilt 5 deg. Palm (0.35,0.165,0.20).
7. Direct joint ramp 4 s to IK of (0.39,0.18,0.22) yaw 30 (pps refused: joint jump vs command >1.2):
   palm -> (0.332,0.247,0.20); index creeping to 1.10. Opened (hands 0) at once: puppy dropped into the box.
8. Return: waist pitch 0 + R sp -1.7 (hand up to z .35); R el .3 wrists 0 sy 0; R sp -.8 sr -.25; waist 0 (4 s);
   R sp .7 el -.3 sr -.2; both sp .5 el .6 (L sr .2). Final frame 20261102020722fcd313: puppy in box (tail over
   near rim), box upright, hands open behind the edge, tilt 1-5 deg.

## Controller (v1) code/puppy_box/controller.py — steps prep/perceive/grasp/lift/carry/release/home/verify
Box detector code/puppy_box/boxdet.py (pale low-saturation region; sim depth DOES see the box top at rim height,
so the mask accepts table or rim height). Next: reset 2 and run the controller step by step.

## Status at 02:22 (session ~25 min used of 60)
- Runs: 1 by hand SUCCESS, 2 controller+3 manual fixes SUCCESS, 3a stop (out of reach), 3 controller v2 SUCCESS
  (no motion corrections), 4 unattended run.py stopped at perceive (puppy out of reach after reset 5).
- All 5 resets used. Final scene: puppy on the table at x 0.46-0.69 (operator's last reset), OUT OF REACH; robot home,
  hands open, behind the edge. Task end state therefore NOT satisfied at session end (it was after runs 1-3).
- Skill library: plush-pick-place-box-one-hand v1 (draft) merged (main ab078f3); plush-pinch-grasp evidence added.
- Controller: code/puppy_box/controller.py (copy in the skill). handoff/README.md updated.
