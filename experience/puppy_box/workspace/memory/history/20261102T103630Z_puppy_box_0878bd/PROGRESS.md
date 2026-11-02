# PROGRESS - session 20261102T103630Z_puppy_box_0878bd (sim)

## Layout (this session)
Box LEFT/ahead (rim corners pelvis after the walk: (0.248,-0.115) (0.539,-0.172) (0.618,0.226) (0.327,0.283), table-aligned,
robot heading ~10 deg left of the table). Puppy RIGHT, head LEFT (snout toward the box's right wall), legs to the robot,
body y -0.20 (nose) .. ~-0.56, x 0.18-0.32, depth top +0.03 pelvis (real top ~+0.065: palm stalled there). Table z -0.035.

## Status: SOLVED run 1 (attempt 2), operator confirmed INSIDE (10:58). 0 resets used.
- stance: same-side skill `run.py left stance` -> edge 0.303 -> 0.21 (one walk).
- pb3 prep; perceive by pb3/cr perceive FAILED (arm hides view, boxdet on the wrong area) -> own look: cr.py look (both arms
  tucked, waist cmd -0.7, meas -0.23) + pup.py slices + layout2 rim corners (sim depth sees the rim plane).
- Attempt 1: grasp (0.255,-0.385) yaw 15, waist (0.45,0.25): pb3 grasp press -> tilt 7.56 (hold, raised). grasp_soft stopped
  on tilt 6.8 at palm z 0.06, only middle finger engaged (0.6/1.53) -> lift ok, lost on the first lateral carry step; puppy fell
  back on the table, diagonal now (head at (0.33,-0.25)).
- Attempt 2 (SUCCESS): waist (0.45, 0.15) (tilt 1.5 instead of 2.6), grasp (0.25,-0.35) behind the neck, fingers yaw -36
  (perpendicular to the diagonal body), cr over -> lr hover -> cr grasp_soft (palm stalled 0.067, fingers 0.92/0.84) ->
  lift (fingers crept to 1.36/1.30 = still held) -> cr carry 6 cm/5 s yaw -36->15, STALL 1.45 -> palm (0.414,0.042,0.317)
  -> open -> up -> home_cr.py (path-checked prep then tuck). Head draped over the right rim, body inside.
## Run 2 (reset 1 used): FAILED safely. run_cr look,plan (box clipped -> BOX override), chain prep..grasp: 3 closes
(yaw -12 at (.268,-.335); yaw -30 + 2 cm press at (.248,-.34); yaw -10 at (.215,-.33)) all empty fists, tilt up to 7.0; plush
pushed toward the table edge (still on the table). Stopped per the rule (3 approaches), home_cr -> standing tucked, tilt 1.6.
## Done: controller code/puppy_box/run_cr.py (+cr.py, pup.py, home_cr.py, plan_cr.py); skill library cross-right-hand v3
merged (main 4237dc3); handoff section written. Resets used 1/5.
## Next (if more time): operator places the robot ~10 cm right; grasp yaw -36..-45 at the measured mid-body.

## Run 3 (reset 2): FAILED safely (2 more empty fists, yaw -40). Tally 1 success / 3 runs, 1 / 6 grasp closes. Resets used 2/5.
Stopped grasp attempts (rule: >3 approaches failed), informed the operator. Robot standing, arms tucked, hands open, tilt 1.55,
box untouched, puppy on the table. Skill library re-merged with run 3 evidence.
