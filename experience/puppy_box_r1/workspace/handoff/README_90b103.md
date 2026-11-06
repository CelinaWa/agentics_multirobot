# Handoff - puppy_box_r1 on r1sonic (sim), session 20261106T080133Z_puppy_box_r1_90b103

Outcome: 7 placements confirmed INSIDE by the operator's ground truth (s7, s9, s11, s13, s14, s16, s17) in 17
grasp-stance runs (11 with the final recipe -> 7 inside); 5 of them fully unattended runs of the library skill
(look -> retreat). Final state: puppy inside, robot .42 m back, arms home, grippers open.
Reset walk-ins: 0/4 placements (W0 initial scene: no stance reached; R1, R2, R3: stance reached in 4-7 walk calls,
lift aborted 4/4 there).
Falls 0; one object incident (s12: puppy fell off the table after an abort at the top of the lift - fixed in v2).
Resets 3/10, stance placements 17/30. Older handoffs: README_2fbfb7.md (own chain, 3/21) and earlier.
Library: **r1-leg-lowtilt-lift-advance-carry v3 (verified, depth .03 default)** merged; uses r1-leg-axis-grasp-short-carry (perception).

## Run it (from the operator grasp stance)
    python code/puppy_box/run_skill.py <tag>          # skill main(): HOME, look, grasp, low-tilt lift, advance, carry,
                                                      # lower, release, retreat (walk back, HOME)  ~80 s
    request_operator(help): "Question only, please do not move anything: is the puppy inside the box?"
    (reset walk-in first: python code/puppy_box/home.py 0.6; python code/puppy_box/walkin3.py 0.38 0.35 6)

## What changed vs earlier sessions (and why it works)
1. Do NOT use r1_leg_lift for the lift: 4/5 aborted here (width +3-4 mm at lift step 2, then falling), with the +5 deg
   yaw offset AND with the engineering inputs (exact yaw, paw centre, d .02/.03). Its approach tilts up to .5 during
   the arc. Own servo-OFF lift with the approach nearly LEVEL (z component 0 -> .15): widths flat or slowly rising
   in 7/11 stance lifts (vs ~1/5 with the tool); depth .03 2/2, .02 5/9.
2. Level ADVANCE of 6 cm along the leg at the top of the lift (2 x 3 cm, servo off): the hand ends at x ~.475, so the
   carry is ~.16 m = 2 walk steps. Without it (.225 m, 3 steps) the grip dropped on step 3 and the puppy landed
   BRIDGED across the box top (s6). With it: carry 7/7, inside 7/7 (s14 dropped on step 2 past the wall: still in).
3. r1_carry_release works after an own lift (it does not require r1_leg_lift first): target = wall + .10 -
   (hand x + .06), lower_m .06, step_m .10.
4. Abort safely: running width peak from lift step 0 (the close width is a transient), abort if it falls > 4 mm in
   steps 0-5, RETRACE the lift path downward, open only low. A direct put-back from the top was refused and the
   gripper opened at z .22: the puppy fell off the table (s12).

## How to plan each subtask in a differently laid-out scene
1. Stance: the operator stance (paw (.32,-.20,.03), leg yaw 15-16 deg, edge .38, square) is where the lift works
   (6/9). Walk-in stances so far (paw x .34-.37, y -.15..-.20, 2.5-6 deg off square, leg yaw 8.5-22) gave 0/4 lifts:
   aim the walk-in at the operator stance geometry (paw x .31-.33, y -.20, square) and treat x > .34 or |edge yaw|
   > 3 deg as "not there yet". Straight forward walks drift the robot RIGHT (paw y +2..+4 cm per walk, but R2 also
   went -3.6 cm once); turns are arcs (+45 moved .22 fwd; -45 reported -6.8 deg and moved .36). The paw is invisible
   below x ~.33 (head .6): a short final walk (cmd .07 walked .055-.11) can lose it; walk back -.05 walked -.13/-.22.
   Unsolved: a precise last 3-5 cm. Idea: measure at x ~.37 then use the waist (r1_move_joints waist_yaw) or lean
   instead of walking; or let the arm reach the farther paw and keep the lift profile in the LEG frame.
2. Grasp: paw-ball sphere fit + leg axis (x-slices of the leg's right edge); grasp point 3 cm past the paw centre
   along the leg (v3; my paw fit reads ~1 cm further than the operator's .31; .03 widths ROSE in the lift 2/2), approach whose
   ACHIEVED yaw equals the leg yaw (choose_approach preview), fingers horizontal across the leg, close to 0.
   Width .034-.0365.
3. Lift: 6 servo-off steps (+1.2..0.5 cm along the leg, +3-4 cm up), approach z component <= .15, 2 s per step, to
   cmd z .25 (hand z ~.22), then the 6 cm level advance. Watch the width every step.
4. Carry/release: r1_carry_release ~.16 m in 2 steps of .10 (walked .094-.122 + .042-.07), width drops ~5-7 mm on
   walk 1, lowers .06, opens. Torso lands 5-8 cm short of the box centre (fine: interior 27 cm deep).
5. Retreat: v4.out (open, up, back, out beside the body, walk back .2, HOME). Never push a bridged puppy with the arm
   (s6: two pushes raised it onto the forearm).
6. Verify: only the operator's ground truth; questions must not contain "paw"/"overhang"/"position" (routed as an
   overhang request) and "do not move" in a REQUEST makes it a question.

## Files
memory/PROGRESS.md, memory/RUNS.md (90b103 table: W0, s1-s14, R1, R2), memory/TIPS.md (90b103 rows),
memory/INCIDENTS.md, memory/PARAMS.json; code/puppy_box/{run_skill.py, v8.py, v7.py, walkin3.py, walkin2.py,
pushin.py, overlay.py, scan.py}; skill code skill-library/skills/r1-leg-lowtilt-lift-advance-carry/code/run.py;
results output/skill_runs.jsonl; frames runtime/frames/<id>/ (ids in RUNS.md).
