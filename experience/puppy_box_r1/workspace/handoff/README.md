# Handoff - puppy_box_r1 on r1sonic (sim), session 20261106T112411Z_puppy_box_r1_3d54de

Outcome: reset WALK-INS: 2 of 10 inside (R6, R8), both with the new chain (walk-in v5 + chain lift), i.e. 2 of 5
walk-ins with the final chain (R6-R10); the first five resets (R1-R5, older lifts) gave 0 inside (R3 on the rim).
Stance placements: s1 (r1_leg_lift chain) and s14 (cl.py v6) inside, s4 rim, the rest mostly lift aborts (see
memory/RUNS.md). Falls 0, puppy-off-table 0; one stuck retreat (s14) recovered (hand up, HOME, walk back) and that
recovery is now in the code. Resets 10/10 used. THE WEAK STAGE IS THE LIFT (chain lift 8/26, r1_leg_lift 2/7). Stance placements used 19/20.
LIFT BY SCENE SOURCE: reset walk-in scenes lifted 3/4, operator stance placements 4/19 (no separation by leg yaw,
paw x or close width) - do not tune the lift on stance placements alone. The regrasp-deeper rule is OFF (s18).
Library: NEW **r1-walkin-leg-chain-lift-box** v2 (draft, merged 83707ce); r1-leg-lowtilt-lift-advance-carry v4.2
(waist-look fixes). Older handoffs:
README_5600c1.md and earlier.

## Run it
    bash code/puppy_box/full.sh <tag>        # from the reset start (~.6 m back): home -> walkin5 -> cl.py -> one retry
    python code/puppy_box/cl.py <tag>        # from a stance (paw x .30-.37, y -.30..-.12): look -> grasp -> chain lift
                                             #   -> top gate -> r1_carry_release -> retreat
    python code/puppy_box/stance_batch.py <n0> <n>   # sim: n operator stance placements, ground truth logged
    # then (sim) request_operator(help): "Question only, please do not move anything: is the puppy inside the box?"
Library copy: skill-library/skills/r1-walkin-leg-chain-lift-box/code/{run.py, walkin.py, chainlift.py}.

## The three lessons of this session
1. LIFT: the chain.py profile (approach yaw rotated from the leg yaw to 2 deg over the lift, approach z 0 -> .5,
   z up to ~+.43, 1.2 s/step, grasp 4.5 cm past my paw fit) completes more lifts than r1_leg_lift (2/7 here) or the
   low-tilt lift. Signature of a good lift: the width dips 2-4 mm at steps 1-4, then RISES 6-10 mm (.045-.053 at the
   top) = the paw ball is held as a stopper. A width that keeps falling to < .031 drops in the carry (s6) -> put back.
   Still ~1/3 aborts at steps 2-3 (s5, s7, s8, R8 first try): a retry in place works when the paw still overhangs.
2. CARRY: r1_carry_release COUNTS advance_m in target_forward_m. The torso lands at hand x + .02..+.05 (operator
   ground truth: R8, R10, s4). Total target = wall + .09 - (hand x + .035); the old edge bound (pelvis >= .22 from the
   edge) cut R10's walk to .147 -> torso on the front rim. R8 walked .216 from edge .363 (pelvis .147 from the edge)
   without contact -> bound .16. Lower .12 before opening (release from hand z ~.19-.25).
3. STANCE: walks drift 0-5 cm sideways EITHER way and rotate 0-3 deg; each waist look shifts the pelvis 2-5 cm more
   (paw y more negative). Far paws (x > .37) are an IK dead end for the chain lift's first step; close paws (< .30)
   give lateral grasp misses at the cramped reach. Walk-in v5: fewest calls, cmd .85*(x-.35), stop at x <= .375,
   one corrective turn at x >= .40 if |y + .215| > .045.

## How to plan each subtask in a differently laid-out scene
1. Walk-in: perceive the clean overhanging paw (n >= 300) from the start; straight walks (.07-.20 commands) re-selecting
   the paw each time; stop while it is still visible (x <= .375) or when it drops below the view (then waist look).
   Lateral: correct once with a small turn while >= .40 away; never step back near the table.
2. Look: paw sphere fit + leg axis (x-slices of the leg's right edge) + edge + box rim. Paw hidden -> waist yaw -.35,
   select in the waist-0 frame through shared paws (lowtilt v4.2). Keep the waist-0 edge (post-look edge fits jump).
3. Grasp: horizontal fingers across the leg, approach yaw = leg yaw (chosen by preview), pregrasp 8 cm short,
   G = paw + .045 along the leg (<= .405 - paw x), close to 0 -> .032-.047.
4. Lift: chain profile (above), servo OFF, never re-solve IK while holding without an approach (branch flip = drop).
   Abort (retrace down, open low) if the width falls > 6 mm below its peak in steps 0-5; top gate .031.
5. Carry + release: r1_carry_release(total target as above, lower .12). With the box elsewhere (beside), the carry
   becomes a waist yaw / lateral problem - not tested here.
6. Retreat: v4.out (open, up, back, out (.14,-.40,.14), walk back .2, HOME). Final state checked: robot .37-.44 m
   behind the edge, arms home, grippers open, tilt < 2.
7. Verify with the operator; the clear box fools images and depth.

## Not solved / next
- [param] where: sim. Lift aborts (~60%) at steps 2-5: the body leaving the table pulls the leg through the pads (width +2-4 mm, then
  -6 mm). Close width only weakly predicts it (>= .039 lifted 6/13, < .039 0/13); regrasping deeper to widen it lifted 1/5.
  Ideas not tried: lift slower only through z .08-.20 (secs 2-3 there); a grasp perpendicular to the leg (paw ball
  as a geometric stopper on the pad edge) - needs a reachable pose; vertical fingers were tried before (5600c1).
- Lateral stance control: drift and waist looks; consider looking with the head only (no waist) by stopping at
  x .34-.37 and accepting slightly far stances with the reduced grasp depth + waist-assisted first lift steps
  (added after R7, untested on a far stance).
- The new carry target (BODY_OFF .035, EDGE_MIN .16) has few runs (see RUNS.md batch rows s8-s11).

## Files
memory/PROGRESS.md, memory/RUNS.md (3d54de section), memory/TIPS.md (3d54de rows), memory/INCIDENTS.md;
code/puppy_box/{full.sh, walkin5.py, cl.py, bl.py, paws.py, stance_batch.py, lowtilt_v4.py (v4.2 look fixes),
ctl.py}; logs output/logs/<tag>.log; batch output/stance_batch.jsonl; frames runtime/frames/<id>/.
