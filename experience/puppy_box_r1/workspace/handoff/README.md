# Handoff - puppy_box_r1 on r1sonic (sim), session 20261106T060138Z_puppy_box_r1_2fbfb7

Outcome: 3 placements confirmed INSIDE by the operator's ground truth (st9, st15, st18) in 21 grasp-stance runs + 4
reset walk-ins (0 placements); falls 0; resets 4/10, stance placements 21/25. Every success ended with the robot backed off
(.34-.41 m from the edge), arms HOME, grippers open, box untouched. Older handoffs: README_cf6e88.md, README_7c03aa.md...
Library: r1-leg-axis-grasp-short-carry v3/v4 (draft) merged (see skill-library/INDEX.md).

## Run it (from the grasp stance)
    python code/puppy_box/home.py 0.6         # ALWAYS after an operator placement (stale arm command)
    bash code/puppy_box/attempt.sh <tag>      # look -> pre -> v4.py go 0.045 -0.004 0.06 4 0.06 1 0 1 (re-grasps on exit 4)
    python code/puppy_box/v4.py out 0.2       # open, up, out (.14,-.40,.14), walk back (repeats if short), HOME both
    then request_operator(help): "Question only, please do not move anything: is the puppy inside the box?"

## How to plan each subtask in a differently laid-out scene
1. Stance: stand square to the table with the target paw (clean overhanging paw, no neighbour within 9 cm) at pelvis
   (.31-.34, -.19..-.22), front edge .37-.39 ahead. Head .6 sees table-height paws only at x >= .33. The reset start
   (edge .62) needs ~.25 of walking AND ~5 cm of lateral shift; short walks overshoot 1.5-2.7x (.07 -> .106-.121,
   -.05 -> -.133) and drift 2-7 cm; turns of +-8..10 deg rotate 5-12 deg and shift the scene ~4.4 cm in y, but the
   next walk partly undoes the heading. WORKING (R3, 1/1): `python code/puppy_box/walkin.py 0.33 6 0.37` - track the
   start-view target paw (.557,-.25) and walk cmd = .8 x remaining (>= .08, <= .20), STOP as soon as the paw is at
   x <= .37 (2 calls: .134 + .063 -> (.356,-.175)); the forward walks themselves drift the scene ~+5 cm in y, so no
   turns are needed. Then `OVERHANG_MIN=0.02 bash code/puppy_box/attempt.sh` (relaxed overhang selection; the
   paw-blob dedupe in chain.select is needed there). That stance is ~5 cm further out than the operator stance: the
   carry needs ~.05 more (R3 released 8 cm short of the wall). R4: paw at .372 (2 mm above the stop) -> one more
   .08 walk went .104 -> paw lost below the view; the step back drifted 9.5 cm right. So walk-in is 1/2 with v3;
   next: widen the stop to x <= .40 and make the grasp reach x .33-.40 (preview IK; use_waist) instead of walking.
   R1/R2 (turns + fine steps) failed.
2. Grasp: leg axis from x-slices (chain.py look); grasp point 4.5 cm past the paw-ball centre along the axis
   (the pads then cover part of the ball), horizontal fingers across the leg, close to 0. The ball acts as a STOPPER
   when the leg is pulled: the width RISES during the lift (.038 -> .047-.054) and the grip survives walking (st9, st14,
   st15, st18). If the width FALLS during the lift (.034 -> .026), the leg is sliding out: v4 aborts in the first 4
   steps (exit 4: width < running peak - .004 in steps 0-5) - no drops in 3 aborts, but re-grasping the put-back
   paw failed 0/4 (no clean overhang any more): ask the operator for the overhang before re-grasping. The TASK's "deep shaft grasp .031-.035" slipped on walk 1 4/4 here.
3. Lift: chain.py LIFT profile (11 servo-off steps of +3.5-4.5 cm, tilt to .5, approach yaw rotated to 2 deg so the
   body hangs AHEAD of the hand); refused steps retried as half steps; ends at hand z .39-.42. Do NOT look, raise or
   move the waist afterwards: walk at once (pauses let the grip creep, raises slipped).
4. Carry: r1_walk cmd .07 repeatedly (walks .055-.107 each) until the body centroid (~hand x + .06) is .06 past the
   box front wall (front wall = edge + .22); 2 walks usually. Hand drifts 2-4 cm right per walk.
5. Release: LOWER the hand .06 (retry .03 if refused) so the body sits in the box, then open. Releasing high left the
   puppy upright on the rim (st14) or swung it to the floor (st17).
6. Retreat: v4.py out. Never HOME before the walk back is confirmed (a short walk back + HOME drove the hand onto the
   table edge, tilt 6.65).
7. Verify: ONLY the operator's ground truth. Depth and images both showed a puppy lying in FRONT of the clear box as
   if bridged on it (st2, st6).

## Files
memory/PROGRESS.md (story + recipe), memory/RUNS.md (per-attempt table + funnel), memory/TIPS.md (tagged rows),
memory/INCIDENTS.md, memory/PARAMS.json; code/puppy_box/{v4.py, attempt.sh, walkin.py, inbox.py, chain.py, home.py};
frames runtime/frames/<id>/ (ids in RUNS.md).
