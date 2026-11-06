# Handoff - puppy_box_r1 on r1sonic (sim), session 20261106T033337Z_puppy_box_r1_7c03aa

Outcome: FIRST PLACEMENT (assisted). Reset 1 (whole chain, walked in): leg-axis grasp -> lift -> 2 short walks ->
the puppy dropped onto the box FRONT RIM -> two forward pushes with the closed gripper tipped it INSIDE (upright,
head/front paws over the rim). Final frame runtime/frames/20261105203524734959. Stance practice: lift 12/12, 2 walks
held 6/12, 3rd walk 0/3, clean placement by release 0. Resets 1/10, stance placements 14/20, falls 0.
Older handoffs: README_cdc45b.md (pivot pinch, servo-off lift), README_f6b331.md, README_c4f6c3.md, e0ac0c, 08803f.
Library: r1-leg-axis-grasp-short-carry (draft, merged main 41105ce). Controller: code/puppy_box/chain.py.

## What the sim does now (verify first)
- Operator notices: the paw ball is SOFT; jaws clamp the leg core (clean = width .031-.035 along the leg axis;
  .040-.058 = diagonal/two legs). Carry only in .05-.08 walk steps.
- After a reset/stance placement the backend can hold a STALE arm command: send HOME joints + gripper open first.
- Layout (reset and stance): puppy head right, box behind (front wall edge+.22..+.23, rim table+.145), front-right
  paw overhanging 7.4-8.1 cm, leg yaw 19.5-24 deg vs the table normal, level. A variant layout (stance 3/12) is
  shifted ~.2 m right: the code stops there.

## How to plan each subtask in any layout
1. Stance: paw at pelvis x .31-.33, y -.19..-.21 (edge ~.39). Walk in with .05-.1 steps near the end (walks vary
   .02-.13; backward overshoots 2x). Head .6 cannot see table-height points at x < .33: if the paw hides, use a
   reference paw + the offset measured one frame earlier.
2. Perceive: sphere-fit paws; leg axis from x-slices (right edge of the leg run + 1.6 cm) - never assume the yaw.
3. Grasp: choose the commanded approach by preview so the ACHIEVED yaw equals the leg yaw (cmd +2..4, pitch -15);
   waypoint 12 cm behind at z .12, pregrasp 10 cm short, re-localize via the reference paw, advance to 5-6 cm past
   the paw along the axis at paw z - .004, close to 0. Chain grasp->lift->carry in ONE script (creep while idle).
4. Lift: servo OFF, 11 arc steps to z ~.40, rotate the approach yaw to ~2 deg (moves the hanging body ahead of the
   hand). End-of-lift width < .028 => expect a drop.
5. Carry: the box is ~.22 m too far for the arm; the grip survives only ~2 short walks (~.1-.15 m). Release right
   after them (no reach: x <= .46 at z .40). The puppy then lands on the front rim or in front of the box.
6. Finish: if it lies on the front rim, push it in: closed gripper, approach (1,0,0), fingers (0,0,1), waypoint
   (.20,-.28,.16) behind the edge, (.36,-.20,.16), slowly to (.52,-.20,.16) at z rim+.04 (walk ~.05 closer if the
   rim is beyond reach). Check the image: body behind the clear wall down to the floor = inside.
7. Retreat: hand straight back, open, out (.14,-.40,.08), walk back .1-.2, HOME. Never HOME / move forward from
   below table level near the edge (2 table-edge hits).
Open problem: a carry that does not depend on luck - e.g. set the puppy down near the box after 2 walks and re-grasp,
or design the release so it always lands on the rim and then push (worked 1/1).

## Files
code/puppy_box/chain.py (look | pre | go 0 <past> <dz> | retreat <back>; state output/chain_state.json),
legaxis.py, hold.py; memory/PROGRESS.md, RUNS.md, TIPS.md (tagged), INCIDENTS.md, PARAMS.json.
