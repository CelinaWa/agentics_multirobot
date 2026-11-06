# Handoff - puppy_box_r1 on r1sonic (sim), session 20261106T043952Z_puppy_box_r1_cf6e88

Outcome: NO placement this session (0/11 attempts: 9 operator grasp stances + 2 full walk-in chains, resets 3/10 used,
stance placements 12/25, falls 0). Grasp + lift is reliable (lift to z >= .40: 9/11); the CARRY of the hanging puppy
to the box is the unsolved stage. Older handoffs: README_7c03aa.md (first assisted placement), README_cdc45b.md, ...
Library: r1-leg-axis-grasp-short-carry v2 (draft) merged -> main 877b319. Controller: code/puppy_box/chain.py +
code/puppy_box/carry.py (step tools), code/puppy_box/home.py (stale-command HOME after every operator placement).

## What the sim does now (verify first)
- Reset / stance layout unchanged: puppy head right, box BEHIND it (front wall = edge + .22, rim = table + .145),
  front-right paw overhanging ~6 cm, leg yaw 12-24 deg. Reset start: edge .62 ahead.
- After every operator placement the backend may hold a stale LEFT/RIGHT arm command: run code/puppy_box/home.py first.
- Reset wording: "Please reset the scene and the robot to the start pose." (no "overhang" in it, or it is mis-routed).

## How to plan each subtask in any layout
1. Walk-in: from the reset start, walk .2 (walks .12-.14), re-measure, then cmd >= .08 steps (smaller commands are
   random, even backward). Stop with the target paw at pelvis x .31-.35, y -.19..-.22, visible at head .6. If the paw
   drops below the view, step back (-.06) rather than guessing: box-rim y re-localization fails with the puppy in front.
   Lateral drift 4-8 cm per walk: correct with a small turn (cmd -10 -> ~10 deg) while still >= .4 m from the edge.
2. Perceive: chain.py look (paw sphere fits, front edge, leg axis by x-slices with outlier rejection, box pose).
3. Grasp: chain.py pre; chain.py go 0 .06 -.004 1 (lift_only): closes, servo-off arc lift, exits HOLDING with the
   last commanded pose stored. Clean = .030-.034 after the lift; .048-.056 = diagonal (slips on walks).
4. LOOK at the hanging body before carrying: carry.py look .15-.3 (head up). Expect it 5-10 cm in front of the box
   front wall, bottom ~at rim height.
5. Carry (OPEN PROBLEM): one walk of cmd .07 holds a clean grip (2/2) but the body ends beside the right wall or low
   against the front wall. Ideas not yet tried: (a) slow waist yaw +.15..+.2 BEFORE the walk to centre the body over the
   box's y range and keep it moving with the torso (waist yaw while holding is safe 2/2); (b) re-command the hand UP
   after each walk with a RELATIVE move (carry.py move 0 0 .03) to undo the 4-5 cm sag; (c) a grasp stance where the
   box is closer (a different layout: ask for "box beside the puppy on my RIGHT"? - the left variant puts the box out
   of the right hand's reach); (d) a two-stage place: put the puppy ON the box front rim, then re-grasp a leg that
   overhangs the box and lift it in.
6. Release: carry.py release; judge by DEPTH (x beyond the front wall), not the image (the clear wall misleads).
7. Retreat: carry.py out <back> (open, up, out to the right, HOME, head .6, walk back). Never move forward or HOME
   from below the table level near the edge.

## Files
memory/PROGRESS.md (story), RUNS.md (table per attempt), TIPS.md (tagged rows), PARAMS.json (sim values);
frames under runtime/frames/<id>/cam_head.jpg (ids in RUNS.md / PROGRESS.md).
