# Handoff — puppy_box_r1 on r1sonic (sim), session 20261105T194732Z_puppy_box_r1_1d45ef

Outcome: NOT solved (0 placements in 3 layouts). Robot ended safe (arms home, grippers open, standing).
Copied from /tmp/claude-1001/ws (the agent workspace was mounted read-only this session).

## Plan that the evidence supports for a new layout
1. Observe from the start pose with head pitch 0.5 (walking resets it to 0 — re-command after every walk).
   Table plane + front edge: code/puppy_box/percep.py (median of per-4cm near-edge x, low-quantile line fit).
   Paws: brown spheres; box: use rim (top edge) pixels — depth sees through the walls.
2. Choose the stance BEFORE moving: the nearest hind paw must end at y in [-0.20,-0.05] (right arm is picked by the
   paw's side) and x 0.30-0.38; the box interior must be within the right-hand reach envelope
   (x<=0.60@y-0.2, 0.55@y-0.3, 0.50@y-0.4 with waist). If the paw is on the left, change heading with ONE turn
   (cmd >=15deg) while >=0.5 m from the edge, then walk straight in. No backward walking exists.
3. Approach in 0.15-0.2 m commands, measuring paw x each time. Stop at paw x <= 0.38.
4. r1_grasp_paw. Immediately check side/held/width, and lift + carry without delay (the hold slipped within ~1 min).
5. Box behind the puppy (the only layout seen): the hand must go ~0.2 m forward of the grasp over a 14 cm rim with
   the puppy hanging below it. Untested ideas: (a) lift to z>=0.40 before moving forward, (b) step forward with the
   puppy held (forward walks still work near the table), (c) try r1_place_in_box once the hand is within the rim —
   its acceptance radius is unknown.
6. Retreat: hand up, back in front of the edge, then HOME joints; never HOME straight from a hand over the table.

## What failed and why
- Attempt 1: L90/walk/R90 lateral shift drifted 0.38 m forward; backward walk then crept forward -> edge 0.09 m. Reset.
- Attempt 2: help requests (overhang) never answered; scene shifted 0.12 m (robot drift); paws ended on the left. Reset.
- Attempt 3: r1_grasp_paw held (right, 0.034 m) but the leg slipped while I previewed the next move; puppy rolled on
  its back. Manual upright-leg grasps chased a target that moved 6-12 cm each time the arm reached (whole-body shift).
  Return to HOME drove the hand onto the table (7deg tilt), recovered.

## Files
memory/PROGRESS.md (full story, frames), memory/TIPS.md, code/puppy_box/{percep.py,controller.py,reachgrid.py,
pathcheck.py (needs mujoco: pip install --target <dir> mujoco)}, frames under runtime/frames/ (ids in PROGRESS).

## INCIDENT (attempt 4): r1_grasp_paw -> robot FELL (tilt 81 deg), disarmed; operator did not answer in 600 s.
Frame 2026110513063419ba79. Treat r1_grasp_paw as a hazard: 1 success, 1 fall from nearly the same stance.
