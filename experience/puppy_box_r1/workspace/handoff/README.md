# Handoff — puppy_box_r1 on r1sonic (sim), session 20261106T010905Z_puppy_box_r1_f6b331

Outcome: NOT solved (0 placements). 7 attempts, resets 6/10, falls 0. NEW: the TASK.md leg-SHAFT grasp held twice
(width .0315/.0351) and the arc lift + waist-assisted raise lifted the hanging puppy to hand z .365 (2/2), but the
grip CREEPS and let go (a) after ~2 min held still, (b) at the first waist-moving extension. Carry never started.
Older handoffs: README_c4f6c3.md (turn-arc walk model, box-behind geometry), README_e0ac0c.md, README_08803f.md.
Workspace was writable this time; code in code/puppy_box/, library skill r1-leg-shaft-grasp-lift (merged, draft).

## SIM UPDATE at 18:13 (operator notice) - read first
The slips on pauses / waist moves were a simulator contact artifact (fixed). r1_walk now self-corrects distance and
heading (< 3 cm / 6 deg) but turns still displace the robot (100 deg turn moved .58 m; a +90/.15/-90 side-step went
RIGHT). After the update the overhang layout changed: hind paw (.551,-.068) with its neighbour only 6.5 cm left; the
lower FRONT paw (.548,-.25) also overhangs and is clean (neighbour 12.5 cm away); both legs point +11..13 deg.
First job next session: re-measure the new walk (turns 15-90, fwd .1-.4) far from the table, then plan the stance.

## Layout (identical on every reset before the update)
Start square, edge .626; puppy head RIGHT, box BEHIND (front wall = edge + .22, rim z = table + .145 = pelvis ~.11),
overhang hind paw (.546,-.023,-.002) from the start, other hind paw 11 cm right / 5 cm back, leg (paw->hip) yaw
-9..-13 deg vs the table normal, horizontal. The reset re-applies the overhang (no help request needed).

## How to plan each subtask in any layout
1. Stance. Wanted: paw at pelvis x .36-.39, y -.20..-.29 WITH THE ARM AT PREGRASP (waist cmd -.35), heading so the
   leg is within ~15 deg of +x. From this start: `run.py approach` (back -.1 x3, turn 70, turn -70) -> paw ~(.51-.53,
   -.19..-.25); `run.py ready` (arm up front, so its reach shift happens now); `run.py fwd .15`, `fwd .1` with the arm
   at ready. Each forward call: .05-.13 m, rotates the robot 0-4 deg and moves the paw 0-6 cm sideways (either way!).
   Paw y at pregrasp > -.17 = restance (reaching left yaws the waist +.1..+.2 and drags the paw further left).
2. Look. The camera cannot see table height at x < ~.36-.40; the hand is never in view near the paw. Use waist yaw
   cmd -.6 to see the paw fully, measure (sphere fit + 4 shaft-top points for the leg yaw), go back to -.35 and
   re-localize (code/puppy_box/reloc.py ICP x/y grid, 18 s; the yaw-grid version takes 8 min - do not).
   Partial sphere fits at the image border were 4-6 cm wrong in x and 4 cm in y -> never grasp on them.
3. Grasp. Align 7-9 cm short with y error < 5 mm, then ONE advance <= 9 cm to paw + 1.5-2.2 cm along the leg at
   paw z + .012 (ik must be <= .03: x .35-.40 at y -.17..-.29), close to 0: .0315/.0351 = held, .0145/.0227 = weak,
   < .012 = miss. Width > .071 before closing = a finger touched the ball (it gets pushed 3-6 cm).
4. Lift: `run.py lift` right after the close (6 arc steps + 3 waist-assisted raise steps): hand (.39..48, y, .27..37),
   width flat .031. The puppy hangs ~22 cm below the hand (head lowest) -> the 14 cm rim needs the hand >= .35.
5. Carry (OPEN PROBLEM): the box front wall is ~.24-.30 m beyond the lifted hand -> ~.25 m of walking with the hand
   at z .36 and an edge clearance ending ~.15 m. Next session: right after the raise, do NOT extend with the waist;
   immediately run `run.py carryloop` (walk .15 cmds, width check, release only >= 3 cm past the box front edge).
   Or test first whether the grip survives 10-30 s held still at z .36 (it slipped after ~2 min at z .275).
   Alternative worth testing: lift lower (z .27) and walk only while the puppy is still in front of the table, so
   the long walk happens before the hand needs the height.
6. Retreat: open, hand up/out (.25,-.32,.15), HOME (waist 0), walk back -.1. Worked every time (no incidents).

## Files
code/puppy_box/run.py (step controller: approach, ready, fwd, look, pre, align, grasp, lift, carryloop, release,
retreat), m.py/look2.py (observe + paw fits), p3d.py (pixel -> 3-D of saved frames), reloc.py (ICP), topdown.py,
plan6/7/8.py (walk planners with TURN_RATIO env; turn ratios this session .74-.96 -> do not trust for < 5 cm).
memory/PROGRESS.md (attempt log), RUNS.md, TIPS.md, INCIDENTS.md, PARAMS.json.
