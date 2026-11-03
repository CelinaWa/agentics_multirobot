# Handoff: puppy_box (G1 + Dex3 under SONIC), curated by the operator, 2026-11-01

## Start here (curated for the real robot, 2026-11-03)

1. [strategy] One program for every layout: `code/puppy_box/run.py` v18.8 (commit 80b8524 of `code/`), the same as the
   library skill `plush-pick-place-box-any-layout` (library main 604b743). Run it as the f20de8 and 3b3f99 sections near
   the end describe: `scene` -> approach -> `grasp,lift` -> carry -> place, looking at each checkpoint overlay. The
   per-layout skills, `code/<session>/` folders and the session sections in between are history: read them for
   failure signatures, do not start from them.
2. [strategy] Evidence (all simulation): last no-code-change evaluation of v18.8 placed 7 of 8 (box right of the puppy
   5/5, box left 1/1, box behind 1/2; one diagonal puppy dropped on the table); v18.2 5/8 on a low table. No fall in any
   evaluation. Remaining weak spots: a low table with the puppy left of centre, puppies lying far back, diagonal puppies.
3. [param] Every number in `memory/PARAMS.json` was measured in simulation. Before the first committal motion, measure
   here and `record()`: the table edge with both arms tucked (`table.edge_from_pelvis_m`), the table height from depth
   (`scene.table_height_m`), the empty-hand reach at table height (one hover), whether the closed hand holds the puppy
   through a 5 cm raise, and how the clear box shows in the real colour and depth images (in simulation depth saw the rim).
4. [sim-only] In simulation a scripted operator slid the table to 0.20 m on request and laid out a new scene on every
   reset; its replies carried ground truth. On the robot a person does this by eye and cannot give distances.

## History: the 2026-11-01 curation

This is not one session's handoff. Eleven earlier sim sessions ran on a puppy model that was wrong (one rigid
body, nothing could squash it) and on layouts that were mostly out of reach. Their grasp conclusions are void and
are not copied here. What follows are facts about the ROBOT that those sessions measured, which still hold.
Nobody has solved this task yet: there is no known working grasp.

## What changed in the scene
- [sim-only] The puppy is soft: fingers sink 2-4 cm into it, and its head, legs and tail flop on soft joints. Grasp tests
  that failed on the old rigid model (tail pinch, leg wrap, paw, torso straddle) are open questions again.
- [sim-only] The robot now starts 0.15-0.20 m from the table edge, between the puppy and the box, and the puppy and a drop
  point inside the box are reachable without leaning (checked on the robot model for each layout).

## Robot facts (each seen in at least one session; re-check, do not assume)
1. Falls: one session fell (20261101T011122Z): waist lean 0.30 held with the right arm stretched far forward-down;
   the pelvis slid back 8 cm during the 4 s ramp, waist yaw drifted 0.15, and it fell to its right side 14 s into
   the hold. Repeated leans of 0.21-0.42 slid the pelvis back 19 cm in total (20261101T004902Z). Lean at most
   0.2, in its own ramp, never hold a leaned reach.
2. The pelvis moves back several cm with arm work at the table. Re-measure the table edge (or another landmark)
   after every motion before trusting a target in the pelvis frame.
3. First frame after start is wrong: the first observation is taken before the controller eases into its
   standing pose (5-7 cm off). Make one small motion or wait 2 s, then observe again before planning.
4. The idle arm: a waist turn swings the whole upper body, so the idle hand sweeps across the table. Tuck it
   back and up first (shoulder pitch about +0.45, elbow about -0.5).
5. The start pose holds both forearms forward with the hands about 0.19 m ahead of the pelvis and a few cm above
   the table top. Raise or tuck a hand before moving it sideways: an arm swept along the table pushes objects.
6. g1_free_space_move (cuRobo) works. It refuses to plan from a pose where a thumb touches a hip and now says so
   (status start_state_in_self_collision with the link pairs): move that hand a few cm first. planning_failed
   with no pairs usually means the target is out of reach.
7. Joint tracking: SONIC follows arm references with 0.2-0.5 rad lag during 2-4 s ramps; a joint that stops
   under a growing command is in contact, not lagging. Check measured joints, not the commands.
8. Hands: the Dex3 opens only about 12 cm between thumb tip and finger tips. Fully closed on nothing is a fist;
   fingers that stop short of the fist mean something is in the hand.
9. The clear box is invisible in depth: find its rim in the colour image; pixel_to_position on a wall pixel
   returns the table behind it.
10. A safe end pose: shoulder pitch back to about +0.7 (hand back), then elbow 0.3, wrist 0, shoulder roll -0.2,
    then elbow 0.6 and shoulder pitch 0.45.

## Library
skill-library has drafts from those sessions: palm-pose-servo (own IK + measured-FK servo), place-reach-check
(span check), start-scene-check (no-motion checks after start), dex3-grasp-geometry (hand geometry on the model),
hand-claw-pregrasp. The tools are fine; their grasp-failure evidence is from the rigid puppy (see the notes at
the top of each).

## Session 20261102T005206Z (sim, soft puppy): what was learned - task still NOT solved
Layout here: puppy right of the robot (torso ~0.23-0.34 m ahead, y -0.21), clear box left (x 0.36-0.65, y 0-0.40).
Puppy ended on the floor after 7 grasps; box untouched; no falls.

How to plan each subtask in a new layout:
1. Perceive: `code/puppy_box/percep.py` (depth cloud in pelvis/level frames, table height histogram, cream blob
   above the table excluding the box footprint, ray/plane intersection for the box's two faint rectangles at
   table and rim height, `draw_box` overlay to check). Move the hand out of the line of sight first (it occludes).
2. Grasp: library `plush-pinch-grasp` (pre-curled pinch, palm-down fingers forward, palm xy 6-8 cm in front of the
   puppy's far edge at mid-body, descend to stall, close 3 s, finger-stall check). 4/5 lifted.
3. Transport is the open problem. The arm on the box side must do the carry; palm-down it reaches only x ~0.44
   (left arm, box ahead-left), the right arm cannot reach into a box on the left even with waist yaw (tracks ~40%).
   Plan the whole carry before grasping: compute IK (palm-pose-servo `ik.solve` on the measured joints) for every
   waypoint incl. the release point over the box centre + puppy half-width, and do not grasp if the release point
   is out of reach. Candidate fixes: a stance closer to the box (ask whether walking is allowed; first walk does a
   warm-up turn stepping 0.1-0.2 m forward), grasping nearer the puppy's near side so it hangs forward, release as
   soon as the body clears the wall and nudge the rest in from above.
4. Return: open, default hand shapes (`g1_hand_shape` with no shapes), arms via sp 0.45/el -0.5 -> sp 0.7 ->
   el 0.3 -> el 0.6 + sp 0.45-0.6; check fingers read ~0 (a finger at 1.0 means it touches the table edge).

## Session 20261102T012011Z (sim, soft puppy): what was learned - task still NOT solved
Layout: puppy LEFT (head left, legs over the table's front edge, torso x 0.32-0.45 / y 0.06-0.23 at start), clear
box RIGHT (x 0.37-0.65, y -0.46..-0.08, rim z +0.06 pelvis). The controller's start easing left the robot 0.27-0.28 m
from the edge (task says 0.15-0.20) and dragged the puppy off the table (the start-pose left hand touched it).
8 grasp closes, 5 resets, puppy on the floor at the end; robot never tilted > 4 deg, box untouched.

How to plan each subtask in a new layout (in this order):
1. Stance first. Measure the edge (code/puppy_box/layout.py) and run plush-pinch-grasp `reach_check` for the palm at
   (far edge - 0.07, torso centre y, top - 0.015) AND for the release point over the box centre (+ puppy half-width
   inside the near wall) for the hand you will use. If either fails, fix the stance before touching the puppy
   (ask the operator: walking barely advanced here; or ask them to reposition the robot/puppy). Here, at 0.25 m,
   the left palm reached at most (0.42, -0.10, 0.17) even with waist yaw -0.5 measured: 2 cm inside the box's
   left wall, so the carry could not have worked from that stance either.
2. Waist: set lean (<= 0.2) / yaw / roll, wait, set the command equal to the measured value (`settle_waist`), observe,
   THEN bring the arm over the puppy. Yaw tracks ~40-50 %; unwinding it later swept the puppy off the table.
3. Grasp: pre-curled pinch with the palm pressed 1-2 cm INTO the plush (palm stall), fingertips past the far edge.
   Resting on top = fist (4/4). Deeper finger curl held at close but slipped at <= 3 cm (2/2): the shape is not the
   fix, the pressed hook geometry is. Do not press repeatedly near the edge: it pushes the puppy off.
4. If the puppy's legs overhang the edge, ask the operator to put it further in before grasping (every slip = floor).
   [operator 2026-11-02: in that session the sim puppy also crept toward the edge by itself (a model bug, fixed
   since); the 5/5 is not a clean measure of how easily a real plush rolls off.]
5. Return: open, `g1_hand_shape` with no shapes, waist 0, arms sp 0.45/el -0.5 -> sp 0.7 -> el 0.3 -> el 0.6 + sp 0.5.

## Session 20261102T015700Z (sim, soft puppy): TASK SOLVED 3/3 placements (2 with a hand-in-the-loop, 1 with steps chained, no corrections)
Layout: puppy RIGHT (x 0.18-0.40, y -0.05..-0.31, head right, tail toward the box, legs toward the robot), clear
box LEFT (near wall x 0.245-0.33, inner wall y +0.04..+0.06, long side left-right). Edge 0.18-0.21 m.
Use the library skill `plush-pick-place-box-one-hand` (code = code/puppy_box/controller.py, steps prep, perceive,
grasp, lift, carry, release, home, verify). How to plan each subtask in a new layout:
1. Perceive (no motion): tuck the box-side arm, swing the plush-side arm out (roll -0.6) and look. `boxdet.locate_box`
   finds the clear box as the pale low-saturation region over the wood (sim depth shows its rim top at +0.14; the
   real depth will not, the mask accepts both). Its far rim reads ~4 cm long: use near wall + 0.15 as the centre depth.
   Check the overlay yourself. If the plush is beyond x ~0.40, it is out of reach: ask the operator (2 of 5 resets
   in this session put it at x 0.46-0.74 with the legs away).
2. Grasp: plush-pinch-grasp pre-curled pinch, palm x = far edge - 0.07 (back off toward the robot until the pressed
   pose top - 0.03 is reachable), y = mid-body (the blob runs out of the image on the outer side: inner end - 0.15).
   Closes at index 0.73-0.90 / middle 0.93-1.07 = held (3/3).
3. Lift: 3 s steps to z 0.20 straight up, then toward the body centre (0.31, -0.12, 0.27). The plush hangs with its
   legs toward the robot, lowest point z ~0.13-0.18: clear of a 14 cm rim.
4. Cross-body carry (the hard part, solved): the hand on the plush side cannot reach the box by itself. Move the
   box-side arm far back (sp 0.8, el -0.3), waist yaw toward the box cmd 0.5 then 0.8 (measured 0.29 -> 0.41 ->
   creeps to 0.47-0.60), waist pitch 0.15, WAIT 3 s, IK on the measured joints, 4 s ramp, re-solve on the measured
   waist and correct once. Palm lands ~(0.33-0.37, 0.215-0.25, 0.19-0.20) = 7-9 cm inside the near wall. The grip
   creeps (index 0.5 -> 1.1) on these moves but held 3/3. No fall, tilt <= 6.8 deg.
5. Release at once (no holding while thinking): open, then waist pitch 0 + shoulder pitch -1.7 together (hand up to
   z 0.35), then wrists 0 / elbow 0.3, sp -0.8, waist 0, sp 0.7 el -0.3, both arms sp 0.5 el 0.6.
6. Margin: a robot 5 cm further back than 0.21 m would probably not get the body inside (visible plush only 46-49 %
   over the box footprint at release with the box near wall at 0.33). Then do not lean past 0.2: ask the operator.
For a box on the SAME side as the plush, skip the waist turn: plush-pinch-grasp + a straight carry.
- [strategy] [operator 2026-11-02, from the sim ground truth] Run 4 (unattended run.py) did NOT find the puppy out of reach
  after a bad reset: the reset put it 0.31 m ahead (in reach); the controller's prep step then unfolded the right
  arm forward in ~2 s, the hand swept up through the puppy and pushed it 20 cm away (to 0.52-0.6 m ahead), and the
  pelvis rocked back 8 cm. Fix prep before reusing it: raise the hand above the puppy's top (z >= top + 0.08)
  BEFORE moving it forward, slow ramps, and observe before and after prep; never unfold an arm through the space
  above the puppy.
- [strategy] [operator 2026-11-02, from the sim ground truth] In the 02:38 success the LEFT fingertips (index, middle) brushed
  the puppy for ~5 s during the lift: the left arm was raised in front of the body, not tucked. Rule now: the left
  hand must never touch the puppy (such an attempt does not count). Tuck it back and up (shoulder pitch +0.45, elbow
  -0.5) before the first motion and check it is still there before every grasp and lift.

## Session 20261102T022251Z (sim, soft puppy): SOLVED 2x with the RIGHT hand, box on the SAME side (right)
Layout: puppy centre-left (head left, legs to the robot, body y -0.05..0.20, far edge 0.30-0.34, top ~0.09 pelvis), clear
box RIGHT (near wall x 0.24, inner wall y -0.125, later -0.203 after the operator re-placed the robot). Rule: right hand
only, left tucked. Library skill `plush-pick-place-box-same-side` (code = code/puppy_box/pb3.py; mv.py, carry.py, reach2.py).
How to plan each subtask in a new layout:
1. THE key fact: solve palm IK on the COMMANDED row (incl. the commanded waist), not the measured waist. SONIC tracks
   the whole-body pose: a waist that under-tracks (cmd 0.45 -> meas 0.2) is compensated by the arm, FK(commanded) ~
   measured palm within 1.5 cm. Earlier skills (palm-pose-servo, the one-hand carry) solve on the measured waist and
   miss by ~10 cm, which is where their creep/bias corrections came from.
2. Raise the hand without sweeping the box: shoulder pitch back to 0.9 first, then elbow -0.9, then sp 0.1, wrist roll -1.3.
3. Perceive from two views (each hides one end of the puppy); box from a view with the box unclipped (turn the waist
   toward it if needed). Mid-body = blob centre 6 cm away from the head; then LOOK at the hover frame and shift 2-3 cm
   toward the tail (a high hand projects toward the head).
4. Choose ONE commanded waist that reaches both the pressed grasp and the release (reach2.best on the commanded row,
   candidates 0..0.55); never change the waist while the plush hangs (lost it on a 0.7 -> 0.45 swing).
5. Grasp: pinch shape (set it again after any home: home restores the flat default -> fist), press to top - 0.06 cmd
   (palm stalls ~0.09), close 3 s; lift 4 cm / 3 s steps from the press command; carry arm-only 5 s / <= 8 cm steps at
   palm z 0.30; release with the palm >= 8 cm inside the walls; home.
6. The robot drifts back from the table over a session (operator measured 0.175 -> 0.399 m after ~25 min of arm work).
   Re-measure every run; if the grasp x > 0.36, ask the operator to re-place the robot (no walking).
For a box on the LEFT with the right hand: the cross-body version (plush-pick-place-box-one-hand) should also switch to
commanded-row IK; untested.

## Session 20261102T025843Z_puppy_box_152e7f (sim): SOLVED 2/2 after fixes (runs 3-4), right hand, box on the same side
Layout: puppy centre-left (head left, legs to robot, y -0.06..0.27, far edge 0.38-0.41, top +0.04 pelvis), clear box RIGHT
(near wall x 0.27-0.31, inner wall y -0.14..-0.15, centre y -0.34). TABLE LOWER than before: table z -0.052 pelvis.
Controller: code/puppy_box/pb3.py (= library plush-pick-place-box-same-side v2 controller3.py). Run per step:
prep, perceive left, [lean 0.1 + perceive left again if hover reports no reach], hover, grasp, lift, carry, release, home.
How to plan each subtask in a new layout:
1. Reach decides everything: a low table (or the far end of the 0.15-0.20 m stance) puts the pressed grasp (top-0.06)
   out of reach. Order of remedies, each checked by multi-seed IK on the COMMANDED row before moving: waist yaw cmd up to
   0.7 (measured only ~0.3) -> back the palm off <= 3 cm / press less (>= top-0.03, but shallow = slip) -> lean 0.1 in its
   own ramp, re-perceive (pelvis slides ~1 cm) -> ask the operator to move the robot closer (they may not).
2. Grasp point: blob y-centre shifted only 3 cm away from the head (6 cm grasped the rear 2/2: the press and close shove
   the plush toward its head). Look at the hover frame: hand over the FRONT half of the body is right.
3. A plush pinched by one end still hangs (finger check says "lost"): lower it to palm z ~0.16, open, raise, reset.
4. Carry/release unchanged from 810c60: arm-only 5 s steps, release at once with the palm >= 8 cm inside the walls.
5. The operator's reply about the left hand never came (asked twice): it stayed tucked >= 16 cm above the table; unverified.
6. Run 5: `python code/puppy_box/run_pb3.py left` chained everything (lean fallback built in) and the puppy went into the
   box; only the post-release raise was refused (jump 1.05), fixed since (max_jump 1.6). Next session: run it
   unattended on a new layout, then look at the hover/grasp/lift frames it saved, and mark the skill verified after 2 clean runs.

## Session 20261102T033140Z_puppy_box_221440 (sim): NOT solved - puppy RIGHT (head right), box LEFT, right hand only
Layout: edge 0.18, table z -0.032..-0.037 pelvis (high table). Box LEFT: footprint x 0.26-0.58, y -0.075..0.35 (long side
left-right), from colour-image corners via percep.plane_hit (boxdet was 0.2 m off). Puppy RIGHT: tail tip y -0.13..-0.19 next
to the box's right wall, body to y ~-0.43..-0.49, mid-body y -0.30..-0.36, far edge 0.36-0.41, top +0.043..0.047, legs 4-5 cm
from the front edge. Head outside the camera view (waist yaw cmd -0.7 measured only -0.06..-0.26).
What happened: 4 palm-down pinch closes (waist yaw cmd .55-.7, waist roll .15-.2, yaw -30/0, mid and rear body) all ended
in a fist; 2 pushed the puppy off the front edge; no fall, box untouched. Root cause: the grasp sits at the right hand's
LATERAL reach limit; there the palm rides 4.5-8.7 cm above its commanded-row target and deflects 5 cm back/right under
load, so it never presses into the plush.
How to plan this layout next time (untested ideas, in order):
1. Check the MEASURED free-space palm at (far-0.07, mid-body y, top-0.03) before any press (hover there with margin);
   if it does not get there, the pinch will fail: do not press near the edge.
2. Ask early and concretely for the puppy to be placed with its mid-body at |y| <= 0.25 (this operator did not move it on
   help requests; a reset returns it to roughly the same place, 0.46-0.52 m from the pelvis).
3. A grasp that needs no downward press: a fingers-down claw on the TAIL (thin, at y -0.13..-0.19, x 0.28-0.35, well
   inside reach) - lift the plush head-down high enough (palm z >= ~0.45 pelvis for a 14 cm rim), carry over the box
   centre (0.40, 0.06..0.10 reachable at waist cmd 0.7 / roll 0.15), lower until the head touches the box floor, open.
   Hand orientation for that claw is not worked out (see dex3-grasp-geometry).
4. The release side is easy here: (0.40, 0.06..0.10, 0.29-0.40) is IK-reachable at waist cmd 0.55-0.7.
5. (04:01 run) When the puppy was at mid-body y ~-0.20 (far .39), pb3 hover/grasp/lift/carry WORKED (waist cmd 0.7) - the
   failure moved to the release: with the box on the LEFT the right palm stops at y ~0.04 (7 cm past the box's right wall)
   whatever the command (cmd 0.08-0.13, waist cmd up to 1.0, measured <= 0.49), and the plush hung right of the palm and
   fell beside the box. Next time: (a) require release >= 12 cm inside the inner wall reachable on the MEASURED palm (hover
   there empty first, before grasping); (b) grasp the head-side half of the mid-body so the body hangs toward the box;
   (c) consider releasing lower (palm z ~0.22, legs/body just above the rim) so it drops in rather than tipping out;
   (d) if the release cannot reach, ask the operator to move the robot (or the box) ~10 cm to the left.

## Session 20261102T041502Z_puppy_box_6a0bfc (sim): SOLVED 1/1 - box BEHIND the puppy, right hand only
Layout: puppy straight ahead/right (head right, legs to the robot), clear box directly behind it. Library skill
`plush-pick-place-box-behind` (code = code/puppy_box/pbb.py + lean_release.py; pb3 primitives underneath).
How to plan each subtask in a new layout:
1. Stance first, before any motion: measure edge, puppy and box (box floor corners in colour via percep.plane_hit or
   pbb.box_behind) and run random-seed IK (reach_rs.best_rs) on the commanded row for the grasp AND a release
   >= 8 cm past the near wall. Here the robot started 0.26-0.27 m from the edge (it had slid; the operator disarmed,
   re-armed and moved the objects ~8 cm closer) and nothing was reachable -> asked; operator allowed g1_walk 0.12 m
   (carry=true, arms lowered beside the hips first: shoulder pitch back to 0.45 BEFORE bending the elbow, the start-pose
   fingertips were on the puppy's paws at the table edge). Each 0.12 m walk advanced only ~3 cm; legs straighten after
   a walk (table 3 cm lower in the pelvis frame); g1_crouch(0, forward .05, arms_home False) restores it but slides 1.7 cm back.
2. Expect the pelvis to slide back ~4 cm with lean 0.1 + prep: perceive after the lean.
3. Grasp: palm far-0.10 (not -0.07) when the 3 cm press is otherwise out of reach; yaw 0; held at idx 1.02 / mid 0.91.
4. Carry straight forward at palm z 0.24, waist fixed (yaw cmd 0.7, lean 0.1). At the reach limit the palm lands ~4 cm
   short of the command; check the hanging plush (cream + dark paws from depth) against the near wall before opening.
5. Release: gated lean (lean_release.py 0.2 <wall+0.03> x y 0.28): open only if the measured palm is past the wall,
   then hand back/up + unlean at once. The puppy fell fully inside. Then pb3 home.
6. If the box's near wall is beyond ~0.44 m from the pelvis after the lean, it is out of reach: ask the operator.
7. Runs 2-3 after resets FAILED (1/3 overall): the reach-limited shallow grasp (far-0.10..0.11, press top-0.03) slipped
   on the lift both times and the puppy fell off the front edge. A close with finger BASE joints (index_0/middle_0)
   < 0.7 is a weak hook even though it passes the < 1.3 check: open, raise, re-grasp. The real fix is reach: a stance
   ~0.15 m from the edge so the deep press (far-0.07, top-0.06) works - ask the operator for it before grasping.
   Session ended with the robot home, hands open, box in place, puppy on the floor (operator informed).

## Session 20261102T045636Z_puppy_box_37fe31 (sim): NOT solved - box BEHIND the puppy, stance out of reach, stopped safely
Layout: puppy straight ahead (head left, legs to the robot, far edge 0.42, mid-body y -0.03, top +0.03 pelvis), clear box
directly behind it (near wall 0.49-0.53, centre ~(0.62-0.66, -0.02), y -0.21..0.17). The start easing left the robot 0.32 m
from the edge and the start-pose hands dragged the puppy to the floor before any command (reset 1). Operator allowed two
walks (0.12, 0.15 m, carry=true, arms beside the hips via code/puppy_box/arms_down.py): +4 cm, then +0 cm (edge 0.28).
How to plan each subtask in a new layout:
1. Stance gate, no motion, FIRST: perceive (pbb.py perceive; check box corners in colour with percep.plane_hit), then
   `skill-library/skills/plush-pick-place-box-behind/code/reach_probe.py check <obs.json> <near_wall> <y>` (~3 min).
   It subtracts the ~4 cm model bias at the reach limit. If not ok, ask the operator at once for a closer stance;
   walking near the table does not fix it (2 walks = +4 cm). Here it said: need 0.575, expected real max 0.50.
2. Prep: do not swing the right arm forward next to a puppy lying straight ahead; pb3 prep bumped it (~40 deg rotation,
   nose over the edge). Raise the hand at the side (y <= -0.30) first, then bring it over.
3. If out of reach: put the arms beside the hips, hands open (the end pose), and report; do not touch the puppy.
- [param] [operator 2026-11-02, from the sim ground truth] Stance and walking: the robot starts ~0.3 m behind the table edge so
  its hanging hands (they reach ~0.26 m ahead of the pelvis at table height) clear the edge. g1_walk now stops on leg
  odometry: forward walks of 0.1 / 0.2 / 0.3 m moved 0.10 / 0.22 / 0.28 m (GT), carrying 0.15 -> 0.12; the reply's
  feedback.odometry_m says how far it went. Before this, short walks barely moved the robot. To step up: tuck both
  arms back and up, g1_walk forward with carry=true by (measured edge distance - 0.20 m), measure the edge again.
- [strategy] [operator 2026-11-02, from the sim ground truth] Your table_edge() (1st percentile of x at table height) reads your
  own hanging hands as the edge: standing, they hang at table height 0.20-0.26 m ahead of the pelvis. Seed 27: it said
  0.20 m, the edge was 0.33 m; with the arms tucked it said 0.23 for a true 0.25. Tuck the arms before measuring, or
  ignore points within ~10 cm of your hands (forward kinematics) and take the median over y-bins of each bin's nearest
  table point (the table spans the view; the hands cover a few bins). On the real robot the same will happen.

## Session 20261102T062330Z_puppy_box_89afad (sim): SOLVED - puppy RIGHT (head right), box LEFT, right hand only
Library skill `plush-pick-place-box-cross-right-hand` (code = code/puppy_box/lr.py on top of pb3.py). How to plan each
subtask in a new layout:
1. Stance: start was 0.33 m from the edge (operator); my depth read 0.20 on the very first (tilted) frame - trust the
   operator's number. Tuck both arms, g1_walk carry=true: 0.13 -> 0.094 m, 0.07 -> 0.042 m (odometry). Re-measure.
2. Box: always from colour corners (percep.plane_hit at table z + 0.14 for rim corners, table z for floor corners);
   boxdet put a box on the left 0.2 m too far left again. Two views agreed within 1-2 cm.
3. Waist for a cross-body task: yaw cmd 1.1 + ROLL +0.15 (roll is what makes the pressed grasp at y -0.23 reachable),
   set once in its own ramp before the arm goes over the plush; never changed until after the release.
4. Arm over the plush: the IK branch at waist 1.1 (sp .68, yaw -1.48) is ~2 rad from the prep pose: go through a raised
   via-config and FK-check the joint path (lr.py over). Never ramp an unchecked random-seed IK result (one went overhead).
5. Grasp the head-side half of the mid-body (perceived centre - 3 cm toward the head) + 1.5 cm in x (palm lands short).
6. Release: the palm cannot reach far left (measured y .09-.13 for cmd .20). Instead turn the palm yaw 15 -> 75 at the
   release point: the body swings forward-left over the interior (lr.py swing with a depth check), open at once.
For head LEFT with the box on the left (untested): grasp the head-side half (right of centre) and work out the swing
direction from where the body hangs relative to the palm in the carry frame.
- [param] Run 2 (after a reset, same layout): SUCCESS again (2/2). Yaw cmd 1.1 gave a standing tilt of 6.5 deg this time; 0.9 + roll
  0.15 (tilt 4.3) reached everything, so the skill's run.py now defaults to 0.9. The swing depth gate under-counts when
  the arm hides the body: look at the swing frame and release at once if the body hangs over the interior.
  Controller: `python -c` with skill-library/skills/plush-pick-place-box-cross-right-hand/code/run.py main(ctx, box=...)
  after pb3 prep + perceive right and your own colour-corner box (copy in code/puppy_box/run_lr.py).

## Session 20261102T070019Z_puppy_box_170401 (sim): SOLVED 2/2 (+ run 3, see memory/RUNS.md) - box RIGHT, puppy centre-left head left
Layout: puppy far edge .38, y -.095..+.234, top +.015 pelvis; box x .23-.55, y -.165..-.555 (long side left-right); low table.
Library skill `plush-pick-place-box-same-side` v3 (run: `PYTHONPATH=. python skill-library/skills/plush-pick-place-box-same-side/code/run.py left`).
How to plan each subtask in a new layout:
1. Stance: robot started ~0.28 m from the edge. Tuck both arms (code/puppy_box/tuck.py), measure the edge with the arms
   tucked (code/puppy_box/layout2.py on the saved obs), g1_walk carry=true by (edge - 0.20) (0.09 -> odo 0.069).
2. After the walk the legs are straight (pelvis 0.80): restore with g1_crouch(0, forward_rad 0.05, arms_home False) BEFORE
   perceive/hover; without it the pressed grasp was 4-9 cm out of reach at every waist. It slides the pelvis ~3 cm back.
3. Then the skill chain (prep, perceive, hover, grasp, lift, carry, release, home). Run 1 I looked at the hover frame and
   shifted grasp y 2 cm toward the tail (code/puppy_box/shift.py 0 -0.02); run 2 had no shift and still closed mid-body.
4. Confirm placement: home frame sees the puppy through the clear walls; the operator confirms.
For other layouts use the library skills listed in skill-library/INDEX.md (box left: cross-right-hand; box behind: box-behind).

## Session 20261102T091205Z_puppy_box_8cef10 (sim): SOLVED once (run 5 of 6) - box BEHIND and RIGHT of the puppy, right hand only
Layout: puppy head right, legs to the robot, far edge 0.36, body y -0.06..-0.31, top +0.03; clear box behind it, rotated ~14 deg
(near wall 0.36-0.44 from right to left), table z -0.0625 pelvis after the walk. Start edge 0.295 (arms tucked).
How to plan each subtask in a new layout:
1. Stance: walking did NOT advance here (0.09/0.09/0.12/0.15 m -> 5.6/0/0/1 cm, odometry ~0). If the edge is > 0.22 after one walk,
   ask the operator at once to place the robot ~0.18 m from the edge; from 0.24 the deep press was unreachable.
2. Arm over the puppy: from the hand-out pose go via waist-0 waypoints, then waist 0.35, then 0.7 with the hand high (direct: jump 2.71 refused).
3. Grasp gate (new): close only when the measured palm z <= plush top (pressed in), and lift only if a finger base joint >= 0.7.
   pb3 grasp's stall rule can stop at command top+0 with the palm 4 cm above the top (run 1); forcing top-0.06 at x 0.25-0.30 still
   left the palm at top+0.017 with tilt 6 (run 2). Both slipped at palm z 0.23-0.24. Untested next ideas: grasp nearer the robot
   (far-0.10) where the palm can press in, fingers yawed toward the head to wrap the body's thickness, or a closer stance.
4. Re-perceive the box before the carry: the pelvis heading drifted +7 deg with the waist-yaw arm work (box corners moved 5-8 cm).
5. End pose: skill-library .../plush-pick-place-box-behind/code/home_safe.py (both arms tucked, waist 0, hands open; tilt 0.8).
6. UPDATE runs 3-4 (after reset 1): the grasp that WRAPPED and lifted = fingers perpendicular to the body axis (PCA of body points from a
   full view: waist joint -0.25 alone with the arm aside), palm 0.096 short of the far side, LIGHT press (bb_8cef10.py grasp_deep with
   DEPTHS=0,-0.015,-0.03 GATE=0.012 GY=<yaw>): base joints 1.00/0.92, lift to 0.27 held. A deep press at far-0.07 put the fingertips on the
   table (base 0). The carry then failed: the grip crept after 8 cm and the palm stopped at x 0.39 = the near wall (release 0.49 refused).
   Next time: same grasp, but get the stance closer FIRST (operator) so the release is >= 8 cm past the near wall on the MEASURED palm
   (hover there empty before grasping), or tilt the hand to swing the body over the wall and open at once.
7. SOLVED in run 5 (operator: INSIDE), after reset 2, same layout. Sequence = skill-library plush-pick-place-box-behind v3 `code/run_bb.py`
   (also code/puppy_box/run_bb.py): prep -> hand aside + perceive (PCA yaw, palm far-0.105) -> lean 0.1 -> staged approach -> hover (LOOK)
   -> light-press grasp -> lift 0.27 -> carry straight forward at the grasp y (cmd 0.465, measured 0.438 = wall + 0.03) -> gated lean release
   (lean 0.2, palm 0.483, open) -> home_safe. Run 6 (same code, reset 3) dropped the puppy to the floor: the grasp landed 2 cm further
   forward, the base joints fell to 0.65/0.60 in the lift and the plush hung by one end on the robot side at the open. Before the next run
   add: base joints >= 0.8 at the end of the lift (else lower to palm z 0.16 over the table, open, re-grasp), and wait ~1 s after the open
   before the hand retreats. 1/2 with this sequence; 2 resets were left at the end.

## Session 20261102T095006Z_puppy_box_cab719 (sim): SOLVED 2/5 (runs 4-5 stopped safely: box not detected) - box RIGHT, puppy centre head left, right hand only
Library skill `plush-pick-place-box-same-side` v4 (merged 94691ea): `PYTHONPATH=. python skill-library/skills/plush-pick-place-box-same-side/code/run.py left stance,prep,perceive,hover`
then LOOK at the hover frame, then `... run.py left grasp,lift` (LOOK at the lift frame), then `... run.py left carry,release,home`.
How to plan each subtask in a new layout:
1. Stance: `stance` step (tuck both arms by the back path, edge = median of per-bin nearest table points, walk carry=true,
   crouch(0, fwd .08) after each walk). Start .278 -> .221 in 3 walks (the first walk's warm-up moved ~0). Check the IMU yaw it
   returns against the start heading; turn back in place with g1_walk(forward 0, turn) - it creeps LEFT with arm work and crouches.
2. Perceive: the skill's two views need the box inside view B. If perceive reports a box far away (x > 0.6) or on the wrong side,
   it is out of view: do not grasp. Re-square the robot first; if the box is still not visible, ask the operator.
3. Hover: LOOK. Hand over the neck/front legs -> shift 2-3 cm toward the tail before grasping (the one chained run failed there).
4. Grasp/lift/carry as v3. If the carry's grip flag fires but the plush still hangs (look), finish the carry in 2 short moves and open.
5. Home: v4 ends with the arms down beside the hips. The old end pose at a 0.20 m stance hit the table edge -> self-step 0.17 m.
6. Runs 4-5: once the robot's base had drifted (~8 cm left, heading creeping), the box stayed out of perceive view B and locate_box
   returned garbage (x 0.9, or the drawer unit on the left). Add a plausibility gate on the box (side = task side, near wall
   0.18-0.40 m) and an extra tucked-arm view before hover; ask the operator early if the box is not in any view.
End state: robot standing, arms down, hands open; puppy on the table (not in the box); box upright.

## Session 20261102T103630Z_puppy_box_0878bd (sim): SOLVED once (attempt 2 of run 1), run 2 failed safely - box LEFT ahead, puppy far RIGHT head LEFT
Layout: box ahead-left (rim x .25-.62, y -.17..+.28, table-aligned; robot heading ~10 deg left of the table), puppy on the right,
snout toward the box's right wall (gap ~5 cm), body y -.20..~-.56, x .18-.32. Library: cross-right-hand v3 `code/headleft_0878bd/`.
How to plan each subtask in a new layout:
1. Stance: same-side skill `run.py left stance` (0.303 -> 0.21 in one walk). Fine.
2. Perceive: with the box on the left do NOT use pb3 perceive (raised arm hides the scene). `run_cr.py look` (both arms tucked,
   waist cmd -0.7) shows the whole puppy; `plan` gives the PCA axis, a reachable grasp and the yaw. Measure the box from a waist-0
   tucked view with layout2 (rim corners) and pass BOX=...; the turned view clips the box.
3. Reach: check with plan_cr.py / reach2.best on the COMMANDED row before moving. Mid-body at y -0.42 is NOT pressable (err 4-9 cm
   at every waist); the press is reachable at y >= -0.38 with waist (0.45, 0.15-0.25). Roll 0.25 = tilt 5.4 standing, 7.5 pressing.
4. Grasp: `cr.py grasp_soft` (1.5 cm steps, stop on tilt/stall, +2 cm, close). Worked once at yaw -36 on a diagonal body (fingers
   0.92/0.84); three closes at yaw -10..-30 on a body along y ended in a fist and pushed the plush ~5 cm toward the robot each.
   The plush top is ~3.5 cm higher than depth says (palm stalls at +0.065).
5. Carry: arm only, 6 cm / 5 s at z 0.33, palm yaw to +15 for the release reach; empty fist is 1.52-1.57, a held plush can read
   1.30-1.40 (STALL 1.45). At the box centre the body hung over the interior: opened at once, no swing.
6. Home: home_cr.py (FK-checked path to the high prep pose with waist 0, then tuck the right arm like the left).
Best next idea for this layout: ask the operator to place the robot ~10 cm further right (mid-body then reachable without the
tilt limit), keep yaw -36..-45, grasp at the measured mid-body.
- [param] Run 3 (reset 2): yaw -40 at the run-1 success point and on the torso: 2 more empty fists. Final tally 1 success / 3 runs,
  1 / 6 grasp closes. For this layout the GRASP is the blocker: the neck is too thin for the Dex3 and the mid-body (y ~-0.42) cannot
  be pressed from a 0.21 m stance with the pelvis ~0.4 m left of it. Get a stance further right first (operator), then use run_cr.
- [strategy] Recovery from a low pressed command: sync the arm command to the measured joints, straight up in 6 cm moves, then home_cr.py.

## Session 20261102T111240Z_puppy_box_2a8660 (sim): SOLVED once (run 1, 4th close), runs 2-3 failed safely - puppy RIGHT head RIGHT, box LEFT
Layout: box ahead-left, table-aligned (x .33-.61 / y -.03..+.38; after reset 1: x .26-.56 / y -.015..+.38), puppy right, head right,
legs to the robot, body centre (.30-.33, -.28..-.32), edge .196-.226 with arms tucked (no walk; the 0.344 first-frame reading was the
pre-settle frame). Library: cross-right-hand v4 `code/x2a/run_x2a.py` (copy: code/puppy_box/run_x2a.py). How to plan each subtask:
1. Stance: same-side `stance` (tuck by the back path, edge from per-bin nearest table points). Measure the edge only with the arms tucked.
2. Perceive: run_x2a `look` (both arms tucked, waist -0.7) + `plan` (PCA). NOT pb3 perceive for a head-right puppy: it cut the head off
   and the grasp landed on the rear (2 empty fists). Box from colour corners (percep.plane_hit on rim/floor corners): layout2 was
   3-4 cm off twice and once found half the box; pass BOX=x0,x1,y0,y1.
3. Waist 0.7/0.15 (0.9 tilted 6.24 with the arm over y -.30). Hover with one xy correction on the measured palm.
4. Grasp is the bottleneck (1 lift in 3 holds): diagonal fingers (yaw -57.6 from PCA on a diagonal body) held to z .36; fingers
   across a body lying along y (yaw -12/+5) slipped at palm z .18-.24; forcing yaw -45 made the palm ride 4 cm high -> empty fist.
   Next: gate the close on palm z <= plush top + 0.015 (8cef10) and try yaw -30..-45 with that gate.
5. Lift 3 cm / 3 s steps; carry arm-only at z .35 to (box x0+.10, y0+.13, .30) with IK tol 2 cm; palm-yaw swing 50/75 put the body over
   the interior; LOOK, open at once; home_cr. Every failed lift dropped the puppy back near the table's front edge: ask for a reset.
- [strategy] Runs 4-5 (same session): GYAW=-30 forced (body along y): run 4 lifted to z .31 then slipped on the 1st carry step; run 5 lifted
  (one finger), carried, but after the swing the body stuck out TOWARD THE ROBOT in front of the near wall (held near one end).
  Moving the palm 8 cm forward with yaw 30 (step `adjust`) put it over the interior -> INSIDE. Tally 2/5. Always check the hanging
  body against the box footprint in x AND y (depth points + look) before opening; the swing's own check only tests y.

## Session 20261102T115009Z_puppy_box_eed312 (sim): NOT solved (0/3) - box BEHIND the puppy (head right), puppy's back 3-4 cm from the box wall
Layout: robot edge .26 (stance walks .304 -> .254, two more walks moved 0); box behind and rotated with the robot heading (near-wall
rim .436 at y -.07 -> .38 at y -.36 in the pelvis frame); puppy body x .27-.40, y -.12..-.37+, crest x .33 z +.035, head out of view.
Controller: code/puppy_box/run_eed.py (= library plush-pick-place-box-behind v4 code/run_eed.py). How to plan each subtask next time:
1. Stance and gap FIRST, before touching anything: measure the puppy's far side vs the near-wall line per y-slice (depth profile code in
   memory/PROGRESS.md / run_eed perceive). If the gap is < ~6 cm, the fingers-forward pinch has no safe palm x (fingers on the crest at
   palm <= wall-0.10, on the rim at palm >= wall-0.09). Ask the operator for a closer stance (~0.18) and/or a wider puppy-wall gap.
2. Release: the near wall is slanted in the robot frame: release at the y where it is closest (run_eed picks it) and open only at
   palm >= wall + 0.07 (run 1 opened at +0.04 and the body fell back out).
3. Grasp that lifted once: palm over the body crest, fingers perpendicular, light press, base joints .87/.86 -> lift to .27 held.
4. Safety: if a finger may be over the rim, open and raise STRAIGHT UP in small Cartesian steps; if IK refuses (branch jump), hold and
   ask. Never move the hand back or improvise joint-space raises there: that dragged the box 20 cm and tipped it (run 3).
5. End pose: home_safe (both arms tucked, hands open) - but only once the hand is clear of the box (it swept the dragged box).

## Session 20261102T122445Z_puppy_box_ad5d0f (sim): NOT solved - box directly BEHIND the puppy (head right), gap ~5 cm
Layout: box near wall x .445 (y -.35..+.06), puppy crest x .348, body y -.04..-.27, head right; stance stuck at 0.23-0.27 m from the edge.
How to plan each subtask next time:
1. Stance decides this layout. Walks did not advance (odometry ~0); g1_crouch(0, forward_rad 0.2, arms_home False) gains ~2 cm per 0.1.
   If the edge is still > 0.21 m, ask the operator to place the robot ~0.17 m from the edge BEFORE touching anything; from 0.22-0.25
   the palm stalls 5-8 cm above its press command on the plush (2/2 empty closes here, 8cef10/eed312 the same).
2. Prep with skill-library .../plush-pick-place-box-behind/code/prep_side.py (hand raised at the side), then run_eed lean/perceive/hover.
3. Hover: compute finger-link FK (code/fk.py) against the near wall; yaw cmd -65 keeps the links <= wall - 0.02 at palm cmd crest - 0.03.
4. Grasp: deep press gated on palm z <= depth top; a hold needs BOTH base joints short of the fist; raise 3 cm and look before carrying.
5. Release/home as run_eed + home_safe (open, straight up with code/up.py, then home_safe).
- [strategy] UPDATE (same session, later): SOLVED once (operator: INSIDE) with skill-library plush-pick-place-box-behind v5 `code/run_ad5.py`
  (copy code/pb_ad5/run_ad5.py). How to plan it in a new box-behind layout: perceive with the depth profile (far side per y slice, wall
  at the mid-body y; need gap >= 3 cm), palm (measured) = far - 0.096 with fingers ACROSS the body (yaw 0 for a body along y), light
  press, then a closed 3 cm raise (rolls the plush into the hand), lift 4 cm steps to z .25, carry straight forward at the grasp y to
  cmd .48, release_up.py (lean 0.2, open only at palm >= wall + 0.045, flat hand, wait, straight up). 1/3 runs: also expect a grip loss
  in the lift (1/3) - then open over the table and re-run from perceive; if the plush ends closer than far ~0.33, reset it.

## Session 20261102T130220Z_puppy_box_996d97 (sim): SOLVED 3/3 placements in 4 runs - box RIGHT, puppy centre-left head LEFT, right hand only
Layout: puppy far edge .36-.41, body y -.13..+.20, top +.021..+.028 pelvis; box x .24-.55, y -.18..-.60 (long side left-right); start edge .285.
Library skill `plush-pick-place-box-same-side` v5 (`code/run_v5.py`, copy code/puppy_box/run_996.py). How to plan each subtask in a new layout:
1. Stance: `run_v5.py stance` (tuck by the back path, 2 walks .285 -> .19). Later walks of 5 cm did not move the robot (odometry -0.6 cm);
   the robot drifts back ~5 cm and the heading ~9 deg left over 25 min: plan the attempts early, re-measure the far edge each run.
2. `run_v5.py prep,perceive,hover,shift` then LOOK at the shift frame: hand over the middle of the body (between front and hind legs).
   Check the perceive numbers: a puppy only a few cm wide or a box beyond x 0.6 = occluded / out of view -> do not grasp (run 4).
3. `grasp2,lift2`: press until the palm stalls (~2 cm above the depth top, tilt 6-7), close there 4 s, 3 cm / 4 s lift steps. If grasp2
   stops on tilt > 7: raise the open hand straight up (controller3.move to top+0.04, then +0.12), home, re-run from prep.
4. `carry,release,home`: arm-only carry at z .30 to ~(near wall + .09, box centre y); grip threshold 1.45 (middle finger alone holds it).
5. For box LEFT or BEHIND use the cross-right-hand / box-behind skills; the grasp2/lift2 changes should transfer to them (untested).
End state: robot standing at the table, arms down, hands open, tilt ~6.5 (steady); puppy on the table after reset 3 (run 4 stopped before contact).

## Session 20261102T133348Z_puppy_box_f0a010 (sim): SOLVED 2/2 runs after 1 failure - box directly BEHIND the puppy, gap ~3 cm, right hand only
Layout: puppy head right, legs to the robot, body x .155-.257 / y -.22..-.025 (head hidden behind the tucked right hand), top +.047;
clear box behind it, near wall x ~.288 (slanted .262@y-.40 -> .300@y-.025), centre (.43, -.23). Library skill `plush-pick-place-box-behind`
v6 `code/run_f0.py` (workspace copy code/f0a010/ctl.py). How to plan each subtask in a new layout:
1. [strategy] Stance: the same-side `stance` step OVERSHOT: its edge reading was the bottom of the camera view (.193) while two walks
   (odometry .126 + .112 from .296) put the robot ~0.10 m from the edge. Check walks against odometry and a far landmark (box label
   bottom via percep.plane_hit) and stop at (start edge - summed odometry) ~ 0.20. Close was still workable here (box near), but the
   reset puppy then lay right under the tucked hands and fell to the floor after reset 1 (an arm motion swept it).
2. [strategy] Perceive with no motion (run_f0 `perceive WALL [GY]`): wall from colour (rim corners / label), never from depth. If the head is
   hidden, pass GY = centre of the high body slices ~2 cm toward the head.
3. [strategy] Approach: run_f0 `hover` (FK-path-checked waypoints) worked from the side-raised pose; from the low tuck it produced an
   untrackable arm (0.6-0.9 rad error): sync the arm command to the measured joints, then `hover_cu` (cuRobo, self-collision-free).
4. [param] Grasp in a small gap: fingers forward, palm-down; the pre-curled finger links reach 0.115 m ahead of the palm and the press
   pushes the palm ~2 cm forward. Measured palm = far - 0.105, press command 1 cm further back, stop if link FK x > wall - 0.007.
   Run 1 without this put the links 2 cm past the wall and DRAGGED the box ~9 cm toward the robot.
5. [param] Lift: close at the stall (palm ~top + .015), closed raise, 3 cm / 4 s steps; with the palm 0.14 m ahead of the pelvis straight
   up stops at z ~.21 (IK), so rise diagonally (3 cm forward per step, <= wall - .07) to z .33. Paws hang lowest (z .20-.24 vs rim .10).
6. [param] Carry 4 cm / 5 s at z .33 to palm x = wall + .13; check the paws by depth (x .36-.39) and open flat; it falls inside.
7. [strategy] End pose: home_safe, then shoulders 0.6: the tucked right finger links sit only ~1 cm in front of the box wall at sp .45.

## Session 20261102T140446Z_puppy_box_e578ed (sim): SOLVED once (attempt 6 of 6 closes) - box BEHIND, puppy head right, near wall far (.39)
Layout: start edge .285 (arms tucked); puppy far side .33-.35, top +.02-.035, mid-body y ~-.13 (head hidden behind the tucked right hand); box
near wall x .38-.39 (colour floor/rim lines), y -.33..+.07. Library: plush-pick-place-box-behind v7, code/e578/ctl.py. How to plan each subtask:
1. [strategy] Stance: tuck both arms (code/e578/tuck_measure.py), walk carry=true by edge - .20, crouch(0, forward .08) after each walk, re-measure.
   [param] .09 -> 3.2 cm (first walk, warm-up), .05 -> 3.4 cm; 2 walks .285 -> ~.19.
2. [strategy] Reach before touching: code/e578/reachgrid.py on the commanded row for the grasp and a release ~wall + .11. A near wall at .39 needs
   waist yaw CMD 0.7 (measured only .17-.30) - set it in its own ramps (waist.py 0.35, 0.7) with the hand raised at the side (prep_side.py).
3. [strategy] Approach: check the measured palm against its command at every hover waypoint (ctl hover refuses at > 3 cm). If off: sync.py, then short
   goto2.py hops. cuRobo is refused at a turned waist [sim-only observed].
4. [strategy] Grasp/lift: fingers forward (yaw 0) palm far - .105, press gate top + 0, close 4 s, closed raise from the MEASURED palm, straight-up
   2 cm / 4 s lift (LIFT_DX 0) to the IK limit (~palm .22). Diagonal lifts and yaw +30 fingers lost the grip at lift-off 5/5.
5. [strategy] Carry at z .24 cmd, last cm with nudge.py (max jump .8); before opening measure the near paws by depth (pixel -> pelvis x): open only when
   they are past the wall line. [param] opened at palm wall + .075 measured -> INSIDE.
6. [strategy] End: skill-library .../plush-pick-place-box-behind/code/home_safe.py (both arms tucked, waist 0, hands open).
Run command (from the workspace root, after run_f0 perceive WALL): `PYTHONPATH=.:code/puppy_box python code/e578/ctl.py hover` -> LOOK ->
`... ctl.py grasp` -> `LIFT_DX=0 ... ctl.py lift` -> LOOK -> `... ctl.py carry` -> depth check of paws vs wall -> `... ctl.py release` -> home_safe.py.
- [strategy] Repeat test (attempt 7, reset 4, same recipe) slipped at lift-off (palm z .12); the press stalled at palm .047 (top .033) instead of
  reaching the top. Tally 1 placement in 7 closes: the GRIP at lift-off is this layout's bottleneck, not reach or release. Untested next ideas:
  stand closer (~.15 m: f0a010's grasps 0.15 ahead of the pelvis pressed in and held 2/2) by asking the operator, grasp at waist 0 where
  the arm tracks its command, and re-close when the press has not reached the top. Also: perceive with BOTH arms tucked at waist 0 before
  prep_side (the side-raised arm hides the far side at waist cmd 0.7: far read .27 instead of .33).

## Session 20261102T144058Z_puppy_box_4ddb74 (sim): SOLVED 1/1 (operator: INSIDE) - box directly BEHIND, puppy head LEFT, gap ~2.4 cm
Layout: start edge .29 (arms tucked); puppy at the front edge (paws overhang), body x .17-.35, mid-body y -.11, top +.041; box near wall
x .36-.38 (colour rim/floor lines via code/e578/rimhit.py, slanted 3 deg), centre ~(.52,-.05). Library: plush-pick-place-box-behind v8 code/pb4/.
How to plan each subtask in a new layout:
1. [strategy] Stance: the tucked fingertips (x .28, z .10-.17) would walk into a puppy lying at the edge. Walk pose = right arm DOWN
   (sp .7 then el .6, hand behind the pelvis), left kept tucked further back (sp .9 el -.5): code/pb4/walkpose.py. [param] walks .09 (warm-up)
   -> edge .29->.226; .04 -> ~0; crouch(0, fwd .1) -> .234 (slid back); .07 -> .178 (odometry .03). Reach check (code/pb4/reach.py) at .23
   said the release x .51 needs waist yaw .7 + lean .1: get to ~.18 instead.
2. [strategy] Prep: code/pb4/prep.py raises the right hand at the side (sp 1.4, roll -1.1, el -.9, wrist roll -1.3) with an FK check of every
   stage; prep_side's order (sp .9, roll -.45, el -.9) swept the fingers 2 cm in front of the overhanging paws. Then a high side waypoint
   (.02,-.32,.30) and code/pb4/hover2.py hops (run_f0's own hover IK was refused from there).
3. [strategy] Perceive: run_f0 perceive WALL GY - override GY to mid-body (its gy landed on the neck for a head-left puppy).
4. [param] Grasp: press command x .225 (palm far-.105 cmd gave a finger path to the wall); levels 0/-.015/-.03 stopped on tips_x .369 vs wall
   .371 with the palm at top+.017; closed there anyway (forward fingers flex down/back, away from the wall): base .79/.91 -> after a closed
   3 cm raise .76/.86 = held. Lift to z .33 held on the middle finger (base 1.10, index 1.54).
5. [param] Carry stopped at cmd .441 (IK); palm .375 = at the wall. release_up.py 0.2 0.42 0.48 -0.11 0.30 reached palm .465 (wall+.095), opened,
   puppy stood upright in the box leaning on the forearm; operator: INSIDE; home_safe -> it stayed inside (sitting). Tilt <= 3.3 all run.

## Session 20261102T145458Z_puppy_box_258bd8 (sim): NOT solved (0/5 closes) - box ahead-LEFT (rotated 12 deg), puppy far RIGHT head LEFT
Layout: robot ~12 deg left of square to the table, edge .27 at y 0 / .22 at y -.2 (arms tucked); box rim corners (.31,-.114) (.396,.293)
(.682,.218) (.604,-.177); puppy snout at the box's right wall (y -.17), front legs y -.32..-.36, mid-body y -.40 (x .26-.37), body to y -.52.
Library: cross-right-hand v5 (code/x2a/run_x2a.py with WAIST/GC/LIFT_STEP/LIFT_SECS env + step `lean`; code/x258 tools). How to plan next time:
1. [strategy] Stance decides this layout. Walks did not advance (4 walks, odometry ~0) [sim-only]. Before ANY press, compare the grasp's
   lateral offset with what held before: mid-body at |y| <= .30 held 2/5 (2a8660); at y -.40 here 0/5. If the mid-body is beyond ~.33 m
   to the side, ask the operator at once to put the robot 10-15 cm further right (or the puppy left); do not spend resets on press variants.
2. [strategy] Perceive: tucked look (run_x2a look) + `code/x258/edge.py` (table z, edge per y bin); pass TABLE_Z (the histogram locked onto the
   tucked left arm here) and BOX from colour corners (`code/x258/corners.py <obs> rim u,v ... floor u,v ...`). The PCA centre fell on the
   front legs when the tail was out of view: set GC to the mid-body between front and hind legs.
3. [param] Reach: offline IK (commanded row) says waist yaw .5 + roll .25 reaches a pressed grasp at y -.40 (roll is what helps; .15 misses by
   2.5 cm), standing tilt 1.4. Measured palm still rode 4-5 cm above its press command (palm .064-.079): treat IK-reachable as NOT pressable
   when the hover already sits >= 2 cm above its command.
4. [strategy] After a failed close: `code/x258/open_up.py 0.34`, run_x2a `home` (its FK check refused from z .25), `look` again - each failed
   press moved the puppy 6-8 cm (toward the edge twice, away once); re-pressing in place closed on nothing.
5. [param] The release side was never reached; plan said (.43,.02,.30) is IK-reachable at waist .5/.25 (palm 9 cm inside the near wall).
End state: robot standing, both arms tucked, hands open, tilt .6; puppy on the table (pushed back); box upright, untouched; 3 of 5 resets used.

## Session 20261102T152531Z_puppy_box_da90c5 (sim): SOLVED 2/3 runs (operator INSIDE x2) - box RIGHT, puppy centre-left head LEFT, FAR stance
Layout: start edge .30 (arms tucked); table LOW (pelvis z -.07), puppy top ~.00, far side .43-.45, body y -.10..+.23; box near wall .28-.30,
y -.16..-.55 (colour corners put the inner wall ~4 cm too far left; release y -.35 still well inside). Library: plush-pick-place-box-same-side v6
(merged 0f6d388), code/da90/ctl.py. How to plan each subtask in a new layout:
1. [sim-only] Stance: walks did not advance here (0.1/0.1/0.15/0.15/0.2 m -> odometry ~0); crouch(0, forward .2, arms_home False) gained 2-3 cm
   twice. Plan for a 0.24-0.25 m stance and check reach offline (code/da90/reachscan.py x y z) before touching anything.
2. [strategy] Do not lean to gain reach at the far stance: v5's lean fallback slid the pelvis 1.7 cm back (worse). Turn + roll the waist instead.
3. [param] Waist yaw cmd .7 + roll .15 (measured yaw .47, tilt 5.4) reached grasp (far-.10, mid-body, press to the first stall) and release
   (near wall + .145, box centre y). Yaw .9 tilted 7.2-7.6 once the robot had drifted 2 cm back: the press then stops early -> empty fist.
4. [strategy] Hover: LOOK; DY -.03 (toward the tail) put the hand between front and hind legs both times. Close at the stall (palm ~top+.03-.04),
   closed 3 cm raise, lift only if a base joint < 1.3. Two empty closes rotated the puppy: reset instead of pressing a third time.
5. [param] Lift cmd .28 -> palm .29-.35; carry arm-only to palm ~(wall+.09-.10, -.37, .34); open at once; v5 home (arms beside the hips).
6. [strategy] Walks/crouches with arms tucked let the wrist yaws creep to +-.7 (hands turning in over the puppy): code/da90/retuck.py.
Run: `PYTHONPATH=. python code/da90/ctl.py restore,prep` -> `WAIST_YAW=0.7 ... ctl.py waist` -> `DY=-0.03 ... ctl.py hover` -> LOOK ->
`... ctl.py grasp,raise,lift` -> LOOK -> `PYTHONPATH=. python skill-library/skills/plush-pick-place-box-same-side/code/run_v5.py carry,release,home`.

## Session 20261102T155840Z_puppy_box_3a46e1 (sim): ONE program for every layout - code/puppy_box/run.py (library plush-pick-place-box-any-layout v2)
New setup this session: the robot starts ~0.27 m from the edge; tuck both arms and ask the operator "bring the table closer" (the first
generic reply did not move it; the 2nd, exact request did) -> edge 0.18-0.20 (drifts back to ~0.23 over 2 h). No walking needed.
Results: development 6 placed / 9 runs; EVALUATION (memory/EVAL.md, code 90f2404) 2/8 placed (1 clean, 1 needed a manual 8 cm raise),
0 falls, box never knocked over during the evaluation (once in development, run 4). Fixes after the eval are in v9/v10 (partly untested).
Run (workspace root): `PYTHONPATH=. python code/puppy_box/run.py scene` -> LOOK overlay -> `... approach` -> LOOK hover (optional
`... shift DX DY`) -> `... pick` -> LOOK lift -> `... carry` -> LOOK -> `... place`. `... abort` from anywhere after approach.
How to plan each subtask in a new layout:
1. [strategy] Perceive with both arms DOWN behind the table edge (LOOK pose sp .9 / el 0: hands out of the image); tucked hands hide the
   plush. If the plush blob touches an image border, add a view with the waist turned toward it (cmd .7, measured ~.26) and merge the
   pelvis-frame points. [sim-only] the box rim is in depth (table + .145); real depth will not see it: use the colour fallback.
2. [strategy] Mid-body = midpoint of the two dark paw groups along the body axis; the eye (near-black blob >= 5.5 cm above the table)
   gives the head end and checks the mid (eye -> mid .20 m). The 2 cm tail shift is NOT reliable: 4 of 9 hovers were over the rear and
   needed shift +-.02-.03 toward the head at the LOOK. Next: drop TAIL_SHIFT to 0.
3. [strategy] Waist: plan() tries yaw/roll (0..0.9, 0/.15) on the COMMANDED row and keeps the one whose release is most preferred (box
   centre first). Set it once before the hand goes over the plush; LEFT arm to the guard pose (sp .9 el -.9) first (with the waist turned
   left the plain tuck put the left fingertips where a head-left plush's head hangs after the lift).
4. [param] Grasp: palm far-.10, fingers across the body, press levels top+.02..-.045 (+1 past the first stall), close 4 s, closed 3 cm
   raise and the lift from the MEASURED palm xy (the press pushes the palm ~3 cm past its command), 3 cm / 4 s steps, no sideways drift
   (2 of 3 slips came at a drift onset). Hold = middle base < 1.45.
5. [strategy] Plush orientation decides the lift: legs TOWARD the robot held 10/12; legs AWAY 0/4 (slips at z .085-.21, or the press slid
   past the body). Untested ideas: grasp nearer the leg side / fingers yawed to hook under the belly; otherwise stop and report.
6. [param] Carry arm-only <= 6 cm / 5 s at rim + .21. mv() syncs the arm command to the measured joints when they differ > .5 rad (IK
   branches like shoulder roll -2.2 are not tracked; the next step then jumps 1.6-3.4 rad and is refused - sync and resume works).
7. [param] Release: box beside the plush with the head toward a wall: the MEASURED palm must be >= .18 from that wall (placed at
   .185-.205, tipped out at .11/.12/.145). v10 nudges toward the centre before releasing (untested).
8. [strategy] Release check: open (flat shape), 10 cm up, count cream pixels near the palm (> 1500 = the plush is draped over the back of
   the palm-down hand; happened in 3 of 5 box-LEFT cross-body releases). Then lower to rim + .19, roll the wrist (+.8, FK-chosen toward
   the box) -> it slid in (1/1), up, re-check. Never go home with the plush on the hand (run 4 tipped the box that way).
9. [strategy] End pose: home = hand high at waist 0, both arms tucked (sp .45 / el -.5), hands open; operator confirms INSIDE.
10. [strategy] Box BEHIND: plan() takes the smallest waist that reaches any release (the centre-first search asked for waist .9). Untested
    end to end this session (run 10 slipped in the lift). Past sessions' leaned release (lean .2, gated on the measured palm) is not in run.py.
Open problems, in order: lift reliability (legs away 0/4, legs toward 10/12), v10 head-wall nudge untested, box-behind release reach,
real-depth box detection (rim invisible on the real robot: colour fallback only), robot drift back ~5 cm over 2 h (re-ask the operator).

## Session 20261102T175700Z_puppy_box_d65273 (sim): run.py v17.1 - EVAL 3/8 (v14.1), 5 dev placements; one program for every layout
Run (workspace root, robot standing, arms down): ask the operator to slide the table to ~0.20 m, then per layout
`PYTHONPATH=. python code/puppy_box/run.py scene` -> LOOK overlay -> `... approach` -> LOOK hover (`... shift 0 -0.015` when the hand is
over the chest, it was 9/9 times) -> `... grasp,lift` -> LOOK lift -> `... carry` -> LOOK carry -> `... place` -> ask the operator.
After a failed grasp/lift: `... abort`, then `scene` again (one retry). Library: plush-pick-place-box-any-layout v4.
How to plan each subtask in a new layout:
1. [strategy] Stance and safe poses: keep both arms in the LOOK pose (shoulder pitch .9, elbow .6: hands behind/below the table edge)
   whenever nothing is held - before resets, for perception and at the end. Tucked hands sit over the table edge: a reset put the plush
   between them and the next arm move dragged it to the floor. The scene step refuses to move arms with the plush < 6 cm from a hand.
2. [strategy] Perception: arms down, one frame; side look (waist +-.7) when the plush runs off the image or is not found. Box from the
   sim depth rim (table + .145) [sim-only]; the real robot needs the colour fallback (boxdet) - verify it first on real frames.
3. [strategy] Waist: the smallest of (0, .3, .45/0-.15, .6/.15, .7/.15) whose commanded-row IK reaches the pressed grasp AND the release
   (box centre first). Never > .7 (oscillation). Set once before the hand goes over the plush; keep it until home.
4. [param] Grasp: palm .075 behind the body's far side at the paw-pair mid-body, 1.5 cm toward the tail, fingers across the body
   (box left + grasp beyond y -.25: palm yaw -45). Hover and every press level re-aim on the MEASURED palm; press on while the palm is
   above the plush top; close 4 s; closed 3 cm raise; hold = middle base < 1.45.
5. [param] Lift 3 cm / 4 s from the measured palm to rim + .19; carry arm-only <= 6 cm / 5 s at rim + .21; release at the box centre
   (side layouts: measured palm >= 13 cm past the near wall; behind: >= 6.5-8 cm past the near wall, look at the paws first).
6. [strategy] Home: Cartesian retreat (up to z .40, back to x .24) at the current waist, THEN waist 0, then arms to the LOOK pose.
   The old single joint-space home dragged the box (with the plush) off the table at waist .6/.15.
Open problems, in order: (a) lift slips at z .14-.22 (3 of 10 attempts in the eval) - near the front edge every slip is a floor; try
lifting over the table interior / a deeper hook; (b) lateral cross-body grasp: yaw -45 is 1/2 (held at waist .45/0, empty at .7/.15 - try forcing waist .45/0 when it reaches); (c) v15/v17.2 recovery paths
(lift inward fallback, set-down of a plush stuck on the hand, near-wall x-nudge, abort hand-empty check) are untested on the robot; (d) real-depth box detection.

## Session 20261102T202144Z_puppy_box_4f16bd (sim): EVALUATION ONLY of run.py 41edb62 (v14.1) on the elliptic friction cone - 5/8 placed
No code or PARAMS changes. Same protocol as d65273 (memory/EVAL.md block 3, one row per trial). Result 5/8 (last eval 3/8 on the
pyramidal cone): box RIGHT 3/3, BEHIND 2/3, LEFT 0/2. Funnel: grasp closed 9/11 attempts -> lift held 6/9 -> carry 5/6 -> release 5/5.
What it means for planning each subtask in a new layout:
1. [strategy] Before any reset: arms in the LOOK pose (`run.py look`), not tucked. With tucked arms two resets laid the puppy's legs
   against the right hand and `scene` refused to move (correct); the scripted operator cannot move the puppy alone.
2. [sim-only] Grasp + lift + carry on the SAME side (box right of a centre puppy, legs toward the robot) is now reliable in sim
   (3/3, grip steady at .45-.80 rad): with the new contact model, earlier slips on this layout were partly a friction-cone artefact.
   On the real Dex3 expect the same or better (operator note), but measure the grip trace on the first real lifts.
3. [strategy] Box LEFT with the right hand (puppy at y ~-.28, waist .6-.7/.15): 0 of 4 closes survived a lift (2 empty, 2 lost on the
   first 3 cm). This is the layout to work on next; v17's palm yaw -45 (1/2 in d65273) was NOT part of this eval. Alternatives worth a
   test: ask the operator for a layout with the puppy nearer the centre, waist .45/0 forced, or grasp nearer the robot-side body edge.
4. [strategy] Box BEHIND: the far carry (palm to x .48-.49) drives the middle finger to the fist while the index holds; 2/3 placed, one
   dropped at the first carry step (to the floor over the front edge). Lift and carry over the table interior, and consider a carry
   height or speed change only after measuring which finger lets go.
5. [param] Home/abort: the left arm's TUCK (sp .45, el -.5) puts its fingertips at x .267 / 16 cm above the table; a box whose near wall
   is closer than ~.27 m gets hooked by the next move to LOOK and tipped off (trial 2). Go LEFT_GUARD -> LOOK directly
   (tmp/safe_home.py = abort + home without the left TUCK; used once, box stayed). Still in v17.2 (step_home line 832): fix before reuse.
6. [strategy] After an empty close, check the open hand is empty and retreat over the table before home (trial 7 dragged the plush along
   the front edge to the floor with the 41edb62 abort path; v17.2 has the hand-empty check).

## Session 20261102T212757Z_puppy_box_f20de8 (sim): run.py v18 - box LEFT solved; EVAL 4/8 clean, 5/8 placed (low table)
Run (workspace root, robot standing, arms in LOOK, table slid to ~0.20 m by the operator): `PYTHONPATH=. python code/puppy_box/run.py scene`
-> LOOK overlay -> `... approach` -> LOOK hover (`... shift 0 -0.015` when the hand is over the chest) -> `... grasp,lift` -> LOOK lift ->
`... carry` -> LOOK carry -> `... place` -> ask the operator. After a failed close/lift: `... abort`, `... scene`, once more. Library:
plush-pick-place-box-any-layout v6 (main f76e034). Code 6ab33de/e5e5be0 = v18.3/18.4 (eval ran 1fa815b = v18.2).
How to plan each subtask in a new layout:
1. [strategy] Reach margin first. For every candidate waist (yaw, roll) solve the pressed grasp on the COMMANDED row 3 cm deeper than the
   press target; a grasp reachable with no margin is not pressable (arm held off its command, weak close). Waist roll .25-.35 lowers the
   right shoulder for a puppy on the robot's right; nothing helps a puppy left of centre on a low table from 0.20 m - stop and ask.
2. [strategy] Grasp waist and release waist are separate (box LEFT): grasp at the first waist with margin, lift straight up, rise to
   carry height, then turn the waist in 3 slow ramps with arm IK holding the palm fixed (`swing()`), then the arm-only carry. Keep the
   palm yaw constant from grasp to release (choose the release yaw nearest the grasp yaw).
3. [param] Grasp: palm .075 behind the far side at the paw-pair mid-body, fingers across the body (axis + 10 deg), press until the palm
   is at/below the plush top, close 4 s, closed 3 cm raise; hold = middle base < 1.45 (held grips read .5-1.0).
4. [param] Carry the palm at rim + .27 (dangling plush clears the wall), <= 6 cm / 5 s steps; re-sync the arm only on a palm error > 4 cm.
5. [strategy] Release only with the measured palm over the box interior and near the planned height (z <= release + .04); box RIGHT
   releases at a turned-left waist are blind (box out of the camera) - trust the measured palm, and step down before opening.
6. [strategy] Home: retreat up/back at the current waist, waist 0, right arm high -> sp .8, then LOOK for both arms. Never through TUCK
   (fingertips x .275 hook a near box).
7. [strategy] Box BEHIND far wall (near wall >= .40): the last carry step is out of reach at carry height and the middle finger lets go
   when it is lowered + re-synced; next: descend before the last step or release at the reachable x if the paws are past the wall.
Open problems: far puppies (far side > ~.43) close empty and get pushed further; box-right centre-left puppies on a low table need
waist .6-.7/.15 with no margin (1 unreachable, 1 grasped fine); real-depth box detection (rim invisible) still only has the colour fallback.
Post-eval additions (v18.5-18.8, dev runs 9-11 all placed): carry height reachable at the release chosen before the carry (behind far
release), box rim extents from the dense run of 2 cm bins (a thin strip at rim height made a 0.72 m box), `to_look` returns an arm that
cannot reach LOOK (drawer unit beside the robot) to its guard pose, hand-clearance gate ignores the raised hand's own pixels. Controller
HEAD 80b8524 (v18.8), library main 4035a67.
8. [strategy] End pose / reset pose: LOOK (arms down behind the edge) when it is free; if `to_look` reports an arm blocked, that arm stays
   in guard (up/back near the chest). Ask for resets with the arms in LOOK; a guard-pose hand over the front edge attracts the reset puppy.

## Session 20261102T235110Z_puppy_box_3b3f99 (sim): EVALUATION ONLY of run.py v18.8 (80b8524) - 7/8 clean
Run exactly as the f20de8 section above (arms in LOOK, table slid to ~0.20 m, `run.py scene` -> approach -> grasp,lift -> carry -> place,
LOOK at each overlay). Library main 604b743 (plush-pick-place-box-any-layout with this eval's 8 evidence rows). memory/EVAL.md block 5.
1. [strategy] Trust the program on box RIGHT and standard box LEFT/BEHIND layouts: this eval needed no shift and no retry; the hover
   overlay showed the hand centred between the leg pairs every time it was visible (box-LEFT hover and box-RIGHT carries are blind).
2. [strategy] Watch for a puppy lying DIAGONALLY with its head toward the robot (scene reports head=None): plan a release with margin, and
   after the release do not go home until the hand is confirmed empty - the near-hand pixel gate misses a plush riding on top of the hand.
3. [param] Release margins that worked: box RIGHT palm at the box centre (near wall + .12), BEHIND near wall + .080, LEFT palm .14 inside the
   near wall at frac .58; carry heights rim + .21-.27 as v18.8 chooses them.
4. [strategy] Still untested by v18.8 evals: LOW table (top ~.022) with a centre-left box-RIGHT puppy, far puppies (far side > .43), legs-away puppies.
