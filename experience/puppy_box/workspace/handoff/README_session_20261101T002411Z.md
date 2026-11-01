# Handoff — puppy_box (G1 + Dex3 under SONIC), session 20261101T002411Z (sim rehearsal)

Result: NOT solved (3rd session). Box out of reach from the start stance; two grasp attempts (palm-down hand over the neck; thumb low in front of the head) failed.
Robot left standing, arms home, hands open, puppy on the table, box untouched.
Earlier handoffs: README_session_20261031T234111Z.md (grasp details), README_session_20261031T211606Z.md.
Library: `place-reach-check` (draft, new) and `hand-claw-pregrasp` v2 (adds code/palm_down.py).

## Order of work in a new layout (do it in this order)
1. Start: if g1_status shows "console start failed", request_operator(help) -> armed; then g1_hold.
2. Find the box FIRST. It may be outside the camera view. Waist roll +0.4 (2.5 s) swings the view left
   (negative right); for more, a waist yaw command 1.3 gives ~0.6 rad and a wide view, but undo it at once and
   re-measure (it contorts the arms/hips and shifts the pelvis frame). Deproject the box FLOOR corners in the
   colour image (white panel through the clear walls) with g1_pixel_to_position; rim/label pixels give
   table+0.14.
3. Run `place-reach-check` main(side, target=interior point 8 cm inside the walls, z = rim + ~0.10). If not ok,
   the place is impossible from this stance: stop and tell the operator (the scripted operator only replies
   "approved, go ahead"; it moved nothing). Do not spend grasps first. Do not walk at a table whose edge is
   0.28 m ahead with the pelvis at table height.
   Reference: left arm, lean pitch 0.4 / roll 0.45 reaches y ~0.50 at x 0.40 (z 0.25); y 0.60 misses by 6 cm.
4. Grasp (still UNSOLVED, 4 approaches failed over 3 sessions): claws (tilt 0.52/0.79), yawed claw, tail claw
   (held 5 cm, slipped at 13 cm), palm-down hand over the neck (stalls on top, fist on lift). The Dex3's curled
   fingers reach only ~5-8 cm below the palm while the plush is 11 cm tall and ~20 cm across the torso: any
   top-down grip pinches above the equator. Untried ideas: (a) approach from the side with the palm vertical
   (wrist roll ~0) and fingers under the far side at table level, thumb on top; (b) push the plush against the
   box wall / a second support and lift; (c) two hands (rules here allowed only one).
5. Positioning that works on SONIC: `palm_down.go` (wrist roll 1.3 fixed, 4 proximal joints, bias clip 0.9):
   1-2 cm in free space. The claw IK branch (elbow 1.4, wrist roll 1.9) does NOT track: avoid it.
6. End pose: `hand-claw-pregrasp/code/run.py: retract_home` + waist 0 + right arm (sp 0.6, el 0.6): verified.
