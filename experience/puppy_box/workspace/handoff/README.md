# Handoff: puppy_box (G1 + Dex3 under SONIC), session 20261101T012043Z (sim rehearsal, 6th session)

Result: NOT solved, no fall, nothing touched. The task was not attempted because of the LAYOUT: puppy middle at
pelvis (0.51, +0.31) (left, head pointing right toward the box), clear box x 0.29-0.57, y -0.17..-0.56 (right).
Neither arm can reach both the puppy and the box from this stance (library place-reach-check v2 span mode: left arm
23 cm short of the box interior, right arm 23 cm short of the puppy). Walking/turning is forbidden unless the operator
says so; the scripted operator answered three explicit requests ("move the puppy", "allow a turn") with the same
generic "approved, go ahead ... the dog is 0.6x m from your pelvis", and nothing moved.
Earlier handoffs: README_session_20261101T011122Z.md (5th, fall), ..._004902Z.md, ..._002411Z.md, ..._234111Z.md, ..._211606Z.md.

## Order of work in a new layout (do this first, it costs 2 minutes)
1. request_operator(help) to arm -> g1_hold -> observe. Heightmap: `cd code/pp && python3 scene.py <frame_id>`
   (rows = x, columns 1.5 cm from y +0.6 downward, digits = height above table / 2 cm; box rim shows as 7).
   Deproject puppy middle, box label/rim with g1_pixel_to_position.
2. SPAN CHECK before anything else: `python3 code/pb6/span_run.py` (edit grasp/place targets) or
   place-reach-check `main(ctx, side, target=place, grasp_target=grasp)`. Only if ok for the puppy-side arm,
   continue. Only leans (0,0) and (0.2,0.2) count as executable (0.3 + far reach fell last session).
3. If no arm spans: ask the operator once, concretely; if the reply is generic, do not walk on it.
   Untested option for a future session where walking is explicitly allowed: grasp with the puppy-side hand,
   then g1_walk(turn_deg toward the box, carry=true) - but a turn steps 0.1-0.2 m forward and the table edge was
   only ~0.27 m ahead of the pelvis here.

## What was verified here
- Left palm-down fingers-forward hover (palm_quat(0,'left')) at (0.32, 0.25, 0.18) with palm-pose-servo main:
  2 iterations, 2.5 cm error, tilt 1.5; unfold('left') and retract_home('left') worked (tilt 2.4 / 3.0).
- After this one arm motion the scene measured ~4 cm CLOSER (table z -0.052 -> -0.067): the robot crept forward,
  not back as in earlier sessions. Re-measure after every motion.
- Left palm-down yaw -90/-60 (fingers across the legs, the orientation that would wrap the leg bundles that lie
  along x) is NOT reachable at x >= 0.33 without a lean (IK 4-12 cm short); only fingers-forward reaches low.

## Grasp (still UNSOLVED after 6 sessions) - see README_session_20261101T011122Z.md for the leg-bundle geometry.
