# PROGRESS (session 20261102T155840Z_puppy_box_3a46e1, sim)
Started 15:58 UTC; 3 h budget -> EVAL must start by ~17:55 at the latest (aim ~17:30).
## Current step
Unified controller code/puppy_box/run.py written (scene, approach, shift, grasp, lift, carry, release, home, verify, abort).
Runs: 1 R ok, 2 R ok, 3 L ok, 4 L FAIL (box tipped; fixed), 5 L ok after a retry. 4 resets used of 20.
Next: one 'behind' layout if a reset gives it, library skill draft + merge, then EVAL (>= 8 trials) with
  run.py pick -> LOOK -> carry -> LOOK -> place.
## Setup facts this session
- Start edge ~0.27 (tucked); request_operator "bring the table closer" (2nd ask, exact phrase) -> edge 0.194-0.203.
- Tucked right fingers sit ~1 cm above the box rim when the box is on the right: first move = shoulder pitch back (0.9).
- Paw blobs (dark brown) give mid-body = midpoint of front/hind paw pairs; worked run 1.
- Box rim clipped by the image's right border -> extend to the known 0.39/0.28 size (run.py does it).
## Run command
PYTHONPATH=. python code/puppy_box/run.py scene ; ... approach (LOOK) ; [shift DX DY] ; grasp (LOOK) ; lift,carry (LOOK) ; release,home,verify
## Incidents
- Run 4: untracked IK branch (cmd vs measured arm 2.2 rad apart) -> carry refused; after sync the plush stayed on the open hand
  and home tipped the box. Fixes in run.py v4: mv() syncs the ARM (never legs) when |meas-cmd| > 0.5 rad; release checks
  cream pixels near the raised palm (replay: stuck 15165 vs dropped 0-41, HANG_PX 1500), wiggle once, else stop before home.

## Status at 17:53 UTC (session end approaching)
- EVAL done (memory/EVAL.md): 2/8 placed with code 90f2404. Post-eval v9 (release at box centre, wrist-roll dump) and v10 (measured
  head-wall clearance nudge, untested) committed in code/ (git). Run 9 with v9 failed (tipped out at 14.5 cm) -> v10.
- Library: plush-pick-place-box-any-layout v1 merged (4a69d79); v2 with eval results to merge now.
- Next session: test v10 on box-right head-left layouts; legs-away grasp (0/4) is the biggest open problem; drop TAIL_SHIFT.
- 17:56 run 10 (v10 + behind smallest-waist fix): slip at z .195 on a legs-toward plush (box behind). Session ends with the robot home,
  arms tucked, hands open; puppy on the table; box upright. Resets used: 17 of 20. Final pose: both arms tucked (sp .45 el -.5), hands open, tilt 0.6, frame 20261102175602737b70.
