# PROGRESS - session 20261106T060138Z_puppy_box_r1_2fbfb7 (sim), 2 h wall clock

## Current step (updated ~76 min in; see bottom for final state)
FIRST FULL SUCCESS at stance 9 (operator ground truth: puppy INSIDE the box; robot backed off, arms home, open).
Next: repeat the st9 recipe on more stances (count k/n), then >= 2 full walk-in chains from resets, then skill
library (new skill) + handoff.

## Working recipe (st9)
python code/puppy_box/home.py 0.6            # stale command after every operator placement
python code/puppy_box/chain.py look          # paw sphere fits, front edge, leg axis, box
python code/puppy_box/chain.py pre           # waypoint + pregrasp 10 cm short along the leg axis
python code/puppy_box/v4.py go 0.06 -0.004 0.06 4 0 1 5 1
   = yaw probe (close at leg yaw +0/+5/+10 deg, keep the narrowest width), grasp 6 cm past the paw centre,
     chain.py LIFT profile (11 servo-off steps to cmd z ~.42, approach yaw rotated to 2 deg, tilt .5),
     NO look/raise, walk cmd .07 at once until body centroid (hand x + .06) is .06 past the box front wall
     (dead reckoning on walked_m, <= 4 walks, edge kept >= .12 ahead), open.
python code/puppy_box/v4.py out 0.2          # open, up, out (.14,-.40,.14), walk back (repeats if short), HOME both
Then ASK the operator "is the puppy inside the box?" (ground truth; the clear box fools the camera AND depth).

## Key findings this session
- Engineering low lift (cmd z .265, leg yaw kept, tilt .5): walk 1 slipped 4/4 (st1, st3, st6, st7), even with a
  clean .034 grasp (st7). The chain.py lift (z .42, yaw -> 2 deg) held 3 walks in st9.
- Raising the hanging body after the lift slipped (st5); pauses/looks while holding let the width creep (st8).
- Walking BACKWARD while holding held 1/1 (st8) -> the drops are not caused by the box wall alone.
- Depth/image judgement of "in the box" was WRONG twice (puppy on the table in front of the box looked bridged on
  the box). Use the operator's ground truth ("is the puppy inside the box?" answered in 1-2 s, nothing moved).
- Yaw probe: leg yaw +5 deg gave .032-.035 (clean) vs .035-.038 at the measured yaw (st7, st8, st9).
- Retreat: walk back can come up short (.031 for -0.2) -> never HOME before the walk back is confirmed.

## Budget
resets used 0/10, stance placements 9/20, falls 0.

## Results
memory/RUNS.md (session 2fbfb7 section).

## Final state (~76 min)
- 3 operator-confirmed placements: st9 (probe + chain lift, past .06), st15 (past .045 + chain lift + LOWER .06),
  st18 (past .045, slip during an extra walk over the box). Each ended with the robot backed off, arms HOME, open.
- Best recipe (v5): `bash code/puppy_box/attempt.sh <tag>` then `python code/puppy_box/v4.py out 0.2`, then ask the
  operator "is the puppy inside the box?". 2/5 inside + 1 on the rim with it (st14-st18).
- Reset walk-ins: R1 rotated stance (released in front of the wall), R2 no stance, R3 walkin v3 reached (.356,-.175)
  in 2 walks but the falling-width grip slipped at the lowering, 8 cm short of the wall; R4 overshot the stance by
  one walk (paw .372 -> .27) and was abandoned. 0/4 placements from resets.
- st19-st21: lift-abort rule (exit 4) prevented drops 3/3 but the re-grasp after a put-back never produced a clean
  overhang again (0/4) -> next session: ask the operator for the overhang before a re-grasp.
- Budget at the end: resets 4/10, stance placements 21/25, falls 0. Robot left ~.6 m from the edge, arms HOME, open.
- Skill library: r1-leg-axis-grasp-short-carry v3 merged (13a3aba), v4 merged (edd9167 .. 5d44efb).
