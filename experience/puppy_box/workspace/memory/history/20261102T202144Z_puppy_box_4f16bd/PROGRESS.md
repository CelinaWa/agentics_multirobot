# PROGRESS - session 20261102T202144Z_puppy_box_4f16bd (sim), evaluation only
Task: re-evaluate run.py at code commit 41edb62 (v14.1) on the new sim contact model (elliptic friction cone), 8 trials,
same protocol as EVAL.md block 2 (d65273). No code/param changes. Code checked out: `git -C code checkout 41edb62 -- .`
## State (21:20) - EVALUATION DONE
- memory/EVAL.md block 3: 5/8 placed (trials 1, 3, 4 retry, 6, 8); fails: 2 (box-left lift + box tipped by home's left TUCK),
  5 (behind, carry slip -> floor), 7 (box-left 2 empty closes, plush dragged to the floor on the way home; I used tmp/safe_home.py
  instead of abort to avoid the trial-2 box hazard).
- TIPS.md and handoff/README.md updated (tagged rows). Resets used 10 of 12.
- Robot standing in the LOOK pose, hands open, puppy inside the box (trial 8).
- After the eval: code working tree restored to its HEAD (f8db6cd, v17.2) so the next session starts from the latest program.
## Next (for a development session)
- Fix home's left TUCK (go guard -> look) in v17.2; box-left lateral grasp; box-behind carry (middle finger to fist).
- 21:24 skill library merged (main 989f549: any-layout eval evidence). Session done.
