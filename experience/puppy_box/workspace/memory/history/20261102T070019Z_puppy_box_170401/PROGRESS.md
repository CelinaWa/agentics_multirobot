# PROGRESS - session 20261102T070019Z_puppy_box_170401 (sim, 1 h) - DONE
Layout: puppy centre-left head LEFT, legs to robot; clear box RIGHT. Skill plush-pick-place-box-same-side (now VERIFIED, merged b9c195f).
Runs: 3/3 SUCCESS (run 1 steps + one look-shift; runs 2-3 fully unattended). Resets used 2/5. No falls, box untouched.
Key facts: after g1_walk the legs straighten (pelvis .80) -> g1_crouch(0, fwd .05, arms_home False) before perceive/hover.
Final state: robot home, hands open, tilt ~3 deg, puppy in the box (operator confirmed).
Open: only one layout tested this session; left-hand-contact question not answered by the operator (left arm tucked in all frames).
Note: `pgrep -f <pattern>` in a wait loop matches its own shell: wait on the background task notification instead.
