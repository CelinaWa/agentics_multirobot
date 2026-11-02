# PROGRESS - session 20261102T111240Z_puppy_box_2a8660 (sim) - FINAL
Status: SOLVED 2/5 runs (runs 1 and 5, operator: INSIDE); runs 2-4 failed safely (lift slip, empty fist, carry slip).
End state: robot standing, both arms tucked (sp .45 el -.5), hands open, tilt 0.5; puppy in the box (run 5), box in place.
Resets used: 3 of 5. No falls, max tilt 6.3, box never touched, left hand stayed tucked.
Controller: code/puppy_box/run_x2a.py (library: plush-pick-place-box-cross-right-hand v4 code/x2a/run_x2a.py, merged 597f456).
Run: PYTHONPATH=. python code/puppy_box/run_x2a.py stance,look,plan | prep,waist,over,hover,grasp,lift | carry,swing [| adjust] | release,home
(BOX=x0,x1,y0,y1 from colour corners, GYAW=-30 used in runs 4-5). Details: memory/RUNS.md, handoff/README.md last section.
Open problems: grasp/lift/carry hold (3 of 5 lifts held, 2 of those carried); release check is the agent's look + depth points.
