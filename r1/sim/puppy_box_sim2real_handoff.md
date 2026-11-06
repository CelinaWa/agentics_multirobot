# R1 Puppy-Box — Sim → Real Handoff

Purpose: the real R1 agent keeps exploring; this is the **prior** it starts from — what sim validated, the
boundary sim found, and the first-real-run checklist. (Full diagnosis: memory `r1-g1-embodiment-transfer.md`.)

## Validated priors (start from these)
- **Grasp = EDGE-OVERHANG.** A flat-lying leg can't be pinched (lower jaw hits the table). The operator nudges a
  hind leg to hang ~6 cm past the front edge; grip the **hanging** leg from the front, jaws straddling it (lower jaw
  in the open space beneath).
- **Pads must be GRIPPY.** Sim needed gripper friction 1.5 → 3.0 to match the real TPU pads (from the user's photo).
  **Biggest sim assumption — verify real pad grip first.** With flat/slippery pads the leg squeezes out on lift.
- **Control = 3-point teleop (SONIC_R1).** Command the wrist; the policy balances and solves the arm. cuRobo is the
  wrong lever here (5-DOF, open-space grasp). It earns its keep on G1 (7-DOF, clutter).
- **Scene must be stable before grasping.** A free table that drifts even ~mm after the operator push shifts the
  edge and breaks the grasp (this was a real bug — pin/settle the scene, then act; don't grasp mid-motion).
- **Reliability = adaptive retry + operator reposition**, not single-shot (the task allows 5 resets).

## The boundary sim found (what real sensing must solve)
- The SONIC teleop policy places the wrist **~4 cm off and saturates** (closed-loop can't push closer — measured),
  and the open-loop grasp outcome is **sub-millimeter sensitive** (a 1 mm context difference flips hold/miss).
- Therefore **autonomous single-shot thin-leg grasp is not reliable** on R1's 2-jaw gripper (~3.7 cm half-window).
  The standalone sim 21/21 is a knife's-edge result (ground-truth positions + one lucky trajectory) — it does not
  transfer (2/5 through the live server) and would not survive real cm-scale noise.
- **Real-world precision must come from: perception (camera/depth leg localization) + feedback (servo toward the
  perceived leg) + operator alignment + retry.** Close the last cm with sensing, not open-loop.

## First real-run checklist
1. **Verify real pad grip** holds a leg (the one sim assumption from the photo).
2. **Stand up camera → depth leg localization** (sandbox GL renderer couldn't; real has depth). Measure its error.
3. Run the loop: observe → localize leg → walk until `r1_move_hand` to a point above the table returns a small
   error (reachable standoff) → request operator overhang → grasp toward the **perceived** leg → verify held →
   if not, operator reposition + retry (≤5) → place in box → verify.
4. **Re-calibrate sim-tuned numbers on hardware**: standoff ~0.28 m, the settle duration, the torso lean.

## Failure modes to watch (seen in sim)
- Legs clustered / far to one side → operator reposition to present a clean leg.
- Gripper closes on nothing (jaws off the leg) → the precision/perception gap → retry / reposition.
- Leg squeezes out on lift → pad grip insufficient → check real friction.

## Pointers
- Validated sim controller: `tools/scratch/diag/adaptive_solve.sh` (+ `full_pipe_oh_ft.py`, `flat_pipe_ft.py`) on
  the grippy-pad copies `tools/scratch/scene_test_ft/`.
- Grasp core (shared): `sim/oh_grasp_core.py`; pin-fix in `sim/r1_sim_server.py` `edge_grasp` (+ ZMQ op).
- Live-stack adaptive loop (reference): `tools/scratch/diag/verify_retry.sh` + `attempt.py`.
- Benchmark `sim/scenes/*` are root-owned; the friction-3.0 fix lives only on the private copies (apply to the
  benchmark needs permission).

## Perception note (sim camera/depth — for the real agent)
- **Headless rendering works with `MUJOCO_GL=osmesa`** (the sim `camera` op fails without a GL backend — that was a
  `KeyError: 'height'` from the op erroring server-side; set `MUJOCO_GL=osmesa` to render depth headless).
- **Depth leg-localization is a dev task, not done here.** A first pass (`tools/diag/perceive_leg.py`) returned 0 leg
  points — the depth→point-cloud→pelvis back-projection (camera axes), table-plane fit, and hanging-leg segmentation
  need building/tuning. The **real agent does this via `r1_observe`** with real depth; treat perceive_leg.py as a stub.
- Implication: the sim grasp numbers used **ground-truth** leg positions; real perception adds cm-scale error on top
  of the policy's ~4 cm, which is why the real loop must close the last cm with feedback + operator assist (above).

## Quantified boundary (measured this session — the error budget)
Measured on the live stack (full_pipe_oh_ft, lean, seeds 16/21/22/25/27):
- **Policy reach residual ~6 cm.** After the closed-loop servo (the grasp ALREADY servos the wrist toward the leg
  using GT position + FK), the gripper saturates ~5.6-6.3 cm short of the leg, almost all in +x (forward).
- **Not fixable by operator distance.** Sweeping `reach_ahead` 0.20-0.28 m left the residual ~6 cm or worse
  (seed25 got worse: 5.8->8.2 cm); seed27 succeeded at 0.28 and failed at 0.24. Success is a per-seed parameter
  coincidence (the blind forward-advance happening to close the gap), not a robust grasp.
- **Perception error 9-31 cm** (first-pass depth localizer, head pitched to 0.55 rad). Crucially, even a PERFECT
  localizer = GT = 0 cm, which the grasp already uses and still saturates. So:
      perception err (9-31 cm)  >>  policy residual (~6 cm)  >  grasp window (~3.7 cm).
  **Perception cannot fix the grasp** — it only adds error on top of the GT the servo already has.
- **Conclusion:** the bottleneck is the frozen SONIC_R1 policy's ~6 cm wrist under-reach vs the 2-jaw window.
  No sim-side lever (perception, operator distance, closed-loop servo, lean) crosses it reliably. The real-world
  route (operator physically aligns the leg to the gripper + real contact feedback + retry) is the only path,
  and sim has now proven *why* (quantified), not just asserted it.

## New sim capability added this session (infra the real perception pipeline needs)
- **`head` ZMQ op** on the sim server: command head pitch (rad, + = down), held like the real `r1_head`. The sim
  was missing head control; without it the camera stares at the far wall.
- **Headless depth render** confirmed via `MUJOCO_GL=osmesa`. At head pitch ~0.55 rad the grasp workspace
  (leg at x~0.27 m) comes into view (at pitch 0 the nearest view is x~0.56 m, past the leg).
- `tools/diag/perceive_leg.py` now runs end-to-end (pitch -> depth -> point cloud -> leg localize -> error vs GT).

## Backward walking (found by the first RAX agent run, 2026-11-05)
The agent discovered `r1_walk` with negative `forward_m` did not move the robot back (drifted forward/right) and
ended up 0.08 m from the table. Cause: the sim's scripted gait was forward-only; "backward" only reversed the
reference root. Fix: flip the hip-swing sign for backward (`walk_sign`), which sets the gait's propulsion
direction — backward now walks ~1.0-1.25x the commanded distance (forward ~0.7x). Time-reversing the gait phase
did NOT work (the policy does not step back from it). Sim-only detail: the real stack commands locomotion via the
deploy's planner `movement` vector; on the real R1 verify backward walking separately.

## First RAX agent-loop exploration (2026-11-05, session 20261105T194732Z_puppy_box_r1_1d45ef)
R1 was explored through the real RAX workflow (r1sonic backend + agent loop, seeded from G1's puppy_box via
cross-embodiment). Result: **0/6 placements**; funnel approach 4/6 -> grasp held 1/3 -> carry 0 -> place 0; 1 fall,
puppy off the table 2x; robot ended safe. The RAX-native experience is in `experience/puppy_box_r1/`
(RUNS/TIPS/PARAMS/handoff, TRANSFER_REPORT: 8 strategy / 3 param / 1 sim-only, 10 params measured in sim).
Key findings the agent loop surfaced that scripted runs never did: `r1_grasp_paw` (flat grasp) is the wrong tool for
the hanging leg and its reach-lean tipped the robot over at paw 0.38 m; the hold slips within ~1 min if the carry is
delayed; reaching shifts the body 6-12 cm (whole-body coupling); walk/turn calibration is wide (0.25 cmd -> 0.11-0.35).
Harness fixes queued: auto-arm sim sessions (lost 22 min), a scripted operator for reset/overhang (help requests
time out), writable agent workspace (memory landed in /tmp and had to be synced by hand), wire `edge_grasp` into
the backend / drop the misleading `grasp_paw` tool, and `--environment sim` must be passed at configure time.

## RAX session 2 (2026-11-05, 20261105T213131Z_puppy_box_r1_08803f, seeded from session 1)
Still 0 placements, but the funnel moved: approach 7/7; **from-the-front paw pinch held 4/4** (widths .032-.035 =
on the leg core; `grasp_paw` 1/3 + 1 fall, now abandoned by the agent); lift to rim height 1/5; **carried the puppy
0.15 m and 0.08 m while walking** (new - needs the walk fix). Bottleneck now: the pinch is friction-limited and
slips once the puppy's weight hangs from it (z 0.12-0.29), and the carry over the box rim. The agent built a
continuous-IK lift planner (code/tools/contik.py) after an IK configuration flip dropped the puppy, and drafted
the skill `r1-paw-pinch-grasp` (merged into the r1sonic library). Bugs it found, fixed here: the backend kept the
stored gripper command at "open" after `grasp_paw`, so the next motion re-opened the hand and dropped a carried
puppy in front of the box (attempt 4 would likely have placed it). Harness: sim sessions now auto-arm and get
answers in seconds (rax/scripts/r1_sim_operator.py); the agent workspace is still mounted read-only (synced by hand).

## RAX session 3 (2026-11-05, 20261105T222431Z_puppy_box_r1_e0ac0c, seeded from session 2; 10-reset sim budget)
0 placements, **0 falls** (4 of 10 resets used; the agent ended early on its own). Funnel: approach 5/5; vertical pinch
2/2 held but slips under load (z ~0.04); **horizontal "pivot pinch" on the lower front paw 3/3 held and lifted the
puppy to z 0.27 with the width steady** (pads left/right of the paw ball, so the hanging weight pulls straight down
on the pads) - the lift problem is solved in principle. Carry-walk 0/1 (a 0.19 m walk shook the leg out).
**The blocker it found: the paw pair.** Lying on its side, the two legs of a pair sit ~5 cm apart and touching, so a
finger cannot get between them; the pivot pinch only works once the other paw is >= 3 cm further back (happened
once by accident; deliberate pushes spring back to ~2.3 cm). Skill `r1-paw-pivot-pinch` merged (b8569a9).
Other measured facts: forward walks drift 2-6 cm right; reaching right of y > -0.08 hits the shoulder limit (1.5-3.5
cm off); the pelvis moves ~2 cm back when the arm extends; an IK wrist flip can drop a held paw (its safe_move
guard now refuses flips). Handoff plan: separate the paws >= 3 cm, pivot-pinch, lift along the recorded
waypoints, carry in <= 0.1 m walks checking the grip, lower over the box, release.

## RAX session 4 (2026-11-05, sim) and the grasp-depth measurement
- Session 4 (`20261105T232821Z_puppy_box_r1_c4f6c3`): 0 placements, 5/10 resets, 0 falls. The agent's pivot pinch
  held 3/3 in its proven stance and lifted the puppy to a full hang (hand z 0.285), but every walk toward the box
  dropped it, and the box-behind layout put the box interior 12-15 cm beyond no-walk reach. New skills:
  `r1-walk-approach` (turn-arc walk model + planner) and `r1-paw-pivot-pinch` v2.
- Engineering measurement afterwards (private sim, the agent's own `r1_move_hand`/`r1_gripper`/`r1_walk` path,
  contact forces logged): the clamp is 5-6 N per jaw at friction 3.0, so friction is not the problem. The grasp site
  sits only ~2 cm from the fingertip end of the pads, so a pinch centred on the paw ball lets the leg pull out after
  ~2 cm of slip, and a straight vertical lift forces that slip (the leg pivots at the hip 7 cm away).
  Grasping ~3 cm up the shaft and lifting along the arc about the hip: lifts held 4/5 (vs ~50% before) and the
  held puppy survived three `r1_walk` calls, including the hand passing over the table edge, with the width
  unchanged (0.031 m). The held puppy's body hangs only ~15 cm below the hand (the hip joint locks), so a hand
  ~0.30 m above the table clears the 14 cm rim. These facts are now in the task's gripper note for the agent.
- **Real-robot check:** the 2 cm pad margin is a property of the model's grasp site vs the PiPER pads: confirm on
  the hardware where the real pads end relative to the commanded grasp point, and that a real plush leg (soft,
  longer) behaves like the sim's 7 cm rigid capsule only loosely - the arc lift should matter less with fabric.
- Sim/harness fixes from this round: a puppy limb arranged by the operator kept its spring rest across resets and
  could prop the re-placed puppy up like a kickstand (fixed: `reset`/`reset_scene` clear the puppy's joints);
  sessions now live outside the protected `rax/` checkout (`rax/runtime` is a link to `~/rax-runtime-r1`), so the
  agent's workspace is writable and its skill-library merges work without the manual patch dance.
