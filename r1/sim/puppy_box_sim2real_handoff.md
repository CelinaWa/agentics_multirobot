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
