# R1 puppy-box: transfer from simulation to the real robot

Written 2026-11-06 after 11 RAX simulation sessions (12 ground-truth-confirmed placements, 0 falls). This is the plan
for taking that work to the real Unitree R1 + PiPER grippers under SONIC_R1 teleop, and the reasoning behind it.

## 1. What we are transferring, and what we are not

Transfer **code and principles**, never simulation numbers as facts.

| Transfers | How |
|---|---|
| The harness, unchanged | same `rax/` launcher, robot service, operator protocol, arming, memory/skill formats; all its bugs were found in sim |
| Two robot-service primitives | `r1_leg_lift(paw_position, leg_direction)`, `r1_carry_release(target_forward_m, advance_m, lower_m)` in `rax/services/r1sonic/backend.py`: the measured grasp-lift-carry chain, built only from the agent-facing hand/gripper/walk commands with the agent's perceived inputs |
| The agent's verified skill | `r1-leg-lowtilt-lift-advance-carry` (library head): its own chain, 7/7 completed lifts ended in the box in session 11, the rest safe aborts |
| Principles (as `[strategy]` rows) | edge-overhang presentation; grasp the leg shaft along its measured axis, fingers across it; width 0.031-0.036 = clean shaft, >= 0.040 = second leg or diagonal; a falling width during the lift = sliding out, abort and re-grasp; low-tilt arc lift; no arm/waist moves under load; carry in short steps with no pause; lower before release; verify the placement, never trust a camera view through the clear walls |
| Numbers (as `[param]` rows, `PARAMS.json`) | 76 sim-measured values become PRIORS the real agent must re-measure (lift height 0.265, step 0.07, advance 0.06, lower 0.06, stance window paw x 0.29-0.33 ...) |

| Does not transfer | Why |
|---|---|
| Everything tagged `[sim-only]` (20 rows after the transfer retag) | this simulator's scripted gait (random short steps, right veer, turn arcs), its soft-contact creep, its operator services (stance accelerator, box moves, ground-truth answers), harness bugs since fixed |
| The sim server, scripted operator, sim task file, evidence videos | `r1/sim/r1_sim_server.py`, `rax/scripts/r1_sim_operator.py`, `tasks/puppy_box_r1_sim.md`, `r1/sim/evidence/` |

RAX does the split mechanically: inheriting the export into a `--environment real` session moves `[sim-only]` rows aside
and writes `memory/TRANSFER.md` (what is unverified here, what to measure first). `sim_to_real.py lint` on the export
reports the tag counts; it is clean (0 untagged).

## 2. Why this way, and what we would do differently next time

The structure is right: one harness, one tool set, one memory format on both sides, so what the agent learned is in
the form the real session reads, and the operator loop is the same loop a human will run. Two lessons for next time:

1. **Simulator fidelity mattered more than session count.** More than half of the sim time went to artifacts of our
   reimplemented stack (walking scatter, contact creep, kickstand resets). The better investment is to run the real
   deployment stack itself in simulation (`gear_sonic_deploy`'s `r1_sim_cpu` with the zmq_manager protocol): walking
   and control then behave as on the robot, and the bridge below gets written and tested before the robot day.
2. **Prune at transfer time, do not ship raw history.** 11 sessions of notes contain many sim-flavoured conclusions; the
   tags make dropping them mechanical, but someone has to retag rows that turned out to be artifacts (done here).

## 3. The one new component: the stack bridge

The backend expects `stack.py`'s JSON-over-ZMQ protocol (`state`, `stream`, `hold`, `walk`). The real deployment speaks
zmq_manager (topic + 1280-byte JSON header + packed little-endian payload). `rax/services/r1sonic/bridge_real.py` is the
protocol server with the adapter stubbed; the TODOs are the robot-day work:

1. `state` at >= 50 Hz from telemetry, in the backend's field names (joint order from the deployment contract).
2. `stream`: 24-float rows (L wrist, R wrist, chest pose in the reference frame; 2 gripper widths; head pitch) ->
   the stack's 3-point teleop reference at 50 Hz. Reference frame = heading at session start.
3. `walk(forward_m, turn_deg)`: the stack's locomotion command; block until still; odometry in the reply.
4. `hold`: freeze the reference. `grasp_paw` / `place_in_box` answer `refused`.

Validate with `r1/tools/diag/carry_chain_test.py` pointed at the bridge port (it drives the backend exactly as the agent
does) before any RAX session.

## 4. Robot days, in order

Day 1 (no agent):
1. Bridge: `state`, `r1_observe`, `r1_move_hand` to points above the table, reach window of the right hand.
2. **Pads**: pinch the overhanging leg (`r1_move_hand` + `r1_gripper 0`), lift 10 cm, hold 60 s, two `r1_walk 0.07`
   steps: does the width stay? Sim's least trusted assumption. If it fails, the carry strategy changes (two-handed,
   or the stopper grasp behind the paw bulge) before anything else.
3. Walk precision: 10 x `r1_walk 0.07` and 10 x `0.15`, scatter and lateral drift -> task file numbers.
4. Camera: paw-ball localisation error at 0.3-0.4 m vs a tape measure.
5. Re-tune `r1_leg_lift` (lift height) and `r1_carry_release` (step, advance, lower) by hand; record in PARAMS as real.

Day 2 (RAX):
6. First session: `configure_session.py --task puppy_box_r1 --task-file tasks/puppy_box_r1.md --robot r1sonic
   --environment real --experience experience/puppy_box_r1/workspace --backend-options <bridge host/port>`;
   5 resets; a human operator answering requests within a minute (overhang, reset, "what do you see").
7. Expect the agent to spend its first attempts re-measuring; that is the design. Export after every session.

## 5. Success criterion for "transferred"

The real agent places the puppy from a walk-in start in at least 3 of 5 resets in one session, with no fall, using
the primitives or its skill, and its export carries `real` values for the lift and carry parameters.
