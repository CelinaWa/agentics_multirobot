# Real-robot deploy workflow: RAX on the Unitree G1 + Dex3 under SONIC

Written 2026-10-26 from the code in `2/homebody/.cache/rax-g1real` (branch `g1real`, backend
`services/g1sonic/`) and from the sim harness in `3/eng`. Paths below are relative to
`~/workspace/multirobot`. Numbers are the backend defaults (`DEFAULTS` in
`services/g1sonic/backend.py`) unless a FAILURE_LOG row is cited.

Sections: 1 system map, 2 safety layers, 3 one-time setup, 4 a session step by step,
5 what the tools do on the real robot, 6 faults and recovery, 7 sim workflow vs real workflow,
8 what we know about robustness today, 9 recommended first-day sequence on the robot.

---

## 1. System map (real robot)

```
 operator (person at the e-stop)            dashboard  scripts/run_dashboard.sh (loopback, tokenized URL)
   arm / disarm  -> <session>/operator/armed.json        requests / replies -> <session>/operator/requests/
   gantry, keyboard O / gamepad Select on the deploy stack (damps all joints = robot sits)

 Claude Code agent  (host process, cwd <session>/agent-workspace, bubblewrap sandbox, --permission-mode dontAsk)
   |  MCP stdio -> harness/claude_cli/mcp_bridge.py -> Unix socket
   v
 robot service  services.controller.frontend --socket   (the ONLY process that commands the robot; audit log mcp.jsonl)
   |  G1SonicBackend  (services/g1sonic/backend.py)
   |   - validates every reference row (limits, step caps), streams 50 Hz semantic40 references
   |   - reads the stack's state every tick, applies the tilt guard, owns arming checks
   |  ZMQ PUB  tcp 5556  topic pose / command / planner   -->+
   |  ZMQ SUB  tcp 5557  topic g1_debug (msgpack state)   <--+
   v                                                        |
 SONIC deploy stack  gear_sonic_deploy/g1_deploy_onnx_ref   (frozen encoder + decoder + locomotion planner, TensorRT)
   started by the operator with <a standing reference dir rather than reference/example/, hygiene per row 167-168> --input-type zmq_manager --output-type zmq --planner-file planner/target_vel/V2/planner_sonic.onnx
   --zmq-host <service host> --zmq-port 5556 --zmq-out-port 5557 ; its own PD loop and safety check every tick
   |  DDS (unitree_sdk2) on the robot network 192.168.123.x
   v
 G1 motors + Dex3 hands            camera: HomeBody camera_server on the G1's Jetson 192.168.123.164:7790 (RGB + depth)
                                            or MA-Teleop's ZED publisher (ZMQ 5620, colour only, needs camera_intrinsics)
```

- The backend never touches DDS. It only speaks the stack's ZMQ wire (`services/g1sonic/zmq_protocol.py`).
- The agent's sandbox has no loopback, no access to the repository, credentials, docker/X11 sockets or
  `RAX_PROTECTED_PATHS`; internet only through Claude Code's proxy, which refuses the robot controller.
  The agent can therefore reach the robot only through the MCP tools.
- Subagents never command the robot. Gemini (image checks) is brokered by the robot service with a budget.

## 2. Safety layers, outermost first

| Layer | What stops the robot | Default / note |
|---|---|---|
| 1 Physical | E-stop; the stack's own stop (keyboard `O`, gamepad Select, or a `stop` command on the wire) damps every joint, the robot sits down | always the outermost layer |
| 2 Operator arming | Every motion tool refuses unless `operator/armed.json` says armed; the backend re-reads it before and during every motion, a disarm stops a running motion within one 50 Hz tick and the robot holds its last reference | arming can carry an expiry (`arm 30` = 30 min) |
| 3 Backend guards | joint references inside URDF limits minus `joint_limit_margin_rad` 0.05; `max_joint_step_rad` 0.15 between consecutive rows (7.5 rad/s); `max_first_step_rad` 0.35 between the held pose and a call's first row; root-rotation jump slerped to `max_root_delta_deg` 26 per tick; a call longer than 5 rows must end in 5 identical rows (a pose to hold); state older than `state_max_age_s` 0.2 s = lost link, motion refused or stopped; tilt over `fall_tilt_deg` 60 = stop (damp) + session disarmed as a fault | the operator must arm again on purpose after a fault |
| 4 Deploy stack | its safety check every tick, motor limits | NVIDIA's code |

`g1_hold` is always allowed, armed or not, but it is a hold, not an emergency stop.

## 3. One-time setup on the robot host

1. Network: the host sits on the robot network (`192.168.123.x`), reaches the G1 (`ping 192.168.123.164`),
   and the deploy stack's machine. Stop every other process that commands the robot (teleop, other
   publishers): one owner.
2. Two Python environments, as in the RAX README: `RAX_PYTHON` for the robot service (numpy, scipy,
   opencv-headless, pyzmq, msgpack, cuRobo for `g1_free_space_move`, ...), `RAX_AGENT_PYTHON` for the agent
   (no robot SDK, no ZMQ access to the stack).
3. `RAX_PROTECTED_PATHS`: colon-separated paths the agent must never read or run (the SONIC deploy
   checkout, unitree_sdk2, teleop stacks, the robot-service venv). Empty is unsafe and the launcher warns.
4. Model credential: `scripts/setup_claude_experiment_login.sh` (a dedicated experiment account), or
   `--key-file` for an API key. Optional OpenRouter key for Gemini.
5. Storage: sessions, frames and logs under `runtime/` or `RAX_ARTIFACT_ROOT`.
6. Operator command: the `rby1` script in `scripts/` is robot-agnostic (it only writes `armed.json` and answers
   requests of the newest session); symlink it as the G1 operator command or call it as is.
7. Backend options JSON for `configure_session.py --backend-options` (anything in `DEFAULTS`); the ones that
   differ from the sim harness are listed in section 7. Set `camera`, `camera_host`, `state_host`,
   `zmq_host`, and keep `wall_time_scale` at 1.0 on the real robot.

## 4. A session step by step

Hardware state at the start: the G1 hangs from the gantry, feet about 0.2 m above the floor (lowered onto its feet,
straps still attached, before the console START: with no ground contact the policy paddles its legs), motors on,
the deploy stack running (`g1_deploy_onnx_ref` with the flags in section 1, planner loaded) and in INIT:
it publishes **no state until its control runs**.

1. **Configure** (writes files, starts nothing):
   ```
   $RAX_PYTHON scripts/configure_session.py --task <id> --task-file task.md --robot g1sonic \
       --backend-options g1_real_options.json --observation-profile rgbd --session-hours 4
   ```
   It prints `<session>/operator-input.json`.
2. **Start the agent and the robot service**: `bash scripts/start_agent.sh --config <session>/operator-input.json`
   (`--non-interactive` without the terminal UI; `--resume` continues a stopped session). Ctrl-C stops both.
   Open the dashboard (`scripts/run_dashboard.sh`) to see arming, requests, frames and live state.
3. **The agent reads TASK.md and its skills**, calls `status` and `g1_status` (refused motion is reported as
   "motion is not armed"), and asks for arming through `request_operator` if it needs to.
4. **Operator arms**: `rby1 arm [minutes]`, confirm by typing ARM with a person at the e-stop and the
   workspace clear. Disarm any time with `rby1 disarm`.
5. **First armed `g1_hold` (or `g1_step`) starts the stack's control.** This is the critical boot
   (`_ensure_control` in the backend):
   - the backend sends `start` + `planner` commands (repeated up to `start_timeout_s` 8 s) until the first
     state message arrives; the robot now stands under SONIC's planner in place;
   - it waits `planner_settle_s` 3 s, then until quiet (`settle_quiet_s` 1 s with tilt < `settle_tilt_deg` 10
     and every joint slower than `settle_joint_speed` 1 rad/s, or motionless 3 s at tilt < 14), giving up
     after `settle_timeout_s` 15 s with "the robot did not come to rest ... the operator checks it";
   - **the operator lowers the gantry until the feet take the weight during this settle window, but keeps the
     band attached (slack)**; in sim `gantry.py` lowers 0.2 m, waits for the first state message, 8 s of control
     on the ground, then fades the band over 6 s. On the real robot the band comes off only after the first
     `g1_walk` has completed, with the feet about shoulder width apart and the robot quiet for 2 s (see Robustness
     status: both boot-fall classes);
   - the hold is primed from the measured stance, the stack is switched to streamed motion, and the hold
     eases to SONIC's default standing pose over `stand_ramp_s` 1.5 s. Releasing earlier dropped the robot
     mid-start into whatever stance it had; jumping to the default stance in one frame tipped it over.
6. **Exploration loop** (what the agent does, all through tools): `g1_observe` (RGB-D + camera pose in the
   pelvis frame + joints + root orientation), `g1_pixel_to_position` (depth pixels to pelvis-frame and level-frame
   points, tied to the observation's step_id), `g1_free_space_move` (cuRobo arm plan, `preview_only` then
   `g1_execute_motion_plan`), `g1_step` (1 to 100 semantic40 rows, 2 s max per call), `g1_hand_shape`
   (Dex3 open/closed joint targets a step's hand fraction interpolates), `g1_walk` (planner locomotion), `g1_hold`.
   The agent writes code in `agent-workspace/code/` and runs it under its own observation; what works goes
   into the skill library (`skill_library_merge`).
7. **Operator requests**: `request_operator` (kinds help, reset) blocks the agent until `rby1 reply "done"`
   or the dashboard answers; the robot holds meanwhile. Use it for scene resets and for re-hanging the robot.
8. **End**: `rby1 disarm`, then Ctrl-C in the agent terminal; stop the stack with keyboard `O` (the robot
   sits: have the gantry band on first) and power off. Everything the session did is in `<session>/mcp.jsonl`
   (every tool call and result), `agent-workspace/runtime/frames/` (observations) and the dashboard's
   telemetry; `rby1 ship` archives finished sessions.

## 5. What the tools do on the real robot (semantics that matter for safety)

- **References are not motor targets.** Joints track to a few degrees on an empty floor; the root drifts tens
  of centimetres over a walk. The robot holds the last row of every call until the next call.
- **Root xy and height in a row are ignored**: the robot translates only by stepping (leg references) or by
  `g1_walk`. Heading is the rot6d block relative to the heading at the session's first motion.
- **`g1_walk`** hands the robot to SONIC's own locomotion planner: `forward_m` 0 to 3 (forward only), `turn_deg`
  within ±45 per call, `speed_mps` 0.1 to 0.6. Inside the backend: the first walk after a start begins with
  +15/−15 degree warm-up turns (the cold planner lagged its reference 0.2 to 0.7 m and lunged); walks longer
  than `walk_segment_m` 0.5 run as segments with their own planner start and stop (0.9 m walks fell in sim);
  facing turns at `walk_turn_rate_deg_s` 30 (45 deg/s spun the waist at 7 rad/s and the robot fell); turns
  step forward at 0.1 m/s (a pivot right after a walk with staggered feet fell); after the move the planner idles
  `walk_settle_s` 1.5 s and the backend waits up to `walk_quiet_timeout_s` 6 s for quiet before handing back to
  the streamed hold. **If the robot is still stepping after that, the backend stays in planner mode and the next
  armed call hands over** (see section 8: this hand-over gap is a known failure mode).
- **`carry=true`** keeps the waist, both arms and both hands at the last commanded frame through the walk (the
  planner otherwise swings the arms and drops anything held).
- **No odometry on the wire**: walk distances are commanded, not measured; the agent must re-localize against
  the scene. Our sim harness added leg odometry from the joint state (`3/eng/legodo.py`), which uses only the
  5557 state stream and the robot model, so it is real-robot feasible but untested there.
- **`g1_free_space_move`** avoids self-collision only; furniture and objects are unknown to the planner.
  Comfortable reach is 0.35 to 0.45 m ahead of the pelvis; beyond it contact pushes the robot back.

## 6. Faults and recovery

| Event | Backend behaviour | Operator action |
|---|---|---|
| tilt > 60 deg during any motion or in the state watch | `stop` sent to the stack (damp, `fall_damp` true), session disarmed with a `fall` fault; every motion tool refuses until re-armed | catch or let the robot sit, re-hang on the gantry, inspect, `rby1 arm` again (an arming newer than the fault clears it); the next armed hold restarts control (section 4 step 5) |
| state older than 0.2 s | motion refused ("the link is lost or the stack stopped") or a running call stopped; the robot holds its last reference inside the stack | check the stack's terminal and the network |
| state older than `stack_restart_s` 2 s while streaming | "control reset": the hold is forgotten; the next armed hold or step starts the stack again | relaunch the stack if it died, gantry on before the restart |
| operator disarm | running motion stops within a tick, robot holds | — |
| robot not quiet after the start (hanging unevenly, leaning on something) | start fails after 15 s, session can retry | level the robot, clear contacts, arm again |
| keyboard `O` / gamepad Select / e-stop | all joints damped, robot sits | gantry band on first whenever possible |

## 7. Sim workflow (3/eng) vs real workflow: what is different

The sim harness drives the **same backend, the same MCP tools and the same tool semantics** (that is the point of
sim2sim), but almost everything around the backend is different.

| Topic | Sim (3/eng, container `sonic-sim2sim`) | Real robot |
|---|---|---|
| Robot | gear_sonic MuJoCo sim (`sim_headless.py`), scene XMLs `scene_office*.xml` with the desk, plate, fridge, bin | the G1 in the lab; object positions unknown to the code |
| Deploy stack launch | `run_sim2sim.sh start`: tmux windows sim / camera / deploy, interface `lo`, `--disable-crc-check`, CycloneDDS on loopback, `LOCKSTEP` and `SONIC_TIME_SCALE` to slow the control period on a busy GPU | started by the operator on the robot network (MA-Teleop's `g1_deploy.sh` or NVIDIA's instructions), real-time, no CRC bypass, same `--input-type zmq_manager --output-type zmq --planner-file` flags (a deploy without the planner cannot be started over ZMQ) |
| Operator | code: `trial.McpSession` / `rob.Robot.start` write `armed.json` themselves and run `gantry.py` (keys to the sim's band over ZMQ 5590) | a person: `rby1 arm` with ARM typed, the physical gantry lowered and released by hand during the settle window, e-stop |
| Backend options | `state_max_age_s` 1.5, `planner_settle_s` 0, `wall_time_scale` = `SONIC_TIME_SCALE`, camera `network` at 127.0.0.1:7790 | `state_max_age_s` 0.2 (default), `planner_settle_s` 3, `wall_time_scale` 1.0, camera at the Jetson 192.168.123.164:7790 (or `ma-teleop` + `camera_intrinsics`, rgb-only profile) |
| Camera | `sim_camera_server.py` renders the scene's `head_cam` (RGB + perfect depth) | HomeBody `camera_server.py` on the Jetson: RealSense-class depth with holes and noise at plate rims and shiny surfaces; `camera_pose_override` if the mount differs from the URDF `d435_link` |
| Ground truth | `gt.py` reads the sim's qpos stream (tcp 5591): `trace.jsonl` at 10 Hz, plate/wrist/pelvis positions, automatic success scoring, fall classification (`fall_stats.py`, `fall_causes.py`) | none. Success is judged by perception or by the operator; falls are seen, not measured. Everything in `ctl.py`/`skills.py` is already GT-free (GT is scoring only), so the controllers port; the scoring does not |
| Falls | the harness restarts the stack (`rob.restart_stack`, `FRESH=1`) and the trial continues; the robot respawns hanging | a fall is damage. The backend damps and disarms; a person re-hangs the robot and re-arms. Every sim topple counts as a real incident |
| Boot | automatic: fresh stack per trial, `START_TILT` gate, `gantry.py --settle 8 --fade 6` | manual lowering and release; timing depends on the operator. `start_check.sh` (sim) is the rehearsal of this step |
| Walk calibration | `ctl.py` tables (`_CAL_CMD/_CAL_MOVED`, `CARRY_RATIO` 0.66, `short_cmd`) measured with GT in sim at time scale 1.0 | must be re-measured on the real floor (tape measure or lidar); friction, battery and the real-time control period change them |
| Odometry | `legodo.py` validated against GT within mm (row 165: within 1.1 cm over a 3 cm creep) | same code should run (joint state + IMU only), but foot slip on the real floor is untested; validate against a tape measure first |
| Turn safety | `ctl.py` wrapper: turns split into ≤ `SAFE_TURN_MAX` 30 deg segments, no locomotion while tilt > `SAFE_TURN_TILT` 8, chain stopped after a segment with max tilt > `SAFE_TURN_WOBBLE` 12 | identical code; the thresholds were tuned on sim tilt statistics (normal turns 3 to 5 deg, pre-fall turns 9 to 14) and should be re-checked on real IMU data |
| Contacts and grasps | MuJoCo contact with `SIM_DT` 0.002 for finger contacts; plate rim pinch success 5/85 live so far | Dex3 compliance, real plate weight and friction; expect different grasp outcomes, same fall mechanics |
| Agent | the harness calls the tools from Python (`rob_mcp.McpRobot`), no LLM in the loop for the eng trials; a real RAX session runs Claude Code in the sandbox | Claude Code agent in the sandbox, `--permission-mode dontAsk`, 4 h sessions, 2 subagents without robot access |
| Logs | `3/eng/runs/<name>.log`, `runs/<trial>/` with overview video, `trace.jsonl`, stack logs; `FAILURE_LOG.md` rows | `<session>/mcp.jsonl`, `runtime/frames/`, dashboard telemetry, `rby1 ship` archives |

Things that exist only in sim and must not be relied on: the GT qpos stream, the sim keys (band, release, fade),
`SONIC_TIME_SCALE`/`LOCKSTEP`, perfect depth, automatic stack restarts, object respawn (`restore_objects.py`).

## 8. What we know about robustness today (sim, FAILURE_LOG rows 160 to 165)

The user's criterion: task failure is acceptable, a fall or an uncontrolled motion is not.

- **Boot falls, two classes (rows 167-171)**: 94 boot-stage falls in 1132 sim boots.
  **(1) The first real stride, ~5-10 % of boots.** Per-boot replays with the stack's own targets recorded
  (`bootcheck.py`: `base_trans_target`, `body_q_target` next to ground truth) show the planner's first-stride plan is
  identical in fallen and clean boots (base 0.30 m over 1.3 s); in the fall the frozen policy fails the swing (swing
  knee overshoots its target, the foot is still in the air when the plan has it down, tilt 10 -> 60 deg in under a
  second, pelvis 0.7-1.2 m/s). Not the reference motion (a standing-only reference changed nothing), not the
  commanded speed (0.2 m/s and a 0.1 -> 0.3 ramp fell the same way, and `walk_ramp_s` > 0 made 15/22 short 0.2 m
  walks not step at all), and not fixable by a stance check (an asymmetric stance, knee R-L >= 0.09 rad with the left
  foot ahead, raises the odds, but a textbook-symmetric stance fell too; re-standing the joints did not help). What
  never fell: planner shuffles (turns at 0.1 m/s, knee targets <= 0.15, 571/571 turn segments), and walks after the
  first one (0 task-time falls in the ball run). **Real robot: the first `g1_walk` of the session, with its warm-up
  turns and first forward segment, is done with the gantry band still attached (slack); unhook after it.**
  **(2) Standing after the release, ~1/30 boots.** Hanging in the band the robot marches in place for 10-13 s (the
  sim band carries part of the weight: the policy's feet are unloaded); in the fallen boots the march ended with the
  feet 0.135-0.160 m apart (clean: 0.264-0.270 m), and the robot pitched forward 2-3 s after the band was gone, with
  no command in flight. Foot placement follows from the joint state (`3/eng/fk.py`, forward kinematics, within 1-2 cm
  of ground truth), so the check is GT-free. **Real robot: before unhooking, the feet are about shoulder width
  apart and the robot has been quiet for 2 s; feet together means re-hang and let it stand again.**
  A third release variant (4/40 in the afternoon's control arm, row 173): the robot comes off the band in a staggered
  stance (feet 15-18 cm apart front-to-back, ~10 deg tilt); shuffle turns then drive it into a 45-50 cm split that
  never settles, and a forced walk falls. The same pre-unhook check covers it: feet side by side, shoulder width,
  quiet. Control-arm rates (40 boots, row 173): 29 clean, 4 first-stride falls or lunges, 3 post-release falls,
  4 bad-stance releases. A GT-free stance rule that preceded 9/9 stride falls and 0/36 clean boots (row 172:
  pelvis > 8 cm ahead of the ankles, a foot > 11 cm behind, knees uneven by >= 0.09 rad, or feet < 0.20 m apart)
  is under test as a gate (`bootcheck.py STANCE_GATE=2`).
  The standing reference stays as hygiene (no side effects, removes the dance frame on planner timeouts) but is not
  a boot fix.
- **Turn chains**: 10 of the last 15 task-time topples came during or right after chained 45 degree planner
  turns (row 162). The turn safety wrapper (above) is the fix; the ball run with it on is in progress (row 166).
- **Fridge pulls**: 3 topples while pulling the door (rows 110 to 121). Door opening works 10/10, open + close
  8/10; nothing has been carried to or placed in the fridge yet.
- **Arm work hardly falls**: the plate rounds that fail do so standing (tilt 2 to 3 deg); failures are reach,
  creep (the robot slides back about 3 mm/s while leaning with the arms out) and the planner hand-over gap.
- **Hand-over gap**: after a walk the robot sometimes keeps marching in place; `g1_hold` is a no-op in planner
  mode, so a client that waits for the hold stalls for a minute. Mitigation in `rob.py`: a 0.1 m re-settle walk
  after 20 s and 40 s. On the real robot this shows as the robot stepping in place indefinitely: the operator
  can simply disarm, and the planner keeps it balanced.
- **Stack restarts in sim collapse the robot** (76 of 192 tilt events were restart collapses, not topples): this
  class does not exist on the real robot, but a stack crash there leaves the robot damped on the floor.

## 9. Recommended first-day sequence on the robot

Mirror `start_check.sh` and `dry_session.py` by hand, one layer at a time, robot on the gantry throughout:

1. Stack up, robot hanging: `g1_status` before arming (nothing moves; expect "publishes no state until its
   control runs"), a `g1_step` while disarmed must be refused.
2. `rby1 arm 5`, `g1_hold`: watch the planner stand, lower and release the gantry in the settle window, confirm
   `g1_status` reports mode streamed, tilt under 2 degrees, no fault. Repeat the full boot several times before
   anything else: this is the step with the worst sim record.
3. One small `g1_step` (arm raise, 1 s ramp, held tail), compare the joint RMSE to the sim's 0.11 to 0.12 rad.
4. `g1_observe` and `g1_pixel_to_position` on a known object: check the depth-to-pelvis transform with a ruler
   (this is where the camera mount and depth noise differ from sim).
5. Walk warm-up only (`g1_walk` with turn 15, then −15), then 0.3 m forward, measure the distance covered,
   rebuild the `ctl.py` calibration table.
6. Only then the tasks, starting with arm-only ones (plate pinch at the desk) and leaving fridge door pulls
   and multi-turn navigation for last, with the band kept slack on the robot.
