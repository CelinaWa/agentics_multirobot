# agentics_multirobot — real Unitree G1 + Dex3 under SONIC, driven by RAX

Everything needed to run the G1 for real, packed from the sim2sim-validated setup (DGX, 2026-10). Layout for the
target machine: **one local host runs both SONIC (whole-body controller) and RAX (robot service + agent)**; the
G1's Jetson streams the head camera.

```
rax/                 RAX real-robot branch + the complete G1 port (services/g1sonic, 5 G1 agent skills, console,
                     bring-up scripts). Upstream rax ships only the RB-Y1; everything G1 is ours.
rax/scripts/         g1_up.sh (one tmux: camera / sonic / console / dashboard / work), g1_check_all.sh (first-day
                     characterization), g1_check_quick.sh (power-on check), g1_motion_check.py, g1_console.py
jetson_camera/       camera_server.py + camera.py: RealSense (--camera realsense) or ZED (--camera zedsdk|zed) → TCP 7790
sonic/               SONIC commit pin, build/launch notes, standing-only reference motion, sim-only patch
docs/                REAL_ROBOT_WORKFLOW.md (power-on → check → agent, Chinese), REAL_ROBOT_DEPLOY*.md,
                     AGENT_WORKFLOW.md, g1_real_options.example.json
setup_rax_host.sh    one-time: two venvs (robot service / agent), settings templates, unit test
requirements-*.txt   robot service env, agent env, camera machine
```

## One-time setup (RAX/SONIC host, Ubuntu 22.04, NVIDIA GPU)

1. `bash setup_rax_host.sh --with-sonic` → envs in `~/envs/rax-g1` and `~/envs/rax-agent`, writes `~/.g1_up.env` and
   `~/g1_real_options.json` from the templates. Edit the IPs. Same-host SONIC: `state_host` 127.0.0.1, `RAX_HOST_IP` 127.0.0.1.
2. `--with-sonic` also clones SONIC (fork @ 6c821f1), fetches the **v1.1** models and builds it; details and the by-hand path: `sonic/SONIC_SETUP.md`.
3. Camera. Stock RealSense: copy `jetson_camera/` to the Jetson (`scp -r jetson_camera unitree@192.168.123.164:~/`),
   `pip install -r requirements-jetson-camera.txt` there, ssh key so `g1_up.sh` can start it. The camera pose then
   comes from the URDF (`d435_link`), no calibration. A helmet ZED instead needs `camera_pose_override`
   (pelvis-frame position + quaternion wxyz) in `g1_real_options.json` and the ZED SDK on the machine it is plugged into.
4. Agent only: install Claude Code (`npm i -g @anthropic-ai/claude-code`), then in `rax/`:
   `python -m harness.claude_cli.experiment_login` with the experiment account (see `docs/AGENT_WORKFLOW.md`).

## Every power-on

```bash
cd rax && bash scripts/g1_up.sh check      # first day: full characterization (tape measure, ~20 min)
cd rax && bash scripts/g1_up.sh check --quick-check   # later days: status + hold + arms, then stop
cd rax && bash scripts/g1_up.sh agent --task-file task.md
```
Keys: `sonic` pane `Y`; `console` pane `s` stand, `SPACE` arm/pause (toggle), `o` STOP; `work` pane answers
`>>> … [ENTER]` prompts. Full walkthrough with pictures of what each step does: `docs/REAL_ROBOT_WORKFLOW.md`.

## Not in this repo

SONIC code/models (2 GB, see `sonic/`), the sim2sim MuJoCo stack and the 900+ sim session logs, filled-in
settings (`~/.g1_up.env`, `~/g1_real_options.json`) and any credential. Never commit those.
