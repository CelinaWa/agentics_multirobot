# SONIC on the RAX host (same machine)

SONIC (NVIDIA GR00T-WholeBodyControl) is not vendored here: the repo plus models is ~2 GB. This folder pins
what we run and carries the two small things that are ours.

| Item | Where |
|---|---|
| Code | fork `https://github.com/Jinghy123/GR00T-WholeBodyControl` (public; the `Jay-Ye` fork is private), commit in `GR00T_WBC_COMMIT.txt` (6c821f1, 2026-04-29). Upstream: `NVlabs/GR00T-WholeBodyControl` |
| Models | **SONIC v1.1** from `huggingface.co/nvidia/GEAR-SONIC`: `sonic_v1_1/model_{encoder,decoder}.onnx` + `sonic_v1_1/observation_config.yaml` → `policy/release/`, `planner_sonic.onnx` → `planner/target_vel/V2/`. Hashes in `models.sha256`. **Do not use the fork's `download_from_hf.py` or the checkout's own `observation_config.yaml`: those are the older v1.0 set, not what the sim2sim runs validated.** `download_models.py` fetches the right files |
| Standing reference | `reference_standing/stand_000/` here → copy to `gear_sonic_deploy/reference/standing/`. 500 identical frames of the default standing pose, made by `make_standing_ref.py`. With it the console's `s` stands the robot up without planner steps |
| Container-only patch | `sim_time_scale_and_planner_diag.patch`: `SONIC_TIME_SCALE` env (slows the control period for sim2sim) and a `[planner-starved]` print. **Not needed on the real robot**; apply only if you also want to run sim2sim on this host |

## Build (once)

Automated: `bash setup_rax_host.sh --with-sonic` does everything below and checks the model hashes. By hand:

```bash
git clone https://github.com/Jinghy123/GR00T-WholeBodyControl.git ~/GR00T-WholeBodyControl
cd ~/GR00T-WholeBodyControl && git checkout $(cat <this repo>/sonic/GR00T_WBC_COMMIT.txt)
cd gear_sonic_deploy
bash scripts/install_deps.sh          # unitree_sdk2, onnxruntime/TensorRT, zmq, cmake deps (Ubuntu 22.04, CUDA GPU)
python3 <this repo>/sonic/download_models.py .     # needs: pip install huggingface_hub
sha256sum -c <this repo>/sonic/models.sha256
cp -r <this repo>/sonic/reference_standing/stand_000 reference/standing/
source scripts/setup_env.sh && just build
```

Needs CUDA 12 + TensorRT 10 on the machine (the sim container: TensorRT 10.13, CUDA 12.9); `install_deps.sh` installs the rest (unitree_sdk2, yaml-cpp, zmq, clang, just). The machine needs a wired NIC on the robot network `192.168.123.x`; `deploy.sh real` auto-detects it.

## Launch (every power-on) — `rax/scripts/g1_up.sh` opens this in the `sonic` pane

```bash
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy
./deploy.sh real --input-type zmq_manager --output-type zmq \
    --planner planner/target_vel/V2/planner_sonic.onnx \
    --motion-data reference/standing/ \
    --zmq-host 127.0.0.1          # RAX robot service on the same machine
```

Answer `Y`. Wait for `Init Done` (robot PD-held in the gantry, not balancing). Then the console: `s` = start (stand), `SPACE` = arm/pause, `o` = STOP (damping — robot must hang). Ports: RAX publishes actions on tcp 5556, SONIC publishes state on 5557, both on loopback.
