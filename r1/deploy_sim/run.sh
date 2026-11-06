#!/bin/bash
# The REAL R1 deployment in simulation, for testing rax/services/r1sonic/bridge_real.py before a robot day:
#   lab R1 MuJoCo DDS simulator (MA-Teleop teleop/clients/sim/r1_sonic_sim.py, DDS domain 42 on lo, --lockstep)
#   + r1_sim_cpu (gear_sonic_deploy, CPU build of r1_deploy_onnx_ref), zmq_manager input on 6556, telemetry on 6557.
# Run INSIDE the container that has both (DGX: docker exec sonic-sim2sim bash r1/deploy_sim/run.sh start).
# Domain 42 / ports 6556-6557 do not touch a G1 sim on domain 0 / 5556-5557.
#   run.sh start | stop        env: LOCKSTEP (1 default; 0 = free-running like the robot), POLICY_LOG, R1_ROOT (r1 workspace), SIM_PY (python with mujoco + unitree_sdk2py), OUT
set -e
R1_ROOT=${R1_ROOT:-/home/junjieye/workspace/multirobot/r1}
SIM_PY=${SIM_PY:-/opt/sonic/.venv_sim/bin/python}
OUT=${OUT:-$R1_ROOT/tools/scratch/deploy_sim}
BUNDLE=$R1_ROOT/SONIC_R1_100k; MODEL=$BUNDLE/exports/100000/model/exported
DEPLOY=$R1_ROOT/gr00t-wbc-r1/gear_sonic_deploy
case "$1" in
  stop)
    pkill -f "[r]1_sim_cpu lo" || true; pkill -f "[r]1_sonic_sim.py --headless" || true ;;
  start)
    "$0" stop; sleep 2; mkdir -p "$OUT/data/r1"
    # the lab script expects MA-Teleop's submodule layout: point it at gr00t-wbc-r1 and write its files under $OUT
    sed -e "s|^ROOT = Path(__file__).resolve().parents\[3\]|ROOT = Path(\"$OUT\")|" \
        -e "s|^GROOT = ROOT / \"submodules/GR00T-WholeBodyControl\"|GROOT = Path(\"$R1_ROOT/gr00t-wbc-r1\")|" \
        -e "s|row = dict(wall_s=elapsed, sim_s=float(data.time), root_z=|row = dict(wall_s=elapsed, sim_s=float(data.time), root_x=float(data.qpos[0]), root_y=float(data.qpos[1]), yaw_deg=float(np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))), root_z=|" \
        -e "s|if elapsed - last_log >= 1:|if elapsed - last_log >= 0.25:|" \
        -e "s|mujoco.mjtObj.mjOBJ_BODY, m.body(\"pelvis\").id, velocity, 1)|mujoco.mjtObj.mjOBJ_XBODY, m.body(\"pelvis\").id, velocity, 1)|" \
        "$R1_ROOT/MA-Teleop-junjie/teleop/clients/sim/r1_sonic_sim.py" > "$OUT/r1_sonic_sim.py"
    cd "$OUT" && nohup "$SIM_PY" r1_sonic_sim.py --headless $( [ "${LOCKSTEP:-1}" = 1 ] && echo --lockstep ) --bundle "$BUNDLE" --log "$OUT/sim.jsonl" > "$OUT/sim.out" 2>&1 &
    sleep 4
    cd "$DEPLOY" && (tail -f /dev/null | SONIC_DDS_DOMAIN=42 nohup ./target/release/r1_sim_cpu lo \
        "$MODEL/model_step_100000_decoder.onnx" "$OUT/data/r1/reference/" \
        --obs-config policy/r1/observation_config_sonic_r1_v1_1.yaml --encoder-file "$MODEL/model_step_100000_encoder.onnx" \
        --input-type zmq_manager --output-type zmq --zmq-host 127.0.0.1 --zmq-port 6556 --zmq-out-port 6557 \
        --disable-crc-check ${POLICY_LOG:+--policy-input-logfile "$POLICY_LOG"} > "$OUT/deploy.out" 2>&1 &)
    echo "started: logs $OUT/sim.out $OUT/deploy.out (wait for 'Init Done' in deploy.out, ~30 s)" ;;
  *) echo "usage: $0 start|stop"; exit 1 ;;
esac
