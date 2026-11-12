#!/bin/bash
# The two-robot office simulation, inside the sonic-sim2sim container (run with: docker exec sonic-sim2sim bash team_sim.sh ...).
#   team_sim.sh start [--no-g1-deploy]   tmux session "teamsim": r1 brain (CPU), the world (CPU), the G1 deploy stack (GPU)
#   team_sim.sh stop
# Ports: G1 deploy pose/state 5796/5797, G1 camera 7796, sim keys 5790, qpos 5792, R1 tick 5795, R1 stack protocol 5791. DDS domain 44.
T=/home/junjieye/workspace/multirobot/singler1g1_wt/sim_team
G=/home/junjieye/workspace/multirobot/r1/tools/scratch/g1cmp
BIN=/home/junjieye/workspace/multirobot/r1/gr00t-wbc-r1/gear_sonic_deploy/target/release/g1_deploy_onnx_ref
SCENE=$T/scene/office_g1_r1.xml
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
export PYTHONUNBUFFERED=1
unset PYTHONPATH
case "${1:-}" in
  start)
    tmux kill-session -t teamsim 2>/dev/null; rm -f $T/logs/*.log; mkdir -p $T/logs
    SPAWN=$(/opt/r1venv/bin/python -c "import json; r=json.load(open('$T/scene/layout.json'))['robots']['g1']; print(f\"{r['pos'][0]},{r['pos'][1]},{r['yaw_deg']}\")")
    tmux new-session -d -s teamsim -n brain "cd $T && MUJOCO_GL=osmesa CUDA_VISIBLE_DEVICES= exec /opt/r1venv/bin/python r1_brain.py --scene $SCENE --tick-port 5795 --port 5791 2>&1 | tee $T/logs/brain.log"
    sleep 4
    tmux new-window -t teamsim -n sim "cd /opt/sonic && SIM_DDS_DOMAIN=44 CUDA_VISIBLE_DEVICES= exec /opt/sonic/.venv_sim/bin/python $T/office_sim.py --env-name default --scene $SCENE --spawn=$SPAWN --key-port 5790 --qpos-port 5792 --state-port 5797 --r1-tick-port 5795 --lockstep 2>&1 | tee $T/logs/sim.log"
    sleep 3   # the G1's head camera and the third-person recording: mirrors the world's qpos, renders on the CPU
    tmux new-window -t teamsim -n camera "cd /opt/sonic && MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa CUDA_VISIBLE_DEVICES= exec /opt/sonic/.venv_sim/bin/python $T/../rax/scripts/g1sonic_sim2sim/sim_camera_server.py --scene $SCENE --port 7796 --qpos-port 5792 --snapshot-dir $T/logs/snap --record $T/logs/overview.mp4 2>&1 | tee $T/logs/camera.log"
    if [ "${2:-}" != "--no-g1-deploy" ]; then
      sleep 6
      tmux new-window -t teamsim -n deploy "cd $G/sonic && SONIC_DDS_DOMAIN=44 LD_LIBRARY_PATH=/opt/onnxruntime/lib:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu CUDA_HOME=/usr/local/cuda exec $BIN lo policy/release/model_decoder.onnx reference/standing/ --obs-config policy/release/observation_config.yaml --encoder-file policy/release/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type zmq --zmq-host 127.0.0.1 --zmq-port 5796 --zmq-out-port 5797 --disable-crc-check 2>&1 | tee $T/logs/deploy.log"
    fi
    echo started ;;
  stop)
    tmux send-keys -t teamsim:deploy O 2>/dev/null; tmux send-keys -t teamsim:camera C-c 2>/dev/null; sleep 2; tmux kill-session -t teamsim 2>/dev/null; echo stopped ;;
  *) echo "usage: $0 start [--no-g1-deploy] | stop"; exit 1 ;;
esac
