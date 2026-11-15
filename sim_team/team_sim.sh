#!/bin/bash
# The two-robot office simulation, inside the sonic-sim2sim container (run with: docker exec sonic-sim2sim bash team_sim.sh ...).
#   team_sim.sh start [--no-g1-deploy]   tmux session "teamsim": r1 brain (CPU), the world (CPU), the G1 deploy stack (GPU)
#   team_sim.sh restart-deploy           a new G1 deploy stack (after a fall the old one has stopped)
#   team_sim.sh stop
# Ports: G1 deploy pose/state 5796/5797, G1 camera 7796, sim keys 5790, qpos 5792, R1 tick 5795, R1 stack protocol 5791. DDS domain 44.
T=/home/junjieye/workspace/multirobot/singler1g1_wt/sim_team
G=/home/junjieye/workspace/multirobot/r1/tools/scratch/g1cmp
BIN=/home/junjieye/workspace/multirobot/r1/gr00t-wbc-r1/gear_sonic_deploy/target/release/g1_deploy_onnx_ref
# Several simulations side by side: TEAM_INSTANCE=n (default 0) shifts every port by 100 n, the DDS domain by n, and
# uses tmux session teamsim<n> and logs<n>/; TEAM_GPU picks the G1 deploy stack's GPU.
N=${TEAM_INSTANCE:-0}; OFF=$((N * 100)); SFX=$([ "$N" -gt 0 ] && echo "$N")
SES=teamsim$SFX; LOGS=$T/logs$SFX; DOMAIN=$((44 + N))
P_KEYS=$((5790 + OFF)); P_R1=$((5791 + OFF)); P_QPOS=$((5792 + OFF)); P_TICK=$((5795 + OFF)); P_POSE=$((5796 + OFF)); P_STATE=$((5797 + OFF)); P_CAM=$((7796 + OFF))
SCENE_DIR=${TEAM_SCENE_DIR:-$T/scene}      # another scene directory (office_g1_r1.xml + layout.json): TEAM_SCENE_DIR
SCENE=$SCENE_DIR/office_g1_r1.xml
DEPLOY="cd $G/sonic && SONIC_DDS_DOMAIN=$DOMAIN ${TEAM_GPU:+CUDA_VISIBLE_DEVICES=$TEAM_GPU }LD_LIBRARY_PATH=/opt/onnxruntime/lib:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu CUDA_HOME=/usr/local/cuda exec $BIN lo policy/release/model_decoder.onnx reference/standing/ --obs-config policy/release/observation_config.yaml --encoder-file policy/release/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type zmq --zmq-host 127.0.0.1 --zmq-port $P_POSE --zmq-out-port $P_STATE --disable-crc-check 2>&1 | tee $LOGS/deploy.log"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
export PYTHONUNBUFFERED=1
unset PYTHONPATH
case "${1:-}" in
  start)
    tmux kill-session -t $SES 2>/dev/null; rm -f $LOGS/*.log; mkdir -p $LOGS
    SPAWN=$(/opt/r1venv/bin/python -c "import json; r=json.load(open('$SCENE_DIR/layout.json'))['robots']['g1']; print(f\"{r['pos'][0]},{r['pos'][1]},{r['yaw_deg']}\")")
    tmux new-session -d -s $SES -n brain "cd $T && MUJOCO_GL=osmesa CUDA_VISIBLE_DEVICES= exec /opt/r1venv/bin/python r1_brain.py --scene $SCENE --tick-port $P_TICK --port $P_R1 2>&1 | tee $LOGS/brain.log"
    sleep 4
    tmux new-window -t $SES -n sim "cd /opt/sonic && SIM_DDS_DOMAIN=$DOMAIN CUDA_VISIBLE_DEVICES= exec /opt/sonic/.venv_sim/bin/python $T/office_sim.py --env-name default --scene $SCENE --spawn=$SPAWN --key-port $P_KEYS --qpos-port $P_QPOS --state-port $P_STATE --r1-tick-port $P_TICK --lockstep 2>&1 | tee $LOGS/sim.log"
    sleep 3   # the G1's head camera and the third-person recording: mirrors the world's qpos, renders on the CPU
    tmux new-window -t $SES -n camera "cd /opt/sonic && MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa CUDA_VISIBLE_DEVICES= exec /opt/sonic/.venv_sim/bin/python $T/../rax/scripts/g1sonic_sim2sim/sim_camera_server.py --scene $SCENE --port $P_CAM --qpos-port $P_QPOS --snapshot-dir $LOGS/snap --record $LOGS/overview.mp4 2>&1 | tee $LOGS/camera.log"
    if [ "${2:-}" != "--no-g1-deploy" ]; then
      sleep 6
      tmux new-window -t $SES -n deploy "$DEPLOY"
    fi
    echo started ;;
  restart-deploy)
    tmux send-keys -t $SES:deploy O 2>/dev/null; sleep 1; tmux kill-window -t $SES:deploy 2>/dev/null
    pkill -f "^$BIN .*--zmq-port $P_POSE " 2>/dev/null; sleep 1
    mv -f $LOGS/deploy.log $LOGS/deploy.prev.log 2>/dev/null   # the new stack's log starts empty
    tmux new-window -t $SES -n deploy "$DEPLOY"
    echo restarted ;;
  stop)
    tmux send-keys -t $SES:deploy O 2>/dev/null; tmux send-keys -t $SES:camera C-c 2>/dev/null; sleep 2; tmux kill-session -t $SES 2>/dev/null; echo stopped ;;
  *) echo "usage: $0 start [--no-g1-deploy] | restart-deploy | stop"; exit 1 ;;
esac
