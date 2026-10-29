#!/bin/bash
# Start the SONIC sim2sim pair for the g1sonic backend: the headless MuJoCo G1 (gear_sonic, DDS on
# loopback) and g1_deploy_onnx_ref (--input-type zmq_manager with the planner, pose 5556, state 5557),
# each in its own tmux window with a log under $SONIC_ROOT/logs. Usage: run_sim2sim.sh start|stop|status
# NO_PLANNER=1 starts the deploy without the planner (what MA-Teleop's g1_deploy.sh does unless given
# --planner-file): a start over ZMQ only works in planner mode, so such a deploy cannot be started.
# RECORD=<mp4> also records the scene's overview camera for the whole run (a session's video).
# LOCKSTEP=1 makes the sim wait for each deploy control tick (a deploy on a shared, busy GPU).
# SONIC_TIME_SCALE=1.5 (with LOCKSTEP) stretches the deploy's control period in wall time (each tick is still 20 ms of
# robot time), so a sim on a busy host keeps up; the backend then needs the same `wall_time_scale` option.
# SIM_DT=0.002 shortens the physics step (finger contacts with small objects).
# SCENE=<abs xml> swaps the sim scene (build_office_scene.py) and adds a `camera` window serving the
# scene's head_cam on tcp 7790 (sim_camera_server.py). EXTRA_DEPLOY_ARGS appends to the deploy line.
# Then: RAX_ARTIFACT_ROOT=... $SONIC_ROOT/.venv_sim/bin/python probe.py [--camera]
#
# SONIC_ROOT (default /opt/sonic) holds: gear_sonic_deploy/ built with TensorRT 10.13 (target/release/
# g1_deploy_onnx_ref; models in policy/release/ with the v1.1 observation_config.yaml, planner in
# planner/target_vel/V2/), gear_sonic/ (the python sim), external_dependencies/unitree_sdk2_python/,
# and .venv_sim from make_venv.sh. On the DGX this is the `sonic-sim2sim` container (GPU 1, host
# network); an nvcr image needs PYTHONPATH unset and the LD_LIBRARY_PATH below for the venv's torch.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SONIC_ROOT=${SONIC_ROOT:-/opt/sonic}
LOGS=$SONIC_ROOT/logs
mkdir -p "$LOGS"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
export onnxruntime_DIR=/opt/onnxruntime/lib/cmake/onnxruntime
# Not the image's LD_LIBRARY_PATH: it puts the system (python 3.12) torch libraries first, which
# breaks the venv's torch, and lists the CUDA forward-compat directory.
export LD_LIBRARY_PATH=/opt/onnxruntime/lib:/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu
unset PYTHONPATH
export CUDA_HOME=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
export PYTHONUNBUFFERED=1
PLANNER_ARGS=(--planner-file planner/target_vel/V2/planner_sonic.onnx)
[ -n "${NO_PLANNER:-}" ] && PLANNER_ARGS=()  # NO_PLANNER=1 reproduces a deploy without its planner: the first ZMQ start stops it
# The motion the policy is parked on for ~100 ms at every switch into planner mode (zmq_manager.hpp polls the planner
# init every 100 ms): with reference/example/ that is a dance and the robot lunges and falls (FAILURE_LOG 167); the
# neutral standing motion from 3/eng/make_standing_ref.py is the default when present. REF_DIR overrides.
REF_DIR=${REF_DIR:-$([ -d "$SONIC_ROOT/gear_sonic_deploy/reference/standing" ] && echo reference/standing/ || echo reference/example/)}
DEPLOY_ARGS=(lo policy/release/model_decoder.onnx $REF_DIR
  --obs-config policy/release/observation_config.yaml
  --encoder-file policy/release/model_encoder.onnx
  "${PLANNER_ARGS[@]}"
  --input-type zmq_manager --output-type zmq
  --zmq-host 127.0.0.1 --zmq-port 5556 --zmq-out-port 5557 --disable-crc-check)
case "${1:-status}" in
  start)
    pkill -TERM -f "[s]im_camera_server.py" 2>/dev/null && sleep 1.5
    tmux kill-session -t sim2sim 2>/dev/null
    pkill -f "[g]antry.py" 2>/dev/null; rm -f "$LOGS"/sim.log "$LOGS"/deploy.log "$LOGS"/camera.log "$LOGS"/gantry.log  # no stale READY lines
    tmux new-session -d -s sim2sim -n sim "cd $SONIC_ROOT && exec $SONIC_ROOT/.venv_sim/bin/python $HERE/sim_headless.py --env-name default ${SCENE:+--scene $SCENE} ${LOCKSTEP:+--lockstep} ${SIM_DT:+--sim-dt $SIM_DT} ${SPAWN:+--spawn $SPAWN} ${DOOR_SEAL:+--door-seal $DOOR_SEAL} 2>&1 | tee $LOGS/sim.log"
    sleep 1
    if [ -n "${SCENE:-}" ]; then
      tmux new-window -t sim2sim -n camera "cd $SONIC_ROOT && MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa exec $SONIC_ROOT/.venv_sim/bin/python $HERE/sim_camera_server.py --scene $SCENE --snapshot-dir $LOGS/snap ${RECORD:+--record $RECORD} 2>&1 | tee $LOGS/camera.log"
    fi
    tmux new-window -t sim2sim -n deploy "cd $SONIC_ROOT/gear_sonic_deploy && SONIC_TIME_SCALE=${SONIC_TIME_SCALE:-1} exec ./target/release/g1_deploy_onnx_ref ${DEPLOY_ARGS[*]} ${EXTRA_DEPLOY_ARGS:-} 2>&1 | tee $LOGS/deploy.log"
    echo "started; logs: $LOGS/sim.log $LOGS/deploy.log" ;;
  stop)
    tmux send-keys -t sim2sim:deploy O 2>/dev/null
    pkill -TERM -f "[s]im_camera_server.py" 2>/dev/null; sleep 1.5  # lets the recording close its file
    tmux kill-session -t sim2sim 2>/dev/null; echo stopped ;;
  status)
    tmux ls 2>/dev/null; echo "--- sim:"; tail -5 "$LOGS/sim.log" 2>/dev/null; echo "--- camera:"; tail -3 "$LOGS/camera.log" 2>/dev/null; echo "--- deploy:"; tail -15 "$LOGS/deploy.log" 2>/dev/null ;;
esac
