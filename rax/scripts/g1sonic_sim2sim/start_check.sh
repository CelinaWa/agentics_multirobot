#!/bin/bash
# Validate the start procedure N times (default 3) inside the sim container: fresh stack with the
# scene, gantry.py lowering the robot first, dry_session.py's armed hold (start under the planner,
# settle, the gantry releases), one step, then height, tilt, yaw and drift 8, 20 and 40 s later.
# Usage (in the container): start_check.sh [N] ; env SCENE (default: the office scene)
N=${1:-3}
HERE=$(cd "$(dirname "$0")" && pwd)
SONIC_ROOT=${SONIC_ROOT:-/opt/sonic}
SCENE=${SCENE:-$SONIC_ROOT/gear_sonic/data/robot_model/model_data/g1/scene_office.xml}
export SCENE  # (the stability sampler below loads the same scene: a task scene has more free joints)
PY="env -u PYTHONPATH LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu $SONIC_ROOT/.venv_sim/bin/python"
for n in $(seq 1 "$N"); do
  echo "=== start check $n/$N"
  SCENE=$SCENE RECORD=${RECORD_DIR:+$RECORD_DIR/start_check_$n.mp4} bash "$HERE/run_sim2sim.sh" start >/dev/null
  for i in $(seq 1 60); do
    sleep 5
    grep -q "Init Done" "$SONIC_ROOT/logs/deploy.log" 2>/dev/null && grep -q READY "$SONIC_ROOT/logs/camera.log" 2>/dev/null && break
  done
  echo "  stack ready after $((i*5)) s"
  (cd "$SONIC_ROOT" && $PY "$HERE/gantry.py" > "$SONIC_ROOT/logs/gantry.log" 2>&1 &)
  sleep 4
  (cd "$SONIC_ROOT" && RAX_ARTIFACT_ROOT=/tmp/rax-startcheck timeout 240 $PY "$HERE/dry_session.py" --no-drop --camera 2>&1 \
     | grep "g1_hold armed\|g1_status: mode\|g1_step:\|g1_observe:\|rror" | cut -c1-160)
  cd "$SONIC_ROOT" && $PY - <<'PY'
import math, os, time, mujoco, numpy as np, zmq
m = mujoco.MjModel.from_xml_path(os.environ["SCENE"]); d = mujoco.MjData(m)
s = zmq.Context().socket(zmq.SUB); s.connect("tcp://127.0.0.1:5591"); s.setsockopt(zmq.SUBSCRIBE, b""); s.setsockopt(zmq.CONFLATE, 1); s.RCVTIMEO = 3000
def sample():
    d.qpos[:] = np.frombuffer(s.recv(), dtype=np.float64)
    w, x, y, z = d.qpos[3:7]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return d.qpos[0], d.qpos[1], d.qpos[2], tilt, yaw
first = None
for t in (8, 20, 40):
    time.sleep(t - (0 if first is None else prev))
    prev = t
    x, y, z, tilt, yaw = sample()
    first = first or (x, y, yaw)
    print(f"  {t:2d} s after the step: pelvis z {z:.2f} m, tilt {tilt:.1f} deg, yaw {yaw:+.1f} deg, "
          f"drift {math.hypot(x - first[0], y - first[1]):.2f} m / {yaw - first[2]:+.1f} deg")
PY
  tail -1 "$SONIC_ROOT/logs/gantry.log"
done
bash "$HERE/run_sim2sim.sh" stop >/dev/null
echo "start checks done"
