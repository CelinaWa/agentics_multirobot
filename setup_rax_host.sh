#!/usr/bin/env bash
# One-time setup of the RAX host (the same machine runs SONIC in this layout). Creates the two Python envs,
# installs the requirements and drops the settings templates in $HOME. Re-runnable.
#   bash setup_rax_host.sh                 # envs under $HOME/envs
#   bash setup_rax_host.sh --with-sonic    # also clone + build SONIC and fetch its models (sonic/SONIC_SETUP.md)
#   ENVS=/data/envs SONIC_DIR=/data/GR00T-WholeBodyControl bash setup_rax_host.sh --with-sonic
set -euo pipefail
WITH_SONIC=0; [ "${1:-}" = "--with-sonic" ] && WITH_SONIC=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENVS=${ENVS:-$HOME/envs}
PY=${PYTHON:-python3}
command -v "$PY" >/dev/null || { echo "need python3 (3.10+)"; exit 1; }
command -v tmux >/dev/null || echo "WARNING: tmux missing (sudo apt install tmux) - scripts/g1_up.sh needs it"

mkdir -p "$ENVS"
for env in rax-g1:requirements-robot.txt rax-agent:requirements-agent.txt; do
    name=${env%%:*}; req=${env##*:}
    [ -x "$ENVS/$name/bin/python" ] || "$PY" -m venv "$ENVS/$name"
    "$ENVS/$name/bin/python" -m pip install --upgrade pip wheel >/dev/null
    "$ENVS/$name/bin/python" -m pip install -r "$HERE/$req"
    echo "env ready: $ENVS/$name"
done

# Claude Code CLI for the agent (only needed for 'g1_up.sh agent'; the motion check does not use it)
if ! command -v claude >/dev/null; then
    echo "NOTE: 'claude' CLI not found. Install: npm install -g @anthropic-ai/claude-code  (needs node >= 18)"
fi

# settings templates (never commit the filled-in copies)
[ -f "$HOME/.g1_up.env" ] || { sed "s|\$HOME/envs|$ENVS|g" "$HERE/g1_up.env.same_host.example" > "$HOME/.g1_up.env"; echo "wrote ~/.g1_up.env - edit the IPs/paths"; }
[ -f "$HOME/g1_real_options.json" ] || { cp "$HERE/docs/g1_real_options.example.json" "$HOME/g1_real_options.json"; echo "wrote ~/g1_real_options.json - fill in state_host (127.0.0.1 for same-host SONIC) and camera_host"; }

# unit tests that need no robot
"$ENVS/rax-g1/bin/python" -m pytest -q "$HERE/rax/local_tests/unit/test_g1sonic_backend.py" -x 2>&1 | tail -3 || true

if [ "$WITH_SONIC" = 1 ]; then
    SONIC_DIR=${SONIC_DIR:-$HOME/GR00T-WholeBodyControl}
    COMMIT=$(cat "$HERE/sonic/GR00T_WBC_COMMIT.txt")
    echo "=== SONIC: $SONIC_DIR @ ${COMMIT:0:7}"
    [ -d "$SONIC_DIR/.git" ] || git clone https://github.com/Jay-Ye/GR00T-WholeBodyControl.git "$SONIC_DIR"
    git -C "$SONIC_DIR" fetch -q origin && git -C "$SONIC_DIR" checkout -q "$COMMIT"
    DEPLOY="$SONIC_DIR/gear_sonic_deploy"
    # the standing-only reference the console's 's' uses
    mkdir -p "$DEPLOY/reference/standing" && cp -r "$HERE/sonic/reference_standing/stand_000" "$DEPLOY/reference/standing/"
    # the v1.1 model set we validated (the checkout's own observation_config.yaml is the OLD v1.0 one and gets replaced)
    "$ENVS/rax-g1/bin/python" -m pip install -q "huggingface_hub>=0.23"
    "$ENVS/rax-g1/bin/python" "$HERE/sonic/download_models.py" "$DEPLOY"
    (cd "$DEPLOY" && sha256sum -c "$HERE/sonic/models.sha256") || { echo "MODEL HASH MISMATCH: do not deploy"; exit 1; }
    # C++ deps (apt/sudo: unitree_sdk2, yaml-cpp, zmq, cmake, clang, just) and the build. Needs CUDA + TensorRT 10
    # already installed (the sim container had TensorRT 10.13 / CUDA 12.9); install_deps.sh does not install those.
    (cd "$DEPLOY" && chmod +x scripts/install_deps.sh && ./scripts/install_deps.sh)
    (cd "$DEPLOY" && set +e && source scripts/setup_env.sh && set -e && just build)
    echo "SONIC built. The TensorRT engines (*.trt) are generated on the first deploy.sh run (several minutes)."
    [ -f "$HOME/.g1_up.env" ] && sed -i "s|^SONIC_DEPLOY_DIR=.*|SONIC_DEPLOY_DIR=$DEPLOY|" "$HOME/.g1_up.env"
fi
echo "done. Next: 'bash rax/scripts/g1_up.sh check' on the first robot day (README.md)"
[ "$WITH_SONIC" = 1 ] || echo "SONIC not set up: re-run with --with-sonic, or follow sonic/SONIC_SETUP.md by hand"
