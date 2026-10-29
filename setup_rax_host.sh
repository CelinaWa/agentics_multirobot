#!/usr/bin/env bash
# One-time setup of the RAX host (the same machine runs SONIC in this layout). Creates the two Python envs,
# installs the requirements and drops the settings templates in $HOME. Re-runnable.
#   bash setup_rax_host.sh            # envs under $HOME/envs
#   ENVS=/data/envs bash setup_rax_host.sh
set -euo pipefail
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
echo "done. Next: build SONIC (sonic/SONIC_SETUP.md), then 'bash rax/scripts/g1_up.sh check'"
