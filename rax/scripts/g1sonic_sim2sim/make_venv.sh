#!/bin/bash
# The MuJoCo sim2sim venv next to gear_sonic (SONIC_ROOT, default /opt/sonic): CPU torch, gear_sonic[sim],
# the Unitree SDK and what sim_camera_server.py / probe.py need. Needs uv on PATH.
export PATH=$HOME/.local/bin:$PATH
cd "${SONIC_ROOT:-/opt/sonic}" || exit 1
rm -rf .venv_sim
set -e
uv venv .venv_sim --python 3.10
uv pip install --python .venv_sim/bin/python torch --index-url https://download.pytorch.org/whl/cpu
uv pip install --python .venv_sim/bin/python -e "gear_sonic[sim]"
uv pip install --python .venv_sim/bin/python -e external_dependencies/unitree_sdk2_python
uv pip install --python .venv_sim/bin/python pyzmq msgpack "numpy==1.26.4" scipy opencv-python-headless pillow pytest pyyaml
echo "venv ok"
