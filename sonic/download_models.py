#!/usr/bin/env python3
"""Download the SONIC v1.1 model set we validated in sim2sim into a gear_sonic_deploy checkout.

    python download_models.py <gear_sonic_deploy dir>

nvidia/GEAR-SONIC on Hugging Face is not gated; no token needed. The fork's own download_from_hf.py fetches the
top-level (older) encoder/decoder, which is NOT the set our sim2sim runs used, so the files are taken from
sonic_v1_1/ here and checked against models.sha256 afterwards (setup_rax_host.sh does the check)."""
import shutil
import sys
from pathlib import Path

from huggingface_hub import hf_hub_download

REPO = "nvidia/GEAR-SONIC"
FILES = {  # hf filename -> path under gear_sonic_deploy
    "sonic_v1_1/model_encoder.onnx": "policy/release/model_encoder.onnx",
    "sonic_v1_1/model_decoder.onnx": "policy/release/model_decoder.onnx",
    "sonic_v1_1/observation_config.yaml": "policy/release/observation_config.yaml",
    "planner_sonic.onnx": "planner/target_vel/V2/planner_sonic.onnx",
}


def main(deploy_dir: str) -> int:
    root = Path(deploy_dir).resolve()
    for hf_name, rel in FILES.items():
        dest = root / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        cached = hf_hub_download(REPO, hf_name)
        shutil.copyfile(cached, dest)
        print(f"{hf_name} -> {dest} ({dest.stat().st_size / 1e6:.0f} MB)", flush=True)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
