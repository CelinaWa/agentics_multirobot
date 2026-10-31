#!/usr/bin/env bash
# Pack what a finished session learned, for the next machine: its memory/, handoff/ and code/ plus a git
# bundle of the robot's skill library. Run where the session ran (the sim host or the robot host):
#   bash scripts/export_experience.sh <session root> <destination dir>
# Then commit <destination dir> (e.g. experience/<task>/ in the release repo), and on the other machine
#   bash scripts/import_experience.sh <that dir>          # loads the library bundle into this host's library
#   bash scripts/g1_up.sh agent --task-file task.md --experience <that dir>/workspace
set -euo pipefail
ROOT=${1:?session root}; DEST=${2:?destination dir}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=$ROOT/agent-workspace
[[ -d "$WS" ]] || { echo "$ROOT has no agent-workspace" >&2; exit 2; }
mkdir -p "$DEST/workspace"
for d in memory handoff code; do
    [[ -d "$WS/$d" ]] || continue
    rm -rf "$DEST/workspace/$d"
    # text only: frames and logs stay with the session (the skill library carries its own evidence images)
    python3 - "$WS/$d" "$DEST/workspace/$d" <<'PY'
import shutil, sys
shutil.copytree(sys.argv[1], sys.argv[2], symlinks=False,
                ignore=shutil.ignore_patterns('__pycache__', '*.png', '*.jpg', '*.jpeg', '*.npy', '*.npz', '*.mp4', '*.pyc'))
PY
done
ROBOT=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['robot'])" "$ROOT/configuration.json")
LIB=${RAX_SKILL_LIBRARY:-$(dirname "$(dirname "$(realpath "$ROOT")")")/skill-library}/$ROBOT
if [[ -d "$LIB/.git" ]]; then
    git -C "$LIB" bundle create "$DEST/skill-library-$ROBOT.bundle" main 2>/dev/null
    echo "skill library $LIB ($(git -C "$LIB" rev-parse --short main)) -> $DEST/skill-library-$ROBOT.bundle"
else
    echo "no skill library at $LIB (nothing merged yet)"
fi
{ echo "session: $(basename "$ROOT")"; echo "robot: $ROBOT"; echo "exported: $(date -u +%Y-%m-%dT%H:%M:%SZ)";
  echo "host: $(hostname)"; } > "$DEST/SOURCE.txt"
# the session directory is private to the harness user (often root); hand the export to the repository's owner
if [[ $(id -u) -eq 0 ]]; then chown -R "$(stat -c %u:%g "$(dirname "$(realpath "$DEST")")")" "$DEST"; fi
echo "exported to $DEST:"; find "$DEST" -type f | sed "s#^$DEST/#  #"
