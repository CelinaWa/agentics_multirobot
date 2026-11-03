#!/usr/bin/env bash
# Load an exported skill-library bundle into this host's library (RAX_SKILL_LIBRARY or <artifact root>/skill-library),
# so the next session's skill-library/ clone contains the skills learned elsewhere (e.g. in simulation).
#   bash scripts/import_experience.sh <exported dir>      # the dir export_experience.sh wrote
# The memory/handoff part needs no import: pass it to the session with --experience <exported dir>/workspace.
set -euo pipefail
SRC=$(realpath "${1:?exported dir}")   # absolute: the merge below runs git inside the library
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT=${RAX_ARTIFACT_ROOT:-$SCRIPT_DIR/../runtime}
LIBS=${RAX_SKILL_LIBRARY:-$ROOT/skill-library}
shopt -s nullglob
bundles=("$SRC"/skill-library-*.bundle)
[[ ${#bundles[@]} -gt 0 ]] || { echo "no skill-library-*.bundle in $SRC (memory/handoff only: use --experience $SRC/workspace)"; exit 0; }
for b in "${bundles[@]}"; do
    robot=$(basename "$b" .bundle); robot=${robot#skill-library-}
    lib=$LIBS/$robot
    git bundle verify "$b" >/dev/null
    if [[ -d "$lib/.git" ]]; then
        git -C "$lib" fetch -q "$b" main:imported
        git -C "$lib" merge -q --no-edit imported
        git -C "$lib" branch -q -D imported
        echo "merged $b into $lib ($(git -C "$lib" rev-parse --short main))"
    else
        mkdir -p "$LIBS"; git clone -q -b main "$b" "$lib"   # the bundle carries main only (no HEAD)
        git -C "$lib" config user.name 'rax harness'; git -C "$lib" config user.email harness@rax.local
        echo "cloned $b into $lib ($(git -C "$lib" rev-parse --short main))"
    fi
done
echo "next: bash scripts/g1_up.sh agent --task-file <task.md> --experience $(realpath "$SRC")/workspace"
