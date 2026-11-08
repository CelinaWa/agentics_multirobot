#!/bin/bash
# Build a SEPARATE mode-1 deploy binary on the R1 Jetson, without committing to or leaving changes in the lab repos.
#
#   jetson_mode1.sh check    the pinned source is the one the patch was made for, and the patch applies
#   jetson_mode1.sh build    apply the patch, build into build/rax-mode1, save target/release/r1_deploy_onnx_ref_mode1,
#                            then reverse the patch and restore the official binary (the source tree is clean again)
#   jetson_mode1.sh remove   delete the mode-1 binary and its build folder
#
# The official target/release/r1_deploy_onnx_ref is never left modified. Run the mode-1 binary with the same arguments
# as the official one, and start the bridge with --send-vr-points (README 8.7).
set -euo pipefail
DEPLOY=${DEPLOY:-$HOME/MA-Teleop/submodules/GR00T-WholeBodyControl/gear_sonic_deploy}
PATCH=${PATCH:-$(cd "$(dirname "$0")" && pwd)/protocol1_vr_position_mode1.patch}
PINNED=50aec78119c974daffc607a3644a75c716147031      # MA-Teleop junjie's submodule pin, the patch's base
SRC=$(cd "$DEPLOY/.." && pwd)
BIN=$DEPLOY/target/release/r1_deploy_onnx_ref
OUT=$DEPLOY/target/release/r1_deploy_onnx_ref_mode1
BUILD=$DEPLOY/build/rax-mode1

check() {
    [ -f "$PATCH" ] || { echo "no patch at $PATCH"; exit 1; }
    rev=$(git -C "$SRC" rev-parse HEAD)
    [ "$rev" = "$PINNED" ] || { echo "source is at $rev, the patch was made for $PINNED: stop and compare first"; exit 1; }
    git -C "$SRC" diff --quiet HEAD -- gear_sonic_deploy || { echo "gear_sonic_deploy has local changes: stop"; exit 1; }
    git -C "$SRC" apply --check "$PATCH"
    [ -x "$BIN" ] || { echo "no official binary at $BIN"; exit 1; }
    echo "ok: source $rev clean, patch applies, official binary $(sha256sum "$BIN" | cut -c1-16)"
}

build() {
    check
    sum=$(sha256sum "$BIN" | cut -d' ' -f1)
    cp -p "$BIN" "$BIN.rax-keep"
    git -C "$SRC" apply "$PATCH"
    restore() {                                     # always: source clean again, official binary back
        git -C "$SRC" apply -R "$PATCH" 2>/dev/null || true
        [ -f "$BIN.rax-keep" ] && mv -f "$BIN.rax-keep" "$BIN"
    }
    trap restore EXIT
    # the flags of MA-Teleop tools/r1/build_onboard.py, in our own build folder
    HAS_ROS2=0 cmake -S "$DEPLOY" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
        "-DTensorRT_FIND_COMPONENTS=nvinfer;nvinfer_plugin;nvonnxparser" -DTensorRT_INCLUDE_DIR=/usr/include/aarch64-linux-gnu \
        -Donnxruntime_INCLUDE_DIR=/opt/onnxruntime/include -Donnxruntime_LIBRARY=/opt/onnxruntime/lib/libonnxruntime.so \
        -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/usr/src/googletest
    HAS_ROS2=0 cmake --build "$BUILD" --target r1_deploy_onnx_ref -j"${JOBS:-3}"
    mv -f "$BIN" "$OUT"                             # the build wrote target/release/r1_deploy_onnx_ref
    restore; trap - EXIT
    [ "$(sha256sum "$BIN" | cut -d' ' -f1)" = "$sum" ] || { echo "official binary changed: check!"; exit 1; }
    git -C "$SRC" diff --quiet HEAD -- gear_sonic_deploy || { echo "source not clean after the build: check!"; exit 1; }
    echo "built $OUT ($(sha256sum "$OUT" | cut -c1-16)); official binary and source unchanged"
}

remove() { rm -f "$OUT"; rm -rf "$BUILD"; echo "removed the mode-1 binary and $BUILD"; }

case "${1:-}" in check) check ;; build) build ;; remove) remove ;; *) sed -n 2,12p "$0"; exit 1 ;; esac
