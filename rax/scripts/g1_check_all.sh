#!/usr/bin/env bash
# The first-day motion check of the G1 under SONIC, one step at a time (scripts/g1_motion_check.py), with a
# prompt between steps. Results: <session>/runtime/characterization/g1_motion_check.jsonl, for G1.md.
#   bash scripts/g1_check_all.sh <session_root>
# Needs: SONIC "Init Done", the console open on the same session (scripts/g1_console.py --session <root>),
# a person at the stop. The console keys work while a step says "waiting"; between steps they are ignored.
set -uo pipefail
ROOT=${1:?session root}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."
PY=${RAX_PYTHON:-python3}
export PYTHONPATH=$PWD
LOG=$ROOT/runtime/characterization/g1_check_all.log
mkdir -p "$(dirname "$LOG")"

declare -A WHAT=(
    [status]="moves nothing: prints the link to SONIC and the arming state"
    [disarmed]="moves nothing: one g1_step must be refused while disarmed"
    [hold]="moves nothing: 10 s of standing still, measuring drift and tilt"
    [arms]="MOVES: right then left arm, shoulder -0.3 / elbow +0.3 / wrist +0.3 rad, each out 1.5 s, hold, back 1.5 s (6 moves, ~35 s)"
    [waist]="MOVES: waist yaw +0.2 rad and back, waist pitch +0.15 rad (small lean) and back (~15 s)"
    [hands]="MOVES: right hand closes fully and opens, then the left (~10 s)"
    [pause]="MOVES: right shoulder pitch -0.5 rad over 1.8 s; YOU press SPACE as soon as it moves, the step measures the stop"
    [turn]="MOVES (steps!): warm-up turns +-15 deg, then +30 and -30 deg, twice (~60 s)"
    [walk]="MOVES (steps!): walks 1.0 m forward, twice; asks for the tape measure after each"
    [crouch]="MOVES: crouches 0.1, 0.2 then 0.3 m (3 s down, 2 s hold, 3 s up each); stops at the first level that tilts over 15 deg (~30 s)"
    [camera]="moves nothing: one observation, surface heights against your tape values"
)
step() {  # step <name> [args...]: say what the step does, wait for ENTER, run it; ask before going on if it failed
    echo; echo "=================== $1"
    echo "    ${WHAT[$1]}"
    read -rp ">>> ENTER to run '$1' (Ctrl-C stops the check; SPACE on the console pauses the robot at any time): " _
    if ! "$PY" -u scripts/g1_motion_check.py "$ROOT" "$@" 2>&1 | tee -a "$LOG"; then
        read -rp ">>> step $1 FAILED or was paused. ENTER to continue with the next step, Ctrl-C to stop: " _
    fi
}
ask() { echo; read -rp ">>> $1  [ENTER] " _; }

echo "motion check, session $ROOT" | tee -a "$LOG"
ask "Robot in the gantry, SONIC window shows 'Init Done' (it ramped to its standing pose while hanging). Console window open. Start?"
step status
step disarmed
ask "LOWER the gantry until the FEET CARRY the robot, the straps still lightly supporting it (after Init Done it is a rigid PD-held
    statue with no balance: it topples if the straps go fully slack now). Then console: press s. Within a few seconds after s,
    SLACKEN the straps completely (attached, no load: a strap pulling on the pelvis makes the balancing policy oscillate).
    Never press s with the feet off the floor (SONIC paddles its legs in the air). When it is still: press SPACE (arm). Then ENTER here."
step hold
step arms
step waist
step hands
step pause
ask "Slacken the straps (attached, no load). Clear 1.5 m around. Console: SPACE (arm again). ENTER."
step turn --repeat 2
ask "Mark the toes on the floor with tape. 2 m clear ahead. Straps slack. ENTER."
step walk --repeat 2 --forward 1.0
ask "Straps slack, 1 m clear around (the robot drifts back 10-15 cm while crouching). ENTER."
step crouch
echo
read -rp "Tape: pelvis height above the floor, standing (m), ENTER to skip: " PZ
read -rp "Tape: desk top above the floor (m), ENTER to skip: " DZ
step camera ${PZ:+--pelvis-z "$PZ"} ${DZ:+--desk-z "$DZ"}
echo
echo "=================== done. The table below goes into G1.md (also: $ROOT/runtime/characterization/g1_motion_check.jsonl)"
"$PY" scripts/g1_motion_check.py "$ROOT" report | tee -a "$LOG"
echo
echo "The robot keeps standing under SONIC. Next: either 'bash scripts/g1_up.sh agent --task-file task.md' (replaces the"
echo "console and work panes; do not press s again), or end the day: hang the robot, console o, then tmux kill-session -t g1."
