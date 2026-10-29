#!/usr/bin/env bash
# Bring up every window of the real-G1 procedure in one tmux session, and say where you still press a key.
#
#   bash scripts/g1_up.sh check                      # first day / after an upgrade: the motion check, no agent
#   bash scripts/g1_up.sh agent --task-file task.md  # a task session: robot service + Claude agent
#   bash scripts/g1_up.sh agent --session <dir>      # reuse an existing session directory
#   bash scripts/g1_up.sh agent --task-file task.md --quick-check   # 1-minute check first, then the agent
#
# Run again while the tmux session is up (after the check, or for the next task): only the console and work
# windows are replaced; camera, SONIC and the dashboard keep running. Do NOT press `s` again then: the robot
# already stands under SONIC; the agent's first hold takes it over where it is. SPACE arms it.
#
# Default: ONE tmux window split into 5 panes (work pane large on the left; camera, sonic, console, dashboard
# stacked on the right; Ctrl-b + arrow keys move the cursor between panes, keys go to the pane with the cursor).
# --windows puts each in its own tab instead (Ctrl-b 1..5).
# Settings come from ~/.g1_up.env (template: scripts/g1_up.env.example; G1_UP_ENV overrides the path).
# Before this: the robot hangs in the gantry, is powered on and self-checked, every other control program is
# off. This script never moves the robot: SONIC asks "Proceed? [Y/n]" in its own window and nothing stands up
# before you press `s` on the console.
set -euo pipefail
MODE=${1:-}; shift || true
[[ "$MODE" == check || "$MODE" == agent ]] || { sed -n 2,12p "$0"; exit 2; }
TASK_FILE=""; SESSION=""; QUICK=""; WINDOWS=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --task-file) TASK_FILE=$2; shift 2 ;;
        --session) SESSION=$2; shift 2 ;;
        --quick-check) QUICK=1; shift ;;
        --windows) WINDOWS=1; shift ;;
        *) echo "unknown argument $1" >&2; exit 2 ;;
    esac
done
ENV_FILE=${G1_UP_ENV:-$HOME/.g1_up.env}
[[ -f "$ENV_FILE" ]] || { echo "missing $ENV_FILE: copy scripts/g1_up.env.example there and fill it in" >&2; exit 2; }
# shellcheck disable=SC1090
source "$ENV_FILE"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export RAX_PYTHON RAX_AGENT_PYTHON RAX_PROTECTED_PATHS PYTHONPATH="$PROJECT_ROOT"
TMUX_SESSION=g1

say() { printf '\033[1m%s\033[0m\n' "$*"; }
fail() { echo "ERROR: $*" >&2; exit 1; }

# ---- preflight: nothing here touches the robot ----------------------------------------------
say "== preflight"
command -v tmux >/dev/null || fail "tmux is not installed"
[[ -x "$RAX_PYTHON" ]] || fail "RAX_PYTHON=$RAX_PYTHON is not executable"
[[ -x "$RAX_AGENT_PYTHON" ]] || fail "RAX_AGENT_PYTHON=$RAX_AGENT_PYTHON is not executable"
[[ -f "$OPTIONS_JSON" ]] || fail "OPTIONS_JSON=$OPTIONS_JSON not found"
grep -q '<' "$OPTIONS_JSON" && fail "$OPTIONS_JSON still has a <placeholder>: fill in the IPs"
REUSE=""
if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    REUSE=1
    echo "tmux session '$TMUX_SESSION' is up: camera, SONIC and the dashboard stay; console and work are replaced"
fi
PING_FROM=(); [[ -n "${SONIC_SSH:-}" ]] && PING_FROM=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$SONIC_SSH")
if "${PING_FROM[@]}" ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "robot network ok: $ROBOT_IP answers${SONIC_SSH:+ (from $SONIC_SSH)}"
else
    fail "$ROBOT_IP does not answer${SONIC_SSH:+ from $SONIC_SSH}: robot off, or not on the 192.168.123.x network"
fi
ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_SSH" true 2>/dev/null || fail "cannot ssh to $JETSON_SSH without a password (set up a key)"
if [[ -n "${SONIC_SSH:-}" ]]; then
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$SONIC_SSH" "test -x '$SONIC_DEPLOY_DIR/deploy.sh' && test -d '$SONIC_DEPLOY_DIR/$SONIC_REF_DIR'" \
        || fail "on $SONIC_SSH: $SONIC_DEPLOY_DIR/deploy.sh or $SONIC_REF_DIR is missing"
else
    [[ -x "$SONIC_DEPLOY_DIR/deploy.sh" ]] || fail "$SONIC_DEPLOY_DIR/deploy.sh is missing"
    [[ -d "$SONIC_DEPLOY_DIR/$SONIC_REF_DIR" ]] || fail "$SONIC_DEPLOY_DIR/$SONIC_REF_DIR is missing (copy reference/standing/ there)"
fi

# ---- session directory ----------------------------------------------------------------------
say "== session"
if [[ -z "$SESSION" ]]; then
    if [[ "$MODE" == check ]]; then
        SESSION=$("$RAX_PYTHON" "$SCRIPT_DIR/configure_session.py" --task motion_check --task-text "motion check" \
                  --robot g1sonic --observation-profile rgbd --backend-options "$OPTIONS_JSON" --agent-python "$RAX_AGENT_PYTHON")
    else
        [[ -n "$TASK_FILE" ]] || fail "agent mode needs --task-file task.md (or --session <dir>)"
        [[ -f "$TASK_FILE" ]] || fail "$TASK_FILE not found"
        TASK_ID=$(basename "$TASK_FILE" .md | tr -c 'A-Za-z0-9_\n' '_')
        SESSION=$("$RAX_PYTHON" "$SCRIPT_DIR/configure_session.py" --task "$TASK_ID" --task-file "$TASK_FILE" \
                  --robot g1sonic --observation-profile rgbd --backend-options "$OPTIONS_JSON" --agent-python "$RAX_AGENT_PYTHON")
    fi
    SESSION=$(dirname "$SESSION")   # configure_session prints <root>/operator-input.json
fi
[[ -f "$SESSION/operator-input.json" ]] || fail "$SESSION has no operator-input.json"
echo "session: $SESSION"

# ---- windows ----------------------------------------------------------------------------------
WORK_CMD="bash scripts/start_agent.sh --config '$SESSION/operator-input.json'"
[[ "$MODE" == check ]] && WORK_CMD="bash scripts/g1_check_all.sh '$SESSION'"
[[ -n "$QUICK" && "$MODE" == agent ]] && WORK_CMD="bash scripts/g1_check_quick.sh '$SESSION' && $WORK_CMD"
CONSOLE_CMD="'$RAX_PYTHON' scripts/g1_console.py --session '$SESSION'"
pane_of() { tmux list-panes -s -t "$TMUX_SESSION" -F '#{pane_id} #{pane_title}' | awk -v t="$1" '$2 == t {print $1; exit}'; }
hold() { echo "bash -c \"$1; echo; echo '[exited: $2]'; exec bash\""; }   # a finished program leaves a shell, not a dead pane
if [[ -n "$REUSE" ]]; then
    say "== replacing the console and work panes"
    for pair in "console|$CONSOLE_CMD" "work|$WORK_CMD"; do
        name=${pair%%|*}; cmd=${pair#*|}
        pane=$(pane_of "$name")
        [[ -n "$pane" ]] || fail "no pane titled '$name' in tmux session $TMUX_SESSION"
        tmux respawn-pane -k -t "$pane" -c "$PROJECT_ROOT" "$(hold "$cmd" "$name")"
        tmux select-pane -t "$pane" -T "$name"
    done
    echo "console and work point at $SESSION. Do not press s again (the robot already stands); SPACE arms."
    echo "attach: tmux attach -t $TMUX_SESSION"
    exit 0
fi
say "== tmux session '$TMUX_SESSION'"
DEPLOY_CMD="cd '$SONIC_DEPLOY_DIR' && ./deploy.sh real --input-type zmq_manager --output-type zmq --planner '$SONIC_PLANNER' --motion-data '$SONIC_REF_DIR' --zmq-host '$RAX_HOST_IP'"
CAMERA_CMD="ssh -t $JETSON_SSH \"$JETSON_CAMERA_CMD\""
SONIC_CMD=$DEPLOY_CMD; [[ -n "${SONIC_SSH:-}" ]] && SONIC_CMD="ssh -t $SONIC_SSH \"$DEPLOY_CMD\""
DASH_CMD="bash scripts/run_dashboard.sh"
if [[ -n "$WINDOWS" ]]; then
    tmux new-session -d -s "$TMUX_SESSION" -n camera -c "$PROJECT_ROOT" "$(hold "$CAMERA_CMD" camera)"
    tmux new-window -t "$TMUX_SESSION" -n sonic -c "$PROJECT_ROOT" "$(hold "$SONIC_CMD" sonic)"
    tmux new-window -t "$TMUX_SESSION" -n console -c "$PROJECT_ROOT" "$(hold "$CONSOLE_CMD" console)"
    tmux new-window -t "$TMUX_SESSION" -n dashboard -c "$PROJECT_ROOT" "$(hold "$DASH_CMD" dashboard)"
    tmux new-window -t "$TMUX_SESSION" -n work -c "$PROJECT_ROOT" "$(hold "$WORK_CMD" work)"
    for w in camera sonic console dashboard work; do tmux select-pane -t "$TMUX_SESSION:$w" -T "$w"; done
    tmux select-window -t "$TMUX_SESSION:work"
else
    tmux new-session -d -s "$TMUX_SESSION" -n g1 -c "$PROJECT_ROOT" "$(hold "$WORK_CMD" work)"
    WORK_PANE=$(tmux display-message -p -t "$TMUX_SESSION" '#{pane_id}')
    tmux select-pane -t "$WORK_PANE" -T work
    for pair in "camera|$CAMERA_CMD" "sonic|$SONIC_CMD" "console|$CONSOLE_CMD" "dashboard|$DASH_CMD"; do
        name=${pair%%|*}; cmd=${pair#*|}
        P=$(tmux split-window -v -P -F '#{pane_id}' -t "$TMUX_SESSION" -c "$PROJECT_ROOT" "$(hold "$cmd" "$name")")
        tmux select-pane -t "$P" -T "$name"
    done
    tmux set-option -t "$TMUX_SESSION" pane-border-status top >/dev/null
    tmux set-option -t "$TMUX_SESSION" pane-border-format ' #{pane_title} ' >/dev/null
    tmux set-window-option -t "$TMUX_SESSION" main-pane-width 55% >/dev/null   # work pane: left 55 %; the four others stacked right
    tmux select-layout -t "$TMUX_SESSION" main-vertical >/dev/null
    tmux set-hook -t "$TMUX_SESSION" client-attached "select-layout -t $TMUX_SESSION main-vertical" >/dev/null 2>&1 || true
    tmux select-pane -t "$WORK_PANE"
fi

cat <<EOF

$(say "== up. One screen, five panes (or tabs with --windows). Ctrl-b + arrow keys move the cursor; keys go to the pane under the cursor.")
  camera     nothing (Jetson camera server; type the password if it asks)
  sonic      Y   when it asks "Proceed with deployment? [Y/n]"; then wait for "Init Done". Touch nothing else here.
             Its O key is the outermost software stop (every joint goes limp: only while hanging).
  console    s      robot stands in the gantry (after Init Done); then lower the gantry until the feet carry it
             SPACE  toggles the key: 1st press = arm (motion allowed), next press = pause, next = resume
             o      stop (hangs limp; relaunch deploy.sh afterwards)
  dashboard  nothing (open the URL it prints in a browser)
  work       $( [[ "$MODE" == check ]] && echo "the motion check. It prints '>>> ... [ENTER]' and waits: do what it says, then press ENTER here." \
                                       || echo "the agent: this is where you talk to it; Ctrl-C ends the session." )

  attach:  tmux attach -t $TMUX_SESSION        detach (leaves everything running): Ctrl-b d        session dir: $SESSION
EOF
