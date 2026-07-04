#!/usr/bin/env bash
set -eo pipefail

# Flash a built ESP32-S3 project and capture `idf.py monitor` output for a
# fixed duration (monitor never exits on its own), then write an
# ANSI-stripped log file that's easy to read/grep afterward.
#
# Usage: flash_monitor.sh <data_capture|imu_testing> [port] [duration_seconds] [log_path]

PROJECT="${1:?usage: flash_monitor.sh <data_capture|imu_testing> [port] [duration_seconds] [log_path]}"
PORT="${2:-/dev/ttyACM0}"
DUR="${3:-30}"
LOG="${4:-/tmp/esp32-idf/${PROJECT}.log}"

IDF_ENV="$HOME/.espressif/tools/activate_idf_v5.3.5.sh"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PROJ_DIR="$REPO_ROOT/firmware/$PROJECT"

[ -d "$PROJ_DIR" ] || { echo "no such project dir: $PROJ_DIR" >&2; exit 1; }

mkdir -p "$(dirname "$LOG")"

# `script` gives idf.py monitor a pty so it behaves as if run interactively;
# `timeout -s SIGINT` sends the same signal Ctrl+C would after $DUR seconds so
# monitor exits cleanly instead of being killed mid-write.
# See build.sh for why we call the venv python + idf.py tool script
# directly instead of relying on the idf.py alias.
timeout -s SIGINT "$DUR" \
  script -qec "bash -c 'source \"$IDF_ENV\" && cd \"$PROJ_DIR\" && \"\$IDF_PYTHON_ENV_PATH/bin/python\" \"\$IDF_PATH/tools/idf.py\" -p \"$PORT\" flash monitor'" "$LOG" || true

CLEAN_LOG="${LOG%.log}_clean.log"
sed -e 's/\x1b\[[0-9;]*[A-Za-z]//g' -e 's/\r//g' "$LOG" > "$CLEAN_LOG"
echo "$CLEAN_LOG"
