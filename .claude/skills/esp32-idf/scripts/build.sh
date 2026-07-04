#!/usr/bin/env bash
set -eo pipefail

# Build one of the ESP32-S3 firmware projects.
# Usage: build.sh <data_capture|imu_testing>

PROJECT="${1:?usage: build.sh <data_capture|imu_testing>}"
IDF_ENV="$HOME/.espressif/tools/activate_idf_v5.3.5.sh"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PROJ_DIR="$REPO_ROOT/firmware/$PROJECT"

[ -d "$PROJ_DIR" ] || { echo "no such project dir: $PROJ_DIR" >&2; exit 1; }

# activate_idf_v5.3.5.sh refuses to run unless it detects it's being sourced
# from an interactive-shell-like $0 (bash/sh/etc). Sourcing it directly from
# inside this script file fails that check, so source it inside a nested
# `bash -c` instead, where $0 is "bash".
#
# It also only defines idf.py as a shell alias, which (a) needs
# `expand_aliases` in a non-interactive shell and (b) still wouldn't expand
# here anyway, since an alias defined earlier in the same &&-chained command
# line isn't visible to later words on that same line. Call the venv python
# + idf.py tool script directly instead, via the env vars the activate
# script exports (IDF_PYTHON_ENV_PATH, IDF_PATH) — those work immediately.
bash -c "source \"$IDF_ENV\" && cd \"$PROJ_DIR\" && \"\$IDF_PYTHON_ENV_PATH/bin/python\" \"\$IDF_PATH/tools/idf.py\" build"
