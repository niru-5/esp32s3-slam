#!/usr/bin/env bash
# Copy the esp-dl model file(s) onto a mounted SD card, in the layout the
# firmware expects when "model location" is set to "sdcard" in menuconfig.
#
# Usage: tools/prepare_sdcard.sh /media/$USER/SDCARD
#
# Result on the card:   <card>/models/s3/espdet_pico_224_224_hand.espdl
#
# The .espdl file comes from the espressif/hand_detect component that the IDF
# component manager downloads on first build (managed_components/), so run
# `idf.py build` once before this. The card must be FAT32. Do not rename the
# file: esp-dl looks it up by exact name.
set -euo pipefail

CARD="${1:?usage: $0 <sd-card-mount-point>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$HERE/managed_components/espressif__hand_detect/models/s3"
MODEL="espdet_pico_224_224_hand.espdl"

[ -d "$CARD" ]        || { echo "not a directory: $CARD" >&2; exit 1; }
[ -f "$SRC/$MODEL" ]  || { echo "missing $SRC/$MODEL -- run 'idf.py build' once first" >&2; exit 1; }

mkdir -p "$CARD/models/s3"
cp -v "$SRC/$MODEL" "$CARD/models/s3/$MODEL"
sync
echo "done. Eject the card, insert it in the board, then in menuconfig set"
echo "  models: hand_detect -> model location -> sdcard   and rebuild/flash."
