#!/usr/bin/env bash
# Wire this project's airframe and Gazebo model into a PX4-Autopilot checkout.
#
# Idempotent: safe to re-run. Uses symlinks, not copies, so regenerating from
# cad/params.py takes effect without re-installing -- the whole point of
# generating these files is that there is exactly one source of truth.
#
# Usage:  bash install_into_px4.sh [/path/to/PX4-Autopilot]
#         defaults to ~/PX4-Autopilot
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

AIRFRAME_FILE="4030_gz_tri_tiltrotor"
MODEL_NAME="tri_tiltrotor"

AIRFRAME_SRC="$HERE/airframes/$AIRFRAME_FILE"
MODEL_SRC="$HERE/models/$MODEL_NAME"
WORLD_SRC="$HERE/worlds/test.sdf"

AIRFRAME_DST_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
GZ_DIR="$PX4_DIR/Tools/simulation/gz"
MODEL_DST_DIR="$GZ_DIR/models"
WORLD_DST_DIR="$GZ_DIR/worlds"

fail() { echo "ERROR: $*" >&2; exit 1; }

[ -d "$PX4_DIR" ]        || fail "PX4 checkout not found at $PX4_DIR"
[ -f "$AIRFRAME_SRC" ]   || fail "missing $AIRFRAME_SRC (run cad/gen_airframe.py)"
[ -d "$MODEL_SRC" ]      || fail "missing $MODEL_SRC (run cad/gen_sdf.py)"
[ -d "$AIRFRAME_DST_DIR" ] || fail "not a PX4 tree: $AIRFRAME_DST_DIR missing"

echo "PX4 checkout : $PX4_DIR"
echo "project      : $HERE"
echo

# --- 1. Airframe -----------------------------------------------------------
ln -sfn "$AIRFRAME_SRC" "$AIRFRAME_DST_DIR/$AIRFRAME_FILE"
chmod +x "$AIRFRAME_SRC"
echo "  linked airframe -> $AIRFRAME_DST_DIR/$AIRFRAME_FILE"

# --- 2. Register the airframe in CMakeLists --------------------------------
# PX4 only bakes airframes listed here into ROMFS. Missing this step produces
# a build that succeeds and an airframe that silently does not exist.
CML="$AIRFRAME_DST_DIR/CMakeLists.txt"
if grep -q "$AIRFRAME_FILE" "$CML"; then
    echo "  already registered in CMakeLists.txt"
else
    # Insert alphabetically after the last 40xx entry so the list stays tidy.
    LAST_40=$(grep -nE "^\s+40[0-9][0-9]_" "$CML" | tail -1 | cut -d: -f1)
    [ -n "$LAST_40" ] || fail "could not find the 40xx block in $CML"
    cp "$CML" "$CML.bak-tritilt"
    sed -i "${LAST_40}a\\\t$AIRFRAME_FILE" "$CML"
    echo "  registered in CMakeLists.txt after line $LAST_40 (backup: $CML.bak-tritilt)"
fi

# --- 3. Gazebo model and world ---------------------------------------------
mkdir -p "$MODEL_DST_DIR" "$WORLD_DST_DIR"
ln -sfn "$MODEL_SRC" "$MODEL_DST_DIR/$MODEL_NAME"
echo "  linked model    -> $MODEL_DST_DIR/$MODEL_NAME"

if [ -f "$WORLD_SRC" ]; then
    ln -sfn "$WORLD_SRC" "$WORLD_DST_DIR/tri_tiltrotor_test.sdf"
    echo "  linked world    -> $WORLD_DST_DIR/tri_tiltrotor_test.sdf"
fi

echo
echo "Installed. Rebuild and run with:"
echo "    cd $PX4_DIR"
echo "    make px4_sitl gz_${MODEL_NAME}"
