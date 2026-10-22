#!/usr/bin/env bash
# px4_launch_sims.sh [N]   start Gazebo + N PX4 x500 quadcopters (default 2). Ctrl+C stops everything.
#
# Instance i  ->  MAVLink offboard/API port 14540+i  (MAVSDK connects here)
#                 PX4 onboard listen port 14580+i     (px4_gps_spoofer.py sends here)
#
# Default world: drone 0 at (0,0), drone 1 at (15,-15), so the default "two" layout paths cross at (15,0).
# Campus world:  export PX4_GZ_WORLD=kent_campus first (see campus/make_campus_world.py). Spawn points are then
#                read from campus/layout.json.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N="${1:-2}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
LAYOUT="${LAYOUT:-$SCRIPT_DIR/campus/layout.json}"
[ -x "$BIN" ] || { echo "PX4 not built: $BIN (run px4_setup.sh)"; exit 1; }

POSES=("0,0" "15,-15" "-15,-15" "30,0")
if [ -n "${PX4_GZ_WORLD:-}" ] && [ -f "$LAYOUT" ]; then
  for i in 0 1 2 3; do
    p="$(python3 - "$LAYOUT" "$PX4_GZ_WORLD" "$i" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
i = int(sys.argv[3])
if d.get("world") == sys.argv[2] and i < len(d["drones"]):
    print("%s,%s" % tuple(d["drones"][i]["spawn"]))
PY
)"
    [ -n "$p" ] && POSES[$i]="$p"
  done
  echo "world: $PX4_GZ_WORLD   spawn poses: ${POSES[*]}"
fi

cd "$PX4_DIR"
trap 'echo stopping...; pkill -P $$ || true; pkill -f "bin/px4" || true; exit 0' INT TERM EXIT

# instance 0 also starts Gazebo and the world
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="${POSES[0]},0,0,0,0" \
  "$BIN" -i 0 -d > /tmp/px4_0.log 2>&1 &
echo "started instance 0 (log /tmp/px4_0.log) - waiting for Gazebo..."
sleep 15

for i in $(seq 1 $((N - 1))); do
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="${POSES[$i]},0,0,0,0" \
    "$BIN" -i "$i" -d > "/tmp/px4_$i.log" 2>&1 &
  echo "started instance $i (log /tmp/px4_$i.log)"
  sleep 6
done
echo "all running. Ctrl+C to stop."
wait
