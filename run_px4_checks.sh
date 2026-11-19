#!/usr/bin/env bash
# run_px4_checks.sh  -  run several PX4 scenarios one after another, restarting Gazebo each time.
#
#   cd ~/drone_repo && bash run_px4_checks.sh                       # gps_off gps_spoof_walk baro_off, campus layout
#   bash run_px4_checks.sh "baseline gps_off" campus                # your own list, layout
#
# Run it from a terminal where the virtual environment is NOT active (it activates .venv itself for Python).
# Needs: ~/PX4-Autopilot built, .venv with mavsdk<4, patched px4_run_scenario.py. Takes about 6 minutes per scenario.
# Output: px4_results/<scenario>/ (drone0.csv, actions.json) and px4_results/<scenario>/analysis.txt
set -u
SCEN="${1:-gps_off gps_spoof_walk baro_off}"
LAYOUT="${2:-campus}"
cd "$(dirname "$0")"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-kent_campus}"

cleanup() { pkill -f px4_launch_sims.sh 2>/dev/null; pkill -f "bin/px4" 2>/dev/null; pkill -f "gz sim" 2>/dev/null
            pkill -9 -f mavsdk_server 2>/dev/null; pkill -9 -f px4_run_scenario 2>/dev/null; sleep 3; }
trap 'cleanup; exit 130' INT TERM
cleanup
for sc in $SCEN; do
  echo "=================== $sc ($(date +%H:%M:%S)) ==================="
  bash ./px4_launch_sims.sh 2 > /tmp/launch_$sc.log 2>&1 &
  sleep 45                                   # Gazebo + both drones
  if ! grep -q "all running" /tmp/launch_$sc.log; then echo "!! simulator did not start (see /tmp/launch_$sc.log)"; cleanup; continue; fi
  ( source .venv/bin/activate
    python px4_run_scenario.py "$sc" --layout "$LAYOUT" --fault-drone 0 2>&1 | tee "/tmp/run_$sc.log"
    mkdir -p "px4_results/$sc"
    python px4_analyze.py "px4_results/$sc" 2>&1 | tee "px4_results/$sc/analysis.txt" )
  cleanup
done
echo "ALL DONE. Results in px4_results/ - send me px4_results/*/analysis.txt (or the folder)."
