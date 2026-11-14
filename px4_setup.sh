#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_PARENT="${PX4_PARENT:-$HOME}"

sudo apt-get update
sudo apt-get install -y git python3-pip python3-venv curl

cd "$PX4_PARENT"
[ -d PX4-Autopilot ] || git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# installs the toolchain AND Gazebo (gz) - takes a while; reboot/log out afterwards if it asks we to
bash ./Tools/setup/ubuntu.sh

# build the SITL binary (does not start the simulator)
make px4_sitl_default

# python env for the scripts in this project
cd "$PROJECT_DIR"
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

cat <<EOF

Setup done.
  1. (optional) install QGroundControl to watch the vehicles: https://qgroundcontrol.com
  2. . .venv/bin/activate
  3. python make_feeds.py
  4. ./px4_launch_sims.sh 2          # starts Gazebo + 2 drones (leave running)
  5. python px4_run_scenario.py --list
  6. python px4_run_scenario.py gps_off --layout two
  7. python px4_analyze.py px4_results/gps_off
EOF
