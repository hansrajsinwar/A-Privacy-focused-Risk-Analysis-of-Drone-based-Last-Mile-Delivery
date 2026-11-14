# A Privacy-focused Risk Analysis of Drone-based Last-Mile Delivery

A test bench that flies one or two simulated drones over a small area, feeds them **false data** (spoofed GPS, broken
sensors, wrong battery readings, a lying neighbour drone, a delayed command link, a failed motor) and records how each
drone reacts. The question for every scenario is the same: does the drone notice, does it react safely, and where does it
end up?

It has three parts, from fastest to most realistic:

| Part | What it is | Runs on | Status |
|---|---|---|---|
| **A. Python simulator** (`sim.py`) | Simple drone, sensors, Kalman filter and autopilot written for this project; 25 fault scenarios | Any computer with Python | **Tested.** 12 unit tests pass; all 25 scenarios run; results below |
| **B. PX4 + Gazebo** (`px4_*`) | The real PX4 flight software flying two x500 drones in Gazebo Harmonic | Windows with WSL2, Ubuntu 22.04 | **Set up and running.** |
| **C. Kent campus world** (`campus/`) | The University of Kent OpenStreetMap export turned into a Gazebo world with 141 extruded buildings | Same as B | **Built and loads in Gazebo.**  |

## Environment used

| Item | Value |
|---|---|
| Computer | Windows laptop, WSL2 |
| Linux | Ubuntu 22.04 (the default newer Ubuntu release is not supported by PX4) |
| Flight software | PX4-Autopilot, SITL target `gz_x500`, autostart 4001 |
| 3D simulator | Gazebo Harmonic |
| Python | virtual environment `.venv`, `mavsdk<4` (version 4 and newer breaks `mavsdk.failure`) |
| Drones | 2 x x500 quadcopters; instance *i* uses MAVSDK UDP port 14540+*i* and spoofer port 14580+*i* |

## Repository layout

```
sim.py  scenarios.py  run_all.py  report.py  make_feeds.py  test_sim.py      A. Python simulator
px4_setup.sh  px4_launch_sims.sh  px4_run_scenario.py  px4_gps_spoofer.py  px4_analyze.py     B. PX4 + Gazebo
campus/make_campus_world.py  campus/kent_map.pdf                              C. campus world generator and map
feeds/        false-data CSV files (GPS walk-off, circle, PX4 ghost tracks)
results/      Python simulator output         px4_results/   PX4 run output 
config/  docs/  experiments/  src/            
```

## A. Python simulator

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
pip install "mavsdk<4"
python make_feeds.py                    # writes feeds/*.csv (the false-data files)
python run_all.py --list                # the 25 scenario names
python run_all.py --runs 5 --out results_exp1     # every scenario 5 times (about 25 s)
python -m unittest -v                   # 12 regression tests
```

Useful options: `--only baseline gps_step_jump`, `--runs 7`, `--seed 1`, `--no-plots`, `--custom my_case.json`.
Every run uses a different noise seed (`seed + run number`), so results are repeatable.

### Results (real data: 25 scenarios x 5 runs, default seed 1)

Outcomes: **PASS** mission done and landed at home; **SAFE_ABORT** fault detected and landed safely; **DEGRADED** landed
but far from where it believed it was; **HUNG** stuck in the air; **FAIL** crash, collision or left the geofence.
Error columns are the worst case over the runs; detection latency is the mean.

| # | Scenario | Outcome (5 runs) | Max estimate error (m) | Landing error (m) | Detection latency (s) | Min drone separation (m) | First failsafe |
|---|---|---|---|---|---|---|---|
| 1 | baseline | PASS 5/5 | 0.95 | 1.57 | - | - | - |
| 2 | wind_gust | PASS 5/5 | 1.0 | 1.62 | - | - | - |
| 3 | gps_dropout_short | PASS 5/5 | 0.95 | 1.57 | - | - | - |
| 4 | gps_dropout_long | SAFE_ABORT 4/5, DEGRADED 1/5 | 7.91 | 40.16 | 0.82 | - | GPS data lost for 1.0s |
| 5 | gps_step_jump | SAFE_ABORT 4/5, DEGRADED 1/5 | 6.34 | 39.36 | 1.0 | - | GPS spoof detector: position change disagrees with GPS velocity by 13.3 m |
| 6 | gps_drift_slow | SAFE_ABORT 4/5, DEGRADED 1/5 | 9.66 | 33.24 | 10.2 | - | GPS spoof detector: position change disagrees with GPS velocity by 3.9 m |
| 7 | gps_drift_slow_no_detector | DEGRADED 5/5 | 14.56 | 12.82 | - | - | - |
| 8 | gps_drift_consistent | DEGRADED 5/5 | 15.3 | 14.29 | - | - | - |
| 9 | gps_feed_walkoff | DEGRADED 5/5 | 12.03 | 10.74 | - | - | - |
| 10 | gps_feed_circle | SAFE_ABORT 4/5, DEGRADED 1/5 | 8.6 | 42.43 | 3.76 | - | GPS spoof detector: position change disagrees with GPS velocity by 4.5 m |
| 11 | gps_freeze | DEGRADED 4/5, SAFE_ABORT 1/5 | 15.02 | 38.62 | 2.28 | - | GPS innovation test failing for 2.0s (inconsistent with IMU/baro) |
| 12 | gps_noise_high | SAFE_ABORT 3/5, DEGRADED 2/5 | 10.35 | 38.87 | 1.64 | - | GPS spoof detector: position change disagrees with GPS velocity by 7.0 m |
| 13 | baro_stuck_climb | PASS 5/5 | 3.31 | 1.56 | - | - | - |
| 14 | baro_bias | PASS 5/5 | 0.95 | 1.57 | - | - | - |
| 15 | imu_bias_then_gps_loss | FAIL 5/5 | 73.21 | - | 5.62 | - | GPS data lost for 1.0s |
| 16 | battery_false_low | SAFE_ABORT 5/5 | 0.95 | 34.31 | 0.0 | - | MISSION -> LAND: battery critical (9%) |
| 17 | battery_false_high | FAIL 5/5 | 9.74 | - | - | - | - |
| 18 | cmd_latency_0p4s | PASS 5/5 | 1.59 | 1.85 | - | - | - |
| 19 | cmd_latency_1s | HUNG 5/5 | 1.0 | - | - | - | - |
| 20 | two_drones_baseline | PASS 5/5 | 0.95 | 1.33 | - | 3.96 | - |
| 21 | two_drones_no_avoidance | FAIL 4/5, PASS 1/5 | 13.19 | 1.33 | - | 0.12 | - |
| 22 | two_drones_v2v_freeze | FAIL 4/5, PASS 1/5 | 13.19 | 1.33 | - | 0.12 | - |
| 23 | two_drones_v2v_drop | FAIL 4/5, PASS 1/5 | 13.19 | 1.33 | - | 0.12 | - |
| 24 | two_drones_v2v_ghost | HUNG 5/5 | 0.95 | 1.15 | - | 6.06 | - |
| 25 | two_drones_gps_spoof_d1 | DEGRADED 5/5 | 25.36 | 23.64 | - | 8.94 | - |

What the data shows:

- **Baseline is clean.** 5/5 PASS, maximum estimate error 0.95 m, landing 1.57 m from home. Wind gusts, a 0.6 s GPS drop,
  a frozen or offset barometer and 0.4 s command delay do not change the outcome.
- **Detected GPS faults end safely.** A long GPS loss, a position jump, a frozen fix, high GPS noise and a slow walk-off
  (spoofed position only) are caught within 0.8 to 10 s and the drone lands (mostly SAFE_ABORT, some DEGRADED because it
  lands up to about 40 m from home).
- **A consistent GPS spoof is not detected.** When position and velocity drift together (`gps_drift_consistent`,
  `gps_feed_walkoff`) every check passes: 5/5 DEGRADED, the drone lands about 14 m from home while believing it is at
  home. The same slow drift with the detector switched off (`gps_drift_slow_no_detector`) also ends DEGRADED 5/5.
- **A poisoned dead-reckoning chain crashes.** An accelerometer bias followed by GPS loss (`imu_bias_then_gps_loss`)
  gives FAIL 5/5 with an estimate error of 73 m.
- **False battery data is dangerous both ways.** A false low reading causes a needless emergency landing (SAFE_ABORT 5/5);
  a false high reading hides a nearly flat battery and the drone is lost (FAIL 5/5).
- **Control delay.** 0.4 s is tolerated; 1.0 s makes the loop unstable (HUNG 5/5).
- **Drone-to-drone data is a weak point.** With avoidance working, drone 1 yields and stays 3.96 m away (PASS 5/5).
  Without avoidance, or with replayed or dropped neighbour messages, the drones collide in 4 of 5 runs (minimum
  separation 0.12 m). A fake neighbour makes drone 1 freeze forever (HUNG 5/5, a denial of service).
- A GPS spoof on one of two drones in shared airspace leaves it DEGRADED, 23.6 m off target (`two_drones_gps_spoof_d1`).

Known simplifications: point-mass drone (no attitude dynamics), 2-state filter per axis, flat Earth, no motor model.
The simulator shows how the *logic* behaves, not how a specific airframe flies.

### Fault types

Faults take `t_start`, optional `t_end` and `drone`. Types: `gps_dropout`, `gps_bias_step`, `gps_drift`, `gps_feed`
(our own CSV), `gps_freeze`, `gps_noise`, `baro_stuck`, `baro_bias`, `imu_bias`, `battery_offset`, `battery_stuck`,
`cmd_delay`, `wind_step`, `v2v_drop`, `v2v_freeze`, `v2v_ghost`.

Add a scenario in `scenarios.py`:
```python
single("my_case", "GPS jumps 20 m east at t=30", [Fault("gps_bias_step", 30, params={"offset": [20, 0, 0]})]),
```
or as JSON and `python run_all.py --custom my_case.json`. Our own false data goes in `feeds/*.csv` with columns
`t,dx,dy,dz` (seconds since the fault start, metre offsets) and is used with `Fault("gps_feed", 20, params={"file": "mine.csv"})`.

## B. PX4 + Gazebo on Windows (WSL2)

One-time install, in Ubuntu 22.04, from the Linux home folder (not `/mnt/c`):

```bash
./px4_setup.sh                 # installs PX4, Gazebo and the Python packages (long)
```
If the PX4 build fails with `No module named 'menuconfig'`, a virtual environment was active: run `deactivate` and build again.

### Every time: two windows

```bash
# Window 1: simulator (no .venv). Leave it open.
export PX4_GZ_WORLD=kent_campus            # or leave unset for the small default area
cd ~/drone_repo && ./px4_launch_sims.sh 2
# wait about 30 s for "all running" and both drones in Gazebo

# Window 2: one test (with .venv)
cd ~/drone_repo && source .venv/bin/activate
python px4_run_scenario.py --list
python px4_run_scenario.py baseline --layout campus --fault-drone 0
python px4_analyze.py px4_results/baseline
```

Restart window 1 (Ctrl+C, start again) between scenarios, because a failed sensor or a crashed drone carries into the next
run. Stop a run with Ctrl+C, never Ctrl+Z (a paused run keeps the network ports busy).

### PX4 scenarios (16)

`--fault-drone N` chooses which drone receives the fault. Times are seconds from the start of the mission.

| Scenario | False data fed in | Starts (s) |
|---|---|---|
| baseline | none | n/a |
| gps_off, gps_off_short | GPS off for good, or for 3 s | 25 |
| gps_stuck, gps_wrong, gps_intermittent | GPS frozen, wrong or flickering | 25 |
| baro_stuck, baro_wrong | barometer frozen or wrong | 15 |
| accel_wrong, gyro_stuck, mag_wrong | motion and compass sensors bad | 20 |
| motor_off | one motor stops | 20 |
| gps_spoof_freeze, gps_spoof_walk, gps_spoof_walk_stop | spoofed GPS track from `feeds/px4_ghost_*.csv` (experimental) | 20 (stop at 40) |
| link_delay_500ms | 500 ms network delay (needs sudo) | 20 to 40 |

Failure injection (GPS, barometer, accelerometer, gyro, compass, motor) uses the MAVSDK failure plugin with
`SYS_FAILURE_EN=1` and is the most reliable way to feed false sensor data. The GPS spoofer sends MAVLink `GPS_INPUT`
messages; some PX4 versions ignore them, so if a spoof run looks like the baseline use `gps_wrong` and `gps_stuck`.

### Repeating every scenario 5 to 7 times

PX4 needs a fresh simulator for every run, so a batch script restarts Gazebo between runs and saves each repeat in
`px4_results/run1/<scenario>/`, `run2/...` and so on, then `px4_summary.py` combines them (mean, minimum and maximum per
scenario). Estimated time is about 6 minutes per run, so 15 scenarios x 5 repeats takes 7 to 8 hours. Describe the
experiment in `experiment.json` (name, objective, hypothesis, altitude, speed, repeats, notes); it is copied next to the
results so the numbers stay tied to their setup. The scripts and full instructions are in the *Drone Fault Experiment Manual*.

### Where results are saved

| Folder | Made by | Contents |
|---|---|---|
| `results/<scenario>/` | `run_all.py` | `plot.png`, `log_drone*.csv` (10 Hz truth, estimate, sensors), `events.csv`, `metrics.json`; plus `results/summary.md` and `summary.csv` |
| `px4_results/<scenario>/` | `px4_run_scenario.py` | `drone0.csv`, `drone1.csv` (time, position, speed, flight mode, battery, GPS status), `actions.json` (when each fault was fed in) |
| `px4_results/run<k>/<scenario>/` | the batch script | the same, one folder per repeat |

Running a scenario again overwrites its folder. In Windows, open the Ubuntu folders through
`\\wsl$\Ubuntu-22.04\home\hans\drone_repo`.

## C. University of Kent campus world

`campus/make_campus_world.py` turns the OpenStreetMap PDF `campus/kent_map.pdf` into a Gazebo world: it finds buildings
by the standard OpenStreetMap building colour (141 found), extrudes them into 3D blocks, lays the map over the ground and
sets the GPS origin to the campus.

```bash
sudo apt install -y poppler-utils
source .venv/bin/activate && pip install opencv-python-headless pillow
cd campus && python make_campus_world.py && cd ..
```

| Setting | Value used | Note |
|---|---|---|
| Map width (`--width-m`) | 1170 m | **Estimate.** Replace with the real width from the OpenStreetMap Share > Image corner coordinates |
| Building height (`--height`) | 14 m | A guess; the map has no heights |
| Position (`--lat`, `--lon`) | 51.2975, 1.0690 | Approximate campus position |
| Flight height, speed | 30 m, 8 m/s | `--alt`, `--speed` |
| Geofence, timeout | 900 m, 420 s | Set in `layout.json` |

Flights: drone 0 starts on the field by Eliot College and flies to the Library, then Cornwallis. Drone 1 starts south of
Keynes and flies to the Sports Centre, Eliot College and Rutherford. To choose own paths, we can edit `LANDMARKS`, `SPAWNS`
and `MISSIONS` at the top of `make_campus_world.py` (map positions as fractions of the picture, x from the left and y from
the top), rebuild, and check `campus/preview.png`.

## Troubleshooting (problems met while building this)

| Symptom | Cause | Fix |
|---|---|---|
| `Unable to locate package gz-harmonic` | Unsupported default Ubuntu release | Install and use Ubuntu 22.04 |
| `No module named 'menuconfig'` during the PX4 build | Virtual environment active | `deactivate`, then build again |
| `LegacyModuleError ... mavsdk.failure` | `mavsdk` 4 or newer | `pip install "mavsdk<4"` |
| `python: command not found` | New window without the environment | `source .venv/bin/activate` |
| `bind error: Address in use` | An old run (often paused with Ctrl+Z) holds the ports | `pkill -9 -f mavsdk_server; pkill -9 -f px4_run_scenario`, then rerun |
| Only one drone visible in Gazebo | The camera follows drone 1 | Zoom out or use Move to in the Entity Tree |
| Run refuses to arm (no ground-station connection) | PX4 safety check | Set `NAV_DLL_ACT` to 0 |
| `FutureWarning: 'mavsdk' has been renamed` | Library notice | Harmless |

## Known issues and not yet done

- **PX4 fault runs not yet recorded.** The simulator and both drones start correctly; no `px4_results` have been collected
  yet, so this README contains no PX4 result numbers. The PX4 scripts (mission upload, failure injection, GPS spoofer, batch
  script, summary script) were written from the PX4 and MAVSDK documentation and checked only on made-up data.
- **Campus ground picture missing.** In Gazebo the buildings appear but the ground is plain green instead of the map.
  Check the `albedo_map` path in the generated `kent_campus.sdf` and try `--flip-y`.
- **Estimated map scale (1170 m) and building height (14 m)** need real values.
- **Spoofed GPS (`gps_spoof_*`)** is experimental and may be ignored by some PX4 versions.

## Experiment record

| Field | Value |
|---|---|
| Name | Privacy-focused risk analysis of drone-based last-mile delivery |
| Simulators | Python simulator; PX4 + Gazebo Harmonic |
| World | University of Kent campus (`kent_campus`), 2 drones, 30 m altitude |
| Repeats | 5 to 7 per scenario |
| Date of Python results above | 2026-11 (25 scenarios x 5 runs, seed 1) |
| Researcher | _your name_ |
| Hypothesis | _write your own, for example: a consistent GPS spoof is not detected_ |
