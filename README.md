# A Privacy-focused Risk Analysis of Drone-based Last-Mile Delivery - simulation

Simulation part of the MSc project *A Privacy-focused Risk Analysis of Drone-based Last-Mile Delivery*
(University of Kent, COMP7300). It tests how a delivery drone's control logic behaves when sensor, navigation,
battery, command-link or drone-to-drone (V2V) data is wrong or spoofed.

> **Scope.** Two studies share one simulator. **(1) Fault study** (`run_multi.py`, 25 scenarios): how a drone's control
> logic responds to wrong or spoofed data. **(2) Remote ID study** (`remote_id.py`): whether repeated Remote ID
> broadcasts of delivery flights reveal customer addresses to a passive observer, and what simple mitigations cost.
> Results describe this simplified model only, not a particular airframe, commercial autopilot, Remote ID receiver or
> real delivery service.

| | **A. Python simulator** (`sim.py`) | **B. PX4 SITL + Gazebo** (`px4_*`) |
|---|---|---|
| Runs | anywhere with Python, ~1 s per scenario | WSL2/Ubuntu 22.04, Gazebo Harmonic |
| Autopilot | a simple hand-written one (editable) | the real PX4 flight stack |
| False data | any sensor, any shape, via CSV feeds | PX4 failure injection + GPS_INPUT spoofer |
| Status | **tested and reproducible**: 20 unit tests, 25 scenarios x 5 seeds | **set up; baseline flight recorded. Fault runs: implemented, not yet run** |

## Reproduce the reported results (Python simulator)

```bash
python3 -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python make_feeds.py                  # creates feeds/*.csv (the false-data files)
python -m pytest -q                   # 20 tests, expect "20 passed"
python run_multi.py --runs 5 --seed 1 # 25 scenarios x seeds 1..5  (~30 s)  -> results_multi/
```

`results_multi/` contains everything the reported tables and figures are built from:

| file | content |
|---|---|
| `seed1/ ... seed5/<scenario>.json` | full metrics of every single run (per drone) |
| `runs.csv` | one row per (scenario, seed): outcome, max error, landing error, latency, min separation |
| `stats.csv`, `stats.md` | outcome counts, mean, SD and 95 % CI per scenario - recomputed from `runs.csv` |
| `outcomes.png` | outcome chart (Figure 1 of the report) |
| `errors_ci.png` | mean ± 95 % CI of estimation and landing error per scenario |
| `manifest.json` | exact command, seeds, Python/package versions, SHA-256 of the code and feeds |

`python run_multi.py --analyze-only` rebuilds the statistics from the saved per-run files without re-simulating.
**Single run with logs and plots:** `python run_all.py` writes `results/` (seed 1 only; one `plot.png` per scenario showing true vs believed path, altitude, estimation error and battery; 10 Hz
`log_drone*.csv`, `events.csv`, `metrics.json`, `summary.md`). That folder is *one run per scenario*; the
five-run statistics come only from `results_multi/`.

### Metric and outcome definitions

* **Time step** 0.02 s; logs are written every 0.1 s. **Max est. error** = max over *every* 0.02 s step of
  |estimated position - true position| (3-D, metres). `max_est_error_logged_m` in the JSON is the 0.1 s-sampled value.
* **Landing error** = horizontal distance from home to the true touchdown point (only if it landed).
* **Detection latency** = time from the first fault start to the first failsafe event (only if a failsafe fired).
* **Min separation** = smallest 3-D distance between the two airborne drones; **collision** if < 1 m.
* **PASS** mission completed and landed within 5 m of home. **SAFE_ABORT** emergency landing in place that drifted
  <= 5 m during the descent (says nothing about obstacles, people or operational safety). **DEGRADED** landed, but
  > 5 m from home after a normal landing, or the emergency descent drifted > 5 m. **FAIL** crash, collision or true
  position outside the geofence. **HUNG** no terminal state (landed/crashed) by the end of the simulation.
* **Statistics**: mean +/- 95 % CI = t(0.975, n-1) x SD / sqrt(n), over the runs where the metric exists. With
  n = 5 the t value is 2.776. A metric present in only one run is reported as "(1 run)" with no interval.

### Other commands

`python run_all.py --list`, `--only baseline gps_step_jump`, `--runs 20 --out results20`, `--no-plots`,
`--custom my_case.json`. Per scenario in `results/<name>/`: `plot.png`, `log_drone*.csv`, `events.csv`, `metrics.json`.

### What is simulated

Drone physics (truth, with lag, drag, wind, gusts, battery, crash detection) -> noisy sensors (IMU, 5 Hz GPS,
baro, battery) -> **fault layer** -> Kalman filter with innovation gating -> autopilot (takeoff, waypoints, RTL,
land, geofence, battery failsafes, GPS-lost failsafe, GPS-spoof detector, V2V yielding). The autopilot only sees
the *estimate*, so false data makes the true drone behave differently from what the autopilot believes - that gap is the
thing you are measuring (`est_err` in the logs).

### Fault types

| type | params | meaning |
|---|---|---|
| `gps_dropout` | - | no GPS fixes |
| `gps_bias_step` | `offset:[x,y,z]` | sudden position jump |
| `gps_drift` | `rate:[x,y,z]` m/s, `spoof_velocity` | spoofing walk-off; `true` = velocity spoofed consistently (hard to detect) |
| `gps_feed` | `file`, `spoof_velocity` | offsets read from a CSV `t,dx,dy,dz` (your own false data) |
| `gps_freeze` | - | replay of a stale fix |
| `gps_noise` | `scale` | multiply GPS noise (jamming-like) |
| `baro_stuck` / `baro_bias` | - / `offset` | barometer frozen / offset |
| `imu_bias` | `bias:[x,y,z]` m/s2 | accelerometer bias |
| `battery_offset` / `battery_stuck` | `offset` | false battery readout |
| `cmd_delay` | `delay` s | control latency |
| `wind_step` | `vec:[x,y,z]` | environment, not an attack |
| `v2v_drop` / `v2v_freeze` | `drone` (receiver), `target` (sender) | neighbour data lost / replayed with fresh timestamps |
| `v2v_ghost` | `pos`, `id` | fake neighbour injected |

All faults take `t_start`, optional `t_end`, and `drone`.

### Add your own scenario

In `scenarios.py` append to `_LIST`:

```python
single("my_case", "GPS jumps 20 m east at t=30", [Fault("gps_bias_step", 30, params={"offset": [20, 0, 0]})]),
```

or a JSON file (`python run_all.py --custom my_case.json`):

```json
{"name": "my_case", "description": "baro offset then GPS loss", "duration": 100,
 "drones": [{"id": 0, "home": [0, 0], "waypoints": [[30, 0], [30, 30]]}],
 "faults": [{"type": "baro_bias", "t_start": 15, "params": {"offset": 5}},
            {"type": "gps_dropout", "t_start": 30, "t_end": 50}]}
```

### Your own false-data feed

Make a CSV `t,dx,dy,dz` (seconds since fault start, metre offsets), save in `feeds/`, and use
`Fault("gps_feed", 20, params={"file": "mine.csv", "spoof_velocity": True})`. Edit `make_feeds.py` to generate shapes in code.

### Testing your own failsafe logic

Thresholds live in `Params` (`sim.py`): GPS reject/timeout times, spoof-detector window/threshold, battery limits,
avoidance distances. The logic itself is `Autopilot` - change it, re-run `python run_all.py`, compare `results/summary.md`.
Outcomes: `PASS` (mission done, landed at home) - `SAFE_ABORT` (landed safely, mission not done) - `HUNG` (never finished)
- `DEGRADED` (landed far from where it should, e.g. undetected spoofing) - `FAIL` (crash, collision, left geofence).

### What the results show (5 runs per scenario, seeds 1-5; see `results_multi/stats.md`)

- Short GPS loss is tolerated; long GPS loss lands in place within ~1 s of detection.
- Step jumps and noisy/frozen GPS are caught by the innovation gate or the spoof detector.
- A slow walk-off is caught only if velocity is *not* spoofed; a **consistent** spoof (position + velocity) passes every check,
  and the drone lands ~14 m away believing it is home. Good place to add your own detection (e.g. IMU-vs-GPS velocity check).
- False battery data causes either a needless landing or a crash. In the single crossing geometry tested, disabled, dropped or frozen neighbour data each ended in a collision in 4 of 5 seeds; this does not show the three have equivalent effects elsewhere. A fake neighbour makes the drone hover until the simulation ends (HUNG).
- Command latency of 1 s makes the control loop unstable (drone never completes).

Known simplifications: point-mass drone (no attitude dynamics), hand-written 2-state Kalman filter per axis, flat Earth,
fixed noise settings, **no motor model (motor failure is NOT evaluated in the Python simulator; it exists only as a PX4 scenario)**.
It shows *how this model's logic behaves*, not how a specific airframe or autopilot flies.

## B. PX4 SITL + Gazebo

> **Files.** The `px4_*` scripts are NOT included in this folder: keep the ones already in your repository (yours include the campus layout). Apply the fixes below with `python3 patch_px4.py px4_run_scenario.py` (keeps a `.bak` copy), and run several scenarios unattended with `bash run_px4_checks.sh`.

**Status.** Set up and the baseline flight has been recorded with real PX4 data in `results/px4_results/baseline/` (2 drones, campus world, 30 m altitude: max distance from home 368 m and 447 m, both returned to within 1.4 m, minimum separation 13.0 m). The fault scenarios below are
*implemented but not yet run*; the batch/summary scripts were only checked with synthetic data. No PX4 fault
outcome should be reported until logs exist in `px4_results/`. PX4 only accepts some failure types per sensor
(baro/accel/gyro/mag: off or stuck; GPS: off, stuck or wrong-fix-type; motor/battery: off or wrong), so the
scenario list differs from the Python one. Motor failure is a PX4 scenario only.


```bash
./px4_setup.sh                      # installs PX4 + Gazebo + python env (once; long)
. .venv/bin/activate && python make_feeds.py
./px4_launch_sims.sh 2              # terminal 1: Gazebo + 2 drones (leave running)
python px4_run_scenario.py --list   # terminal 2
python px4_run_scenario.py baseline --layout two
python px4_run_scenario.py gps_off --layout two --fault-drone 0
python px4_analyze.py px4_results/gps_off [--ulog path/to/log.ulg]
```

- **Failure injection** (`inject` actions): MAVSDK failure plugin. Supported pairs only: baro/accel/gyro/mag off or stuck; GPS off, stuck or wrong (fix type only); motor/battery off or wrong. Unsupported pairs are recorded as `rejected by PX4`.
  Needs `SYS_FAILURE_EN=1` (the runner sets it). This is the most reliable way to feed false sensor data in PX4.
- **GPS spoofing** (`gps_spoof_*`): turns the simulated GPS off and starts `px4_gps_spoofer.py`, which sends MAVLink
  `GPS_INPUT` following a ghost track from `feeds/px4_ghost_*.csv`. **Experimental** - check PX4 accepts the messages
  (`listener sensor_gps` in the PX4 shell, or watch QGroundControl).
  It is open-loop (the reported position follows the CSV regardless of reality), unlike the simulator's closed-loop offset feeds.
- **Link problems**: `link_delay_500ms` uses `tc netem` on loopback (sudo).
- PX4 drone paths are in `px4_run_scenario.py` (`LAYOUTS`); the two-drone layout crosses at the same point/time like in the simulator.
  PX4 has no V2V avoidance, so the metric is minimum separation.
- Port assumptions: instance *i* -> MAVSDK on UDP `14540+i`, spoofer to `14580+i`. If your PX4 version differs, change `connect()` and the spoofer port.
- `MissionItem(...)` argument lists change between MAVSDK versions; if you get a `TypeError`, compare with the mission example in your installed MAVSDK-Python.

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

> **Not yet validated.** The map width (1170 m) and building height (14 m) are estimates and the ground image does not show (plain green). Do not draw conclusions about real flight paths or clearance until scale and building data are checked against an independent source and the world is regenerated.

## D. Remote ID study (`remote_id.py`)

Question: do repeated Remote ID broadcasts of delivery flights reveal where customers live, and what do mitigations cost?

```bash
python remote_id.py                    # 5 seeds x 8 customers x 3 deliveries x 6 conditions (~90 s) -> results_remote_id/
python -m pytest -q test_remote_id.py  # 5 tests
```

Model: depot at (0,0); K customer addresses 60 to 220 m away (at least 80 m apart); the drone flies out, hovers 12 s to drop
the parcel and returns. Once per second while airborne it broadcasts (pseudonym, *estimated* position, time). A passive
observer hears everything (no range limit, no packet loss), estimates each address as the mean broadcast position around
the middle of the flight, then clusters the estimates (40 m radius) into "addresses". The mitigation is applied to the
broadcast; the flights are identical across conditions (paired comparison).

| condition | what changes |
|---|---|
| `none` | broadcast the estimate; one fixed ID |
| `id_rotation` | new random pseudonym for every flight |
| `noise_25`, `noise_100` | Gaussian noise (sigma 25 m / 100 m) on every broadcast position |
| `grid_100` | positions snapped to a 100 m grid |
| `gps_spoof` | no mitigation, but a consistent GPS spoof acts on the drone from 20 s (links to the fault study) |

Metrics: flight error (m), % of flights estimated within 30 m, % of addresses recovered, % of flights linkable by ID,
and broadcast-vs-true position error (utility loss or integrity). Output: `runs.csv` (per seed and condition),
`flights.csv` (every flight), `stats.md` / `stats.csv` (mean +/- 95 % CI), `remote_id_summary.png`, `remote_id_map.png`,
`manifest.json`.

## Files

```
sim.py  scenarios.py  run_all.py  run_multi.py  make_outcome_chart.py  make_ci_chart.py  report.py  make_feeds.py  test_sim.py     fault study
remote_id.py  test_remote_id.py     Remote ID study (results_remote_id/)
patch_px4.py  run_px4_checks.sh     PX4 helpers (your own px4_*.py / px4_*.sh stay as they are)
feeds/   false-data CSVs          results/   one-run output          results_multi/   5-seed records and statistics
```

## Environment used (PX4 part)

| Item | Value |
|---|---|
| Computer | Windows laptop, WSL2 |
| Linux | Ubuntu 22.04 (newer default Ubuntu releases are not supported by PX4) |
| Flight software | PX4-Autopilot, SITL target `gz_x500`, autostart 4001 |
| 3D simulator | Gazebo Harmonic |
| Python | virtual environment `.venv`, `mavsdk<4` (version 4 and newer breaks `mavsdk.failure`) |
| Drones | x500 quadcopters; instance *i* uses MAVSDK UDP port 14540+*i*, spoofer port 14580+*i* |
| Windows access to results | `\\wsl$\Ubuntu-22.04\home\hans\drone_repo` |

Every time: window 1 (no `.venv`) `export PX4_GZ_WORLD=kent_campus; ./px4_launch_sims.sh 2`; window 2 (with `.venv`)
`python px4_run_scenario.py <scenario>`. Restart window 1 between scenarios. Stop with Ctrl+C, never Ctrl+Z.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Unable to locate package gz-harmonic` | Unsupported Ubuntu release | Use Ubuntu 22.04 |
| `No module named 'menuconfig'` in the PX4 build | Virtual environment active | `deactivate`, build again |
| `LegacyModuleError ... mavsdk.failure` | `mavsdk` 4 or newer | `pip install "mavsdk<4"` |
| `python: command not found` | New window without the environment | `source .venv/bin/activate` |
| `bind error: Address in use` | Old run (often paused with Ctrl+Z) holds the ports | `pkill -9 -f mavsdk_server; pkill -9 -f px4_run_scenario`, rerun |
| `FailureError UNSUPPORTED` | PX4 does not support that failure type for that unit | Use the supported pairs listed in section B |
| Only one drone visible in Gazebo | Camera follows drone 1 | Zoom out or use Move to in the Entity Tree |
| `FutureWarning: 'mavsdk' has been renamed` | Library notice | Harmless |
