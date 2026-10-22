# Drone fault / false-data simulation

Two ways to test how a drone behaves when its data is wrong:

| | **A. Python simulator** (`sim.py`) | **B. PX4 SITL + Gazebo** (`px4_*`) |
|---|---|---|
| Runs | anywhere with Python, ~1 s per scenario | Ubuntu 22.04/24.04, needs a GPU-ish desktop |
| Autopilot | a simple one included (you can edit it) | the real PX4 flight stack |
| False data | any sensor, any shape, via CSV feeds | PX4 failure injection + GPS_INPUT spoofer |
| Status | **tested** (12 unit tests pass, 25 scenarios run) | **written from the docs, not run** - expect small tweaks |

Start with A to design scenarios and metrics, then repeat the interesting ones in B.

## A. Python simulator

```bash
pip install numpy matplotlib
python make_feeds.py          # creates feeds/*.csv  (the false-data files)
python run_all.py             # all 25 scenarios -> results/summary.md + per-scenario plots
python -m unittest -v         # regression tests
```

Other commands: `--list`, `--only baseline gps_step_jump`, `--runs 20` ,
`--no-plots`, `--custom my_case.json`.

Per scenario we get `results/<name>/`: `plot.png` (true vs believed path, altitude, estimation error,
battery or separation), `log_drone*.csv` (10 Hz truth/estimate/sensor log), `events.csv` (timeline of faults,
failsafes, avoidance), `metrics.json`.

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

### What the default results show

- Short GPS loss is tolerated; long GPS loss lands in place within ~1 s of detection.
- Step jumps and noisy/frozen GPS are caught by the innovation gate or the spoof detector.
- A slow walk-off is caught only if velocity is *not* spoofed; a **consistent** spoof (position + velocity) passes every check,
  and the drone lands ~14 m away believing it is home. Good place to add your own detection (e.g. IMU-vs-GPS velocity check).
- False battery data causes either a needless landing or a crash; V2V replay/jamming causes collisions; a fake neighbour freezes a drone forever.
- Command latency of 1 s makes the control loop unstable (drone never completes).

Known simplifications: point-mass drone (no attitude dynamics), 2-state filter per axis, flat Earth, no motor model.
It tells you *how the logic behaves*, not how your specific airframe flies.

## B. PX4 SITL + Gazebo

```bash
./px4_setup.sh                      # installs PX4 + Gazebo + python env (once; long)
. .venv/bin/activate && python make_feeds.py
./px4_launch_sims.sh 2              # terminal 1: Gazebo + 2 drones (leave running)
python px4_run_scenario.py --list   # terminal 2
python px4_run_scenario.py baseline --layout two
python px4_run_scenario.py gps_off --layout two --fault-drone 0
python px4_analyze.py px4_results/gps_off [--ulog path/to/log.ulg]
```

- **Failure injection** (`inject` actions): MAVSDK failure plugin -> GPS/baro/accel/gyro/mag off, stuck, wrong, intermittent; motor off.
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

## Files

```
sim.py  scenarios.py  run_all.py  report.py  make_feeds.py  test_sim.py     simulator
px4_setup.sh  px4_launch_sims.sh  px4_run_scenario.py  px4_gps_spoofer.py  px4_analyze.py     PX4 part
feeds/   false-data CSVs          results/   simulator output
```

## C. Your campus map in Gazebo (University of Kent example)

`campus/make_campus_world.py` turns a top-down OpenStreetMap export (PDF or PNG) into a Gazebo world:
the map becomes the ground, the building outlines found in the map are extruded into 3D blocks, and the
GPS origin is set to the campus. Two drones then fly over real landmarks (library, colleges, sports centre).

```bash
sudo apt install -y poppler-utils                 # once, to read the PDF
source .venv/bin/activate
pip install opencv-python-headless pillow         # once
cd campus && python make_campus_world.py && cd ..
deactivate
export PX4_GZ_WORLD=kent_campus                   # in the SAME window you launch from
./px4_launch_sims.sh 2                            # window 1 (no .venv)
python px4_run_scenario.py baseline --layout campus      # window 2 (with .venv)
```

Things to check / tune
- **Scale.** `--width-m` (default 1170) is an estimate of how many metres wide the map is. For exact values use the
  corner coordinates from OpenStreetMap (Share > Image) or a known distance, e.g. `--width-m 1250`.
- **Alignment.** The ground picture and the 3D buildings must line up. If they do not, rerun with `--flip-y` (or `--flip-x`).
- **Building height** is a guess (OSM export has none): `--height 14`. Flights are at 30 m (`--alt`).
- **Your own map.** `python make_campus_world.py --map mymap.png --width-m <metres> --lat <lat> --lon <lon> --name mycampus`,
  then edit `LANDMARKS`, `SPAWNS` and `MISSIONS` at the top of the script. The map must be top-down and north-up, and
  OSM standard style (buildings #d9d0c9) for the automatic buildings.
- `campus/preview.png` shows the detected buildings (red), spawn points (dots) and the planned flights.
- Not run against real PX4/Gazebo by the author: the textured ground (`albedo_map`) and building `polyline` geometry follow
  the Gazebo SDF format, but check the result visually.
