# Experiment plan and record

This file lists every experiment in the repository: what question it answers, the exact settings, the command that
runs it, where the results are, and what has and has not been done. All results are from simulation and describe the
models in this repository only (see `docs/assumptions.md`).

## Overview

| # | Experiment | Question | Code | Results | Status |
|---|---|---|---|---|---|
| E1 | Fault and false-data response (Python) | How does a delivery drone's control logic respond to faulty or spoofed data, and does it notice? | `run_multi.py`, `sim.py`, `scenarios.py` | `results_multi/` | Done: 25 scenarios x 5 seeds |
| E2 | Remote ID inference and mitigations (Python) | Do Remote ID broadcasts of delivery flights reveal customer addresses to a passive observer, and what do simple mitigations cost? | `remote_id.py` | `results_remote_id/` | Done: 6 conditions x 5 seeds |
| E3 | PX4 + Gazebo baseline (real autopilot) | Does the real PX4 autopilot fly the two-drone campus mission as expected? | `px4_run_scenario.py`, `px4_analyze.py` | `px4_results/baseline/` | Done: one baseline flight |
| E4 | PX4 + Gazebo fault runs | Do the fault behaviours seen in E1 also appear with real PX4? | `px4_run_scenario.py`, `run_px4_checks.sh` | `px4_results/` | Partly done: gps_off, gps_wrong, gps_stuck reported; 3 runs incomplete, 2 not run |

## E1. Fault and false-data response

**Hypotheses**
- H1.1 A short GPS dropout is tolerated; a long dropout triggers the GPS-lost failsafe within a few seconds.
- H1.2 A GPS spoof that moves position and velocity together (`gps_drift_consistent`) is not detected, so the drone lands away from home while believing it is at home.
- H1.3 False battery readings cause either a needless landing (reads low) or loss of the drone (reads high).
- H1.4 In the crossing geometry tested, disabling V2V avoidance, or dropping or freezing neighbour data, leads to collisions.

**Settings**
- Single drone: home (0,0), 30 m square route, cruise altitude 10 m, speed 4 m/s, faults from 20 to 25 s (see `scenarios.py`).
- Two drones: crossing paths at (15,15), same altitude and time, collision threshold 1 m.
- Time step 0.02 s, log every 0.1 s. Seeds 1 to 5 (run i uses seed 1 + i). Seeds change sensor noise and gusts only.
- 25 scenarios: baseline; wind_gust; GPS (dropout short and long, step jump, slow drift with and without detector, consistent drift, two false-data feeds, freeze, high noise); baro (stuck climb, bias); imu_bias_then_gps_loss; battery (false low, false high); command latency (0.4 s, 1 s); two-drone (baseline, no avoidance, V2V freeze, drop, ghost, GPS spoof on drone 1).

**Run**
```bash
python run_multi.py --runs 5 --seed 1        # about 30 s
python run_all.py                            # one run per scenario with plots -> results/
python -m pytest -q                          # 20 tests
```

**Outputs:** `results_multi/runs.csv` (every run), `stats.md` and `stats.csv` (mean +/- 95 % CI, t distribution, 4 degrees of
freedom), `outcomes.png`, `errors_ci.png`, `manifest.json` (command, seeds, versions, code hashes), per-run JSON in `seed1/` to `seed5/`.

**Metrics:** max estimation error (every 0.02 s step), landing error, detection latency, minimum separation, and the outcomes
PASS, SAFE_ABORT, DEGRADED, FAIL, HUNG (definitions in `README.md`).

**Result summary (5 seeds):** by most-common outcome, PASS 7, SAFE_ABORT 6, DEGRADED 5, FAIL 5, HUNG 2 scenarios.
Baseline PASS 5/5. Consistent GPS spoof DEGRADED 5/5 with landing error 13.5 +/- 0.9 m and no detection. Full table in
`results_multi/stats.md`.

**Limits:** point-mass vehicle, hand-written Kalman filter and autopilot, one route and one crossing geometry, five seeds
(wide intervals). In the collision scenarios the large maximum estimation error is a crash artefact.

## E2. Remote ID inference and mitigations

**Hypotheses**
- H2.1 Unmodified Remote ID broadcasts let a passive observer place each delivery within 30 m of the customer address.
- H2.2 Pseudonym rotation stops linking flights by ID but does not stop address inference.
- H2.3 Position noise or coarse snapping reduces inference only when large, and increases the error between the
  broadcast and the true position (utility cost).
- H2.4 Repeated deliveries to the same address weaken noise-based protection.

**Settings** (`RID` in `remote_id.py`)
- Depot (0,0); 8 customer addresses 60 to 220 m from the depot, at least 80 m apart; 3 deliveries per customer (24 flights per seed).
- Drone flies out at 5 m/s, altitude 10 m, hovers 12 s at the address, returns. Flights are run by `sim.py`.
- Broadcast: 1 Hz while airborne, pseudonym + estimated position + time.
- Observer: hears everything (no range limit, no packet loss); estimates each address as the mean broadcast position around the
  middle of the flight; clusters estimates with a 40 m radius; a hit is within 30 m.
- Conditions: `none`, `id_rotation`, `noise_25`, `noise_100` (Gaussian sigma in metres, per message), `grid_100` (100 m grid), `gps_spoof`
  (consistent spoof from 20 s). Mitigations act on the broadcast only; flights are identical across conditions.
- Seeds 1 to 5.

**Run**
```bash
python remote_id.py                          # about 90 s -> results_remote_id/
python -m pytest -q test_remote_id.py
```

**Outputs:** `runs.csv` (per seed and condition), `flights.csv` (every flight), `stats.md` and `stats.csv` (mean +/- 95 % CI),
`remote_id_summary.png`, `remote_id_map.png`, `manifest.json`.

**Metrics:** flight error (m), % flights within 30 m, % addresses recovered, % flights linkable by ID, broadcast-vs-true position error (m).

**Result summary (5 seeds):**

| Condition | Flights within 30 m (%) | Addresses recovered (%) | Broadcast vs true position (m) |
|---|---|---|---|
| none | 100 | 100 | 0.2 |
| id_rotation | 100 | 100 | 0.2 |
| noise_25 | 100 | 100 | 31.4 |
| noise_100 | 53.3 | 82.5 | 125.2 |
| grid_100 | 40.0 | 40.0 | 43.1 |
| gps_spoof | 100 | 100 | 13.1 |

**Limits:** synthetic customers and routes, one drone airborne at a time, a worst-case observer, no real Remote ID message
formats or receivers, wide intervals for the noise_100 and grid_100 rows.

## E3. PX4 + Gazebo baseline

- Setup: WSL2 Ubuntu 22.04, PX4 SITL `gz_x500`, Gazebo Harmonic, world `kent_campus`, 2 drones, 30 m altitude, `mavsdk<4`.
- Command: `python px4_run_scenario.py baseline --layout campus --fault-drone 0`, then `python px4_analyze.py <folder>`.
- Result (`px4_results/baseline/`): both drones flew the mission, returned (RTL) and held at home. Maximum distance from home
  368.4 m and 446.6 m, final distance 0.0 m, maximum altitude 30.1 m, minimum separation between the drones 13.1 m.
  An earlier baseline run (`results/px4_results/baseline/`) gave 1.4 m final distance and 13.0 m separation, so expect small
  run-to-run differences.
- Limits: campus map scale (1170 m) and building height (14 m) are estimates; the ground image does not render.

## E4. PX4 + Gazebo fault runs

**Question:** do the GPS fault behaviours seen in the Python study (E1) also appear with the real PX4 autopilot?

**Hypotheses**
- H4.1 A lost GPS fix is detected within a few seconds and the drone lands (as `gps_dropout_long`, H1.1).
- H4.2 A frozen GPS position is not detected (as `gps_freeze`).

**Settings:** the fault is injected into drone 0 at about 25 s (`SYS_FAILURE_EN=1`, MAVSDK failure plugin); drone 1 flies normally.
Run time limit 420 s. PX4 accepts only some failure types per sensor (GPS: off, stuck, wrong, where "wrong" only changes the fix
type), so the Python scenarios have no exact PX4 equivalent. Compare behaviour, not numbers.

**Run:** `python px4_run_scenario.py <scenario> --layout campus --fault-drone 0`, then `python px4_analyze.py px4_results/<scenario>`
(or `bash run_px4_checks.sh` for several). One run per scenario, no repeats.

**Result (one run each):**

| PX4 scenario | What happened to drone 0 | Landed / final position | Min separation | Closest Python scenario |
|---|---|---|---|---|
| baseline | Normal mission, RTL, hold | 0.0 m from home | 13.1 m | baseline |
| gps_off | Fix became NO_GPS; flight mode changed to UNKNOWN 4.8 s after the fault; descended and landed | about 78.6 m from home, about 50 s | 258.8 m | gps_dropout_long |
| gps_wrong | Fix became FIX_2D; same reaction 4.9 s after the fault; descended and landed | about 78.5 m from home | 259.8 m | gps_dropout_long |
| gps_stuck | Fix stayed FIX_3D, mode stayed MISSION, no failsafe; drone kept flying until the 420 s time limit | 57.1 m from home, still airborne | 47.5 m | gps_freeze |

Drone 1 flew its normal mission in `gps_off` and `gps_wrong` (RTL after about 181 s).

**What this shows (for these runs only)**
- Real PX4 detected a lost or degraded GPS fix within about 5 s and brought the drone down. This matches H4.1 and the Python
  `gps_dropout_long` behaviour (detected, safe abort). It landed about 78 m from home; the Python scenario landed about 38 m away, so the
  distances differ and are not comparable.
- A frozen GPS position (`gps_stuck`) was not detected: no failsafe, no mode change, the drone kept flying on the stuck position.
  This matches H4.2 and the Python `gps_freeze` behaviour (mostly undetected). The minimum separation to drone 1 fell to 47.5 m.
- Detection depends on the fix type reported by the receiver. A fault that keeps the fix valid (stuck, or the consistent
  spoof of E1) is not caught by that check.

**Incomplete runs, not reported:** `baro_wrong` (ended at 15 s, the failure type is unsupported by PX4 and was run before
`patch_px4.py`), `accel_stuck` (no actions log; the drone hovered near home for 420 s, so the fault was probably not applied), `gyro_off`
(only `drone1.csv`). Do not cite these. Re-run them after `python3 patch_px4.py px4_run_scenario.py`, using `baro_off`, `accel_stuck`, `gyro_off`.

**Not yet run:** `gps_spoof_walk` (experimental), `baro_off`.

**Limits:** one run per scenario, so no confidence intervals; PX4 behaviour depends on version and parameters; the landing positions
depend on the campus layout and mission; the map scale is unvalidated.

## Reproducibility checklist

- [ ] `pip install -r requirements.txt` (Python 3.10 or newer)
- [ ] `python -m pytest -q` shows 20 passed
- [ ] `python run_multi.py --runs 5 --seed 1` gives PASS 7, SAFE_ABORT 6, DEGRADED 5, FAIL 5, HUNG 2 and identical `runs.csv`
- [ ] `python remote_id.py` gives the table in E2 and identical `runs.csv`
- Reference environment: Python 3.13, numpy 2.5.x, scipy 1.18.1, matplotlib 3.11.2 (recorded in each `manifest.json`).

## Open items

- Agree the research focus with the supervisor: both studies are now in the repository.
- Re-run `baro_off`, `accel_stuck`, `gyro_off` and run `gps_spoof_walk` with the patched script (E4); repeat the GPS runs if repeats are wanted; validate or label the campus map.
- Literature-review corrections from the 18 November feedback (done in the dissertation text, not in this repository).
