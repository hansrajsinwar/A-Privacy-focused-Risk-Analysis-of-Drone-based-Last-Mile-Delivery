# Experiment plan and record

This file lists every experiment in the repository: what question it answers, the exact settings, the command that
runs it, where the results are, and what has and has not been done. All results are from simulation and describe the
models in this repository only (see `docs/assumptions.md`).

## Overview

| # | Experiment | Question | Code | Results | Status |
|---|---|---|---|---|---|
| E1 | Fault and false-data response (Python) | How does a delivery drone's control logic respond to faulty or spoofed data, and does it notice? | `run_multi.py`, `sim.py`, `scenarios.py` | `results_multi/` | Done: 25 scenarios x 5 seeds |
| E2 | Remote ID inference and mitigations (Python) | Do Remote ID broadcasts of delivery flights reveal customer addresses to a passive observer, and what do simple mitigations cost? | `remote_id.py` | `results_remote_id/` | Done: 6 conditions x 5 seeds |
| E3 | PX4 + Gazebo baseline (real autopilot) | Does the real PX4 autopilot fly the two-drone campus mission as expected? | `px4_run_scenario.py`, `px4_analyze.py` | `results/px4_results/baseline/` | Done: one baseline flight |
| E4 | PX4 + Gazebo fault runs | Do the fault behaviours seen in E1 also appear with real PX4? | `px4_run_scenario.py`, `run_px4_checks.sh` | `px4_results/` | Planned: not yet reported |

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
- Result (in `results/px4_results/baseline/`): max distance from home 368 m and 447 m, both drones returned to within 1.4 m,
  minimum separation 13.0 m.
- Limits: campus map scale (1170 m) and building height (14 m) are estimates; the ground image does not render.

## E4. PX4 + Gazebo fault runs (planned)

- Scenarios to run first: `gps_off`, `gps_spoof_walk` (experimental), `baro_off`. PX4 accepts only some failure types per sensor,
  so the Python scenarios have no exact PX4 equivalent; compare behaviour, not numbers.
- Run: `python3 patch_px4.py px4_run_scenario.py`, then `bash run_px4_checks.sh` (about 20 minutes). Output `px4_results/<scenario>/`.
- Report PX4 outcomes only from logs in `px4_results/`.

## Reproducibility checklist

- [ ] `pip install -r requirements.txt` (Python 3.10 or newer)
- [ ] `python -m pytest -q` shows 20 passed
- [ ] `python run_multi.py --runs 5 --seed 1` gives PASS 7, SAFE_ABORT 6, DEGRADED 5, FAIL 5, HUNG 2 and identical `runs.csv`
- [ ] `python remote_id.py` gives the table in E2 and identical `runs.csv`
- Reference environment: Python 3.13, numpy 2.5.x, scipy 1.18.1, matplotlib 3.11.2 (recorded in each `manifest.json`).

## Open items

- Agree the research focus with the supervisor: both studies are now in the repository.
- Run and report the PX4 fault scenarios (E4); validate or label the campus map.
- Literature-review corrections from the 18 November feedback (done in the dissertation text, not in this repository).
