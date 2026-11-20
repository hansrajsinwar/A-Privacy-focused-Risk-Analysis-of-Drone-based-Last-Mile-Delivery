# Assumptions and limitations of the Python simulator

This file lists what the model assumes. Every result in the report is a result *of this model*.

## Vehicle and environment
- Point-mass vehicle with first-order lag, linear drag and a simple crash/impact rule; no attitude dynamics, no motors.
- Flat Earth, uniform wind plus a gust process; wind is a fault type only in `wind_gust`.
- Time step 0.02 s; logs every 0.1 s. GPS 5 Hz, barometer 25 Hz, V2V broadcast 2 Hz.

## Sensors and estimator
- Fixed noise levels for IMU, GPS, barometer and battery (see `Sensors` in `sim.py`).
- Hand-written two-state (position, velocity) Kalman filter per axis with an innovation gate.
- The autopilot only sees the estimate, never the truth.

## Autopilot
- Programmed state machine: takeoff, waypoints, return-to-launch, land, geofence, battery and GPS-lost failsafes,
  a GPS-spoof detector (position change versus GPS velocity) and V2V yielding.
- It is not a commercial autopilot; behaviour of a real airframe may differ.

## Scenarios
- 25 scenarios: one 30 m square route (single drone) and one crossing geometry (two drones, same altitude and time).
- Collision threshold 1 m. Fault start times are fixed (see `scenarios.py`).
- Five seeds (1 to 5) per scenario; the seed only changes sensor noise and gust randomness.

## Outcome labels
- PASS, SAFE_ABORT, DEGRADED, FAIL, HUNG are defined in README.md. SAFE_ABORT only means the emergency descent
  drifted 5 m or less; it does not assess obstacles, people or operational safety.

## Remote ID study (`remote_id.py`)
- Broadcast once per second while airborne: pseudonym, estimated position, time. Real Remote ID messages carry more fields and have specific rates and formats; only what the observer can use is modelled.
- Observer: passive, hears every message (no range limit, no packet loss, no receiver position) = worst case for privacy. It knows the depot and that the delivery includes a short hover.
- Customers: 8 random addresses 60 to 220 m from the depot, 3 deliveries each; one drone in the air at a time. No real map, buildings or road network.
- Mitigations are applied to the broadcast only (noise, grid snapping, pseudonym rotation), not to the flight path. Per-message independent noise is assumed.
- Inference rule: mean broadcast position around the middle of the flight; greedy clustering radius 40 m; hit radius 30 m.
- Results are for this synthetic setup only; they do not show how any real observer or Remote ID receiver would perform.

## Not modelled
- Real Remote ID message formats and regulation details, multiple simultaneous drones, observer placement and range, side information (order times, addresses), adaptive attackers.
- Motor failure (a PX4 scenario only), obstacles, buildings, radio propagation, regulations.
- PX4 fault outcomes: only the PX4 baseline has been flown; fault scenarios are implemented.
