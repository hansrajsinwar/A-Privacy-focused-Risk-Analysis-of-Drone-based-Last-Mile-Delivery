"""
scenarios.py - the test matrix. Add your own at the bottom (or load a JSON file with --custom).

Geometry
  single drone : home (0,0). Flies a 30 m square, returns home, lands. Cruise 10 m, 4 m/s.
  two drones   : crossing paths at (15,15) at the SAME altitude and SAME time, so without
                 working V2V avoidance they collide. Drone 1 yields to drone 0.

Fault types (see README for the parameters of each)
  GPS      : gps_dropout, gps_bias_step, gps_drift, gps_feed, gps_freeze, gps_noise
  Baro     : baro_stuck, baro_bias
  IMU      : imu_bias
  Battery  : battery_offset, battery_stuck
  Control  : cmd_delay
  Env      : wind_step
  V2V      : v2v_drop, v2v_freeze, v2v_ghost
"""
import json

from sim import DroneConfig, Fault, SimConfig

SQUARE = [(30, 0), (30, 30), (0, 30)]      # then RTL to (0,0)
T_FAULT = 25.0                             # mid-mission for the single-drone square


def single(name, desc, faults=(), **kw):
    dkeys = ("cruise_alt", "cruise_speed", "initial_soc")
    dkw = {k: kw.pop(k) for k in list(kw) if k in dkeys}
    d = DroneConfig(id=0, home=(0, 0), waypoints=list(SQUARE), **dkw)
    return SimConfig(name=name, description=desc, drones=[d], faults=list(faults), **kw)


def double(name, desc, faults=(), **kw):
    d0 = DroneConfig(id=0, home=(0, 15), waypoints=[(30, 15), (0, 15)])
    d1 = DroneConfig(id=1, home=(15, 0), waypoints=[(15, 30), (15, 0)])
    return SimConfig(name=name, description=desc, drones=[d0, d1], faults=list(faults), **kw)


_LIST = [
    # ---------------- reference ----------------
    single("baseline", "No faults. Reference run to compare everything against."),
    single("wind_gust", "Wind step of (8,4) m/s from 20-45 s. Environmental disturbance, not an attack.",
           [Fault("wind_step", 20, 45, params={"vec": [8, 4, 0]})]),

    # ---------------- GPS ----------------
    single("gps_dropout_short", "GPS lost for 0.6 s. Should be tolerated (no failsafe).",
           [Fault("gps_dropout", T_FAULT, T_FAULT + 0.6)]),
    single("gps_dropout_long", "GPS lost from 25 s onward. Expect GPS-lost failsafe -> land.",
           [Fault("gps_dropout", T_FAULT, 60)]),
    single("gps_step_jump", "GPS position suddenly jumps (12,-8) m. Expect innovation gate to reject it -> land.",
           [Fault("gps_bias_step", T_FAULT, params={"offset": [12, -8, 0]})]),
    single("gps_drift_slow", "Slow spoofing walk-off 0.4 m/s, velocity NOT spoofed. Detector should catch it.",
           [Fault("gps_drift", 20, params={"rate": [0.4, 0.2, 0.0], "spoof_velocity": False})]),
    single("gps_drift_slow_no_detector", "Same walk-off with the spoof detector disabled. Shows what happens undetected.",
           [Fault("gps_drift", 20, params={"rate": [0.4, 0.2, 0.0], "spoof_velocity": False})],
           drift_detector=False),
    single("gps_drift_consistent", "Smarter spoof: position AND velocity drift together. Detector is blind to this.",
           [Fault("gps_drift", 20, params={"rate": [0.4, 0.2, 0.0], "spoof_velocity": True})]),
    single("gps_feed_walkoff", "False-data feed from feeds/gps_walkoff.csv (smooth walk-off, velocity spoofed too).",
           [Fault("gps_feed", 20, params={"file": "gps_walkoff.csv", "spoof_velocity": True})]),
    single("gps_feed_circle", "False-data feed from feeds/gps_circle.csv (15 m circle, velocity not spoofed).",
           [Fault("gps_feed", 20, params={"file": "gps_circle.csv", "spoof_velocity": False})]),
    single("gps_freeze", "GPS replays a stale fix (frozen position + velocity) from 25 s.",
           [Fault("gps_freeze", T_FAULT)]),
    single("gps_noise_high", "GPS noise x10 from 25-60 s (jamming-like degradation).",
           [Fault("gps_noise", T_FAULT, 60, params={"scale": 10})]),

    # ---------------- other sensors ----------------
    single("baro_stuck_climb", "Barometer freezes during climb-out (t=4 s). GPS height should save the day.",
           [Fault("baro_stuck", 4)]),
    single("baro_bias", "Barometer gets +8 m offset at 20 s.",
           [Fault("baro_bias", 20, params={"offset": 8.0})]),
    single("imu_bias_then_gps_loss", "Accelerometer bias 0.5 m/s2 from 20 s, then GPS lost at 25 s -> dead reckoning is poisoned.",
           [Fault("imu_bias", 20, params={"bias": [0.5, 0.5, 0.0]}), Fault("gps_dropout", T_FAULT, 80)]),

    # ---------------- battery ----------------
    single("battery_false_low", "Battery reading falsely drops 85 points at 20 s -> unnecessary emergency landing.",
           [Fault("battery_offset", 20, params={"offset": -85})]),
    single("battery_false_high", "Real battery is nearly flat (20%) but reads +70 -> drone flies until power dies.",
           [Fault("battery_offset", 0, params={"offset": 70})], initial_soc=20.0, batt_drain=0.4),

    # ---------------- control link ----------------
    single("cmd_latency_0p4s", "0.4 s command latency from 15 s (control loop delay).",
           [Fault("cmd_delay", 15, params={"delay": 0.4})]),
    single("cmd_latency_1s", "1.0 s command latency from 15 s - control loop becomes unstable.",
           [Fault("cmd_delay", 15, params={"delay": 1.0})]),

    # ---------------- two drones ----------------
    double("two_drones_baseline", "Crossing paths, V2V avoidance working. Drone 1 yields.", duration=120),
    double("two_drones_no_avoidance", "Same, avoidance disabled -> shows why it is needed.", avoidance=False, duration=120),
    double("two_drones_v2v_freeze", "Drone 0's broadcast is replayed (frozen) to drone 1 from 5 s -> yield never triggers.",
           [Fault("v2v_freeze", 5, drone=1, target=0)], duration=120),
    double("two_drones_v2v_drop", "Drone 1 stops receiving drone 0's broadcasts from 5 s (jamming).",
           [Fault("v2v_drop", 5, drone=1, target=0)], duration=120),
    double("two_drones_v2v_ghost", "Fake neighbour injected on drone 1's path at 12 s -> drone 1 freezes forever (denial of service).",
           [Fault("v2v_ghost", 12, drone=1, params={"pos": [15, 6, 10], "id": -1})], duration=120),
    double("two_drones_gps_spoof_d1", "Drone 1 is GPS-spoofed with a consistent drift (0,0.5 m/s) from 5 s while sharing airspace.",
           [Fault("gps_drift", 5, drone=1, params={"rate": [0.0, 0.5, 0.0], "spoof_velocity": True})], duration=120),
]

ALL = {c.name: c for c in _LIST}


def load_custom(path):
    """Load a scenario from JSON. Example in README."""
    from sim import DroneConfig, Fault, SimConfig
    with open(path) as fh:
        d = json.load(fh)
    drones = [DroneConfig(**x) for x in d.pop("drones")]
    faults = [Fault(**x) for x in d.pop("faults", [])]
    return SimConfig(drones=drones, faults=faults, **d)
