"""
sim.py - lightweight multi-drone simulator with fault / false-data injection.

Layers (all in this file):
  Fault / FaultManager : describes and applies injected faults
  Sensors              : IMU, GPS, barometer, battery models (+ fault hooks)
  Estimator            : per-axis Kalman filter with innovation gating
  Autopilot            : mission state machine, failsafes, GPS-spoof detector, V2V avoidance
  Drone                : "truth" physics (what really happens)
  Simulation           : runs everything, logs, computes metrics

Coordinate frame: x = east, y = north, z = up (metres). Home is z = 0.
The autopilot only ever sees *estimated* state, so false sensor data makes the
real (true) drone behave differently from what the autopilot believes.
"""
from __future__ import annotations

import copy
import csv
import json
import math
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

G = 9.81
INF = math.inf
ZERO3 = np.zeros(3)

# fault types that are environment effects, not "attacks" (ignored for detection latency)
NON_FAULT_TYPES = {"wind_step"}


# ----------------------------------------------------------------------------
# Configuration objects
# ----------------------------------------------------------------------------
@dataclass
class Fault:
    """One injected fault / false-data feed.

    type    : see FAULT_TYPES in the README
    t_start : sim time (s) the fault becomes active
    t_end   : sim time (s) it stops (default: never)
    drone   : id of the drone affected (for v2v_*: the *receiving* drone)
    target  : for v2v_*: id of the neighbour whose data is corrupted
    params  : type specific parameters
    """
    type: str
    t_start: float
    t_end: float = INF
    drone: int = 0
    target: Optional[int] = None
    params: dict = field(default_factory=dict)
    state: dict = field(default_factory=dict, repr=False)

    def active(self, t: float) -> bool:
        return self.t_start <= t < self.t_end


@dataclass
class DroneConfig:
    id: int
    home: tuple
    waypoints: list
    cruise_alt: float = 10.0
    cruise_speed: float = 4.0
    initial_soc: float = 100.0


@dataclass
class SimConfig:
    name: str
    description: str = ""
    drones: list = field(default_factory=list)
    faults: list = field(default_factory=list)
    duration: float = 150.0
    dt: float = 0.02
    fence_radius: float = 50.0       # horizontal geofence around home (m)
    fence_max_alt: float = 30.0      # vertical geofence (m)
    wind: tuple = (0.0, 0.0, 0.0)    # steady wind (m/s)
    gust_sigma: float = 0.5          # random gust strength (m/s)
    arm_time: float = 2.0
    batt_drain: float = 0.3          # % of battery per second while armed
    drift_detector: bool = True      # GPS/velocity consistency detector
    avoidance: bool = True           # V2V collision avoidance


@dataclass
class Params:
    """Autopilot tuning and failsafe thresholds - tweak these to test your own logic."""
    kp_pos: float = 0.8
    kv: float = 2.5
    vmax_up: float = 2.5
    vmax_dn: float = 1.5
    amax_h: float = 5.0
    amax_up: float = 4.0
    amax_dn: float = 3.0
    wp_radius: float = 1.2
    land_speed: float = 1.5
    land_speed_low: float = 0.6
    # estimator gate / GPS health
    gps_reject_time: float = 2.0     # s of continuous innovation rejection -> GPS declared failed
    gps_timeout: float = 1.0         # s without GPS data -> GPS declared failed
    # GPS spoof / drift detector (pos change vs integrated GPS velocity)
    drift_window: float = 15.0
    drift_min_span: float = 10.0
    drift_thresh: float = 3.5        # m
    drift_persist: float = 1.0       # s
    # battery
    min_arm_soc: float = 35.0
    soc_rtl: float = 30.0
    soc_land: float = 15.0
    # V2V avoidance (drone yields to neighbours with a LOWER id)
    avoid_enter: float = 12.0
    avoid_exit: float = 16.0
    v2v_stale: float = 3.0


# ----------------------------------------------------------------------------
# Fault manager
# ----------------------------------------------------------------------------
class FaultManager:
    def __init__(self, faults, feed_dir="feeds"):
        self.faults = faults
        self.feed_dir = feed_dir
        self._feeds = {}

    def active(self, drone, ftype, t, target=None):
        out = []
        for f in self.faults:
            if f.type != ftype or f.drone != drone or not f.active(t):
                continue
            if target is not None and f.target is not None and f.target != target:
                continue
            out.append(f)
        return out

    def feed(self, name):
        """Load a CSV false-data feed: columns t,dx,dy,dz (offsets in metres)."""
        if name not in self._feeds:
            path = name if os.path.exists(name) else os.path.join(self.feed_dir, name)
            self._feeds[name] = np.loadtxt(path, delimiter=",", skiprows=1)
        return self._feeds[name]


# ----------------------------------------------------------------------------
# Sensors (with fault hooks = where false data gets in)
# ----------------------------------------------------------------------------
class Sensors:
    GPS_SIG_H = 0.5
    GPS_SIG_V = 1.0
    GPS_SIG_VEL = 0.15
    BARO_SIG = 0.3
    ACC_SIG = 0.05

    def __init__(self, drone_id, faults: FaultManager, rng):
        self.id = drone_id
        self.F = faults
        self.rng = rng
        self.acc_bias = rng.normal(0, 0.03, 3)
        self.baro_bias = rng.normal(0, 0.3)

    def imu(self, t, acc_true):
        bias = self.acc_bias.copy()
        for f in self.F.active(self.id, "imu_bias", t):
            bias += np.array(f.params["bias"], float)
        return acc_true + bias + self.rng.normal(0, self.ACC_SIG, 3)

    def gps(self, t, pos, vel):
        F, i = self.F, self.id
        scale = 1.0
        for f in F.active(i, "gps_noise", t):
            scale *= f.params.get("scale", 5.0)
        sig = np.array([self.GPS_SIG_H, self.GPS_SIG_H, self.GPS_SIG_V])
        p = pos + self.rng.normal(0, 1, 3) * sig * scale
        v = vel + self.rng.normal(0, self.GPS_SIG_VEL, 3) * scale
        if F.active(i, "gps_dropout", t):
            return None
        for f in F.active(i, "gps_bias_step", t):
            p = p + np.array(f.params["offset"], float)
        for f in F.active(i, "gps_drift", t):
            rate = np.array(f.params["rate"], float)
            p = p + rate * (t - f.t_start)
            if f.params.get("spoof_velocity", False):   # "consistent" spoof: velocity matches
                v = v + rate
        for f in F.active(i, "gps_feed", t):
            data = F.feed(f.params["file"])
            el = t - f.t_start

            def off(x):
                return np.array([np.interp(x, data[:, 0], data[:, k]) for k in (1, 2, 3)])
            p = p + off(el)
            if f.params.get("spoof_velocity", False):
                v = v + (off(el + 0.5) - off(el - 0.5)) / 1.0
        for f in F.active(i, "gps_freeze", t):          # replay of a stale fix
            if "pos" not in f.state:
                f.state["pos"], f.state["vel"] = p.copy(), v.copy()
            p, v = f.state["pos"].copy(), f.state["vel"].copy()
        return {"t": t, "pos": p, "vel": v}

    def baro(self, t, z_true):
        val = z_true + self.baro_bias + self.rng.normal(0, self.BARO_SIG)
        for f in self.F.active(self.id, "baro_bias", t):
            val += f.params["offset"]
        for f in self.F.active(self.id, "baro_stuck", t):
            if "val" not in f.state:
                f.state["val"] = val
            val = f.state["val"]
        return val

    def battery(self, t, soc_true):
        rep = soc_true
        for f in self.F.active(self.id, "battery_offset", t):
            rep += f.params["offset"]
        for f in self.F.active(self.id, "battery_stuck", t):
            if "val" not in f.state:
                f.state["val"] = rep
            rep = f.state["val"]
        return float(min(max(rep, 0.0), 100.0))


# ----------------------------------------------------------------------------
# Estimator: per-axis [pos, vel] Kalman filter, IMU as input, GPS+baro as updates
# ----------------------------------------------------------------------------
class AxisKF:
    __slots__ = ("p", "v", "a", "b", "c")

    def __init__(self, p0):
        self.p, self.v = p0, 0.0
        self.a, self.b, self.c = 0.5 ** 2, 0.0, 0.2 ** 2   # P = [[a, b], [b, c]]

    def predict(self, acc, dt, q):
        self.p += self.v * dt + 0.5 * acc * dt * dt
        self.v += acc * dt
        a, b, c = self.a, self.b, self.c
        self.a = a + 2 * dt * b + dt * dt * c + q * dt ** 4 / 4
        self.b = b + dt * c + q * dt ** 3 / 2
        self.c = c + q * dt * dt

    def update_pos(self, z, R, gate):
        y = z - self.p
        S = self.a + R
        nis = abs(y) / math.sqrt(S)
        if nis > gate:
            return False, nis
        k0, k1 = self.a / S, self.b / S
        self.p += k0 * y
        self.v += k1 * y
        a, b, c = self.a, self.b, self.c
        self.a, self.b, self.c = a - k0 * a, b - k0 * b, c - k1 * b
        return True, nis

    def update_vel(self, z, R, gate):
        y = z - self.v
        S = self.c + R
        nis = abs(y) / math.sqrt(S)
        if nis > gate:
            return False, nis
        k0, k1 = self.b / S, self.c / S
        self.p += k0 * y
        self.v += k1 * y
        a, b, c = self.a, self.b, self.c
        self.a, self.b, self.c = a - k0 * b, b - k0 * c, c - k1 * c
        return True, nis


class Estimator:
    GATE = 5.0   # innovation gate in sigmas (similar idea to PX4's EKF2 gates)

    def __init__(self, home):
        self.ax = [AxisKF(home[0]), AxisKF(home[1]), AxisKF(0.0)]
        self.q = 0.35 ** 2
        self.R_ph, self.R_pv = 0.6 ** 2, 1.2 ** 2
        self.R_v, self.R_baro = 0.2 ** 2, 0.4 ** 2

    @property
    def pos(self):
        return np.array([k.p for k in self.ax])

    @property
    def vel(self):
        return np.array([k.v for k in self.ax])

    def predict(self, acc, dt):
        for i in range(3):
            self.ax[i].predict(acc[i], dt, self.q)

    def update_gps(self, gps):
        rejected, nis_max = False, 0.0
        for i in range(3):
            Rp = self.R_ph if i < 2 else self.R_pv
            ok1, n1 = self.ax[i].update_pos(gps["pos"][i], Rp, self.GATE)
            ok2, n2 = self.ax[i].update_vel(gps["vel"][i], self.R_v, self.GATE)
            rejected = rejected or not (ok1 and ok2)
            nis_max = max(nis_max, n1, n2)
        return {"rejected": rejected, "nis": nis_max}

    def update_baro(self, z):
        ok, nis = self.ax[2].update_pos(z, self.R_baro, self.GATE)
        return {"rejected": not ok, "nis": nis}


# ----------------------------------------------------------------------------
# Autopilot: mission logic + failsafes (this is the "system under test")
# ----------------------------------------------------------------------------
ACTIVE_MODES = ("TAKEOFF", "MISSION", "RTL", "LAND")


class Autopilot:
    def __init__(self, cfg: DroneConfig, sim: SimConfig, params: Params):
        self.cfg, self.sim, self.p = cfg, sim, params
        self.mode = "PREFLIGHT"
        self.armed = False
        self.home = np.array([cfg.home[0], cfg.home[1], 0.0])
        self.wp_idx = 0
        self.mission_done = False
        self.gps_ok = True
        self.last_gps_t = 0.0
        self.rej_since = None
        self.drift_since = None
        self.baro_rej_since = None
        self.baro_warned = False
        self.gbuf = deque()
        self.soc = 100.0
        self.land_xy = None
        self.avoiding = False
        self.hold = None
        self.events = []

    # -- helpers -------------------------------------------------------------
    def event(self, t, kind, msg):
        self.events.append({"t": round(t, 2), "drone": self.cfg.id, "kind": kind, "msg": msg})

    def set_mode(self, t, mode, reason, kind="MODE"):
        if mode == self.mode:
            return
        if self.mode in ("LAND", "LANDED") and mode != "LANDED":
            return                      # LAND is latched
        self.event(t, kind, f"{self.mode} -> {mode}: {reason}")
        self.mode = mode
        if mode == "LAND":
            self.land_xy = None

    # -- monitors (run every tick, before control) ---------------------------
    def update_monitors(self, t, gps, gps_res, baro_res, soc):
        P = self.p
        self.soc = soc
        if gps is not None:
            self.last_gps_t = t
            if self.gps_ok:
                if gps_res and gps_res["rejected"]:
                    if self.rej_since is None:
                        self.rej_since = t
                else:
                    self.rej_since = None
                self._drift_check(t, gps)
        if self.gps_ok and self.mode in ACTIVE_MODES:
            if self.rej_since is not None and t - self.rej_since >= P.gps_reject_time:
                self._gps_fail(t, f"GPS innovation test failing for {t - self.rej_since:.1f}s (inconsistent with IMU/baro)")
            elif t - self.last_gps_t > P.gps_timeout:
                self._gps_fail(t, f"GPS data lost for {t - self.last_gps_t:.1f}s")
        if baro_res is not None:
            if baro_res["rejected"]:
                if self.baro_rej_since is None:
                    self.baro_rej_since = t
                elif t - self.baro_rej_since > 3.0 and not self.baro_warned:
                    self.baro_warned = True
                    self.event(t, "WARN", "barometer rejected by estimator for >3 s (using GPS height)")
            else:
                self.baro_rej_since = None

    def _gps_fail(self, t, msg):
        self.gps_ok = False
        self.event(t, "FAILSAFE", msg)
        if self.mode in ("TAKEOFF", "MISSION", "RTL"):
            self.set_mode(t, "LAND", "GPS unusable -> land with dead reckoning")

    def _drift_check(self, t, gps):
        P = self.p
        if not self.sim.drift_detector:
            return
        self.gbuf.append((t, gps["pos"].copy(), gps["vel"].copy()))
        while self.gbuf and t - self.gbuf[0][0] > P.drift_window:
            self.gbuf.popleft()
        if t - self.gbuf[0][0] < P.drift_min_span:
            return
        buf = list(self.gbuf)
        integ = np.zeros(2)
        for (t0, _, v0), (t1, _, v1) in zip(buf, buf[1:]):
            integ += 0.5 * (v0[:2] + v1[:2]) * (t1 - t0)
        resid = float(np.linalg.norm((buf[-1][1][:2] - buf[0][1][:2]) - integ))
        if resid > P.drift_thresh:
            if self.drift_since is None:
                self.drift_since = t
            elif t - self.drift_since >= P.drift_persist:
                self._gps_fail(t, f"GPS spoof detector: position change disagrees with GPS velocity by {resid:.1f} m")
        else:
            self.drift_since = None

    def _failsafes(self, t, p):
        if self.mode not in ("TAKEOFF", "MISSION", "RTL"):
            return
        P = self.p
        if self.soc < P.soc_land:
            self.set_mode(t, "LAND", f"battery critical ({self.soc:.0f}%)", "FAILSAFE")
            return
        if self.soc < P.soc_rtl and self.mode != "RTL":
            self.set_mode(t, "RTL", f"battery low ({self.soc:.0f}%)", "FAILSAFE")
        dist = float(np.linalg.norm(p[:2] - self.home[:2]))
        if (dist > self.sim.fence_radius or p[2] > self.sim.fence_max_alt) and self.mode != "RTL":
            self.set_mode(t, "RTL", f"geofence breach (est. {dist:.0f} m from home, {p[2]:.0f} m high)", "FAILSAFE")

    # -- guidance ------------------------------------------------------------
    def _vel_ctrl(self, v, vd):
        P = self.p
        a = P.kv * (vd - v)
        h = math.hypot(a[0], a[1])
        if h > P.amax_h:
            a[:2] *= P.amax_h / h
        a[2] = min(max(a[2], -P.amax_dn), P.amax_up)
        return a

    def _goto(self, p, v, tgt, vmax_h):
        P = self.p
        vd = P.kp_pos * (tgt - p)
        h = math.hypot(vd[0], vd[1])
        if h > vmax_h:
            vd[:2] *= vmax_h / h
        vd[2] = min(max(vd[2], -P.vmax_dn), P.vmax_up)
        return self._vel_ctrl(v, vd)

    def _avoid(self, t, p, tgt, neighbors):
        if not self.sim.avoidance:
            return tgt
        P = self.p
        mind = INF
        for nid, (trx, npos) in neighbors.items():
            if nid >= self.cfg.id or t - trx > P.v2v_stale:
                continue                # only yield to lower ids; ignore stale reports
            mind = min(mind, float(np.linalg.norm(p - npos)))
        if not self.avoiding and mind < P.avoid_enter:
            self.avoiding, self.hold = True, p.copy()
            self.event(t, "AVOID", f"traffic {mind:.1f} m away -> holding position")
        elif self.avoiding and mind > P.avoid_exit:
            self.avoiding = False
            self.event(t, "AVOID", "traffic clear -> resuming")
        return self.hold if self.avoiding else tgt

    # -- main control step ---------------------------------------------------
    def control(self, t, p, v, neighbors, on_ground):
        c, P = self.cfg, self.p
        if self.mode in ("LANDED", "PREARM_FAIL"):
            return ZERO3.copy()
        if self.mode == "PREFLIGHT":
            if t >= self.sim.arm_time:
                if self.soc < P.min_arm_soc:
                    self.event(t, "WARN", f"pre-arm check failed: battery {self.soc:.0f}% < {P.min_arm_soc:.0f}%")
                    self.mode = "PREARM_FAIL"
                else:
                    self.armed = True
                    self.home = p.copy()
                    self.set_mode(t, "TAKEOFF", "armed")
            return ZERO3.copy()
        if self.mode == "LAND" and on_ground:
            self.set_mode(t, "LANDED", "touchdown, disarmed")
            self.armed = False
            return ZERO3.copy()

        self._failsafes(t, p)
        m = self.mode

        if m == "TAKEOFF":
            tgt = np.array([self.home[0], self.home[1], c.cruise_alt])
            a = self._goto(p, v, tgt, c.cruise_speed)
            if p[2] > c.cruise_alt - 0.7:
                if c.waypoints:
                    self.set_mode(t, "MISSION", "takeoff complete")
                else:
                    self.mission_done = True
                    self.set_mode(t, "RTL", "no waypoints")
            return a

        if m == "MISSION":
            wp = np.array(c.waypoints[self.wp_idx], float)
            tgt = np.array([wp[0], wp[1], c.cruise_alt])
            if math.hypot(*(p[:2] - wp)) < P.wp_radius and abs(p[2] - c.cruise_alt) < 2.0:
                self.wp_idx += 1
                self.event(t, "INFO", f"waypoint {self.wp_idx}/{len(c.waypoints)} reached")
                if self.wp_idx >= len(c.waypoints):
                    self.mission_done = True
                    self.set_mode(t, "RTL", "mission complete")
            tgt = self._avoid(t, p, tgt, neighbors)
            return self._goto(p, v, tgt, c.cruise_speed)

        if m == "RTL":
            tgt = np.array([self.home[0], self.home[1], c.cruise_alt])
            if math.hypot(*(p[:2] - self.home[:2])) < P.wp_radius:
                self.land_xy = self.home[:2].copy()
                self.set_mode(t, "LAND", "above home")
            tgt = self._avoid(t, p, tgt, neighbors)
            return self._goto(p, v, tgt, c.cruise_speed)

        if m == "LAND":
            if self.land_xy is None:
                self.land_xy = p[:2].copy()
            vz = -P.land_speed if p[2] > 3.0 else -P.land_speed_low
            if self.gps_ok:
                vh = P.kp_pos * (self.land_xy - p[:2])
                n = float(np.linalg.norm(vh))
                if n > 1.5:
                    vh *= 1.5 / n
            else:
                vh = np.zeros(2)         # no position reference: just null the velocity
            return self._vel_ctrl(v, np.array([vh[0], vh[1], vz]))
        return ZERO3.copy()


# ----------------------------------------------------------------------------
# Truth physics
# ----------------------------------------------------------------------------
class Drone:
    TAU = 0.15            # attitude/thrust response lag (s)
    DRAG = 0.15           # linear drag coefficient (1/s)
    CRASH_SPEED = 3.0     # m/s impact speed considered a crash

    def __init__(self, cfg: DroneConfig, sim: SimConfig, faults: FaultManager, rng):
        self.id, self.sim, self.F, self.rng = cfg.id, sim, faults, rng
        self.pos = np.array([cfg.home[0], cfg.home[1], 0.0])
        self.vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.acc_total = np.zeros(3)
        self.hist = deque(maxlen=600)
        self.gust = np.zeros(3)
        self.soc = cfg.initial_soc
        self.power_loss = False
        self.collided = False
        self.airborne = False
        self.landed = False
        self.crashed = False
        self.on_ground = True
        self.impact_speed = None

    def step(self, t, cmd, dt, armed):
        self.hist.append(np.array(cmd, float))
        delay = 0.0
        for f in self.F.active(self.id, "cmd_delay", t):
            delay = max(delay, f.params.get("delay", 0.5))
        n = int(round(delay / dt))
        a_cmd = self.hist[-1 - n].copy() if n < len(self.hist) else np.zeros(3)
        h = math.hypot(a_cmd[0], a_cmd[1])
        if h > 6.0:
            a_cmd[:2] *= 6.0 / h
        a_cmd[2] = min(max(a_cmd[2], -4.0), 5.0)

        self.gust += (-self.gust / 3.0 * dt
                      + self.sim.gust_sigma * math.sqrt(2 * dt / 3.0) * self.rng.normal(0, 1, 3) * np.array([1, 1, 0.3]))
        wind = np.array(self.sim.wind, float) + self.gust
        for f in self.F.active(self.id, "wind_step", t):
            wind = wind + np.array(f.params["vec"], float)

        if self.power_loss:
            a_thrust = np.array([0.0, 0.0, -G])
        else:
            self.acc += (a_cmd - self.acc) * dt / self.TAU
            a_thrust = self.acc
        self.acc_total = a_thrust - self.DRAG * (self.vel - wind)
        if self.on_ground and a_thrust[2] <= 0.05:
            self.acc_total = np.zeros(3)           # sitting on the ground

        self.vel += self.acc_total * dt
        self.pos += self.vel * dt
        if self.pos[2] > 0.5:
            self.airborne = True
        if self.pos[2] <= 0.0 and self.vel[2] <= 0.0:
            if self.airborne and not self.landed:
                self.impact_speed = float(np.linalg.norm(self.vel))
                if self.impact_speed > self.CRASH_SPEED:
                    self.crashed = True
                self.landed = True
            self.pos[2] = 0.0
            self.vel[:] = 0.0
            self.acc[:] = 0.0
            self.on_ground = True
        else:
            self.on_ground = False

        if armed and not self.landed:
            self.soc -= self.sim.batt_drain * (1 + 0.05 * float(np.linalg.norm(self.vel))) * dt
            if self.soc <= 0:
                self.soc = 0.0
                self.power_loss = True


# ----------------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------------
class Vehicle:
    def __init__(self, dc, cfg, faults, seed, params):
        rng = np.random.default_rng([seed, dc.id])
        self.cfg = dc
        self.phys = Drone(dc, cfg, faults, rng)
        self.sens = Sensors(dc.id, faults, rng)
        self.est = Estimator(dc.home)
        self.ap = Autopilot(dc, cfg, params)
        self.neighbors = {}
        self.last_gps = None
        self.last_baro = float("nan")
        self.soc_rep = dc.initial_soc
        self.rows = []
        self.land_start = None      # true xy where the LAND phase began (to judge emergency landings)

    @property
    def terminal(self):
        return self.phys.landed or self.phys.crashed or self.ap.mode == "PREARM_FAIL"


class Simulation:
    def __init__(self, cfg: SimConfig, seed=1, params: Params = None, feed_dir="feeds"):
        self.cfg, self.seed = cfg, seed
        self.params = params or Params()
        self.faults = FaultManager(copy.deepcopy(cfg.faults), feed_dir)
        self.veh = [Vehicle(dc, cfg, self.faults, seed, self.params) for dc in cfg.drones]
        self.by_id = {v.cfg.id: v for v in self.veh}
        self.events = []
        self.min_sep = INF
        self.t_end = 0.0

    # -- main loop -----------------------------------------------------------
    def run(self):
        cfg, dt = self.cfg, self.cfg.dt
        gps_div = int(round(0.2 / dt))
        baro_div = int(round(0.04 / dt))
        v2v_div = int(round(0.5 / dt))
        log_div = int(round(0.1 / dt))
        n = int(cfg.duration / dt)
        done_at = None
        t = 0.0
        for k in range(n):
            t = k * dt
            self._announce_faults(t)
            for v in self.veh:
                acc_meas = v.sens.imu(t, v.phys.acc_total)
                v.est.predict(acc_meas, dt)
                gps = gres = bres = None
                if k % gps_div == 0:
                    gps = v.sens.gps(t, v.phys.pos, v.phys.vel)
                    if gps is not None:
                        v.last_gps = gps
                        if v.ap.gps_ok:
                            gres = v.est.update_gps(gps)
                if k % baro_div == 0:
                    v.last_baro = v.sens.baro(t, v.phys.pos[2])
                    bres = v.est.update_baro(v.last_baro)
                v.soc_rep = v.sens.battery(t, v.phys.soc)
                v.ap.update_monitors(t, gps, gres, bres, v.soc_rep)
            if k % v2v_div == 0:
                self._v2v(t)
            cmds = [v.ap.control(t, v.est.pos, v.est.vel, v.neighbors, v.phys.on_ground) for v in self.veh]
            for v, c in zip(self.veh, cmds):
                if v.ap.mode == "LAND" and v.land_start is None:
                    v.land_start = v.phys.pos[:2].copy()
                v.phys.step(t, c, dt, v.ap.armed)
            self._separation(t)
            if k % log_div == 0:
                self._log(t)
            if all(v.terminal for v in self.veh):
                done_at = t if done_at is None else done_at
                if t - done_at > 1.0:
                    break
        self.t_end = t
        return self.metrics()

    # -- helpers -------------------------------------------------------------
    def _announce_faults(self, t):
        for f in self.faults.faults:
            if f.active(t) and not f.state.get("_on"):
                f.state["_on"] = True
                self.events.append({"t": round(t, 2), "drone": f.drone, "kind": "FAULT",
                                    "msg": f"START {f.type} {json.dumps(f.params, default=str)}"})
            if t >= f.t_end and f.state.get("_on") and not f.state.get("_off"):
                f.state["_off"] = True
                self.events.append({"t": round(t, 2), "drone": f.drone, "kind": "FAULT", "msg": f"END {f.type}"})

    def _v2v(self, t):
        """2 Hz position broadcast between drones. This is where V2V false data enters."""
        for tx in self.veh:
            report = tx.est.pos.copy()
            for rx in self.veh:
                if rx is tx:
                    continue
                if self.faults.active(rx.cfg.id, "v2v_drop", t, tx.cfg.id):
                    continue
                frz = self.faults.active(rx.cfg.id, "v2v_freeze", t, tx.cfg.id)
                if frz:
                    f = frz[0]
                    if "pos" not in f.state:
                        prev = rx.neighbors.get(tx.cfg.id)
                        f.state["pos"] = prev[1].copy() if prev else report.copy()
                    rx.neighbors[tx.cfg.id] = (t, f.state["pos"].copy())   # fresh timestamp, stale position
                else:
                    rx.neighbors[tx.cfg.id] = (t, report)
        for f in self.faults.faults:
            if f.type == "v2v_ghost" and f.active(t):
                rx = self.by_id[f.drone]
                rx.neighbors[f.params.get("id", -1)] = (t, np.array(f.params["pos"], float))

    def _separation(self, t):
        for i in range(len(self.veh)):
            for j in range(i + 1, len(self.veh)):
                a, b = self.veh[i].phys, self.veh[j].phys
                d = float(np.linalg.norm(a.pos - b.pos))
                if a.airborne and b.airborne and not a.landed and not b.landed:
                    self.min_sep = min(self.min_sep, d)
                    if d < 1.0 and not a.collided:
                        a.collided = b.collided = True
                        a.power_loss = b.power_loss = True
                        self.events.append({"t": round(t, 2), "drone": -1, "kind": "COLLISION",
                                            "msg": f"drones {self.veh[i].cfg.id} and {self.veh[j].cfg.id} collided"})

    def _log(self, t):
        for v in self.veh:
            ph, ap = v.phys, v.ap
            e, ev = v.est.pos, v.est.vel
            g = v.last_gps["pos"] if v.last_gps else (float("nan"),) * 3
            v.rows.append({
                "t": round(t, 2),
                "x": ph.pos[0], "y": ph.pos[1], "z": ph.pos[2],
                "vx": ph.vel[0], "vy": ph.vel[1], "vz": ph.vel[2],
                "ex": e[0], "ey": e[1], "ez": e[2],
                "evx": ev[0], "evy": ev[1], "evz": ev[2],
                "gps_x": g[0], "gps_y": g[1], "gps_z": g[2], "baro_z": v.last_baro,
                "mode": ap.mode, "gps_ok": int(ap.gps_ok), "avoiding": int(ap.avoiding),
                "soc_true": ph.soc, "soc_reported": v.soc_rep,
                "est_err": float(np.linalg.norm(e - ph.pos)),
            })

    # -- metrics -------------------------------------------------------------
    def metrics(self):
        cfg = self.cfg
        rank = ["PASS", "SAFE_ABORT", "HUNG", "DEGRADED", "FAIL"]
        res = {"scenario": cfg.name, "description": cfg.description, "seed": self.seed,
               "sim_time_s": round(self.t_end, 1),
               "min_separation_m": round(self.min_sep, 2) if len(self.veh) > 1 and self.min_sep < INF else None,
               "drones": {}}
        worst = 0
        for v in self.veh:
            R, ph, ap = v.rows, v.phys, v.ap
            true = np.array([[r["x"], r["y"], r["z"]] for r in R])
            err = np.array([r["est_err"] for r in R])
            home = np.array(v.cfg.home, float)
            dist = np.linalg.norm(true[:, :2] - home, axis=1)
            breach = bool(dist.max() > cfg.fence_radius or true[:, 2].max() > cfg.fence_max_alt)
            land_err = float(np.linalg.norm(ph.pos[:2] - home)) if (ph.landed and not ph.crashed) else None
            ftimes = [f.t_start for f in self.faults.faults if f.drone == v.cfg.id and f.type not in NON_FAULT_TYPES]
            t_fault = min(ftimes) if ftimes else None
            fs = [e for e in ap.events if e["kind"] == "FAILSAFE" and (t_fault is None or e["t"] >= t_fault)]
            latency = round(fs[0]["t"] - t_fault, 2) if (fs and t_fault is not None) else None

            if ph.crashed or ph.collided:
                outcome = "FAIL"
                why = "mid-air collision" if ph.collided else f"hard impact at {ph.impact_speed:.1f} m/s" if ph.impact_speed else "power loss"
            elif breach:
                outcome, why = "FAIL", "true position left the geofence"
            elif ph.landed:
                if ap.mission_done:
                    # normal landing: should be at home
                    if land_err > 5.0:
                        outcome, why = "DEGRADED", f"landed {land_err:.1f} m from home (believed it was at home)"
                    else:
                        outcome, why = "PASS", "mission completed, landed at home"
                else:
                    # emergency landing in place: judge how far it drifted during the descent
                    drift = float(np.linalg.norm(ph.pos[:2] - v.land_start)) if v.land_start is not None else 0.0
                    if drift > 5.0:
                        outcome, why = "DEGRADED", f"emergency landing drifted {drift:.1f} m during descent"
                    else:
                        outcome, why = "SAFE_ABORT", f"emergency landing in place (drift {drift:.1f} m), mission not completed"
            elif ap.mode == "PREARM_FAIL":
                outcome, why = "SAFE_ABORT", "refused to arm"
            else:
                outcome, why = "HUNG", f"still in {ap.mode} at end of simulation"
            worst = max(worst, rank.index(outcome))
            res["drones"][v.cfg.id] = {
                "outcome": outcome, "reason": why,
                "mission_completed": bool(ap.mission_done),
                "crashed": bool(ph.crashed), "collided": bool(ph.collided),
                "final_mode": ap.mode,
                "geofence_breach_true": breach,
                "max_dist_from_home_m": round(float(dist.max()), 1),
                "max_est_error_m": round(float(err.max()), 2),
                "landing_error_m": None if land_err is None else round(land_err, 2),
                "fault_start_s": t_fault,
                "first_failsafe": fs[0]["msg"] if fs else None,
                "detection_latency_s": latency,
                "failsafe_triggered": bool(fs),
                "final_soc_true": round(ph.soc, 1),
            }
        D = res["drones"].values()
        res["outcome"] = rank[worst]
        res["max_est_error_m"] = max(d["max_est_error_m"] for d in D)
        le = [d["landing_error_m"] for d in D if d["landing_error_m"] is not None]
        res["landing_error_m"] = max(le) if le else None
        lat = [d["detection_latency_s"] for d in D if d["detection_latency_s"] is not None]
        res["detection_latency_s"] = min(lat) if lat else None
        res["first_failsafe"] = next((d["first_failsafe"] for d in D if d["first_failsafe"]), None)
        res["reason"] = "; ".join(f"d{k}: {d['reason']}" for k, d in res["drones"].items())
        return res

    # -- output --------------------------------------------------------------
    def all_events(self):
        ev = list(self.events)
        for v in self.veh:
            ev += v.ap.events
        return sorted(ev, key=lambda e: e["t"])

    def save(self, outdir, metrics=None):
        os.makedirs(outdir, exist_ok=True)
        for v in self.veh:
            if v.rows:
                with open(os.path.join(outdir, f"log_drone{v.cfg.id}.csv"), "w", newline="") as fh:
                    w = csv.DictWriter(fh, fieldnames=list(v.rows[0].keys()))
                    w.writeheader()
                    w.writerows(v.rows)
        with open(os.path.join(outdir, "events.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=["t", "drone", "kind", "msg"])
            w.writeheader()
            w.writerows(self.all_events())
        with open(os.path.join(outdir, "metrics.json"), "w") as fh:
            json.dump(metrics or self.metrics(), fh, indent=2, default=str)
