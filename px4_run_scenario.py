#!/usr/bin/env python3
"""
px4_run_scenario.py - fly a mission on PX4 SITL (Gazebo), inject faults at set times, log telemetry.

  python px4_run_scenario.py --list
  python px4_run_scenario.py baseline
  python px4_run_scenario.py gps_off
  python px4_run_scenario.py gps_spoof_walk --layout single
  python px4_run_scenario.py baro_stuck --layout two          # both drones fly, fault goes to drone 0

Prereq: ./px4_launch_sims.sh N is running. Written against MAVSDK-Python docs; NOT run by the author
(no PX4 in the authoring environment) - expect to tweak ports/params for your PX4 version.

Output: px4_results/<scenario>/drone<i>.csv, actions.json
"""
import argparse
import asyncio
import csv
import json
import math
import os
import subprocess
import sys
import time

from mavsdk import System
from mavsdk.failure import FailureType, FailureUnit
from mavsdk.mission import MissionItem, MissionPlan

# ---------------------------------------------------------------------------------------------
# Mission geometry (east, north) metres, relative to each drone's own home.
# Matches the pure-Python simulator.
# ---------------------------------------------------------------------------------------------
LAYOUTS = {
    "single": [[(30, 0), (30, 30), (0, 30), (0, 0)]],
    "two": [[(30, 0), (0, 0)],       # drone 0 spawns at (0,0)   flies east and back
            [(0, 30), (0, 0)]],      # drone 1 spawns at (15,-15) flies north and back -> crossing at (15,0)
}
ALT_M = 10.0
SPEED = 4.0
CFG = {"alt": ALT_M, "speed": SPEED, "fence_hor": 50.0, "fence_ver": 30.0, "timeout": 240.0}

# Campus layout: written by campus/make_campus_world.py. Waypoints there are world metres, so convert
# them to offsets from each drone's own spawn point (that is where PX4 sets "home").
CAMPUS_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "campus", "layout.json")
_CAMPUS = None
if os.path.exists(CAMPUS_JSON):
    _CAMPUS = json.load(open(CAMPUS_JSON))
    LAYOUTS["campus"] = [[(w[0] - d["spawn"][0], w[1] - d["spawn"][1]) for w in d["waypoints"]] + [(0.0, 0.0)]
                         for d in _CAMPUS["drones"]]

# ---------------------------------------------------------------------------------------------
# Scenarios: list of timed actions. t = seconds after mission start. "drone" = which vehicle.
#   inject : MAVSDK failure plugin (needs SYS_FAILURE_EN=1, set automatically)
#            unit  in FailureUnit.*, type in OK/OFF/STUCK/GARBAGE/WRONG/SLOW/DELAYED/INTERMITTENT
#   spoof  : start px4_gps_spoofer.py with a feed CSV (also turns real GPS off)
#   unspoof: stop spoofer and restore real GPS
#   shell  : run a shell command (e.g. tc netem for link loss; needs sudo) - edit to taste
#   param  : set a PX4 parameter {"name","value"}
# ---------------------------------------------------------------------------------------------
SCENARIOS = {
    "baseline": [],
    "gps_off": [{"t": 25, "do": "inject", "unit": "SENSOR_GPS", "type": "OFF"}],
    "gps_off_short": [{"t": 25, "do": "inject", "unit": "SENSOR_GPS", "type": "OFF"},
                      {"t": 28, "do": "inject", "unit": "SENSOR_GPS", "type": "OK"}],
    "gps_stuck": [{"t": 25, "do": "inject", "unit": "SENSOR_GPS", "type": "STUCK"}],
    "gps_wrong": [{"t": 25, "do": "inject", "unit": "SENSOR_GPS", "type": "WRONG"}],
    "gps_intermittent": [{"t": 25, "do": "inject", "unit": "SENSOR_GPS", "type": "INTERMITTENT"}],
    "baro_stuck": [{"t": 15, "do": "inject", "unit": "SENSOR_BARO", "type": "STUCK"}],
    "baro_wrong": [{"t": 15, "do": "inject", "unit": "SENSOR_BARO", "type": "WRONG"}],
    "accel_wrong": [{"t": 20, "do": "inject", "unit": "SENSOR_ACCEL", "type": "WRONG"}],
    "gyro_stuck": [{"t": 20, "do": "inject", "unit": "SENSOR_GYRO", "type": "STUCK"}],
    "mag_wrong": [{"t": 20, "do": "inject", "unit": "SENSOR_MAG", "type": "WRONG"}],
    "motor_off": [{"t": 20, "do": "inject", "unit": "SYSTEM_MOTOR", "type": "OFF", "instance": 1}],
    "gps_spoof_freeze": [{"t": 20, "do": "spoof", "feed": "feeds/px4_ghost_freeze.csv"}],
    "gps_spoof_walk": [{"t": 20, "do": "spoof", "feed": "feeds/px4_ghost_walk.csv"}],
    "gps_spoof_walk_stop": [{"t": 20, "do": "spoof", "feed": "feeds/px4_ghost_walk.csv"},
                            {"t": 40, "do": "unspoof"}],
    # link degradation on loopback (needs sudo; remove afterwards: sudo tc qdisc del dev lo root)
    "link_delay_500ms": [{"t": 20, "do": "shell", "cmd": "sudo tc qdisc add dev lo root netem delay 500ms"},
                         {"t": 40, "do": "shell", "cmd": "sudo tc qdisc del dev lo root"}],
}

FAILURE_UNITS = {u.name: u for u in FailureUnit}
FAILURE_TYPES = {u.name: u for u in FailureType}


def enu_to_latlon(lat0, lon0, east, north):
    return (lat0 + north / 111_320.0, lon0 + east / (111_320.0 * math.cos(math.radians(lat0))))


async def connect(idx):
    drone = System(port=50040 + idx)           # one embedded mavsdk_server per vehicle
    await drone.connect(system_address=f"udpin://0.0.0.0:{14540 + idx}")
    async for st in drone.core.connection_state():
        if st.is_connected:
            break
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            break
    return drone


def build_plan(home, wps_enu):
    items = []
    for east, north in wps_enu:
        lat, lon = enu_to_latlon(home.latitude_deg, home.longitude_deg, east, north)
        items.append(MissionItem(lat, lon, CFG["alt"], CFG["speed"], True, float("nan"), float("nan"),
                                 MissionItem.CameraAction.NONE, float("nan"), float("nan"),
                                 float("nan"), float("nan"), float("nan"),
                                 MissionItem.VehicleAction.NONE))
    return MissionPlan(items)


async def run_drone(idx, scenario, wps, actions, outdir, timeout, results):
    drone = await connect(idx)
    home = None
    async for h in drone.telemetry.home():
        home = h
        break
    print(f"[d{idx}] connected, home {home.latitude_deg:.6f},{home.longitude_deg:.6f}")

    # safety/fault-injection parameters
    await drone.param.set_param_int("SYS_FAILURE_EN", 1)
    await drone.param.set_param_int("GF_ACTION", 3)                 # geofence -> return
    await drone.param.set_param_float("GF_MAX_HOR_DIST", CFG["fence_hor"])
    await drone.param.set_param_float("GF_MAX_VER_DIST", CFG["fence_ver"])
    for a in actions:
        if a["do"] == "param" and a.get("drone", 0) == idx:
            v = a["value"]
            await (drone.param.set_param_int if isinstance(v, int) else drone.param.set_param_float)(a["name"], v)

    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(build_plan(home, wps))

    # telemetry sampling
    st = {"mode": "", "in_air": False, "bat": float("nan"), "sats": 0, "fix": ""}
    pos, vel = [float("nan")] * 4, [float("nan")] * 3

    async def watch(stream, fn):
        async for x in stream:
            fn(x)

    def on_pos(p):
        pos[:] = [p.latitude_deg, p.longitude_deg, p.absolute_altitude_m, p.relative_altitude_m]

    def on_vel(v):
        vel[:] = [v.north_m_s, v.east_m_s, v.down_m_s]
    tasks = [
        asyncio.create_task(watch(drone.telemetry.position(), on_pos)),
        asyncio.create_task(watch(drone.telemetry.velocity_ned(), on_vel)),
        asyncio.create_task(watch(drone.telemetry.flight_mode(), lambda m: st.update(mode=m.name))),
        asyncio.create_task(watch(drone.telemetry.in_air(), lambda b: st.update(in_air=b))),
        asyncio.create_task(watch(drone.telemetry.battery(), lambda b: st.update(bat=b.remaining_percent))),
        asyncio.create_task(watch(drone.telemetry.gps_info(), lambda g: st.update(sats=g.num_satellites, fix=g.fix_type.name))),
    ]
    os.makedirs(outdir, exist_ok=True)
    fh = open(os.path.join(outdir, f"drone{idx}.csv"), "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t", "lat", "lon", "alt_amsl", "alt_rel", "vn", "ve", "vd", "mode", "in_air", "battery_pct", "gps_sats", "gps_fix"])
    t0 = None
    done_actions = []
    spoof_proc = None

    async def logger():
        while True:
            if t0 is not None:
                w.writerow([f"{time.time() - t0:.2f}"] + pos + vel + [st["mode"], int(st["in_air"]), st["bat"], st["sats"], st["fix"]])
                fh.flush()
            await asyncio.sleep(0.1)
    tasks.append(asyncio.create_task(logger()))

    await drone.action.arm()
    await drone.mission.start_mission()
    t0 = time.time()
    print(f"[d{idx}] mission started")

    mine = sorted([a for a in actions if a.get("drone", 0) == idx and a["do"] != "param"], key=lambda a: a["t"])
    try:
        was_in_air = False
        while time.time() - t0 < timeout:
            el = time.time() - t0
            while mine and mine[0]["t"] <= el:
                a = mine.pop(0)
                print(f"[d{idx}] t={el:5.1f}s  {a}")
                if a["do"] == "inject":
                    await drone.failure.inject(FAILURE_UNITS[a["unit"]], FAILURE_TYPES[a["type"]], a.get("instance", 0))
                elif a["do"] == "spoof":
                    await drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OFF, 0)      # silence sim GPS
                    spoof_proc = subprocess.Popen([
                        sys.executable, "px4_gps_spoofer.py", "--feed", a["feed"], "--instance", str(idx),
                        "--anchor-lat", str(pos[0]), "--anchor-lon", str(pos[1]), "--anchor-alt", str(pos[2])])
                elif a["do"] == "unspoof":
                    if spoof_proc:
                        spoof_proc.terminate()
                    await drone.failure.inject(FailureUnit.SENSOR_GPS, FailureType.OK, 0)
                elif a["do"] == "shell":
                    subprocess.run(a["cmd"], shell=True, check=False)
                done_actions.append({"drone": idx, "t_actual": round(el, 2), **a})
            was_in_air = was_in_air or st["in_air"]
            if was_in_air and not st["in_air"]:
                print(f"[d{idx}] landed")
                await asyncio.sleep(2)
                break
            await asyncio.sleep(0.2)
        else:
            print(f"[d{idx}] timeout - forcing land")
            await drone.action.land()
    finally:
        if spoof_proc:
            spoof_proc.terminate()
        for t in tasks:
            t.cancel()
        fh.close()
    results[idx] = done_actions


async def amain(a):
    if a.scenario not in SCENARIOS:
        print("unknown scenario; use --list")
        return 2
    wps_all = LAYOUTS[a.layout]
    if a.layout == "campus":
        CFG.update(alt=_CAMPUS["alt_m"], speed=_CAMPUS["speed_m_s"], fence_hor=_CAMPUS["fence_hor_m"],
                   fence_ver=_CAMPUS["fence_ver_m"], timeout=_CAMPUS["timeout_s"])
    if a.timeout is None:
        a.timeout = CFG["timeout"]
    actions = [dict(x) for x in SCENARIOS[a.scenario]]
    for x in actions:
        x.setdefault("drone", a.fault_drone)
    outdir = os.path.join(a.out, a.scenario)
    results = {}
    await asyncio.gather(*[run_drone(i, a.scenario, wps, actions, outdir, a.timeout, results)
                           for i, wps in enumerate(wps_all)])
    with open(os.path.join(outdir, "actions.json"), "w") as fh:
        json.dump(results, fh, indent=2)
    print(f"done -> {outdir}   (analyze: python px4_analyze.py {outdir})")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--layout", choices=list(LAYOUTS), default="single")
    ap.add_argument("--fault-drone", type=int, default=0, help="which drone receives the fault actions")
    ap.add_argument("--timeout", type=float, default=None, help="seconds before forcing a landing (default 240, campus 420)")
    ap.add_argument("--out", default="px4_results")
    a = ap.parse_args()
    if a.list or not a.scenario:
        for n, acts in SCENARIOS.items():
            print(f"{n:22s} {[(x['t'], x['do'], x.get('unit', ''), x.get('type', '')) for x in acts]}")
        return 0
    return asyncio.run(amain(a))


if __name__ == "__main__":
    sys.exit(main())
