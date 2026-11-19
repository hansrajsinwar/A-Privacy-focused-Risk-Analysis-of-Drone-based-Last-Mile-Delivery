#!/usr/bin/env python3
"""
px4_gps_spoofer.py - EXPERIMENTAL false-GPS feed for PX4 SITL (sends MAVLink GPS_INPUT).

Open-loop "ghost track": the position PX4 is told is  anchor + feed(t)  regardless of where the
vehicle really is. Feed CSV columns: t,north,east,down  (metres, t = seconds since spoofing began).

  feeds/px4_ghost_freeze.csv  -> vehicle believes it is hovering still while it really flies on
  feeds/px4_ghost_walk.csv    -> vehicle believes it is walking north at 0.5 m/s

To make PX4 use ONLY this GPS, silence the simulated one first (px4_run_scenario.py does this in the
`gps_spoof_*` scenarios: failure inject SENSOR_GPS OFF, then this script).

Not run by the author - verify in PX4 with `listener sensor_gps` (needs the pxh shell, i.e. start
px4 without -d) or watch the EKF/position in QGroundControl. PX4 accepts GPS_INPUT in recent versions;
if yours ignores it, use the failure-injection scenarios instead.

  python px4_gps_spoofer.py --feed feeds/px4_ghost_walk.csv --instance 0 \
        --anchor-lat 47.397742 --anchor-lon 8.545594 --anchor-alt 488.0
"""
import argparse
import math
import time

import numpy as np
from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--feed", required=True)
    ap.add_argument("--instance", type=int, default=0, help="PX4 instance (port 14580+instance)")
    ap.add_argument("--anchor-lat", type=float, required=True, help="where the vehicle is when spoofing starts (deg)")
    ap.add_argument("--anchor-lon", type=float, required=True)
    ap.add_argument("--anchor-alt", type=float, required=True, help="AMSL metres")
    ap.add_argument("--rate", type=float, default=5.0, help="Hz")
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--no-velocity", action="store_true", help="send zero velocity (inconsistent spoof)")
    a = ap.parse_args()

    feed = np.loadtxt(a.feed, delimiter=",", skiprows=1)
    port = 14580 + a.instance
    link = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=255, source_component=190)

    def pos(el):
        return [float(np.interp(el, feed[:, 0], feed[:, k])) for k in (1, 2, 3)]

    t0, period = time.time(), 1.0 / a.rate
    print(f"spoofing instance {a.instance} via udp 127.0.0.1:{port}, feed {a.feed}")
    while True:
        el = time.time() - t0
        if el > a.duration:
            break
        n, e, d = pos(el)
        n2, e2, d2 = pos(el + 0.5)
        n1, e1, d1 = pos(max(el - 0.5, 0.0))
        vn, ve, vd = ((n2 - n1), (e2 - e1), (d2 - d1)) if not a.no_velocity else (0.0, 0.0, 0.0)
        lat = a.anchor_lat + n / 111_320.0
        lon = a.anchor_lon + e / (111_320.0 * math.cos(math.radians(a.anchor_lat)))
        alt = a.anchor_alt - d
        link.mav.gps_input_send(
            int(time.time() * 1e6),   # time_usec
            0,                        # gps_id
            0,                        # ignore_flags: use everything
            0, 0,                     # time_week_ms, time_week
            3,                        # fix_type: 3D
            int(lat * 1e7), int(lon * 1e7), alt,
            0.8, 1.0,                 # hdop, vdop
            vn, ve, vd,               # velocity (m/s, NED)
            0.2, 0.5, 0.8,            # speed / horizontal / vertical accuracy
            12,                       # satellites
        )
        time.sleep(period)


if __name__ == "__main__":
    main()
