#!/usr/bin/env python3
"""
make_feeds.py - generate the false-data feed files in ./feeds

Simulator feeds (offset applied ON TOP of the true GPS position, closed-loop):
    columns: t,dx,dy,dz        t = seconds since the fault started, offsets in metres
    gps_walkoff.csv   smooth pull-off: spoofed drift speed ramps 0 -> 0.5 m/s over 20 s (heading NE)
    gps_circle.csv    15 m radius circle, period 60 s

PX4 "ghost track" feeds (open-loop, fed to px4_gps_spoofer.py as GPS_INPUT):
    columns: t,north,east,down  displacement of the *reported* position from where it was when spoofing began
    px4_ghost_freeze.csv   reported position never moves (vehicle "believes" it is hovering)
    px4_ghost_walk.csv     reported position walks north at 0.5 m/s

Edit this file or drop in your own CSVs (same columns) to feed any false data you like.
"""
import os

import numpy as np

os.makedirs("feeds", exist_ok=True)


def save(name, header, *cols):
    np.savetxt(os.path.join("feeds", name), np.column_stack(cols), delimiter=",", header=header, comments="", fmt="%.4f")
    print("wrote feeds/" + name)


# --- simulator feeds
t = np.arange(0, 301, 1.0)
speed = 0.5 * np.clip(t / 20.0, 0, 1)
dist = np.concatenate([[0], np.cumsum((speed[1:] + speed[:-1]) / 2)])
save("gps_walkoff.csv", "t,dx,dy,dz", t, dist * np.sqrt(0.5), dist * np.sqrt(0.5), np.zeros_like(t))

w = 2 * np.pi / 60.0
save("gps_circle.csv", "t,dx,dy,dz", t, 15 * np.sin(w * t), 15 * (1 - np.cos(w * t)), np.zeros_like(t))

# --- PX4 ghost tracks
t = np.arange(0, 601, 1.0)
z = np.zeros_like(t)
save("px4_ghost_freeze.csv", "t,north,east,down", t, z, z, z)
save("px4_ghost_walk.csv", "t,north,east,down", t, 0.5 * t, z, z)
