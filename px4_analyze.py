#!/usr/bin/env python3
"""
px4_analyze.py - metrics + plots for a px4_results/<scenario> folder (and optionally a PX4 .ulg log).

  python px4_analyze.py px4_results/gps_off
  python px4_analyze.py px4_results/gps_off --ulog ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-10-16/12_00_00.ulg

Reports: flight modes visited (failsafe reaction), time from first fault to first mode change,
max distance from home, landing distance from home, minimum separation between drones,
and (with --ulog) PX4's own logged messages and nav_state transitions.
"""
import argparse
import glob
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

NAV_STATE = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
             10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF",
             18: "AUTO_LAND", 19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND"}


def to_enu(df, lat0, lon0):
    n = (df.lat - lat0) * 111_320.0
    e = (df.lon - lon0) * 111_320.0 * math.cos(math.radians(lat0))
    return e.to_numpy(), n.to_numpy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--ulog")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.run_dir, "drone*.csv")))
    if not files:
        print("no drone*.csv in", a.run_dir)
        return 2
    acts = {}
    p = os.path.join(a.run_dir, "actions.json")
    if os.path.exists(p):
        acts = json.load(open(p))
    fault_times = [x["t_actual"] for lst in acts.values() for x in lst if x["do"] in ("inject", "spoof", "shell")]
    t_fault = min(fault_times) if fault_times else None

    dfs = [pd.read_csv(f) for f in files]
    ref = dfs[0].dropna(subset=["lat"]).iloc[0]
    lat0, lon0 = ref.lat, ref.lon

    print(f"== {a.run_dir}")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    for i, d in enumerate(dfs):
        d = d.dropna(subset=["lat"]).reset_index(drop=True)
        e, n = to_enu(d, lat0, lon0)
        dfs[i] = d.assign(e=e, n=n)
        home_e, home_n = e[0], n[0]
        dist = np.hypot(e - home_e, n - home_n)
        modes = d["mode"].ne(d["mode"].shift()).to_numpy()
        trans = d.loc[modes, ["t", "mode"]]
        print(f"-- drone {i}")
        print("   modes:", " -> ".join(f"{r.mode}@{r.t:.0f}s" for r in trans.itertuples()))
        print(f"   max distance from home: {dist.max():.1f} m, final: {dist[-1]:.1f} m, max alt: {d.alt_rel.max():.1f} m")
        if t_fault is not None:
            after = trans[trans.t > t_fault + 0.5]
            if len(after):
                r = after.iloc[0]
                print(f"   reaction: {r['mode']} {r['t'] - t_fault:.1f} s after first fault")
            else:
                print("   reaction: no flight-mode change after the fault")
        ax1.plot(e, n, label=f"drone {i}")
        ax2.plot(d.t, d.alt_rel, label=f"drone {i}")
    if len(dfs) > 1:
        a_, b_ = dfs[0], dfs[1]
        m = pd.merge_asof(a_[["t", "e", "n", "alt_rel"]].sort_values("t"), b_[["t", "e", "n", "alt_rel"]].sort_values("t"),
                          on="t", suffixes=("_0", "_1"), direction="nearest", tolerance=0.2).dropna()
        sep = np.sqrt((m.e_0 - m.e_1) ** 2 + (m.n_0 - m.n_1) ** 2 + (m.alt_rel_0 - m.alt_rel_1) ** 2)
        print(f"-- minimum separation between drones: {sep.min():.1f} m")
    if t_fault is not None:
        ax2.axvline(t_fault, color="red", ls=":", label="first fault")
    ax1.set(title="Ground track (ENU, m)", xlabel="east", ylabel="north")
    ax1.set_aspect("equal", "datalim")
    ax1.legend()
    ax2.set(title="Relative altitude", xlabel="t (s)", ylabel="m")
    ax2.legend()
    fig.tight_layout()
    out = os.path.join(a.run_dir, "plot.png")
    fig.savefig(out, dpi=110)
    print("plot ->", out)

    if a.ulog:
        from pyulog import ULog
        u = ULog(a.ulog)
        print("\n== PX4 log messages (warnings and worse)")
        for m in u.logged_messages:
            if m.log_level <= 4:
                print(f"   {m.timestamp / 1e6:8.1f}s  {m.message}")
        try:
            vs = u.get_dataset("vehicle_status").data
            ns = vs["nav_state"]
            ts = vs["timestamp"] / 1e6
            print("== nav_state transitions")
            last = None
            for t, s in zip(ts, ns):
                if s != last:
                    print(f"   {t:8.1f}s  {NAV_STATE.get(int(s), int(s))}")
                    last = s
        except Exception as ex:  # noqa: BLE001
            print("   (could not read vehicle_status:", ex, ")")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
