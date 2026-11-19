#!/usr/bin/env python3
"""
remote_id.py - Remote ID broadcast, a passive observer and privacy mitigations, built on the existing simulator.

QUESTION: do repeated Remote ID broadcasts of delivery flights reveal where customers live, and how much do
simple mitigations help (and what do they cost)?

MODEL (all simulated, see docs/assumptions.md)
  * A depot at (0,0) and K customer addresses (random, fixed per seed). The drone flies depot -> customer,
    hovers `dwell` s to drop the parcel, returns. Every customer receives `deliveries` deliveries.
    Flights are run by sim.py (same physics, sensors, Kalman filter and autopilot as the fault study).
  * Remote ID: once per second while airborne the drone broadcasts (pseudonym, ESTIMATED position, time).
    Because the broadcast uses the drone's own estimate, GPS faults corrupt it.
  * Observer: passive receiver that hears every broadcast (range not limited, no packet loss - worst case for privacy).
    It knows the depot and that a delivery has a short hover half-way through the flight. For each flight it estimates the
    address as the mean broadcast position around the middle of the flight, then clusters flight estimates
    (greedy, radius `cluster_radius`) to list "addresses".
  * Conditions (the mitigation is applied to the broadcast, the flights are identical = paired comparison):
      none           broadcast the estimated position, one fixed ID for the drone (static serial number)
      id_rotation    new random pseudonym for every flight (positions unchanged)
      noise_25/100   add Gaussian noise (sigma 25 m / 100 m) to every broadcast position, independently
      grid_100       snap every broadcast position to a 100 m grid
      gps_spoof      no mitigation, but a consistent GPS spoof (position + velocity) acts on the drone from t = 20 s
  * Metrics per condition and seed:
      flight_err_m        mean distance between the observer's estimate and the true customer address
      flight_hit_pct      % of flights estimated within `hit_radius` of the true address
      address_recovery_pct  % of true addresses that have an inferred cluster centre within `hit_radius`
      id_linkable_pct     % of flights the observer can link to the same drone by its ID (100 = fully linkable)
      broadcast_err_m     mean distance between what is broadcast and where the drone really is (utility loss/integrity)

  python remote_id.py                 # 5 seeds -> results_remote_id/
  python remote_id.py --seeds 7 --customers 8 --deliveries 3
  python remote_id.py --analyze-only  # rebuild stats/charts from runs.csv
"""
import argparse
import csv
import json
import os
import platform
import sys
import time
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

from sim import DroneConfig, Fault, Simulation, SimConfig

CONDITIONS = ["none", "id_rotation", "noise_25", "noise_100", "grid_100", "gps_spoof"]
METRICS = ["flight_err_m", "flight_hit_pct", "address_recovery_pct", "id_linkable_pct", "broadcast_err_m"]
SPOOF = Fault("gps_drift", 20, params={"rate": [0.4, 0.2, 0.0], "spoof_velocity": True})


@dataclass
class RID:
    customers: int = 8
    deliveries: int = 3
    r_min: float = 60.0
    r_max: float = 220.0
    min_sep: float = 80.0        # customers are at least this far apart (m)
    speed: float = 5.0
    dwell: float = 12.0          # hover at the address (s)
    alt: float = 10.0
    hit_radius: float = 30.0     # an estimate within this distance of the address counts as a hit
    cluster_radius: float = 40.0


def place_customers(cfg, seed):
    rng = np.random.default_rng([seed, 99])
    pts = []
    while len(pts) < cfg.customers:
        r = np.sqrt(rng.uniform(cfg.r_min ** 2, cfg.r_max ** 2))
        a = rng.uniform(0, 2 * np.pi)
        p = np.array([r * np.cos(a), r * np.sin(a)])
        if all(np.linalg.norm(p - q) >= cfg.min_sep for q in pts):
            pts.append(p)
    return np.array(pts)


def fly(cfg, addr, seed, spoof=False):
    """One delivery flight. Returns 1 Hz arrays of true and estimated xy while airborne."""
    d = DroneConfig(id=0, home=(0, 0), waypoints=[tuple(addr)], cruise_alt=cfg.alt,
                    cruise_speed=cfg.speed, dwell_s=cfg.dwell)
    sc = SimConfig(name="delivery", drones=[d], faults=[SPOOF] if spoof else [], duration=260.0,
                   fence_radius=600.0, fence_max_alt=60.0)
    sim = Simulation(sc, seed=seed, feed_dir="feeds")
    m = sim.run()
    rows = [r for r in sim.veh[0].rows if abs(r["t"] - round(r["t"])) < 1e-6 and r["z"] > 1.0]   # 1 Hz, airborne
    return {"t": np.array([r["t"] for r in rows]),
            "true": np.array([[r["x"], r["y"]] for r in rows]),
            "est": np.array([[r["ex"], r["ey"]] for r in rows]),
            "outcome": m["outcome"]}


def mitigate(cond, est, rng):
    if cond in ("none", "id_rotation", "gps_spoof"):
        return est.copy()
    if cond.startswith("noise_"):
        return est + rng.normal(0.0, float(cond.split("_")[1]), est.shape)
    if cond.startswith("grid_"):
        g = float(cond.split("_")[1])
        return (np.floor(est / g) + 0.5) * g
    raise ValueError(cond)


def observer_estimate(cfg, t, pos):
    """Mean broadcast position in a window around the middle of the flight (where the delivery hover happens)."""
    tm = 0.5 * (t[0] + t[-1])
    w = cfg.dwell / 2 + 2
    sel = np.abs(t - tm) <= w
    return pos[sel].mean(axis=0)


def cluster(points, radius):
    cents, members = [], []
    for p in points:
        for i, c in enumerate(cents):
            if np.linalg.norm(p - c) <= radius:
                members[i].append(p)
                cents[i] = np.mean(members[i], axis=0)
                break
        else:
            cents.append(p.copy())
            members.append([p])
    return np.array(cents)


def run_seed(cfg, seed):
    addrs = place_customers(cfg, seed)
    plan = [(c, k) for c in range(len(addrs)) for k in range(cfg.deliveries)]
    flights = {False: {}, True: {}}
    for spoof in (False, True):
        for (c, k) in plan:
            flights[spoof][(c, k)] = fly(cfg, addrs[c], seed * 1000 + c * 10 + k, spoof)
    out, detail = [], []
    for cond in CONDITIONS:
        spoof = cond == "gps_spoof"
        rng = np.random.default_rng([seed, 7, CONDITIONS.index(cond)])
        est_pts, errs, berr = [], [], []
        for (c, k), f in flights[spoof].items():
            b = mitigate(cond, f["est"], rng)
            e = observer_estimate(cfg, f["t"], b)
            est_pts.append(e)
            errs.append(float(np.linalg.norm(e - addrs[c])))
            berr.append(float(np.linalg.norm(b - f["true"], axis=1).mean()))
            detail.append({"seed": seed, "condition": cond, "customer": c, "delivery": k,
                           "est_x": round(float(e[0]), 1), "est_y": round(float(e[1]), 1),
                           "addr_x": round(float(addrs[c][0]), 1), "addr_y": round(float(addrs[c][1]), 1),
                           "err_m": round(errs[-1], 1), "outcome": f["outcome"]})
        cents = cluster(est_pts, cfg.cluster_radius)
        rec = [any(np.linalg.norm(a - c) <= cfg.hit_radius for c in cents) for a in addrs]
        out.append({"seed": seed, "condition": cond, "flights": len(errs),
                    "flight_err_m": round(float(np.mean(errs)), 2),
                    "flight_hit_pct": round(100.0 * float(np.mean([x <= cfg.hit_radius for x in errs])), 1),
                    "address_recovery_pct": round(100.0 * float(np.mean(rec)), 1),
                    "id_linkable_pct": 0.0 if cond == "id_rotation" else 100.0,
                    "broadcast_err_m": round(float(np.mean(berr)), 2),
                    "inferred_addresses": len(cents), "true_addresses": len(addrs)})
    return out, detail


def stat(vals):
    from run_multi import stat as s
    return s(vals)


def analyze(outdir):
    runs = list(csv.DictReader(open(os.path.join(outdir, "runs.csv"))))
    by = OrderedDict((c, [r for r in runs if r["condition"] == c]) for c in CONDITIONS)
    rows = []
    for c, rs in by.items():
        row = {"condition": c, "seeds": len(rs)}
        for m in METRICS:
            s = stat([r[m] for r in rs])
            row.update({f"{m}_mean": s["mean"], f"{m}_ci95": s["ci95"]})
        rows.append(row)
    with open(os.path.join(outdir, "stats.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    def f(r, m, nd=1):
        mean, ci = r[f"{m}_mean"], r[f"{m}_ci95"]
        return "" if mean is None else (f"{mean:.{nd}f}" if ci is None else f"{mean:.{nd}f} ± {ci:.{nd}f}")
    hdr = ["Condition", "Flight error (m)", "Flights within hit radius (%)", "Addresses recovered (%)",
           "Flights linkable by ID (%)", "Broadcast vs true position (m)"]
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for r in rows:
        lines.append("| " + " | ".join([r["condition"], f(r, "flight_err_m"), f(r, "flight_hit_pct"),
                                        f(r, "address_recovery_pct"), f(r, "id_linkable_pct", 0),
                                        f(r, "broadcast_err_m")]) + " |")
    open(os.path.join(outdir, "stats.md"), "w").write(
        "# Remote ID inference results\n\n" + "\n".join(lines) +
        f"\n\nMean ± 95 % CI (t distribution) over {rows[0]['seeds']} seeds. Hit radius 30 m, cluster radius 40 m.\n")
    chart(outdir, rows)
    return rows


def chart(outdir, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = [r["condition"] for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6), dpi=150)
    spec = [("flight_hit_pct", "Flights estimated within 30 m (%)", "#c23b3b"),
            ("address_recovery_pct", "Addresses recovered (%)", "#d98e1f"),
            ("broadcast_err_m", "Broadcast vs true position (m)\n(utility loss / integrity)", "#2f6db5")]
    for a, (m, title, col) in zip(ax, spec):
        for i, r in enumerate(rows):
            v = r[f"{m}_mean"]
            ci = r[f"{m}_ci95"] or 0.0
            a.bar(i, v, color=col, width=0.6)
            a.errorbar(i, v, yerr=ci, color="#222", capsize=2, linewidth=1)
            a.text(i, v + ci + (1 if m != "broadcast_err_m" else 1.5), f"{v:.0f}", ha="center", fontsize=7)
        a.set_xticks(range(len(names)))
        a.set_xticklabels(names, rotation=35, ha="right", fontsize=7)
        a.set_title(title, fontsize=9)
        a.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Remote ID observer: what is inferred, and what each mitigation costs (mean, 95% CI bars)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "remote_id_summary.png"))
    plt.close(fig)
    # illustrative map for seed 1
    det = list(csv.DictReader(open(os.path.join(outdir, "flights.csv"))))
    seed = min(int(d["seed"]) for d in det)
    show = ["none", "noise_100", "grid_100"]
    fig, axs = plt.subplots(1, 3, figsize=(12, 4.2), dpi=150, sharex=True, sharey=True)
    for a, c in zip(axs, show):
        d = [x for x in det if x["condition"] == c and int(x["seed"]) == seed]
        ad = {(float(x["addr_x"]), float(x["addr_y"])) for x in d}
        a.scatter([p[0] for p in ad], [p[1] for p in ad], marker="x", s=70, color="black", label="true address", zorder=3)
        a.scatter([float(x["est_x"]) for x in d], [float(x["est_y"]) for x in d], s=14, color="#c23b3b",
                  alpha=0.8, label="observer estimate (one per flight)", zorder=2)
        a.scatter([0], [0], marker="s", s=60, color="#2f6db5", label="depot", zorder=3)
        a.set_title(c, fontsize=10)
        a.set_aspect("equal")
        a.set_xlabel("x east (m)")
        a.spines[["top", "right"]].set_visible(False)
    axs[0].set_ylabel("y north (m)")
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=3, fontsize=8, frameon=False)
    fig.suptitle(f"Where the observer thinks each delivery went (seed {seed})", fontsize=10)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(os.path.join(outdir, "remote_id_map.png"))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--first-seed", type=int, default=1)
    ap.add_argument("--customers", type=int, default=8)
    ap.add_argument("--deliveries", type=int, default=3)
    ap.add_argument("--out", default="results_remote_id")
    ap.add_argument("--analyze-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if not a.analyze_only:
        cfg = RID(customers=a.customers, deliveries=a.deliveries)
        t0 = time.time()
        allrows, alldet = [], []
        for sd in range(a.first_seed, a.first_seed + a.seeds):
            o, d = run_seed(cfg, sd)
            allrows += o
            alldet += d
            print(f"seed {sd} done ({len(d) // len(CONDITIONS)} flights x 2)", flush=True)
        for name, rows in (("runs.csv", allrows), ("flights.csv", alldet)):
            with open(os.path.join(a.out, name), "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
        import scipy
        json.dump(OrderedDict(command="python " + " ".join(sys.argv), config=cfg.__dict__,
                              seeds=list(range(a.first_seed, a.first_seed + a.seeds)), conditions=CONDITIONS,
                              python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                              wall_time_s=round(time.time() - t0, 1), finished=time.strftime("%Y-%m-%d %H:%M:%S")),
                  open(os.path.join(a.out, "manifest.json"), "w"), indent=1)
    rows = analyze(a.out)
    print(open(os.path.join(a.out, "stats.md")).read())
    return 0


if __name__ == "__main__":
    sys.exit(main())
