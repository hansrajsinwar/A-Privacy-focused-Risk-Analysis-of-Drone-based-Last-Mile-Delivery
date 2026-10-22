#!/usr/bin/env python3
"""
run_all.py - run the scenario matrix and write logs, plots and a summary.

  python run_all.py                       # every scenario, 1 run each
  python run_all.py --list                # list scenarios
  python run_all.py --only baseline gps_step_jump
  python run_all.py --runs 20             # Monte-Carlo: 20 random seeds per scenario
  python run_all.py --custom my_case.json
  python run_all.py --no-plots

Output: results/<scenario>/{log_drone*.csv, events.csv, metrics.json, plot.png}
        results/summary.csv, results/summary.md
"""
import argparse
import csv
import os
import sys
import time
from collections import Counter

import numpy as np

from report import markdown_table, plot_run
from scenarios import ALL, load_custom
from sim import Simulation


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", help="scenario names to run")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--runs", type=int, default=1, help="Monte-Carlo runs per scenario (different noise seeds)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="results")
    ap.add_argument("--feeds", default="feeds", help="folder with false-data CSV feeds")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--custom", help="JSON scenario file to run in addition")
    args = ap.parse_args()

    scenarios = dict(ALL)
    if args.custom:
        c = load_custom(args.custom)
        scenarios[c.name] = c
    if args.list:
        for n, c in scenarios.items():
            print(f"{n:32s} {c.description}")
        return 0
    names = args.only or list(scenarios)
    unknown = [n for n in names if n not in scenarios]
    if unknown:
        print("unknown scenario(s):", ", ".join(unknown), "(use --list)")
        return 2
    if not os.path.isdir(args.feeds):
        print(f"feeds folder '{args.feeds}' missing - run: python make_feeds.py")
        return 2

    os.makedirs(args.out, exist_ok=True)
    rows = []
    t0 = time.time()
    for name in names:
        ms = []
        for r in range(args.runs):
            sim = Simulation(scenarios[name], seed=args.seed + r, feed_dir=args.feeds)
            m = sim.run()
            ms.append(m)
            if r == 0:
                sim.save(os.path.join(args.out, name), m)
                if not args.no_plots:
                    plot_run(sim, os.path.join(args.out, name, "plot.png"), m)
        rows.append(summarize(name, ms))
        print(f"{name:32s} {rows[-1]['outcome']:34s} {rows[-1]['reason'][:70]}")
    write_summary(rows, args.out, args.runs)
    print(f"\nDone in {time.time() - t0:.0f}s -> {args.out}/summary.md")
    return 0


def summarize(name, ms):
    cnt = Counter(m["outcome"] for m in ms)
    outcome = ms[0]["outcome"] if len(ms) == 1 else ", ".join(f"{k} {v}/{len(ms)}" for k, v in cnt.most_common())

    def avg(key):
        v = [m[key] for m in ms if m[key] is not None]
        return round(float(np.mean(v)), 2) if v else None

    def mx(key):
        v = [m[key] for m in ms if m[key] is not None]
        return round(float(np.max(v)), 2) if v else None
    return {"scenario": name, "outcome": outcome, "reason": ms[0]["reason"],
            "max_est_error_m": mx("max_est_error_m"), "landing_error_m": mx("landing_error_m"),
            "detection_latency_s": avg("detection_latency_s"),
            "min_separation_m": min([m["min_separation_m"] for m in ms if m["min_separation_m"] is not None], default=None),
            "first_failsafe": ms[0]["first_failsafe"], "runs": len(ms)}


def write_summary(rows, out, runs):
    fields = list(rows[0].keys())
    with open(os.path.join(out, "summary.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    hdr = ["Scenario", "Outcome", "Max est. error (m)", "Landing error (m)", "Detect latency (s)", "Min sep (m)", "First failsafe"]
    body = [[r["scenario"], r["outcome"], r["max_est_error_m"], r["landing_error_m"], r["detection_latency_s"],
             r["min_separation_m"], (r["first_failsafe"] or "")[:60]] for r in rows]
    note = f"\n\n{runs} run(s) per scenario. Max/min are worst case over runs; latency is the mean.\n"
    with open(os.path.join(out, "summary.md"), "w") as fh:
        fh.write("# Scenario summary\n\n" + markdown_table(body, hdr) + note)


if __name__ == "__main__":
    sys.exit(main())
