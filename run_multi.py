#!/usr/bin/env python3
"""
run_multi.py - repeat every scenario with several seeds, keep EVERY per-run record,
and compute mean +/- 95 % confidence intervals from those records.

  python run_multi.py                         # 25 scenarios x 5 seeds (1..5) -> results_multi/
  python run_multi.py --runs 7 --seed 1       # 7 seeds (1..7)
  python run_multi.py --only baseline gps_drift_consistent
  python run_multi.py --analyze-only          # rebuild stats from the saved per-run files (no simulation)

Files written to results_multi/ (nothing is hidden: the statistics are recomputed from runs.csv):
  manifest.json          command, seeds, python/package versions, SHA-256 of the code used
  seed<k>/<scenario>.json  full metrics of one run (per drone)
  runs.csv               one row per (scenario, seed)  <- raw per-run records
  stats.csv / stats.md   per scenario: outcome counts, mean, SD, 95 % CI (t distribution, n-1 df)
  outcomes.png           stacked outcome bars (make_outcome_chart.py)
  errors_ci.png          mean +/- 95 % CI of estimation and landing error (make_ci_chart.py)

95 % CI = mean +/- t(0.975, n-1) * SD / sqrt(n); n = number of runs in which the metric exists.
"""
import argparse
import csv
import hashlib
import json
import os
import platform
import sys
import time
from collections import Counter, OrderedDict

import numpy as np

METRICS = ["max_est_error_m", "landing_error_m", "detection_latency_s", "min_separation_m"]
OUTCOMES = ["PASS", "SAFE_ABORT", "DEGRADED", "FAIL", "HUNG"]
HERE = os.path.dirname(os.path.abspath(__file__))


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:16]


def run(args):
    from scenarios import ALL
    from sim import Simulation
    names = args.only or list(ALL)
    bad = [n for n in names if n not in ALL]
    if bad:
        sys.exit(f"unknown scenario(s): {bad}  (python run_all.py --list)")
    if not os.path.isdir(args.feeds):
        sys.exit(f"feeds folder '{args.feeds}' missing - run: python make_feeds.py")
    os.makedirs(args.out, exist_ok=True)
    seeds = [args.seed + i for i in range(args.runs)]
    t0 = time.time()
    rows = []
    for name in names:
        for sd in seeds:
            m = Simulation(ALL[name], seed=sd, feed_dir=args.feeds).run()
            d = os.path.join(args.out, f"seed{sd}")
            os.makedirs(d, exist_ok=True)
            json.dump(m, open(os.path.join(d, f"{name}.json"), "w"), indent=1, default=str)
        print(f"{name:32s} done ({len(seeds)} seeds)", flush=True)
    write_manifest(args, names, seeds, time.time() - t0)
    # (re)build runs.csv from ALL per-seed files so partial reruns never lose data
    collect_runs(args.out)


def write_manifest(args, names, seeds, secs):
    import matplotlib
    import scipy
    man = OrderedDict(
        command="python " + " ".join(sys.argv),
        scenarios=names, seeds=seeds, runs_per_scenario=len(seeds),
        python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
        matplotlib=matplotlib.__version__, platform=platform.platform(),
        code_sha256_16={f: sha(os.path.join(HERE, f)) for f in
                        ["sim.py", "scenarios.py", "run_multi.py"]},
        feeds_sha256_16={f: sha(os.path.join(args.feeds, f)) for f in sorted(os.listdir(args.feeds))
                         if f.endswith(".csv")},
        wall_time_s=round(secs, 1),
        run_finished=time.strftime("%Y-%m-%d %H:%M:%S"))
    json.dump(man, open(os.path.join(args.out, "manifest.json"), "w"), indent=1)


def collect_runs(out):
    recs = []
    for sd in sorted((d for d in os.listdir(out) if d.startswith("seed")), key=lambda s: int(s[4:])):
        for f in sorted(os.listdir(os.path.join(out, sd))):
            if f.endswith(".json"):
                m = json.load(open(os.path.join(out, sd, f)))
                recs.append({"scenario": m["scenario"], "seed": m["seed"], "outcome": m["outcome"],
                             **{k: m.get(k) for k in METRICS}, "sim_time_s": m["sim_time_s"],
                             "reason": m["reason"]})
    from scenarios import ALL
    order = {n: i for i, n in enumerate(ALL)}
    recs.sort(key=lambda r: (order.get(r["scenario"], 999), r["seed"]))
    with open(os.path.join(out, "runs.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(recs[0].keys()))
        w.writeheader()
        w.writerows(recs)
    return recs


def tcrit(n):
    from scipy import stats
    return float(stats.t.ppf(0.975, n - 1))


def stat(vals):
    v = [float(x) for x in vals if x not in (None, "")]
    if not v:
        return dict(n=0, mean=None, sd=None, ci95=None)
    n = len(v)
    if n == 1:
        return dict(n=1, mean=v[0], sd=None, ci95=None)
    sd = float(np.std(v, ddof=1))
    return dict(n=n, mean=float(np.mean(v)), sd=sd, ci95=tcrit(n) * sd / np.sqrt(n))


def analyze(out):
    recs = list(csv.DictReader(open(os.path.join(out, "runs.csv"))))
    by = OrderedDict()
    for r in recs:
        by.setdefault(r["scenario"], []).append(r)
    rows = []
    for name, rs in by.items():
        cnt = Counter(r["outcome"] for r in rs)
        row = {"scenario": name, "runs": len(rs),
               "outcomes": ", ".join(f"{k} {cnt[k]}/{len(rs)}" for k, _ in cnt.most_common()),
               "most_common": cnt.most_common(1)[0][0]}
        for m in METRICS:
            s = stat([r[m] for r in rs])
            row.update({f"{m}_n": s["n"], f"{m}_mean": s["mean"], f"{m}_sd": s["sd"], f"{m}_ci95": s["ci95"]})
        rows.append(row)
    with open(os.path.join(out, "stats.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    def fmt(row, m):
        mean, ci, n = row[f"{m}_mean"], row[f"{m}_ci95"], row[f"{m}_n"]
        if mean is None:
            return ""
        if ci is None:
            return f"{mean:.2f} (1 run)"
        return f"{mean:.2f} ± {ci:.2f}"
    hdr = ["Scenario", "Outcomes", "Max est. error (m)", "Landing error (m)", "Detect latency (s)", "Min sep (m)"]
    lines = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for r in rows:
        lines.append("| " + " | ".join([r["scenario"], r["outcomes"]] + [fmt(r, m) for m in METRICS]) + " |")
    n_runs = sorted({r["runs"] for r in rows})
    note = (f"\n\nRuns per scenario: {n_runs}. Values are mean ± 95 % CI (t distribution, n-1 df) over the runs in which "
            "the metric exists; a metric that exists in only one run is shown as '(1 run)'. "
            "Max est. error is the maximum over every 0.02 s step.\n")
    open(os.path.join(out, "stats.md"), "w").write("# Results over repeated runs\n\n" + "\n".join(lines) + note)
    tally = Counter(r["most_common"] for r in rows)
    print("scenarios by most-common outcome:", dict(tally))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1, help="first seed; run i uses seed+i")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--out", default="results_multi")
    ap.add_argument("--feeds", default="feeds")
    ap.add_argument("--analyze-only", action="store_true")
    a = ap.parse_args()
    if not a.analyze_only:
        run(a)
    else:
        collect_runs(a.out)
    analyze(a.out)
    try:
        from make_outcome_chart import make_chart
        make_chart(a.out)
        from make_ci_chart import make_ci_chart
        make_ci_chart(a.out)
    except Exception as ex:  # chart is optional
        print("chart skipped:", ex)
    print(f"-> {a.out}/runs.csv, stats.csv, stats.md, outcomes.png, errors_ci.png")


if __name__ == "__main__":
    main()
