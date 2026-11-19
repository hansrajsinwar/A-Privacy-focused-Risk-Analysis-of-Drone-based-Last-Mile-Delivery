"""make_outcome_chart.py - stacked outcome bar per scenario from results_multi/runs.csv (counts inside bars)."""
import csv
import os
import sys
from collections import Counter, OrderedDict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUTCOMES = ["PASS", "SAFE_ABORT", "DEGRADED", "FAIL", "HUNG"]
COL = {"PASS": "#2e7d4f", "SAFE_ABORT": "#2f6db5", "DEGRADED": "#d98e1f", "FAIL": "#c23b3b", "HUNG": "#7a7a7a"}


def make_chart(out="results_multi"):
    rs = list(csv.DictReader(open(os.path.join(out, "runs.csv"))))
    by = OrderedDict()
    for r in rs:
        by.setdefault(r["scenario"], []).append(r["outcome"])
    names = list(by)
    n_runs = max(len(v) for v in by.values())
    fig, ax = plt.subplots(figsize=(9, 0.3 * len(names) + 1.6), dpi=150)
    for i, n in enumerate(names):
        c = Counter(by[n])
        left = 0
        for o in OUTCOMES:
            if c[o]:
                w = 100 * c[o] / len(by[n])
                ax.barh(i, w, left=left, color=COL[o], edgecolor="white", linewidth=1.2, height=0.8)
                ax.text(left + w / 2, i, str(c[o]), ha="center", va="center", color="white", fontsize=7, fontweight="bold")
                left += w
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names, fontsize=7)
    ax.invert_yaxis()
    ax.set_xlim(0, 100)
    ax.set_xlabel(f"% of runs (number in each bar = runs out of {n_runs})")
    ax.set_title(f"Outcome of each scenario over {n_runs} runs", fontsize=10)
    ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=COL[o]) for o in OUTCOMES], labels=OUTCOMES,
              ncol=5, loc="upper center", bbox_to_anchor=(0.5, -0.06), frameon=False, fontsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "outcomes.png"))
    plt.close(fig)


if __name__ == "__main__":
    make_chart(sys.argv[1] if len(sys.argv) > 1 else "results_multi")
