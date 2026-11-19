"""make_ci_chart.py - mean +/- 95 % CI of max estimation error and landing error per scenario (from results_multi/stats.csv)."""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def make_ci_chart(out="results_multi"):
    rows = list(csv.DictReader(open(os.path.join(out, "stats.csv"))))
    n_runs = rows[0]["runs"]
    names = [r["scenario"] for r in rows]
    fig, axes = plt.subplots(1, 2, figsize=(11, 0.3 * len(names) + 1.6), dpi=150, sharey=True)
    for ax, key, title, col in ((axes[0], "max_est_error_m", "Max estimation error (m)", "#d98e1f"),
                                (axes[1], "landing_error_m", "Landing error from home (m)", "#2f6db5")):
        for i, r in enumerate(rows):
            m = r[f"{key}_mean"]
            if m == "":
                ax.text(0.5, i, "no landing" if key == "landing_error_m" else "", va="center", fontsize=6, color="#888")
                continue
            m = float(m)
            ci = float(r[f"{key}_ci95"]) if r[f"{key}_ci95"] else 0.0
            ax.barh(i, m, color=col, height=0.6)
            ax.errorbar(m, i, xerr=ci, color="#222", capsize=2, linewidth=1)
            ax.text(m + ci + 0.8, i, f"{m:.1f}", va="center", fontsize=6)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("metres, mean with 95% CI bar")
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="x", alpha=0.25)
    axes[0].set_yticks(range(len(names)))
    axes[0].set_yticklabels(names, fontsize=7)
    axes[0].invert_yaxis()
    fig.suptitle(f"Error per scenario, mean over {n_runs} runs (bars = 95% CI; single-run values have no bar)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "errors_ci.png"))
    plt.close(fig)


if __name__ == "__main__":
    make_ci_chart(sys.argv[1] if len(sys.argv) > 1 else "results_multi")
