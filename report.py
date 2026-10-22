"""report.py - plots and summary tables."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = ["tab:blue", "tab:orange", "tab:green"]


def plot_run(sim, path, metrics):
    cfg = sim.cfg
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    ax_xy, ax_alt, ax_err, ax4 = axs.ravel()
    xs, ys = [], []
    for v, col in zip(sim.veh, COLORS):
        R = v.rows
        t = np.array([r["t"] for r in R])

        def g(k):
            return np.array([r[k] for r in R], float)
        i = v.cfg.id
        ax_xy.plot(g("x"), g("y"), color=col, lw=2, label=f"drone {i} true path")
        ax_xy.plot(g("ex"), g("ey"), color=col, ls="--", lw=1.2, label=f"drone {i} believed path")
        wps = np.array([v.cfg.home] + list(v.cfg.waypoints), float)
        ax_xy.plot(wps[:, 0], wps[:, 1], "k:", lw=0.8)
        ax_xy.plot(*v.cfg.home, "ks")
        xs += list(g("x")) + list(g("ex")) + list(wps[:, 0])
        ys += list(g("y")) + list(g("ey")) + list(wps[:, 1])
        ax_alt.plot(t, g("z"), color=col, label=f"drone {i} true")
        ax_alt.plot(t, g("ez"), color=col, ls="--", label=f"drone {i} believed")
        ax_err.plot(t, g("est_err"), color=col, label=f"drone {i}")
    ax_xy.margins(0.08)
    ax_xy.set_aspect("equal", "datalim")
    ax_xy.set(title="Top-down: true vs believed position (squares = home)", xlabel="x east (m)", ylabel="y north (m)")
    ax_xy.legend(fontsize=7)
    ax_alt.set(title="Altitude", xlabel="time (s)", ylabel="z (m)")
    ax_err.set(title="Estimation error |believed - true|", xlabel="time (s)", ylabel="m")
    ax_err.legend(fontsize=7)

    # second row panel 4
    if len(sim.veh) > 1:
        a, b = sim.veh[0].rows, sim.veh[1].rows
        n = min(len(a), len(b))
        sep = [np.linalg.norm([a[k]["x"] - b[k]["x"], a[k]["y"] - b[k]["y"], a[k]["z"] - b[k]["z"]]) for k in range(n)]
        ax4.plot([a[k]["t"] for k in range(n)], sep, color="purple")
        ax4.axhline(1.0, color="red", ls=":", label="collision (1 m)")
        ax4.set(title="True separation between drones", xlabel="time (s)", ylabel="m")
        ax4.legend(fontsize=7)
    else:
        R = sim.veh[0].rows
        t = [r["t"] for r in R]
        ax4.plot(t, [r["soc_true"] for r in R], label="true")
        ax4.plot(t, [r["soc_reported"] for r in R], ls="--", label="reported")
        ax4.set(title="Battery %", xlabel="time (s)", ylabel="%")
        ax4.legend(fontsize=7)

    t_end = sim.t_end
    for ax in (ax_alt, ax_err, ax4):
        for f in sim.faults.faults:
            ax.axvspan(f.t_start, min(f.t_end, t_end), color="red", alpha=0.10)
    ymax = ax_alt.get_ylim()[1]
    for e in sim.all_events():
        if e["kind"] == "FAILSAFE":
            ax_alt.axvline(e["t"], color="red", ls="-.", lw=1)
            ax_alt.text(e["t"] + 0.3, ymax * 0.98, e["msg"][:45], rotation=90, va="top", fontsize=6, color="red")
    ax_alt.legend(fontsize=7)
    fig.suptitle(f"{cfg.name}  -  {metrics['outcome']}\n{cfg.description}", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def markdown_table(rows, headers):
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    for r in rows:
        out.append("| " + " | ".join("" if c is None else str(c) for c in r) + " |")
    return "\n".join(out)
