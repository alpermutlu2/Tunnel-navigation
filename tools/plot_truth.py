"""Review plot of the truth generator for every scenario (static PNG)."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tunnelsim import build_scenarios, generate_truth  # noqa: E402

S1, S2 = "#2a78d6", "#eb6834"
INK, INK2, GRID, TUN = "#0b0b0b", "#52514e", "#e6e5e0", "#ecebe6"

plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.6,
})


def shade_t(ax, tr):
    t_in = tr.t[tr.in_tunnel]
    if len(t_in):
        ax.axvspan(t_in[0], t_in[-1], color=TUN, zorder=0, lw=0)


def main(out):
    scs = build_scenarios()
    fig, axes = plt.subplots(len(scs), 4, figsize=(15, 2.7 * len(scs)), constrained_layout=True)
    for row, (key, sc) in zip(axes, scs.items()):
        tr = generate_truth(sc)
        road = sc.road
        ax = row[0]
        ax.plot(tr.pos_n[:, 0], tr.pos_n[:, 1], color=INK2, lw=1)
        m = tr.in_tunnel
        ax.plot(tr.pos_n[m, 0], tr.pos_n[m, 1], color=S1, lw=3, label="tunnel")
        for ex in road.exits:
            p = road.position(ex.s)
            ax.plot(*p[:2], "o", ms=8, color=S2, label=ex.name)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(f"{key}: {sc.title}", loc="left", fontweight="bold")
        ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
        ax.legend(frameon=False, fontsize=8, loc="best")

        ax = row[1]
        ax.axvspan(road.tunnel.s_entry, road.tunnel.s_exit, color=TUN, zorder=0, lw=0)
        ax.plot(tr.s, tr.pos_n[:, 2], color=S1)
        ax.set_title("Altitude", loc="left"); ax.set_xlabel("s [m]"); ax.set_ylabel("[m]")

        ax = row[2]
        shade_t(ax, tr)
        ax.plot(tr.t, tr.v * 3.6, color=S1)
        ax.set_title("Speed", loc="left"); ax.set_xlabel("t [s]"); ax.set_ylabel("[km/h]")

        ax = row[3]
        shade_t(ax, tr)
        ax.plot(tr.t, np.rad2deg(tr.w_b[:, 2]), color=S1, label="yaw rate")
        ax.plot(tr.t, np.rad2deg(tr.w_b[:, 1]) * 10, color=S2, label="pitch rate ×10")
        ax.set_title("Body angular rate", loc="left"); ax.set_xlabel("t [s]"); ax.set_ylabel("[deg/s]")
        ax.legend(frameon=False, fontsize=8, loc="best")
    fig.suptitle("Truth generator review (shaded = inside tunnel)", x=0.01, ha="left",
                 color=INK, fontsize=11)
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "truth_review.png")
