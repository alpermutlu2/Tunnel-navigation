"""Along-track error vs distance into the tunnel, from one long tunnel (S1-5km): one run covers the
whole 0.5-5 km range (owner, 2026-10-20). Truth projection as in the evaluation.

    python tools/plot_error_vs_distance.py [seeds]   -> reports/<version>/error_vs_distance.{png,json}
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.experiments import eskf_experiment  # noqa: E402
from tunnelsim.scenario import build_scenarios  # noqa: E402


def main(seeds=30, key="S1-5km"):
    sc = build_scenarios()[key]
    r = eskf_experiment(key, seeds=seeds, keep_tracks=True)
    T = r["_tracks"]
    d_in = T["s_true"] - sc.road.tunnel.s_entry
    m = (d_in >= 0) & (d_in <= sc.road.tunnel.length)
    bins = np.arange(0, sc.road.tunnel.length + 1, 100.0)
    rows = []
    for a in bins:
        k = np.nonzero(m & (d_in >= a))[0]
        if not len(k):
            continue
        k = k[0]
        e, s = T["e_s"][:, k], T["sig_s"][:, k]
        rows.append({"d_m": float(a), "rms_m": float(np.sqrt(np.mean(e ** 2))), "p95_m": float(np.percentile(np.abs(e), 95)),
                     "sigma_mean_m": float(s.mean()), "mean_norm_sq": float(np.mean((e / s) ** 2))})
    d = np.array([x["d_m"] for x in rows]) / 1000
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(d, [x["rms_m"] for x in rows], label="rms")
    ax.plot(d, [x["p95_m"] for x in rows], label="95th percentile |e|")
    ax.plot(d, [x["sigma_mean_m"] for x in rows], "--", label="filter σ (mean)")
    ax.axvline(sc.road.exits[0].s / 1000 - sc.road.tunnel.s_entry / 1000, color="0.6", lw=0.8)
    ax.text(sc.road.exits[0].s / 1000 - sc.road.tunnel.s_entry / 1000, ax.get_ylim()[1] * 0.92, " diverge", color="0.4", fontsize=8)
    ax.set_xlabel("distance into the tunnel [km] (80 km/h, S1 geometry)")
    ax.set_ylabel("along-track error [m]")
    ax.set_title(f"{key}: error vs distance into the tunnel, {seeds} seeds")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout()
    out = ROOT / "reports" / tunnelsim.__version__
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "error_vs_distance.png", dpi=120)
    (out / "error_vs_distance.json").write_text(json.dumps({"scenario": key, "seeds": seeds, "rows": rows}, indent=1))
    for x in rows[::5]:
        print(f"{x['d_m']:6.0f} m  rms {x['rms_m']:6.1f}  p95 {x['p95_m']:6.1f}  sigma {x['sigma_mean_m']:6.1f}  e2 {x['mean_norm_sq']:.2f}")


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 30)
