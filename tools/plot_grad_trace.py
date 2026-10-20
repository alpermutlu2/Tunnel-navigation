"""Gradient-state cost mechanism (owner's hypothesis, 2026-10-20): with the barometer pressure-
gradient state on, the barometer trend is split between pitch and gradient, the pitch hold
loosens, and gravity leaks into speed. S5 seed 0, state off vs on.

    python tools/plot_grad_trace.py [key] [seed]   -> reports/<version>/grad_trace_<key>.png
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.budget import _tangent  # noqa: E402
from tunnelsim.eskf import EskfConfig, run_eskf  # noqa: E402
from tunnelsim.experiments import simulate_run  # noqa: E402
from tunnelsim.frames import attitude_error_n  # noqa: E402


def main(key="S5", seed=0):
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, np.array([seed]))
    s_e = sc.road.tunnel.s_entry
    fig, ax = plt.subplots(5, 1, figsize=(8, 11), sharex=True)
    for name, on in (("gradient state off", False), ("gradient state on (σ_g 3e-3)", True)):
        out = run_eskf(t, F, W, gnss, bt, bh, tr.C_bp, tr.lever_arm_b, EskfConfig(baro_grad=on))
        idx = np.clip(np.round((out.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
        tn = _tangent(tr, idx)
        pR = out.p[0] - np.einsum("kij,j->ki", out.C[0] @ tr.C_bp.T, tr.lever_arm_b)
        e_s = np.einsum("ki,ki->k", pR - tr.pos_n[idx], tn)
        Ct = (tr.C_nb @ tr.C_bp)[idx]
        th = np.array([attitude_error_n(out.C[0, k], Ct[k]) for k in range(len(idx))])
        side = np.cross(np.array([0.0, 0.0, 1.0]), tn)
        tilt = np.rad2deg(np.einsum("ki,ki->k", th, side))
        spd = np.linalg.norm(out.v[0], axis=1) - np.linalg.norm(tr.vel_phone_n[idx], axis=1)
        d = (tr.s[idx] - s_e) / 1000
        m = (d > -0.3) & (d < (sc.road.tunnel.length + 300) / 1000)
        ax[0].plot(d[m], e_s[m], label=name)
        ax[1].plot(d[m], tilt[m])
        ax[2].plot(d[m], spd[m])
        ax[3].plot(d[m], out.aux["g_h"][0][m] * 1000)
        ax[4].plot(d[m], out.aux["cruise"][0][m])
        print(name, "tilt mean in tunnel %.3f deg" % tilt[m & (d > 0) & (d < sc.road.tunnel.length / 1000)].mean(),
              "| e_s at diverge %.1f m" % e_s[np.argmax(tr.s[idx] >= sc.road.exits[0].s)])
    labels = ["along-track\nerror [m]", "along-track\ntilt error [°]", "speed error\n[m/s]", "ĝ_h [m/km]", "cruise mode"]
    for a, l in zip(ax, labels):
        a.set_ylabel(l); a.grid(alpha=0.3)
        a.axvspan(0, sc.road.tunnel.length / 1000, color="0.92", zorder=0)
        a.axvline((sc.road.exits[0].s - s_e) / 1000, color="0.6", lw=0.8)
    ax[0].legend(fontsize=8)
    ax[-1].set_xlabel("distance from the portal [km] (grey: tunnel; line: diverge)")
    ax[0].set_title(f"{key}, seed {seed}: barometer gradient state off vs on")
    fig.tight_layout()
    out_dir = ROOT / "reports" / tunnelsim.__version__
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / f"grad_trace_{key}.png", dpi=110)
    print("wrote", out_dir / f"grad_trace_{key}.png")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "S5", int(sys.argv[2]) if len(sys.argv) > 2 else 0)
