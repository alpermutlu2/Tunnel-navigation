"""Example trace for review 7: how the sideslip-violated lateral NHC in the exit curve spreads into
the attitude and velocity states and produces a ~20 m along-track transient (gate off vs on).

    python tools/plot_nhc_trace.py [seed]   -> reports/<version>/nhc_trace_S1.png
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


def trace(seed=0, key="S1"):
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, np.array([seed]))
    ex = sc.road.exits[0]
    res = {}
    for name, gate in (("lateral NHC always on", 0.0), ("lateral NHC gated in turns (0.3 m/s²)", 0.3)):
        out = run_eskf(t, F, W, gnss, bt, bh, tr.C_bp, tr.lever_arm_b, EskfConfig(nhc_turn_gate=gate))
        idx = np.clip(np.round((out.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
        tn = _tangent(tr, idx)
        pR = out.p[0] - np.einsum("kij,j->ki", out.C[0] @ tr.C_bp.T, tr.lever_arm_b)
        e_s = np.einsum("ki,ki->k", pR - tr.pos_n[idx], tn)
        C_true = (tr.C_nb @ tr.C_bp)[idx]
        th = np.array([attitude_error_n(out.C[0, k], C_true[k]) for k in range(len(idx))])
        # along-track tilt error: rotation about the horizontal axis normal to the track
        side = np.cross(np.array([0.0, 0.0, 1.0]), tn)
        tilt = np.einsum("ki,ki->k", th, side)
        spd = np.linalg.norm(out.v[0], axis=1) - np.linalg.norm(tr.vel_phone_n[idx], axis=1)
        res[name] = dict(s=tr.s[idx] - ex.s, e_s=e_s, yaw=np.rad2deg(th[:, 2]), tilt=np.rad2deg(tilt),
                         spd=spd, acc=9.81 * tilt)
    return res, tr, ex


def main(seed=0):
    res, tr, ex = trace(seed)
    fig, ax = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
    for name, r in res.items():
        m = (r["s"] > -300) & (r["s"] < 700)
        ax[0].plot(r["s"][m], r["e_s"][m], label=name)
        ax[1].plot(r["s"][m], r["yaw"][m])
        ax[2].plot(r["s"][m], r["tilt"][m])
        ax[3].plot(r["s"][m], r["spd"][m])
    for a in ax:
        a.axvspan(0, ex.curve_len, color="0.85", zorder=0)
        a.grid(alpha=0.3)
    ax[0].set_ylabel("along-track\nerror [m]")
    ax[1].set_ylabel("yaw error [°]")
    ax[2].set_ylabel("along-track\ntilt error [°]")
    ax[3].set_ylabel("speed error [m/s]")
    ax[3].set_xlabel("distance from the diverge [m] (grey: exit curve, 20° over 150 m)")
    ax[0].legend(fontsize=8)
    ax[0].set_title(f"S1, seed {seed}: sideslip in the exit curve vs the lateral NHC")
    fig.tight_layout()
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    fig.savefig(d / "nhc_trace_S1.png", dpi=120)
    print(f"wrote {d / 'nhc_trace_S1.png'}")


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
