"""Visual simulator: run one scenario (truth -> phone sensors -> estimators) and write a
self-contained HTML player.

    python tools/simulate.py S2                       # scenario S2, seed 0
    python tools/simulate.py S5 --seed 7 --override gnss_lagging --override vib_noisy
    python tools/simulate.py S4 --zupt oracle         # stops given by the truth (upper bound)
    python tools/simulate.py --list                   # scenarios and named overrides

Output: sim_<scenario>_seed<k>.html (open in a browser; no server, no internet needed).
The page shows the road from above with the tunnel and the exit ramp, the true car, the ESKF
estimate (with its along-track 2-sigma bar), the constant-velocity (CV) reference, GNSS fixes,
the exit callout, and time plots of along-track error and speed.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tunnelsim.budget import _tangent  # noqa: E402
from tunnelsim.eskf import EskfConfig, run_eskf  # noqa: E402
from tunnelsim.experiments import CALLOUT_LEAD_M, simulate_run  # noqa: E402
from tunnelsim.profiles import OVERRIDES  # noqa: E402
from tunnelsim.reference import gnss_entry_state  # noqa: E402
from tunnelsim.scenario import build_scenarios  # noqa: E402


def _main_line(road, ex):
    if not ex.on_route:
        return None
    L = road.tunnel.s_exit - ex.s
    p0 = road.position(np.array([ex.s]))[0, :2]
    psi = float(road.heading(ex.s))
    d = np.array([np.cos(psi), np.sin(psi)])   # heading measured from east, counter-clockwise
    pts = p0[None] + np.arange(0.0, L + 1.0, 50.0)[:, None] * d[None]
    return {"x": pts[:, 0].round(1).tolist(), "y": pts[:, 1].round(1).tolist()}


def run(key, seed, overrides, zupt):
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, [seed], tuple(overrides))
    zmask = (tr.v < 0.05) if zupt == "oracle" else None
    out = run_eskf(t, F, W, gnss, bt, bh, tr.C_bp, tr.lever_arm_b, EskfConfig(), zupt_mask=zmask)
    idx = np.clip(np.round((out.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    tn = _tangent(tr, idx)
    pR = out.p[0] - np.einsum("kij,j->ki", out.C[0] @ tr.C_bp.T, tr.lever_arm_b)
    e_s = np.einsum("ki,ki->k", pR - tr.pos_n[idx], tn)
    sig_s = np.sqrt(np.einsum("ki,kij,kj->k", tn, out.P_pos[0], tn))
    i_e = int(np.argmax(tr.in_tunnel))
    t_e = tr.t[i_e]
    # CV reference: GNSS entry state, then constant speed along the road
    t_fix, p0, v0 = gnss_entry_state(gnss["t"], gnss["pos"][0], gnss["vel"][0], t_e - 5.0)
    k_fix = int(round((t_fix - tr.t[0]) / tr.dt))
    s_cv0 = tr.s[k_fix] + (p0 - tr.pos_phone_n[k_fix]) @ _tangent(tr, np.array([k_fix]))[0]
    vh = float(np.linalg.norm(v0[:2]))
    # GNSS stays primary for CV outside the tunnel: CV is shown only from the entry fix to the
    # first position fix after the exit
    g_ok = np.isfinite(gnss["pos"][0][:, 0])
    t_back = float(gnss["t"][(gnss["t"] > t_e) & g_ok][0]) if np.any((gnss["t"] > t_e) & g_ok) else tr.t[-1]
    # decimate to 2 Hz for the page
    step = max(1, int(round(0.5 / np.median(np.diff(out.t)))))
    sel = np.arange(0, len(out.t), step)
    road = sc.road
    s_line = np.arange(0.0, road.length, 10.0)
    line = road.position(s_line)
    tun = (s_line >= road.tunnel.s_entry) & (s_line <= road.tunnel.s_exit)
    s_hat = tr.s[idx] + e_s
    data = {
        "title": f"{sc.key}: {sc.title}",
        "description": sc.description,
        "seed": seed, "overrides": overrides, "zupt": zupt,
        "road": {"x": line[:, 0].round(1).tolist(), "y": line[:, 1].round(1).tolist(), "tunnel": tun.tolist()},
        "exits": [{"name": ex.name, "s": ex.s, "xy": road.position(np.array([ex.s]))[0, :2].round(1).tolist(),
                   "call_s": ex.s - CALLOUT_LEAD_M,
                   # main tunnel continuation after the diverge (map only; the route takes the branch)
                   "main": _main_line(road, ex)} for ex in road.exits],
        "t_entry": round(float(t_e), 2),
        "t_exit": round(float(tr.t[len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1])) - 1]), 2),
        "t": out.t[sel].round(2).tolist(),
        "true_xy": tr.pos_n[idx[sel], :2].round(2).tolist(),
        "est_xy": pR[sel, :2].round(2).tolist(),
        "heading": np.arctan2(tn[sel, 1], tn[sel, 0]).round(4).tolist(),
        "s_true": tr.s[idx[sel]].round(2).tolist(),
        "s_hat": s_hat[sel].round(2).tolist(),
        "e_s": e_s[sel].round(2).tolist(),
        "sig_s": sig_s[sel].round(2).tolist(),
        "v_true": tr.v[idx[sel]].round(3).tolist(),
        "v_est": np.linalg.norm(out.v[0, sel], axis=1).round(3).tolist(),
        "zupt_on": out.zupt[0, sel].tolist(),
        "cv": {"t0": t_fix, "t1": t_back, "s0": s_cv0, "v": vh},
        "gnss": [[round(float(tt), 1), round(float(x), 1), round(float(y), 1)]
                 for tt, (x, y) in zip(gnss["t"], gnss["pos"][0][:, :2]) if np.isfinite(x)],
        "events": out.events,
    }
    return data


HTML = Path(__file__).with_name("simulate_template.html")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?", default="S2")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--override", action="append", default=[], help="named override (repeatable)")
    ap.add_argument("--zupt", choices=["detector", "oracle"], default="detector")
    ap.add_argument("--out", default=None)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list:
        for k, sc in build_scenarios().items():
            print(f"{k}: {sc.title} - {sc.description}")
        print("overrides:", ", ".join(sorted(OVERRIDES)))
        return
    data = run(a.scenario, a.seed, a.override, a.zupt)
    out = Path(a.out or f"sim_{a.scenario}_seed{a.seed}.html")
    out.write_text(HTML.read_text().replace("/*DATA*/null", json.dumps(data)))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
