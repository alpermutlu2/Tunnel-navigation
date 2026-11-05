"""What the screen shows while the branch decision is "not yet" (owner 2026-11-05, review P1). Three
display policies on the same filter output, scored as the horizontal distance between the shown point
and the car, from 50 m before the diverge to the portal, every 0.5 s; and the shown point's distance from
the road the car is on (off-road: the wrong road shown), overall and while "not yet":

  route      (today) the route polyline (it takes the branch) until "main" is decided
  main       the main tunnel until "branch" is decided
  hold       the gore point (the diverge) while "not yet" and ŝ is past it; then as decided

The products only: the ESKF on a Note10, the 10-state filter on a low-end phone (supervisor class rule).
Each case runs with the car taking the branch and going straight on.

    python tools/notyet_display.py 50 100  -> reports/<version>/notyet_display_50-99.json
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tunnelsim  # noqa: E402

LO, HI = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (50, 100)
CASES = [(f, k, br) for f, keys in (("eskf", ("S1", "S4", "S2", "S1-5km")), ("ekf10", ("S1", "S4", "S2", "S1-5km")))
         for k in keys for br in (True, False)]
POLICIES = ("route", "main", "hold")


def one(case):
    filt, key, br = case
    from tunnelsim.ekf10 import Ekf10Config, run_ekf10
    from tunnelsim.eskf import EskfConfig, run_eskf
    from tunnelsim.experiments import _eskf_along, filter_mount, get_scenario, simulate_run
    from tunnelsim.mapview import MapView
    seeds = np.arange(LO, HI)
    ov = () if filt == "eskf" else ("imu_lowend",)
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, seeds, ov, branch=br)
    road = get_scenario(key, branch=True).road
    mv = MapView(road)
    F, W, C, md = filter_mount(tr, t, F, W, gnss, seeds, "estimated", None)
    prior = tuple(np.deg2rad(md["yaw_sigma_deg_b"]))
    if filt == "eskf":
        out = run_eskf(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, EskfConfig(mount_yaw_sigma=prior), road_map=mv)
        tt = out.t
        s_hat, _ = _eskf_along(mv, out.t, out.p, out.C, out.P_pos, C, tr.lever_arm_b)
        state = out.aux["branch_state"]
    else:
        o = run_ekf10(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, road, Ekf10Config(mount_yaw_sigma=prior), EskfConfig())
        tt, s_hat = o.t, o.s
        bt_ = np.where(np.isfinite(o.branch_time), o.branch_time, np.inf)
        state = np.where(tt[None] >= bt_[:, None], o.branch_decision[:, None], 0)
    idx = np.clip(np.round((tt - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    truth = tr.pos_n[idx, :2]
    s_true = tr.s[idx]
    s_div = mv.s_div
    i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_x = tr.t[min(i_x, len(tr.t) - 1)]
    win = (s_true >= s_div - 50.0) & (tt <= t_x)
    gore = mv.at(np.array([s_div]))[0][0]
    B = len(seeds)
    res = {"car": "branch" if br else "main", "seeds": [LO, HI - 1]}
    t_div = tt[int(np.argmax(s_true >= s_div))]
    und = (state == 0) & (tt[None] >= t_div) & win[None]
    dt_ = float(np.median(np.diff(tt)))
    res["undecided_after_diverge_s"] = {"mean": round(float(und.sum(1).mean() * dt_), 1),
                                        "p95": round(float(np.percentile(und.sum(1) * dt_, 95)), 1),
                                        "at_portal": int((state[:, int(np.argmin(np.abs(tt - t_x)))] == 0).sum())}
    final = state[:, int(np.argmin(np.abs(tt - t_x)))]
    res["decision_at_portal"] = {"branch": int((final == 1).sum()), "main": int((final == -1).sum()), "none": int((final == 0).sum())}
    ks = np.nonzero(win)[0]
    ks = ks[np.searchsorted(tt[ks], np.arange(tt[ks[0]], tt[ks[-1]], 0.5))]          # every 0.5 s
    j0 = int(np.argmax(tr.s >= s_div - 100.0))
    road_xy = tr.pos_n[j0::max(1, int(round(0.05 / tr.dt))), :2]                    # the road the car drove
    for pol in POLICIES:
        err = np.zeros((B, len(ks))); off = np.zeros((B, len(ks))); undk = np.zeros((B, len(ks)), bool)
        for j, k in enumerate(ks):
            s_k = s_hat[:, k]; st = state[:, k]
            main = (st == -1) if pol in ("route", "hold") else (st <= 0)
            c = mv.at(s_k, main=main)[0]
            if pol == "hold":
                h = (st == 0) & (s_k > s_div)
                c[h] = gore
            err[:, j] = np.linalg.norm(c - truth[k][None], axis=1)
            off[:, j] = np.sqrt(((c[:, None] - road_xy[None]) ** 2).sum(-1).min(axis=1))
            undk[:, j] = (st == 0) & (tt[k] >= t_div)
        mx = err.max(axis=1); mo = off.max(axis=1)
        mo_u = np.where(undk.any(1), np.where(undk, off, 0).max(axis=1), 0.0)
        res[pol] = {"max_err_mean_m": round(float(mx.mean()), 1), "max_err_p95_m": round(float(np.percentile(mx, 95)), 1),
                    "runs_max_err_gt_50m": int((mx > 50).sum()),
                    "max_offroad_mean_m": round(float(mo.mean()), 1), "max_offroad_p95_m": round(float(np.percentile(mo, 95)), 1),
                    "runs_offroad_gt_10m": int((mo > 10).sum()),
                    "offroad_while_undecided_max_mean_m": round(float(mo_u.mean()), 1),
                    "time_offroad_gt_10m_mean_s": round(float((off > 10).sum(1).mean() * 0.5), 1)}
    return f"{filt}/{key}/{'branch' if br else 'straight'}", res


if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    f = d / f"notyet_display_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if f"{c[0]}/{c[1]}/{'branch' if c[2] else 'straight'}" not in res]
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, jobs):
            res[tag] = r
            f.write_text(json.dumps(res, indent=1) + "\n")
            print(tag, json.dumps(r), flush=True)
