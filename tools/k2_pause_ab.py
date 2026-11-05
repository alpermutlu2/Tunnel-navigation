"""K2 A/B (owner 2026-11-05): the 10-state filter's map rows paused while its branch decision is "not
yet" and ŝ is past the diverge. off (v0.5.14) / on (shipped form) / on with the ESKF's window
(max(100 m, 2σ_s) before the diverge). Car takes the branch and goes straight on.

Straight on: decision at the portal (main / none / branch), time "not yet", along-track error at the portal,
its largest value after the diverge, σ_s at the portal. Branch: decision delay, branch share, landmark
updates, diverge and portal rms, (e/σ)² at the portal, callouts > 1 s late (k = 1).

    python tools/k2_pause_ab.py 50 70  -> reports/<version>/k2_pause_ab_50-69.json
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tunnelsim  # noqa: E402

LO, HI = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (50, 70)
CUSTOM = "custom:L=5000,g=3,v=110"
CASES = [(imu, k, br) for imu, keys in (("lowend", ("S1", "S4", "S2", "S1-5km", CUSTOM, "S6")), ("note10", ("S1", "S4")))
         for k in keys for br in (True, False)]
CONFIGS = {"off": {"pause_undecided": False}, "on": {}, "on-pre2": {"pause_pre_sigma_k": 2.0}}


def one(case):
    imu, key, br = case
    from tunnelsim.ekf10 import Ekf10Config, run_ekf10
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import CALLOUT_LEAD_M, filter_mount, get_scenario, simulate_run
    seeds = np.arange(LO, HI)
    ov = ("imu_lowend",) if imu == "lowend" else ()
    sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, seeds, ov, branch=br)
    road = get_scenario(key, branch=True).road
    F, W, C, md = filter_mount(tr, t, F, W, gnss, seeds, "estimated", None)
    prior = tuple(np.deg2rad(md["yaw_sigma_deg_b"]))
    s_div = road.exits[0].s
    out = {}
    for name, kw in CONFIGS.items():
        o = run_ekf10(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, road, Ekf10Config(mount_yaw_sigma=prior, **kw), EskfConfig())
        idx = np.clip(np.round((o.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
        st = tr.s[idx]
        e = o.s - st[None]
        i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
        t_x = tr.t[min(i_x, len(tr.t) - 1)]
        kx = int(np.argmin(np.abs(o.t - (t_x - 0.05))))
        kd = int(np.argmax(st >= s_div)); t_div = o.t[kd]
        bd, btm = o.branch_decision, o.branch_time
        dec = np.where(np.isfinite(btm) & (btm <= t_x), bd, 0)
        und = np.where(np.isfinite(btm) & (btm <= t_x), np.maximum(btm - t_div, 0.0), t_x - t_div)
        m = {"decision": {"branch": int((dec == 1).sum()), "main": int((dec == -1).sum()), "none": int((dec == 0).sum())},
             "not_yet_s_mean": round(float(und.mean()), 1), "events": {k: v for k, v in o.events.items()
                                                                        if k in ("landmark", "map_paused", "branch10", "main10")}}
        rms = lambda a: round(float(np.sqrt(np.mean(a ** 2))), 1)
        if br:
            s_call = s_div - CALLOUT_LEAD_M
            hit = (o.s + o.sig_s) >= s_call
            kh = np.where(hit.any(1), np.argmax(hit, 1), hit.shape[1] - 1)
            d = o.t[kh] - o.t[int(np.argmax(st >= s_call))]
            m.update(diverge=rms(e[:, kd]), portal=rms(e[:, kx]),
                     nees_portal=round(float(np.mean((e[:, kx] / o.sig_s[:, kx]) ** 2)), 2),
                     fail=int((np.abs(e[:, kx]) > 100).sum()), late1=int((d > 1.0).sum()),
                     delay_s=round(float(np.nanmean(np.where(dec == 1, btm - t_div, np.nan))), 1) if (dec == 1).any() else None)
        else:
            win = (o.t >= t_div) & (o.t <= t_x)
            m.update(portal_err=rms(e[:, kx]), max_err_mean=round(float(np.abs(e[:, win]).max(axis=1).mean()), 1),
                     sigma_portal=round(float(o.sig_s[:, kx].mean()), 1),
                     nees_portal=round(float(np.mean((e[:, kx] / o.sig_s[:, kx]) ** 2)), 2))
        out[name] = m
    return f"{imu}/{key}/{'branch' if br else 'straight'}", out


if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"k2_pause_ab_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if f"{c[0]}/{c[1]}/{'branch' if c[2] else 'straight'}" not in res]
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, jobs):
            res[tag] = r
            f.write_text(json.dumps(res, indent=1) + "\n")
            print(tag, json.dumps(r), flush=True)
