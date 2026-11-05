"""The hybrid A/B of v0.5.14 (owner 2026-11-05, reviews v10/v11), both filters on one matrix: the map grade
as a pitch measurement (10-state: θ; ESKF: the vertical velocity) with the map-grade scale error as a
per-tunnel state or white in R, the tunnel pressure-gradient state, and the mean-only one-way constraint.
Map grade correct or 10 % too steep; gradient 0 / ±30 Pa/km (custom 3 % 5 km tunnel: ±25); straight-on control.

    python tools/hybrid_ab.py ekf10 50 70   -> reports/<version>/hybrid_ab_ekf10_50-69.json (low-end and Note10)
    python tools/hybrid_ab.py eskf 50 70    -> reports/<version>/hybrid_ab_eskf_50-69.json  (Note10)
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tunnelsim  # noqa: E402

PG = lambda p: (f"set:baro.tunnel_grad_pa_per_km={p}",) if p else ()
CUSTOM = "custom:L=5000,g=3,v=110"


def cases(imu):
    b = ([("S1", imu + PG(p), True) for p in (0, 30, -30)] + [("S3", imu + PG(p), True) for p in (0, 30, -30)]
         + [(CUSTOM, imu + PG(p), True) for p in (0, 25, -25)] + [("S1-5km", imu + PG(p), True) for p in (0, 30)]
         + [(k, imu, True) for k in ("S2", "S4", "S5", "S6")] + [("S4", imu, False), ("S1", imu + PG(30), False)])
    return b


HYB = {"grade_update": True, "baro_grad": True}
CONFIGS = {
    "ekf10": {"v0.5.13": {"no_reverse_mode": "soft"}, "mean": {},
              "hybR": dict(HYB, grade_error_state=False), "hybK": dict(HYB),
              "hybR10": dict(HYB, grade_error_state=False, grade_scale=1.1), "hybK10": dict(HYB, grade_scale=1.1)},
    "eskf": {"v0.5.13": {"no_reverse_mode": "soft"}, "mean": {}, "grad": {"baro_grad": True},
             "hybR10": dict(HYB, grade_error_state=False, grade_scale=1.1), "hybK": dict(HYB), "hybK10": dict(HYB, grade_scale=1.1)}}
FILT = sys.argv[1] if len(sys.argv) > 1 else "ekf10"
LO, HI = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (50, 70)
CASES = cases(("imu_lowend",)) + cases(()) if FILT == "ekf10" else cases(())


def tag(c):
    return f"{c[0]}/{'+'.join(c[1]) or 'note10'}/{'branch' if c[2] else 'straight'}"


def one(c):
    key, ov, br = c
    from tunnelsim.ekf10 import Ekf10Config
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import ekf10_experiment, eskf_experiment, get_scenario, simulate_run
    seeds = np.arange(LO, HI)
    s_div = get_scenario(key).road.exits[0].s
    sim = simulate_run(key, seeds, ov, branch=br) if FILT == "ekf10" else None
    out = {}
    for name, kw in CONFIGS[FILT].items():
        if FILT == "ekf10":
            r = ekf10_experiment(key, seeds, ov, cfg10=Ekf10Config(**kw), sim=sim, branch=br, keep_tracks=br)
            br_ = r.get("branch10") or {}
            m = {"branch": br_.get("branch"), "main": br_.get("main")}
        else:
            r = eskf_experiment(key, seeds, ov, eskf_cfg=EskfConfig(**kw), branch=br, keep_tracks=True)
            bc = r["branch_confirm"]
            m = {"branch": bc["branch_reported_frac"], "main": bc["main_reported_frac"]}
        if not br:
            m["horiz"] = r["portal_horizontal_m"]["rmse"] if "portal_horizontal_m" in r else r["real_exit"]["rmse"]
            r.pop("_tracks", None); out[name] = m; continue
        T = r.pop("_tracks")
        kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
        e_x = T["e_s"][:, kx]
        if FILT == "eskf":
            kd = int(np.argmax(np.asarray(T["s_true"]) >= s_div))
            nd = round(float(np.mean((T["e_s"][:, kd] / T["sig_s"][:, kd]) ** 2)), 3)
        else:
            nd = r["consistency_diverge"]["mean_norm_sq"]
        m.update(diverge=r["at_diverge"]["rmse"], nees_div=nd,
                 late1=r["callout_k"]["1.0"]["late_gt_1s_frac"], portal=r["real_exit"]["rmse"],
                 nees=r["consistency_real_exit"]["mean_norm_sq"], fail=int(np.sum(np.abs(e_x) > 100.0)))
        out[name] = m
    return tag(c), out


if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"hybrid_ab_{FILT}_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if tag(c) not in res]
    with ProcessPoolExecutor(2) as ex:
        for t_, r in ex.map(one, jobs):
            res[t_] = r
            f.write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(t_, json.dumps(r), flush=True)
