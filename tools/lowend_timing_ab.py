"""Low-end callout timing with the hybrid (owner 2026-11-05): on flat tunnels the shown σ_s grows far more
than the error and the k·σ rule calls out early. Options on the 10-state filter (the low-end product):

  shipped      hybrid, gradient prior σ_g = 3e-3 m/m (≈ 35 Pa/km)
  sg/2, sg/3   the same with σ_g halved / a third (≈ 18 / 12 Pa/km)
  grade only   the grade row without the gradient state
  v0.5.13      no hybrid (mean-only one-way form, as shipped)

and, on the same tracks, the callout margin k = 1 / 0.7 / 0.5. Nominal flat and long tunnels for the
timing, the gradient cases for the protection the hybrid was built for.

    python tools/lowend_timing_ab.py 50 70  -> reports/<version>/lowend_timing_ab_50-69.json
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
PG = lambda p: (f"set:baro.tunnel_grad_pa_per_km={p}",) if p else ()
CUSTOM = "custom:L=5000,g=3,v=110"
CASES = [("S1", 0), ("S5", 0), ("S1-5km", 0), ("S4", 0), ("S3", 0), (CUSTOM, 0),
         ("S1", 30), ("S3", 30), (CUSTOM, 25), (CUSTOM, -25), ("S1-5km", 30)]
SG = 3e-3
CONFIGS = {"shipped": ({}, {}), "sg/2": ({}, {"baro_grad_sigma": SG / 2}), "sg/3": ({}, {"baro_grad_sigma": SG / 3}),
           "grade only": ({"baro_grad": False}, {}), "v0.5.13": ({"grade_update": False, "baro_grad": False}, {})}
KS = (1.0, 0.7, 0.5)


def one(case):
    key, pg = case
    from tunnelsim.ekf10 import Ekf10Config
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import CALLOUT_LEAD_M, ekf10_experiment, get_scenario, simulate_run
    seeds = np.arange(LO, HI)
    ov = ("imu_lowend",) + PG(pg)
    sim = simulate_run(key, seeds, ov, branch=True)
    s_call = get_scenario(key).road.exits[0].s - CALLOUT_LEAD_M
    out = {}
    for name, (k10, ke) in CONFIGS.items():
        r = ekf10_experiment(key, seeds, ov, cfg10=Ekf10Config(**k10), eskf_cfg=EskfConfig(**ke), sim=sim,
                             branch=True, keep_tracks=True)
        T = r["_tracks"]
        s_true = T["s_hat"] - T["e_s"]
        t_true = T["t"][np.argmax(s_true >= s_call, axis=1)]
        m = {"diverge": r["at_diverge"]["rmse"], "nees_div": r["consistency_diverge"]["mean_norm_sq"],
             "portal": r["real_exit"]["rmse"], "branch": (r.get("branch10") or {}).get("branch")}
        for k in KS:
            hit = (T["s_hat"] + k * T["sig_s"]) >= s_call
            kh = np.where(hit.any(1), np.argmax(hit, 1), hit.shape[1] - 1)
            d = T["t"][kh] - t_true
            m[f"k{k}"] = {"mean_s": round(float(d.mean()), 2), "late_gt_1s": int((d > 1.0).sum()),
                          "early_gt_5s": int((d < -5.0).sum())}
        out[name] = m
    return f"{key}/{pg}Pa", out


if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    f = d / f"lowend_timing_ab_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if f"{c[0]}/{c[1]}Pa" not in res]
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, jobs):
            res[tag] = r
            f.write_text(json.dumps(res, indent=1) + "\n")
            print(tag, json.dumps(r), flush=True)
