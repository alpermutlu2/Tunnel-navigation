"""10-state filter, the hybrid A/B (v0.5.14, owner 2026-11-05 after reviews v10/v11): the map grade as a weak
pitch measurement, the tunnel pressure-gradient state, and both, with a correct map grade, one 10 % too steep
and no grade. Low-end and Note10, gradient 0 / ±30 Pa/km (custom 3 % tunnel: ±25), plus the straight-on control.

    python tools/ekf10_hybrid_ab.py 50 70     -> reports/<version>/ekf10_hybrid_ab_50-69.json
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
LOW = ("imu_lowend",)
CUSTOM = "custom:L=5000,g=3,v=110"
CASES = ([("S1", LOW + PG(p), True) for p in (0, 30, -30)] + [("S3", LOW + PG(p), True) for p in (0, 30, -30)]
         + [(CUSTOM, LOW + PG(p), True) for p in (0, 25, -25)] + [("S1-5km", LOW + PG(p), True) for p in (0, 30)]
         + [(k, LOW, True) for k in ("S2", "S4", "S5", "S6")]
         + [(k, (), True) for k in ("S1", "S4", "S5")] + [("S1", PG(30), True)]
         + [("S4", LOW, False), ("S1", LOW + PG(30), False)])
CONFIGS = {"today": {}, "grad": {"baro_grad": True}, "grade": {"grade_update": True},
           "grade+grad": {"grade_update": True, "baro_grad": True},
           "grade10": {"grade_update": True, "grade_scale": 1.1},
           "grade10+grad": {"grade_update": True, "grade_scale": 1.1, "baro_grad": True}}


def tag(c):
    return f"{c[0]}/{'+'.join(c[1]) or 'note10'}/{'branch' if c[2] else 'straight'}"


def one(c):
    key, ov, br = c
    from tunnelsim.ekf10 import Ekf10Config
    from tunnelsim.experiments import ekf10_experiment, simulate_run
    seeds = np.arange(LO, HI)
    sim = simulate_run(key, seeds, ov, branch=br)
    out = {}
    for name, kw in CONFIGS.items():
        r = ekf10_experiment(key, seeds, ov, cfg10=Ekf10Config(**kw), sim=sim, branch=br, keep_tracks=br)
        m = {"branch10": r.get("branch10")}
        if br:
            m.update(diverge=r["at_diverge"]["rmse"], nees_div=r["consistency_diverge"]["mean_norm_sq"],
                     late1=r["callout_k"]["1.0"]["late_gt_1s_frac"], portal=r["real_exit"]["rmse"],
                     p95=r["real_exit"]["p95_abs"], nees=r["consistency_real_exit"]["mean_norm_sq"])
            T = r.pop("_tracks")
            kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
            m["fail"] = int(np.sum(np.abs(T["e_s"][:, kx]) > 100.0))
        else:
            m.update(horiz=r["portal_horizontal_m"]["rmse"])
        out[name] = m
    return tag(c), out


LO, HI = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (50, 70)
if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"ekf10_hybrid_ab_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if tag(c) not in res]
    with ProcessPoolExecutor(2) as ex:
        for t_, r in ex.map(one, jobs):
            res[t_] = r
            f.write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(t_, json.dumps(r), flush=True)
