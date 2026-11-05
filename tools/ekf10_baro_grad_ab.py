"""10-state filter: the tunnel pressure-gradient state, A/B (v0.5.14, owner 2026-11-05).

Off vs on (Ekf10Config.baro_grad) on the same simulated data: low-end + gradient (the failure found in
the live simulator), and the nominal cases where the ESKF's same state had a cost (DESIGN decision note:
S3/S5). 20 seeds to look, 300 for the number.

    python tools/ekf10_baro_grad_ab.py 50 70      -> reports/<version>/ekf10_baro_grad_ab_50-69.json
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tunnelsim  # noqa: E402

PG = lambda p: (f"set:baro.tunnel_grad_pa_per_km={p}",)
LOW, N10 = ("imu_lowend",), ()
CASES = [("S1", LOW + PG(30), True), ("S3", LOW + PG(30), True), ("S1-5km", LOW + PG(30), True),
         ("custom:L=5000,g=3,v=110", LOW + PG(25), True), ("S1", LOW + PG(10), True), ("S1", LOW + PG(-30), True),
         ("S1", N10 + PG(30), True), ("S3", N10 + PG(30), True)] + \
        [(k, LOW, True) for k in ("S1", "S2", "S3", "S4", "S5", "S6", "S1-5km")] + \
        [(k, N10, True) for k in ("S1", "S3", "S4", "S5")] + \
        [("S4", LOW, False), ("S1", LOW + PG(30), False)]


def one(c):
    key, ov, br = c
    from tunnelsim.ekf10 import Ekf10Config
    from tunnelsim.experiments import ekf10_experiment, simulate_run
    seeds = np.arange(LO, HI)
    sim = simulate_run(key, seeds, ov, branch=br)
    out = {}
    for name, bg in (("off", False), ("on", True)):
        r = ekf10_experiment(key, seeds, ov, cfg10=Ekf10Config(baro_grad=bg), sim=sim, branch=br)
        m = {"branch10": r.get("branch10")}
        if br:
            m.update(diverge=r["at_diverge"]["rmse"], portal=r["real_exit"]["rmse"], p95=r["real_exit"]["p95_abs"],
                     nees=r["consistency_real_exit"]["mean_norm_sq"], within2=r["consistency_real_exit"]["frac_within_2sigma"])
        else:
            m.update(horiz=r["portal_horizontal_m"]["rmse"])
        out[name] = m
    return f"{key}/{'+'.join(ov) or 'note10'}/{'branch' if br else 'straight'}", out


LO, HI = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (50, 70)
if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"ekf10_baro_grad_ab_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if f"{c[0]}/{'+'.join(c[1]) or 'note10'}/{'branch' if c[2] else 'straight'}" not in res]
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, jobs):
            res[tag] = r
            f.write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(tag, json.dumps(r), flush=True)
