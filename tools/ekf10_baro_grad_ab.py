"""10-state filter: the tunnel pressure-gradient state, A/B (v0.5.14, owner 2026-11-05).

Off vs on (Ekf10Config.baro_grad) on the same simulated data: low-end + gradient (the failure found in
the live simulator), and the nominal cases where the ESKF's same state had a cost (DESIGN decision note:
S3/S5). 20 seeds to look, 300 for the number.

    python tools/ekf10_baro_grad_ab.py 50 70      -> reports/<version>/ekf10_baro_grad_ab_50-69.json
    python tools/ekf10_baro_grad_ab.py 50 70 sigma -> ..._sigma_50-69.json: prior σ 35 / 12 / 6 Pa/km
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
    from tunnelsim.eskf import EskfConfig
    out = {}
    variants = (("off", False, None), ("on", True, None)) if not SIGMA else \
        tuple((f"{pa}Pa", True, pa * PA_PER_M_GRAD) for pa in (35, 12, 6)) + (("off", False, None),)
    for name, bg, sg in variants:
        ecfg = EskfConfig() if sg is None else EskfConfig(baro_grad_sigma=sg)
        r = ekf10_experiment(key, seeds, ov, cfg10=Ekf10Config(baro_grad=bg), eskf_cfg=ecfg, sim=sim, branch=br)
        m = {"branch10": r.get("branch10")}
        if br:
            m.update(diverge=r["at_diverge"]["rmse"], nees_div=r["consistency_diverge"]["mean_norm_sq"],
                     late1=r["callout_k"]["1.0"]["late_gt_1s_frac"],
                     portal=r["real_exit"]["rmse"], p95=r["real_exit"]["p95_abs"],
                     nees=r["consistency_real_exit"]["mean_norm_sq"], within2=r["consistency_real_exit"]["frac_within_2sigma"])
        else:
            m.update(horiz=r["portal_horizontal_m"]["rmse"])
        out[name] = m
    return f"{key}/{'+'.join(ov) or 'note10'}/{'branch' if br else 'straight'}", out


LO, HI = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (50, 70)
SIGMA = len(sys.argv) > 3 and sys.argv[3] == "sigma"
PA_PER_M_GRAD = 1.0 / 11.8e3     # 1 Pa/km as a height gradient: ρg ≈ 11.8 Pa/m at sea level [ASSUMED ρ = 1.2 kg/m³]
if SIGMA:
    CASES = [c for c in CASES if c[2] and (c[0], c[1]) in {
        ("S1", ("imu_lowend",) + PG(30)), ("S3", ("imu_lowend",) + PG(30)), ("S1-5km", ("imu_lowend",) + PG(30)),
        ("custom:L=5000,g=3,v=110", ("imu_lowend",) + PG(25)), ("S1", ("imu_lowend",) + PG(10)),
        ("S1", ("imu_lowend",)), ("S2", ("imu_lowend",)), ("S3", ("imu_lowend",)), ("S5", ("imu_lowend",)),
        ("S1-5km", ("imu_lowend",)), ("S1", ()), ("S5", ()), ("S4", ())}]
if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"ekf10_baro_grad_ab{'_sigma' if SIGMA else ''}_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    jobs = [c for c in CASES if f"{c[0]}/{'+'.join(c[1]) or 'note10'}/{'branch' if c[2] else 'straight'}" not in res]
    with ProcessPoolExecutor(2) as ex:
        for tag, r in ex.map(one, jobs):
            res[tag] = r
            f.write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(tag, json.dumps(r), flush=True)
