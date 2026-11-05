"""v0.5.14 A/B (owner 2026-11-05): Note10 10-state overconfidence on S4 (stop-and-go). Hypothesis: the
accelerometer bias moves after the stops by more than the random-walk model allows (the GM terms of the
truth are not covered by the RW). Test: enlarge accel_bias_rw ×3 and ×10, with and without the hybrid.
Nominal cases check what the larger RW costs.

    python tools/ekf10_ba_rw_ab.py 50 70  -> reports/<version>/ekf10_ba_rw_ab_50-69.json
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
CASES = ["S4", "S1", "S3", "S5", "S2", "custom:L=5000,g=3,v=110"]
HYB = {"grade_update": True, "baro_grad": True}
MULT = (1, 3, 10)


def one(key):
    from tunnelsim.ekf10 import Ekf10Config
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import ekf10_experiment, simulate_run
    seeds = np.arange(LO, HI)
    sim = simulate_run(key, seeds, (), branch=True)
    base = EskfConfig().accel_bias_rw
    out = {}
    for hyb in (False, True):
        for m in MULT:
            r = ekf10_experiment(key, seeds, (), cfg10=Ekf10Config(**(HYB if hyb else {})),
                                 eskf_cfg=EskfConfig(accel_bias_rw=base * m), sim=sim, branch=True)
            br = r.get("branch10") or {}
            out[f"{'hybK' if hyb else 'mean'}/rw×{m}"] = {
                "diverge": r["at_diverge"]["rmse"], "nees_div": r["consistency_diverge"]["mean_norm_sq"],
                "cov2_div": r["coverage"]["diverge"]["within_2sigma"],
                "portal": r["real_exit"]["rmse"], "nees": r["consistency_real_exit"]["mean_norm_sq"],
                "late1": r["callout_k"]["1.0"]["late_gt_1s_frac"], "branch": br.get("branch")}
    return key, out


if __name__ == "__main__":
    d = ROOT / "reports" / tunnelsim.__version__.split(".dev")[0]
    f = d / f"ekf10_ba_rw_ab_{LO}-{HI - 1}.json"
    res = json.loads(f.read_text()) if f.exists() else {}
    with ProcessPoolExecutor(2) as ex:
        for k, r in ex.map(one, [c for c in CASES if c not in res]):
            res[k] = r
            f.write_text(json.dumps(res, indent=1, default=str) + "\n")
            print(k, json.dumps(r), flush=True)
