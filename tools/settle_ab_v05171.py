"""v0.5.17.1 A/B: the re-acquisition settle counter counting gate-rejected fixes (v0.5.17) against
counting only accepted, uncapped ones (v0.5.17.1). Per case: how many position fixes the gate rejected
inside the phase, and whether the exit step, recovery and portal error move. Research ESKF.

    python tools/settle_ab_v05171.py [first_seed] [n]       # default 50 20 (design tier)
"""
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tunnelsim import __version__                      # noqa: E402
from tunnelsim.eskf import EskfConfig                  # noqa: E402
from tunnelsim.experiments import eskf_experiment      # noqa: E402

S0 = int(sys.argv[1]) if len(sys.argv) > 1 else 50
N = int(sys.argv[2]) if len(sys.argv) > 2 else 20
SEEDS = np.arange(S0, S0 + N)
CASES = ["S1", "S2", "S3", "S4", "S5", "S6", "S1-5km", "S4_straight_on"]
COND = {"nominal": dict(map_err=None, overrides=()),
        "stress": dict(map_err=(6.0, 1.015), overrides=("set:gnss.exit_multipath_m=45",))}
out = {"version": __version__, "seeds": [S0, S0 + N - 1], "runs": {}}
for cond, kw in COND.items():
    for case in CASES:
        key, so = case.split("_straight_on")[0], case.endswith("_straight_on")
        row = {}
        for name, acc_only in (("v0.5.17", False), ("v0.5.17.1", True)):
            r = eskf_experiment(key, SEEDS, overrides=kw["overrides"], map_err=kw["map_err"], branch=not so,
                                eskf_cfg=replace(EskfConfig(), reacq_settle_accepted_only=acc_only), keep_tracks=True)
            tk = r["_tracks"]; t_, es_, tx_ = np.asarray(tk["t"]), np.asarray(tk["e_s"]), tk["t_exit"]
            w_ = (t_ >= tx_) & (t_ <= tx_ + 60.0)
            jmp = np.abs(np.diff(es_[:, w_], axis=1)).max(axis=1)
            row[name] = {"rej_in_phase": r["events"]["gnss_pos_rej_reacq"],
                         "exit_step_p95_m": round(float(np.percentile(jmp, 95)), 3),
                         "rec_5m_median_s": r["recovery_time"].get("to_5m_median_s"),
                         "portal_rms_m": r["real_exit"]["rmse"]}
        same = row["v0.5.17"] == row["v0.5.17.1"] or all(row["v0.5.17"][k] == row["v0.5.17.1"][k] for k in ("exit_step_p95_m", "rec_5m_median_s", "portal_rms_m"))
        out["runs"][f"{cond}/{case}"] = {**row, "identical": bool(same)}
        print(cond, case, row, "identical" if same else "DIFFERENT", flush=True)
dst = ROOT / "reports" / "0.5.17.1"
dst.mkdir(parents=True, exist_ok=True)
(dst / f"settle_ab_{S0}-{S0 + N - 1}.json").write_text(json.dumps(out, indent=1))
