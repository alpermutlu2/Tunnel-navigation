"""v0.5.17.1 A/B on the re-acquisition settle counter. Variants:
  v0.5.17          rejected fixes count as settled (the phase ends after 5 of them, after 10 s)
  fix, no exit     only accepted, uncapped fixes count; nothing else ends the phase
  v0.5.17.1        only accepted, uncapped fixes count; 5 rejected fixes in a row also end the phase
                   (locked out: the normal path's gate-lock recovery takes over)
Note10 nominal and under the v0.5.17 map stress, and the low-end phone (where the ESKF can be far off
at the exit and the gate rejects every fix). Research ESKF.

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
STRESS = dict(map_err=(6.0, 1.015), overrides=("set:gnss.exit_multipath_m=45",))
CASES = {f"note10/{c}": dict(key=c, overrides=(), map_err=None) for c in ("S1", "S2", "S4", "S5", "S1-5km")} | \
        {"note10/S4_straight_on": dict(key="S4", overrides=(), map_err=None, branch=False)} | \
        {f"stress/{c}": dict(key=c, **STRESS) for c in ("S1", "S4", "S1-5km")} | \
        {f"lowend/{c}": dict(key=c, overrides=("imu_lowend",), map_err=None) for c in ("S1", "S2", "S3", "S4", "S5")} | \
        {"lowend/S1_pressure30": dict(key="S1", overrides=("imu_lowend", "baro_tunnel_30pa"), map_err=None)}
VAR = {"v0.5.17": dict(reacq_settle_accepted_only=False, reacq_rej_exit_n=0),
       "fix, no exit": dict(reacq_settle_accepted_only=True, reacq_rej_exit_n=0),
       "v0.5.17.1": dict(reacq_settle_accepted_only=True, reacq_rej_exit_n=5)}
out = {"version": __version__, "seeds": [S0, S0 + N - 1], "runs": {}}
for case, kw in CASES.items():
    row = {}
    for name, v in VAR.items():
        r = eskf_experiment(kw["key"], SEEDS, overrides=kw["overrides"], map_err=kw["map_err"],
                            branch=kw.get("branch", True), eskf_cfg=replace(EskfConfig(), **v), keep_tracks=True)
        tk = r["_tracks"]; t_, es_, tx_ = np.asarray(tk["t"]), np.asarray(tk["e_s"]), tk["t_exit"]
        w_ = (t_ >= tx_) & (t_ <= tx_ + 60.0)
        jmp = np.abs(np.diff(es_[:, w_], axis=1)).max(axis=1)
        e60 = np.abs(es_[:, int(np.argmin(np.abs(t_ - (tx_ + 60.0))))])
        rt = r["recovery_time"]
        row[name] = {"rej_in_phase": r["events"]["gnss_pos_rej_reacq"], "exit_locked": r["events"]["reacq_exit_locked"],
                     "exit_step_p95_m": round(float(np.percentile(jmp, 95)), 2),
                     "err_60s_after_exit_max_m": round(float(e60.max()), 1),
                     "rec_5m_median_s": rt.get("to_5m_median_s"), "not_5m_in_60s": rt.get("not_5m_in_60s"),
                     "worst_shown_p95_m": rt.get("worst_shown_m_p95"), "portal_rms_m": r["real_exit"]["rmse"]}
    out["runs"][case] = row
    print(case, json.dumps(row), flush=True)
dst = ROOT / "reports" / "0.5.17.1"
dst.mkdir(parents=True, exist_ok=True)
(dst / f"settle_ab_{S0}-{S0 + N - 1}.json").write_text(json.dumps(out, indent=1))
