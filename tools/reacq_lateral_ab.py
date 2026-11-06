"""v0.5.17 A/B: the re-acquisition cap on the whole position correction (v0.5.16) against the cap on its
along-track part only (v0.5.17), nominal and under a map stress test (map lateral offset 6 m, length
+1.5 %, exit multipath 45 m: three times the assumed map errors). Research ESKF, Note10.

Per case: the along-track jump after the exit (the screen's concern), along-track recovery, and the
cross-track error after the exit: time until it stays under 2 m, overshoot after that, largest
sideways step per epoch.

    python tools/reacq_lateral_ab.py [first_seed] [n]       # default 50 20 (design tier)
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
SEEDS = list(range(S0, S0 + N))
CASES = ["S1", "S4", "S5", "S1-5km"]
COND = {"nominal": dict(map_err=None, overrides=()),
        "stress": dict(map_err=(6.0, 1.015), overrides=("set:gnss.exit_multipath_m=45",))}
VAR = {"v0.5.16 cap all": False, "v0.5.17 cap along": True}


def lateral_metrics(r):
    t, el, tx = np.asarray(r["t"]), np.asarray(r["e_lat"]), r["t_exit"]
    w = (t >= tx) & (t <= tx + 60.0)
    k = np.nonzero(w)[0]
    settle, over, step = [], [], []
    for b in range(el.shape[0]):
        good = np.abs(el[b, k]) < 2.0
        i = next((j for j in range(len(k)) if good[j:].all()), None)
        settle.append(np.nan if i is None else t[k[i]] - tx)
        j0 = next((j for j in range(len(k)) if good[j]), None)
        over.append(0.0 if j0 is None else float(np.abs(el[b, k[j0]:k[-1] + 1]).max()))
        step.append(float(np.abs(np.diff(el[b, k])).max()))
    s = np.array(settle)
    prof = {}
    for dt_ in (5, 10, 15, 20, 30):                  # |cross-track| at fixed times after the exit
        i_ = int(np.argmin(np.abs(t - (tx + dt_))))
        prof[f"lat_{dt_}s_median_m"] = float(np.median(np.abs(el[:, i_])))
        prof[f"lat_{dt_}s_max_m"] = float(np.abs(el[:, i_]).max())
    w30 = (t >= tx) & (t <= tx + 30.0)
    prof["lat_rms_0_30s_m"] = float(np.sqrt(np.mean(el[:, w30] ** 2)))
    return {**prof, "lat_at_exit_max_m": float(np.abs(el[:, k[0]]).max()),
            "lat_settle_2m_median_s": float(np.nanmedian(s)) if np.isfinite(s).any() else None,
            "lat_settle_2m_max_s": float(np.nanmax(s)) if np.isfinite(s).any() else None,
            "lat_not_settled_60s": int(np.isnan(s).sum()),
            "lat_after_settle_max_m": float(np.max(over)),
            "lat_step_max_m": float(np.max(step))}


out = {"version": __version__, "seeds": [S0, S0 + N - 1], "runs": {}}
for cond, kw in COND.items():
    for case in CASES:
        for vname, along in VAR.items():
            cfg = replace(EskfConfig(), reacq_cap_along_only=along)
            r = eskf_experiment(case, SEEDS, overrides=kw["overrides"], eskf_cfg=cfg, keep_tracks=True,
                                map_err=kw["map_err"])
            tk = r["_tracks"]; t_, es_, tx_ = np.asarray(tk["t"]), np.asarray(tk["e_s"]), tk["t_exit"]
            w_ = (t_ >= tx_) & (t_ <= tx_ + 60.0)
            jmp = np.abs(np.diff(es_[:, w_], axis=1)).max(axis=1)     # largest along-track step per run
            m = {"exit_step_p95_m": float(np.percentile(jmp, 95)), "exit_step_max_m": float(jmp.max()),
                 "along_recovery_5m_median_s": r["recovery_time"].get("to_5m_median_s"),
                 "portal_rms_m": r["real_exit"]["rmse"], "nees_portal": r["coverage"]["portal"]["mean_norm_sq"],
                 **lateral_metrics(r["_tracks"])}
            out["runs"][f"{cond}/{case}/{vname}"] = m
            print(f"{cond:8s} {case:7s} {vname:18s} " + "  ".join(f"{k}={v:.1f}" if isinstance(v, float) else f"{k}={v}" for k, v in m.items()), flush=True)
dst = ROOT / "reports" / "0.5.17"
dst.mkdir(parents=True, exist_ok=True)
(dst / f"reacq_lateral_ab_{S0}-{S0 + N - 1}.json").write_text(json.dumps(out, indent=1))
