"""Pitch prior from the road grade and the trusted-stop ZUPT gate: A/B matrix (v0.5.11, owner 2026-10-25).

    python tools/pitch_prior_ab.py           # 30 seeds -> reports/<version>/pitch_prior_ab_30.json

Variants (EskfConfig fields; all paired on the same seeds):
  base              current filter
  trust2            ZUPT applied whatever the filter speed once the detector has held "stopped" ≥ 2 s
  nogate            ZUPT speed gate removed (reference only)
  off{σ}+trust2     prior with the offset state δγ, σ_θ = σ degrees, 10 Hz, v̂ < 10 m/s
  abs1+trust2       prior without the offset state (δγ fixed at 0), σ_θ = 1°
Risks: a graded stop-and-go (custom tunnel, 3 % grade, 30 km/h, 60 s stop) with and without the
grade in the map; the filter's mount pitched 1° and 3° wrong on S4.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.eskf import EskfConfig  # noqa: E402
from tunnelsim.experiments import eskf_experiment  # noqa: E402

D = np.deg2rad
V = {
    "base": {},
    "trust2": dict(zupt_trust_s=2.0),
    "nogate": dict(zupt_speed_max=50.0),
    "off0.5+trust2": dict(pitch_prior_sigma=D(0.5), zupt_trust_s=2.0),
    "off1+trust2": dict(pitch_prior_sigma=D(1.0), zupt_trust_s=2.0),
    "abs1+trust2": dict(pitch_prior_sigma=D(1.0), pitch_prior_offset_sigma=1e-9, pitch_prior_offset_rw=0.0,
                        zupt_trust_s=2.0),
}
GRADED = "custom:L=3000,g=3,v=30,stop=60"


def pick(r):
    out = {"diverge": r.get("at_diverge", {}).get("rmse"), "portal": r["real_exit"]["rmse"],
           "portal_p95": r["real_exit"]["p95_abs"], "nees": r["consistency_real_exit"]["mean_norm_sq"],
           "late_gt_1s": r.get("callout_k", {}).get("1.0", {}).get("late_gt_1s_frac")}
    if "branch_confirm" in r:
        out["branch_ok"] = r["branch_confirm"].get("confirmed_frac")
    return out


def run(seeds=30):
    sid = np.arange(seeds)
    res = {"version": tunnelsim.__version__, "seeds": seeds, "runs": {}}

    done = {}
    part = ROOT / "reports" / tunnelsim.__version__ / f"pitch_prior_ab_{seeds}.partial.json"
    if part.exists():
        done = json.loads(part.read_text())
        res["runs"].update(done)

    def go(tag, key, ov, kw, **extra):
        if tag in done:
            return
        r = eskf_experiment(key, sid, ov, eskf_cfg=EskfConfig(**kw), **extra)
        res["runs"][tag] = pick(r)
        print(tag, res["runs"][tag], flush=True)

    for key in ("S4", "S1", "S2", "S3", "S5", "S6", "S1-5km"):
        for name in V:
            if key != "S4" and name in ("nogate", "abs1+trust2", "off0.5+trust2"):
                continue
            go(f"lowend/{key}/{name}", key, ("imu_lowend",), V[name])
    for name in ("base", "trust2", "nogate", "off1+trust2"):
        go(f"note10/S4/{name}", "S4", (), V[name])
    for key in ("S1", "S2", "S3", "S5", "S6", "S1-5km"):
        go(f"note10/{key}/off1+trust2", key, (), V["off1+trust2"])
    for name in ("base", "off1+trust2", "abs1+trust2"):
        for mg in (True, False):
            kw = dict(V[name], pitch_prior_map_grade=mg)
            go(f"lowend/graded/{name}/map_grade={mg}", GRADED, ("imu_lowend",), kw)
    for me in (1.0, 3.0):
        for name in ("base", "off1+trust2", "abs1+trust2"):
            go(f"lowend/S4/mount{me:g}/{name}", "S4", ("imu_lowend",), V[name], mount_err_deg=(0.0, me, 0.0))
    return res


if __name__ == "__main__":
    s = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    res = run(s)
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    (d / f"pitch_prior_ab_{s}.json").write_text(json.dumps(res, indent=1) + "\n")
