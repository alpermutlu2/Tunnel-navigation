"""Monte Carlo evaluation of the final estimator (v0.5): the numbers behind the write-up.

    python tools/evaluate.py                    # all scenarios, 50 seeds -> reports/<version>/evaluation_50.json
    python tools/evaluate.py --seeds 20 S1 S4   # subset
    python tools/evaluate.py --negative         # also the straight-on negative control (branch check)

Per scenario: along-track error when the car passes the diverge (straight-tunnel part) and at the
portal, consistency (e/σ)², exit callout timing (rms, late rates, claimed σ_t and its coverage),
branch confirmation, exit recovery. Prints a markdown table.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.experiments import eskf_experiment  # noqa: E402

ALL = ("S1", "S2", "S3", "S4", "S5", "S6s", "S6", "S6L")


def run(keys=ALL, seeds=50, negative=False):
    out = {"version": tunnelsim.__version__, "seeds": seeds, "runs": {}}
    for key in keys:
        r = eskf_experiment(key, seeds=seeds)
        r.pop("_tracks", None)
        out["runs"][key] = r
        if negative:
            n = eskf_experiment(key, seeds=seeds, branch=False)
            n.pop("_tracks", None)
            out["runs"][key + "_straight_on"] = n
    return out


def table(res):
    rows = ["| | " + " | ".join(k for k in res["runs"]) + " |",
            "|---|" + "---|" * len(res["runs"])]

    def row(name, f):
        cells = []
        for r in res["runs"].values():
            try:
                cells.append(f(r))
            except (KeyError, TypeError):
                cells.append("–")
        rows.append(f"| {name} | " + " | ".join(cells) + " |")

    row("error at diverge, rms [m]", lambda r: f"{r['at_diverge']['rmse']:.1f}")
    row("error at portal, rms [m]", lambda r: f"{r['real_exit']['rmse']:.1f}")
    row("error at portal, p95 [m]", lambda r: f"{r['real_exit']['p95_abs']:.1f}")
    row("mean (e/σ)² at portal", lambda r: f"{r['consistency_real_exit']['mean_norm_sq']:.2f}")
    row("callout rms [s]", lambda r: f"{r['callout']['timing_error_s']['rmse']:.2f}")
    row("callout bias [s] (+ late)", lambda r: f"{r['callout']['timing_error_s']['bias']:+.2f}")
    row("callout > 1 s late", lambda r: f"{100 * r['callout']['late_gt_1s_frac']:.0f} %")
    row("claimed σ_t [s] / coverage 2σ", lambda r: f"{r['callout']['sigma_t_mean_s']:.2f} / {100 * r['callout']['within_2sigma_t']:.0f} %")
    row("branch reported / main reported", lambda r: f"{100 * r['branch_confirm']['branch_reported_frac']:.0f} / {100 * r['branch_confirm']['main_reported_frac']:.0f} %")
    row("decision delay mean [s]", lambda r: f"{r['branch_confirm']['delay_s']['bias']:.1f}")
    row("exit step p95 [m]", lambda r: f"{r['recovery']['max_step_jump_m']['p95_abs']:.1f}")
    row("not < 10 m 60 s after exit", lambda r: f"{r['recovery']['not_converged_in_60s']}/{res['seeds']}")
    return "\n".join(rows)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("scenarios", nargs="*", default=list(ALL))
    p.add_argument("--seeds", type=int, default=50)
    p.add_argument("--negative", action="store_true")
    a = p.parse_args()
    res = run(a.scenarios, a.seeds, a.negative)
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    f = d / f"evaluation_{a.seeds}.json"
    f.write_text(json.dumps(res, indent=1, default=str) + "\n")
    print(table(res))
    print(f"\nwrote {f}")
