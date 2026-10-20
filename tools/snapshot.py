"""Write a metrics snapshot for the current version, so every version can be compared.

Usage: python tools/snapshot.py          -> reports/<version>/metrics.json (+ test summary)
       python tools/snapshot.py --diff A B  -> print changed metrics between two versions

A released version's report is never overwritten unless --force is given; development
versions (containing "dev") are overwritten freely. Release = drop ".devN" from
tunnelsim.__version__, then snapshot.

Only numbers that the version actually produces go in here. As the project grows
(sensors, estimators, Monte Carlo) each stage adds its own section.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import tunnelsim  # noqa: E402
from tunnelsim import build_scenarios, generate_truth  # noqa: E402


def truth_metrics():
    import test_truth as T
    out = {}
    for key, sc in T.ALL.items():
        tr = generate_truth(sc)
        C_bn = np.transpose(tr.C_nb, (0, 2, 1))
        vb = np.einsum("nij,nj->ni", C_bn, tr.vel_phone_n)
        m = {
            "duration_s": round(float(tr.t[-1]), 2),
            "time_in_tunnel_s": round(float(tr.in_tunnel.sum() * tr.dt), 2),
            "strapdown_max_err_m": float(f"{np.linalg.norm(T.strapdown(tr) - tr.pos_phone_n, axis=1).max():.3e}"),
            "nhc_lat_vel_max_mps": round(float(np.abs(vb[:, 1]).max()), 4),
            "nhc_up_vel_max_mps": round(float(np.abs(vb[:, 2]).max()), 4),
            "sideslip_max_deg": round(float(np.rad2deg(np.abs(tr.beta).max())), 3),
        }
        if key in ("S2", "S3", "MIX"):
            m["centreline_discretisation_m"] = float(f"{T.centreline_error(sc.road):.3e}")
        out[key] = m
    return out


def sensor_metrics():
    """Sensor-level verification (v0.3): every metric carries target, achieved, tolerance."""
    import test_sensors_baro as Bt
    import test_sensors_gnss as Gt
    import test_sensors_imu as I
    from tunnelsim.profiles import NOMINAL, param_table
    return {
        "gnss": {
            "params_nominal": param_table(NOMINAL.gnss, "gnss."),
            "open_sky": Gt.open_sky_report(),
            "calibration_floor": Gt.calibration_floor_report(),
            "outage": Gt.outage_report(),
            "exit": Gt.exit_report(),
            "time_offset": Gt.offset_report(),
        },
        "baro": {
            "params_nominal": param_table(NOMINAL.baro, "baro."),
            "grade_floor": Bt.grade_floor_report(),
            "tunnel_gradient": Bt.tunnel_gradient_report(),
        },
        "imu": {
            "params_nominal": {**param_table(NOMINAL.imu, "imu."), **param_table(NOMINAL.vib, "vib."),
                               **param_table(NOMINAL.timing, "timing.")},
            "bi_analytic_fit": I.bi_fit_report(),
            "allan_24h": I.allan_static_report(),
            "interpolation": I.interpolation_report(),
            "timestamps": I.timestamp_report(),
            "vibration": I.vibration_report(),
        }
    }


def ins_metrics():
    """v0.4: strapdown INS verification (error-free, convergence order, irregular dt)."""
    import test_ins as N
    return {"error_free": N.error_free_report(), "convergence": N.convergence_report(),
            "variable_dt": N.variable_dt_report()}


def budget_metrics():
    """v0.4 step 3: fixed 180 s INS-only budget (200 seeds, development) + analytic check."""
    import test_budget as Bt
    from tunnelsim.budget import budget_report
    return {"analytic_check_s1": Bt.analytic_check_report(), **budget_report(200)}


def eskf_metrics():
    """v0.5: final estimator, Monte Carlo over all scenarios (20 seeds here; the write-up uses
    tools/evaluate.py with 50 seeds)."""
    sys.path.insert(0, str(ROOT / "tools"))
    import evaluate
    res = evaluate.run(seeds=20)
    keep = ("at_diverge", "real_exit", "consistency_real_exit", "callout", "branch_confirm", "recovery")
    return {k: {kk: r[kk] for kk in keep if kk in r} for k, r in res["runs"].items()}


def run_tests():
    r = subprocess.run([sys.executable, "-m", "pytest", "-q", "--runslow", str(ROOT / "tests")],
                       capture_output=True, text=True, cwd=ROOT)
    return r.stdout.strip().splitlines()[-1], r.returncode


def snapshot(force: bool = False):
    ver = tunnelsim.__version__
    target = ROOT / "reports" / ver / "metrics.json"
    if target.exists() and "dev" not in ver and not force:
        sys.exit(f"refusing to overwrite released report {target} (use --force)")
    summary, code = run_tests()
    data = {"version": ver, "tests": summary, "tests_ok": code == 0, "truth": truth_metrics(),
            "sensors": sensor_metrics(), "ins": ins_metrics(), "budget": budget_metrics(), "eskf": eskf_metrics()}
    d = ROOT / "reports" / ver
    d.mkdir(parents=True, exist_ok=True)
    (d / "metrics.json").write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {d / 'metrics.json'}  ({summary})")


def flatten(d, prefix=""):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flatten(v, f"{prefix}{k}.")
        else:
            yield f"{prefix}{k}", v


def diff(a, b):
    A = dict(flatten(json.loads((ROOT / "reports" / a / "metrics.json").read_text())))
    B = dict(flatten(json.loads((ROOT / "reports" / b / "metrics.json").read_text())))
    for k in sorted(set(A) | set(B)):
        if A.get(k) != B.get(k):
            print(f"{k:55s} {str(A.get(k)):>14s} -> {B.get(k)}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--diff", nargs=2, metavar=("OLD", "NEW"))
    p.add_argument("--force", action="store_true", help="overwrite a released report")
    args = p.parse_args()
    diff(*args.diff) if args.diff else snapshot(args.force)
