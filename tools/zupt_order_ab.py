"""Stop-detector filter order A/B (v0.5.10, owner 2026-10-25).

    python tools/zupt_order_ab.py      -> reports/<version>/zupt_order_ab.json

For the offline (forward-backward) detector and the causal one-pass detector at several
band-pass orders: idle and 15 km/h detection rates (static test signal), coverage of the true
stops on S4 (50 seeds), seeds with more than 0.5 s of "stopped" while the car moves, the largest
true speed at a detection, and the ESKF on S4 (30 seeds).
"""
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import tunnelsim  # noqa: E402
from tunnelsim.eskf import EskfConfig, zupt_detector  # noqa: E402
from tunnelsim.experiments import eskf_experiment, simulate_run  # noqa: E402
from tunnelsim.profiles import NOMINAL  # noqa: E402
from tunnelsim.sensors import ImuTruthSource, Switches, simulate_imu  # noqa: E402

VARIANTS = {
    "offline (band 4, low-pass 2, forward-backward)": dict(zupt_causal=False),
    "causal band 2": dict(zupt_causal=True, zupt_band_order=2, zupt_lp_order=2),
    "causal band 3": dict(zupt_causal=True, zupt_band_order=3, zupt_lp_order=2),
    "causal band 4": dict(zupt_causal=True, zupt_band_order=4, zupt_lp_order=2),
    "causal band 8, low-pass 4": dict(zupt_causal=True, zupt_band_order=8, zupt_lp_order=4),
}


def main():
    stop = ImuTruthSource.static(30.0)
    sw = Switches.only("vib_road", "vib_engine", "a_white")
    m0, _ = simulate_imu(stop, NOMINAL, sw, seed=1)
    drive = ImuTruthSource(t=stop.t, f_p=stop.f_p, w_p=stop.w_p, v=np.full(stop.t.size, 15 / 3.6), C_pb=stop.C_pb)
    m1, _ = simulate_imu(drive, NOMINAL, sw, seed=1)
    sc, tr, t, F, W, g, bt, bh = simulate_run("S4", np.arange(50))
    stopped = tr.v < 0.05
    out = {"version": tunnelsim.__version__, "variants": {}}
    for name, kw in VARIANTS.items():
        cfg = replace(EskfConfig(), **kw)
        idle = zupt_detector(m0.t, m0.f[None], cfg, 100.0)[0, 200:-200].mean()
        d15 = zupt_detector(m1.t, m1.f[None], cfg, 100.0)[0, 200:-200].mean()
        st = zupt_detector(t, F, cfg, 100.0)
        moving = st & (tr.v[None] > 0.05)
        r = eskf_experiment("S4", seeds=30, eskf_cfg=cfg)
        row = {"idle_detected": round(float(idle), 3), "drive_15kmh_detected": round(float(d15), 3),
               "s4_stop_coverage": round(float(st[:, stopped].mean()), 3),
               "s4_seeds_false_gt_0p5s": int(((moving.sum(1) / 100.0) > 0.5).sum()),
               "s4_max_true_speed_at_detection": round(float(tr.v[moving.any(0)].max()) if moving.any() else 0.0, 2),
               "eskf_s4_30": {"diverge_rmse": r["at_diverge"]["rmse"], "portal_rmse": r["real_exit"]["rmse"],
                              "portal_p95": r["real_exit"]["p95_abs"],
                              "nees": r["consistency_real_exit"]["mean_norm_sq"],
                              "k1_mean_s": r["callout_k"]["1.0"]["mean_s"],
                              "k1_late_gt_1s": r["callout_k"]["1.0"]["late_gt_1s_frac"]}}
        out["variants"][name] = row
        print(name, row, flush=True)
    d = ROOT / "reports" / tunnelsim.__version__
    d.mkdir(parents=True, exist_ok=True)
    (d / "zupt_order_ab.json").write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
