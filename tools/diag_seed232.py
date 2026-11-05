"""Diagnosis of low-end S4 seed 232 (v0.5.14; reviews v10/v11 P0): which update collapses σ_s while the error
stays hundreds of metres, and whether the true position was inside the landmark's candidate window.
Evaluation side: runs the 10-state filter with its debug hook and compares with the truth afterwards.

    python tools/diag_seed232.py [estimated|known] [seed]
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tunnelsim.ekf10 import Ekf10Config, run_ekf10  # noqa: E402
from tunnelsim.eskf import EskfConfig  # noqa: E402
from tunnelsim.experiments import filter_mount, get_scenario, simulate_run  # noqa: E402

mount = sys.argv[1] if len(sys.argv) > 1 else "estimated"
seed = int(sys.argv[2]) if len(sys.argv) > 2 else 232
seeds = np.array([seed])
ov = ("imu_lowend",)
sc, tr, t, F, W, gnss, bt, bh = simulate_run("S4", seeds, ov)
F, W, C, md = filter_mount(tr, t, F, W, gnss, seeds, mount, None)
cfg10 = Ekf10Config(mount_yaw_sigma=tuple(np.deg2rad(md["yaw_sigma_deg_b"]))) if md else Ekf10Config()
dbg = []
road = get_scenario("S4").road
o = run_ekf10(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, road, cfg10, EskfConfig(), debug=dbg)
src = (ROOT / "tunnelsim" / "ekf10.py").read_text().splitlines()
def tag(line):
    if isinstance(line, str):
        return line
    for j in range(line - 1, max(line - 22, 0), -1):
        L = src[j]
        for key, name in (("--- map heading", "heading"), ("--- curvature-gyro", "curvature"), ("lateral_speed", "lateral"),
                          ("--- ZUPT", "zupt"), ("no_reverse", "no_reverse"), ("--- barometer", "baro"), ("Hl[:, I_S]", "landmark"),
                          ("okv.any()", "gnss_vel"), ("okp.any()", "gnss_pos")):
            if key in L:
                return name
    return f"line{line}"
s_true = lambda tt: np.interp(tt, tr.t, tr.s)
i_e = int(np.argmax(tr.in_tunnel)); t_e = tr.t[i_e]
i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1])) - 1; t_x = tr.t[i_x]
lever = float(tr.lever_arm_b[0])
print(f"mount {mount}, seed {seed}: tunnel {t_e:.0f}-{t_x:.0f} s; diverge s = {road.exits[0].s:.0f} m; mount diag {md and md['err_rms_deg']}")
k = np.searchsorted(o.t, [t_e + d for d in range(0, int(t_x - t_e), 30)])
for kk in k:
    print(f"  t+{o.t[kk]-t_e:5.0f}: e_s {o.s[0,kk]-s_true(o.t[kk]):8.1f}  σ_s {o.sig_s[0,kk]:6.1f}  v̂ {o.v[0,kk]:5.2f}  v {np.interp(o.t[kk], tr.t, tr.v):5.2f}  θ̂ {np.rad2deg(o.theta[0,kk]):5.2f}°")
print("\nupdates in the tunnel that cut σ_s by more than 15 % (or move s by > 20 m):")
agg = {}
for r in dbg:
    if not (t_e <= r["t"] <= t_x) or r["line"] == "landmark_eval":
        continue
    nm = tag(r["line"])
    if not r["sel"][0]:
        continue
    d_sig = r["sig1"][0] / r["sig0"][0]; d_s = r["s1"][0] - r["s0"][0]
    a = agg.setdefault(nm, [0, 0.0, 1.0]); a[0] += 1; a[1] += d_s; a[2] *= d_sig
    if d_sig < 0.85 or abs(d_s) > 20:
        e0 = r["s0"][0] - s_true(r["t"]); e1 = r["s1"][0] - s_true(r["t"])
        print(f"  t+{r['t']-t_e:6.1f} {nm:10s} e {e0:8.1f} -> {e1:8.1f}  σ {r['sig0'][0]:6.1f} -> {r['sig1'][0]:6.1f}  "
              f"v {r['v0'][0]:5.2f}->{r['v1'][0]:5.2f}  z {r['z'][0]: .4f}  z²/S {r['z'][0]**2/r['S'][0]:6.2f}  H_s {r['Hs'][0]: .3g} H_v {r['Hv'][0]: .3g}  ok {r['ok'][0]}")
print("\nper update type in the tunnel: count, total Δs, product of σ ratios")
for nm, (n, ds, pr) in sorted(agg.items(), key=lambda z: z[1][2]):
    print(f"  {nm:10s} {n:5d}  Δs {ds:8.1f} m  σ×{pr:.3g}")
print("\nlandmark evaluations (true position against the candidate window):")
for r in dbg:
    if r["line"] == "landmark_eval" and t_e <= r["t"] <= t_x and (r["seen"][0] or r["go"][0]):
        st = s_true(r["t"]) + lever
        print(f"  t+{r['t']-t_e:6.1f} window [{r['cand_lo'][0]:7.0f}, {r['cand_hi'][0]:7.0f}]  true {st:7.0f} "
              f"({'inside' if r['cand_lo'][0] <= st <= r['cand_hi'][0] else 'OUTSIDE'})  best {r['best'][0]:7.0f}  "
              f"J {r['Jbest'][0]:6.1f} (χ²n {r['chi_n']:.1f})  ΔJ {r['dJ'][0]:6.1f}  edge {r['edge'][0]}  br10 {r['br10'][0]}  go {r['go'][0]}")
print(f"\nportal: e_s {o.s[0,-1]-s_true(o.t[-1]):.1f} (end of run); branch decision {o.branch_decision}")
