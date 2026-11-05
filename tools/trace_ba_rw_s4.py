"""v0.5.14: Note10 S4, 10-state with the hybrid, accel_bias_rw ×1 vs ×10: per seed e/σ at the diverge, and for
the worst seed b̂_a and θ̂ through the stops (is b̂_a absorbing a bias-sized change or a tilt-sized one?)."""
import sys; sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]))
import numpy as np
from tunnelsim.ekf10 import Ekf10Config, run_ekf10
from tunnelsim.eskf import EskfConfig
from tunnelsim.experiments import filter_mount, get_scenario, simulate_run
seeds = np.arange(50, 70)
sc, tr, t, F, W, gnss, bt, bh = simulate_run("S4", seeds, ())
F, W, C, md = filter_mount(tr, t, F, W, gnss, seeds, "estimated", None)
road = get_scenario("S4").road
s_div = road.exits[0].s
base = EskfConfig().accel_bias_rw
outs = {}
for m in (1, 10):
    outs[m] = run_ekf10(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, road,
                        Ekf10Config(grade_update=True, baro_grad=True, mount_yaw_sigma=tuple(np.deg2rad(md["yaw_sigma_deg_b"]))),
                        EskfConfig(accel_bias_rw=base * m))
s_true = np.interp(outs[1].t, tr.t, tr.s)
kd = int(np.argmax(s_true >= s_div))
print("seed   e×1    σ×1   z×1 |  e×10  σ×10  z×10")
z1 = []
for b, sd in enumerate(seeds):
    e1 = outs[1].s[b, kd] - s_true[kd]; e10 = outs[10].s[b, kd] - s_true[kd]
    z1.append(abs(e1 / outs[1].sig_s[b, kd]))
    print(f"{sd:4d} {e1:6.1f} {outs[1].sig_s[b, kd]:6.1f} {e1 / outs[1].sig_s[b, kd]:5.1f} | {e10:6.1f} {outs[10].sig_s[b, kd]:5.1f} {e10 / outs[10].sig_s[b, kd]:5.1f}")
w = int(np.argmax(z1))
print(f"\nworst seed {seeds[w]}: b̂_a [m/s²] and θ̂ [deg] every 20 s from the portal, ×1 | ×10; true speed")
i_e = int(np.argmax(tr.in_tunnel)); te = tr.t[i_e]
for dt in range(0, int(outs[1].t[kd] - te) + 20, 20):
    k = int(np.argmin(abs(outs[1].t - (te + dt))))
    print(f"  t+{dt:3d} v {np.interp(outs[1].t[k], tr.t, tr.v):5.2f}  b̂_a {outs[1].b_a[w, k]:.4f} θ̂ {np.rad2deg(outs[1].theta[w, k]):6.3f} e {outs[1].s[w, k] - s_true[k]:6.1f}"
          f" | b̂_a {outs[10].b_a[w, k]:.4f} θ̂ {np.rad2deg(outs[10].theta[w, k]):6.3f} e {outs[10].s[w, k] - s_true[k]:6.1f}")
