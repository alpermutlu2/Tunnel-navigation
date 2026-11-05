"""Low-end 10-state filter, a car going straight on at the diverge (S1-5km, seeds 50-53): the along-track
error, σ_s, the speed error and the heading while the branch decision stays "not yet" (v0.5.14).

    python tools/trace_notyet_ekf10.py > reports/0.5.14/trace_notyet_ekf10_s1-5km.txt
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tunnelsim.ekf10 import Ekf10Config, run_ekf10  # noqa: E402
from tunnelsim.eskf import EskfConfig  # noqa: E402
from tunnelsim.experiments import filter_mount, get_scenario, simulate_run  # noqa: E402

key = sys.argv[1] if len(sys.argv) > 1 else "S1-5km"
seeds = np.arange(50, 54)
sc, tr, t, F, W, gnss, bt, bh = simulate_run(key, seeds, ("imu_lowend",), branch=False)
F, W, C, md = filter_mount(tr, t, F, W, gnss, seeds, "estimated", None)
road = get_scenario(key, branch=True).road
o = run_ekf10(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, road,
              Ekf10Config(mount_yaw_sigma=tuple(np.deg2rad(md["yaw_sigma_deg_b"]))), EskfConfig())
idx = np.clip(np.round((o.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
st = tr.s[idx]
sd = road.exits[0].s
kd = int(np.argmax(st >= sd))
i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
tx = tr.t[min(i_x, len(tr.t) - 1)]
print(f"{key}, low-end, car goes straight on; seeds {seeds.tolist()}; decision at the portal {o.branch_decision.tolist()}")
print("t from the true diverge: e_s [m] | σ_s [m] | v̂ − v [m/s] | ψ̂ [deg] (the main tunnel's heading is 0, the branch's −20)")
for dt in range(-20, 400, 10):
    k = int(np.argmin(abs(o.t - (o.t[kd] + dt))))
    if o.t[k] > tx:
        break
    print(f"  t{dt:+4d} s  e {np.round(o.s[:, k] - st[k], 0)}  σ {np.round(o.sig_s[:, k], 0)}  "
          f"v̂−v {np.round(o.v[:, k] - tr.v[idx[k]], 2)}  ψ̂ {np.round(np.rad2deg(o.psi[:, k]), 1)}")
