"""Trace of the 10-state filter's pressure-gradient state against the truth, low-end S1 ±30 Pa/km, seeds 50-53
(v0.5.14). Sign: g_h is the barometer offset's drift per metre, so +30 Pa/km (pressure rising along the tunnel,
the barometer reading lower) is g_h ≈ −2.5e-3."""
import sys, numpy as np
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]))
from tunnelsim.experiments import simulate_run, filter_mount, get_scenario
from tunnelsim.ekf10 import Ekf10Config, run_ekf10
from tunnelsim.eskf import EskfConfig
seeds=np.arange(50,54)
for pg in (30,-30):
    ov=("imu_lowend", f"set:baro.tunnel_grad_pa_per_km={pg}")
    sc,tr,t,F,W,gnss,bt,bh=simulate_run("S1",seeds,ov)
    F,W,C,md=filter_mount(tr,t,F,W,gnss,seeds,"estimated",None)
    o=run_ekf10(t,F,W,gnss,bt,bh,C,tr.lever_arm_b,get_scenario("S1").road,Ekf10Config(baro_grad=True,mount_yaw_sigma=np.deg2rad(md["yaw_sigma_deg"])),EskfConfig())
    kt=np.clip(np.round((o.t-tr.t[0])/tr.dt).astype(int),0,len(tr.t)-1)
    i_e=int(np.argmax(tr.in_tunnel)); te=tr.t[i_e]
    print("pg",pg,"true g_h [m/m]",round(pg/11.8e3,5))
    for dt in (10,30,60,90,120,150):
        k=int(np.argmin(abs(o.t-(te+dt))))
        print(f"  t+{dt:3d}: g_hat {np.round(o.grad[:,k]*1e3,2)} e-3  sig {np.round(o.sig_grad[:,k]*1e3,2)} e-3  e_s {np.round(o.s[:,k]-tr.s[kt[k]],0)}  e_v {np.round(o.v[:,k]-tr.v[kt[k]],2)}")
