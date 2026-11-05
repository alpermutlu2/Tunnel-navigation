"""Note10 S4 trace, the 10-state filter with and without the map-grade row (v0.5.14): θ̂ against the body pitch, σ_s."""
import sys; sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]))
import numpy as np
from tunnelsim.ekf10 import Ekf10Config, run_ekf10
from tunnelsim.eskf import EskfConfig
from tunnelsim.experiments import filter_mount, get_scenario, simulate_run
seed=int(sys.argv[1]); seeds=np.array([seed])
sc,tr,t,F,W,gnss,bt,bh=simulate_run("S4",seeds,())
F,W,C,md=filter_mount(tr,t,F,W,gnss,seeds,"estimated",None)
road=get_scenario("S4").road
for gu in (False,True):
    dbg=[]
    o=run_ekf10(t,F,W,gnss,bt,bh,C,tr.lever_arm_b,road,Ekf10Config(grade_update=gu,mount_yaw_sigma=tuple(np.deg2rad(md["yaw_sigma_deg_b"]))),EskfConfig(),debug=dbg)
    i_e=int(np.argmax(tr.in_tunnel)); te=tr.t[i_e]
    # truth body pitch
    pitch_b=np.arcsin(np.clip(tr.C_nb[:,2,0],-1,1))
    print("grade" if gu else "today")
    for dt in range(0,330,20):
        k=int(np.argmin(abs(o.t-(te+dt)))); j=int(round((o.t[k]-tr.t[0])/tr.dt))
        print(f"  t+{dt:3d} e_s {o.s[0,k]-tr.s[j]:7.1f} σ {o.sig_s[0,k]:5.1f} v̂ {o.v[0,k]:5.2f} v {tr.v[j]:5.2f}  θ̂ {np.rad2deg(o.theta[0,k]):6.3f} θ_body {np.rad2deg(pitch_b[j]):6.3f} b_a {o.b_a[0,k]:.4f}")
