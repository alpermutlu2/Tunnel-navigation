"""v0.5.15: map rows paused while the branch decision is "not yet" (K2), the drawn point near the diverge
(K1), the callout margin by IMU class (K3)."""
import numpy as np

from tunnelsim import supervisor as SV
from tunnelsim.display import shown_xy
from tunnelsim.ekf10 import Ekf10Config, run_ekf10
from tunnelsim.eskf import EskfConfig
from tunnelsim.experiments import filter_mount, get_scenario, simulate_run
from tunnelsim.mapview import MapView


def test_callout_k_by_class():
    k = SV.callout_k(np.array(["note10", "mpu9250", "other"]))
    assert k.tolist() == [1.0, 0.5, 1.0]


def test_paused_rows_keep_a_straight_on_car_from_being_pulled_back():
    seeds = np.arange(50, 52)
    sc, tr, t, F, W, gnss, bt, bh = simulate_run("S1", seeds, ("imu_lowend",), branch=False)
    F, W, C, md = filter_mount(tr, t, F, W, gnss, seeds, "estimated", None)
    road = get_scenario("S1", branch=True).road
    prior = tuple(np.deg2rad(md["yaw_sigma_deg_b"]))
    err = {}
    for on in (False, True):
        o = run_ekf10(t, F, W, gnss, bt, bh, C, tr.lever_arm_b, road, Ekf10Config(mount_yaw_sigma=prior, pause_undecided=on),
                      EskfConfig())
        idx = np.clip(np.round((o.t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
        i_x = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
        k = int(np.argmin(np.abs(o.t - tr.t[min(i_x, len(tr.t) - 1)])))
        err[on] = np.abs(o.s[:, k] - tr.s[idx[k]])
        if on:
            assert o.events.get("map_paused", 0) > 0
    assert err[True].max() < err[False].min()


def test_main_first_draws_the_main_tunnel_and_limits_the_move_across():
    road = get_scenario("S1", branch=True).road
    mv = MapView(road)
    t = np.arange(0.0, 20.0, 0.1)
    s = mv.s_div - 50.0 + 20.0 * t                                   # 20 m/s through the diverge
    state = np.where(t >= 8.0, 1, 0)[None]                           # "branch" decided at 8 s
    xy_r, _ = shown_xy(mv, t, s[None], state, "route", lat_rate=None)
    xy_m, w = shown_xy(mv, t, s[None], state, "main_first")
    main = mv.at(s, main=np.ones(len(s), bool))[0]
    k7 = int(np.argmin(np.abs(t - 7.5)))
    assert np.linalg.norm(xy_m[0, k7] - main[k7]) < 1e-6                # not yet: on the main tunnel
    assert np.linalg.norm(xy_r[0, k7] - main[k7]) > 1.0                 # the route draws the branch
    step = np.linalg.norm(np.diff(xy_m[0], axis=0), axis=1) - np.abs(np.diff(s))
    assert step.max() <= SV.HANDOVER_RATE * 0.1 + 1e-6
    assert w[0, -1] == 1.0                                            # it reaches the branch
