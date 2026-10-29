"""v0.5.12: branch window, the 10-state curve landmark and branch decision, the one-way speed constraint."""
import dataclasses

import numpy as np
import pytest

from tunnelsim.ekf10 import Ekf10Config
from tunnelsim.eskf import EskfConfig
from tunnelsim.experiments import eskf_experiment, ekf10_experiment, simulate_run


def _portal(r):
    T = r["_tracks"]
    k = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
    return T["e_s"][:, k]


def test_branch_window_recovers_s4_seed_247():
    """A dead-reckoning error larger than 100 m put the turn outside the fixed window (v0.5.11)."""
    seeds = np.array([247])
    e0 = _portal(eskf_experiment("S4", seeds, eskf_cfg=EskfConfig(map_pre_sigma_k=0.0), keep_tracks=True))
    e2 = _portal(eskf_experiment("S4", seeds, eskf_cfg=EskfConfig(map_pre_sigma_k=2.0), keep_tracks=True))
    assert abs(e0[0]) > 100.0 and abs(e2[0]) < 20.0


def test_landmark_and_guard_are_one_switch():
    """(a) without (b) is consistent but worse; it must not be configurable."""
    names = {f.name for f in dataclasses.fields(Ekf10Config)}
    assert "curve_landmark" in names and "curve_s_guard" not in names and "landmark" not in names


def test_curve_landmark_fixes_lowend_s4_seed_15():
    seeds = np.array([15])
    sim = simulate_run("S4", seeds, ("imu_lowend",))
    off = _portal(ekf10_experiment("S4", seeds, ("imu_lowend",), sim=sim, keep_tracks=True, mount="known",
                                   cfg10=Ekf10Config(curve_landmark=False, no_reverse=False, lateral_speed=False)))
    on = ekf10_experiment("S4", seeds, ("imu_lowend",), sim=sim, keep_tracks=True, mount="known",
                          cfg10=Ekf10Config(lateral_speed=False))
    assert abs(off[0]) > 100.0 and abs(_portal(on)[0]) < 30.0
    assert on["branch10"]["branch"] == 1.0


def test_own_branch_decision_keeps_landmark_off_when_going_straight():
    seeds = np.arange(3)
    r = ekf10_experiment("S4", seeds, ("imu_lowend",), branch=False)
    assert r["events"].get("landmark", 0) == 0
    assert r["branch10"]["branch"] == 0.0


@pytest.mark.parametrize("cfg_off", [Ekf10Config(no_reverse=False)])
def test_no_reverse_does_nothing_while_speed_is_positive(cfg_off):
    """Note10 S1: v̂ never goes negative, so the one-sided constraint never acts."""
    seeds = np.arange(2)
    sim = simulate_run("S1", seeds)
    a = ekf10_experiment("S1", seeds, sim=sim, keep_tracks=True)
    b = ekf10_experiment("S1", seeds, sim=sim, keep_tracks=True, cfg10=cfg_off)
    assert a["events"].get("no_reverse", 0) == 0
    np.testing.assert_array_equal(a["_tracks"]["e_s"], b["_tracks"]["e_s"])


def test_mount_estimate_matches_truth_with_an_approach_manoeuvre():
    """v0.5.13: gravity tilt + magnetometer yaw + velocity-direction yaw, S1 approach, cup holder."""
    from dataclasses import replace
    from tunnelsim.experiments import CUPHOLDER_RPY_DEG, get_scenario
    from tunnelsim.magnetometer import simulate_mag
    from tunnelsim.mount_init import estimate_mount, mount_error_deg
    veh = replace(get_scenario("S1").vehicle, mount_rpy_deg=CUPHOLDER_RPY_DEG)
    sc, tr, t, F, W, gnss, bt, bh = simulate_run("S1", np.arange(3), (), vehicle=veh)
    mt, M = simulate_mag(tr, np.arange(3))
    C, info = estimate_mount(t, F, W, mt, M, gnss)
    err = mount_error_deg(C, tr.C_bp)
    assert np.abs(err).max() < 2.5 and all(i["yaw_from"] == "mag+accel" for i in info)
