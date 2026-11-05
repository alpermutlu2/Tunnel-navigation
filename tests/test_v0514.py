"""v0.5.14: per-member mount-yaw prior, prompts that never fire."""
import numpy as np

from tunnelsim.ekf10 import Ekf10Config
from tunnelsim.experiments import ekf10_experiment, simulate_run
from tunnelsim.navmetrics import prompts, summarise_prompts


def test_mount_yaw_prior_is_per_member():
    """A member's prior must not change another member's estimate (review: batch majority)."""
    seeds = np.arange(2)
    sim = simulate_run("S2", seeds, ("imu_lowend",))
    one = np.deg2rad(1.0)
    a = ekf10_experiment("S2", seeds, ("imu_lowend",), sim=sim, mount="known", keep_tracks=True,
                         cfg10=Ekf10Config(mount_yaw_sigma=(one, one)))
    b = ekf10_experiment("S2", seeds, ("imu_lowend",), sim=sim, mount="known", keep_tracks=True,
                         cfg10=Ekf10Config(mount_yaw_sigma=(one, np.deg2rad(6.0))))
    np.testing.assert_array_equal(a["_tracks"]["e_s"][0], b["_tracks"]["e_s"][0])
    assert not np.array_equal(a["_tracks"]["e_s"][1], b["_tracks"]["e_s"][1])


def test_prompt_that_never_fires_is_not_scored():
    t = np.arange(0.0, 100.0, 1.0)
    s_true = 20.0 * t                                   # reaches the gore (1500 m) at t = 75 s
    s_hat = np.stack([s_true, np.full_like(t, 0.0)])    # member 1 never moves: no prompt fires
    sig = np.ones_like(s_hat); v = np.full_like(s_hat, 20.0)
    p = prompts(t, s_hat, sig, s_true, v, 1500.0, k=1.0)
    assert p["now"]["fired"].tolist() == [True, False] and np.isnan(p["now"]["err_m"][1])
    assert p["now"]["missed"].tolist() == [False, True]
    sm = summarise_prompts(p)
    assert sm["1000 m"]["not_fired"] == 0.5 and abs(sm["1000 m"]["err_mean_m"]) < 25.0



def test_hybrid_default_by_class():
    """The hybrid is on for the products (ESKF Note10, 10-state low-end) and only where left at None."""
    from tunnelsim.eskf import EskfConfig, resolve_hybrid
    e = resolve_hybrid(EskfConfig(), "eskf", "note10")
    assert e.grade_update and e.baro_grad
    assert not resolve_hybrid(EskfConfig(), "eskf", "mpu9250").grade_update
    k = resolve_hybrid(Ekf10Config(), "ekf10", "mpu9250")
    assert k.grade_update and k.baro_grad
    assert not resolve_hybrid(Ekf10Config(), "ekf10", "note10").baro_grad
    assert resolve_hybrid(Ekf10Config(grade_update=False), "ekf10", "mpu9250").grade_update is False


def test_map_without_grade_turns_the_hybrid_off():
    """Owner 2026-11-05: a missing grade is not a zero grade; without it the filter is the v0.5.13 one."""
    from tunnelsim.eskf import EskfConfig
    from tunnelsim.experiments import eskf_experiment
    seeds = np.arange(2)
    off = dict(grade_update=False, baro_grad=False)
    a = eskf_experiment("S3", seeds, eskf_cfg=EskfConfig(**off, no_reverse_mode="soft"), keep_tracks=True)
    b = eskf_experiment("S3", seeds, keep_tracks=True, map_grade=False)
    np.testing.assert_array_equal(a["_tracks"]["e_s"], b["_tracks"]["e_s"])
    sim = simulate_run("S3", seeds, ("imu_lowend",))
    c = ekf10_experiment("S3", seeds, ("imu_lowend",), sim=sim, keep_tracks=True, cfg10=Ekf10Config(**off))
    d = ekf10_experiment("S3", seeds, ("imu_lowend",), sim=sim, keep_tracks=True, map_grade=False)
    np.testing.assert_array_equal(c["_tracks"]["e_s"], d["_tracks"]["e_s"])


def test_no_grade_road_hides_grade():
    from tunnelsim.experiments import get_scenario
    from tunnelsim.mapview import MapView, map_has_grade, without_grade
    road = get_scenario("S3").road
    assert map_has_grade(road) and not map_has_grade(without_grade(road))
    assert MapView(road).has_grade and not MapView(without_grade(road)).has_grade
