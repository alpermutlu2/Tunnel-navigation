"""Tests for the ESKF (v0.5)."""
import ast
from pathlib import Path

import numpy as np
import pytest

from tunnelsim.eskf import EskfConfig, run_eskf, skew_b, triad, zupt_detector
from tunnelsim.experiments import eskf_experiment, simulate_run
from tunnelsim.frames import G0, mount_rotation, rot_z, so3_exp
from tunnelsim.profiles import NOMINAL
from tunnelsim.sensors import ImuTruthSource, Switches, simulate_imu

ROOT = Path(__file__).resolve().parents[1]


def test_eskf_imports_nothing_from_truth_side():
    tree = ast.parse((ROOT / "tunnelsim" / "eskf.py").read_text())
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert mods <= {"__future__", "dataclasses", "numpy", "scipy.signal", "frames", "ins", "meas"}, mods


def test_triad_recovers_attitude():
    C_bp = mount_rotation(1.0, 12.0, 3.0)
    C_nb = rot_z(0.7)
    C_np = C_nb @ C_bp
    f_p = C_np.T @ np.array([0, 0, G0])
    v_n = C_nb @ np.array([20.0, 0, 0])
    np.testing.assert_allclose(triad(f_p, C_bp.T[:, 0], v_n), C_np, atol=1e-12)


def test_nhc_jacobian_finite_difference():
    C_bp = mount_rotation(1, 12, 3)
    C = rot_z(0.3) @ C_bp
    v = np.array([20.0, 5.0, 0.3])
    vb = lambda C_, v_: (C_bp @ C_.T) @ v_
    H_th = (C_bp @ C.T) @ skew_b(v[None])[0]
    for i in range(3):
        d = np.zeros(3); d[i] = 1e-6
        np.testing.assert_allclose((vb(so3_exp(d) @ C, v) - vb(C, v)) / 1e-6, H_th[:, i], atol=1e-4)


def test_zupt_detector_separates_idle_from_driving():
    """Engine idle (narrow 25 Hz) is outside the road band; driving at 15 km/h is inside.
    Offline (forward-backward): idle always detected. Causal (default since v0.5.10): one pass
    rejects the engine tone less, so idle is detected only part of the time (deliberately
    conservative); driving is never taken as standing."""
    stop = ImuTruthSource.static(30.0)
    m0, _ = simulate_imu(stop, NOMINAL, Switches.only("vib_road", "vib_engine", "a_white"), seed=1)
    drive = ImuTruthSource(t=stop.t, f_p=stop.f_p, w_p=stop.w_p, v=np.full(stop.t.size, 15 / 3.6), C_pb=stop.C_pb)
    m1, _ = simulate_imu(drive, NOMINAL, Switches.only("vib_road", "vib_engine", "a_white"), seed=1)
    for cfg, idle_min in ((EskfConfig(zupt_causal=False), 0.99), (EskfConfig(), 0.5)):
        s0 = zupt_detector(m0.t, m0.f[None], cfg, 100.0)[0, 200:-200]
        s1 = zupt_detector(m1.t, m1.f[None], cfg, 100.0)[0, 200:-200]
        assert np.mean(s0) > idle_min
        assert np.mean(s1) < 0.01


def test_zupt_detector_on_s4_stops():
    """S4: detected stops cover the true stops, and the detector does not hold "stopped" while
    the car moves (a launch after a stop must not be taken as standing). Offline: > 90 % of the
    stop time, nothing above 0.5 m/s. Causal (default): > 70 % of the stop time, no run of more
    than 0.5 s while the car moves faster than 0.5 m/s (10 seeds)."""
    from tunnelsim.experiments import simulate_run
    sc, tr, t, F, W, g, bt, bh = simulate_run("S4", np.arange(10))
    stop = tr.v < 0.05
    st = zupt_detector(t, F, EskfConfig(zupt_causal=False), 100.0)
    assert np.mean(st[0, stop]) > 0.9
    assert np.max(tr.v[st[0]]) < 0.5
    st = zupt_detector(t, F, EskfConfig(), 100.0)
    assert np.mean(st[:, stop]) > 0.7
    moving = st & (tr.v[None] > 0.5)
    assert (moving.sum(axis=1) / 100.0 <= 0.5).all()


def test_nominal_s1_tracks_through_tunnel():
    """Nominal sensors, 2 seeds: the along-track error at the S1 exit stays far below the
    INS-only kilometre scale, and open-sky GNSS is never rejected."""
    r = eskf_experiment("S1", seeds=2, overrides=())
    assert r["real_exit"]["rmse"] < 100.0, r["real_exit"]
    assert r["events"]["gnss_pos_rej"] == 0


def test_capped_update_limits_position_step_and_keeps_covariance_valid():
    from tunnelsim.eskf import NX, _update_capped
    rng = np.random.default_rng(0)
    A = rng.normal(size=(NX, NX))
    P = np.stack([A @ A.T * 100.0 + np.eye(NX)])
    x = np.zeros((1, NX))
    H = np.zeros((1, 3, NX)); H[0, :, 0:3] = np.eye(3)
    z = np.array([[40.0, -30.0, 5.0]])                       # 50 m innovation
    R = np.eye(3)[None] * 9.0
    ok, alpha = _update_capped(x, P, z, H, R, None, 2.0)
    assert np.linalg.norm(x[0, 0:3]) == pytest.approx(2.0, rel=1e-9)
    assert alpha[0] < 1.0
    assert np.all(np.linalg.eigvalsh(P[0]) > 0)


def test_reacquisition_reduces_exit_jump():
    """S2, 4 seeds: the re-acquisition phase keeps the largest exit step small."""
    on = eskf_experiment("S2", seeds=4)
    off = eskf_experiment("S2", seeds=4, eskf_cfg=EskfConfig(reacq=False))
    assert on["recovery"]["max_step_jump_m"]["p95_abs"] < 0.3 * off["recovery"]["max_step_jump_m"]["p95_abs"]
    assert on["real_exit"]["rmse"] == pytest.approx(off["real_exit"]["rmse"], rel=1e-6)


def test_speed_clone_increment_carries_no_information_without_drift():
    """B (increment mode): right after cloning, speed − clone has zero variance, so the
    increment pseudo-measurement must not reduce any covariance (the clone holds the
    correlation that the absolute hold ignored)."""
    from tunnelsim.eskf import NX, _update
    rng = np.random.default_rng(3)
    NA = NX + 1
    A = rng.normal(size=(NA, NA))
    P = A @ A.T + np.eye(NA)
    u = np.array([0.6, 0.8, 0.0])
    J = np.zeros(NA); J[3:6] = u
    row = J @ P
    P[NX, :] = row; P[:, NX] = row; P[NX, NX] = row @ J
    P = P[None]
    P0 = P.copy()
    H = np.zeros((1, 1, NA)); H[0, 0, 3:6] = u; H[0, 0, NX] = -1.0
    x = np.zeros((1, NA))
    _update(x, P, np.array([[0.3]]), H, np.full((1, 1, 1), 1e-4), None)
    assert np.allclose(P, P0, atol=1e-9)
    assert np.allclose(x, 0.0)


def test_gm_speed_identity_carries_no_information_at_start():
    """gm cruise mode (review 6): right after the start (v_ref cloned, d ~ N(0, σ_d²)),
    v∥ − v_ref − d has zero variance, so the first pseudo-measurement must not change P."""
    from tunnelsim.eskf import NX, _update
    rng = np.random.default_rng(5)
    NA, ID = NX + 2, NX + 1
    A = rng.normal(size=(NX, NX))
    P = np.zeros((NA, NA)); P[:NX, :NX] = A @ A.T + np.eye(NX)
    u = np.array([0.0, 0.6, 0.8])
    J = np.zeros(NA); J[3:6] = u
    row = J @ P
    sd2 = 0.25
    P[NX, :] = row; P[:, NX] = row; P[NX, NX] = row @ J + sd2
    P[NX, ID] = P[ID, NX] = -sd2; P[ID, ID] = sd2
    assert np.all(np.linalg.eigvalsh(P) > -1e-9)
    P = P[None]; P0 = P.copy()
    H = np.zeros((1, 1, NA)); H[0, 0, 3:6] = u; H[0, 0, NX] = -1.0; H[0, 0, ID] = -1.0
    x = np.zeros((1, NA))
    _update(x, P, np.array([[0.2]]), H, np.full((1, 1, 1), 1e-4), None)
    assert np.allclose(P, P0, atol=1e-9)


def test_interactive_server_page_builds():
    """tools/serve.py: one parameter set (non-default switches) renders a simulator page."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import serve
    page = serve.run_page({"scenario": "S1-0.5km", "seed": "1", "mount_err": "2", "cruise": "0",
                           "nhc_turn_gate": "0.5", "gnss_coasting": "1"})
    assert "let D = {" in page and "est_heading" in page


def test_mapview_matching_and_main_continuation():
    """Map view (review 9): windowed nearest-point matching, lateral offset stress, and the
    straight main-tunnel continuation after the diverge."""
    from tunnelsim.mapview import MapView
    from tunnelsim.scenario import build_scenarios
    road = build_scenarios()["S2"].road
    mv = MapView(road)
    s0 = np.array([1000.0, road.exits[0].s + 120.0])
    p = road.position(s0)[:, :2]
    np.testing.assert_allclose(mv.nearest(p, np.array([np.nan, s0[1] + 50.0])), s0, atol=1.0)
    c, psi, n = mv.at(s0)
    np.testing.assert_allclose(c, p, atol=1.0)
    mo = MapView(road, lat_offset_m=2.0)
    c2, _, _ = mo.at(s0)
    np.testing.assert_allclose(np.einsum("bi,bi->b", c2 - c, n), 2.0, atol=0.05)
    cm, psim, _ = mv.at(s0, main=np.array([False, True]))
    assert abs(psim[1] - mv.main_psi) < 1e-12 and abs(psi[1] - mv.main_psi) > np.deg2rad(5)


def test_levelling_jacobian_finite_difference():
    """h(C, b_a) = −Cᵀg + b_a with Ĉ = Exp(δθ)C: ∂h/∂δθ = −Cᵀ[g×], ∂h/∂δb_a = I."""
    from tunnelsim.frames import GRAVITY_N
    C = rot_z(0.4) @ mount_rotation(2, 10, 4)
    h = lambda C_: -C_.T @ GRAVITY_N
    H_th = -C.T @ skew_b(GRAVITY_N[None])[0]
    for i in range(3):
        d = np.zeros(3); d[i] = 1e-6
        np.testing.assert_allclose((h(so3_exp(d) @ C) - h(C)) / 1e-6, H_th[:, i], atol=1e-4)


def test_imu_class_detection_separates_note10_and_mpu9250():
    """Review 11: gyro white noise in the first 10 s picks the filter profile (noisy vibration
    included); a Note10 batch keeps the default tuning."""
    from tunnelsim.eskf import detect_imu_class
    cfg = EskfConfig()
    for ov, want in (([], "note10"), (["vib_noisy"], "note10"), (["imu_lowend"], "mpu9250"), (["imu_lowend", "vib_noisy"], "mpu9250")):
        _, _, t, F, W, gnss, _, _ = simulate_run("S4", [0, 1, 2], ov)
        cls, sig = detect_imu_class(t, W, cfg, int(np.searchsorted(t, gnss["t"][0])))
        assert (cls == want).all(), (ov, sig)


def test_custom_tunnel_scenario():
    """Review 12: a live-call scenario from parameters (length, grade, speed, one stop)."""
    from tunnelsim.scenario import get_scenario
    from tunnelsim.truth import generate_truth
    sc = get_scenario("custom:L=2000,g=3,v=70,stop=20,at=0.3")
    tr = generate_truth(sc)
    s_e, s_x = sc.road.tunnel.s_entry, sc.road.tunnel.s_exit
    assert abs((s_x - s_e) - 2000.0) < 1e-6
    inside = (tr.s > s_e + 150) & (tr.s < s_x - 150)
    ds = np.diff(tr.s[inside]); mv = ds > 1e-3
    dz = np.diff(tr.pos_n[inside, 2])[mv] / ds[mv]
    assert abs(np.median(dz) - 0.03) < 2e-3                      # 3 % grade inside
    stopped = inside & (tr.v < 0.05)
    assert 15.0 < stopped.sum() * tr.dt < 25.0                   # one ~20 s stop
    with pytest.raises(ValueError):
        get_scenario("custom:L=500,stop=30,at=0.5")              # no room before the diverge


def test_live_slider_overrides_keep_tags():
    """Owner 2026-10-23: slider values arrive as "set:path=v" / "scale:path=f" and stay tagged."""
    from tunnelsim.profiles import configure, NOMINAL
    cfg = configure("set:gnss.exit_multipath_m=45", "scale:imu.accel_bias_instab=2")
    assert cfg.gnss.exit_multipath_m.v == 45.0 and cfg.gnss.exit_multipath_m.tag == "ASSUMED"
    assert cfg.imu.accel_bias_instab.v == tuple(2 * x for x in NOMINAL.imu.accel_bias_instab.v)
    assert cfg.imu.accel_bias_instab.src == "live slider"
