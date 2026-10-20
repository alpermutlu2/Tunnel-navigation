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
    """Engine idle (narrow 25 Hz) is outside the road band; driving at 15 km/h is inside."""
    stop = ImuTruthSource.static(30.0)
    m0, _ = simulate_imu(stop, NOMINAL, Switches.only("vib_road", "vib_engine", "a_white"), seed=1)
    drive = ImuTruthSource(t=stop.t, f_p=stop.f_p, w_p=stop.w_p, v=np.full(stop.t.size, 15 / 3.6), C_pb=stop.C_pb)
    m1, _ = simulate_imu(drive, NOMINAL, Switches.only("vib_road", "vib_engine", "a_white"), seed=1)
    cfg = EskfConfig()
    s0 = zupt_detector(m0.t, m0.f[None], cfg, 100.0)[0, 200:-200]
    s1 = zupt_detector(m1.t, m1.f[None], cfg, 100.0)[0, 200:-200]
    assert np.mean(s0) > 0.99
    assert np.mean(s1) < 0.01


def test_zupt_detector_on_s4_stops():
    """S4: detected stops cover the true stops; no detection while the car moves faster than
    0.5 m/s (a launch after a stop must not be taken as standing)."""
    from tunnelsim.experiments import simulate_run
    sc, tr, t, F, W, g, bt, bh = simulate_run("S4", [0])
    st = zupt_detector(t, F, EskfConfig(), 100.0)[0]
    stop = tr.v < 0.05
    assert np.mean(st[stop]) > 0.9
    assert np.max(tr.v[st]) < 0.5


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
