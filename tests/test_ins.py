"""Tests for the strapdown INS (v0.4 step 2).

The harness functions here read the truth (oracle_init, ideal attitude, perfect calibration);
`tunnelsim/ins.py` itself never does (import test). Report functions feed tools/snapshot.py.
"""
import ast
from pathlib import Path

import numpy as np
import pytest

from tunnelsim.frames import attitude_error_n
from tunnelsim.ins import ImuCalibration, InsInit, so3_exp_batch, strapdown, strapdown_meas
from tunnelsim.frames import so3_exp
from tunnelsim.profiles import NOMINAL
from tunnelsim.sensors import ImuTruthSource, Switches, simulate_imu
from tunnelsim.truth import generate_truth
from test_truth import ALL

ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------------------------
# Harness (oracle side)
# ---------------------------------------------------------------------------------------------
def oracle_init(tr, t0=None) -> InsInit:
    """Truth state at the TRUE time of the first IMU sample (docs/contracts.md 4). With sampling
    jitter that time is not on the truth grid: position/velocity by spline, attitude by
    propagating the nearest truth sample with its body rate."""
    if t0 is None:
        t0 = tr.t[0]
    from scipy.interpolate import CubicSpline
    i = int(np.clip(np.round((t0 - tr.t[0]) / tr.dt), 0, len(tr.t) - 1))
    C = tr.C_nb[i] @ tr.C_bp @ so3_exp(tr.w_p[i] * (t0 - tr.t[i]))
    return InsInit(p_n=CubicSpline(tr.t, tr.pos_phone_n, axis=0)(t0),
                   v_n=CubicSpline(tr.t, tr.vel_phone_n, axis=0)(t0), C_np=C)


def truth_at(tr, t):
    """Phone position and attitude of the truth at times t (nearest truth sample; exact on the
    truth grid, used only when the IMU grid equals the truth grid)."""
    idx = np.clip(np.round((t - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    assert np.allclose(tr.t[idx], t, atol=1e-9), "truth_at needs IMU samples on the truth grid"
    return tr.pos_phone_n[idx], tr.C_nb[idx] @ tr.C_bp


def run_errors(tr, res, t_true):
    """Position (m) and attitude (rad) error vs truth at the true sample times."""
    from scipy.interpolate import CubicSpline
    p_true = CubicSpline(tr.t, tr.pos_phone_n, axis=0)(t_true)
    dp = np.linalg.norm(res.p_n[0] - p_true, axis=1)
    idx = np.clip(np.round((t_true - tr.t[0]) / tr.dt).astype(int), 0, len(tr.t) - 1)
    ks = range(0, len(t_true), 50)
    dth = np.array([np.linalg.norm(attitude_error_n(res.C_np[0, k], tr.C_nb[idx[k]] @ tr.C_bp)) for k in ks])
    return float(dp.max()), float(dth.max())


# ---------------------------------------------------------------------------------------------
# Report functions
# ---------------------------------------------------------------------------------------------
def error_free_report() -> dict:
    """All sensor errors and vibration off: the INS must reproduce the truth."""
    out = {}
    for key, sc in ALL.items():
        tr = generate_truth(sc)
        m, e = simulate_imu(ImuTruthSource.from_truth(tr), NOMINAL, Switches.only(), seed=0)
        res = strapdown_meas(m, oracle_init(tr))
        dp, dth = run_errors(tr, res, e.t_true)
        out[key] = {"max_pos_err_m": float(f"{dp:.3e}"), "max_att_err_rad": float(f"{dth:.3e}")}
    return out


def analytic_motion(rate_hz, two_axis, T=60.0, t=None):
    """Closed-form trajectory: exact position, attitude, specific force and body rate, at any
    times t (default: uniform grid at rate_hz). Attitude C = Rz(psi) (one axis) or
    Rz(psi) Ry(theta) (two axes, coning present)."""
    from tunnelsim.frames import GRAVITY_N, rot_y, rot_z
    if t is None:
        t = np.arange(int(round(T * rate_hz)) + 1) / rate_hz
    psi = 0.3 * np.sin(0.2 * t) + 0.05 * t
    psid = 0.06 * np.cos(0.2 * t) + 0.05
    th = 0.05 * np.sin(0.3 * t) if two_axis else 0 * t
    thd = 0.015 * np.cos(0.3 * t) if two_axis else 0 * t
    p = np.column_stack([20 * t + 5 * np.sin(0.1 * t), 3 * np.cos(0.15 * t), 0.5 * np.sin(0.2 * t)])
    v = np.column_stack([20 + 0.5 * np.cos(0.1 * t), -0.45 * np.sin(0.15 * t), 0.1 * np.cos(0.2 * t)])
    a = np.column_stack([-0.05 * np.sin(0.1 * t), -0.0675 * np.cos(0.15 * t), -0.02 * np.sin(0.2 * t)])
    Ry = rot_y(th)
    C = rot_z(psi) @ Ry
    f = np.einsum("nji,nj->ni", C, a - GRAVITY_N)
    w = np.einsum("nji,nj->ni", Ry, np.column_stack([0 * t, 0 * t, psid])) + np.column_stack([0 * t, thd, 0 * t])
    return t, p, v, C, f, w


def convergence_report(rates=(25.0, 50.0, 100.0, 200.0, 400.0)) -> dict:
    """Error-free mechanisation at several rates.
    analytic_*: closed-form motion, isolates the mechanisation; expected order 2.
    MIX: truth generator at each rate; plateaus at the truth's own discretisation floor
    (np.gradient for w_dot, sideslip FOH, event timing on the sample grid), not a mechanisation
    error."""
    out = {}
    for name in ("analytic_1axis", "analytic_2axis", "MIX"):
        errs = []
        for r in rates:
            if name == "MIX":
                tr = generate_truth(ALL["MIX"], r)
                res = strapdown(tr.t, tr.f_p, tr.w_p, oracle_init(tr))
                errs.append(float(np.linalg.norm(res.p_n[0] - tr.pos_phone_n, axis=1).max()))
            else:
                t, p, v, C, f, w = analytic_motion(r, name.endswith("2axis"))
                res = strapdown(t, f, w, InsInit(p[0], v[0], C[0]))
                errs.append(float(np.linalg.norm(res.p_n[0] - p, axis=1).max()))
        ratios = [errs[i] / errs[i + 1] for i in range(len(errs) - 1)]
        out[name] = {"rates_hz": list(rates), "max_pos_err_m": [float(f"{x:.3e}") for x in errs],
                     "order": [round(float(np.log2(x)), 2) for x in ratios]}
    return out


def variable_dt_report() -> dict:
    """Irregular dt from timestamps.

    analytic_2axis: closed-form motion sampled at jittered instants (0.5 ms, clipped 3 sigma)
    with dropped samples (3 per event, 1 per 120 s, as the nominal profile); f and w are exact
    at every instant, so this isolates the mechanisation. Compared with the uniform grid.
    S1/S4/MIX: the IMU generator's jitter and gaps on the truth (informative): off-grid
    instants need the truth interpolated, which leaves a small resampling floor that grows
    with run length (S4, 585 s run: ~1 cm), not a mechanisation error.
    """
    out = {}
    rng = np.random.default_rng(7)
    T, r = 600.0, 100.0
    tg = np.arange(int(T * r) + 1) / r
    tj = tg + np.clip(rng.normal(0, 0.5e-3, tg.size), -1.5e-3, 1.5e-3)
    tj[0] = 0.0
    keep = np.ones(tg.size, bool)
    for st in rng.integers(1, tg.size - 3, int(T / 120)):
        keep[st:st + 3] = False
    errs = {}
    for name, tt in (("uniform", tg), ("jitter_gaps", tj[keep])):
        t, p, v, C, f, w = analytic_motion(r, True, t=tt)
        res = strapdown(t, f, w, InsInit(p[0], v[0], C[0]))
        errs[name] = float(np.linalg.norm(res.p_n[0] - p, axis=1).max())
    out["analytic_2axis_600s"] = {"uniform_max_pos_err_m": float(f"{errs['uniform']:.3e}"),
                                  "jitter_gaps_max_pos_err_m": float(f"{errs['jitter_gaps']:.3e}"),
                                  "dropped_samples": int((~keep).sum())}
    for key in ("S1", "S4", "MIX"):
        tr = generate_truth(ALL[key])
        src = ImuTruthSource.from_truth(tr)
        row = {}
        for label, sw in (("error_free", Switches.only()), ("jitter_gaps", Switches.only("ts_jitter", "ts_gaps"))):
            m, e = simulate_imu(src, NOMINAL, sw, seed=1)
            dp, _ = run_errors(tr, strapdown_meas(m, oracle_init(tr, e.t_true[0])), e.t_true)
            row[f"{label}_max_pos_err_m"] = float(f"{dp:.3e}")
        row["run_s"] = round(float(tr.t[-1]), 1)
        out[key] = row
    return out


# ---------------------------------------------------------------------------------------------
def test_ins_imports_nothing_from_truth_side():
    tree = ast.parse((ROOT / "tunnelsim" / "ins.py").read_text())
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            mods.add(node.module or "")
    assert mods <= {"__future__", "dataclasses", "numpy", "frames", "meas"}, mods


def test_so3_exp_batch_matches_single():
    rng = np.random.default_rng(0)
    phi = rng.normal(size=(50, 3)) * rng.uniform(0, 2, size=(50, 1))
    phi[0] = 0.0
    phi[1] = 1e-10
    for k in range(50):
        np.testing.assert_allclose(so3_exp_batch(phi[k:k + 1])[0], so3_exp(phi[k]), atol=1e-12)


@pytest.mark.parametrize("key", list(ALL))
def test_error_free_mechanisation(key):
    """All errors off: reproduces the truth (v0.3 truth strapdown: <= 0.13 cm on S1-S5)."""
    r = error_free_report_cached()[key]
    assert r["max_pos_err_m"] < 2e-3, r
    assert r["max_att_err_rad"] < 1e-6, r


def test_second_order_convergence():
    """Closed-form motion: error falls at second order with dt (one- and two-axis rotation)."""
    rep = convergence_report()
    for name in ("analytic_1axis", "analytic_2axis"):
        assert all(1.8 < o < 2.2 for o in rep[name]["order"]), (name, rep[name])
    assert rep["MIX"]["max_pos_err_m"][2] < 1e-3                 # 100 Hz, truth-limited


def test_variable_dt_from_timestamps():
    """Mechanisation with irregular dt (exact signals): no worse than the uniform grid beyond
    a small margin; scenario resampling floor stays below 2 cm."""
    rep = variable_dt_report()
    a = rep["analytic_2axis_600s"]
    assert a["dropped_samples"] > 0
    assert a["jitter_gaps_max_pos_err_m"] < 1.5 * a["uniform_max_pos_err_m"] + 1e-4, a
    for key in ("S1", "S4", "MIX"):
        assert rep[key]["jitter_gaps_max_pos_err_m"] < 0.02, (key, rep[key])


def test_calibration_interface_removes_constant_errors():
    """Turn-on bias, scale, misalignment on; perfect calibration 'all' given as parameters ->
    identical to the error-free run (corrections applied inside the INS)."""
    tr = generate_truth(ALL["MIX"])
    src = ImuTruthSource.from_truth(tr)
    terms = ("a_turnon", "g_turnon", "a_scale", "g_scale", "a_misalign", "g_misalign")
    m, e = simulate_imu(src, NOMINAL, Switches.only(*terms), seed=5)
    m0, _ = simulate_imu(src, NOMINAL, Switches.only(), seed=5)
    k = e.constants
    I3 = np.eye(3)
    cal = ImuCalibration(accel_bias=k["b_a_on"], gyro_bias=k["b_g_on"],
                         accel_matrix=I3 + k["S_a"] + k["M_a"], gyro_matrix=I3 + k["S_g"] + k["M_g"])
    r_cal = strapdown_meas(m, oracle_init(tr), cal)
    r_ref = strapdown_meas(m0, oracle_init(tr))
    np.testing.assert_allclose(r_cal.p_n, r_ref.p_n, atol=1e-6)
    r_raw = strapdown_meas(m, oracle_init(tr))
    assert np.abs(r_raw.p_n - r_ref.p_n).max() > 10.0          # uncorrected: large drift


def test_ideal_attitude_ignores_gyro():
    """Attitude supplied as input: gyro errors have no effect."""
    tr = generate_truth(ALL["S2"])
    src = ImuTruthSource.from_truth(tr)
    m, e = simulate_imu(src, NOMINAL, Switches.only("g_turnon", "g_bi", "g_white"), seed=2)
    _, att = truth_at(tr, e.t_true)
    r = strapdown_meas(m, oracle_init(tr), attitude=att)
    m0, _ = simulate_imu(src, NOMINAL, Switches.only(), seed=2)
    r0 = strapdown_meas(m0, oracle_init(tr), attitude=att)
    np.testing.assert_allclose(r.p_n, r0.p_n, atol=1e-9)
    r_free = strapdown_meas(m, oracle_init(tr))
    assert np.abs(r_free.p_n - r0.p_n).max() > 1.0


def test_batch_equals_single():
    tr = generate_truth(ALL["S5"])
    src = ImuTruthSource.from_truth(tr)
    runs = [simulate_imu(src, NOMINAL, Switches.only("a_white", "g_white", "a_bi"), seed=s)[0] for s in range(3)]
    t = np.stack([m.t for m in runs]); f = np.stack([m.f for m in runs]); w = np.stack([m.w for m in runs])
    rb = strapdown(t, f, w, oracle_init(tr))
    for i, m in enumerate(runs):
        rs = strapdown_meas(m, oracle_init(tr))
        np.testing.assert_allclose(rb.p_n[i], rs.p_n[0], atol=1e-9)


_EF = {}


def error_free_report_cached():
    if not _EF:
        _EF.update(error_free_report())
    return _EF
