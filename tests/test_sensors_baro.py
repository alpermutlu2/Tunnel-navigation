"""Tests for the barometer generator (v0.3, decision K7).

Grade floor: least-squares slope of the barometric altitude over a window T. Three numbers per
case: exact (p' Sigma p with the LS slope weights p and the noise covariance), Monte Carlo from
the generator's own noise blocks, and the closed-form approximations used in the design
discussion (white: sigma sqrt(12/(f T^3)); GM for T << tau: sqrt(6q/(5T)), q = 2 sigma^2/tau).
The approximation-to-exact ratio shows where T << tau breaks down.
"""
from dataclasses import fields

import numpy as np
import pytest

from tunnelsim import build_scenarios, generate_truth
from tunnelsim.frames import G0
from tunnelsim.profiles import BARO, NOMINAL, Param, configure
from tunnelsim.sensors import (BaroTruthSource, Switches, gm_sum, simulate_baro, term_rng,
                               tunnel_pressure_altitude, white)

WINDOWS_S = [5.0, 10.0, 20.0, 30.0, 60.0]
SPEED_MPS = 25.0
FLOOR_CASES = ["nominal", "baro_white_only", "baro_gm_0p3_300", "baro_gm_0p3_60"]
MC_WINDOWS = 2000


def _cfg(name):
    return NOMINAL if name == "nominal" else configure(name)


def _slope_weights(n, dt):
    t = np.arange(n) * dt
    A = np.column_stack([t, np.ones(n)])
    return np.linalg.pinv(A)[0]


def slope_std_exact(p, white_sigma, gm_sigma, gm_tau, dt):
    n = p.size
    var = white_sigma**2 * np.sum(p * p)
    if gm_sigma > 0:
        lag = np.abs(np.subtract.outer(np.arange(n), np.arange(n))) * dt
        var += gm_sigma**2 * p @ np.exp(-lag / gm_tau) @ p
    return float(np.sqrt(var))


def slope_std_approx(white_sigma, gm_sigma, gm_tau, f, T):
    var = white_sigma**2 * 12.0 / (f * T**3)
    if gm_sigma > 0:
        var += 6.0 * (2.0 * gm_sigma**2 / gm_tau) / (5.0 * T)
    return float(np.sqrt(var))


def grade_floor_report(seed: int = 77) -> dict:
    out = {}
    for case in FLOOR_CASES:
        b = _cfg(case).baro
        f = float(b.rate_hz.v)
        dt = 1.0 / f
        ws, gs, gt = float(b.white_sigma_m.v), float(b.gm_sigma_m.v), float(b.gm_tau_s.v)
        rows = {}
        for T in WINDOWS_S:
            n = int(round(T * f))
            p = _slope_weights(n, dt)
            N = n * MC_WINDOWS
            x = white(term_rng(seed, "b_white"), N, dt, ws * np.sqrt(dt))[:, 0]
            if gs > 0:
                x += gm_sum(term_rng(seed, "b_gm"), N, dt, (gs,), (gt,))
            mc = float(np.std(x.reshape(MC_WINDOWS, n) @ p))
            ex = slope_std_exact(p, ws, gs, gt, dt)
            ap = slope_std_approx(ws, gs, gt, f, T)
            rows[f"T{T:g}s"] = {
                "sigma_hdot_exact_mps": float(f"{ex:.3e}"),
                "sigma_hdot_mc_mps": float(f"{mc:.3e}"),
                "mc_vs_exact": round(mc / ex - 1, 4),
                "tolerance": 0.06,                       # ~3 sigma of a 2000-window MC std
                "approx_vs_exact": round(ap / ex - 1, 4),
                "grade_pct_at_25mps": float(f"{100 * ex / SPEED_MPS:.4f}"),
                "accel_equiv_mmps2": float(f"{1000 * G0 * ex / SPEED_MPS:.3f}"),
            }
        out[case] = rows
    return out


def tunnel_gradient_report() -> dict:
    sc = build_scenarios()["S3"]
    tr = generate_truth(sc)
    src = BaroTruthSource.from_truth(tr, sc)
    out = {}
    for case in ("baro_tunnel_10pa", "baro_tunnel_30pa"):
        b = configure(case).baro
        _, e = simulate_baro(src, configure(case), Switches.only("b_tunnel"))
        L = src.s_exit - src.s_entry
        expect = float(b.tunnel_grad_pa_per_km.v) * L / 1000.0 / float(b.rho_g_pa_per_m.v)
        out[case] = {"tunnel_length_m": L, "exit_step_expected_m": round(expect, 4),
                     "exit_step_achieved_m": round(float(-e.components["b_tunnel"].min()), 4),
                     "fake_grade_pct": round(100 * expect / L, 4)}
    return out


# ---------------------------------------------------------------------------------------------
def test_every_number_is_tagged():
    for f in fields(BARO):
        assert isinstance(getattr(BARO, f.name), Param), f.name


@pytest.mark.parametrize("case", FLOOR_CASES)
def test_grade_floor_mc_matches_exact(case):
    rep = grade_floor_report()[case]
    for T, r in rep.items():
        assert abs(r["mc_vs_exact"]) <= r["tolerance"], (case, T, r)


def test_white_only_closed_form_is_exact_enough():
    for r in grade_floor_report()["baro_white_only"].values():
        assert abs(r["approx_vs_exact"]) < 0.02


def test_tunnel_gradient_step_and_zero_outside():
    rep = tunnel_gradient_report()
    for r in rep.values():
        assert r["exit_step_achieved_m"] == pytest.approx(r["exit_step_expected_m"], rel=1e-3)
    s = np.array([0.0, 100.0, 600.0, 1100.0, 1200.0])
    d = tunnel_pressure_altitude(s, 100.0, 1100.0, 11.8, 11.8)       # 11.8 Pa/km -> 1 m per km
    np.testing.assert_allclose(d, [0.0, 0.0, -0.5, -1.0, 0.0])


def test_noise_levels_and_offset():
    sc = build_scenarios()["S1"]
    src = BaroTruthSource.from_truth(generate_truth(sc), sc)
    _, e = simulate_baro(src, NOMINAL, Switches.all_on(), seed=3)
    assert np.std(e.components["b_white"]) == pytest.approx(float(BARO.white_sigma_m.v), rel=0.03)
    assert np.all(e.components["b_cabin"] == 0.0)                         # A6 baseline: off
    assert np.all(e.components["b_tunnel"] == 0.0)                        # nominal: off
    offs = [simulate_baro(src, NOMINAL, Switches.only("b_offset"), seed=k)[1].constants["b_h0"]
            for k in range(400)]
    assert np.std(offs) == pytest.approx(float(BARO.offset_sigma_m.v), rel=0.1)


def test_cabin_override():
    sc = build_scenarios()["S1"]
    src = BaroTruthSource.from_truth(generate_truth(sc), sc)
    stds = [np.std(simulate_baro(src, configure("baro_cabin"), Switches.only("b_cabin"), seed=k)[1]
                   .components["b_cabin"]) for k in range(20)]
    assert np.mean(stds) == pytest.approx(0.3, rel=0.1)


def test_measurement_is_truth_plus_components():
    sc = build_scenarios()["S3"]
    src = BaroTruthSource.from_truth(generate_truth(sc), sc)
    m, e = simulate_baro(src, configure("baro_tunnel_10pa", "baro_cabin"), Switches.all_on(), seed=5)
    np.testing.assert_allclose(m.h, e.h_true + sum(e.components.values()), atol=1e-12)
    np.testing.assert_allclose(np.diff(m.t), 1.0 / float(BARO.rate_hz.v), rtol=1e-9)


def test_switching_baro_terms_off_does_not_change_others():
    sc = build_scenarios()["S3"]
    src = BaroTruthSource.from_truth(generate_truth(sc), sc)
    _, a = simulate_baro(src, NOMINAL, Switches.all_on(), seed=8)
    _, b = simulate_baro(src, NOMINAL, Switches.all_on().without("b_gm", "b_offset"), seed=8)
    np.testing.assert_array_equal(a.components["b_white"], b.components["b_white"])
