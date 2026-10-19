"""Tests for the IMU generator (v0.3, decisions K1, K2, K4 to K6 and R1 to R4).

The report functions at the top return the numbers that tools/snapshot.py writes into
metrics.json; the tests assert on the same numbers, each with target, achieved and tolerance.
"""
import ast
from dataclasses import fields
from pathlib import Path

import numpy as np
import pytest
from scipy.interpolate import CubicSpline

from tunnelsim import build_scenarios, generate_truth
from tunnelsim.allan import avar_gm, avar_model, avar_target, gm_change_rms, model_fit_error, oadev
from tunnelsim.profiles import (MPU9250_LOWEND, NOMINAL, NOTE10_LSM6DSR, OVERRIDES, TIMING, VIB_CALM,
                                ImuProfile, Param, configure)
from tunnelsim.sensors import (GM1_SIGMA_PER_B, GM_ADEV_PEAK, TERM_IDS, ImuTruthSource, Switches,
                               bi_components, gm_sum, simulate_imu, term_rng, white)

ROOT = Path(__file__).resolve().parents[1]
AXES = [("accel", "x", 0), ("accel", "y", 1), ("accel", "z", 2),
        ("gyro", "x", 0), ("gyro", "y", 1), ("gyro", "z", 2)]
ALLAN_T = [1.0, 10.0, 30.0, 100.0, 300.0, 1000.0, 3000.0]
OUTAGE_HORIZON_S = 180.0

# Run-to-run 1-sigma relative scatter of OADEV for one 24 h record, worst axis of the Note10
# model (tools/allan_scatter.py, 150 runs, 2026-10-19). Tolerance = max(10 %, 3 sigma).
ALLAN_24H_SCATTER = {1.0: 0.008, 10.0: 0.008, 30.0: 0.013, 100.0: 0.026, 300.0: 0.041,
                     1000.0: 0.076, 3000.0: 0.15}


def _NB(prof: ImuProfile, sensor: str, i: int):
    if sensor == "accel":
        return float(prof.accel_vrw.a[i]), float(prof.accel_bias_instab.a[i])
    return float(prof.gyro_arw.a[i]), float(prof.gyro_bias_instab.a[i])


# ---------------------------------------------------------------------------------------------
# Report functions (shared with tools/snapshot.py)
# ---------------------------------------------------------------------------------------------
def bi_fit_report(prof: ImuProfile = NOTE10_LSM6DSR) -> dict:
    """R2(a): analytic model vs the paper's parametric AVAR, gm3 baseline and gm1 reference."""
    rng_ = tuple(prof.bi_fit_range_s.v)
    out = {}
    for sensor, ax, i in AXES:
        N, B = _NB(prof, sensor, i)
        sig3, tau3 = bi_components(N, B, prof)
        sig1, tau1 = (GM1_SIGMA_PER_B * B,), (float(prof.bi_gm1_tau_s.v),)
        out[f"{sensor}_{ax}"] = {
            "gm3_sigmas": [float(f"{s:.4e}") for s in sig3],
            "gm3_taus_s": list(tau3),
            "gm3_max_adev_err": round(model_fit_error(N, B, sig3, tau3, rng_), 4),
            "gm1_max_adev_err": round(model_fit_error(N, B, sig1, tau1, rng_), 4),
            "tolerance": 0.10,
            "bias_change_rms_180s_gm3": float(f"{gm_change_rms(sig3, tau3, OUTAGE_HORIZON_S):.3e}"),
            "bias_change_rms_180s_gm1": float(f"{gm_change_rms(sig1, tau1, OUTAGE_HORIZON_S):.3e}"),
            "flicker_ref_0p94B": float(f"{0.664 * np.sqrt(2) * B:.3e}"),
        }
    return out


def allan_static_report(prof: ImuProfile = NOTE10_LSM6DSR, hours: float = 24.0, seed: int = 2026) -> dict:
    """R2(b): one long static run of the white + bias-instability generators, per axis.

    Uses the same building blocks as simulate_imu (white, gm_sum via bi_components), axis by
    axis to keep memory bounded. The seed is fixed in advance and never tuned.
    """
    dt = 1.0 / float(prof.rate_hz.v)
    n = int(hours * 3600 / dt)
    tol = {T: max(0.10, 3.0 * s) for T, s in ALLAN_24H_SCATTER.items()}
    out = {}
    for sensor, ax, i in AXES:
        N, B = _NB(prof, sensor, i)
        sig, taus = bi_components(N, B, prof)
        tag = "a" if sensor == "accel" else "g"
        y_w = white(term_rng(seed + 10 * i, f"{tag}_white"), n, dt, [N])[:, 0]
        b = gm_sum(term_rng(seed + 10 * i, f"{tag}_bi"), n, dt, sig, taus)
        T, adev = oadev(y_w + b, dt, ALLAN_T)
        m = int(OUTAGE_HORIZON_S / dt)
        rows = {}
        for Tk, ad in zip(T, adev):
            tgt = float(np.sqrt(avar_target(Tk, N, B)))
            mdl = float(np.sqrt(avar_model(Tk, N, sig, taus)))
            in_range = prof.bi_fit_range_s.v[0] <= Tk <= prof.bi_fit_range_s.v[1]
            rows[f"T{Tk:g}s"] = ({"target": float(f"{tgt:.4e}"), "model": float(f"{mdl:.4e}"),
                         "achieved": float(f"{ad:.4e}"),
                         "err_vs_model": round(ad / mdl - 1, 4),
                         "err_vs_target": round(ad / tgt - 1, 4),
                         "tolerance": round(tol[Tk], 3) if in_range else None})
        out[f"{sensor}_{ax}"] = {
            "adev": rows,
            "bias_change_rms_180s": {
                "model": float(f"{gm_change_rms(sig, taus, OUTAGE_HORIZON_S):.3e}"),
                "achieved": float(f"{np.sqrt(np.mean((b[m:] - b[:-m]) ** 2)):.3e}"),
            },
        }
    return {"hours": hours, "seed": seed, "axes": out}


def interpolation_report() -> dict:
    """Cubic interpolation of the truth onto off-grid sample instants.

    One truth at 200 Hz; the spline is built on its even samples (the 100 Hz grid the
    generator uses) and evaluated at the odd samples. Two truths generated at different rates
    are NOT compared: speed events trigger on the sample grid, so they differ by up to one
    sample in event time (S4: 114.000 s vs 114.005 s), which is not an interpolation error.

    The pass/fail metric is velocity-level (integral of f over the run); the pointwise maximum is
    reported for information. Before the sideslip lag (v0.3), beta_dot stepped at clothoid ends
    and S2 failed this test with ~1 mm/s per joint.
    """
    out = {}
    for key in ("S1", "S2", "S3", "S4", "S5"):
        hi = generate_truth(build_scenarios()[key], 200.0)
        x = np.column_stack([hi.f_p, hi.w_p])
        cs = CubicSpline(hi.t[0::2], x[0::2], axis=0)
        pt = np.abs(cs(hi.t[1::2]) - x[1::2]).max(axis=0)
        edges = hi.t[0::200]
        I_spl = np.array([cs.integrate(edges[0], e)[:3] for e in edges])
        I_ref = np.concatenate([[np.zeros(3)], np.cumsum(
            0.5 * (hi.f_p[1:] + hi.f_p[:-1]) * np.diff(hi.t)[:, None], axis=0)])[0::200][:len(edges)]
        out[key] = {"max_pointwise_f_mps2": float(f"{pt[:3].max():.2e}"),
                    "max_pointwise_w_rps": float(f"{pt[3:].max():.2e}"),
                    "max_integral_f_err_mps": float(f"{np.abs(I_spl - I_ref).max():.2e}")}
    return out


def _dv_check(cfg, key="S5", seed=3):
    """Integral of f over the run with reported timestamps vs the exact truth integral."""
    tr = generate_truth(build_scenarios()[key])
    src = ImuTruthSource.from_truth(tr)
    m, e = simulate_imu(src, cfg, Switches.only("ts_jitter", "ts_gaps"), seed)
    cs = CubicSpline(src.t, src.f_p, axis=0)
    exact = cs.integrate(e.t_true[0], e.t_true[-1])
    meas = np.trapezoid(m.f, m.t, axis=0)
    return exact, meas


def timestamp_report() -> dict:
    out = {}
    for name in ("rate_offset_+2pct", "rate_offset_-2pct", "ts_scale_1pct"):
        cfg = configure(name)
        exact, meas = _dv_check(cfg)
        eps = float(cfg.timing.ts_scale_error.v)
        out[name] = {"dv_err_norm_mps": float(f"{np.linalg.norm(meas - exact):.3e}"),
                     "predicted_mps": float(f"{np.linalg.norm(eps * exact):.3e}")}
    return out


def vibration_report() -> dict:
    """Realised vibration rms, rotated back to the body frame, vs profile values."""
    src = ImuTruthSource.static(600.0)
    _, e = simulate_imu(src, NOMINAL, Switches.only("vib_engine"), seed=5)
    eng = np.sqrt(np.mean((e.components["vib_engine_a"] @ src.C_pb) ** 2, axis=0))
    tr = generate_truth(build_scenarios()["S1"])
    sv = ImuTruthSource.from_truth(tr)
    _, e2 = simulate_imu(sv, NOMINAL, Switches.only("vib_road"), seed=5)
    sel = np.abs(np.interp(e2.t_true, tr.t, tr.v) - tr.v.max()) < 0.01
    v_cruise = float(tr.v.max())
    road = np.sqrt(np.mean((e2.components["vib_road_a"][sel] @ sv.C_pb) ** 2, axis=0))
    expect_road = VIB_CALM.road_accel_rms_b.a * v_cruise / float(VIB_CALM.road_v_ref_mps.v)
    return {"engine_accel_rms_b": {"target": list(VIB_CALM.engine_accel_rms_b.v),
                                   "achieved": [round(float(x), 4) for x in eng]},
            "road_accel_rms_b_at_cruise": {"v_mps": round(v_cruise, 2),
                                           "target": [round(float(x), 4) for x in expect_road],
                                           "achieved": [round(float(x), 4) for x in road]},
            "tolerance": 0.05}


# ---------------------------------------------------------------------------------------------
# Profiles, tags, overrides, separation
# ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize("obj", [NOTE10_LSM6DSR, MPU9250_LOWEND, VIB_CALM, TIMING])
def test_every_number_is_tagged(obj):
    for f in fields(obj):
        val = getattr(obj, f.name)
        if isinstance(val, str):
            continue
        assert isinstance(val, Param), f"{type(obj).__name__}.{f.name} has no tag"


def test_overrides_apply_and_keep_tags():
    for name in OVERRIDES:
        cfg = configure(name)
        assert cfg != NOMINAL, name
    assert NOMINAL.imu.gyro_turn_on_sigma.tag == "ASSUMED"          # nominal untouched
    with pytest.raises(KeyError):
        configure("no_such_case")


def test_override_must_be_tagged():
    from tunnelsim import profiles as P
    P.OVERRIDES["_bad"] = {"imu.gyro_scale_sigma": 0.01}
    try:
        with pytest.raises(TypeError):
            configure("_bad")
    finally:
        del P.OVERRIDES["_bad"]


def test_meas_module_imports_nothing_from_truth_side():
    tree = ast.parse((ROOT / "tunnelsim" / "meas.py").read_text())
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            mods.add(node.module or "")
    assert mods <= {"__future__", "dataclasses", "numpy"}, mods


# ---------------------------------------------------------------------------------------------
# K1 / R2 / R3: bias instability
# ---------------------------------------------------------------------------------------------
def test_gm_peak_constants():
    assert GM_ADEV_PEAK == pytest.approx(0.6174, abs=2e-4)
    T = np.logspace(-1, 2, 30001)
    assert T[np.argmax(avar_gm(T, 1.0, 1.0))] == pytest.approx(1.89, abs=0.01)
    assert GM1_SIGMA_PER_B == pytest.approx(1.076, abs=1e-3)


@pytest.mark.parametrize("prof", [NOTE10_LSM6DSR, MPU9250_LOWEND], ids=["note10", "lowend"])
def test_bi_analytic_model_matches_paper(prof):
    """R2(a): white + GM sum vs the paper's parametric AVAR, +-10 % over 10 to 1000 s."""
    rep = bi_fit_report(prof)
    for ax, r in rep.items():
        assert r["gm3_max_adev_err"] <= r["tolerance"], (ax, r)


@pytest.mark.slow
def test_allan_24h_static():
    """R2(b): one 24 h static run reproduces the model curve (code vs model)."""
    rep = allan_static_report()
    for ax, r in rep["axes"].items():
        for row in r["adev"].values():
            if row["tolerance"] is not None:
                assert abs(row["err_vs_model"]) <= row["tolerance"], (ax, row)


def test_white_noise_level():
    rng = term_rng(0, "a_white")
    dt = 0.01
    x = white(rng, 200000, dt, NOTE10_LSM6DSR.accel_vrw.a)
    np.testing.assert_allclose(x.std(axis=0), NOTE10_LSM6DSR.accel_vrw.a / np.sqrt(dt), rtol=0.01)


def test_gm_stationary_variance():
    """Each GM component is stationary from the first sample with the requested variance."""
    s, tau, dt = 2.0, 50.0, 0.01
    starts = [gm_sum(np.random.default_rng(k), 2, dt, (s,), (tau,))[0] for k in range(4000)]
    assert np.std(starts) == pytest.approx(s, rel=0.05)


def test_turn_on_bias_distribution():
    src = ImuTruthSource.static(1.0)
    b = np.array([simulate_imu(src, NOMINAL, Switches.only("a_turnon"), seed=k)[1].constants["b_a_on"]
                  for k in range(400)])
    assert b.std() == pytest.approx(float(NOMINAL.imu.accel_turn_on_sigma.v), rel=0.1)


# ---------------------------------------------------------------------------------------------
# R1 / K5: vibration
# ---------------------------------------------------------------------------------------------
def test_vibration_passes_through_sensor_errors():
    """R1: f = (I + S_a + M_a)(f_true + vib) exactly, with only these terms on."""
    src = ImuTruthSource.static(20.0)
    sw = Switches.only("vib_engine", "a_scale", "a_misalign")
    m, e = simulate_imu(src, NOMINAL, sw, seed=7)
    K = np.eye(3) + e.constants["S_a"] + e.constants["M_a"]
    expect = (e.f_true + e.components["vib_engine_a"]) @ K.T
    np.testing.assert_allclose(m.f, expect, atol=1e-12)


def test_vibration_levels():
    rep = vibration_report()
    tol = rep["tolerance"]
    for key in ("engine_accel_rms_b", "road_accel_rms_b_at_cruise"):
        np.testing.assert_allclose(rep[key]["achieved"], rep[key]["target"], rtol=tol)


def test_no_road_vibration_when_stopped():
    src = ImuTruthSource.static(10.0)
    _, e = simulate_imu(src, NOMINAL, Switches.only("vib_road"), seed=1)
    assert np.abs(e.components["vib_road_a"]).max() == 0.0


# ---------------------------------------------------------------------------------------------
# K6: timestamps
# ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize("name", ["rate_offset_+2pct", "rate_offset_-2pct"])
def test_rate_offset_with_true_timestamps_has_no_effect(name):
    """K6 case 1: sample-rate offset, timestamps correct -> integral of f unaffected."""
    exact, meas = _dv_check(configure(name))
    assert np.linalg.norm(meas - exact) < 1e-3          # m/s over ~240 s, vs ~2400 m/s integral


def test_timestamp_scale_error_accumulates_as_predicted():
    """K6 case 2: reported = (1 + e) t  ->  integral error = e * integral."""
    cfg = configure("ts_scale_1pct")
    exact, meas = _dv_check(cfg)
    eps = float(cfg.timing.ts_scale_error.v)
    np.testing.assert_allclose(meas - exact, eps * exact, rtol=1e-3, atol=1e-3)


def test_timestamps_monotonic_and_gaps():
    tr = generate_truth(build_scenarios()["S4"])
    m, e = simulate_imu(ImuTruthSource.from_truth(tr), NOMINAL, Switches.all_on(), seed=11)
    assert np.all(np.diff(m.t) > 0)
    dropped = e.n_grid - len(m.t)
    expect = float(TIMING.gap_rate_hz.v) * (tr.t[-1] - tr.t[0]) * int(TIMING.gap_len_samples.v)
    assert 0 < dropped < 4 * expect + 10


INTERP_LIMIT_MPS = 0.1 * float(NOTE10_LSM6DSR.accel_vrw.a.min())     # 10 % of VRW over 1 s


@pytest.mark.parametrize("key", ["S1", "S2", "S3", "S4", "S5"])
def test_interpolation_error_negligible(key):
    """Velocity-level interpolation error over the whole run < 10 % of VRW over 1 s."""
    r = interpolation_report_cached()[key]
    assert r["max_integral_f_err_mps"] < INTERP_LIMIT_MPS, r


_INTERP = {}


def interpolation_report_cached():
    if not _INTERP:
        _INTERP.update(interpolation_report())
    return _INTERP


# ---------------------------------------------------------------------------------------------
# Infrastructure: RNG independence (test 6)
# ---------------------------------------------------------------------------------------------
def test_switching_terms_off_does_not_change_other_terms():
    tr = generate_truth(build_scenarios()["S2"])
    src = ImuTruthSource.from_truth(tr)
    _, full = simulate_imu(src, configure("no_gaps"), Switches.all_on(), seed=42)
    _, part = simulate_imu(src, configure("no_gaps"), Switches.all_on().without("a_bi", "vib_road", "g_scale"),
                           seed=42)
    for k in ("a_white", "g_white", "g_bi", "vib_engine_a", "vib_engine_g"):
        np.testing.assert_array_equal(full.components[k], part.components[k])
    for k in ("b_a_on", "b_g_on", "S_a", "M_a", "M_g"):
        np.testing.assert_array_equal(full.constants[k], part.constants[k])


def test_gaps_and_bi_model_do_not_change_other_terms():
    tr = generate_truth(build_scenarios()["S1"])
    src = ImuTruthSource.from_truth(tr)
    _, nogap = simulate_imu(src, configure("no_gaps"), Switches.all_on(), seed=9)
    _, gaps = simulate_imu(src, NOMINAL, Switches.all_on(), seed=9)
    np.testing.assert_array_equal(nogap.components["a_white"][gaps.kept], gaps.components["a_white"])
    _, gm1 = simulate_imu(src, configure("imu_bi_gm1"), Switches.all_on(), seed=9)
    np.testing.assert_array_equal(gm1.components["a_white"], gaps.components["a_white"])


def test_term_ids_unique():
    assert len(set(TERM_IDS.values())) == len(TERM_IDS)
