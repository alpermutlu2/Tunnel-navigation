"""Tests for the error budget harness (v0.4 step 3)."""
import numpy as np
import pytest

from tunnelsim import budget as Bd
from tunnelsim.frames import mount_rotation
from tunnelsim.profiles import NOMINAL, configure
from tunnelsim.scenario import build_scenarios
from tunnelsim.sensors import ImuTruthSource, Switches, simulate_imu
from tunnelsim.truth import generate_truth

C_PB = mount_rotation(1.0, 12.0, 3.0).T
E_FWD, E_PITCH = C_PB @ [1, 0, 0], C_PB @ [0, 1, 0]
ANALYTIC_CFGS = ("a_white", "a_bi", "a_bi_raw", "a_turnon_raw", "g_white", "g_bi", "g_bi_raw", "g_turnon_raw")


def analytic_check_report(n_seeds=200) -> dict:
    """S1 (straight, flat, constant speed): Monte Carlo rmse at 180 s vs closed form.
    Tolerance 15 % = 3 sigma of an rmse estimated from 200 Gaussian samples (1/sqrt(2N))."""
    cfgs = [c for c in Bd.CONFIGS if c.name in ANALYTIC_CFGS]
    r = Bd.run_scenario("S1", np.arange(n_seeds), cfgs)
    k = list(r.t_rel).index(180.0)
    an = Bd.analytic_rmse(NOMINAL.imu, E_FWD, E_PITCH, 180.0)
    out = {}
    for c in cfgs:
        mc = float(np.sqrt(np.mean(r.e_s[c.name][:, k] ** 2)))
        out[c.name] = {"analytic_rmse_m": an[c.name], "mc_rmse_m": float(f"{mc:.4g}"),
                       "rel_err": round(mc / an[c.name] - 1, 4), "tolerance": 0.15}
    out["seeds"] = n_seeds
    return out


def test_compose_equals_generator():
    """Recomposing a measurement from an all-terms run == generating it with only those terms."""
    sc = build_scenarios()["S2"]
    src = ImuTruthSource.from_truth(generate_truth(sc))
    cfg = configure("no_gaps")
    _, e_all = simulate_imu(src, cfg, Switches.only(*Bd.IMU_TERMS), seed=4)
    for terms in (("a_white",), ("a_scale", "vib_road"), ("g_bi", "g_misalign", "vib_engine"), Bd.IMU_TERMS):
        m, _ = simulate_imu(src, cfg, Switches.only(*terms), seed=4)
        f, w = Bd.compose(e_all, terms)
        np.testing.assert_allclose(f, m.f, atol=1e-12)
        np.testing.assert_allclose(w, m.w, atol=1e-12)


def test_stats_definitions():
    x = np.array([-3.0, 1.0, 2.0, -1.0] * 50)
    s = Bd.stats(x)
    assert s["bias"] == pytest.approx(-0.25)
    assert s["rmse"] == pytest.approx(np.sqrt(3.75), rel=1e-3)          # 4 significant digits
    assert s["p95_abs"] == pytest.approx(3.0)


def test_gm_kernels_limits():
    """tau -> inf: hold -> 0 relative to raw (only a random-walk residue 2 min(u,v)/tau is left),
    raw -> a constant bias b t^2/2 (accel) or the gyro kernel t^3/3."""
    t = 100.0
    assert Bd._gm_kernel(t, 1e9, 1, "hold") < 1e-6 * Bd._gm_kernel(t, 1e9, 1, "raw")
    assert Bd._gm_kernel(t, 1e9, 1, "raw") == pytest.approx((t**2 / 2) ** 2, rel=1e-3)
    assert Bd._gm_kernel(t, 1e9, 2, "raw") == pytest.approx((t**3 / 3) ** 2, rel=1e-3)
    for tau in (5.0, 50.0, 500.0):                       # knowing b(0) can only help (pred <= raw, hold)
        assert Bd._gm_kernel(t, tau, 1, "pred") <= Bd._gm_kernel(t, tau, 1, "raw") + 1e-9
        assert Bd._gm_kernel(t, tau, 1, "pred") <= Bd._gm_kernel(t, tau, 1, "hold") + 1e-9


@pytest.mark.parametrize("key", ["S1", "S5"])
def test_timestamp_clock_scale(key):
    """1 % clock-scale error alone: e_s ≈ ε (2 Δs − v0 t) (constant speed: ε v0 t = 40 m on S1)."""
    r = Bd.timestamp_check(key)
    assert r["e_s_m"] == pytest.approx(r["predicted_m"], rel=0.02), r


@pytest.mark.slow
def test_budget_matches_analytic_on_s1():
    rep = analytic_check_report()
    for name in ANALYTIC_CFGS:
        r = rep[name]
        assert abs(r["rel_err"]) <= r["tolerance"], (name, r)
