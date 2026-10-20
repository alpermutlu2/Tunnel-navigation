"""Tests for pre-tunnel calibration and reference algorithms (v0.4 step 4)."""
import ast
from pathlib import Path

import numpy as np
import pytest

from tunnelsim.budget import IMU_TERMS
from tunnelsim.ins import ImuCalibration, InsInit, strapdown
from tunnelsim.profiles import configure
from tunnelsim.reference import gm_conditional_shape, ls_accel_bias
from tunnelsim.scenario import build_scenarios
from tunnelsim.sensors import GnssTruthSource, ImuTruthSource, Switches, simulate_gnss, simulate_imu
from tunnelsim.truth import generate_truth

ROOT = Path(__file__).resolve().parents[1]


def test_reference_imports_nothing_from_truth_side():
    tree = ast.parse((ROOT / "tunnelsim" / "reference.py").read_text())
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert mods <= {"__future__", "numpy", "frames", "meas", "ins"}, mods


def test_ls_recovers_turn_on_bias_with_maneuvers():
    """Only the accel turn-on bias on, error-free GNSS velocity: LS over the approach recovers
    the phone-frame bias (the maneuvers make it identifiable with v0)."""
    sc = build_scenarios()["S1"]
    tr = generate_truth(sc)
    cfg = configure("no_gaps")
    m, e = simulate_imu(ImuTruthSource.from_truth(tr), cfg, Switches.only("a_turnon"), seed=3)
    g, _ = simulate_gnss(GnssTruthSource.from_truth(tr, sc), cfg, Switches.only(), seed=3)
    b, _ = ls_accel_bias(m.t, m.f, tr.C_nb @ tr.C_bp, g.t, g.vel, 10.0, 190.0)
    np.testing.assert_allclose(b, e.constants["b_a_on"], atol=2e-4)


def test_conditional_shape():
    t = np.array([0.0, 10.0, 1e4])
    k = gm_conditional_shape(t, [(1.0, 1.0)] * 3, [(5.0, 50.0)] * 3)
    np.testing.assert_allclose(k[0], 1.0)
    assert np.all(np.diff(k[:, 0]) < 0) and k[-1, 0] < 1e-6
    kc = gm_conditional_shape(t, [(1.0, 1.0)] * 3, [(5.0, 50.0)] * 3, const_var=98.0)
    np.testing.assert_allclose(kc[-1], 0.98, atol=1e-6)       # constant part never decays


def test_zero_velocity_mask_resets_speed():
    t = np.arange(0, 20.0, 0.01)
    f = np.tile([0.1, 0.0, 9.80665], (t.size, 1))              # spurious forward acceleration
    z = (t > 10.0)
    r = strapdown(t, f, np.zeros_like(f), InsInit(np.zeros(3), np.zeros(3), np.eye(3)),
                  zero_velocity=z)
    assert np.all(r.v_n[0, z] == 0.0)
    assert r.p_n[0, -1, 0] == pytest.approx(0.5 * 0.1 * 10.0**2, rel=1e-2)


def test_bias_shape_applied_inside_ins():
    t = np.arange(0, 10.0, 0.01)
    b = np.array([0.2, 0.0, 0.0])
    f = np.tile([0.0, 0.0, 9.80665], (t.size, 1)) + b
    shape = np.ones(t.size)
    r = strapdown(t, f, np.zeros_like(f), InsInit(np.zeros(3), np.zeros(3), np.eye(3)),
                  ImuCalibration(accel_bias=b, accel_bias_shape=shape))
    assert np.abs(r.p_n[0, -1]).max() < 1e-9
