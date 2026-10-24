"""Tests for the 1D map-constrained EKF baseline (v0.5.9)."""
import ast
from pathlib import Path

import numpy as np
import pytest

from tunnelsim.experiments import ekf1d_experiment, eskf_experiment, simulate_run

ROOT = Path(__file__).resolve().parents[1]
SEEDS = np.arange(5)


def test_reference1d_imports_nothing_from_truth_side():
    tree = ast.parse((ROOT / "tunnelsim" / "reference1d.py").read_text())
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert mods <= {"__future__", "dataclasses", "numpy", "frames", "eskf"}, mods


@pytest.fixture(scope="module")
def s1():
    return simulate_run("S1", SEEDS)


def test_s1_diverge_matches_eskf(s1):
    """Straight, level tunnel, known mount: both filters reduce to an INS on the GNSS-calibrated
    bias, so the along-track error at the diverge agrees within 3 m (same seeds)."""
    r1 = ekf1d_experiment("S1", SEEDS, sim=s1)
    re = eskf_experiment("S1", SEEDS)
    assert abs(r1["at_diverge"]["rmse"] - re["at_diverge"]["rmse"]) < 3.0


def test_s3_without_map_grade_breaks():
    """V profile ±5 % with a map that has no grade: gravity is read as acceleration, > 1 km."""
    r = ekf1d_experiment("S3", SEEDS, use_grade=False)
    assert r["real_exit"]["rmse"] > 1000.0
    r = ekf1d_experiment("S3", SEEDS, use_grade=True)
    assert r["real_exit"]["rmse"] < 20.0


def test_s4_overconfident_from_suspension_pitch():
    """Stop-and-go: launch squat and brake dive are read as forward acceleration, which the
    model has no term for, so the filter's σ does not cover its error."""
    r = ekf1d_experiment("S4", SEEDS)
    assert r["consistency_real_exit"]["mean_norm_sq"] > 5.0
