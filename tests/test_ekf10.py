"""Tests for the 10-state map-constrained filter (v0.5.11, owner 2026-10-25)."""
import ast
from pathlib import Path

import numpy as np
import pytest

from tunnelsim.experiments import ekf10_experiment, eskf_experiment, simulate_run

ROOT = Path(__file__).resolve().parents[1]
SEEDS = np.arange(5)


def test_ekf10_imports_nothing_from_truth_side():
    tree = ast.parse((ROOT / "tunnelsim" / "ekf10.py").read_text())
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert mods <= {"__future__", "dataclasses", "numpy", "frames", "eskf"}, mods


@pytest.fixture(scope="module")
def s1():
    return simulate_run("S1", SEEDS)


def test_s1_diverge_matches_eskf(s1):
    r = ekf10_experiment("S1", SEEDS, sim=s1)
    re = eskf_experiment("S1", SEEDS)
    assert abs(r["at_diverge"]["rmse"] - re["at_diverge"]["rmse"]) < 3.0


def test_s1_consistent_and_mount_insensitive(s1):
    r = ekf10_experiment("S1", SEEDS, sim=s1)
    assert r["consistency_real_exit"]["mean_norm_sq"] < 2.0
    rm = ekf10_experiment("S1", SEEDS, sim=s1, mount_err_deg=(0.0, 2.0, 0.0))
    assert abs(rm["real_exit"]["rmse"] - r["real_exit"]["rmse"]) < 1.0


def test_s3_without_map_grade_holds():
    """Pitch is a state held by the barometer, so a V profile needs no grade in the map (the 1D
    filter is off by kilometres there)."""
    r = ekf10_experiment("S3", SEEDS)
    assert r["real_exit"]["rmse"] < 15.0
