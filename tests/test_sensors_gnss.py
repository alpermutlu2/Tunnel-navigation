"""Tests for the GNSS generator (v0.3, decisions K8, K9, K6 time offset).

Report functions feed tools/snapshot.py (sensors.gnss); every metric has target, achieved and
tolerance where a pass/fail applies.
"""
from dataclasses import fields

import numpy as np
import pytest

from tunnelsim import build_scenarios, generate_truth
from tunnelsim.profiles import GNSS, NOMINAL, Param, configure
from tunnelsim.sensors import GnssTruthSource, Switches, simulate_gnss

SEEDS = range(20)


def open_sky_source(duration_s=3600.0, v=20.0):
    """Straight drive, no tunnel within reach (fixes never stop)."""
    t = np.arange(0.0, duration_s + 1e-9, 0.1)
    pos = np.column_stack([v * t, 0 * t, 0 * t])
    vel = np.tile([v, 0.0, 0.0], (t.size, 1))
    return GnssTruthSource(t=t, pos=pos, vel=vel, s=v * t, s_entry=1e9, s_exit=1e9 + 1)


def _sc(key="S1"):
    sc = build_scenarios()[key]
    return GnssTruthSource.from_truth(generate_truth(sc), sc)


# ---------------------------------------------------------------------------------------------
# Report functions
# ---------------------------------------------------------------------------------------------
def open_sky_report() -> dict:
    """Pooled over 20 seeds x 1 h: realised error std per axis vs model sigma, and the
    normalised squared error (err / reported sigma)^2, whose mean is 1 / ratio^2."""
    src = open_sky_source()
    out = {}
    for case in ("nominal", "gnss_optimistic", "gnss_pessimistic"):
        cfg = NOMINAL if case == "nominal" else configure(case)
        g = cfg.gnss
        ep, ev, nis_p, nis_v = [], [], [], []
        for k in SEEDS:
            m, e = simulate_gnss(src, cfg, Switches.all_on(), seed=k)
            dp, dv = m.pos - e.pos_true, m.vel - e.vel_true
            ep.append(dp), ev.append(dv)
            nis_p.append((dp[:, :2] / m.acc_pos_h[:, None]) ** 2)
            nis_v.append((dv[:, :2] / m.acc_vel[:, None]) ** 2)
        ep, ev = np.concatenate(ep), np.concatenate(ev)
        sig_v = float(np.hypot(float(g.vel_white_h_mps.v), float(g.vel_gm_h_mps.v)))
        ratio = float(g.reported_ratio.v)
        out[case] = {
            "pos_std_h_m": {"target": float(g.pos_sigma_h_m.v),
                            "achieved": round(float(ep[:, :2].std()), 3), "tolerance": 0.05},
            "pos_std_v_m": {"target": float(g.pos_sigma_v_m.v),
                            "achieved": round(float(ep[:, 2].std()), 3), "tolerance": 0.05},
            "vel_std_h_mps": {"target": round(sig_v, 4),
                              "achieved": round(float(ev[:, :2].std()), 4), "tolerance": 0.05},
            "nis_pos_mean": {"target": round(1 / ratio**2, 3),
                             "achieved": round(float(np.concatenate(nis_p).mean()), 3), "tolerance": 0.05},
            "nis_vel_mean": {"target": round(1 / ratio**2, 3),
                             "achieved": round(float(np.concatenate(nis_v).mean()), 3), "tolerance": 0.05},
        }
    return out


def calibration_floor_report() -> dict:
    """K8 motivation: accel-bias floor from a least-squares slope of GNSS velocity over a
    window T, exact p' Sigma p with white + GM velocity noise (horizontal axis)."""
    out = {}
    for case in ("nominal", "gnss_vel_white_only", "gnss_vel_corr_heavy", "gnss_vel_rms_x2"):
        g = (NOMINAL if case == "nominal" else configure(case)).gnss
        f = float(g.rate_hz.v)
        w, s, tau = float(g.vel_white_h_mps.v), float(g.vel_gm_h_mps.v), float(g.vel_gm_tau_s.v)
        rows = {}
        for T in (30.0, 60.0, 120.0):
            n = int(T * f) + 1
            tt = np.arange(n) / f
            p = np.linalg.pinv(np.column_stack([tt, np.ones(n)]))[0]
            lag = np.abs(np.subtract.outer(tt, tt))
            var = w**2 * p @ p + s**2 * p @ np.exp(-lag / tau) @ p
            rows[f"T{T:g}s"] = float(f"{np.sqrt(var):.3e}")
        out[case] = {"sigma_bias_mps2": rows,
                     "vel_rms_h_mps": round(float(np.hypot(w, s)), 4)}
    return out


def outage_report() -> dict:
    src = _sc("S1")
    out = {}
    for case in ("nominal", "gnss_coasting"):
        cfg = NOMINAL if case == "nominal" else configure(case)
        m, e = simulate_gnss(src, cfg, Switches.all_on(), seed=1)
        ev = e.events
        inside = (e.t_true >= ev["t_entry"]) & (e.t_true < ev["t_exit"])
        out[case] = {"fixes_inside_tunnel": int(inside.sum()),
                     "coasting_fixes": int(e.masks["coasting"].sum()),
                     "vel_reacq_after_exit_s": round(ev["t_vel0"] - ev["t_exit"], 3),
                     "pos_reacq_after_exit_s": round(ev["t_pos0"] - ev["t_exit"], 3)}
    return out


def exit_report() -> dict:
    """Exit bias only (all noise off): magnitude at the first position fix and after one decay
    time; multipath direction spread over seeds; lagging along-track sign."""
    src = _sc("S1")
    out = {}
    for case in ("nominal", "gnss_lagging"):
        cfg = NOMINAL if case == "nominal" else configure(case)
        g = cfg.gnss
        first, after, dirs, along = [], [], [], []
        for k in SEEDS:
            m, e = simulate_gnss(src, cfg, Switches.only("gnss_exit"), seed=k)
            ev = e.events
            i0 = int(np.nonzero(e.t_true >= ev["t_pos0"])[0][0])
            i1 = int(np.nonzero(e.t_true >= ev["t_pos0"] + float(g.exit_decay_s.v))[0][0])
            d0 = m.pos[i0] - e.pos_true[i0]
            first.append(np.linalg.norm(d0)), after.append(np.linalg.norm(m.pos[i1] - e.pos_true[i1]))
            dirs.append(d0[:2] / np.linalg.norm(d0[:2]))
            vh = e.vel_true[i0, :2] / np.linalg.norm(e.vel_true[i0, :2])
            along.append(float(d0[:2] @ vh))
        v0 = float(np.linalg.norm(e.vel_true[i0]))
        expect = float(g.exit_multipath_m.v) if g.exit_mode == "multipath" else v0 * float(g.lag_s.v)
        out[case] = {"first_fix_bias_m": {"target": round(expect, 3), "achieved": round(float(np.mean(first)), 3)},
                     "after_one_decay_ratio": {"target": round(float(np.exp(-1)), 3),
                                               "achieved": round(float(np.mean(after) / np.mean(first)), 3)},
                     "mean_direction_norm": round(float(np.linalg.norm(np.mean(dirs, axis=0))), 3),
                     "mean_along_track_m": round(float(np.mean(along)), 3)}
    return out


def accelerating_source(duration_s=600.0):
    """Open sky, speed 20 + 3 sin(2 pi t / 60) m/s along East (|a| up to 0.31 m/s^2)."""
    t = np.arange(0.0, duration_s + 1e-9, 0.01)
    w = 2 * np.pi / 60.0
    v = 20.0 + 3.0 * np.sin(w * t)
    x = 20.0 * t + 3.0 / w * (1 - np.cos(w * t))
    z = np.zeros_like(t)
    return (GnssTruthSource(t=t, pos=np.column_stack([x, z, z]), vel=np.column_stack([v, z, z]), s=x,
                            s_entry=1e9, s_exit=1e9 + 1), t, x, v, 3.0 * w * np.cos(w * t))


def offset_report() -> dict:
    """K6: the fix stamped t describes t - offset. Noise off, accelerating open-sky drive:
    position error = p(t - d) - p(t) ~ -v d, velocity error ~ -a d."""
    src, t, x, v, a = accelerating_source()
    out = {}
    for case in ("gnss_offset_100ms", "gnss_offset_200ms"):
        cfg = configure(case)
        d = float(cfg.timing.imu_gnss_offset_s.v)
        m, _ = simulate_gnss(src, cfg, Switches.only(), seed=0)
        x_st, v_st, a_st = (np.interp(m.t, t, q) for q in (x, v, a))
        out[case] = {"offset_s": d,
                     "max_pos_err_m": round(float(np.abs(m.pos[:, 0] - x_st).max()), 4),
                     "expected_max_v_times_d_m": round(float(np.abs(v_st).max() * d), 4),
                     "max_vel_err_mps": round(float(np.abs(m.vel[:, 0] - v_st).max()), 4),
                     "expected_max_a_times_d_mps": round(float(np.abs(a_st).max() * d), 4)}
    return out


# ---------------------------------------------------------------------------------------------
def test_every_number_is_tagged():
    for f in fields(GNSS):
        val = getattr(GNSS, f.name)
        if not isinstance(val, str):
            assert isinstance(val, Param), f.name


def test_open_sky_statistics_and_reported_accuracy():
    rep = open_sky_report()
    for case, r in rep.items():
        for key, m in r.items():
            assert m["achieved"] == pytest.approx(m["target"], rel=m["tolerance"]), (case, key, m)


def test_no_fix_inside_tunnel_nominal_and_reacquisition_order():
    rep = outage_report()["nominal"]
    assert rep["fixes_inside_tunnel"] == 0
    assert rep["coasting_fixes"] == 0
    assert 0 < rep["vel_reacq_after_exit_s"] < rep["pos_reacq_after_exit_s"]
    m, e = simulate_gnss(_sc("S1"), NOMINAL, Switches.all_on(), seed=2)
    first = e.t_true > e.events["t_exit"]
    k = int(np.nonzero(first)[0][0])                       # velocity comes first, position NaN
    assert np.isfinite(m.vel[k]).all() and np.isnan(m.pos[k]).all()


def test_coasting_propagates_receiver_state_not_truth():
    m, e = simulate_gnss(_sc("S1"), configure("gnss_coasting"), Switches.all_on(), seed=3)
    c = e.masks["coasting"]
    assert c.sum() == int(float(configure("gnss_coasting").gnss.coast_s.v))
    k = int(np.nonzero(c)[0][0]) - 1                       # last real fix
    dt = e.t_true[c] - e.t_true[k]
    np.testing.assert_allclose(m.pos[c], m.pos[k] + m.vel[k] * dt[:, None], atol=1e-9)
    assert np.all(np.diff(m.acc_pos_h[c]) > 0)
    assert np.all(e.t_true[c] >= e.events["t_entry"])


def test_exit_bias_models():
    rep = exit_report()
    mp, lag = rep["nominal"], rep["gnss_lagging"]
    for r in (mp, lag):
        assert r["first_fix_bias_m"]["achieved"] == pytest.approx(r["first_fix_bias_m"]["target"], rel=0.02)
        assert r["after_one_decay_ratio"]["achieved"] == pytest.approx(r["after_one_decay_ratio"]["target"],
                                                                       abs=0.03)
    assert mp["mean_direction_norm"] < 0.5                  # random direction over seeds
    assert lag["mean_direction_norm"] > 0.99                # always the same: behind
    assert lag["mean_along_track_m"] == pytest.approx(-lag["first_fix_bias_m"]["target"], rel=0.02)


def test_time_offset():
    for r in offset_report().values():
        assert r["max_pos_err_m"] == pytest.approx(r["expected_max_v_times_d_m"], rel=0.02)
        assert r["max_vel_err_mps"] == pytest.approx(r["expected_max_a_times_d_mps"], rel=0.02)
        assert r["max_vel_err_mps"] > 0.01


def test_calibration_floor_white_matches_closed_form():
    r = calibration_floor_report()["gnss_vel_white_only"]["sigma_bias_mps2"]
    for T, val in r.items():
        Tn = float(T[1:-1])
        n = int(Tn) + 1
        closed = 0.10 * np.sqrt(12.0 / (n * (n * n - 1)))       # exact discrete LS slope, 1 Hz
        assert val == pytest.approx(closed, rel=1e-3)


def test_switching_gnss_terms_off_does_not_change_others():
    src = _sc("S1")
    _, a = simulate_gnss(src, NOMINAL, Switches.all_on(), seed=4)
    _, b = simulate_gnss(src, NOMINAL, Switches.all_on().without("gnss_vel_gm", "gnss_exit"), seed=4)
    np.testing.assert_array_equal(a.components["vel_white"], b.components["vel_white"])
    np.testing.assert_array_equal(a.components["pos_noise"], b.components["pos_noise"])
