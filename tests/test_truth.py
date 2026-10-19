"""Tests for the truth generator.

Two kinds of test, kept apart on purpose:

* Internal consistency: the ideal phone IMU signals, integrated by a strapdown mechanisation,
  must reproduce the phone trajectory. This proves the specific force / angular rate formulas
  agree with the kinematics, but it uses the same model on both sides.
* Independent references: road geometry against closed forms (circle, Fresnel clothoid) and
  against a high-accuracy adaptive integrator, so the centreline discretisation error is
  reported on its own and not mixed into navigation error.
"""
import numpy as np
import pytest
from scipy.integrate import quad, solve_ivp
from scipy.special import fresnel

from tunnelsim.frames import GRAVITY_N, so3_exp
from tunnelsim.scenario import (Exit, HSeg, Road, Scenario, SpeedEvent, SpeedProfile, Tunnel,
                                Vehicle, build_scenarios, kmh)
from tunnelsim.truth import generate_truth

SCENARIOS = build_scenarios()


def mixed_scenario() -> Scenario:
    """Braking while entering a clothoid and while the grade changes: everything at once."""
    R = 200.0
    road = Road(
        hsegs=[HSeg(300), HSeg(80, 0.0, 1 / R), HSeg(250, 1 / R), HSeg(80, 1 / R, 0.0), HSeg(500)],
        grade_points=[(250.0, 0.0), (550.0, 4.0), (900.0, -2.0)],
        tunnel=Tunnel(200.0, 1100.0),
        exits=[Exit(700.0, "X")],
    )
    speed = SpeedProfile(v0=kmh(80), events=[
        SpeedEvent(280.0, kmh(40), 8.0, dwell=5.0),
        SpeedEvent(280.0, kmh(70), 12.0),
    ])
    return Scenario("MIX", "mixed", "test only", road, speed)


ALL = dict(SCENARIOS, MIX=mixed_scenario())


def strapdown(tr):
    """Second-order strapdown in ENU, flat earth, using phone-frame signals and phone states."""
    n = len(tr.t)
    C = tr.C_nb[0] @ tr.C_bp           # C_np
    vel = tr.vel_phone_n[0].copy()
    pos = tr.pos_phone_n[0].copy()
    P = np.empty((n, 3)); P[0] = pos
    dt = tr.dt
    for k in range(n - 1):
        w_mid = 0.5 * (tr.w_p[k] + tr.w_p[k + 1])
        C_next = C @ so3_exp(w_mid * dt)
        f_n = 0.5 * (C @ tr.f_p[k] + C_next @ tr.f_p[k + 1])
        vel_next = vel + (f_n + GRAVITY_N) * dt
        pos = pos + 0.5 * (vel + vel_next) * dt
        vel, C = vel_next, C_next
        P[k + 1] = pos
    return P


# ----------------------------------------------------------------- internal consistency

@pytest.mark.parametrize("key", list(ALL))
def test_strapdown_reproduces_phone_trajectory(key):
    tr = generate_truth(ALL[key])
    err = np.linalg.norm(strapdown(tr) - tr.pos_phone_n, axis=1)
    assert err.max() < 0.05, f"{key}: strapdown drift {err.max():.3f} m"


@pytest.mark.parametrize("key", list(ALL))
def test_derivatives_consistent(key):
    tr = generate_truth(ALL[key])
    for pos, vel, acc in [(tr.pos_n, tr.vel_n, tr.acc_n),
                          (tr.pos_phone_n, tr.vel_phone_n, tr.acc_phone_n)]:
        dpos = np.gradient(pos, tr.dt, axis=0)
        dvel = np.gradient(vel, tr.dt, axis=0)
        assert np.abs(dpos - vel)[5:-5].max() < 0.05
        assert np.abs(dvel - acc)[5:-5].max() < 0.05


# ----------------------------------------------------------------- physics spot checks

def test_flat_constant_speed_specific_force():
    tr = generate_truth(SCENARIOS["S1"])
    assert np.allclose(tr.f_b, [0, 0, 9.80665], atol=1e-9)
    assert np.allclose(tr.w_b, 0, atol=1e-12)


def test_grade_gravity_projection():
    tr = generate_truth(SCENARIOS["S3"])
    k = np.argmin(np.abs(tr.s - (SCENARIOS["S3"].road.tunnel.s_entry + 600.0)))   # mid-descent, constant -5 %, constant speed
    g = np.arctan(-0.05)
    assert tr.f_b[k, 0] == pytest.approx(9.80665 * np.sin(g), abs=1e-6)


def test_steady_curve_with_sideslip():
    sc = SCENARIOS["S2"]
    tr = generate_truth(sc)
    k = np.argmin(np.abs(tr.s - (sc.road.tunnel.s_entry + 500 + 80 + 175)))   # middle of the left arc, R = 400
    v, R = tr.v[k], 400.0
    a_lat = v**2 / R
    beta = -sc.vehicle.sideslip_gain_rad_per_mps2 * a_lat
    assert tr.beta[k] == pytest.approx(beta, rel=1e-6)
    assert tr.w_b[k, 2] == pytest.approx(v / R, rel=1e-6)
    # body is yawed by -beta into the turn: lateral accel leaks into the forward axis
    assert tr.f_b_ref[k, 1] == pytest.approx(a_lat * np.cos(beta), rel=1e-6)
    assert tr.f_b_ref[k, 0] == pytest.approx(a_lat * np.sin(-beta), rel=1e-6)


def test_nhc_violated_at_phone_in_curve():
    """Truth must not satisfy NHC perfectly (review point): rear-axle sideslip plus the
    lever arm give a lateral body velocity at the phone in curves."""
    tr = generate_truth(SCENARIOS["S2"])
    C_bn = np.transpose(tr.C_nb, (0, 2, 1))
    v_b_phone = np.einsum("nij,nj->ni", C_bn, tr.vel_phone_n)
    k = np.argmin(np.abs(tr.s - (SCENARIOS["S2"].road.tunnel.s_entry + 500 + 80 + 175)))
    expected = tr.v[k] * np.sin(tr.beta[k]) + tr.w_b[k, 2] * tr.lever_arm_b[0]
    assert v_b_phone[k, 1] == pytest.approx(expected, rel=1e-3)
    assert abs(v_b_phone[k, 1]) > 0.05


def test_stops_reach_zero_speed():
    tr = generate_truth(SCENARIOS["S4"])
    assert tr.v.min() == pytest.approx(0.0, abs=1e-9)
    assert (tr.v >= -1e-9).all()


# ----------------------------------------------------------------- boundaries and events

@pytest.mark.parametrize("key", list(ALL))
def test_start_and_end_boundaries(key):
    sc = ALL[key]
    tr = generate_truth(sc)
    assert tr.t[0] == 0.0 and tr.s[0] == 0.0
    assert tr.v[0] == pytest.approx(sc.speed.v0)
    assert tr.a[0] == 0.0
    assert sc.road.length - 1.0 <= tr.s[-1] <= sc.road.length + tr.v[-1] * tr.dt
    assert np.all(np.diff(tr.s) >= 0)


@pytest.mark.parametrize("key", list(ALL))
def test_event_trigger_position(key):
    """Each ramp starts within one sample of its trigger point (or later only because of
    the dwell lock of the previous event)."""
    sc = ALL[key]
    tr = generate_truth(sc)
    assert len(tr.event_log) == len(sc.speed.events)
    for idx, t0, s_pred in tr.event_log:
        ev = sc.speed.events[idx]
        k = int(round(t0 / tr.dt))
        assert s_pred >= ev.at_s
        assert abs(tr.s[k] - s_pred) < 1e-3          # prediction matches the realised position
        prev_end = 0.0
        if idx > 0:
            p_idx, p_t0, _ = tr.event_log[idx - 1]
            p = sc.speed.events[p_idx]
            prev_end = p_t0 + p.duration + p.dwell
        if t0 > prev_end + tr.dt:                    # not held back by a dwell: exact trigger
            assert s_pred - ev.at_s <= tr.v[k - 1] * tr.dt + 1e-9


def test_ramp_reaches_target_and_is_smooth():
    tr = generate_truth(ALL["MIX"])
    assert np.abs(np.diff(tr.a)).max() < 0.05      # no acceleration steps (quintic ramp)
    assert tr.v[-1] == pytest.approx(kmh(70))


# ----------------------------------------------------------------- independent geometry

def test_circle_closed_form():
    R = 300.0
    road = Road([HSeg(800.0, 1 / R)], [(0.0, 0.0)], Tunnel(0, 800))
    s = np.linspace(0, 800, 401)
    p = road.position(s)
    ref = np.column_stack([R * np.sin(s / R), R * (1 - np.cos(s / R))])
    assert np.abs(p[:, :2] - ref).max() < 1e-3


def test_clothoid_fresnel():
    L, R = 200.0, 250.0
    road = Road([HSeg(L, 0.0, 1 / R)], [(0.0, 0.0)], Tunnel(0, L))
    a = np.sqrt(np.pi * L * R)
    s = np.linspace(0, L, 201)
    S, C = fresnel(s / a)
    p = road.position(s)
    assert np.abs(p[:, 0] - a * C).max() < 1e-3
    assert np.abs(p[:, 1] - a * S).max() < 1e-3


def test_altitude_against_quad():
    road = SCENARIOS["S3"].road
    s_e = road.tunnel.s_entry
    for s_end in [s_e, s_e + 1200.0, s_e + 2200.0, s_e + 3500.0]:
        ref, _ = quad(lambda x: np.sin(road.grade(x)[0]), 0, s_end, limit=400, epsabs=1e-10)
        assert road.position(s_end)[2] == pytest.approx(ref, abs=1e-3)


def centreline_error(road: Road) -> float:
    """Max distance between the 0.25 m tabulated centreline and an adaptive RK solution."""
    def rhs(s, y):
        g = road.grade(s)[0]
        psi = y[3]
        return [np.cos(g) * np.cos(psi), np.cos(g) * np.sin(psi), np.sin(g), road.curvature(s)[0]]
    s_eval = np.linspace(0, road.length, 2001)
    sol = solve_ivp(rhs, (0, road.length), [0, 0, 0, 0], t_eval=s_eval, rtol=1e-10, atol=1e-9,
                    max_step=5.0)
    return float(np.linalg.norm(road.position(s_eval) - sol.y[:3].T, axis=1).max())


@pytest.mark.parametrize("key", ["S2", "S3", "MIX"])
def test_centreline_discretisation_error(key):
    err = centreline_error(ALL[key].road)
    print(f"{key}: centreline discretisation error {err * 1000:.3f} mm")
    assert err < 0.01
