"""Truth generator: vehicle motion along the road and ideal (error-free) inertial signals.

Model
-----
The vehicle reference point R (rear-axle centre) follows the road centreline. With heading
psi(s), grade gamma(s), horizontal curvature kappa = dpsi/ds and vertical curvature
gamma' = dgamma/ds:

    t_n   = [cos g cos psi, cos g sin psi, sin g]
    v_n   = v t_n
    a_n   = a t_n + v^2 (kappa dt/dpsi + gamma' dt/dgamma)

Attitude chain, nav <- road <- (sideslip yaw) <- (suspension pitch) = body:

    C_nb = Rz(psi) Ry(-gamma) . Rz(-beta) . Ry(-theta_d)
    beta    = -k_beta * a_lat,   a_lat = v^2 kappa      (beta: angle from body x to velocity)
    theta_d =  K * a

Body angular rate (A = Rz(-beta), B = Ry(-theta_d)):

    w_r = psi_dot [sin g, 0, cos g] - gamma_dot [0, 1, 0]
    w_b = (A B)^T w_r + B^T [0, 0, -beta_dot] + [0, -theta_d_dot, 0]

The phone P sits at lever arm r_b from R:

    p_P = p_R + C_nb r_b
    v_P = v_R + C_nb (w_b x r_b)
    a_P = a_R + C_nb (w_b_dot x r_b + w_b x (w_b x r_b))

Ideal IMU outputs (flat earth, earth rotation neglected):

    f_b = C_bn (a_P - g_n),    w_b as above
    f_p = C_pb f_b,            w_p = C_pb w_b       (C_bp from the phone mount)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .frames import GRAVITY_N, mount_rotation, rot_y, rot_z
from .scenario import Scenario, quintic_ramp


@dataclass
class Truth:
    t: np.ndarray            # (N,)   s
    s: np.ndarray            # (N,)   m along road (reference point)
    v: np.ndarray            # (N,)   m/s
    a: np.ndarray            # (N,)   m/s^2 along track
    jerk: np.ndarray         # (N,)   m/s^3
    pos_n: np.ndarray        # (N,3)  ENU m, reference point (rear axle)
    vel_n: np.ndarray        # (N,3)
    acc_n: np.ndarray        # (N,3)
    pos_phone_n: np.ndarray  # (N,3)  ENU m, phone
    vel_phone_n: np.ndarray  # (N,3)
    acc_phone_n: np.ndarray  # (N,3)
    psi: np.ndarray          # (N,)   path heading rad
    gamma: np.ndarray        # (N,)   road grade rad
    beta: np.ndarray         # (N,)   body sideslip rad
    theta_d: np.ndarray      # (N,)   suspension pitch rad
    C_nb: np.ndarray         # (N,3,3)
    f_b_ref: np.ndarray      # (N,3)  specific force at the reference point, body axes
    f_b: np.ndarray          # (N,3)  specific force at the phone, body axes
    w_b: np.ndarray          # (N,3)  angular rate, body FLU
    f_p: np.ndarray          # (N,3)  specific force, phone frame
    w_p: np.ndarray          # (N,3)  angular rate, phone frame
    C_bp: np.ndarray         # (3,3)  phone mount
    lever_arm_b: np.ndarray  # (3,)
    in_tunnel: np.ndarray    # (N,)   bool
    dt: float
    event_log: list = field(default_factory=list)   # (event index, t_start, s_at_start)


# --------------------------------------------------------------------------- speed

def simulate_speed(sc: Scenario, dt: float, t_max: float = 3600.0):
    """Step the event-driven speed profile until the car reaches the end of the road.

    Events are tested against the position predicted for the *current* sample
    (s_prev + v_prev dt), so a ramp starts within one sample after the trigger point.
    Returns t, s, v, a, jerk (a and jerk exact for quintic ramps, s by trapezoidal rule)
    and the trigger log.
    """
    road, prof = sc.road, sc.speed
    events = list(prof.events)
    n_max = int(t_max / dt) + 1

    t = np.empty(n_max); s = np.empty(n_max); v = np.empty(n_max)
    a = np.empty(n_max); j = np.empty(n_max)
    log = []

    v_base = prof.v0
    ramp = None          # (t_start, v_from, dv, duration)
    blocked_until = 0.0  # end of current ramp + dwell
    ev_i = 0
    s_prev, v_prev = 0.0, prof.v0

    k = 0
    while k < n_max:
        tk = k * dt
        s_pred = s_prev + v_prev * dt if k > 0 else 0.0
        if ramp is None and ev_i < len(events) and tk >= blocked_until \
                and s_pred >= events[ev_i].at_s:
            ev = events[ev_i]
            ramp = (tk, v_base, ev.v_target - v_base, max(ev.duration, dt))
            blocked_until = tk + ev.duration + ev.dwell
            log.append((ev_i, tk, s_pred))
            ev_i += 1

        if ramp is not None:
            t0, v_from, dv, T = ramp
            u = (tk - t0) / T
            f, df, d2f = quintic_ramp(u)
            vk = v_from + dv * f
            ak = dv / T * df if u < 1.0 else 0.0
            jk = dv / T**2 * d2f if u < 1.0 else 0.0
            if u >= 1.0:
                v_base = v_from + dv
                ramp = None
        else:
            vk, ak, jk = v_base, 0.0, 0.0

        sk = s_prev + 0.5 * (v_prev + vk) * dt if k > 0 else 0.0
        t[k], s[k], v[k], a[k], j[k] = tk, sk, vk, ak, jk
        s_prev, v_prev = sk, vk
        k += 1
        if sk >= road.length - 1.0:
            break

    sl = slice(0, k)
    return t[sl], s[sl], v[sl], a[sl], j[sl], log


# --------------------------------------------------------------------------- truth

def _cross(a, b):
    return np.cross(a, b)


def generate_truth(sc: Scenario, rate_hz: float = 100.0) -> Truth:
    dt = 1.0 / rate_hz
    t, s, v, a, jerk, log = simulate_speed(sc, dt)
    road, veh = sc.road, sc.vehicle

    psi = road.heading(s)
    g = road.grade(s)
    kap = road.curvature(s)
    kap_s = road.curvature_rate(s)
    gp = road.grade_rate(s)

    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(psi), np.sin(psi)
    t_n = np.column_stack([cg * cp, cg * sp, sg])
    dt_dpsi = np.column_stack([-cg * sp, cg * cp, np.zeros_like(g)])
    dt_dg = np.column_stack([-sg * cp, -sg * sp, cg])

    # reference point kinematics
    pos_n = road.position(s)
    vel_n = v[:, None] * t_n
    acc_n = a[:, None] * t_n + (v**2)[:, None] * (kap[:, None] * dt_dpsi + gp[:, None] * dt_dg)

    # attitude
    a_lat = v**2 * kap
    a_lat_dot = 2 * v * a * kap + v**3 * kap_s
    beta = -veh.sideslip_gain_rad_per_mps2 * a_lat
    beta_dot = -veh.sideslip_gain_rad_per_mps2 * a_lat_dot
    theta_d = veh.pitch_gain_rad_per_mps2 * a
    theta_d_dot = veh.pitch_gain_rad_per_mps2 * jerk

    C_nr = rot_z(psi) @ rot_y(-g)
    A = rot_z(-beta)
    B = rot_y(-theta_d)
    C_rb = A @ B
    C_nb = C_nr @ C_rb

    # angular rate
    psi_dot = v * kap
    g_dot = v * gp
    w_r = np.column_stack([psi_dot * sg, -g_dot, psi_dot * cg])
    C_br = np.transpose(C_rb, (0, 2, 1))
    B_T = np.transpose(B, (0, 2, 1))
    w_b = np.einsum("nij,nj->ni", C_br, w_r)
    w_b += np.einsum("nij,nj->ni", B_T, np.column_stack([0 * beta, 0 * beta, -beta_dot]))
    w_b[:, 1] -= theta_d_dot

    # phone lever arm
    r_b = np.asarray(veh.lever_arm_b, dtype=float)
    w_b_dot = np.gradient(w_b, dt, axis=0)
    wxr = _cross(w_b, r_b)
    acc_rot_b = _cross(w_b_dot, r_b) + _cross(w_b, wxr)
    pos_phone_n = pos_n + np.einsum("nij,j->ni", C_nb, r_b)
    vel_phone_n = vel_n + np.einsum("nij,nj->ni", C_nb, wxr)
    acc_phone_n = acc_n + np.einsum("nij,nj->ni", C_nb, acc_rot_b)

    # specific force
    C_bn = np.transpose(C_nb, (0, 2, 1))
    f_b_ref = np.einsum("nij,nj->ni", C_bn, acc_n - GRAVITY_N)
    f_b = np.einsum("nij,nj->ni", C_bn, acc_phone_n - GRAVITY_N)

    # phone frame
    C_bp = mount_rotation(*veh.mount_rpy_deg)
    C_pb = C_bp.T
    f_p = f_b @ C_pb.T
    w_p = w_b @ C_pb.T

    return Truth(t=t, s=s, v=v, a=a, jerk=jerk, pos_n=pos_n, vel_n=vel_n, acc_n=acc_n,
                 pos_phone_n=pos_phone_n, vel_phone_n=vel_phone_n, acc_phone_n=acc_phone_n,
                 psi=psi, gamma=g, beta=beta, theta_d=theta_d, C_nb=C_nb,
                 f_b_ref=f_b_ref, f_b=f_b, w_b=w_b, f_p=f_p, w_p=w_p, C_bp=C_bp,
                 lever_arm_b=r_b, in_tunnel=road.in_tunnel(s), dt=dt, event_log=log)
