"""Reference algorithms and pre-tunnel calibration (v0.4 step 4, focus decision 2026-10-20).

Estimator side: imports only numpy, frames, meas and ins (docs/contracts.md 6). An attitude
sequence may be passed in (ideal-attitude experiments); the module does not know its origin.

Pre-tunnel accel-bias calibration from GNSS velocity (window [t_ref - T, t_ref]):

    v_k = v0 + I_k - J_k b + e_k,   I_k = ∫ (C f~ + g) dt,   J_k = ∫ C dt     (unknowns v0, b)

solved by unweighted least squares; b is the phone-frame accel bias as the GNSS sees it
(turn-on + bias instability + the constant part of scale/misalignment through gravity).

In-tunnel bias propagation: "hold" (b̂ constant), "condgm" (b̂ · Σσ²e^(-t/τ)/Σσ², the conditional
mean given only the total bias, per axis, from the ASSUMED sensor model), "none".
"""
from __future__ import annotations

import numpy as np

from .frames import GRAVITY_N
from .ins import ImuCalibration, InsInit, strapdown


def cumtrapz(y, t):
    """Cumulative trapezoid along axis 0 (y (N, ...), t (N,))."""
    d = 0.5 * (y[1:] + y[:-1]) * np.diff(t).reshape((-1,) + (1,) * (y.ndim - 1))
    return np.concatenate([np.zeros((1,) + y.shape[1:]), np.cumsum(d, axis=0)])


def ls_accel_bias(t_imu, f, C_np, t_gnss, v_gnss, t_start, t_end):
    """Least-squares accel bias (phone frame) and v0 from GNSS velocity over [t_start, t_end]."""
    m = (t_imu >= t_start) & (t_imu <= t_end)
    ti, fi, Ci = t_imu[m], f[m], C_np[m]
    a_n = np.einsum("nij,nj->ni", Ci, fi) + GRAVITY_N
    I = cumtrapz(a_n, ti)
    J = cumtrapz(Ci, ti)
    g = (t_gnss >= ti[0]) & (t_gnss <= ti[-1]) & np.all(np.isfinite(v_gnss), axis=1)
    tk, vk = t_gnss[g], v_gnss[g]
    Ik = np.stack([np.interp(tk, ti, I[:, j]) for j in range(3)], 1)
    Jk = np.stack([[np.interp(tk, ti, J[:, i, j]) for j in range(3)] for i in range(3)]).transpose(2, 0, 1)
    A = np.zeros((3 * len(tk), 6))
    y = np.zeros(3 * len(tk))
    for k in range(len(tk)):
        A[3 * k:3 * k + 3, :3] = np.eye(3)
        A[3 * k:3 * k + 3, 3:] = -Jk[k]
        y[3 * k:3 * k + 3] = vk[k] - Ik[k]
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    return x[3:], x[:3]                       # b (phone frame), v at t_start


def gm_conditional_shape(t, sigmas_per_axis, taus_per_axis, const_var=0.0):
    """Conditional mean of the bias given only its total value at t = 0, per axis:

        k_i(t) = (σ_c² + Σ_j σ_ij² e^(-t/τ_ij)) / (σ_c² + Σ_j σ_ij²)

    σ_c² = variance of the constant part the GNSS calibration also sees (turn-on bias, scale and
    misalignment through gravity). Without it (review 5's form) the whole estimate decays,
    which is wrong when the constant part dominates. Shape (N, 3)."""
    out = np.empty((len(t), 3))
    cv = np.broadcast_to(np.asarray(const_var, dtype=float), (3,))
    for i in range(3):
        s2 = np.asarray(sigmas_per_axis[i]) ** 2
        num = cv[i] + (s2[None, :] * np.exp(-np.asarray(t)[:, None] / np.asarray(taus_per_axis[i])[None, :])).sum(1)
        out[:, i] = num / (cv[i] + s2.sum())
    return out


def gnss_entry_state(t_gnss, p_gnss, v_gnss, t_ref, avg_s=10.0):
    """Position at the last fix at or before t_ref, velocity averaged over [t_ref - avg_s, t_ref]."""
    ok = np.isfinite(p_gnss[:, 0]) & (t_gnss <= t_ref + 1e-9)
    k = np.nonzero(ok)[0][-1]
    w = np.isfinite(v_gnss[:, 0]) & (t_gnss >= t_gnss[k] - avg_s) & (t_gnss <= t_gnss[k] + 1e-9)
    return float(t_gnss[k]), p_gnss[k], v_gnss[w].mean(0)


def ins_from_entry(t_imu, f, w, C_np_seq, t0, p0, v0, b_hat, mode, shape=None):
    """INS (ideal attitude if C_np_seq is given) from t0 with the calibrated bias propagated by
    `mode` ('hold' | 'condgm' | 'none'). Batch over leading dim of f/b_hat/p0/v0."""
    i0 = int(np.searchsorted(t_imu, t0 - 1e-9))
    t, fs = t_imu[i0:], f[..., i0:, :]
    att = None if C_np_seq is None else C_np_seq[i0:]
    cal = ImuCalibration()
    if mode == "hold":
        cal = ImuCalibration(accel_bias=b_hat)
    elif mode == "condgm":
        cal = ImuCalibration(accel_bias=b_hat, accel_bias_shape=shape(t - t[0]))
    C0 = C_np_seq[i0] if C_np_seq is not None else None
    return strapdown(t, fs, w[..., i0:, :], InsInit(p0, v0, C0), cal, attitude=att), i0
