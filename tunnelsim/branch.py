"""Exit-branch confirmation (review 6, secondary feature). Estimator side: uses only the gyro,
the filter's attitude (for the vertical axis), the filter's position and the map; no truth.

From a window start ŝ0 = s_div − pre_m the measured heading change Δψ̂ = ∫ e_upᵀ C_np ω dt is
compared with each map candidate j (main continuation, branch):
    r_j(ŝ) = Δψ̂ − Δψ_map,j(ŝ)
Reference detector [ASSUMED]: confirm the branch at the first instant with |r_main| > 5° and
|r_branch| < |r_main|. The map heading profile of the branch is the route's; the main tunnel
continues straight (Exit.dpsi_main = 0 in every scenario).
"""
from __future__ import annotations

import numpy as np

CONFIRM_DEG = 5.0      # [ASSUMED] reference threshold (review 6: later normalise by the yaw sigma)
PRE_M = 100.0          # [ASSUMED] window starts this far before the diverge on the map


def map_s(road, p_xy, s_lo, s_hi, step=1.0):
    """Along-track position of horizontal points p_xy (..., 2) by nearest centreline point in
    [s_lo, s_hi] (map matching to the route)."""
    s = np.arange(s_lo, s_hi, step)
    c = road.position(s)[:, :2]
    d2 = ((p_xy[..., None, :] - c) ** 2).sum(-1)
    return s[np.argmin(d2, axis=-1)]


def branch_check(imu_t, w, out_t, out_C, out_p, out_v, road, thr_deg=CONFIRM_DEG, pre_m=PRE_M):
    """Per seed: time of the first decision (NaN if none) and the decision (+1 branch, −1 main,
    0 none). Branch: |r_main| > thr and |r_branch| < |r_main|; main: the mirror condition.
    w (B, N, 3) phone gyro; out_* filter output at its own rate; road = the map (route with the
    branch)."""
    ex = road.exits[0]
    B = out_p.shape[0]
    s_hat = np.stack([map_s(road, out_p[b, :, :2], ex.s - 3000.0, ex.s + 1500.0, 2.0) for b in range(B)])
    t_conf = np.full(B, np.nan)
    choice = np.zeros(B, int)
    thr = np.deg2rad(thr_deg)
    psi0_map = road.heading(ex.s - pre_m)
    for b in range(B):
        k0 = int(np.argmax(s_hat[b] >= ex.s - pre_m))
        if s_hat[b, k0] < ex.s - pre_m:
            continue
        # ŝ after the window start: map-matched start + integrated horizontal speed (no jumps)
        sh = s_hat[b, k0] + np.concatenate([[0.0], np.cumsum(np.linalg.norm(out_v[b, k0 + 1:, :2], axis=1) * np.diff(out_t[k0:]))])
        i0 = int(np.searchsorted(imu_t, out_t[k0]))
        kk = np.clip(np.searchsorted(out_t, imu_t[i0:]) - 1, 0, len(out_t) - 1)
        w_up = np.einsum("ki,ki->k", out_C[b, kk, 2, :], w[b, i0:])
        dpsi = np.concatenate([[0.0], np.cumsum(0.5 * (w_up[1:] + w_up[:-1]) * np.diff(imu_t[i0:]))])
        dpsi_k = np.interp(out_t[k0:], imu_t[i0:], dpsi)
        r_branch = dpsi_k - (road.heading(sh) - psi0_map)
        r_main = dpsi_k - ex.dpsi_main * np.clip((sh - ex.s) / max(ex.curve_len, 1e-9), 0.0, 1.0)
        hit_b = (np.abs(r_main) > thr) & (np.abs(r_branch) < np.abs(r_main))
        hit_m = (np.abs(r_branch) > thr) & (np.abs(r_main) < np.abs(r_branch))
        first = [int(np.argmax(h)) if h.any() else None for h in (hit_b, hit_m)]
        if first[0] is not None and (first[1] is None or first[0] <= first[1]):
            t_conf[b], choice[b] = out_t[k0 + first[0]], 1
        elif first[1] is not None:
            t_conf[b], choice[b] = out_t[k0 + first[1]], -1
    return t_conf, choice
