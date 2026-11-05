"""Where the app draws the car near the diverge (v0.5.15, owner 2026-11-05, K1). Estimator side.

The supervisor's shown track is an along-track position ŝ on the route. Past the diverge the route is
the branch, and the shown filter's branch decision may still be "not yet". Policies:

  "route"       (v0.5.14) the route polyline until "main" is decided
  "main_first"  the main tunnel until "branch" is decided (owner's choice, K1)

When the decision arrives the point moves from one road to the other. That move is rate-limited like the
exit follower (supervisor.HANDOVER_RATE, 2 m/s): the shown point is main + w·(route − main) at the same ŝ,
and its gap to the target road closes at 2 m/s (owner: the follower's limit across as well as along).

Product assumption [ASSUMED]: while the decision is "not yet" the app does not re-route, so a car shown on
the main tunnel is not declared off the route. Not simulated.
"""
from __future__ import annotations

import numpy as np

from .supervisor import HANDOVER_RATE as LAT_RATE     # m/s, the exit follower's rate, used across too


def shown_xy(mv, t, s_disp, state, policy="main_first", lat_rate=LAT_RATE):
    """Shown position (B,K,2) and the weight w on the route (B,K). t (K,), s_disp (B,K) route arc length,
    state (B,K): +1 branch, −1 main, 0 not yet. lat_rate None: no rate limit (the step as it falls).

    The limit is on the gap to the target road, in metres: when the target changes, the gap is the shown
    point's distance to the new target at that ŝ, and it closes at lat_rate. Meanwhile the point moves
    with the roads (they part as the car drives on), so it never lags a diverging branch for long."""
    B, K = s_disp.shape
    xy = np.empty((B, K, 2)); w_out = np.empty((B, K))
    tgt_of = lambda st: np.where(st == 1, 1.0, np.where(st == -1, 0.0, 1.0 if policy == "route" else 0.0))
    w = tgt_of(state[:, 0]); tgt_prev = w.copy(); gap = np.zeros(B)
    for k in range(K):
        s_k = s_disp[:, k]
        c_r = mv.at(s_k)[0]
        c_m = mv.at(s_k, main=np.ones(B, bool))[0]
        target = tgt_of(state[:, k])
        d = np.linalg.norm(c_r - c_m, axis=1)
        if lat_rate is None:
            w = target
        else:
            new = target != tgt_prev
            gap = np.where(new, np.abs(target - w) * d, gap)          # distance to the new target now
            if k > 0:
                gap = np.maximum(gap - lat_rate * (t[k] - t[k - 1]), 0.0)
            frac = np.where(d > 1e-6, np.minimum(gap / np.maximum(d, 1e-6), 1.0), 0.0)
            w = np.where(target == 1.0, 1.0 - frac, frac)
        tgt_prev = target
        xy[:, k] = c_m + w[:, None] * (c_r - c_m)
        w_out[:, k] = w
    return xy, w_out


def display_metrics(t, xy, s_disp, road_xy, truth_xy, win):
    """Off-road distance (shown point to the road the car is on), total horizontal error and the largest
    transition step (the shown step minus the along-track step) over the window win (K,) bool."""
    ks = np.nonzero(win)[0]
    off = np.stack([np.sqrt(((xy[:, k, None] - road_xy[None]) ** 2).sum(-1).min(axis=1)) for k in ks], 1)
    err = np.linalg.norm(xy[:, ks] - truth_xy[ks][None], axis=-1)
    step = np.linalg.norm(np.diff(xy[:, ks], axis=1), axis=-1) - np.abs(np.diff(s_disp[:, ks], axis=1))
    jump = np.maximum(step, 0.0).max(axis=1) if len(ks) > 1 else np.zeros(len(xy))
    mo = off.max(axis=1)
    rnd = lambda a: round(float(a), 2)
    return {"offroad_max_mean_m": rnd(mo.mean()), "offroad_max_p95_m": rnd(np.percentile(mo, 95)),
            "runs_offroad_gt_10m": int((mo > 10).sum()),
            "err_max_mean_m": rnd(err.max(axis=1).mean()),
            "transition_step_max_m": rnd(jump.max()), "transition_step_p95_m": rnd(np.percentile(jump, 95))}
