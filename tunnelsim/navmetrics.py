"""Navigation-layer metrics (v0.5.13, owner 2026-10-29): recovery time after the tunnel and the
timing of the guidance prompts. Evaluation side: these functions read the truth.

Recovery (the displayed track):
    from the first GNSS fix of any kind after the outage, the time until the along-track error of the
    shown position gets under 10 m and under 5 m and stays there for the rest of a 60 s window, and the
    largest error shown meanwhile. The largest step (exit_step) says whether the icon jumps; this says
    when it is right again.

Prompts (owner: 1000 / 500 / 200 m and "now"):
    each prompt fires when ŝ + k·σ reaches s_gore − D (the callout rule of §3); "now" uses
    D = max(30 m, 2 s · v̂) [ASSUMED] with the shown filter's speed. Per prompt: the true distance to the
    gore when it fired minus D (+ = early, metres), the timing against the moment the true car passes
    s_gore − D (late > 1 s), for "now" whether it fired only after the car had reached the gore
    (missed), and the share of prompts worded without a distance (shown 2σ > 100 m: "Exit A ahead").
"""
from __future__ import annotations

import numpy as np

PROMPTS_M = (1000.0, 500.0, 200.0)
NOW_MIN_M = 30.0       # [ASSUMED]
NOW_LEAD_S = 2.0       # [ASSUMED]
LATE_S = 1.0
VAGUE_2SIG_M = 100.0   # [ASSUMED] owner 2026-10-29: above this shown 2σ the prompt names no distance
RECOVERY_WINDOW_S = 60.0


def recovery(t, e, t_back, thresholds=(10.0, 5.0)):
    """t (K,), e (B,K) along-track error of the shown position, t_back (B,) first fix after the outage.
    Returns {thr: (B,) seconds after t_back until |e| < thr for good (NaN: not within the window)},
    and (B,) the largest |e| from t_back until the 10 m time (or the window's end)."""
    B = e.shape[0]
    out = {thr: np.full(B, np.nan) for thr in thresholds}
    worst = np.full(B, np.nan)
    for b in range(B):
        if not np.isfinite(t_back[b]):
            continue
        w = np.nonzero((t >= t_back[b]) & (t <= t_back[b] + RECOVERY_WINDOW_S))[0]
        if len(w) == 0:
            continue
        a = np.abs(e[b, w])
        for thr in thresholds:
            bad = np.nonzero(a >= thr)[0]
            if len(bad) == 0:
                out[thr][b] = 0.0
            elif bad[-1] < len(w) - 1:
                out[thr][b] = t[w[bad[-1] + 1]] - t_back[b]
        t10 = out[thresholds[0]][b]
        stop = len(w) if not np.isfinite(t10) else int(np.searchsorted(t[w], t_back[b] + t10)) + 1
        worst[b] = float(a[:max(stop, 1)].max())
    return out, worst


def prompts(t, s_hat, sig, s_true, v_hat, s_gore, k=1.0):
    """Prompt timing for one estimator. t (K,), s_hat/sig/v_hat (B,K), s_true (K,). Returns a dict of
    per-prompt arrays (B,): err_m (true distance at firing − D, + early), dt_s (firing − true moment,
    + late), and for "now" also missed (fired after the car reached the gore, or never)."""
    B, K = s_hat.shape
    res = {}
    for name, D in [(f"{int(d)} m", np.full((B, K), d)) for d in PROMPTS_M] + \
                   [("now", np.maximum(NOW_MIN_M, NOW_LEAD_S * np.maximum(v_hat, 0.0)))]:
        fire = (s_hat + k * sig) >= (s_gore - D)
        has = fire.any(axis=1)
        kf = np.where(has, np.argmax(fire, axis=1), K - 1)
        Df = D[np.arange(B), kf]
        d_true = s_gore - s_true[kf]
        # the moment the true car passes s_gore − D (D as fired)
        kt = np.array([int(np.argmax(s_true >= s_gore - Df[b])) for b in range(B)])
        r = {"err_m": d_true - Df, "dt_s": t[kf] - t[kt], "fired": has,
             "vague": 2.0 * sig[np.arange(B), kf] > VAGUE_2SIG_M}
        if name == "now":
            r["missed"] = (~has) | (d_true < 0.0)
        res[name] = r
    return res


def summarise_prompts(p):
    """Compact numbers per prompt over the batch."""
    out = {}
    for name, r in p.items():
        e = r["err_m"]
        out[name] = {"err_mean_m": round(float(np.mean(e)), 1), "err_p95_abs_m": round(float(np.percentile(np.abs(e), 95)), 1),
                     "late_gt_1s": round(float(np.mean(r["dt_s"] > LATE_S)), 3),
                     "vague": round(float(np.mean(r["vague"])), 3)}
        if "missed" in r:
            out[name]["missed"] = round(float(np.mean(r["missed"])), 3)
    return out


def summarise_recovery(rec, worst):
    out = {}
    for thr, x in rec.items():
        ok = np.isfinite(x)
        out[f"to_{int(thr)}m_median_s"] = round(float(np.median(x[ok])), 2) if ok.any() else None
        out[f"to_{int(thr)}m_p95_s"] = round(float(np.percentile(x[ok], 95)), 2) if ok.any() else None
        out[f"not_{int(thr)}m_in_60s"] = int((~ok).sum())
    out["worst_shown_m_p95"] = round(float(np.nanpercentile(worst, 95)), 1) if np.isfinite(worst).any() else None
    return out
