"""Supervisor over the three filters (owner 2026-10-25, v0.5.11).

Estimator side: imports only numpy. It sees the three filters' along-track outputs (ŝ, σ_s on a
common time axis), the detected IMU class and the open-sky rehearsal scores; never the truth.

Three parts:

1. Selection before the tunnel (decided once, when the outage starts):
   - "class":     note10 -> ESKF (16 states), mpu9250 -> 10-state            (rule set approved 2026-10-25)
   - "rehearsal": shadow tunnels in open sky. A shadow copy of each filter runs without GNSS for
                  REHEARSAL_LEN_S, then its ŝ is compared with the GNSS fix projected on the map.
                  score = rms error, consistency = mean e²/(σ_s² + σ_fix²) over the windows.
                  Pick the filter with the lowest rms among those with consistency <= NEES_PASS;
                  if none passes, the class rule.
   - "simplest":  as "rehearsal", but the simplest filter whose rms is within SIMPLE_FACTOR of the best
                  and whose consistency passes (1D < 10-state < ESKF).
   All thresholds are fixed before evaluation [ASSUMED] and checked on held-out seeds.
2. Disagreement monitor (does not switch filters): for the selected filter i and each other j,
       d_ij = (ŝ_i − ŝ_j)² / (σ_i² + σ_j²),
       σ_disp² = σ_i² + max(0, agg_j[(ŝ_i − ŝ_j)² − σ_i² − σ_j²]),   agg = max ("any") or min ("majority"),
   alarm while agg_j d_ij > ALARM_D. The excess of the disagreement over what the σs explain is
   added to the uncertainty shown and used by the callout (ŝ + k·σ_disp), so the callout comes
   earlier when the models disagree.
3. Display: inside the tunnel the selected filter; once GNSS is back, a follower that moves with
   the selected filter's speed and closes the gap to its re-locked ŝ at no more than HANDOVER_RATE
   m/s (the 1D and 10-state filters have no re-acquisition cap of their own). The branch decision
   shown comes from the selected filter: the ESKF's, or since v0.5.12 the 10-state filter's own
   (batch test of the yaw-change profile against both geometries); the 1D filter has none (ESKF's).
"""
from __future__ import annotations

import numpy as np

FILTERS = ("1d", "ekf10", "eskf")          # simplest first
CLASS_RULE = {"note10": "eskf", "mpu9250": "ekf10"}
REHEARSAL_LEN_S = 30.0                      # [ASSUMED] shadow outage length
REHEARSAL_FIRST_S = 40.0                    # [ASSUMED] first shadow outage after the filter start
REHEARSAL_PERIOD_S = 45.0                   # [ASSUMED] 30 s outage + 15 s of GNSS to re-converge
NEES_PASS = 4.0                             # [ASSUMED] mean e²/σ² a filter may show in the rehearsals
SIMPLE_FACTOR = 1.5                         # [ASSUMED] "as good as the best" for the simplest rule
ALARM_D = 9.0                               # [ASSUMED] 3σ disagreement
HANDOVER_RATE = 2.0                         # m/s [ASSUMED]: as the ESKF's re-acquisition cap (2 m per fix)


def rehearsal_windows(t_start, t_outage):
    """Shadow outages [(a, b)] that end before the real outage starts."""
    out, a = [], t_start + REHEARSAL_FIRST_S
    while a + REHEARSAL_LEN_S < t_outage - 1.0:
        out.append((a, a + REHEARSAL_LEN_S))
        a += REHEARSAL_PERIOD_S
    return out


def mask_gnss(gnss, windows):
    """Copy of a gnss dict with every fix inside the windows removed (position and velocity)."""
    g = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in gnss.items()}
    m = np.zeros(len(g["t"]), bool)
    for a, b in windows:
        m |= (g["t"] > a) & (g["t"] < b)
    for k in ("pos", "vel"):
        g[k][:, m] = np.nan
    for k in ("acc_pos_h", "acc_pos_v", "acc_vel"):
        g[k][:, m] = np.nan
    return g


def rehearsal_scores(t, s_shadow, sig_shadow, windows, fix_t, s_fix, sig_fix):
    """Per batch member: rms error and mean e²/(σ²+σ_fix²) of a shadow filter at the end of each
    window, against the first GNSS fix after it projected on the map. t (K,), s/sig (B,K);
    fix_t (M,), s_fix/sig_fix (B,M)."""
    E, Z = [], []
    for a, b in windows:
        j = int(np.searchsorted(fix_t, b))
        if j >= len(fix_t):
            continue
        k = max(0, int(np.searchsorted(t, fix_t[j])) - 1)     # shadow just before it sees the fix
        e = s_shadow[:, k] - s_fix[:, j]
        E.append(e)
        Z.append(e * e / (sig_shadow[:, k] ** 2 + sig_fix[:, j] ** 2))
    E, Z = np.array(E), np.array(Z)                            # (W,B)
    return np.sqrt(np.nanmean(E ** 2, axis=0)), np.nanmean(Z, axis=0)


def select(rule, imu_class, scores=None):
    """Selected filter name per batch member. scores: {name: (rms (B,), nees (B,))}."""
    B = len(imu_class)
    base = np.array([CLASS_RULE.get(str(c), "eskf") for c in imu_class], dtype=object)
    if rule == "class" or scores is None:
        return base
    names = [n for n in FILTERS if n in scores]
    rms = np.stack([scores[n][0] for n in names])              # (F,B)
    nees = np.stack([scores[n][1] for n in names])
    ok = nees <= NEES_PASS
    out = base.copy()
    for b in range(B):
        if not ok[:, b].any():
            continue
        r = np.where(ok[:, b], rms[:, b], np.inf)
        if rule == "rehearsal":
            out[b] = names[int(np.argmin(r))]
        elif rule == "simplest":
            best = r.min()
            out[b] = next(n for i, n in enumerate(names) if r[i] <= SIMPLE_FACTOR * best)
        else:
            raise ValueError(rule)
    return out


def monitor(sel, s, sig, mode="any"):
    """Disagreement monitor. sel (B,) names; s, sig: {name: (B,K)}. Returns ŝ_sel, σ_disp, d (B,K).
    mode "any": the largest disagreement with any other filter counts (sensitive; a broken filter
    inflates the selected one's σ). "majority": the smallest, so the selected filter's σ grows only
    when it disagrees with both others (misses a failure two filters share, e.g. a barometer one)."""
    names = list(s)
    S = np.stack([s[n] for n in names]); G = np.stack([sig[n] for n in names])      # (F,B,K)
    i = np.array([names.index(x) for x in sel])
    b = np.arange(len(sel))
    s_i, g_i = S[i, b], G[i, b]
    diff2 = (S - s_i[None]) ** 2
    den = G ** 2 + g_i[None] ** 2
    other = np.ones(S.shape[:2], bool); other[i, b] = False
    d = np.where(other[..., None], diff2 / den, 0.0)
    excess = np.where(other[..., None], diff2 - den, 0.0)
    if mode == "majority":
        big = np.where(other[..., None], 0.0, np.inf)
        ex_, d_ = (excess + big).min(axis=0), (d + big).min(axis=0)
    else:
        ex_, d_ = excess.max(axis=0), d.max(axis=0)
    sig_disp = np.sqrt(g_i ** 2 + np.maximum(0.0, ex_))
    return s_i, sig_disp, d_


def display(t, s_sel, v_sel, t_back):
    """Displayed ŝ: the selected filter until GNSS is back (t_back (B,)); from then on a follower that
    moves with the selected filter's speed and closes the gap to its re-locked ŝ at no more than
    HANDOVER_RATE m/s, so a re-lock jump of the filter is not a jump on the screen."""
    out = s_sel.copy()
    for b in range(len(s_sel)):
        k0 = int(np.searchsorted(t, t_back[b]))
        if k0 <= 0 or k0 >= len(t):
            continue
        d = s_sel[b, k0 - 1]
        for k in range(k0, len(t)):
            dt = t[k] - t[k - 1]
            d = d + v_sel[b, k] * dt
            d = d + np.clip(s_sel[b, k] - d, -HANDOVER_RATE * dt, HANDOVER_RATE * dt)
            out[b, k] = d
    return out
