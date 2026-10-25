"""The navigation map as the estimator sees it (review 9, map in the tunnel).

Estimator side: built from a road-like object (anything with `position(s)`, `heading(s)`,
`length`, `exits`), sampled on a 1 m grid. Map errors for stress tests [ASSUMED]: a constant
lateral offset of the polyline and a length scale (the map's s differs from the road's by a
factor), cf. DESIGN 7.2.
"""
from __future__ import annotations

import numpy as np


class MapView:
    def __init__(self, road, lat_offset_m: float = 0.0, length_scale: float = 1.0, step: float = 1.0):
        self.s = np.arange(0.0, road.length, step)
        xy = road.position(self.s)[:, :2]
        psi = road.heading(self.s)
        n = np.stack([-np.sin(psi), np.cos(psi)], 1)       # left normal
        # length error: the polyline itself is stretched about the tunnel entry (features inside
        # the tunnel, e.g. the exit curve, sit (scale − 1)·distance further on), arc length with it
        k_e = int(round(road.tunnel.s_entry / step))
        xy = xy[k_e] + length_scale * (xy - xy[k_e])
        self.s = self.s[k_e] + length_scale * (self.s - self.s[k_e])
        self.xy = xy + lat_offset_m * n
        self.psi = np.unwrap(psi)
        self.kappa = np.gradient(self.psi, self.s)          # map curvature [1/m]
        # road grade γ(s) [rad] and dγ/ds, when the road object has them (v0.5.11, pitch prior)
        if hasattr(road, "grade"):
            s0 = np.arange(0.0, road.length, step)
            self.gamma = road.grade(s0)
            self.gamma_rate = np.gradient(self.gamma, self.s)
        else:
            self.gamma = np.zeros_like(self.s); self.gamma_rate = np.zeros_like(self.s)
        self.s_entry = self.s[k_e]
        self.xy_entry = self.xy[k_e].copy()
        self.step = step
        self.length = self.s[-1]
        ex = road.exits[0] if road.exits else None
        self.s_div = None if ex is None else self.s[k_e] + length_scale * (ex.s - self.s[k_e])
        self.curve_len = 0.0 if ex is None else ex.curve_len * length_scale
        # main tunnel continuation after the diverge: straight along the diverge heading
        if ex is not None and ex.on_route:
            k = int(np.argmin(np.abs(self.s - self.s_div)))
            self.main_xy0, self.main_psi = self.xy[k], self.psi[k]

    def grade_at(self, s):
        """Road grade γ [rad] and dγ/ds at map s (B,)."""
        k = np.clip(np.searchsorted(self.s, s), 0, len(self.s) - 1)
        return self.gamma[k], self.gamma_rate[k]

    def nearest(self, p_xy, s_prev, window=300.0):
        """Along-track position of points p_xy (B,2): nearest polyline point within ±window of
        s_prev (B,), or over the whole route where s_prev is NaN."""
        out = np.empty(len(p_xy))
        for b in range(len(p_xy)):
            if np.isfinite(s_prev[b]):
                a, e = np.searchsorted(self.s, [s_prev[b] - window, s_prev[b] + window + self.step])
            else:
                a, e = 0, len(self.s)
            d2 = ((self.xy[a:e] - p_xy[b]) ** 2).sum(1)
            out[b] = self.s[a + int(np.argmin(d2))]
        return out

    def at(self, s, main=None, with_kappa=False):
        """Centreline point, heading and left normal at map s (B,). main (B,) bool: use the main
        tunnel continuation (straight) instead of the route beyond the diverge."""
        k = np.clip(np.searchsorted(self.s, s), 0, len(self.s) - 1)
        c, psi, kap = self.xy[k].copy(), self.psi[k].copy(), self.kappa[k].copy()
        if main is not None and self.s_div is not None and main.any():
            m = main & (s > self.s_div)
            d = (s[m] - self.s_div)[:, None]
            c[m] = self.main_xy0 + d * np.array([np.cos(self.main_psi), np.sin(self.main_psi)])
            psi[m] = self.main_psi
            kap[m] = 0.0
        n = np.stack([-np.sin(psi), np.cos(psi)], 1)
        return (c, psi, n, kap) if with_kappa else (c, psi, n)
