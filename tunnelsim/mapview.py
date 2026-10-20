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
        s_true = self.s / length_scale                     # map s -> road s
        xy = road.position(s_true)[:, :2]
        psi = road.heading(s_true)
        n = np.stack([-np.sin(psi), np.cos(psi)], 1)       # left normal
        self.xy = xy + lat_offset_m * n
        self.psi = np.unwrap(psi)
        self.step = step
        self.length = self.s[-1]
        ex = road.exits[0] if road.exits else None
        self.s_div = None if ex is None else ex.s * length_scale
        self.curve_len = 0.0 if ex is None else ex.curve_len * length_scale
        # main tunnel continuation after the diverge: straight along the diverge heading
        if ex is not None and ex.on_route:
            k = int(round(self.s_div / step))
            self.main_xy0, self.main_psi = self.xy[k], self.psi[k]

    def nearest(self, p_xy, s_prev, window=300.0):
        """Along-track position of points p_xy (B,2): nearest polyline point within ±window of
        s_prev (B,), or over the whole route where s_prev is NaN."""
        out = np.empty(len(p_xy))
        for b in range(len(p_xy)):
            if np.isfinite(s_prev[b]):
                a = max(0, int((s_prev[b] - window) / self.step)); e = min(len(self.s), int((s_prev[b] + window) / self.step) + 1)
            else:
                a, e = 0, len(self.s)
            d2 = ((self.xy[a:e] - p_xy[b]) ** 2).sum(1)
            out[b] = self.s[a + int(np.argmin(d2))]
        return out

    def at(self, s, main=None):
        """Centreline point, heading and left normal at map s (B,). main (B,) bool: use the main
        tunnel continuation (straight) instead of the route beyond the diverge."""
        k = np.clip(np.round(s / self.step).astype(int), 0, len(self.s) - 1)
        c, psi = self.xy[k].copy(), self.psi[k].copy()
        if main is not None and self.s_div is not None and main.any():
            m = main & (s > self.s_div)
            d = (s[m] - self.s_div)[:, None]
            c[m] = self.main_xy0 + d * np.array([np.cos(self.main_psi), np.sin(self.main_psi)])
            psi[m] = self.main_psi
        n = np.stack([-np.sin(psi), np.cos(psi)], 1)
        return c, psi, n
