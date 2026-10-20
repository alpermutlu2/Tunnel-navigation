"""Scenario definition: road geometry, tunnel, exits, vehicle, speed profile.

Road geometry is defined along arc length s (metres along the road centreline):
  * horizontal: a chain of segments with linearly varying curvature (straight, arc, clothoid)
  * vertical:   grade breakpoints (s, grade_percent), linearly interpolated in grade angle,
                which gives parabolic-like vertical curves between breakpoints
The speed profile is a cruise speed plus speed-change events that trigger when the car
passes a given s. Each change follows a minimum-jerk (quintic) ramp, so acceleration and
jerk are continuous and zero at both ends.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# --------------------------------------------------------------------------- road

@dataclass(frozen=True)
class HSeg:
    """Horizontal segment with curvature varying linearly from k0 to k1 [1/m]. Left turn > 0."""
    length: float
    k0: float = 0.0
    k1: float | None = None

    @property
    def k_end(self) -> float:
        return self.k0 if self.k1 is None else self.k1


@dataclass(frozen=True)
class Exit:
    s: float
    name: str


@dataclass(frozen=True)
class Tunnel:
    s_entry: float
    s_exit: float

    @property
    def length(self) -> float:
        return self.s_exit - self.s_entry


@dataclass
class Road:
    hsegs: list[HSeg]
    grade_points: list[tuple[float, float]]   # (s [m], grade [%]); held constant outside range
    tunnel: Tunnel
    exits: list[Exit] = field(default_factory=list)
    ds: float = 0.25                          # sampling step of the precomputed centreline

    def __post_init__(self):
        self.length = float(sum(seg.length for seg in self.hsegs))
        self._seg_start = np.concatenate([[0.0], np.cumsum([seg.length for seg in self.hsegs])])
        gp = np.array(sorted(self.grade_points), dtype=float)
        self._g_s = gp[:, 0]
        self._g_ang = np.arctan(gp[:, 1] / 100.0)
        self._build_centreline()

    # analytic profiles ---------------------------------------------------------
    def curvature(self, s):
        s = np.atleast_1d(np.asarray(s, dtype=float))
        k = np.zeros_like(s)
        idx = np.clip(np.searchsorted(self._seg_start, s, side="right") - 1, 0, len(self.hsegs) - 1)
        for i, seg in enumerate(self.hsegs):
            m = idx == i
            if not np.any(m):
                continue
            u = (s[m] - self._seg_start[i]) / seg.length
            k[m] = seg.k0 + (seg.k_end - seg.k0) * np.clip(u, 0.0, 1.0)
        return k

    def curvature_rate(self, s):
        """d kappa / ds [1/m^2] (piecewise constant, non-zero only on clothoids)."""
        s = np.atleast_1d(np.asarray(s, dtype=float))
        idx = np.clip(np.searchsorted(self._seg_start, s, side="right") - 1, 0, len(self.hsegs) - 1)
        rates = np.array([(seg.k_end - seg.k0) / seg.length for seg in self.hsegs])
        out = rates[idx]
        out[(s < 0) | (s > self.length)] = 0.0
        return out

    def _grade_parts(self, s):
        s = np.atleast_1d(np.asarray(s, dtype=float))
        gs, ga = self._g_s, self._g_ang
        if len(gs) < 2:
            return np.full_like(s, ga[0]), np.zeros_like(s)
        idx = np.clip(np.searchsorted(gs, s, side="right") - 1, 0, len(gs) - 2)
        L = gs[idx + 1] - gs[idx]
        u = np.clip((s - gs[idx]) / L, 0.0, 1.0)
        f, df, _ = quintic_ramp(u)
        dg = ga[idx + 1] - ga[idx]
        gamma = ga[idx] + dg * f
        rate = np.where((s > gs[0]) & (s < gs[-1]), dg / L * df, 0.0)
        gamma = np.where(s <= gs[0], ga[0], np.where(s >= gs[-1], ga[-1], gamma))
        return gamma, rate

    def grade(self, s):
        """Road grade angle gamma(s) [rad], nose up > 0.

        Between breakpoints the grade follows a minimum-jerk transition, so the vertical
        curvature d gamma/ds is continuous. A piecewise-linear grade has curvature steps,
        and even ideal strapdown integration turns those into metres of drift through
        gravity leakage (tiny pitch errors integrated twice)."""
        return self._grade_parts(s)[0]

    def grade_rate(self, s):
        """d gamma / ds [rad/m]."""
        return self._grade_parts(s)[1]

    # centreline ---------------------------------------------------------------
    def _build_centreline(self):
        n = int(np.ceil(self.length / self.ds)) + 1
        s = np.linspace(0.0, self.length, n)
        k = self.curvature(s)
        g = self.grade(s)
        h = s[1] - s[0]
        psi = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * h)])
        tx = np.cos(g) * np.cos(psi)
        ty = np.cos(g) * np.sin(psi)
        tz = np.sin(g)

        def cumtrapz(f):
            return np.concatenate([[0.0], np.cumsum(0.5 * (f[1:] + f[:-1]) * h)])

        self._cl_s = s
        self._cl_psi = psi
        self._cl_xyz = np.column_stack([cumtrapz(tx), cumtrapz(ty), cumtrapz(tz)])

    def heading(self, s):
        return np.interp(s, self._cl_s, self._cl_psi)

    def position(self, s):
        s = np.asarray(s, dtype=float)
        return np.stack([np.interp(s, self._cl_s, self._cl_xyz[:, i]) for i in range(3)], -1)

    def in_tunnel(self, s):
        s = np.asarray(s)
        return (s >= self.tunnel.s_entry) & (s <= self.tunnel.s_exit)

    def centreline(self, step: float = 5.0):
        s = np.arange(0.0, self.length + step, step)
        s = np.clip(s, 0.0, self.length)
        return s, self.position(s)


# --------------------------------------------------------------------------- vehicle

@dataclass(frozen=True)
class Vehicle:
    """Vehicle reference point = rear-axle centre (the point that follows the path).

    Effects that make the truth violate the filter's ideal assumptions on purpose:
      * suspension pitch      theta_d = K * a                    (squat / dive)
      * body sideslip         tau_b beta_dot + beta = -k_beta * a_lat
                                                                 (velocity vs body x at rear axle,
                                                                  linear bicycle model, rear tyre slip;
                                                                  first-order lag so beta_dot stays
                                                                  continuous at clothoid ends)
      * phone lever arm       r_b from the reference point       (rotation-induced velocity and
                                                                  acceleration at the phone)
    Not modelled (stated in DESIGN.md): body roll, road roughness heave (handled as sensor
    vibration), tyre longitudinal slip.
    """
    pitch_gain_rad_per_mps2: float = np.deg2rad(1.5) / 9.80665    # [ASSUMED] ~1.5 deg per g
    sideslip_gain_rad_per_mps2: float = np.deg2rad(0.4)           # [ASSUMED] ~0.5 deg at 0.12 g
    sideslip_tau_s: float = 0.2                                   # [ASSUMED] yaw/sideslip + tyre lag;
                                                                  # 0 = algebraic (v0.2 behaviour)
    lever_arm_b: tuple[float, float, float] = (1.6, 0.15, 0.7)    # [ASSUMED] dashboard mount, m
    mount_rpy_deg: tuple[float, float, float] = (1.0, 12.0, 3.0)  # [ASSUMED] phone mount offset


# --------------------------------------------------------------------------- speed profile

@dataclass(frozen=True)
class SpeedEvent:
    """When the car passes s >= at_s, change speed to v_target over `duration` seconds,
    then hold that speed for at least `dwell` seconds before the next event may start."""
    at_s: float
    v_target: float
    duration: float
    dwell: float = 0.0


@dataclass
class SpeedProfile:
    v0: float
    events: list[SpeedEvent] = field(default_factory=list)


def quintic_ramp(u):
    """Minimum-jerk ramp. Returns (shape, d shape/du, d2 shape/du2) for u in [0, 1]."""
    u = np.clip(u, 0.0, 1.0)
    f = 10 * u**3 - 15 * u**4 + 6 * u**5
    df = 30 * u**2 - 60 * u**3 + 30 * u**4
    d2f = 60 * u - 180 * u**2 + 120 * u**3
    return f, df, d2f


# --------------------------------------------------------------------------- scenario

@dataclass
class Scenario:
    key: str
    title: str
    description: str
    road: Road
    speed: SpeedProfile
    vehicle: Vehicle = field(default_factory=Vehicle)
    lead_in_s: float = 0.0   # metres of road before which nothing special happens (info only)


def kmh(v: float) -> float:
    return v / 3.6


def _straight_road(total: float, s_entry: float, tunnel_len: float, grade_pct: float = 0.0,
                   exits=()) -> Road:
    return Road(
        hsegs=[HSeg(total)],
        grade_points=[(0.0, grade_pct), (total, grade_pct)],
        tunnel=Tunnel(s_entry, s_entry + tunnel_len),
        exits=list(exits),
    )


APPROACH_S = 200.0      # [ASSUMED] GNSS time before every portal (v0.4 plan 4: 180 s window + margin)
AFTER_PORTAL_S = 220.0  # [ASSUMED] road after the portal covers >= 220 s at entry speed (180 s budget)
LEAD_OUT_MIN = 800.0    # minimum road after the tunnel exit (recovery)


def _approach(v0: float) -> float:
    return APPROACH_S * v0


def _lead_out(v0: float, tunnel_len: float) -> float:
    return max(LEAD_OUT_MIN, AFTER_PORTAL_S * v0 - tunnel_len)


def _approach_maneuvers(v0: float) -> list[SpeedEvent]:
    """Excitation for the pre-tunnel calibration (v0.4 focus decision, 2026-10-20): accelerate
    +15 km/h, brake to v0 - 15 km/h, return to v0; all ramps done ~90 s before the portal, so the
    entry speed is steady. Why: with constant speed the GNSS-aided filter can only estimate a
    lumped forward offset; speed changes let it show that it estimates the bias rather than
    assume it. [ASSUMED] magnitudes and timing."""
    return [SpeedEvent(20.0 * v0, v0 + kmh(15), 8.0, dwell=15.0),
            SpeedEvent(20.0 * v0, v0 - kmh(15), 8.0, dwell=15.0),
            SpeedEvent(20.0 * v0, v0, 10.0)]


def build_scenarios() -> dict[str, Scenario]:
    """Scenario catalogue (DESIGN.md section 6).

    Approach: 200 s of driving at entry speed before the portal (GNSS calibration windows up to
    180 s). Lead-out: at least 800 m after the exit and enough road for 220 s after the portal
    (fixed 180 s INS budget). All positions inside a scenario are given relative to its portal
    s_e, so the tunnel geometry and speed events are the same as in v0.3; only the approach and
    lead-out lengths changed (v0.4).
    """
    out: dict[str, Scenario] = {}

    # S1: straight, no grade, no curves, constant speed. Hardest case: no map features.
    v0, L_t = kmh(80), 3000.0
    s_e = _approach(v0)
    out["S1"] = Scenario(
        key="S1",
        title="Straight, flat, constant speed",
        description="3 km straight tunnel, 0 % grade, 80 km/h constant. No map features.",
        road=_straight_road(s_e + L_t + _lead_out(v0, L_t), s_e, L_t, 0.0,
                            exits=[Exit(s_e + 2200, "Exit A")]),
        speed=SpeedProfile(v0=v0, events=_approach_maneuvers(v0)),
    )

    # S2: curved tunnel (S-bend with clothoid transitions).
    R, v0, L_t = 400.0, kmh(80), 2500.0
    s_e = _approach(v0)
    hs = [
        HSeg(s_e),                        # approach
        HSeg(500),                        # straight inside
        HSeg(80, 0.0, 1 / R), HSeg(350, 1 / R), HSeg(80, 1 / R, 0.0),
        HSeg(400),
        HSeg(80, 0.0, -1 / R), HSeg(350, -1 / R), HSeg(80, -1 / R, 0.0),
        HSeg(580),                        # straight to the tunnel exit at s_e + 2500
        HSeg(_lead_out(v0, L_t)),         # lead-out
    ]
    out["S2"] = Scenario(
        key="S2",
        title="Curved tunnel",
        description="2.5 km tunnel with an S-bend (R = 400 m), 80 km/h constant.",
        road=Road(hsegs=hs, grade_points=[(0.0, 0.0)], tunnel=Tunnel(s_e, s_e + L_t),
                  exits=[Exit(s_e + 1800.0, "Exit A")]),
        speed=SpeedProfile(v0=v0, events=_approach_maneuvers(v0)),
    )

    # S3: V-profile (undersea type): -5 % down, flat bottom, +5 % up.
    v0, L_t = kmh(70), 3500.0
    s_e = _approach(v0)
    g = [(-100.0, 0.0), (100.0, -5.0), (1100.0, -5.0), (1300.0, 0.0),
         (2100.0, 0.0), (2300.0, 5.0), (3300.0, 5.0), (3500.0, 0.0)]
    out["S3"] = Scenario(
        key="S3",
        title="V-profile tunnel",
        description="3.5 km undersea-type tunnel: -5 % descent, flat bottom, +5 % climb.",
        road=Road(
            hsegs=[HSeg(s_e + L_t + _lead_out(v0, L_t))],
            grade_points=[(s_e + x, gr) for x, gr in g],
            tunnel=Tunnel(s_e, s_e + L_t),
            exits=[Exit(s_e + 2700.0, "Exit A")],
        ),
        speed=SpeedProfile(v0=v0, events=_approach_maneuvers(v0)),
    )

    # S4: stop-and-go inside a flat straight tunnel.
    v0, L_t = kmh(60), 3000.0
    s_e = _approach(v0)
    out["S4"] = Scenario(
        key="S4",
        title="Stop-and-go traffic",
        description="3 km flat straight tunnel, two full stops and a slow crawl inside.",
        road=_straight_road(s_e + L_t + _lead_out(v0, L_t), s_e, L_t, 0.0,
                            exits=[Exit(s_e + 2400, "Exit A")]),
        speed=SpeedProfile(v0=v0, events=_approach_maneuvers(v0) + [
            SpeedEvent(s_e + 400.0, 0.0, 8.0, dwell=20.0),
            SpeedEvent(s_e + 400.0, kmh(30), 10.0, dwell=30.0),
            SpeedEvent(s_e + 1100.0, 0.0, 6.0, dwell=45.0),
            SpeedEvent(s_e + 1100.0, kmh(15), 6.0, dwell=40.0),
            SpeedEvent(s_e + 1300.0, kmh(60), 15.0),
        ]),
    )

    # S5: slow speed drift and braking inside a flat straight tunnel.
    v0, L_t = kmh(90), 3000.0
    s_e = _approach(v0)
    out["S5"] = Scenario(
        key="S5",
        title="Slow drift and braking",
        description="3 km flat straight tunnel: speed drifts 90 -> 82 km/h over 60 s, "
                    "then a brake to 50 km/h and a slow recovery.",
        road=_straight_road(s_e + L_t + _lead_out(v0, L_t), s_e, L_t, 0.0,
                            exits=[Exit(s_e + 2500, "Exit A")]),
        speed=SpeedProfile(v0=v0, events=_approach_maneuvers(v0) + [
            SpeedEvent(s_e + 200.0, kmh(82), 60.0),
            SpeedEvent(s_e + 1800.0, kmh(50), 5.0, dwell=10.0),
            SpeedEvent(s_e + 1800.0, kmh(75), 40.0),
        ]),
    )
    # S6: like S1, but the speed wanders slowly inside the tunnel: +-2 km/h with 40 s ramps
    # (peak 0.026 m/s^2, mean 0.014 m/s^2, below the cruise-mode threshold 0.02). The breaking
    # case for the speed-hold (cruise) mode: a real driver never holds speed exactly.
    # Variants (review 6): S6s +-1 km/h (smaller), S6L +-4 km/h (mean 0.028 m/s^2, slightly
    # above the threshold). [ASSUMED] magnitudes.
    for key, dv in (("S6", 2.0), ("S6s", 1.0), ("S6L", 4.0)):
        v0, L_t = kmh(80), 3000.0
        s_e = _approach(v0)
        out[key] = Scenario(
            key=key,
            title=f"Straight, slow speed wander +-{dv:g} km/h",
            description=f"As S1, but inside the tunnel the speed wanders +-{dv:g} km/h with slow 40 s ramps.",
            road=_straight_road(s_e + L_t + _lead_out(v0, L_t), s_e, L_t, 0.0,
                                exits=[Exit(s_e + 2200, "Exit A")]),
            speed=SpeedProfile(v0=v0, events=_approach_maneuvers(v0) + [
                SpeedEvent(s_e + 100.0, v0 + kmh(dv), 40.0),
                SpeedEvent(s_e + 1200.0, v0 - kmh(dv), 40.0),
                SpeedEvent(s_e + 2100.0, v0, 40.0),   # after the previous ramp ends (~2070 m)
            ]),
        )
    return out
