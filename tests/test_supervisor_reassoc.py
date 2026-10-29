"""Targeted tests for the v0.5.11 behaviour (review v8): the S4 failure re-association fixed, the
straight-on control it must not touch, the disagreement monitor and the display follower."""
import numpy as np

from tunnelsim import supervisor as SV
from tunnelsim.eskf import EskfConfig
from tunnelsim.experiments import eskf_experiment


def _portal(e):
    T = e["_tracks"]
    kx = int(np.argmin(np.abs(T["t"] - (T["t_exit"] - 0.05))))
    return T["e_s"][:, kx]


def test_reassociation_fixes_s4_seed_63():
    """Seed 63: 94 m behind at the diverge; without re-association and gate the heading row writes
    the branch curve into yaw (−694 m at the portal)."""
    # the v0.5.11 mechanism, with the true mount (the estimated mount of v0.5.13 changes this seed)
    off = _portal(eskf_experiment("S4", [63], eskf_cfg=EskfConfig(map_nis_gate=False, map_reassoc=False), keep_tracks=True, mount="known"))
    on = _portal(eskf_experiment("S4", [63], keep_tracks=True, mount="known"))
    assert abs(off[0]) > 300.0
    assert abs(on[0]) < 20.0


def test_reassociation_leaves_straight_on_control_unchanged():
    """A car going straight on: re-association runs only on a decided branch, so the result is
    identical with it on or off (v1 pulled such cars back by 92-170 m)."""
    for key in ("S1", "S4"):
        a = _portal(eskf_experiment(key, np.arange(5), eskf_cfg=EskfConfig(map_reassoc=False), branch=False, keep_tracks=True))
        b = _portal(eskf_experiment(key, np.arange(5), eskf_cfg=EskfConfig(map_reassoc=True), branch=False, keep_tracks=True))
        np.testing.assert_array_equal(a, b)


def test_monitor_majority_and_any():
    """σ_disp² = σ_i² + max(0, agg_j[(ŝ_i − ŝ_j)² − σ_i² − σ_j²]). The selected filter alone off by
    50 m: both modes alarm and inflate. Another filter alone off: "any" alarms, "majority" does not."""
    K = 3
    sig = {n: np.full((1, K), 5.0) for n in SV.FILTERS}
    sel = np.array(["eskf"], dtype=object)
    s = {"1d": np.zeros((1, K)), "ekf10": np.zeros((1, K)), "eskf": np.full((1, K), 50.0)}
    for mode in ("any", "majority"):
        _, g, d = SV.monitor(sel, s, sig, mode)
        np.testing.assert_allclose(g, np.sqrt(25.0 + 50.0 ** 2 - 50.0))
        assert (d > SV.ALARM_D).all()
    s = {"1d": np.full((1, K), 50.0), "ekf10": np.zeros((1, K)), "eskf": np.zeros((1, K))}
    _, g, d = SV.monitor(sel, s, sig, "any")
    assert (d > SV.ALARM_D).all() and (g > 5.0).all()
    _, g, d = SV.monitor(sel, s, sig, "majority")
    assert (d < SV.ALARM_D).all()
    np.testing.assert_allclose(g, 5.0)


def test_display_follower_only_after_gnss_and_rate_limited():
    """Before GNSS is back the display is the selected filter; afterwards a re-lock jump of the
    filter appears on the screen at no more than HANDOVER_RATE·dt beyond the filter's own motion."""
    t = np.arange(0.0, 20.0, 0.1)
    v = np.full((1, t.size), 10.0)
    s = (10.0 * t)[None].copy()
    s[0, t >= 12.0] += 30.0                     # re-lock jump 2 s after GNSS returns
    out = SV.display(t, s, v, np.array([10.0]))
    before = t < 10.0
    np.testing.assert_array_equal(out[0, before], s[0, before])
    step = np.diff(out[0]) - 10.0 * np.diff(t)
    assert np.abs(step[t[1:] >= 10.0]).max() <= SV.HANDOVER_RATE * 0.1 + 1e-9
