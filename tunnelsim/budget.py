"""Error budget harness (v0.4 step 3; docs/decisions/2026-10-19-v0.4-plan.md section 2).

This is an ANALYSIS module: it reads the truth to supply oracle information (initial state,
ideal attitude, calibration values, true mount and lever arm) and to score the INS. The INS
itself (`ins.py`) never sees the truth (docs/contracts.md 6).

Fixed 180 s budget = INS-only propagation from the portal, no GNSS at all, even past a short
tunnel's exit. The run length is max(180 s, time to the real exit), so the same runs also give
the real-exit (operational) error, reported under its own name.

Per seed the IMU is generated ONCE with every IMU error term on (no timing errors), and each
configuration's measurement is recomposed from the stored components with the generator's
own formula, f = (I + S + M)(f_true + vib) + b_on + b_BI + n. Per-term RNG streams make this
identical to calling the generator with only those terms on (tested).

Along-track error at R, first order: e_s = t_n · (p̂_R − p_R), t_n the true road tangent at R,
p̂_R = p̂_P − Ĉ_np C_bpᵀ r_b with the true mount and lever arm (oracle_mount, oracle_lever_arm).
The exact map projection comes with the map model (step 4).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .frames import G0
from .ins import ImuCalibration, InsInit, strapdown
from .profiles import NOMINAL, configure
from .sensors import ImuTruthSource, Switches, bi_components, simulate_imu
from .truth import generate_truth

BUDGET_S = 180.0
CURVE_T = (30.0, 60.0, 90.0, 120.0, 150.0, 180.0)
IMU_TERMS = ("a_white", "g_white", "a_bi", "g_bi", "a_turnon", "g_turnon", "a_scale", "g_scale",
             "a_misalign", "g_misalign", "vib_road", "vib_engine")


@dataclass(frozen=True)
class Config:
    """One budget configuration: which error terms are on, how attitude is obtained, and what
    the INS is told (calibration level at the tunnel-entry epoch)."""
    name: str
    terms: tuple
    attitude: str                     # "ideal" or "free"
    calibration: str | None = None    # None, "bias" (turn-on + BI state at entry), "all" (+ S, M)


ACCEL = [Config("a_white", ("a_white",), "ideal"),
         Config("a_bi", ("a_bi",), "ideal", "bias"),
         Config("a_bi_raw", ("a_bi",), "ideal"),
         Config("a_turnon_raw", ("a_turnon",), "ideal"),
         Config("a_scale_raw", ("a_scale",), "ideal"),
         Config("a_misalign_raw", ("a_misalign",), "ideal")]
GYRO = [Config("g_white", ("g_white",), "free"),
        Config("g_bi", ("g_bi",), "free", "bias"),
        Config("g_bi_raw", ("g_bi",), "free"),
        Config("g_turnon_raw", ("g_turnon",), "free"),
        Config("g_scale_raw", ("g_scale",), "free"),
        Config("g_misalign_raw", ("g_misalign",), "free")]
VIB = [Config(f"{v}_{a}", (v,), a) for v in ("vib_road", "vib_engine") for a in ("ideal", "free")]
ALL_TERMS = [Config(f"all_{a}_{c or 'none'}", IMU_TERMS, a, c)
             for a in ("ideal", "free") for c in (None, "bias", "all")]
CONFIGS = ACCEL + GYRO + VIB + ALL_TERMS


# ---------------------------------------------------------------------------------------------
# Measurement recomposition
# ---------------------------------------------------------------------------------------------
def compose(e, terms, sl=slice(None)):
    """Measurement with only `terms` on, from an all-terms ImuErrors (same formula and order as
    simulate_imu). Returns f, w over the index slice sl."""
    on = set(terms)
    I3 = np.eye(3)
    c, k = e.components, e.constants
    f_in, w_in = e.f_true[sl].copy(), e.w_true[sl].copy()
    for v in ("vib_road", "vib_engine"):
        if v in on:
            f_in += c[f"{v}_a"][sl]
            w_in += c[f"{v}_g"][sl]
    Ka = I3 + (k["S_a"] if "a_scale" in on else 0) + (k["M_a"] if "a_misalign" in on else 0)
    Kg = I3 + (k["S_g"] if "g_scale" in on else 0) + (k["M_g"] if "g_misalign" in on else 0)
    f, w = f_in @ Ka.T, w_in @ Kg.T
    if "a_turnon" in on:
        f += k["b_a_on"]
    if "g_turnon" in on:
        w += k["b_g_on"]
    for term, tgt in (("a_bi", f), ("g_bi", w), ("a_white", f), ("g_white", w)):
        if term in on:
            tgt += c[term][sl]
    return f, w


def calibration_for(e, cfg: Config, i0: int) -> dict:
    """What the INS is told for this config (perfect calibration at the tunnel-entry epoch).
    Only for terms that are on; never the future of the bias process."""
    on = set(cfg.terms)
    if cfg.calibration is None:
        return {}
    k, c = e.constants, e.components
    out = {"ba": np.zeros(3), "bg": np.zeros(3)}
    if "a_turnon" in on:
        out["ba"] = out["ba"] + k["b_a_on"]
    if "g_turnon" in on:
        out["bg"] = out["bg"] + k["b_g_on"]
    if "a_bi" in on:
        out["ba"] = out["ba"] + c["a_bi"][i0]
    if "g_bi" in on:
        out["bg"] = out["bg"] + c["g_bi"][i0]
    if cfg.calibration == "all":
        I3 = np.eye(3)
        out["Ka"] = I3 + (k["S_a"] if "a_scale" in on else 0) + (k["M_a"] if "a_misalign" in on else 0)
        out["Kg"] = I3 + (k["S_g"] if "g_scale" in on else 0) + (k["M_g"] if "g_misalign" in on else 0)
    return out


# ---------------------------------------------------------------------------------------------
# Runs
# ---------------------------------------------------------------------------------------------
@dataclass
class ScenarioRun:
    key: str
    t_rel: np.ndarray                          # (K,) evaluation times after the portal [s]
    exit_rel: float                            # real exit time after the portal [s]
    e_s: dict = field(default_factory=dict)    # config name -> (seeds, K) along-track error [m]


def _tangent(tr, idx):
    g, p = tr.gamma[idx], tr.psi[idx]
    return np.column_stack([np.cos(g) * np.cos(p), np.cos(g) * np.sin(p), np.sin(g)])


def run_scenario(key, seeds, configs=CONFIGS, chunk=100, cfg=None) -> ScenarioRun:
    from .scenario import build_scenarios
    sc = build_scenarios()[key]
    tr = generate_truth(sc)
    cfg = cfg or configure("no_gaps")
    sw = Switches.only(*IMU_TERMS)
    src = ImuTruthSource.from_truth(tr)
    i_entry = int(np.argmax(tr.in_tunnel))
    i_exit = len(tr.in_tunnel) - int(np.argmax(tr.in_tunnel[::-1]))
    t_entry = tr.t[i_entry]
    exit_rel = float(tr.t[min(i_exit, len(tr.t) - 1)] - t_entry)
    L = max(BUDGET_S, exit_rel)
    # IMU grid equals the truth grid (no jitter, no rate offset, no gaps)
    i0, i1 = i_entry, i_entry + int(round(L / tr.dt)) + 1
    sl = slice(i0, i1)
    t = tr.t[sl]
    C_np_true = tr.C_nb[sl] @ tr.C_bp
    r_b, C_bp = tr.lever_arm_b, tr.C_bp
    eval_t = sorted(set(CURVE_T) | {round(exit_rel, 2)})
    k_eval = np.array([int(round(x / tr.dt)) for x in eval_t])
    tn = _tangent(tr, i0 + k_eval)
    pR_true = tr.pos_n[i0 + k_eval]
    init = InsInit(tr.pos_phone_n[i0], tr.vel_phone_n[i0], C_np_true[0])
    out = ScenarioRun(key=key, t_rel=np.array(eval_t), exit_rel=exit_rel)
    res_acc = {c.name: [] for c in configs}
    for start in range(0, len(seeds), chunk):
        errs = []
        for s in seeds[start:start + chunk]:
            _, e = simulate_imu(src, cfg, sw, seed=int(s))
            errs.append(e)
        for c in configs:
            F, W, cal = [], [], []
            for e in errs:
                f, w = compose(e, c.terms, sl)
                F.append(f), W.append(w), cal.append(calibration_for(e, c, i0))
            F, W = np.stack(F), np.stack(W)
            ic = ImuCalibration()
            if cal[0]:
                ic = ImuCalibration(accel_bias=np.stack([x["ba"] for x in cal]),
                                    gyro_bias=np.stack([x["bg"] for x in cal]),
                                    accel_matrix=np.stack([x["Ka"] for x in cal]) if "Ka" in cal[0] else None,
                                    gyro_matrix=np.stack([x["Kg"] for x in cal]) if "Kg" in cal[0] else None)
            att = C_np_true if c.attitude == "ideal" else None
            r = strapdown(t, F, W, init, ic, attitude=att)
            pP, Cnp = r.p_n[:, k_eval], r.C_np[:, k_eval]
            pR = pP - np.einsum("bkij,j->bki", Cnp @ C_bp.T, r_b)
            res_acc[c.name].append(np.einsum("bki,ki->bk", pR - pR_true, tn))
    out.e_s = {k_: np.concatenate(v) for k_, v in res_acc.items()}
    return out


# ---------------------------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------------------------
def stats(x, n_boot=1000, seed=0) -> dict:
    """bias, rmse, p95_abs with a bootstrap 90 % CI for p95_abs (docs/contracts.md 7)."""
    x = np.asarray(x, dtype=float)
    a = np.abs(x)
    rng = np.random.default_rng(seed)
    boot = np.quantile(a[rng.integers(0, a.size, (n_boot, a.size))], 0.95, axis=1)
    return {"bias": float(f"{x.mean():.4g}"), "rmse": float(f"{np.sqrt(np.mean(x * x)):.4g}"),
            "p95_abs": float(f"{np.quantile(a, 0.95):.4g}"),
            "p95_abs_ci90": [float(f"{np.quantile(boot, 0.05):.4g}"), float(f"{np.quantile(boot, 0.95):.4g}")]}


# ---------------------------------------------------------------------------------------------
# Analytic references (along-track, 1D, first order)
# ---------------------------------------------------------------------------------------------
def _proj_var(e, per_axis_var):
    return float(np.sum(np.asarray(e) ** 2 * np.asarray(per_axis_var)))


def _gm_kernel(t, tau, power, mode, n=801):
    """∫∫ (t−u)^p (t−v)^p C(u,v) du dv for a unit-variance stationary GM, where C is the
    covariance of the bias error the INS integrates:
      raw : no calibration,                 C = e^(−|u−v|/τ)
      hold: b(0) known and held constant,   C = e^(−|u−v|/τ) − e^(−u/τ) − e^(−v/τ) + 1
      pred: b(0) known, best prediction b(0) e^(−u/τ) subtracted,
                                            C = e^(−|u−v|/τ) − e^(−(u+v)/τ)
    """
    u = np.linspace(0.0, t, n)
    U, V = np.meshgrid(u, u, indexing="ij")
    E = np.exp(-np.abs(U - V) / tau)
    if mode == "raw":
        C = E
    elif mode == "hold":
        C = E - np.exp(-U / tau) - np.exp(-V / tau) + 1.0
    elif mode == "pred":
        C = E - np.exp(-(U + V) / tau)
    else:
        raise ValueError(mode)
    k = (t - u) ** power
    return float(np.trapezoid(np.trapezoid(k[:, None] * k[None, :] * C, u, axis=1), u))


def analytic_rmse(prof, e_fwd, e_pitch, t) -> dict:
    """RMS along-track error at time t for the single-term configs that have a closed form.
    e_fwd: forward axis in phone frame; e_pitch: pitch axis in phone frame (true mount).
    Bias instability in three forms: *_bi_raw (no calibration), *_bi (b at entry known and
    held: the budget's 'perfect calibration'), *_bi_pred (b at entry known, best GM prediction:
    the lower bound of what knowing the entry bias can give)."""
    Na2, Ng2 = prof.accel_vrw.a ** 2, prof.gyro_arw.a ** 2
    out = {
        "a_white": np.sqrt(_proj_var(e_fwd, Na2) * t**3 / 3.0),
        "g_white": G0 * np.sqrt(_proj_var(e_pitch, Ng2) * t**5 / 20.0),
        "a_turnon_raw": np.sqrt(_proj_var(e_fwd, [float(prof.accel_turn_on_sigma.v) ** 2] * 3)) * t**2 / 2,
        "g_turnon_raw": G0 * np.sqrt(_proj_var(e_pitch, [float(prof.gyro_turn_on_sigma.v) ** 2] * 3)) * t**3 / 6,
    }
    for pre, e, N, B, p, c in (("a", e_fwd, prof.accel_vrw.a, prof.accel_bias_instab.a, 1, 1.0),
                               ("g", e_pitch, prof.gyro_arw.a, prof.gyro_bias_instab.a, 2, G0 / 2)):
        for mode, name in (("hold", f"{pre}_bi"), ("raw", f"{pre}_bi_raw"), ("pred", f"{pre}_bi_pred")):
            var = 0.0
            for i in range(3):
                sig, taus = bi_components(N[i], B[i], prof)
                var += e[i] ** 2 * sum(s_ * s_ * _gm_kernel(t, tau, p, mode) for s_, tau in zip(sig, taus))
            out[name] = c * np.sqrt(var)
    return {k: float(f"{v:.4g}") for k, v in out.items()}


def timestamp_check(key, eps=0.01) -> dict:
    """Clock-scale error alone (all sensor errors off), free attitude, from the portal.
    First-order prediction: e_s ≈ ε (2 Δs − v0 t) (constant speed: ε v0 t)."""
    from .scenario import build_scenarios
    sc = build_scenarios()[key]
    tr = generate_truth(sc)
    cfgx = configure("no_gaps", "ts_scale_1pct") if eps == 0.01 else None
    m, e = simulate_imu(ImuTruthSource.from_truth(tr), cfgx, Switches.only(), seed=0)
    i0 = int(np.argmax(tr.in_tunnel))
    n = int(round(BUDGET_S / tr.dt)) + 1
    t_rep = m.t[i0:i0 + n]
    r = strapdown(t_rep, m.f[i0:i0 + n], m.w[i0:i0 + n],
                  InsInit(tr.pos_phone_n[i0], tr.vel_phone_n[i0], tr.C_nb[i0] @ tr.C_bp))
    k = n - 1
    p_true = tr.pos_phone_n[i0 + k]          # evaluated at the TRUE time of sample k
    tn = _tangent(tr, np.array([i0 + k]))[0]
    e_s = float((r.p_n[0, k] - p_true) @ tn)
    ds = float(tr.s[i0 + k] - tr.s[i0])
    pred = eps * (2 * ds - tr.v[i0] * BUDGET_S)
    return {"scenario": key, "e_s_m": round(e_s, 3), "predicted_m": round(pred, 3)}


# ---------------------------------------------------------------------------------------------
# Full report
# ---------------------------------------------------------------------------------------------
ADDITIVITY = {
    # all-terms config -> the single-term configs whose variances should add up to it
    "all_ideal_bias": ["a_white", "a_bi", "a_scale_raw", "a_misalign_raw",
                       "vib_road_ideal", "vib_engine_ideal"],
    "all_free_bias": ["a_white", "a_bi", "a_scale_raw", "a_misalign_raw", "g_white", "g_bi",
                      "g_scale_raw", "g_misalign_raw", "vib_road_free", "vib_engine_free"],
}


def budget_report(seeds=200, scenarios=("S1", "S2", "S3", "S4", "S5")) -> dict:
    """Fixed 180 s INS-only budget and real-exit error per scenario and configuration, growth
    curves, variance shares and additivity. Paired: every configuration uses the same seeds."""
    from .frames import mount_rotation
    C_pb = mount_rotation(1.0, 12.0, 3.0).T
    e_f, e_p = C_pb @ [1.0, 0, 0], C_pb @ [0, 1.0, 0]
    out = {"seeds": int(seeds), "budget_s": BUDGET_S,
           "flags": {"oracle_init": True, "oracle_mount": True, "oracle_lever_arm": True,
                     "calibration_epoch": "tunnel_entry", "gnss": "none after the portal"},
           "analytic_rmse_m": {f"t{int(t)}s": analytic_rmse(NOMINAL.imu, e_f, e_p, t) for t in (60.0, 120.0, 180.0)},
           "analytic_rmse_m_gm1_reference": {
               f"t{int(t)}s": {k: v for k, v in analytic_rmse(configure("imu_bi_gm1").imu, e_f, e_p, t).items()
                               if "_bi" in k} for t in (60.0, 120.0, 180.0)},
           "scenarios": {}}
    for key in scenarios:
        r = run_scenario(key, np.arange(seeds))
        k180 = list(r.t_rel).index(BUDGET_S)
        kex = int(np.argmin(np.abs(r.t_rel - r.exit_rel)))
        sc = {"exit_after_portal_s": round(r.exit_rel, 2), "configs": {}}
        for name, x in r.e_s.items():
            sc["configs"][name] = {
                "fixed_180s_ins_only": stats(x[:, k180]),
                "real_exit": stats(x[:, kex]),
                "rmse_curve_m": {f"t{int(t)}s": float(f"{np.sqrt(np.mean(x[:, i] ** 2)):.4g}")
                                 for i, t in enumerate(r.t_rel) if t in CURVE_T},
            }
        add = {}
        for allname, parts in ADDITIVITY.items():
            v_all = float(np.var(r.e_s[allname][:, k180]))
            v_parts = {p: float(np.var(r.e_s[p][:, k180])) for p in parts}
            s_parts = sum(v_parts.values())
            add[allname] = {"var_all_m2": float(f"{v_all:.4g}"), "sum_single_var_m2": float(f"{s_parts:.4g}"),
                            "ratio": round(v_all / s_parts, 3),
                            "shares": {p: round(v / s_parts, 4) for p, v in v_parts.items()}}
        sc["additivity_180s"] = add
        out["scenarios"][key] = sc
    out["vibration_rate_sensitivity"] = vibration_rate_report()
    out["timestamp_clock_scale_1pct"] = {k: timestamp_check(k) for k in ("S1", "S5")}
    return out


def vibration_rate_report(key="S1", seeds=100) -> dict:
    """Vibration only, IMU at 100 and 200 Hz, ideal and free attitude, 180 s from the portal."""
    from dataclasses import replace
    from .profiles import ASSUMED
    from .scenario import build_scenarios
    out = {}
    for rate in (100.0, 200.0):
        tr = generate_truth(build_scenarios()[key], rate)
        cfg = configure("no_gaps")
        cfg = replace(cfg, imu=replace(cfg.imu, rate_hz=ASSUMED(rate, "vibration rate sensitivity")))
        src = ImuTruthSource.from_truth(tr)
        i0 = int(np.argmax(tr.in_tunnel))
        n = int(round(BUDGET_S / tr.dt)) + 1
        sl = slice(i0, i0 + n)
        F, W = [], []
        for s in range(seeds):
            m, _ = simulate_imu(src, cfg, Switches.only("vib_road", "vib_engine"), seed=s)
            F.append(m.f[sl]), W.append(m.w[sl])
        init = InsInit(tr.pos_phone_n[i0], tr.vel_phone_n[i0], tr.C_nb[i0] @ tr.C_bp)
        tn = _tangent(tr, np.array([i0 + n - 1]))[0]
        for att in ("ideal", "free"):
            r = strapdown(tr.t[sl], np.stack(F), np.stack(W), init,
                          attitude=(tr.C_nb[sl] @ tr.C_bp) if att == "ideal" else None)
            e = (r.p_n[:, -1] - tr.pos_phone_n[i0 + n - 1]) @ tn
            out[f"{int(rate)}hz_{att}"] = stats(e)
    return out
