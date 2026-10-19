# Through the Tunnel: Design Notes

Living document. Decisions, assumptions and open questions for the Pointr case study.
Source of truth for the final write-up and README.

---

## 1. Problem framing

- Phone-based position estimation through a GNSS outage of a few minutes (tunnel, 500 m to 5 km, single lane).
- The app must (a) keep showing the car, (b) call out an in-tunnel exit at the right moment, (c) recover cleanly at the exit portal.
- Key reframing: the navigation app already has the map, so inside a single-lane tunnel the problem reduces to **1D along-track distance**. Lateral error is removed by the map; the exit callout depends only on along-track error.
- Therefore the real question is: **how well do we know speed inside the tunnel?**

## 2. Assumptions

| # | Assumption | Rationale |
|---|---|---|
| A1 | Phone is fixed in a car mount (rigid, unknown but constant mounting angles) | Typical turn-by-turn use; makes NHC strong and lets mount pitch merge into accel bias (see 4.2) |
| A2 | Sensors: GNSS, accelerometer, gyroscope, barometer, ambient light sensor of an average phone | Task: smartphone sensors only |
| A3 | Map available: tunnel polyline, length, exit locations, speed limit. **Grade profile is NOT assumed** in the baseline | Horizontal geometry is standard nav-map content. Elevation is not: DEM-based elevation inside a tunnel gives the terrain or sea surface above it, not the roadway; tunnel incline is only sometimes tagged (e.g. OSM `incline`). Grade-map matching is an optional layer, reported as a function of map grade error (revised 2026-10-19) |
| A4 | No magnetometer | Tunnel structure and car body distort it |
| A5 | No BLE beacons, no OBD / vehicle speed, no camera | Not available to an average user / not used by nav apps |
| A6 | Barometer is not affected by airflow (closed cabin) | Baseline assumption. Cabin is not sealed (HVAC, piston effect), so a cabin pressure disturbance is added as a **sensitivity scenario** |
| A7 | Tunnels are single lane, 500 m to 5 km | Given by the task |

## 3. Sensor models

Full decision record with reasoning: `docs/decisions/2026-10-19-v0.3-sensors.md` (K1 to K9).

**Parameter tags.** Every number carries one of three tags:
- **[MEAS]** measured in a real experiment (e.g. Suvorkin Table 4),
- **[SPEC]** datasheet / manufacturer specification (e.g. LSM6DSR datasheet typ),
- **[ASSUMED]** chosen for the simulation, not validated.

**Policy (2026-10-19).** No real data is collected at this stage. Every [ASSUMED] parameter is swept rather than pinned to one value. `profiles.py` holds one nominal configuration; sensitivity scenarios are named overrides, and no sensitivity value is ever a default.

**Separation.** `tunnelsim/sensors.py` returns only measured fields (7.2); error components live in a separate object never passed to the estimator. Each error term can be switched off alone and has its own RNG stream (`SeedSequence.spawn`), so switching one term off does not change the samples of any other.

### 3.1 IMU: reference phone

Primary profile: **Samsung Galaxy Note10, STMicroelectronics LSM6DSR**, values from week-long static overlapped Allan deviation (Suvorkin et al., Sensors 2024, Table 4) [MEAS]. ST does not publish these terms in the datasheet.

| Term | x | y | z | Approx. |
|---|---|---|---|---|
| Accel VRW [m/s/√s] | 9.99e-4 | 9.20e-4 | 8.58e-4 | ~0.055 m/s/√h |
| Accel bias instability [m/s²] | 6.63e-4 | 6.79e-4 | 1.62e-3 | ~70 µg horizontal |
| Gyro ARW [rad/√s] | 7.00e-4 | 6.01e-4 | 6.70e-4 | ~2.2 °/√h |
| Gyro bias instability [rad/s] | 8.84e-5 | 1.54e-4 | 2.26e-4 | ~18 to 47 °/h |

The datasheet noise densities [SPEC] are lower: accel 60 µg/√Hz (measured VRW is 1.7x), gyro 5 mdps/√Hz (measured ARW is 8x). No cause is claimed (Android mode, bandwidth, filtering, Allan extraction method); the phone-measured value is used.

Secondary "low-end" profile: **InvenSense TDK MPU9250**, same paper, same method (Table 5) [MEAS]. Gives a consistent sensitivity comparison.

Rate: 100 Hz. Model per phone axis:
```
f̃ = (I + S_a + M_a)(f_p + vib_a) + b_a,on + b_a,BI(t) + n_a
ω̃ = (I + S_g + M_g)(ω_p + vib_g) + b_g,on + b_g,BI(t) + n_g
n ~ N(0, N²/Δt)
```
Vibration is a real specific force / rate, so it passes through the sensor scale and misalignment errors (R1).

| Term | Model | Values |
|---|---|---|
| Bias instability (K1, R2, R3) | Baseline: sum of 3 first-order GM, τ = 5, 50, 500 s [ASSUMED, derived from the target range], weights by NNLS so that white + GM matches `AVAR_target(T) = N²/T + (2ln2/π)B²` over 10–1000 s [ASSUMED range]. Reference: single GM, τ = 300 s, σ = 1.075·B | N, B [MEAS]. Averaging time of B not given in the paper (unknown). Verification: analytic ±10 % over 10–1000 s; one 24 h run (fixed seed) within max(10 %, 3σ(T)) of the model, σ(T) = measured run-to-run scatter (`tools/allan_scatter.py`) |
| Turn-on bias (K2) | Constant per run. Stream: `TYPE_GYROSCOPE_UNCALIBRATED`; Android's bias estimate not used | Accel 10 mg [SPEC, LA_TyOff]. Gyro nominal 0.05 °/s [ASSUMED], sweep 0.2, 1 °/s; ±1 dps [SPEC] is an upper bound only |
| Temperature (K3 → R4) | **Not modelled.** IMU assumed calibrated incl. temperature; residual bias estimated by the filter before the tunnel; in-tunnel temperature change neglected (not observable there) | Known limitation, size with earlier assumptions: ~9.6e-4 m/s² accel, ~8.5e-5 rad/s gyro bias change over 3 min, same order as BI. SPEC coefficients 0.1 mg/°C, 0.005 dps/°C kept for reference |
| Scale / misalignment (K4) | Sensor misalignment (MEMS → phone) is a separate matrix from the mount rotation (vehicle → phone) | Gyro scale 0.3 % [ASSUMED], ±1 % [SPEC] bound; accel scale 3e-3 [ASSUMED]. Curve-matching heading budget ε_ψ ≈ s_g·Δψ |
| Vibration (K5) | Generated at 100 Hz. Road: broadband ≈ 2–30 Hz, scaled with speed. Engine: narrow band ≈ 25 Hz at idle. No wheel tone (5.2) | [ASSUMED], two levels "calm" / "noisy". 1 kHz + LPF + decimation is an optional sensitivity test. **Out of scope:** vibration rectification error (VRE) |
| Timestamps (K6) | Timestamps carry the true measurement time. Scenarios: (1) sample-rate offset ±1–2 %, timestamps correct → no effect expected; (2) timestamp scale error → accumulating error; (3) estimator uses nominal dt → v0.4 | White jitter 0.5 ms, occasional gaps [ASSUMED]. IMU–GNSS time offset nominal 0, sweep 0–200 ms. "1 % rate error → 45 m (25 m/s, 180 s)" holds only for (2) and (3) |

### 3.2 Barometer (K7, implemented in `simulate_baro`)
- Model: `h̃ = h + b_h0 + b_h,GM(t) + n_h + d_cabin(t) + d_tunnel(s)`, 25 Hz, h = true phone altitude.
- Part: the Note10 barometer could not be confirmed. ST LPS22HB is quoted as a representative phone barometer (relative accuracy ±0.1 hPa at 25 °C over 800–1100 hPa, noise 0.0075 hPa rms with embedded filter); values derived from it stay [ASSUMED].
- b_h0: constant within a drive, σ = 0.85 m [ASSUMED, ≈ ±0.1 hPa]; not called a measured bias.
- n_h: white 0.1 m at 25 Hz [ASSUMED] (LPS22HB 0.064 m for comparison).
- b_h,GM: **nominal GM(0.1 m, 60 s)** [ASSUMED], the mildest correlated row of the K7 table, because white-only is known to be optimistic (5.1 note). White-only, GM(0.3 m, 300 s) and GM(0.3 m, 60 s) are named overrides.
- d_cabin: GM, off in the baseline (A6); override 0.3 m, τ 5 s [ASSUMED].
- d_tunnel = −(k_p/ρg)·(s − s_entry) inside the tunnel, zero at the entry: ventilation pressure gradient read as altitude [ASSUMED], nominal off, overrides 10 and 30 Pa/km (illustrative, magnitude not sourced). ρg = 11.8 Pa/m. Level at the exit k_p·L/ρg (S3, 3.5 km: 3.0 m at 10 Pa/km, 8.9 m at 30 Pa/km). After the exit it decays over distance, `d(s_exit)·exp(−(s − s_exit)/L_p)`, L_p = 200 m nominal, 50 and 500 m swept [ASSUMED, sensitivity parameter, not validated] (review 2, 4.2): a step would make portal detection artificially easy. The step is kept as a stress override (`baro_exit_step`). On a constant-grade tunnel it is indistinguishable from a grade offset: 10 Pa/km reads as a 0.085 % grade (≈ 8.3 mm/s²), 30 Pa/km as 0.25 % (≈ 25 mm/s², 2/3 of the S5 mean drift). If real gradients are of this size, they dominate the barometric grade error; a short literature check is worth doing before v0.6.
- Grade floor (least-squares slope over window T): white `σ_ḣ = σ√(12/(fT³))`; correlated (T ≪ τ) `σ_ḣ ≈ √(6q/(5T))`, q = 2σ²/τ. Verified in v0.3 against the exact value `p'Σp` (LS slope weights p, noise covariance Σ) and Monte Carlo (agree within 4 %). Accel equivalent at 25 m/s, exact:

| Noise | T = 5 s | 10 s | 20 s | 30 s | 60 s |
|---|---|---|---|---|---|
| white 0.1 m | 2.4 | 0.86 | 0.30 | 0.17 | 0.06 mm/s² |
| nominal: + GM(0.1 m, 60 s) | 4.2 | 2.5 | 1.7 | 1.3 | 0.83 |
| + GM(0.3 m, 300 s) | 5.3 | 3.4 | 2.3 | 1.9 | 1.3 |
| + GM(0.3 m, 60 s) | 10.6 | 7.2 | 4.9 | 3.9 | 2.5 |

  The T ≪ τ approximation is within 3 % up to T ≈ τ/6, 11 % at T = τ/2, 22 % at T = τ (it overestimates). With correlated noise, longer windows help much less than with white noise (×1/√T instead of ×1/T^1.5).
- Grade from baro depends on speed (sin γ ≈ ḣ/v): error ≈ γ·δv/v. Negligible on a flat tunnel; on a 5 % grade, 1 % speed error gives 0.05 % grade error ≈ 4.9 mm/s², above the white-noise floor.

### 3.3 GNSS (K8, K9, implemented in `simulate_gnss`)
- 1 Hz fixes of the phone antenna in local ENU (no lat/lon, flat earth), position and Doppler velocity with reported 1σ per axis. NaN where a quantity is not available.
- Open sky [ASSUMED]: position GM per axis, 3 m horizontal, 6 m vertical, τ 30 s. Velocity = white + GM per axis; **nominal split 0.07 + 0.07 m/s (τ 10 s), total ≈ 0.10 m/s**; vertical ×1.5; components independent, no position–velocity error correlation (K8 nominal).
- K8 families as overrides: total fixed ≈ 0.10 m/s (white-only 0.10/0; correlation-heavy 0.03/0.095) and total doubled (0.14/0.14).
- Why the split matters (exact LS-slope floor `p'Σp`, horizontal axis, 1 Hz): accel-bias calibration floor over a 60 s window is 7.3e-4 m/s² with white-only noise, **1.8e-3 m/s² nominal**, 2.3e-3 correlation-heavy, at the same total rms. With correlated velocity noise the calibration floor is 2.7× bias instability, so the entry calibration, not the IMU, may set the S1 limit. To be measured in v0.4.
- Entry: sigmas ramp linearly from 1× to 5× over the 4 s before the portal [ASSUMED]. Nominal: fixes stop at the portal. Scenario `gnss_coasting`: 5 s of fake fixes propagated from the receiver's own last fix (position + velocity·Δt), reported accuracy growing 3 m/s [ASSUMED].
- Exit: velocity from 1 s, position from 2 s after the portal [ASSUMED] (on the 1 Hz grid: 1.5 and 2.5 s on S1). Nominal exit bias: random horizontal direction, 30 m at the first position fix, decaying with 8 s [ASSUMED]; overrides 10 and 50 m. Scenario `gnss_lagging`: bias −v·1.5 s along track, same decay; the two mechanisms are never mixed. Velocity noise ×3 at the first velocity fix, same decay [ASSUMED].
- Reported accuracy = ratio × model σ (incl. entry degradation), independent of the realised error; the exit bias is not reported (receivers underestimate it). Ratios 1 (nominal), 0.7, 1.5. Verified: mean normalised squared error = 1/ratio² within 3 %.
- IMU–GNSS time offset (K6): the fix stamped t describes t − δ; nominal 0, overrides 0.1 and 0.2 s. Verified: position error v·δ, velocity error a·δ.

## 4. Back-of-envelope error budget (3 min outage, Note10 profile)

Corrected 2026-10-19 (axis mapping, `docs/decisions/2026-10-19-v0.4-plan.md`): the forward axis is phone −z and the pitch axis phone −x (portrait mount); the first version used phone x/y values. Numbers below are **"position error if a constant bias of BI size were present"** on the nominal axes, a scale estimate, not the RMS/P95 of the GM-realised error; true-mount and realised values come from the v0.4 experiments.

- VRW → position: N·√(t³/3) ≈ 1.2 m (z axis). Negligible.
- Constant BI-sized forward accel bias (z, 1.62e-3 m/s²), b·t²/2: **~26 m** (was ~10 m, x axis).
- Constant BI-sized pitch-gyro bias (x, 8.84e-5 rad/s) integrated freely → gravity leakage, g·b·t³/6: **~0.84 km** (was ~1.4 km, y axis).
- 1 mg residual accel error: ~160 m. 1° grade/pitch error (~17 mg): ~2.7 km.

### 4.3 Measured budget (v0.4 step 3, 200 seeds, `reports/0.4.0.dev0/budget_200.json`)

Fixed 180 s INS-only propagation from the portal, oracle initial state, true mount and lever
arm, calibration at the tunnel-entry epoch; along-track error at R; rmse / p95_abs in metres.
Single-term rows on S1 match the closed forms within 15 % (slow test).

| Configuration | S1 | S2 (curves) | S3 (V) | S4 | S5 |
|---|---|---|---|---|---|
| accel white (ideal attitude) | 1.2 / 2.2 | 1.2 / 2.2 | 1.2 / 2.2 | 1.2 / 2.2 | 1.2 / 2.2 |
| accel BI, entry bias known and held | 33 / 62 | 28 / 53 | 29 / 57 | 33 / 67 | 32 / 62 |
| accel BI, no calibration | 28 / 59 | 24 / 47 | 28 / 55 | 30 / 55 | 30 / 56 |
| accel scale 3e-3, uncalibrated | 140 / 273 | 117 / 229 | 126 / 246 | 137 / 264 | 139 / 270 |
| accel misalignment 0.1°, uncalibrated | 252 / 505 | 240 / 462 | 254 / 507 | 252 / 506 | 252 / 505 |
| gyro ARW (free attitude) | 681 / 1260 | 645 / 1160 | 682 / 1260 | 681 / 1260 | 680 / 1260 |
| gyro BI, entry bias known and held (free) | 983 / 1790 | 1300 / 2640 | 951 / 1770 | 958 / 1760 | 978 / 1940 |
| gyro misalignment (free) | 0 | 82 / 159 | 0.1 | 0 | 0 |
| vibration road+engine, ideal / free | 0.19 / 14 | 0.19 / 14 | 0.16 / 13 | 0.14 / 11 | 0.21 / 16 |
| **all terms, ideal attitude, all constants + entry bias known** | **33 / 62** | **28 / 54** | **29 / 57** | **33 / 67** | **32 / 62** |
| all terms, ideal attitude, entry bias known (S, M not) | 293 / 576 | 270 / 562 | 289 / 559 | 290 / 558 | 291 / 550 |
| all terms, free attitude, entry bias known | 1260 / 2400 | 1510 / 2860 | 1240 / 2350 | 1280 / 2430 | 1280 / 2410 |

Readings:
- **Free pitch is unusable**: ~1.2–1.5 km at 180 s even with the entry bias known; 64 % of the
  variance is gyro BI, 31 % ARW (S1). Pitch must be constrained (NHC, barometer); this is the
  measured version of the section 4 conclusion.
- **With ideal attitude the residual is the accel bias change after the entry**: ~29–33 m
  rmse, 53–67 m p95_abs. Uncalibrated scale and misalignment dominate the 290 m row (misalignment
  75 %, scale 23 %), but on a constant grade they act as a constant forward offset (gravity
  through S, M) that a GNSS calibration estimates as part of the lumped bias, so the "all known"
  row is the relevant reference; on graded roads (S3) the lumped term changes with the grade.
- **Knowing the entry bias and holding it does not beat no calibration** for this BI model
  (33 vs 28 m): the fast components (τ = 5, 50 s) revert to their mean, so a held b(t_entry) adds
  error. Using the entry value through the GM model helps (analytic, 180 s, accel / gyro):
  only the total b(0) known, conditional mean `b̂(t) = b(0)·Σσᵢ²e^(−t/τᵢ)/Σσᵢ²`: **21.6 m / 703 m**;
  each component bᵢ(0) known separately (oracle lower bound, not achievable from one bias
  value): 15.8 m / 507 m. (Corrected after review 5: 15.8 m had been presented as the
  achievable value.) Single-GM model: 11.6 m either way.
- **Forward-axis BI sensitivity**: the forward axis is phone z, whose B (1.62e-3) is 2.4× x/y
  and was measured with z vertical; Suvorkin report a bending of the Allan curve on whichever
  axis was vertical (cause not given; they state it does not change the coefficients). If the
  high value belongs to the vertical orientation rather than the z axis, the forward BI in the
  car is nearer x/y: held-bias error 13.0 m instead of 30.9 m. Unsettled; kept as nominal,
  sensitivity to be added. The ~30 m headline therefore rests on two pessimistic choices (3-GM
  shape, z-axis B), each worth ~2.4×.
- Row names: "accel BI, no calibration" = BI-only with zero correction, not an uncalibrated
  phone (turn-on, scale, misalignment are off in that row).
- **Model-shape sensitivity**: the paper gives one B per axis, not the Allan curve shape. With
  the single-GM reference model (τ = 300 s) the held-bias accel error is 12.5 m instead of 30.9 m
  (analytic, 180 s). The realised drift depends on an assumption the data cannot settle; both are
  reported.
- Gyro misalignment couples yaw rate into pitch in curves (S2: 82 m free); zero on straight roads.
- Vibration: zero-mean in the accel channel (0.2 m), but through the attitude nonlinearity a
  nonzero mean in free attitude (1.4–1.9 m over 180 s); not sensitive to 100 vs 200 Hz.
- Additivity of variances: ratio 1.01–1.03 with ideal attitude, 1.04–1.11 with free attitude
  (gyro–accel interaction), as expected.
- Real-exit errors (operational, own name): S4's exit is 337 s after the portal; with ideal
  attitude and entry bias known its rmse is 1025 m, so long stop-and-go tunnels need the speed
  cues, not the INS.

**Conclusion:** sensor noise is not the limit. The limits are (a) calibration quality before the outage and (b) pitch/grade knowledge. Pitch must be held by NHC + barometer + map grade; gyro is used freely mainly for yaw.

### 4.2 Bias / mount-pitch observability
Along the forward axis, accel bias and mount pitch appear together as `b + g·sin(θ_mount)`. They cannot be separated, but with a rigid mount (A1) the lumped term stays constant in the tunnel, so we estimate the lumped term only.

## 5. Estimator roadmap (layered, each layer's contribution shown separately)

Version scope (decided 2026-10-19):

| Version | Content |
|---|---|
| v0.3 | Sensor generators + sensor-level verification only (Allan/PSD, baro grade floor, timestamps, GNSS timing, reported accuracy). Scenario variants are defined and tested here; sweeps run in v0.4 |
| v0.4 | Temperature residual stress scenario, truth only, off in nominal: `b_T = ḃ_T·(t − t_entry)`, ḃ_T from the R4 size (~1e-3 m/s² per 3 min) [ASSUMED] (review 2, 4.1); strapdown mechanisation; along-track error at the exit with each error term switched on alone, ideal attitude, perfect entry calibration (measured version of the section 4 budget, recomputed); reference algorithms (item 1); observability analysis (item 0) |
| v0.5 | ESKF + NHC + ZUPT (items 2, 3), NEES/NIS |
| v0.6 | IMM (5.1), map/baro grade layer (item 4) |
| v0.7 | Monte Carlo, exit callout, exit recovery (items 5, 6) |
| v0.8 | Browser UI (Pyodide) |

Principles kept across versions: sensor truth, error components and estimator inputs are strictly separate; every physical effect has a sourced or explicitly assumed parameter; nominal and stress scenarios are not mixed; weak-observability scenarios (S1, S5) are reported in every version. Evaluation criterion: not that an algorithm does better, but that we can explain mathematically and experimentally which information it uses to do better.

0. **Observability analysis first** (review 2: no 15-state ESKF before it; cases: constant speed, gentle braking, changing grade, each with GNSS and after loss; plus the all-error-terms Monte Carlo distribution of along-track error over a 3 min outage) (before writing the ESKF): linearised error model during the GNSS segment and the outage; which combinations of v_x, b_ax, pitch error, mount pitch and grade error are observable, and with which manoeuvres. NHC constrains v_y, v_z and helps attitude; it gives no v_x information.
1. **Reference algorithms:** (a) hold last GNSS speed, (b) raw accel integration, (c) oracle with true speed (upper bound, isolates map/geometry error).
2. **ESKF + NHC:** calibrate biases and mount while GNSS is good; in tunnel, vehicle-constrained INS.
3. **+ ZUPT:** detect stops in traffic, reset speed error.
4. **+ Map matching:**
   - Curve matching: gyro yaw rate vs map curvature → along-track fix.
   - Grade matching (optional, needs tunnel grade data, see A3): baro altitude vs grade profile → along-track fix (strong in V-profile tunnels). Reported against map grade error.
5. **Exit callout** uses the along-track uncertainty, not just the point estimate.
6. **Exit recovery:** innovation gating on first fixes, smooth blending, no visible jumps.

### 5.0 Bias calibration design note (for v0.5, decided 2026-10-19)
- **Slow bias estimation.** Accel/gyro biases are estimated slowly while GNSS is good. Correlated GNSS velocity noise is beaten by averaging time, not by detection: exact accel-bias calibration floor (LS slope, nominal GNSS velocity noise) 1.8e-3 / 7.3e-4 / 4.2e-4 / 2.0e-4 m/s² at 60 / 120 / 180 / 300 s, vs 7.3e-4 / 2.6e-4 / 1.4e-4 / 6.6e-5 with white-only noise of the same rms. Correlation costs roughly a doubled window; from ~180 s the floor is below accel bias instability (6.6e-4). The sensor-level floor (`sensors.gnss.calibration_floor`) stays as the reference the filter must beat.
- **Robustness over optimality.** The filter does not have to be optimal; some optimality is traded for robustness. Q (and R) restrict the rate of change of the **bias states only**, not of all states. Reason: a wrong bias estimate costs more the longer the tunnel, δs = δb·t²/2 (1e-3 m/s²: 1.8 m at 60 s, 16 m at 180 s, 45 m at 300 s), so a bias that follows GNSS disturbances is worse than a bias that is slightly late.
- **GNSS velocity correlation in the measurement model.** A filter that treats 1 Hz velocities as independent believes its calibration is 2.5–3× better than it is (180 s: claims 1.4e-4, real 4.2e-4). Baseline: inflate R to the equivalent white density of the correlated noise (nominal ≈ 0.32 m/s at 1 Hz); alternative: GM velocity-error state.
- **Detection.** Innovation gating catches exit multipath and pre-portal degradation; it cannot see correlated open-sky noise (that is normal statistics) and it passes coasting fixes (consistent with constant velocity by construction), so coasting needs its own detector (fix lies exactly on the CV extrapolation, reported-accuracy growth pattern, map portal position). Optimistic reported accuracy weakens gating.
- **Checks.** NEES/NIS overall and for the bias states; calibration error at 60 / 120 / 180 s against the table above. If bias Q is deliberately below the true wander, the bias covariance is optimistic by design: the in-tunnel σ_s must then account for it (e.g. bias covariance inflated at the portal to the true expected error), or the exit callout uncertainty is wrong.
- Open for v0.4 observability: slow calibration estimates the lumped term (bias + g·grade + mount pitch) on the road before the tunnel; a grade change at the portal changes the lumped term.

### 5.1 Motion-mode idea (IMM), under discussion
Instead of always integrating accel: if mean along-track accel (grade-compensated) is ~0, hold entry speed (constant-velocity hypothesis). Formalized as IMM over motion modes:
- **Stationary** → ZUPT
- **Cruise (CV)** → speed held, accel used only as evidence for mode probability
- **Maneuver (CA)** → integrate bias-corrected accel

Open questions: undetectable slow speed drift in CV mode, detection latency on braking, behavior in stop-and-go.

Review notes (2026-10-19):
- IMM does not create information. Its benefit is only in how it uses the same measurements; the detectability floor is set by what the accelerometer can tell apart.
- S5 drift (90 → 82 km/h in 60 s): mean −0.037 m/s², peak −0.069 m/s². If CV holds entry speed for 60 s: 67 m along-track error, then +2.2 m/s × t afterwards.
- That drift is **detectable in principle**: residual bias after GNSS calibration is ~7e-4 m/s² (BI level), ~2 % of the drift. The real floor is **grade knowledge**: a 0.1 % grade error is 9.8e-3 m/s², 0.3 % is 2.9e-2 m/s², the same size as the drift.
- Grade source therefore matters more than bias: barometer vertical rate, if baro noise were white at 25 Hz / 0.1 m, gives grade to ~0.01 % over a 10 s window (~1 mm/s² equivalent). Real baro noise is correlated and the cabin can disturb it; to be measured in the sensor step, not assumed.
  - Update 2026-10-19 (K7): the ~0.01 % figure holds **only for white noise**. With a correlated component the floor rises to 2.5–7.1 mm/s² (GM 0.1–0.3 m, τ 60–300 s [ASSUMED]); the worst case is ~20 % of the S5 mean drift. No real recording at this stage, so the correlation level is swept, not chosen (3.2).
- IMM metrics: besides position error, report mode-switch latency and time spent in the wrong mode.

Decisions (2026-10-17):
- Grade is handled by map/baro matching, so the CV hypothesis matters most on the flat, straight tunnel (S1).
- Accel bias is estimated while GNSS is good; bias *drift* is ignored by the filter (BI-level drift ≈ 10 m over 3 min). Drift stays in the truth model so its effect is measured, not assumed.
  - Measured 2026-10-20 (v0.4, 4.3): the drift after the entry costs ~29–33 m rmse at 180 s with the 3-GM model (12.5 m with the single-GM reference), not ≈ 10 m; the 10 m figure used the wrong axis and a constant-bias picture.
- Slow speed change: multi-time-scale accel filters (fast / medium / slow). Formal equivalent: IMM over Singer acceleration models (first-order Gauss-Markov accel) with different time constants τ. Trade-off: longer τ lowers the detection floor but adds delay.

### 5.2 Bias-free speed cues (candidates for the flat, straight tunnel)
The forward axis is the hard problem: on S1 the map gives no features. These cues do not depend on accel bias. Each is calibrated against GNSS before the tunnel.

| Cue | Sensor | Principle | Calibrated before tunnel | Main risk |
|---|---|---|---|---|
| Wheel rotation frequency | Accel (vertical), 100 Hz | Tire/wheel non-uniformity gives a peak at f = v / (2πr) (~10 Hz at 20 m/s) | Effective rolling radius r | Peak may be weak on smooth asphalt |
| Wheelbase delay | Accel (vertical) | Front and rear axle hit the same joint/bump; Δt = L / v | Effective wheelbase L | Needs discrete bumps (tunnel segment joints) |
| Pitch dynamics (squat/dive) | Gyro y | Pitch angle ≈ K · a_x, pitch rate marks accel/brake onset | Vehicle pitch gain K | Small for gentle drift; best as IMM mode-switch trigger, not magnitude |
| Tunnel lighting | Ambient light sensor | Light fixtures at regular spacing: f = v / d; portal = sharp light drop | Spacing d (not always in map) | Low sensor rate, spacing knowledge |
| Portal event | Ambient light sensor | Entry/exit timestamp → along-track fix at known portal positions | none | none significant |
| Traffic speed prior | Map service | Live/historical segment speed as weak prior | none | Not individual car speed |

Honesty note: we design these sensor models ourselves, so results can be self-fulfilling. Each cue gets conservative parameters and an explicit failure scenario (cue absent or weak).

Status (2026-10-17):
- Tunnel lighting: **rejected** (spacing not known, weak justification).
- Portal event: **deferred**, not in the first version.
- Wheel frequency, wheelbase delay: **deferred**. Only added if a real phone recording shows the cue exists; not simulated on assumption alone.
- Pitch dynamics: kept as IMM mode-switch trigger.
- First version for S1 therefore relies on: GNSS calibration + CV/IMM + ZUPT. Expected limit on S1: entry speed error and undetected slow drift. This is reported as the honest "where it breaks" result.

## 6. Scenarios

Different road geometries, each must be a scenario:
- **S1 Straight, constant grade, no curves** (hardest: no map features)
- S2 Curved tunnel
- S3 V-profile tunnel (undersea type)
- S4 Stop-and-go traffic inside tunnel
- S5 Braking / slow speed drift inside tunnel
- Sensitivity: low-end IMU profile, cabin pressure disturbance, tunnel length 500 m to 5 km

## 7. Simulator architecture

1. Scenario definition: road as curvature κ(s) + grade profile, tunnel start/end, exits, speed profile.
2. Truth generator: s(t), v(t), a(t) → 3D pose; specific force (incl. v²κ, gravity on grade, vibration) and angular rate in vehicle frame; rotated by mount misalignment into phone frame.
3. Sensor simulation (section 3).
4. Estimators (section 5).
5. Visualization.

### 7.1 Truth model (implemented: `tunnelsim/truth.py`)
Frames: n = ENU (flat earth), r = road (tangent, left, up), b = body FLU, p = phone (x right, y up, z out of screen).
- Car follows the centreline with no sideslip: `v_n = v·t_n(s)`, `a_n = a·t_n + v²(κ ∂t/∂ψ + γ' ∂t/∂γ)`.
- Road attitude `C_nr = Rz(ψ)·Ry(−γ)`; suspension pitch `θ_d = K·a` (K ≈ 1.5°/g), `C_rb = Ry(−θ_d)`.
- Angular rate: `ω_r = ψ̇[sin γ, 0, cos γ] − γ̇[0,1,0]`, `ω_b = C_br ω_r − θ̇_d[0,1,0]`, with `ψ̇ = vκ`, `γ̇ = vγ'`.
- Specific force: `f_b = C_bn (a_n − g_n)`; phone: `f_p = C_pb f_b`, `ω_p = C_pb ω_b`.
- Phone mount: portrait, screen to driver, plus body-frame offset (default roll 1°, pitch 12°, yaw 3°).
- Speed: cruise + position-triggered events, minimum-jerk (quintic) ramps, so a and jerk are continuous.
- Grade: minimum-jerk transitions between breakpoints, so vertical curvature is continuous.

Revision 2026-10-19 (after review): the truth no longer satisfies the filter's ideal assumptions.
- Reference point R = rear-axle centre (follows the path). Along-track error is evaluated at R.
- Body sideslip `β = −k_β·a_lat` (k_β = 0.4°/(m/s²), linear bicycle model, [ASSUMED]); attitude chain `C_nb = Rz(ψ)Ry(−γ)·Rz(−β)·Ry(−θ_d)`; rate adds `B^T[0,0,−β̇]`.
- Phone lever arm r_b = (1.6, 0.15, 0.7) m from R [ASSUMED]: `v_P = v_R + C_nb(ω×r)`, `a_P = a_R + C_nb(ω̇×r + ω×(ω×r))`; the IMU senses `f_b = C_bn(a_P − g_n)`.
- Speed events trigger on the predicted current position (fixes a one-sample trigger delay).

Measured effects (truth, no sensor errors):
| | S2 curve | S4 stop-and-go | MIX (brake + clothoid + grade) |
|---|---|---|---|
| max lateral body velocity at phone (NHC-y violation) | 0.11 m/s | 0 | 0.11 m/s |
| max vertical body velocity at phone (NHC-z violation, pitch × lever arm) | 0 | 0.11 m/s | 0.13 m/s |
| max sideslip | 0.49° | 0 | 0.75° |

On a steady R = 400 m curve at 80 km/h, sideslip leaks lateral acceleration into the forward axis: ~1.1 mg. That is larger than the accel bias instability (~70 µg), so a filter that ignores sideslip mis-reads curves as along-track acceleration.

Still not modelled: flat earth, earth rotation (also neglected in the filter), body roll, road-roughness heave (sensor vibration instead), tyre longitudinal slip. Sensor side (section 3): IMU temperature drift (R4), vibration rectification error.

Revision 2026-10-19 (v0.3, found by the IMU resampling test): the algebraic sideslip `β = −k_β·a_lat` made β̇ step at clothoid ends (κ̇ steps), so the truth angular acceleration was an impulse whose height grew with the sample rate (0.18 / 0.38 / 0.77 m/s² at 100 / 200 / 400 Hz) while its area stayed r·Δω ≈ 4 mm/s. A real car does not do this (yaw/sideslip dynamics, tyre relaxation). Fix: first-order lag `τ_β β̇ + β = −k_β·a_lat`, τ_β = 0.2 s [ASSUMED], exact first-order-hold discretisation with β̇ taken from the ODE (not differenced), so attitude and rate stay consistent; τ_β = 0 reproduces v0.2. Effects:
- Resampling error at velocity level, S2: ~1 mm/s per joint → 3e-5 m/s over the run.
- Strapdown consistency: S2 1.3 mm → 0.03 mm, MIX 1.9 cm → 0.17 mm (the v0.2 residuals came from the same impulse).
- Sideslip and NHC metrics barely move (MIX max β 0.751° → 0.748°, lateral NHC violation 0.106 → 0.108 m/s).
- Mutation check re-measured: dropping β̇ still gives 8.3 m on MIX (was 8.4 m).

Verification (`tests/test_truth.py`, 36 tests):
- Internal consistency: strapdown of the ideal phone IMU reproduces the phone trajectory within 0.13 cm on S1 to S5 and 1.9 cm on the mixed manoeuvre (v0.2; with the sideslip lag of v0.3: ≤ 0.13 cm on S1 to S5, 0.017 cm on MIX). Mutation checks show the test is sensitive: dropping the lever-arm acceleration gives 1.6 m error, dropping β̇ gives 8.4 m.
- Independent references: centreline vs closed-form circle and Fresnel clothoid (< 1 mm), altitude vs adaptive quadrature (< 1 mm), centreline vs adaptive RK (rtol 1e-10). **Centreline discretisation error reported separately: 0.05 mm (S2), 0.006 mm (S3), 0.07 mm (MIX)**, so it cannot be confused with navigation error.
- Boundaries and events: start state, end of road, monotonic s, each ramp starts within one sample of its trigger point, no acceleration steps.
- Lesson recorded: with a piecewise-linear grade, even ideal strapdown drifted 7 m on S3, from pitch-rate steps at grade breakpoints leaking gravity. Same mechanism as section 4, at a smaller scale.

### 7.2 Truth / estimator separation (model-mismatch policy)
The estimator never imports truth objects. It receives only:
- sensor streams (section 3),
- a **map model** derived from the road with explicit errors: tunnel length scale error (±0.5 %), exit position error, curvature smoothed and resampled, grade either absent (baseline) or truth + bias + noise (optional layer),
- wrong initial values: mount angles, lever arm unknown, sideslip not modelled in the filter.
Every result is reported together with the mismatch settings that produced it.

**Stack:** pure-Python core package (numpy), run in the browser via **Pyodide** (single static page, GitHub Pages link, no install). Same package runs in notebook/CLI for Monte Carlo. Python computes a full run and returns arrays; JS (canvas + Plotly) only plays it back. Parameter change → rerun in seconds, supports live changes during the interview.

**UI:** top-down map with shaded tunnel, true vs estimated car, along-track uncertainty bar, error vs time, sensor traces, exit callout marker; scenario selector, parameter sliders, play/pause/speed; Monte Carlo tab with histograms.

## 8. Evaluation metrics
- Along-track error vs time and at tunnel exit
- Exit callout timing error distribution (Monte Carlo)
- Recovery: position jump size at exit, time to converge
- Per-layer contribution (section 5)
- **Consistency:** NEES (with truth) and NIS (innovations), along-track σ_s vs realised error, especially at the exit callout
- IMM: mode-switch latency, time in wrong mode
- Grade/baro: experiments that separate baro bias, cabin disturbance and map grade error

Barometer as along-track measurement: `z = h_map(s) + b_h + n`, `∂h/∂s = sin γ(s)`. On a constant grade b_h and s error are indistinguishable; only grade *changes* (S3) separate them. b_h can be calibrated at the portal from GNSS horizontal position + map altitude (σ_s · sin γ at entry).

## 9. Real-device follow-up (later, optional; not part of v0.3 to v0.8, decision 2026-10-19)
- Record own trace (e.g. Sensor Logger) in an Istanbul tunnel; portal positions and GNSS re-acquisition as checkpoints.
- Static recording for own Allan deviation to validate section 3.1.

## 10. References
- Suvorkin V. et al., *Assessment of Noise of MEMS IMU Sensors of Different Grades for GNSS/IMU Navigation*, Sensors 24(6):1953, 2024. https://pmc.ncbi.nlm.nih.gov/articles/PMC10975684
- Bochkati M., Pany T., *Does the Android Operating System Provide what the MEMS-IMU Manufacturers Promise?*, DGON ISS 2021.

## 11. Versioning
- Git, one tag per version (`vMAJOR.MINOR.PATCH`); minor = a new pipeline stage or a model change that moves the numbers.
- Every version: `CHANGELOG.md` entry + `reports/<version>/metrics.json` from `tools/snapshot.py`; `--diff` shows which numbers moved.
- A change that moves a metric must say why in the changelog, so real improvements can be told apart from ones that only exploit simulation conditions.

## 12. Decision log
- 2026-10-17: Python core in HTML (Pyodide). Synthetic simulation primary; real data only for noise realism.
- 2026-10-17: Fixed mount; no mag, BLE, OBD, camera. Map assumed available.
- 2026-10-17: IMU profiles from Suvorkin et al. 2024 (Note10 primary, MPU9250 low-end).
- 2026-10-17: Straight, no-grade, no-curve tunnel kept as a mandatory scenario.
- 2026-10-17: Truth generator implemented and verified by strapdown test. Scenarios S1 to S5 defined (S3 tunnel 3.5 km, symmetric V).
- 2026-10-17: Sensor profile parameters beyond Allan terms are marked [ASSUMED] in `tunnelsim/profiles.py`.
- 2026-10-19: External review applied. Truth adds sideslip, lever arm, event-timing fix; 36 tests incl. independent geometry references and mutation checks. Grade profile no longer assumed in the map (A3). Estimator/truth separation policy (7.2). Observability analysis, reference algorithms incl. oracle, NEES/NIS added to the plan.
- 2026-10-19: v0.3 sensor decisions K1 to K9 accepted (`docs/decisions/2026-10-19-v0.3-sensors.md`). Three tags [MEAS]/[SPEC]/[ASSUMED]; no real data collected, unsourced parameters swept; nominal vs named sensitivity overrides; per-term RNG streams; version scope v0.3 to v0.8 (section 5). From now on every decision and review is committed; the repository is the only sync channel between sessions.
- 2026-10-20: v0.4 step 3 measured budget (4.3): free pitch unusable (~1.2–1.5 km at 180 s); ideal-attitude residual ~30 m from the accel bias change; held entry bias no better than none for the 3-GM model, GM prediction halves it; BI model shape changes the result 2.5×.
- 2026-10-19: v0.4 plan accepted with review 3 (`docs/decisions/2026-10-19-v0.4-plan.md`): axis-mapping correction of section 4, independent oracle flags, LS-ideal vs LS-practical calibration, approach road lengthened to 200 s of GNSS, structural + practical observability, 200/1000 seeds, fixed-180 s and real-exit reports, coordinate contracts (`docs/contracts.md`).
- 2026-10-19: Bias calibration design note (5.0): slow bias estimation, robustness over optimality, Q/R restrict only the bias states' rate of change (wrong bias costs δb·t²/2), R inflated for GNSS velocity correlation, separate coasting detector.
- 2026-10-19: GNSS generator (K8, K9) implemented; unspecified details chosen [ASSUMED] and listed in 3.3 (velocity split, degradation ramp, coasting 5 s, reacquisition 1/2 s, lagging-receiver mechanism, exit Doppler factor).
- 2026-10-19: External review 2 applied: barometer exit transition L_p (nominal 200 m, step kept as stress), nominal baro GM(0.1 m, 60 s) confirmed, temperature residual as a truth-only stress scenario in v0.4 (nominal unchanged, no filter state), dev version + report overwrite guard.
- 2026-10-19: Truth: first-order lag on sideslip, τ_β = 0.2 s [ASSUMED] (option (a) over smoothing clothoid joints or accepting the artifact): the algebraic model put a rate-dependent impulse into the truth angular acceleration (7.1).
- 2026-10-19: IMU structure review (R1 to R4): vibration before sensor errors; Allan target = paper's parametric AVAR over 10–1000 s with τ = 5, 50, 500 s; verification analytic + one 24 h run; IMU temperature drift not modelled (calibration assumed, not observable in tunnel).
