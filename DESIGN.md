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

### 3.1 IMU: reference phone

Primary profile: **Samsung Galaxy Note10, STMicroelectronics LSM6DSR**, values from week-long static overlapped Allan deviation (Suvorkin et al., Sensors 2024, Table 4). ST does not publish these terms in the datasheet.

| Term | x | y | z | Approx. |
|---|---|---|---|---|
| Accel VRW [m/s/√s] | 9.99e-4 | 9.20e-4 | 8.58e-4 | ~0.055 m/s/√h |
| Accel bias instability [m/s²] | 6.63e-4 | 6.79e-4 | 1.62e-3 | ~70 µg horizontal |
| Gyro ARW [rad/√s] | 7.00e-4 | 6.01e-4 | 6.70e-4 | ~2.2 °/√h |
| Gyro bias instability [rad/s] | 8.84e-5 | 1.54e-4 | 2.26e-4 | ~18 to 47 °/h |

Secondary "low-end" profile: **InvenSense TDK MPU9250**, same paper, same method (Table 5). Gives a consistent sensitivity comparison.

Terms added on top of Allan values (not captured by a static, temperature-stable lab test):
- Turn-on bias (random per run)
- Slow temperature-driven bias drift (phone heats up in the car)
- Scale factor and small non-orthogonality
- Vehicle vibration (road/engine), band-limited
- Timestamp jitter / sample-rate switching from the Android pipeline (Bochkati & Pany, ISS 2021)

Rate: 100 Hz.

### 3.2 Barometer
- ~25 Hz, relative accuracy ~0.5 m equivalent, white noise + slow drift (weather, negligible over minutes).
- Optional cabin disturbance (A6 sensitivity).

### 3.3 GNSS
- 1 Hz position + Doppler velocity with reported accuracy.
- Entry: gradual degradation over a few seconds before full loss (SNR drop, growing accuracy).
- Exit: first fixes biased by multipath (10 to 50 m), Doppler recovers faster than position.

## 4. Back-of-envelope error budget (3 min outage, Note10 profile)

- VRW → position: ~1 m. Negligible.
- Accel bias drifting at BI level, perfectly calibrated at entry: ~10 m.
- Gyro pitch integrated freely at BI level → gravity leakage, error ∝ g·b·t³/6: **~1.4 km**.
- 1 mg residual accel error: ~160 m. 1° grade/pitch error (~17 mg): ~2.7 km.

**Conclusion:** sensor noise is not the limit. The limits are (a) calibration quality before the outage and (b) pitch/grade knowledge. Pitch must be held by NHC + barometer + map grade; gyro is used freely mainly for yaw.

### 4.2 Bias / mount-pitch observability
Along the forward axis, accel bias and mount pitch appear together as `b + g·sin(θ_mount)`. They cannot be separated, but with a rigid mount (A1) the lumped term stays constant in the tunnel, so we estimate the lumped term only.

## 5. Estimator roadmap (layered, each layer's contribution shown separately)

0. **Observability analysis first** (before writing the ESKF): linearised error model during the GNSS segment and the outage; which combinations of v_x, b_ax, pitch error, mount pitch and grade error are observable, and with which manoeuvres. NHC constrains v_y, v_z and helps attitude; it gives no v_x information.
1. **Reference algorithms:** (a) hold last GNSS speed, (b) raw accel integration, (c) oracle with true speed (upper bound, isolates map/geometry error).
2. **ESKF + NHC:** calibrate biases and mount while GNSS is good; in tunnel, vehicle-constrained INS.
3. **+ ZUPT:** detect stops in traffic, reset speed error.
4. **+ Map matching:**
   - Curve matching: gyro yaw rate vs map curvature → along-track fix.
   - Grade matching (optional, needs tunnel grade data, see A3): baro altitude vs grade profile → along-track fix (strong in V-profile tunnels). Reported against map grade error.
5. **Exit callout** uses the along-track uncertainty, not just the point estimate.
6. **Exit recovery:** innovation gating on first fixes, smooth blending, no visible jumps.

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
- IMM metrics: besides position error, report mode-switch latency and time spent in the wrong mode.

Decisions (2026-10-17):
- Grade is handled by map/baro matching, so the CV hypothesis matters most on the flat, straight tunnel (S1).
- Accel bias is estimated while GNSS is good; bias *drift* is ignored by the filter (BI-level drift ≈ 10 m over 3 min). Drift stays in the truth model so its effect is measured, not assumed.
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

Still not modelled: flat earth, earth rotation (also neglected in the filter), body roll, road-roughness heave (sensor vibration instead), tyre longitudinal slip.

Verification (`tests/test_truth.py`, 36 tests):
- Internal consistency: strapdown of the ideal phone IMU reproduces the phone trajectory within 0.13 cm on S1 to S5 and 1.9 cm on the mixed manoeuvre. Mutation checks show the test is sensitive: dropping the lever-arm acceleration gives 1.6 m error, dropping β̇ gives 8.4 m.
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

## 9. Real-device follow-up (optional)
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
