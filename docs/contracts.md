# Coordinate and Measurement Contracts

Single reference for frames, points, timing, error signs, state definitions and oracle
boundaries. Code and analysis must follow it; a change here is a design change (decision log +
CHANGELOG). Source of the numbers: `tunnelsim/frames.py`, `tunnelsim/scenario.py`,
`tunnelsim/sensors.py`. Sign conventions are tested in `tests/test_frames.py`.

Revision 2026-10-19 (review 4): attitude error sign fixed (+, left multiplication, Log map);
C_nb chain and independent mount/lever-arm information; calibration epoch; timing concepts
separated; error-free vs vibration tests separated; signed vs absolute metrics; fixed 180 s
INS-only budget; state vector contract (section 8).

## 1. Frames

| Frame | Axes | Origin |
|---|---|---|
| n | ENU, flat earth, no earth rotation | road start |
| r | road: x along tangent, y left, z up (normal to road) | — |
| b | vehicle body FLU: x forward, y left, z up | R (rear-axle centre) |
| p | phone (Android): x right of screen, y top of screen, z out of screen | IMU / antenna point P |

`C_ab` maps b-frame vectors to a: `v_a = C_ab v_b`; `C_ba = C_abᵀ`. Truth attitude chain:
`C_nb = Rz(ψ)·Ry(−γ)·Rz(−β)·Ry(−θ_d)`.

**Pitch direction trap.** In FLU a positive rotation about +y (left) is **nose down**
(`Ry(+θ)·x̂ = (cos θ, 0, −sin θ)`). Grade γ, suspension pitch θ_d and mount pitch are all
**nose-up positive** and therefore enter as `Ry(−angle)`.

Mount: `C_bp = C_off · C_BP_NOMINAL`, `C_off = Rz(yaw)·Ry(−pitch)·Rx(roll)` (small body-frame
offsets, pitch nose-up positive), true roll 1°, pitch 12°, yaw 3° [ASSUMED].

```
C_BP_NOMINAL = [[ 0, 0, -1],
                [-1, 0,  0],
                [ 0, 1,  0]]      (phone portrait, screen to the driver)
```

| Vehicle axis | Nominal phone axis | True mount, e_b in phone frame |
|---|---|---|
| forward +x_b (roll axis) | −z_p | (0.056, −0.207, −0.977) |
| left +y_b (pitch axis) | −x_p | (−0.998, −0.028, −0.051) |
| up +z_b (yaw axis) | +y_p | (−0.017, 0.978, −0.208) |

**Phone → body → nav chain.** The INS computes the phone attitude C_np; the body attitude
needs the mount:

```
C_nb = C_np · C_pb = C_np · C_bpᵀ
```

## 2. Points

- **R**: rear-axle centre; follows the road centreline; along-track position `s` and all
  along-track errors are defined at R.
- **P**: phone; IMU and GNSS antenna are both at P. Lever arm `r_b` (R → P, body frame),
  true (1.6, 0.15, 0.7) m [ASSUMED].
- `p_R = p_P − C_nb r_b = p_P − C_np C_bpᵀ r_b`.
- Three **independent** pieces of information: phone attitude C_np, mount C_bp, lever arm r_b.
  Each is either true (oracle) or assumed/estimated; giving one of them does not correct the
  others (an ideal phone attitude with a wrong mount gives a wrong C_nb).

## 3. Sensor biases and errors: frames

- IMU errors are generated **in the phone frame** (per phone axis): biases, white noise,
  scale factor S, misalignment M. Vibration is generated in the body frame and rotated to
  the phone frame before the sensor errors.
- A vehicle-axis quantity is the projection with the true mount: forward accel bias
  `b_∥ = e_xbᵀ b_p`, pitch-gyro bias `b_gy = e_ybᵀ b_g,p`; variances `σ² = eᵀ Σ_p e`.
- Full error-state models (v0.5): bias states are phone-frame biases, entering vehicle-axis
  equations through the mount matrix in the Jacobian. The reduced v0.4 model uses
  vehicle-axis projections (section 8).

## 4. Timing

| Concept | Symbol | Meaning | Modelled |
|---|---|---|---|
| Sample time | t_sample | true instant the sensor measured | yes |
| Reported time | t_reported | timestamp the app receives | yes |
| Arrival time | t_arrival | when the sample reaches the app (latency, batching) | no (later, for online fusion) |

| Error | Definition | Generator |
|---|---|---|
| Sampling jitter | t_sample varies around the nominal grid; t_reported = t_sample | `ts_jitter` (IMU) |
| Timestamp error | t_reported ≠ t_sample, random | not modelled |
| Clock-scale error | t_reported = t0 + (1+ε)(t_sample − t0) | `timing.ts_scale_error` (IMU) |
| Rate offset | nominal rate ≠ true rate, timestamps correct | `timing.rate_offset` (IMU) |
| GNSS delay | fix stamped t describes the antenna at t − δ; δ > 0 = older than the stamp; both on the phone clock (same scale) | `timing.imu_gnss_offset_s` |

- IMU sample k: `f̃_k`, `ω̃_k` are **instantaneous** values at t_sample,k (not interval
  averages). The INS uses `Δt_k = t_reported,k+1 − t_reported,k`.
- Barometer: t_reported = t_sample, no timing errors modelled.
- **Evaluation.** Errors are evaluated against the truth at the true time of the estimate
  (the harness maps t_reported → t_sample; it may, the estimator may not). All methods are
  compared on the **same set of true epochs**.

## 5. Error sign

Error = estimate − truth for every quantity (δs = ŝ − s, δv = v̂ − v, δb = b̂ − b, ...).

Attitude, nav-frame rotation vector, left multiplication:

```
Ĉ_np = Exp([δθ_n]×) · C_np,      δθ_n = Log(Ĉ_np C_npᵀ)∨     (frames.attitude_error_n)
```

Small angle: `Ĉ_np ≈ (I + [δθ_n]×) C_np`. A positive component is a positive (right-hand)
rotation of the estimate about that nav axis relative to the truth. Tested with positive and
negative roll, pitch and yaw perturbations. (The earlier `(I − [φ]×)` form corresponds to the
opposite error direction and is not used.) Scalar nose-up pitch error =
`−(rotation component about the body y axis)`, by the pitch trap in section 1.

## 6. Oracle boundaries (V1)

- `tunnelsim/ins.py` and every later estimator module import nothing from `truth`,
  `scenario` or `sensors` (only `meas`, `frames`, numpy/scipy). Tested.
- Only the analysis harness reads the truth, and only through **independent** flags:

| Flag | Values | What the harness supplies |
|---|---|---|
| `oracle_init` | bool | position, velocity, phone attitude at the INS start |
| `ideal_attitude` | bool | the true phone attitude sequence C_np(t) |
| `oracle_mount` | bool | true C_bp (else the assumed nominal + offsets) |
| `oracle_lever_arm` | bool | true r_b (else the assumed value) |
| `perfect_calibration` | `None` / `"bias"` / `"all"` | `"bias"`: turn-on + bias-instability state **at the calibration epoch**; `"all"`: additionally scale factor and misalignment. Never the future of the bias process (not physically realisable) |
| `calibration_epoch` | `"ins_start"` / `"tunnel_entry"` | when the bias is known: b(t_INS start) or b(t_entry) |
| `oracle_speed` | bool | true along-track speed (reference algorithm c) |

- Corrections are applied **by the estimator through its interface** (it receives b̂, Ŝ, M̂,
  mount, lever arm as parameters and corrects internally); the harness never pre-corrects
  sensor streams. All methods see the same raw measurements.
- Every result carries the flags that produced it.

## 7. Metrics

- **Localization error** e_s = ŝ − s_true at R. Reported per case as:
  `bias = E[e_s]`, `rmse = √E[e_s²]`, `p95_abs = Q0.95(|e_s|)` (the name always says abs;
  signed percentiles are not used for ranking), with bootstrap CI for p95_abs.
- **Fixed 180 s budget (INS-only propagation)**: the INS runs 180 s from the portal with no
  GNSS at all, even after a short tunnel's exit. For comparing error sources.
- **Real-exit error (operational)**: error at each scenario's own exit. Kept under a separate
  name even in a shared table.
- **Exit-location / callout error**: map exit position error (ε_L·L) and its timing effect;
  never charged to localization.
- σ_s(t) growth curves in addition to end values.
- Monte Carlo: 200 seeds (development), 1000 (final); paired seeds across methods.

Tests are kept apart:
- **Error-free mechanisation**: all sensor errors and vibration off; the INS reproduces the
  truth within numerical tolerances that shrink at the expected order with Δt.
- **Vibration sensitivity**: only vibration on; mean, rmse, p95_abs, sample-rate
  sensitivity. Vibration is injected as sensor signal, not truth motion, so the INS is not
  expected to reproduce the truth with it on.

## 8. State vector contract (reduced longitudinal model, v0.4 observability)

Vehicle longitudinal plane, linearised about the truth trajectory. Order fixed:

| # | State | Unit | Definition (estimate − truth) | Frame / sign |
|---|---|---|---|---|
| 0 | δs | m | along-track position error at R | + ahead |
| 1 | δv | m/s | forward speed error | body x, + forward |
| 2 | δθ | rad | body pitch attitude error | nose-up positive (= −rotation about +y_b) |
| 3 | δb_ax | m/s² | forward accel bias error, `e_xbᵀ δb_a,p` | vehicle x projection with the true mount |
| 4 | δb_az | m/s² | vertical accel bias error, `e_zbᵀ δb_a,p` | vehicle z projection |
| 5 | δb_gy | rad/s | pitch-rate gyro bias error, nose-up positive, `−e_ybᵀ δb_g,p` | vehicle y projection, sign flipped |
| 6 | δθ_m | rad | mount pitch error (phone relative to body) | nose-up positive |
| 7 | δγ | rad | road grade error (map/estimate − true) | nose-up positive |

Rules: a state is kept only if some measurement model in the analysis is actually sensitive
to it; scaled Gramians (by the state units) are used before reading condition numbers; the
exact linear equations (signs derived from section 1 and 5, not from schematic forms) are
written in the v0.4 observability commit and tested against finite differences of the
nonlinear model.
