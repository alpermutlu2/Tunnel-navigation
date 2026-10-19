# Coordinate and Measurement Contracts

Single reference for frames, points, timing, error signs and oracle boundaries. Code and
analysis must follow it; a change here is a design change (decision log + CHANGELOG).
Source of the numbers: `tunnelsim/frames.py`, `tunnelsim/scenario.py`, `tunnelsim/sensors.py`.

## 1. Frames

| Frame | Axes | Origin |
|---|---|---|
| n | ENU, flat earth, no earth rotation | road start |
| r | road: x along tangent, y left, z up (normal to road) | — |
| b | vehicle body FLU: x forward, y left, z up | R (rear-axle centre) |
| p | phone (Android): x right of screen, y top of screen, z out of screen | IMU / antenna point P |

`C_ab` maps b-frame vectors to a: `v_a = C_ab v_b`. Attitude chain (truth):
`C_nb = Rz(ψ)·Ry(−γ)·Rz(−β)·Ry(−θ_d)`.

Mount: `C_bp = C_off · C_BP_NOMINAL`, `C_off = Rz(yaw)·Ry(−pitch)·Rx(roll)` (small body-frame
offsets), true values roll 1°, pitch 12°, yaw 3° [ASSUMED].

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

## 2. Points

- **R**: rear-axle centre; follows the road centreline; along-track position `s` and all
  along-track errors are defined at R.
- **P**: phone; IMU and GNSS antenna are both at P. Lever arm `r_b` (R → P, body frame),
  true (1.6, 0.15, 0.7) m [ASSUMED].
- `p_R = p_P − C_nb r_b`. Ideal experiments may use the true r_b and C_nb; algorithm tests use
  the estimator's assumed lever arm.

## 3. Sensor biases and errors: frames

- IMU errors are generated **in the phone frame** (per phone axis): biases, white noise,
  scale factor S, misalignment M. Vibration is generated in the body frame and rotated to
  the phone frame before the sensor errors.
- A vehicle-axis quantity is the projection with the true mount: forward accel bias
  `b_∥ = e_xbᵀ b_p`, pitch-gyro bias `b_gy = e_ybᵀ b_g,p`; variances `σ² = eᵀ Σ_p e`.
- In error-state models, bias states are phone-frame biases unless stated otherwise; they
  enter vehicle-axis equations through the mount matrix in the Jacobian.

## 4. Timing

- IMU sample k: `f̃_k`, `ω̃_k` are **instantaneous** values at the true instant `t_true,k`
  (not averages over the interval), stamped `t_rep,k` (= t_true,k unless a timestamp scale
  error is on). The INS uses `Δt_k = t_rep,k+1 − t_rep,k`.
- Barometer sample at its stamp (no timing errors modelled).
- GNSS fix stamped t describes the antenna at `t − δ` (IMU–GNSS offset δ, nominal 0).
- **Errors are evaluated against the truth at the true time** of the estimate.

## 5. Error sign

Error = estimate − truth, for every quantity (δs = ŝ − s, δv, δθ, δb = b̂ − b).
Attitude error as a small rotation φ with `Ĉ_np = (I − [φ]×) C_np` (to be fixed with the ESKF
in v0.5; v0.4 reports attitude errors as angle differences per axis).

## 6. Oracle boundaries (V1)

- `tunnelsim/ins.py` and every later estimator module import nothing from `truth`,
  `scenario` or `sensors` (only `meas`, `frames`, numpy/scipy). Tested.
- Only the analysis harness reads the truth, and only through **independent** flags:
  - `oracle_init`: position, velocity, attitude at the start of the INS run
  - `ideal_attitude`: the attitude sequence C_np(t) is supplied as input
  - `perfect_calibration`: `"bias"` (turn-on + bias at the start known) or `"all"`
    (bias, scale factor, misalignment, mount angles known)
  - `oracle_speed`: true along-track speed (reference algorithm c)
- Every result carries the flags that produced it.

## 7. Metrics

- **Localization error**: ŝ − s_true at R, at a fixed 180 s after the portal (budget) and at
  each scenario's real exit (operational).
- **Exit-location / callout error**: map exit position error (ε_L·L) and its timing effect;
  never charged to localization.
- Monte Carlo: 200 seeds (development), 1000 (final); P95 with bootstrap CI; paired seeds
  across methods.
