# Through the Tunnel: write-up

Pointr case study. Estimating a car's position from a phone through a GNSS outage in a road
tunnel, for a navigation app that must (a) keep showing the car, (b) call out an exit inside the
tunnel at the right moment and (c) recover cleanly when the car comes out. All results come from
simulation; there is no real data yet. Numbers are 50-seed Monte Carlo results
(`python tools/evaluate.py`, `reports/0.5.0/`) unless stated otherwise. Every parameter carries a
tag: [MEAS] is measured and cited, [SPEC] is from a datasheet, [ASSUMED] was chosen here.
Section 7 lists the assumptions.

## 1. Framing: it is a 1D speed problem

The app has the map. Inside a single-lane tunnel the car is on a known polyline, so lateral
position is given and the only unknown is the **along-track distance s**. The exit callout ("Exit
A in 300 m") depends only on the along-track error. An error of δs at the callout point shifts the
callout by δs/v: 22 m is 1 s at 80 km/h. So the real question is how well we know the speed
inside the tunnel, and for how long.

The error budget (`DESIGN.md` §4, 200 seeds, 180 s from the portal) set the design:

- **Free pitch is unusable.** A phone-grade gyro (LSM6DSR in a Galaxy Note10, Allan values
  [MEAS] from Suvorkin et al. 2024) integrated freely gives 1.2–1.5 km of along-track error in
  180 s through gravity leakage. Pitch has to be held by the car's motion constraints and the
  barometer.
- **With perfect attitude the accelerometer bias drift is what remains:** ~30 m rms in 180 s.
  Sensor white noise contributes about 1 m.
- **Calibrating before the tunnel matters.** About 120 s of GNSS velocity, with some speed
  changes, is needed to separate the accelerometer bias from the speed. Without calibration the
  INS is off by 1–6 km at the exit.
- **A constant-velocity (CV) guess** from the GNSS entry speed beats the INS at constant speed
  (9.5 vs 19 m on S1) and fails as soon as the driver brakes (S5: 588 m; S4 stop-and-go: 2.6 km).

## 2. Approach

One error-state Kalman filter (ESKF, `tunnelsim/eskf.py`) runs on 100 Hz IMU data. Its covariance
is propagated at 10 Hz. It has 16 core states: position, velocity, attitude, accelerometer and
gyro bias (phone frame), and barometer offset. Two more states are used only in cruise mode.
The layers below are what the filter uses, and why each one is there:

| Layer | What it does | Why |
|---|---|---|
| GNSS position/velocity, NIS gate (χ², 99.9 %) | Calibrates biases and attitude in open sky; rejects outliers | Calibration quality before the outage is the main lever (§1) |
| Bias random walks kept small on purpose | Biases change slowly, even if GNSS velocity disturbs them | Robust over optimal: a bias that follows GNSS noise costs δb·t²/2 in the tunnel |
| Non-holonomic constraint (NHC) | Car body has no lateral or vertical velocity at the rear axle (σ 0.1 m/s, 5 Hz) | Holds pitch and heading; gives no forward-speed information |
| NHC turn gate | Lateral NHC row dropped while \|ω_z\|·v > 0.3 m/s² | Sideslip in curves violates NHC (§4.3) |
| Barometer (1 Hz average, own offset state) | Height, modelled at the window centre: p_z − lag·v_z + b_h | Holds the vertical channel and pitch on grades (§4.4) |
| ZUPT from an IMU stop detector | Zero velocity at stops | Stop-and-go traffic (S4) |
| Cruise mode: speed = v_ref + d (GM, σ_d 0.5 m/s, τ_d 60 s) | When the filter's own along-track acceleration stays below 0.02 m/s² for 5 s without GNSS, it assumes the driver holds speed with a bounded wander | A statistical prior on driver behaviour. It is not a sensor measurement (§4.2) |
| Exit re-acquisition phase | First 3 s after the outage: velocity only. Position R starts inflated and decays; each update moves the estimate by at most 2 m; the phase ends on convergence | No visible jump at the exit, even with multipath on the first fixes |
| Exit callout | Announced when ŝ crosses s_exit − 300 m | Requirement (b) |
| Branch check (`branch.py`) | Gyro heading change since 100 m before the diverge, compared with the map's main and branch profiles. Reports the branch on a 5° turn; reports main only once ŝ − 2σ_s has passed the point where the turn would have shown | Shows the car on the right road after the diverge, without relying on map matching |

Truth and estimator are kept apart: the estimator never imports truth objects
(`docs/contracts.md`). Every error term in the sensor models has its own random stream and can
be switched off (`tunnelsim/sensors.py`).

## 3. Results

The scenarios are listed in the README. Each tunnel is 2.5–3.5 km, and the route always leaves the
main tunnel through a 20° right-hand branch inside it. "At diverge" is the along-track error when
the car reaches the diverge; this is the straight-tunnel part, before the curve adds any
information. "At portal" is the error when the car comes out.

50 seeds per scenario (`reports/0.5.0/evaluation_50.json`):

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L |
|---|---|---|---|---|---|---|---|---|
| error at diverge, rms [m] | 12.1 | 13.4 | 12.2 | 31.0 | 25.1 | 13.4 | 13.3 | 13.2 |
| error at portal, rms [m] | 17.4 | 18.3 | 19.4 | 44.4 | 28.5 | 18.9 | 18.6 | 17.6 |
| error at portal, p95 [m] | 30.1 | 29.4 | 39.4 | 89.8 | 55.9 | 32.1 | 32.9 | 29.6 |
| mean (e/σ)² at portal | 0.36 | 0.90 | 0.41 | 0.69 | 0.85 | 0.42 | 0.40 | 0.35 |
| callout rms [s] | 0.46 | 0.42 | 0.53 | 1.53 | 1.24 | 0.50 | 0.50 | 0.51 |
| callout bias [s] (+ late) | +0.08 | -0.00 | +0.16 | +0.24 | +0.89 | +0.05 | +0.10 | +0.03 |
| callout > 1 s late | 2 % | 2 % | 4 % | 30 % | 40 % | 2 % | 2 % | 2 % |
| claimed σ_t [s] / coverage 2σ | 0.88 / 100 % | 0.63 / 100 % | 1.05 / 100 % | 2.96 / 100 % | 1.67 / 100 % | 0.89 / 100 % | 0.91 / 100 % | 0.92 / 100 % |
| branch reported / main reported | 100 / 0 % | 100 / 0 % | 100 / 0 % | 100 / 0 % | 100 / 0 % | 100 / 0 % | 100 / 0 % | 100 / 0 % |
| decision delay mean [s] | 2.0 | 1.9 | 2.3 | 2.9 | 2.3 | 2.0 | 2.1 | 2.1 |
| exit step p95 [m] | 3.6 | 3.8 | 3.5 | 3.8 | 4.0 | 3.6 | 3.9 | 3.7 |
| not < 10 m 60 s after exit | 0/50 | 0/50 | 2/50 | 12/50 | 0/50 | 0/50 | 0/50 | 0/50 |

Reading:

- **Showing the car.** Along-track error at the diverge is 12–13 m rms on S1–S3 and S6\*, after
  1.5–2 min without GNSS. Stop-and-go (S4, 31 m) and braking (S5, 25 m) are worse. The claimed σ is
  conservative: mean (e/σ)² is 0.3–0.9 and 96–100 % of seeds fall within 2σ.
- **Exit callout.** 0.4–0.5 s rms where the speed is steady; fewer than 4 % of callouts are more
  than 1 s late. S4 and S5 are the weak spots: 30 % and 40 % of callouts are more than 1 s late.
  The claimed timing σ covers every case.
- **Recovery.** The largest step at the exit is 3.5–4 m (p95). Before the re-acquisition phase it
  was 45–76 m. The estimate is within 10 m of truth 15–25 s after the portal. The exceptions are S4 and S3:
  after 5.5 min underground on S4, 12 of 50 seeds take more than 60 s to drain a ~45 m error at
  ≤ 2 m per fix (S3: 2 of 50). That is the accepted price of no jumps.
- **Branch.** When the car takes the branch, the branch is reported in all 400 runs. The report
  comes 2–3 s after the diverge, which is the time the curve needs to reach 5°. In the negative
  control the car goes straight on while the map still has the branch; "main tunnel" is reported
  in all 400 runs and the branch never. The main decision comes later, 3.6–11 s after the
  diverge, because it waits until the position is certain (ŝ − 2σ_s past the 5° point). A first
  version without that condition reported "main" for 18 % of the S4 cars that had taken the
  branch.

## 4. Where it breaks, and why

### 4.1 Stops and braking before the exit (S4, S5): late callouts

ZUPT resets the speed error at every stop. But the ESKF still integrates acceleration through each
start and stop, and a full stop gives the bias time to act. S4's callout comes 337 s after the
portal. S5 brakes from 82 to 50 km/h just before the callout point, and the estimate lags the real
deceleration: mean timing error +0.9 s, 40 % of callouts more than 1 s late.

A late callout is worse than an early one: the driver can miss the exit. An early one only shortens
the reaction time. A possible next step is an uncertainty-aware callout that stays on the early
side, s_call = s_exit − d_warn − k·σ_s; σ_s is already consistent. It is not in v0.5.

### 4.2 The speed-hold assumption: what helps and what it costs

Without GNSS, the INS cannot tell a small real acceleration from accelerometer bias
(ã = a + b_a + n). Holding the speed while the car "looks" steady adds information that does not
come from any sensor. Three forms were tried (`docs/decisions/2026-10-20-v0.5-eskf.md`, B):

1. **Absolute hold** (speed − v_hold = 0, with v_hold taken from the filter's own state): badly
   overconfident, mean (e/σ)² 6–33. The pseudo-measurement's error is the state error itself, so
   every 1 Hz update removes uncertainty that was never observed. Inflating R does not fix this:
   right after the hold, P drops to P_hold·(1 − 1/(2+k)), below the true floor for any finite k.
2. **Speed increment against a stochastic clone** (speed_k − speed_{k−1} = 0): consistent, and
   cuts the portal error by 30–50 % (S1 20 → 13 m, S3 32 → 10 m). It breaks when the driver's
   speed wanders by only ±1 km/h with slow ramps (S6s: (e/σ)² 1.6; ±2 km/h: 58 m, (e/σ)² 4.6). A
   sub-threshold real acceleration is read as bias. Its gain is the value of the "driver holds
   speed exactly" assumption, not of the sensors.
3. **GM speed wander** (speed = v_ref + d, with v_ref cloned together with its cross-covariance):
   consistent in every case, including above-threshold ramps (S6L), but it keeps only a small part
   of the gain. **This is the default.** It makes the estimate robust to bias drift under a stated
   model of the driver; it does not make bias and acceleration separable.

### 4.3 Sideslip against the non-holonomic constraint

A real car slips in a curve: β ≈ 0.6° in the 315 m-radius part of the exit branch [ASSUMED
vehicle], which is v_y ≈ 0.23 m/s against NHC σ = 0.1 m/s, with the same sign through the whole
curve. With the lateral NHC always on, every seed showed a −20 m along-track transient within
60 m of the diverge; the transient disappears when the truth has no sideslip.

The trace (`reports/0.5.0/nhc_trace_S1.png`) shows how it happens. The velocity error stays fixed
in the navigation frame while the body turns, so after α degrees of turning the lateral NHC row
sees the old along-track velocity error with weight sin α. This is real but weak information. The
0.23 m/s of sideslip in that row is amplified by about 1/sin α into an apparent speed error, and
the large position–velocity cross-covariance turns it into a ~20 m position correction.

Inflating R only slows the effect down, because the error is correlated over the whole curve.
Dropping the lateral row while |ω_z|·v > 0.3 m/s² removes the transient:

- S1 portal error goes from 16.6 to 15.6 m (30 seeds).
- The threshold is not critical: 0.2 and 0.5 m/s² give results within 0.5 m.
- Without sideslip in the truth, the gate costs at most 1.2 m (S1) or nothing (S2).

### 4.4 A model error found through a state, not the final number

On S3 (V-profile) the callout got worse when cruise mode was switched on. The cause was the
barometer, not the cruise mode. The 1 s barometer average was stamped at the end of its window
but represents the height 0.48 s earlier. On a 5 % grade that is 0.5 m, and at each grade change
the filter explained it as a 0.13–0.17° pitch error. This showed up as a common jump in the
seeds' pitch and speed errors at the grade changes; the final position error did not reveal it.
The window-centre model fixes it. The remaining second-order error is under 3 cm.

### 4.5 Assumption removal

Four dimensions, nominal against stressing cases, S1 with 30 seeds unless noted
(`reports/0.5.0/assumptions_30.json`). Differences of 1–2 m are within seed noise.

| Dimension | Case | Error at diverge, rms | Error at portal, rms | Mean (e/σ)² | Callout rms |
|---|---|---|---|---|---|
| — | nominal (50 seeds) | 12.1 m | 17.4 m | 0.36 | 0.46 s |
| GNSS entry quality | receiver coasts 5 s into the tunnel | 12.5 | 15.7 | 0.29 | 0.48 |
| | reported accuracy 0.7× the real error (optimistic) | 12.2 | 15.8 | 0.32 | 0.47 |
| Phone mount known to the filter | 1° pitch and yaw error | 11.7 | 16.1 | 0.31 | 0.45 |
| | 3° pitch and yaw error | 12.2 | 16.4 | 0.32 | 0.47 |
| Bias model | single GM (τ 300 s) instead of 3 GM | 11.3 | 17.3 | 0.35 | 0.44 |
| | **low-end IMU (MPU9250 [MEAS])**, same tuning | **3.2 km** | **3.5 km** | 2·10⁴ | 266 s |
| | low-end IMU, filter noise retuned once | 193 | 484 | 27 | 53 |
| Sideslip | 2× gain (0.8°/(m/s²)) | 11.6 | 20.5 | 0.51 | 0.44 |
| | 2× gain, S2 (long curves) | 13.2 | 21.4 | 1.22 (93 % in 2σ) | 0.44 |

- **GNSS entry quality.** Coasting fixes and optimistic accuracy do not hurt. The filter already
  inflates GNSS R (×2 position, ×3.2 velocity), and the slow bias states do not follow short
  disturbances.
- **Mount error.** Up to 3° is absorbed. The NHC turns the filter's attitude so that its (wrong)
  body frame lines up with the velocity. The pitch part then appears as g·sin θ in the
  along-track channel, which GNSS calibrates as part of the lumped accelerometer bias
  (`DESIGN.md` §4.2). This holds while the mount is rigid. A phone that moves in the mount
  during the tunnel is not covered.
- **Bias model.** The design is not sensitive to the shape of the bias model. It is very
  sensitive to the IMU class. The MPU9250's gyro bias instability is 3–7·10⁻³ rad/s, 30–80× the
  Note10, and NHC with the barometer cannot hold pitch against it. One retune of the filter noise
  is not enough. A low-end phone needs a different approach (speed cues, map matching, CV), not
  this INS.
- **Sideslip magnitude.** With twice the assumed sideslip, the portal error grows by 3 m and the
  filter becomes slightly optimistic on S2 (93 % within 2σ). The gate removes the curve itself;
  what remains is the sideslip just below the 0.3 m/s² threshold.

## 5. Limitations

- **Simulation only.**
  - The IMU noise terms are measured values from a published Allan analysis [MEAS]. Their
    time-domain shape (three Gauss–Markov terms), the vibration, sideslip, barometer and GNSS
    behaviour are [ASSUMED].
  - The bias-model shape alone moves the INS budget by a factor of ~2.4 (§4.3 of `DESIGN.md`).
- **The mount is given to the filter.** Errors up to 3° are absorbed while the mount stays
  rigid (§4.5). A phone that shifts in its mount is not covered.
- **One IMU class.** The filter is tuned for a mid-range phone IMU (LSM6DSR). A low-end IMU
  (MPU9250) breaks it (§4.5).
- **Map.**
  - The map is treated as exact: the polyline, the diverge position and the portal positions.
  - The map's grade profile is not used, because tunnel grade is often missing or wrong in maps.
    Barometer-to-grade matching would help in S3-like tunnels.
- **Not covered:** tunnels with several lanes (lane changes), cars that do not take the announced
  exit, phones moved inside the car.

## 6. What next

- An uncertainty-aware callout that stays on the early side (§4.1).
- A motion-mode layer (constant velocity vs INS, IMM) for the steady parts. The CV estimate is
  2× better there, but only when the speed really is constant.
- Real-device traces (a logger app in an Istanbul tunnel, portals as checkpoints), and an IMU
  class check at start-up, so that a low-end phone falls back to CV and map cues.
- A properly normalised branch decision (yaw σ and σ_s), and skipping the lateral NHC row instead
  of setting its R to 10⁶.

## 7. Assumptions

| Assumption | Value |
|---|---|
| Phone | Rigid car mount, attitude constant during the drive; mount angles known to the filter in the baseline [ASSUMED] |
| IMU | Samsung Galaxy Note10, LSM6DSR, Allan VRW/ARW/bias instability [MEAS]; turn-on bias, scale factor, misalignment [SPEC]; 3-GM bias shape, vibration [ASSUMED] |
| GNSS | 1 Hz, 3 m horizontal; correlated velocity noise; degradation near portals, exit multipath, reported accuracy [ASSUMED] |
| Barometer | 25 Hz, 0.10 m white + GM offset [ASSUMED from datasheet]; closed cabin |
| Map | Exact horizontal geometry, diverge and portal positions; no grade |
| Vehicle | Sideslip 0.4°/(m/s²) with 0.2 s lag; NHC holds at the rear axle except for sideslip [ASSUMED] |
| Driver | Steady speed with slow wander (GM σ_d 0.5 m/s, τ_d 60 s) inside the tunnel when the filter detects cruise [ASSUMED] |
| Callout | 300 m before the diverge; more than 1 s late counts as a late warning [ASSUMED] |
| Exit branch | 20° right over 150 m (clothoid 40 m + arc 70 m + clothoid 40 m) [ASSUMED] |

## 8. AI usage

This project was built with an AI coding agent: Claude (Anthropic), working in a cloud
workspace attached to the GitHub repository. The division of work:

- **The author (Alper Mutlu) decided.** He decided the problem framing, the assumptions, every
  filter-design and error-budget choice, the scenarios and the acceptance criteria. Before each
  piece of code, Claude proposed options with formulas and reasons, and the author approved,
  changed or rejected them. His reviews are recorded in `docs/reviews/`, the decisions in
  `docs/decisions/`. Two examples:
  - "robust over optimal: restrict only the bias states' rate of change";
  - correcting the observability claim of the GM speed model (review 6).
- **Claude implemented.** It wrote the simulator, the sensor models, the filter, the tests and
  the experiments. It ran the Monte Carlo studies, reported numbers and diagnoses, and drafted
  the documents, this write-up included. The author reviewed them.
- **Process controls**, used because generated code can be confidently wrong:
  - every number is tagged [MEAS], [SPEC] or [ASSUMED];
  - truth and estimator are separated in code;
  - each version gets a CHANGELOG entry and a metrics snapshot, and a changed metric must be
    explained;
  - every decision and review is a commit;
  - release tags are set by the author, not by the agent.

Errors made on the AI side and caught by this process (all in the decision log):

- **Caught in the author's reviews:**
  - a sign error in the attitude-error convention (review 4);
  - an oracle lower bound in the error budget presented as achievable (15.8 m vs 21.6 m,
    review 5);
  - an observability claim stronger than the model supports (review 6).
- **Caught by consistency metrics, not by the final error:**
  - the absolute speed hold that made the filter overconfident;
  - the barometer timestamp lag;
  - the sideslip/NHC interaction.
- **Implementation bugs caught by tests and Monte Carlo:**
  - an innovation sign;
  - a barometer index that never advanced;
  - a stop detector that fired at launch.

The agent could not tag releases, and it did not use any real sensor data.
