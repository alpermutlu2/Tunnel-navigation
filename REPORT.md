# Through the Tunnel: write-up

Pointr case study. A phone in a car estimates its position through a GNSS outage in a road
tunnel. The navigation app must keep showing the car, call out an exit inside the tunnel at the
right moment, and recover cleanly when the car comes out. All results are from simulation. Unless
a section says otherwise they are 50-seed Monte Carlo results (`tools/evaluate.py`,
`reports/0.5.0/`). Every parameter is tagged: [MEAS] measured and cited, [SPEC] from a datasheet,
[ASSUMED] chosen here (`tunnelsim/profiles.py`, `DESIGN.md` §2).

The simulator runs interactively (`python tools/serve.py`, controls for scenario, sensors and filter
switches) or as a single page per run (`tools/simulate.py`); see the README.

## 1. Framing

The app has the map, and the tunnel is a single lane, so the car's lateral position is known.
The only unknown is the **along-track distance s**. The callout depends only on that: an error
δs shifts it by δs/v, and 22 m is 1 s at 80 km/h. So the real question is how well we know the
speed, and for how long.

A measured error budget set the design (200 seeds, 180 s from the portal, `DESIGN.md` §4):

- Free pitch is unusable. The Galaxy Note10 gyro (LSM6DSR, Allan terms [MEAS], Suvorkin et al.
  2024) integrated freely gives 1.2–1.5 km of error through gravity leakage. NHC and the
  barometer have to hold pitch.
- With perfect attitude, the accelerometer bias drift leaves ~30 m. White noise adds about 1 m.
- Bias calibration needs about 120 s of GNSS, with some speed changes, before the portal.
- Constant velocity (CV) from the entry speed beats the INS at constant speed (9.5 vs 19 m) and
  fails as soon as the driver brakes (0.6–2.6 km).

## 2. Approach

The estimator is an error-state Kalman filter (`tunnelsim/eskf.py`). It runs on 100 Hz IMU data
and propagates its covariance at 10 Hz. It has 16 states: position, velocity, attitude,
accelerometer and gyro bias, and barometer offset; cruise mode adds two more.

| Layer | Role |
|---|---|
| GNSS position and velocity, χ² gate (99.9 %), R inflated ×2 / ×3.2 for correlated noise | Calibrates biases and attitude before the outage |
| Gate-lock recovery: P inflated after 5 open-sky rejections; accel/gyro noise scaled by the GNSS NIS | Safety net; never fires on the nominal scenarios (§4.5) |
| Small bias random walks (robust over optimal) | A bias that follows GNSS noise costs δb·t²/2 in the tunnel |
| Non-holonomic constraint (NHC): no lateral or vertical body velocity, σ 0.1 m/s | Holds pitch and heading. Gives no speed information |
| Lateral NHC dropped while \|ω_z\|·v > 0.3 m/s² | The car's sideslip in curves violates NHC (§4.3) |
| Barometer, 1 s average, modelled at the window centre (p_z − lag·v_z + b_h) | Vertical channel and pitch |
| ZUPT from an IMU stop detector | Stop-and-go traffic |
| Cruise prior: speed = v_ref + d, d Gauss–Markov (σ_d 0.5 m/s, τ_d 60 s) | Assumes the driver holds speed with slow wander, used when the filter's own acceleration stays below 0.02 m/s² (§4.2) |
| Re-acquisition phase: 3 s velocity only, decaying position R, ≤ 2 m per update, ends on convergence | No jump at the exit, even with multipath |
| Callout at ŝ = s_diverge − 300 m. Branch check from the gyro heading change against the map's two profiles (`branch.py`) | Requirements (b), and the car shown on the right road |

The estimator never imports truth objects (`docs/contracts.md`, enforced by a test). Each sensor
error term has its own random stream and can be switched off.

## 3. Results

In every scenario the route leaves the main tunnel through a 20° branch inside it. The callout
comes before that diverge. "At diverge" is the straight-tunnel result, before the curve adds
information; "at portal" is where the car comes out. Each tunnel is 2.5–3.5 km.

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L |
|---|---|---|---|---|---|---|---|---|
| | straight | S-bend | V-profile | stop-and-go | braking | wander ±1 km/h | ±2 km/h | ±4 km/h |
| error at diverge, rms [m] | 12.1 | 13.4 | 12.2 | 31.0 | 25.1 | 13.4 | 13.3 | 13.2 |
| error at portal, rms / p95 [m] | 17 / 30 | 18 / 29 | 19 / 39 | 44 / 90 | 28 / 56 | 19 / 32 | 19 / 33 | 18 / 30 |
| mean (e/σ)² at portal | 0.36 | 0.90 | 0.41 | 0.69 | 0.85 | 0.42 | 0.40 | 0.35 |
| callout rms [s] / > 1 s late | 0.46 / 2 % | 0.42 / 2 % | 0.53 / 4 % | 1.53 / 30 % | 1.24 / 40 % | 0.50 / 2 % | 0.50 / 2 % | 0.51 / 2 % |
| largest exit step, p95 [m] | 3.6 | 3.8 | 3.5 | 3.8 | 4.0 | 3.6 | 3.9 | 3.7 |
| not < 10 m 60 s after the exit | 0/50 | 0/50 | 2/50 | 12/50 | 0/50 | 0/50 | 0/50 | 0/50 |

- **Showing the car.** Error is 12–13 m after 1.5–2 min without GNSS at steady speed, and 25–31 m
  with braking or stops. The filter is consistent and slightly conservative: mean (e/σ)²
  0.35–0.9, and 96–100 % of seeds fall within 2σ.
- **Callout.** 0.4–0.5 s rms at steady speed. The timing uncertainty the filter claims covers
  100 % of the cases.
- **Recovery.** The largest step at the exit is ≤ 4 m (it was 45–76 m before the re-acquisition
  phase). The estimate is within 10 m of truth 15–25 s after the exit. The ≤ 2 m-per-fix cap is
  slow after S4's 5.5 min outage; that is the accepted price of no jumps.
- **Branch.** Taking the branch: reported in all 400 runs, 2–3 s after the diverge, which is the
  time the curve takes to reach 5°. Negative control, with the car going straight on: "main" in
  all 400 runs, branch never. "Main" is decided only once ŝ − 2σ_s is past the 5° point.
  Without that condition, 18 % of S4 cars on the branch were reported as "main".

**Tunnel length** (S1 geometry, 30 seeds, diverge at min(0.73·L, L − 200 m) [ASSUMED]):

| Tunnel | 0.5 km | 1 km | 3 km | 5 km |
|---|---|---|---|---|
| error at portal, rms [m] | 4.2 | 5.3 | 17.4 | 34.1 |
| callout rms / > 1 s late | — ¹ | 0.18 s / 0 % | 0.46 / 2 % | 0.81 / 17 % |

¹ With a 300 m lead the callout point of a 0.5 km tunnel is at the portal, still under GNSS, so the
callout says nothing about the outage there; the error at the portal is the relevant number.

One 5 km run gives the whole curve (`reports/0.5.1/error_vs_distance.png`, 30 seeds): rms
3 m at the portal, 5 m after 1 km, 9 m after 2 km, 15 m after 3 km and 28 m after 4.5 km. The
error grows faster than linearly with outage time (bias drift ~t²), and the cruise prior softens
it. The filter σ covers the error everywhere except in the first ~500 m: at the portal it is
overconfident (σ 1.5 m vs 3.2 m rms), because GNSS degradation just before the portal is not in
its error model. At the 5 km diverge the exit curve still adds a transient of about +7 m rms,
which is not yet explained. At 5 km, one callout in six is more than 1 s late, and 11 of 30 seeds take more than
60 s to settle after the exit.

## 4. Where it breaks, and why

**4.1 Stops and braking before the callout (S4, S5).** ZUPT resets speed at stops, but every start
and stop is integrated, and a long stop gives the bias time to act. S5 brakes just before the
callout point and the estimate lags: mean timing +0.9 s, 40 % of callouts more than 1 s late.
A late callout can make the driver miss the exit; an early one only costs reaction time.
σ_s is consistent, so a callout that stays on the early side, s_call = s_exit − d_warn − k·σ_s,
is the natural next step.

**4.2 Driver behaviour is an assumption, not a measurement.** Without GNSS a small real
acceleration and an accelerometer bias look the same (ã = a + b_a + n). Three speed-hold forms
were tried:

1. **Absolute hold**, with v_hold taken from the filter's own state: overconfident, mean (e/σ)²
   6–33. The pseudo-measurement's error is the state error itself, and inflating R cannot fix
   that.
2. **Cloned speed-increment constraint**: consistent, and 30–50 % better on S1–S5. But it fails
   once the driver's speed wanders by ±1 km/h ((e/σ)² 1.6; at ±2 km/h, 58 m with (e/σ)² 4.6).
3. **Gauss–Markov wander (default)**: consistent in every case, ramps above the detection
   threshold included, but it keeps only a small part of the gain.

The gap between (2) and (3) is the value of assuming that the driver holds speed exactly. It does
not come from the sensors.

**4.3 Sideslip violates the lateral NHC.** In the exit curve the car slips by β ≈ 0.6°
[ASSUMED vehicle], a lateral velocity of 0.23 m/s against NHC σ = 0.1 m/s, with the same sign
through the whole curve. Every seed then showed a −20 m along-track transient, and the transient
vanishes when the truth has no sideslip (`reports/0.5.0/nhc_trace_S1.png`). The mechanism:

- The nav-frame velocity error stays fixed while the body turns, so after α degrees the lateral
  row sees the old speed error weighted by sin α.
- The sideslip in that row is amplified by about 1/sin α into an apparent speed error.
- The position–velocity cross-covariance turns that into a position correction.

Inflating R only slows the effect down. Dropping the row in turns removes it. The 0.2–0.5 m/s²
threshold range gives results within 0.5 m, and the information loss is ≤ 1.2 m when there is no
sideslip.

**4.4 Tunnel pressure gradient: the barometer's weak point.** Ventilation and the piston effect
can create a longitudinal pressure gradient that the barometer reads as height. Its magnitude is
[ASSUMED] and not sourced yet.

- At 10 Pa/km (≈ 0.85 m of apparent height per km), the S3 error at the diverge is 44 m and
  half of the branch decisions are wrong.
- At 30 Pa/km, the error is 110–120 m, (e/σ)² is 9–14, and every branch decision is "main".

The filter reads the apparent climb as pitch, the gravity component integrates into speed, and
the overconfident σ_s lets the branch check decide too early.

A gradient state (ḃ_h = g_h·|v| during the outage, σ_g 3·10⁻³) removes the gradient error: at
30 Pa/km the diverge error drops from 156 to 19 m and every branch decision is right. It is still
off by default, because it costs accuracy on nominal roads: S1 at the portal 17 → 21 m, S3
callouts more than 1 s late 4 → 30 %, S5 diverge 23 → 109 m.

The reason is structural. On a straight tunnel with a constant grade, a constant pitch error gives
(through NHC) a height error linear in s, exactly like a gradient, and the barometer cannot tell
them apart. The barometer trend gets split between pitch and gradient, so the pitch hold loosens
(S5 trace: mean tilt error −0.02° → −0.06°). A strong pitch–gradient correlation builds up,
and S5's braking turns it into a 0.9 m/s speed step (`reports/0.5.1/grad_trace_S5.png`).

The fix has to come from a measured gradient (a real tunnel recording) or from a second
observable, not from a looser model. With the state on, 12 of 30 seeds still do not settle within
60 s at 30 Pa/km. The truth makes the gradient offset decay over 200 m after the exit; the filter
does not model that decay.

**4.5 Low-end IMU: gate lock.** An MPU9250-class IMU [MEAS] has 30–80× the Note10's gyro bias
instability. With the same tuning the filter diverges **before the tunnel**: it becomes
overconfident, and the χ² gate then rejects 61 % of the GNSS position fixes and 71 % of the
velocity fixes. The error reaches 3.5 km at the exit.

Two safety nets are now on by default. Both never fire on the nominal scenarios:

- After 5 consecutive open-sky rejections, P is inflated so that the current innovation fits the
  gate.
- The accelerometer and gyro noise scale with the GNSS NIS (rising over 20 s, decaying over 60 s).

With them the filter accepts GNSS again and the exit error drops to 1.3 km. But (e/σ)² ≈ 2000:
the filter no longer diverges, yet its uncertainty is still far too small. A low-end phone is
outside the scope of this design. It needs an IMU-class check at start-up and a fallback to CV and
map cues.

**4.6 Assumptions that did not matter** (S1, 30 seeds, `reports/0.5.0/assumptions_30.json`):

- GNSS coasting into the tunnel, or optimistic reported accuracy: within ±2 m of nominal.
- A 1–3° mount error known wrongly to the filter: absorbed. NHC aligns the filter's attitude with
  the velocity, and GNSS calibrates the pitch part as accelerometer bias. This holds only while
  the mount stays rigid.
- A single-GM bias model instead of three GM terms: within noise.
- Twice the assumed sideslip: 3 m more at the portal. On S2, 93 % of seeds fall within 2σ.

## 5. Limitations

- **Simulation only.** The IMU noise terms are measured; their time shape, the vibration,
  sideslip, barometer, pressure-gradient and GNSS behaviour are [ASSUMED]. The bias-model shape
  alone moves the INS budget by ~2.4×.
- **No map in the filter.** Inside the tunnel the filter does not use the map. The along-track
  error is the estimate projected onto the true road tangent, which amounts to perfect map
  matching. The map (polyline, diverge, portals) is taken as exact.
- **The ZUPT detector is offline** (forward-backward filters, zero delay). A causal version is
  similar at the exit (S4 46 vs 44 m) but worse for the callout (> 1 s late: 50 vs 30 %).
- **Narrow setting.** The filter is tuned for one IMU class. The car is a single car with a
  rigid phone mount, in a single lane, and it always takes the announced exit (the negative
  control only tests the branch check).

## 6. AI usage

The code, tests, experiments and document drafts were written by an AI coding agent: Claude
(Anthropic), working in a cloud workspace on this repository.

- **Decisions and reviews.** The author (Alper Mutlu) made the decisions: problem framing,
  assumptions, every filter-design and error-budget choice, scenarios and acceptance criteria.
  Before coding each piece, the agent proposed options with formulas, and the author approved,
  changed or rejected them (`docs/decisions/`). The author also had the repository and the plans
  reviewed by other AI assistants and models (the last review by Claude Fable 5.1). He filtered
  those reviews and passed on what he accepted (`docs/reviews/`). For example, the
  pressure-gradient, gate-lock, offline-ZUPT and tunnel-length points above came from that last
  review.
- **Process controls**, used because generated code can be confidently wrong: parameter tags;
  truth/estimator separation enforced by a test; a CHANGELOG entry and a metrics snapshot for
  each version, with any changed metric explained; decisions and reviews kept as commits; and
  release tags set by the author.
- **Errors made on the AI side and caught by this process:**
  - In reviews: an attitude-error sign; an oracle lower bound presented as achievable; an
    observability claim stronger than the model supports.
  - By consistency metrics rather than final error: the overconfident absolute speed hold; a
    0.5 s barometer timestamp lag that showed up only as a common pitch jump at grade changes;
    the sideslip/NHC transient.
  - By tests and Monte Carlo: an innovation sign; a barometer index that never advanced; a stop
    detector that fired at launch.
