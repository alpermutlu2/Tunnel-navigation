# Through the Tunnel: write-up

Pointr case study. A phone in a car estimates its position through a GNSS outage in a road
tunnel. The navigation app must keep showing the car, call out an exit inside the tunnel at the
right moment, and recover cleanly when the car comes out. All results are from simulation. Unless
a section says otherwise they are 50-seed Monte Carlo results (`tools/evaluate.py`,
`reports/0.5.1/`). Every parameter is tagged: [MEAS] measured and cited, [SPEC] from a datasheet,
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
accelerometer and gyro bias, and barometer offset. Cruise mode adds two states, and the map layer
adds two more (map lateral offset and map scale error).

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
| Map in the tunnel: 1 Hz heading (σ 1°) and cross-track (σ 1.5 m) against the route polyline; map lateral offset (σ 2 m) and length error (σ 0.5 %) as states per tunnel; paused around the diverge until the in-loop branch decision | Heading and lateral position, which NHC cannot give in a straight tunnel; along-track information from curves (curve matching), bounded by the map's own uncertainty |
| Callout at ŝ = s_diverge − 300 m. Branch check from the gyro heading change against the map's two profiles (`branch.py`) | Requirements (b), and the car shown on the right road |

The estimator never imports truth objects (`docs/contracts.md`, enforced by a test). Each sensor
error term has its own random stream and can be switched off.

## 3. Results

In every scenario the route leaves the main tunnel through a 20° branch inside it. The callout
comes before that diverge. "At diverge" is the result for the straight tunnel, before the curve
adds information; "at portal" is where the car comes out. 50 seeds per scenario
(`reports/0.5.2/evaluation_50.json`):

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | S1-1km | S1-5km |
|---|---|---|---|---|---|---|---|---|---|---|
| | 3 km straight | S-bend | V-profile | stop-and-go | braking | wander ±1 km/h | ±2 km/h | ±4 km/h | 1 km | 5 km |
| along-track at diverge, rms [m] | 12.1 | 3.2 | 12.1 | 31.1 | 25.0 | 13.4 | 13.3 | 13.2 | 4.8 | 24.6 |
| along-track at portal, rms / p95 [m] | 3.8 / 7 | 3.2 / 6 | 7.2 / 16 | 5.1 / 10 | 4.1 / 8 | 4.0 / 8 | 4.2 / 7 | 4.1 / 7 | 2.0 / 4 | 10.0 / 19 |
| cross-track at portal, rms [m] | 1.5 | 1.4 | 1.5 | 0.8 | 1.5 | 1.4 | 1.5 | 1.5 | 1.5 | 1.5 |
| mean (e/σ)² at portal | 0.15 | 0.24 | 0.20 | 0.23 | 0.28 | 0.16 | 0.18 | 0.16 | 0.18 | 0.34 |
| callout rms [s] / > 1 s late | 0.46 / 2 % | 0.13 / 0 % | 0.53 / 4 % | 1.53 / 30 % | 1.24 / 40 % | 0.50 / 2 % | 0.50 / 2 % | 0.51 / 2 % | 0.17 / 0 % | 0.96 / 24 % |
| largest exit step, p95 [m] | 4.1 | 4.8 | 4.5 | 4.7 | 4.7 | 4.0 | 4.1 | 4.0 | 4.1 | 4.1 |
| not < 10 m 60 s after the exit | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1 |

- **Showing the car.**
  - Along the road: 12–13 m after 1.5–2 min of steady driving in a straight tunnel; 25–31 m with
    braking or stops; 25 m after 3.6 km.
  - Across the road: 1.5 m everywhere, and heading to 0.1°. This comes from the map layer;
    without it, the cross-track error was 26 m on S1 and 106 m at 5 km, because heading is not
    observable in a straight tunnel.
  - Wherever the road curves, the map also gives along-track information. On S2 the error at the
    diverge drops from 13 to 3 m, and the exit curve brings every portal error down to 2–10 m.
  - The filter is consistent and conservative: mean (e/σ)² is 0.15–0.34.
- **Callout.** 0.4–0.5 s rms on straight tunnels at steady speed, 0.13 s where curves come
  before the callout (S2). The timing uncertainty the filter claims covers 96–100 % of the cases.
- **Recovery.** The largest step at the exit is ≤ 5 m (it was 45–76 m before the re-acquisition
  phase). The estimate is within 10 m of truth after the exit in 499 of 500 runs.
- **Branch.**
  - Car takes the branch: the branch is reported in 496 of 500 runs, 2–3 s after the diverge,
    which is the time the curve needs to reach 5°. The exceptions are 2 of 50 S4 runs, where the
    evaluation's offline check said "main"; the filter's own in-loop decision is separate.
  - Car goes straight on (negative control, the map still has the branch): "main" in 500 of 500
    runs, and the filter does not pull the car onto the branch.

**Without the map layer** (v0.5.1, `reports/0.5.1/evaluation_50.json`), the portal error was
17–19 m on steady scenarios, 44 m on S4 and 34 m at 5 km. The cross-track error was 26–106 m,
and after S4 the ≤ 2 m-per-fix recovery took over 60 s for 12 of 50 seeds, because it was
draining a ~100 m cross-track error. The along-track error at the diverge of a straight tunnel
is the same with and without the map (S1: 12.1 m). On a straight road the map gives no
along-track information, so the core question stays the speed.

**Tunnel length.** The diverge sits at min(0.73·L, L − 200 m) [ASSUMED]. Along-track error at the
diverge: 4.8 m at 1 km, 12 m at 3 km, 25 m at 5 km. At 0.5 km the callout point is at the portal,
still under GNSS, so the callout says nothing about the outage there. Without the map, one 5 km
run gives the along-track error as a function of distance (`reports/0.5.1/error_vs_distance.png`).
On the straight part it grows faster than linearly (bias drift ~t²): 5 m after 1 km, 9 m after
2 km, 15 m after 3 km. At the portal the filter is overconfident for the first ~500 m, because the
GNSS degradation just before the portal is not in its error model. At 5 km, 24 % of callouts
are more than 1 s late.

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

**4.7 What the map adds, and what it costs.** In a straight tunnel the map fixes heading and
lateral position but gives no along-track information. Curves give it, but only as well as the
map places them. A map that is 0.5 % too long draws the exit curve 11 m off, and the curve
matching then pulls the estimate by about that much. With the map errors as states the filter
knows this, and treating them as white noise was the mistake to avoid: the same correlated-error
lesson as the absolute speed hold (§4.2) and the sideslip/NHC interaction (§4.3).

## 5. Limitations

- **Simulation only.** The IMU noise terms are measured; their time shape, the vibration,
  sideslip, barometer, pressure-gradient and GNSS behaviour are [ASSUMED]. The bias-model shape
  alone moves the INS budget by ~2.4×.
- **Map quality.**
  - The map layer treats the polyline as correct up to a constant lateral offset (σ 2 m) and a
    length error (σ 0.5 %), both [ASSUMED].
  - Stress test, every seed with a 2 m offset and a +0.5 % length error (the exit curve drawn
    ~11 m too far on): portal error 8–17 m, and 93–97 % of seeds within 2σ. Before the map errors
    became states, only 63–77 % were within 2σ (`reports/0.5.2/map_ab_30.json`).
  - Real polyline errors (shape, missing curves, a wrong diverge position) are not simulated.
    The truth car drives on the lane centre, so lane-keeping wander is covered only by σ 1.5 m.
- **The ZUPT detector is offline** (forward-backward filters, zero delay). A causal version is
  similar at the exit (S4 46 vs 44 m) but worse for the callout (> 1 s late: 50 vs 30 %).
- **Narrow setting.** The filter is tuned for one IMU class. The car is a single car with a
  rigid phone mount, in a single lane. The negative control (car goes straight on at the
  diverge) covers the map layer and the branch decision.

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
