# Through the Tunnel: write-up

A two-page version is in [`SUMMARY.md`](SUMMARY.md).

Pointr case study. A phone in a car estimates its position through a GNSS outage in a road
tunnel. The navigation app has to keep showing the car, call out an exit inside the tunnel at the
right moment, and recover cleanly when the car comes out.

All results come from simulation: 50-seed Monte Carlo runs (`tools/evaluate.py`,
`reports/0.5.3/`) unless a section says otherwise. Every parameter is tagged [MEAS] (measured,
cited), [SPEC] (datasheet) or [ASSUMED] (chosen here); §2.2 lists the assumptions and
`tunnelsim/profiles.py` holds every value. The simulator runs interactively (`python tools/serve.py`) or one page per run
(`tools/simulate.py`); the README has the commands.

## 1. Framing

The app has the map and the tunnel has one lane, so the unknown is the **along-track distance
s**. The exit callout depends only on s: an error δs shifts the callout by δs/v, and 22 m is 1 s
at 80 km/h. The core question is therefore how well we know the speed, and for how long.

A measured error budget set the design (`DESIGN.md` §4, 200 seeds, 180 s):

- **Free pitch is unusable.** The Galaxy Note10 gyro (LSM6DSR, Allan terms [MEAS], Suvorkin et
  al. 2024) integrated freely gives 1.2–1.5 km of error through gravity leakage.
- **With perfect attitude, accelerometer bias drift leaves ~30 m.** White noise adds about 1 m.
- **Calibrating the bias** needs about 120 s of GNSS before the portal, with some speed changes.
- **Constant velocity (CV)** from the entry speed beats the INS at constant speed (9.5 vs 19 m)
  and fails as soon as the driver brakes (0.6–2.6 km).

## 2. Approach

The estimator is an error-state Kalman filter (`tunnelsim/eskf.py`). It runs on 100 Hz IMU data.
Its 16 core states are position, velocity, attitude, accelerometer bias, gyro bias and barometer
offset. Cruise mode adds 2 states and the map layer adds 2 more.

| Layer | Role |
|---|---|
| GNSS position and velocity; χ² gate; R inflated for correlated noise; small bias random walks | Calibrate biases and attitude before the outage (robust over optimal) |
| Non-holonomic constraint (NHC): no lateral or vertical body velocity | Holds pitch. Lateral row skipped while \|ω_z\|·v > 0.3 m/s², because of sideslip in curves (§4.3) |
| Barometer, 1 s average modelled at the window centre | Vertical channel and pitch |
| ZUPT from an IMU stop detector | Stop-and-go traffic |
| Cruise prior: speed = v_ref + d, d Gauss–Markov (σ 0.5 m/s, τ 60 s) | A stated model of the driver, not a measurement (§4.2) |
| Map in the tunnel: 1 Hz heading and cross-track against the route polyline; map lateral offset (σ 2 m) and length error (σ 0.5 %) as states; paused around the diverge until the in-loop branch decision | Heading and lateral position; along-track information where the road curves, bounded by the map's own uncertainty (§4.5) |
| Callout when ŝ + 1·σ_s reaches s_exit − 300 m | Errs early: a late callout can make the driver miss the exit |
| Re-acquisition: 3 s velocity only, decaying position R, ≤ 2 m per update | No jump at the exit, even with multipath |
| Gate-lock recovery, adaptive noise | Safety net; never fires in the nominal scenarios |
| IMU class from the gyro white noise at start-up; a filter profile per class | Low-end phones (§4.6) |

### 2.1 What is simulated

Three layers meet only through sensor streams. The estimator never imports truth objects, and a
test enforces this (`docs/contracts.md`).

- **Truth** (`tunnelsim/truth.py`).
  - Road: curvature and grade with clothoid and minimum-jerk transitions.
  - Car: drives the lane centre with smooth (quintic) speed changes, suspension pitch and sideslip.
  - Phone: on a dashboard mount (12° pitch, 1.6 m lever arm); specific force and angular rate are
    computed at the phone at 100 Hz.
  - Check: strapdown of the ideal IMU reproduces the path within 0.13 cm.
- **Sensors** (`tunnelsim/sensors.py`). Every error term can be switched off alone and has its
  own random stream.
  - IMU: white noise, bias instability as three Gauss–Markov terms (5/50/500 s), turn-on bias,
    scale and misalignment, road and engine vibration.
  - Barometer: 25 Hz, white noise plus a drifting offset.
  - GNSS: 1 Hz position and Doppler velocity, with correlated errors and reported accuracy. It
    fades over the 4 s before the portal and gives no fixes inside. At the exit, velocity
    returns after 1 s and position after 2 s, with a 30 m multipath bias that decays.
- **Drives.** Each drive starts with 200 s under GNSS, including ±15 km/h speed changes so the
  biases can be calibrated. The route leaves through a 20° branch inside the tunnel.

| Scenario | Tunnel | Driving inside |
|---|---|---|
| S1 | 3 km, straight, flat | 80 km/h constant |
| S2 | 2.5 km, S-bend (R 400 m) | 80 km/h |
| S3 | 3.5 km, V-profile ±5 % | 70 km/h |
| S4 | 3 km, straight | 60 km/h, two full stops (20 s, 45 s) and a crawl |
| S5 | 3 km, straight | 90 → 82 km/h drift, brake to 50 km/h |
| S6s / S6 / S6L | 3 km, straight | speed wander ±1 / ±2 / ±4 km/h |
| S1-0.5km / 1km / 5km | as S1 | as S1 |
| custom (live) | 0.5–5 km, any grade | any speed, one stop |

Each scenario runs 50 noise realisations (seeds). The metrics are rms and 95th percentile errors,
and consistency: the mean of (e/σ)², which is 1 for an honest filter and > 1 when it is
overconfident.

### 2.2 Assumptions

| Area | Assumption | Tag |
|---|---|---|
| Tunnel | Single lane, 0.5–5 km | given |
| Phone | Rigid mount, angles known to the filter (1–3° error tested, §4.8) | [ASSUMED] |
| IMU | Galaxy Note10 (LSM6DSR) Allan terms; turn-on bias 10 mg | [MEAS]; [SPEC] |
| IMU | Shape of the bias drift (three Gauss–Markov terms), vibration levels | [ASSUMED] |
| GNSS | 3 m horizontal, 0.1 m/s, correlated (30 s / 10 s); fade before the portal; 30 m exit multipath | [ASSUMED] |
| Barometer | 0.1 m white + 0.1 m offset drift (60 s); closed cabin; no tunnel pressure gradient (30 Pa/km tested, §4.4) | [ASSUMED] |
| Map | Route polyline with the diverge; lateral offset σ 2 m and length error σ 0.5 % (§4.5); no grade profile | [ASSUMED] |
| Vehicle | Sideslip 0.4°/(m/s²) with a 0.2 s lag; suspension pitch 1.5°/g | [ASSUMED] |
| Driver | In cruise the speed wanders like a Gauss–Markov process, σ 0.5 m/s, τ 60 s (the filter's model) | [ASSUMED] |
| Exit | 20° branch over 150 m at about 0.73 of the tunnel; callout 300 m before it, k = 1 | [ASSUMED] |

Sensors left out:
- **Magnetometer:** the tunnel's steel and the car body distort it.
- **Ambient light:** it could mark the portal, but GNSS loss already does, and tunnel lighting
  levels are unknown.
- **Camera, BLE beacons, OBD and wheel speed:** an average navigation user does not have them.

## 3. Results

In every scenario the route leaves the main tunnel through a 20° branch inside it, and the
callout comes before that diverge. "At diverge" is the straight-tunnel result. "At portal" is
the result when the car comes out.

50 seeds per scenario; `reports/0.5.3/evaluation_50.json`.

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | S1-1km | S1-5km |
|---|---|---|---|---|---|---|---|---|---|---|
| | 3 km straight | S-bend | V-profile | stop-and-go | braking | wander ±1 km/h | ±2 km/h | ±4 km/h | 1 km | 5 km |
| along-track at diverge, rms [m] | 12.1 | 3.5 | 12.1 | 31.1 | 25.0 | 13.4 | 13.3 | 13.2 | 4.8 | 24.6 |
| along-track at portal, rms / p95 [m] | 5.6 / 10 | 4.1 / 7 | 9.0 / 17 | 5.6 / 12 | 7.5 / 15 | 6.0 / 10 | 6.2 / 11 | 5.6 / 11 | 2.5 / 5 | 13.7 / 25 |
| cross-track at portal, rms [m] | 1.5 | 3.2 | 2.0 | 1.3 | 1.9 | 1.5 | 1.5 | 1.5 | 1.4 | 2.1 |
| mean (e/σ)² at portal | 0.13 | 0.23 | 0.18 | 0.11 | 0.27 | 0.15 | 0.16 | 0.13 | 0.22 | 0.25 |
| callout timing, filter only (k = 0): mean / > 1 s late | +0.1 / 2 % | +0.1 / 0 % | +0.2 / 4 % | +0.2 / 30 % | +0.9 / 40 % | +0.1 / 2 % | +0.1 / 2 % | +0.0 / 2 % | -0.0 / 0 % | +0.4 / 24 % |
| callout as announced (k = 1): mean [s] (− early) / late / > 1 s late | -0.8 / 2 % / 0 % | -0.3 / 2 % / 0 % | -0.9 / 4 % / 0 % | -2.6 / 4 % / 0 % | -0.8 / 16 % / 2 % | -0.8 / 2 % / 0 % | -0.8 / 2 % / 0 % | -0.9 / 4 % / 0 % | -0.2 / 12 % / 0 % | -1.6 / 4 % / 0 % |
| branch reported (car takes it) / main reported (car goes straight) | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % | 100 / 100 % |
| largest exit step, p95 [m] | 3.4 | 5.5 | 4.0 | 3.5 | 3.7 | 3.4 | 3.3 | 3.6 | 4.9 | 4.3 |
| not < 10 m 60 s after the exit | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1 |

- **Showing the car.**
  - Along the road, in a straight tunnel at steady speed: 12–13 m after 1.5–2 min, 25–31 m
    with braking or stops.
  - Across the road: 1.3–3 m, which is the map's own lateral uncertainty; the filter cannot do
    better than the map here. Without the map layer, the cross-track error was 26 m on S1 and 106 m at 5 km, because heading
    is not observable in a straight tunnel.
  - The exit curve brings the portal error down to 2.5–14 m.
  - The filter is consistent: mean (e/σ)² 0.11–0.27.
- **Callout.** "The right moment" is read as *never late, and only as early as the uncertainty
  requires*: a late callout can make the driver miss the exit. The table gives the filter's own
  timing (k = 0) and the announced callout (ŝ + k·σ_s, k = 1). With k = 1, one callout in 500 is
  more than 1 s late (S5) and 0–16 % are late at all; on average they come 0.2–2.6 s early.
  Without the rule, 30 % (S4) and 40 % (S5) were more than 1 s late. Why k = 1: σ_s is about twice
  the real error ((e/σ)² 0.1–0.3), so k = 1 is in effect ~2σ; k = 1.65 is never late but up to
  4.4 s early (S4). The voice says the estimated distance rounded to 50 m. k has to be chosen
  again once σ is calibrated on real data.
- **Branch.**
  - Car takes the branch: the in-loop decision reports the branch in 500 of 500 runs, 2–3.5 s
    after the diverge.
  - Car goes straight on (negative control, map with the branch): "main" in 500 of 500 runs. The
    portal error then stays at the no-map value (S1 18 m, S4 45 m: no exit curve), which shows that the map's
    along-track gain comes only from curves.
- **Recovery.** The largest step at the exit is ≤ 6 m (p95). The estimate is within 10 m of truth
  60 s after the exit in 499 of 500 runs.

**Tunnel length** (diverge at min(0.73·L, L − 200 m) [ASSUMED]). At the diverge: 4.8 m after
1 km, 12 m after 3 km, 25 m after 5 km. The error grows faster than linearly, since bias drift
goes as ~t² (`reports/0.5.1/error_vs_distance.png`). At 0.5 km the callout point is still under
GNSS.

## 4. Where it breaks, and why

**4.1 Stops and braking.** ZUPT resets speed at stops, but each stop and start is integrated, and
the bias acts during long stops. S4 and S5 have the largest errors (25–31 m at the diverge). The
k·σ callout turns that into early rather than late callouts: 2.6 s early on average.

**4.2 Driver behaviour is an assumption.** Without GNSS a small real acceleration and an
accelerometer bias look the same (ã = a + b_a + n). Three speed-hold forms were tried:

1. **Absolute hold:** overconfident, (e/σ)² 6–33. The pseudo-measurement's error is the state
   error itself.
2. **Cloned speed-increment constraint:** consistent and 30–50 % better, but it fails once the
   speed wanders by ±1 km/h.
3. **Gauss–Markov wander (default):** consistent in every case, but it keeps only a small part of
   the gain.

The gap between (2) and (3) is the value of "the driver holds speed exactly". It does not come
from the sensors.

**4.3 Sideslip violates the lateral NHC.** In the exit curve the car slips by β ≈ 0.6°
[ASSUMED]: a 0.23 m/s lateral velocity against NHC σ 0.1 m/s, with the same sign throughout the
curve. After α degrees of turn, the lateral row sees the old speed error weighted by sin α. The
sideslip is amplified by about 1/sin α into a speed error, and every seed shows a −20 m transient
(`reports/0.5.0/nhc_trace_S1.png`). Inflating R only slows this down; skipping the row in turns
removes it.

**4.4 Tunnel pressure gradient.** Ventilation can create a longitudinal pressure gradient that
the barometer reads as climb. Its size is [ASSUMED]. At 30 Pa/km the filter reads it as pitch:
the error at the diverge is 110–120 m, (e/σ)² 9–14, and the branch decision is wrong. A gradient
state fixes that case (19 m) but costs accuracy on normal roads (S5 23 → 109 m). The reason is
structural: on a straight tunnel with a constant grade, a constant pitch error and a gradient
both give a height error linear in s, and the barometer cannot tell them apart. The state is off
by default; the gradient should be measured in a real tunnel.

**4.5 What the map adds depends on the map.** Map errors drawn per seed from the prior (1×) and
from twice the prior (2×), 30 seeds (`reports/0.5.2/map_random_stress_30.json`):

| | exact map | 1× (2 m, 0.5 %) | 2× (4 m, 1 %) | no map |
|---|---|---|---|---|
| S2: portal rms / (e/σ)² | 4.1 m / 0.23 | 6.7 m / 0.61 | 11.0 m / 1.7 | 17 m |
| 5 km: portal rms / (e/σ)² | 13.5 m / 0.24 | 23.1 m / 0.71 | 39.3 m / 2.1 | 34 m |

A 0.5 % length error draws the 5 km exit curve 18 m off, and curve matching can only be as good
as the curve's position. Treating map errors as white noise gave (e/σ)² 1.7–1.9 already at 1×.
The fix was map-error states, including the length error's lateral effect after a turn. At 2×
the filter is still somewhat optimistic, and at 5 km the map then does worse than no map. The σ
values are [ASSUMED]: OSM tunnel geometry is traced from imagery, so a 1 % error over 5 km is
plausible. They must be measured on real maps.

**4.6 Low-end IMU.** An MPU9250-class IMU [MEAS, Suvorkin et al. 2024 Table 5] has about the
Note10's accelerometer but 9× its gyro white noise and ~30× its gyro bias instability. That table
comes from one development board and is ~100× the datasheet's typical gyro bias, so this profile
may be pessimistic; a real low-end phone must be measured.

With the Note10 tuning the filter is overconfident, the χ² gate locks GNSS out before the tunnel,
and the error at the portal is 0.3–3.6 km. Four fixes (review 11, `reports/0.5.5/`,
`reports/0.5.7/`):

- **IMU class at start-up** from the gyro white noise over the first 10 s, σ̂ = std(Δω)/√2: Note10
  ≤ 0.013 rad/s per sample, MPU9250 ≥ 0.059, threshold 0.027. Every run was classed correctly and
  Note10 results are bit-identical. A real OS may low-pass the gyro, so the sensor name is safer.
- **A filter profile per class:** the Note10 tuning rules applied to the MPU9250 Allan terms.
  Halving or doubling the gyro bias random walk changes little; one parameter does not fix this.
- **Fast bias terms as white noise (v0.5.7).** The MPU9250's bias instability is three
  Gauss–Markov terms (τ 5, 50, 500 s). The two fast ones decorrelate between the filter's updates,
  so a random walk cannot follow them. Their white-noise equivalent, √(2τ)·σᵢ, is added to the gyro
  noise: √(ARW² + vibration² + Σ_{τ ≤ 50 s} 2τσᵢ²) = 0.063 rad/√s on the worst axis. In a sweep from
  ×1 to ×15 of the ARW-only value, this one brings (e/σ)² near 1 across scenarios. Larger values
  make the filter over-cautious.
- **Cruise threshold 0.05 m/s² (v0.5.8).** With this gyro the filter's along-track acceleration is
  noisier, so the 0.02 m/s² cruise-mode test rarely passed and the driver prior seldom engaged.

| MPU9250, 30 seeds | S1 | S2 | S5 | 5 km | S4 |
|---|---|---|---|---|---|
| Note10 tuning, portal rms (10 seeds) | 1.6 km | 340 m | 1.4 km | 3.6 km | 1.5 km |
| profile (v0.5.5): diverge / portal rms | 45 / 37 m | 14 / 16 m | 78 / 85 m | 206 / 614 m | 740 / 1080 m |
| + fast terms as white noise: diverge / portal rms | 21 / 14 m | 17 / 16 m | 35 / 22 m | 40 / 43 m | 756 / 1099 m |
| + cruise threshold 0.05 m/s² (v0.5.8): diverge / portal rms | 17 / 10 m | 14 / 13 m | 29 / 17 m | 25 / 25 m | 603 / 857 m |
| (e/σ)² at the portal, v0.5.5 → v0.5.8 | 4.8 → 0.16 | 2.0 → 0.62 | 34 → 0.55 | 146 → 0.38 | 700 → 155 |
| k = 1 callouts > 1 s late, v0.5.8 | 0 % | 0 % | 0 % | 0 % | 97 % |

With these fixes a low-end phone works, and is honest about its uncertainty, in every scenario
except stop-and-go: within 10–25 m at the portal and with no late callouts. S4 stays broken
(~1 km), and the cause is low speed, not the stops. A trace of S4 seed 0 shows it:
- During the 30 km/h crawl the filter's speed falls from about 7 to 2.6 m/s in 60 s while the true
  speed stays at 8.3 m/s (−320 m).
- At the second stop the filter still believes it is moving at about 6.6 m/s (−250 m more).
- Yaw drifts 5–7° during the stops, but the map corrects it within seconds of moving off, and the
  cross-track error stays under 1 m.

The NHC and the barometer see a pitch error only through v·δθ, so at 8 m/s they hold pitch about a
third as strongly as at 22 m/s. The low-end gyro then tilts the filter and gravity leaks into the
speed. The same IMU gives 14 m on S1 and ~1 km on S4: the difference is the speed. A perfect stop
detector, levelling at stops, the stop detector's speed gate and the cruise threshold do not
change this (`reports/0.5.7/lowend_s4_level_10.json`). On a phone classed as low-end, the app
should not trust σ at low speed. There it needs a different
model: motion modes, or the CV and map fallback of `DESIGN.md` §5.1.

**4.7 Levelling does not help** (review 11 option 3, `EskfConfig.level`, off). At 1 Hz in cruise
mode and at stops, the 1 s mean specific force is modelled as gravity plus the accelerometer bias,
f̄ = −Cᵀg + b_a. With σ = 0.02 m/s² (10 seeds) it halves the S4 diverge error (29 → 14 m) but
worsens the portal on S2, S3, S5, S6 and 5 km, with (e/σ)² up to 13: grade changes and speed
wander are real, correlated accelerations, and the smaller σ_s makes the map's curve matching
correct less at the exit branch (S1 portal 4.4 → 14 m). σ = 0.05 gives little harm and little
gain; at stops only it changes nothing (30 seeds), since the ZUPT already holds there. Restricted to crawling speed (< 30 km/h) it changes S4 by under 1 m (30 seeds, `reports/0.5.7/level_crawl_30.json`). It stays off.

**4.8 Assumptions that did not matter** (S1, 30 seeds): GNSS coasting into the tunnel; optimistic
reported accuracy; a 1–3° mount error (absorbed by the lumped accelerometer bias while the mount
stays rigid); the shape of the bias model.

## 5. Limitations and next steps

- **Simulation only.** The IMU noise terms are measured. The time shape of the noise, vibration,
  sideslip, barometer, map and GNSS behaviour are [ASSUMED].
- **Simplified map errors.** Map errors are a lateral offset and a length scale. Shape errors,
  missing curves and a wrong diverge position are not simulated. The truth car drives on the lane
  centre.
- **The ZUPT detector is offline** (forward-backward filtering). A causal version is similar at
  the exit and worse for the callout.
- **Narrow setting.** Two IMU classes, a rigid phone mount, a single lane, one car. A second car
  would only be another noise realisation of the same sensors; the 50 seeds cover that.
- **Next, with real devices:**
  - static Allan runs to check §3 of `DESIGN.md`;
  - recordings in Istanbul tunnels, with the portals as checkpoints;
  - measuring the map's polyline error and the tunnels' pressure gradient;
  - a causal stop detector;
  - the IMU class from the sensor name, and a low-end profile from a real phone's Allan run.
- **Next, in the filter, for low-end IMUs** (not measured):
  - the barometer at 25 Hz with its own correlation model instead of the 1 s mean, to hold pitch
    more often;
  - the gyro bias as a sum of Gauss–Markov terms (τ 5/50/500 s, as in the truth model) instead of
    a random walk;
  - the CV + map fallback of `DESIGN.md` §5.1 for stop-and-go and long tunnels;
  - Android's calibrated gyro stream, which removes the turn-on bias at stops but not the drift in
    the tunnel.

## 6. AI usage

**How AI was used.**
- **The builder.** An AI coding agent, Claude (Anthropic), worked in a cloud workspace on this
  repository. It wrote the code, the tests, the experiments and the drafts of every document.
- **The reviewers.** The author had the repository reviewed by other AI models, Claude Fable 5.1
  among them. Their reviews are the Markdown files in `docs/reviews/`: reviews 2–5 and 8–12.
  The author read each review and passed on only what he accepted. Reviews 6 and 7 are the
  author's own.
- **The workflow.** Before each piece of code, the agent proposed options with formulas and the
  expected effect. The author approved, changed or rejected them before anything was written
  (`docs/decisions/`). Results came back as versioned metrics snapshots, which the author
  reviewed.

### 6.1 Where the author decided, corrected or overruled

| Area | What the author did | Record |
|---|---|---|
| Scope and rules | Set the assumptions: fixed mount; no magnetometer, BLE, OBD or camera; map available; Suvorkin IMU profiles. Set the working rules: tags, truth/estimator split, consistency as acceptance, robust over optimal, a snapshot per version | DESIGN §2, §12 |
| Sensors and plan | Approved sensor decisions K1–K9 and plan V1–V6, including the independent oracle flags and two separate callout metrics | `docs/decisions/` |
| Focus (review 5) | Redirected the work: first a setup where the bias can be estimated (approach manoeuvres), then remove assumptions one at a time | `2026-10-20-focus.md` |
| Filter | Approved design E1–E6. Decided re-acquisition as "robust over fast". Had both speed-hold variants tested, then chose which to switch on | v0.5 decision note |
| Cruise model (review 6) | Proposed the Gauss–Markov driver-wander model. **Rejected the agent's claim** that wander and bias separate in ~50 s. Required (e/σ)², not only rms, as the acceptance test | review 6 |
| Exit branch (review 6) | A diverge in every scenario, the callout before it, and results reported before and after it so the curve does not flatter the straight tunnel | review 6 |
| NHC in curves (review 7) | Chose the turn gate over R inflation. Required the mechanism on a trace, a gate-sensitivity check, a no-sideslip check and a negative branch test. Put deliverables before extra metres | review 7 |
| Pressure gradient | Accepted the gradient state for testing; proposed the pitch–gradient mechanism behind its cost, confirmed on a trace; kept it off | v0.5 decision note |
| Map in the tunnel | Designed the layer after review 9. Chose map errors as states, not as white noise. Set the acceptance criteria | v0.5 decision note |
| Callout | Asked for callouts that err early, not late. **Overruled k = 1.65** (proposed by the agent): σ_s is already about twice the real error, so k = 1. Added the k = 0 row and the definition of "the right moment" | CHANGELOG v0.5.4 |
| Low-end IMU (review 11) | Chose which options to build: IMU class, profile, levelling A/B | CHANGELOG v0.5.5 |
| Simulator | Set the requirements: one page with tabs, live re-runs, the driver's view, full screen, the results table, the sliders. Found the playback bug at 1–10× on GitHub Pages | CHANGELOG v0.5.4–0.5.6 |
| Low-end S4 mechanism | **Corrected the agent's diagnosis** (yaw drift at stops) from a trace: the filter loses speed while crawling, because pitch is weakly observable at low speed. Found that the in-browser page could not work on GitHub Pages because `*.zip` was git-ignored | §4.6, CHANGELOG v0.5.7 |

### 6.2 Choices the agent made

These are implementation details marked [ASSUMED], proposed by the agent and accepted without
change. The author is ready to explain or change any of them:
- **Noise and gating:** effective process noise including vibration; GNSS R factors 2 and 3.2;
  the χ² gate level; the gate-lock recovery and adaptive-noise thresholds.
- **Detectors:** the stop-detector bands and thresholds; the cruise-mode entry test
  (0.02 m/s² over 5 s).
- **Exit and map:** the re-acquisition timings (3 s velocity only, ×10 R decaying over 8 s,
  ≤ 2 m per step); the map windowing and pause distances.
- **Low-end and levelling:** the IMU-class threshold; the MPU9250 profile rules; the levelling
  gate.
- **Scenario details:** speeds, stop durations, exit positions, branch geometry.

### 6.3 Errors on the AI side, and what caught them

| Error | Caught by |
|---|---|
| Attitude-error sign; an oracle bound presented as achievable | AI reviews 2–5 |
| Observability claim stronger than the model (wander vs bias) | the author (review 6) |
| Innovation sign; a barometer index that never advanced; a stop detector that fired at launch | the agent's own tests and Monte Carlo runs |
| Absolute speed hold overconfident; 0.5 s barometer timestamp lag; map errors as white noise | consistency (e/σ)² as an acceptance metric |
| Sideslip/NHC −20 m transient; its mechanism | Monte Carlo traces; mechanism shown at the author's request (review 7) |
| Cross-track error that no metric measured | AI review 9 |
| A map stress test that only relabelled s and so tested nothing | found before its results were used; redone with a stretched polyline |
| k = 1.65 without a justified magnitude | the author |
| Voice said a fixed "300 m"; early/late label fixed to "early" | AI review 11 |
| Levelling gate on the per-sample gyro maximum (noise) | the agent's sweep (the update never fired in cruise) |
| Playback did not advance at 1–10× | the author, on GitHub Pages |
| Low-end S4 explained as yaw drift at stops; it is low-speed pitch unobservability | the author, from a trace |
| `browser.html` published without its package (`*.zip` git-ignored) | the author |

**Process controls,** used because generated code can be confidently wrong:
- parameter tags;
- a test that enforces the truth/estimator split;
- (e/σ)² as an acceptance criterion;
- a CHANGELOG entry and a metrics snapshot per version, with every changed metric explained;
- decisions and reviews kept as commits;
- release tags set by the author.
