# Through the Tunnel: write-up

A two-page version is in [`SUMMARY.md`](SUMMARY.md).

Pointr case study. A phone in a car estimates its position through a GNSS outage in a road
tunnel. The navigation app has to keep showing the car, call out an exit inside the tunnel at the
right moment, and recover cleanly when the car comes out.

All results come from simulation. The main results are six independent sets of 50 seeds
(`tools/eval_sets.py`, `reports/0.5.13/`) plus a held-out validation set of 100; §3 explains the
three seed tiers. Earlier sections quote 30–50 seeds where they say so. Every parameter is tagged [MEAS] (measured,
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
| Phone | Rigid mount at unknown angles, estimated in the first 90 s of open sky (§4.14; until v0.5.12 known to the filter) | [ASSUMED] |
| Magnetometer | Open sky only: 47 µT, inclination 56°, declination 6°; the car's field 3 µT, soft iron 2 %, noise 0.4 µT | [ASSUMED] |
| IMU | Galaxy Note10 (LSM6DSR) Allan terms; turn-on bias 10 mg | [MEAS]; [SPEC] |
| IMU | Shape of the bias drift (three Gauss–Markov terms), vibration levels | [ASSUMED] |
| GNSS | 3 m horizontal, 0.1 m/s, correlated (30 s / 10 s); fade before the portal; 30 m exit multipath | [ASSUMED] |
| Barometer | 0.1 m white + 0.1 m offset drift (60 s); closed cabin; no tunnel pressure gradient (30 Pa/km tested, §4.4) | [ASSUMED] |
| Map | Route polyline with the diverge; lateral offset σ 2 m and length error σ 0.5 % (§4.5); no grade profile | [ASSUMED] |
| Vehicle | Sideslip 0.4°/(m/s²) with a 0.2 s lag; suspension pitch 1.5°/g | [ASSUMED] |
| Driver | In cruise the speed wanders like a Gauss–Markov process, σ 0.5 m/s, τ 60 s (the filter's model) | [ASSUMED] |
| Exit | 20° branch over 150 m at about 0.73 of the tunnel; callout 300 m before it, k = 1 | [ASSUMED] |

Sensors left out:
- **Magnetometer in the tunnel:** the tunnel's steel distorts it. It is read only in open sky, to
  start the mount yaw (§4.14).
- **Ambient light:** it could mark the portal, but GNSS loss already does, and tunnel lighting
  levels are unknown.
- **Camera, BLE beacons, OBD and wheel speed:** an average navigation user does not have them, and a
  tunnel with beacons is not a fair assumption.

## 3. Results

In every scenario the route leaves the main tunnel through a 20° branch inside it, and the
callout comes before that diverge. "At diverge" is the straight-tunnel result. "At portal" is
the result when the car comes out.

**Seeds, in three tiers.**
- **Development, 0–49:** every tuning in this repository was chosen on these.
- **Feature design, 50–299:** the map re-association of §4.11 was designed on seeds 63, 72, 103,
  113 and 114, and its first form's error was found on all of 0–199. In v0.5.12, 200–299 moved
  down to this tier: the branch window of §4.13 came from seed 247, so those seeds are no longer
  held out.
- **Validation, 400–499:** run once on the v0.5.13 configuration and not used to change anything
  (§3.1). Each version's validation set is spent once: 200–299 (v0.5.11, `reports/0.5.11/`),
  300–399 (v0.5.12, `reports/0.5.12/`); neither was used to design v0.5.13.
- **Method:** 20 seeds to look, 300 for the number. Four times a 10–50-seed result was optimistic
  ("1 in 500" late callouts on 0–49; low-end S4 failures 35 → 0 on 20 seeds against 35 → 6 on 300;
  the mount without a manoeuvre, (e/σ)² 0.89 on 10 seeds against 1.69 on 300).

The ranges below are over six independent sets of 50 seeds (0–299), final configuration: since
v0.5.13 the filters use the mount estimated on the approach, not the true one (§4.14;
`reports/0.5.13/eval_sets_300_default.json`).

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | S1-1km | S1-5km |
|---|---|---|---|---|---|---|---|---|---|---|
| | 3 km straight | S-bend | V-profile | stop-and-go | braking | wander ±1 km/h | ±2 km/h | ±4 km/h | 1 km | 5 km |
| along-track at diverge, rms [m] | 11.5–16.9 | 4.2–5.2 | 11.6–15.7 | 35.0–48.4 | 24.8–35.6 | 11.9–17.6 | 11.6–18.4 | 11.3–18.1 | 4.1–5.4 | 24.5–38.3 |
| along-track at portal, rms [m] | 5.6–7.1 | 4.0–5.5 | 8.9–11.8 | 4.5–7.9 | 6.2–8.7 | 6.0–7.5 | 6.0–8.0 | 5.7–8.0 | 2.4–3.2 | 12.9–16.7 |
| portal p95, worst set [m] | 15 | 10 | 23 | 12 | 16 | 15 | 16 | 15 | 7 | 30 |
| runs > 100 m off at the portal (of 300) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| mean (e/σ)² at portal, worst set | 0.22 | 0.38 | 0.28 | 0.14 | 0.37 | 0.23 | 0.27 | 0.26 | 0.36 | 0.37 |
| callout > 1 s late, worst set: k = 0 / 1 / 1.3 / 1.65 | 12 / 0 / 0 / 0 % | 0 / 0 / 0 / 0 % | 10 / 0 / 0 / 0 % | 46 / 4 / 4 / 2 % | 56 / 12 / 6 / 0 % | 10 / 0 / 0 / 0 % | 14 / 0 / 0 / 0 % | 8 / 0 / 0 / 0 % | 0 / 0 / 0 / 0 % | 46 / 4 / 0 / 0 % |
| callout mean, k = 1 [s] (− early) | -0.6…-0.8 | -0.3 | -0.8…-1.0 | -2.1…-2.5 | -0.3…-0.8 | -0.6…-0.8 | -0.6…-0.8 | -0.6…-0.9 | -0.1…-0.2 | -1.2…-1.6 |
| largest exit step p95, worst set [m] | 4.4 | 6.3 | 4.1 | 4.1 | 4.6 | 4.4 | 4.2 | 4.3 | 4.9 | 4.3 |
| shown position under 5 m after the first fix, median per set [s] (v0.5.13) | 13–17 | 12–16 | 11–17 | 11–18 | 12–14 | 13–16 | 12–17 | 12–17 | 0–16 | 10–12 |
| "now" prompt: mean true distance minus the target [m] (+ early), missed (of 300) | 15–21, 0 | 6–7, 0 | 16–21, 0 | 40–48, 2 | 2–11, 0 | 14–20, 0 | 14–20, 0 | 14–22, 0 | 4–5, 0 | 26–36, 0 |

Cross-track error at the portal is 1.3–3 m (v0.5.10, 50 seeds), the map's own lateral uncertainty.
Branch decision: the branch is reported in 2999 of 3000 runs that take it (S4 seed 271 decided
"main", 41 m off at the portal; 3000 of 3000 with the true mount). When the car goes straight on,
"main" is reported in 599 of 600 on 0–299 and in 200 of 200 in validation; one run stayed
undecided and none reported the branch.

- **Showing the car.**
  - Along the road, in a straight tunnel at steady speed: 12–18 m after 1.5–2 min, 25–50 m with
    braking or stops.
  - Across the road: 1.3–3 m, which is the map's own lateral uncertainty; the filter cannot do
    better than the map here. Without the map layer, the cross-track error was 26 m on S1 and
    106 m at 5 km, because heading is not observable in a straight tunnel.
  - The exit curve brings the portal error down to 2.5–16 m.
  - The filter is consistent: mean (e/σ)² ≤ 0.38 in every set.
  - Low-end phones: §4.12 (the 10-state filter is shown there, 16–21 m at the portal on S4 in five
    of six sets in v0.5.13).
- **Callout.** "The right moment" is read as *never late, and only as early as the uncertainty
  requires*: a late callout can make the driver miss the exit.
  - **Without the rule (k = 0):** 0–56 % of callouts come more than 1 s late (worst set per
    scenario; 0 % only on the S-bend and the 1 km tunnel).
  - **With k = 1:** 20 of 3000 callouts are more than 1 s late (15 on S5, 3 on S4, 2 at 5 km),
    and on average they come 0.1–2.5 s early. The development set alone gave "1 in 500"; over 300
    seeds S5 is at 2–12 % per set.
  - **With k = 1.3:** 5 of 3000 (3 on S5, 2 on S4); with k = 1.65, 1 (S4). In the v0.5.13
    validation, k = 1 gives 6 of 1000 and k = 1.3 none (§3.1). The cost is 0.3–0.9 s more advance (S4: 2.2 →
    3.0 → 4.0 s early on average).
  - **Why k = 1 stays:** σ_s is about twice the real error ((e/σ)² 0.1–0.4), so k = 1 is in effect
    ~2σ. The k = 1.3 row is the measured alternative; it cuts the late callouts fourfold (20 → 5 in 3000).
  - k has to be chosen again once σ is calibrated on real data. The voice says the estimated
    distance rounded to 50 m.
- **Branch.**
  - Car takes the branch: the in-loop decision reports it 2–3.5 s after the diverge.
  - Car goes straight on (negative control, map with the branch): "main" in 599 of 600. The
    portal error then stays at the no-map value: S1 18–28 m rms, S4 52–79 m rms, because there is
    no exit curve. Until "main" is decided (5 s on S1, 13 s on S4 after the diverge) the screen
    shows the car on the branch, up to 38 m from the road it is on (§4.13).
  - **On S4, 34 of 300 straight-on runs are more than 100 m off at the portal** (3, 5, 7, 9, 2
    and 8 per set; 9 of 100 in validation). About one stop-and-go drive in eight stays in the main
    tunnel and is shown more than 100 m off when it comes out. This is the measured cost of "the map's
    along-track gain comes only from curves": with no curve before the exit, nothing corrects the
    dead reckoning until GNSS returns, and then the exit follower.
- **Recovery.** The largest step at the exit is ≤ 6.3 m (p95, worst set). The time until the shown
  position is under 5 m for good is a median of 10–18 s per set (v0.5.13; the supervisor's
  follower in §4.12); 12–34 runs of 300 per scenario are not under 5 m within 60 s.

**3.1 Validation on untouched seeds** (400–499, v0.5.13 configuration, run once;
`reports/0.5.13/validation_400-499.json`, `ekf10_validation_400-499.json`, `supervisor_validation.json`).
Nothing was changed after this run.

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | 1 km | 5 km |
|---|---|---|---|---|---|---|---|---|---|---|
| diverge rms, two sets [m] | 13.9 / 16.5 | 4.4 / 4.5 | 14.2 / 17.9 | 37.8 / 39.5 | 26.8 / 30.8 | 15.8 / 17.5 | 15.8 / 17.8 | 15.1 / 17.1 | 4.6 / 4.9 | 27.8 / 34.2 |
| portal rms, two sets [m] | 6.3 / 7.6 | 5.2 / 5.2 | 9.6 / 11.9 | 6.5 / 6.7 | 7.2 / 8.3 | 7.1 / 8.0 | 7.1 / 8.2 | 7.2 / 8.0 | 3.2 / 3.1 | 15.7 / 15.2 |
| runs > 100 m off (of 100) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| worst (e/σ)² | 0.24 | 0.35 | 0.28 | 0.15 | 0.33 | 0.27 | 0.28 | 0.26 | 0.35 | 0.33 |
| > 1 s late of 100, k = 1 / 1.3 | 0 / 0 | 0 / 0 | 0 / 0 | 2 / 0 | 3 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 1 / 0 |

- **Inside the ranges of §3,** except the S3 diverge error in one set (17.9 m against 11.6–15.7 m)
  and the S5 portal error in one set (8.3 m, at the top of 6.2–8.7 m).
- **No run is more than 100 m off;** the branch is reported in 1000 of 1000 runs that take it.
- **Straight-on control:** "main" in 200 of 200; 9 of the 100 S4 runs are more than 100 m off at
  the portal (§3).
- **Low-end, product configuration (supervisor):** S4 portal 14.4 / 20.0 m, (e/σ)² 0.48 / 1.21
  without the monitor and 0.24 / 0.35 with it (§4.12). The 10-state filter alone: 17.4 m, no run
  over 100 m (v0.5.12 configuration: 19.0 m).
- **The validation sets of v0.5.11 and v0.5.12** each found something (one S4 failure; ranges
  exceeded on S4). This one found nothing new; with one set of 100 that is not a guarantee for
  failures rarer than about 1 in 100.

**Tunnel length** (diverge at min(0.73·L, L − 200 m) [ASSUMED]). At the diverge: 4.8 m after
1 km, 12 m after 3 km, 25 m after 5 km. The error grows faster than linearly, since bias drift
goes as ~t² (`reports/0.5.1/error_vs_distance.png`). At 0.5 km the callout point is still under
GNSS.

## 4. Where it breaks, and why

**4.1 Stops and braking.** ZUPT resets speed at stops, but each stop and start is integrated, and
the bias acts during long stops. S4 and S5 have the largest errors (25–35 m at the diverge). The
k·σ callout turns that into early rather than late callouts: 2.3 s early on average.

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

With the causal stop detector (v0.5.10), S4 is 638 / 894 m with (e/σ)² 183. The other columns
have no stops and do not change.

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

**4.9 Three filter sizes: 4, 10 and 16 states** (owner 2026-10-24/25; `tunnelsim/reference1d.py`,
`tunnelsim/ekf10.py`, `tools/compare_filters.py`, `reports/0.5.11/compare_filters_50.json`; the ESKF
column is the final configuration with re-association, seeds 0–49). Two
smaller filters ran on the same sensor data and seeds as the ESKF, with the same stop detector,
GNSS R inflation, P0 and IMU-class profile. Both treat the map as a hard constraint.
- **4 states (1D):**
  - states: s, v, a forward accelerometer bias, a yaw-gyro bias;
  - pitch is read from the map's grade: v̇ = f_x − b_a − g·sin γ(s);
  - the gyro gives speed in curves: ω_z = v·κ(s) + b_g, at 1 Hz on the 1 s mean rate, with the gyro
    bias instability in R;
  - a common design, re-implemented here from its description.
- **10 states:**
  - states: s, v, height, pitch, yaw, forward accelerometer bias, pitch and yaw gyro biases,
    barometer offset, yaw-gyro scale;
  - pitch is a state held by the barometer; the map grade is not used;
  - map heading is matched globally along the arc (H_s = −κ);
  - the curve-speed update carries sideslip and lane wander in R;
  - built in a separate AI session from 8 states upwards, each addition from a trace (below).

50 seeds, paired, causal stop detector. Along-track rms at the diverge / at the portal [m], (e/σ)² at
the portal:

| Note10 | 4 states | 10 states | 16 states (ESKF) |
|---|---|---|---|
| S1 straight | 10.8 / 8.6, 6.0 | 10.6 / 5.1, 0.77 | 12.1 / 5.5, 0.13 |
| S2 S-bend | 3.9 / 3.7, 6.4 | 3.8 / 3.5, 1.1 | 3.5 / 4.1, 0.23 |
| S3 V profile ±5 % | 5.1 / 4.7, 1.0 | 12.0 / 8.7, 0.45 | 12.1 / 9.0, 0.18 |
| S4 stop-and-go | 35.2 / 94.3, 914 | 22.7 / 25.2, 8.8 | 34.7 / 4.7, 0.08 |
| S5 braking | 18.5 / 5.5, 6.6 | 20.1 / 3.8, 0.92 | 25.0 / 6.5, 0.21 |
| S6 speed wander | 12.4 / 9.2, 6.2 | 12.0 / 5.2, 0.77 | 13.3 / 6.1, 0.15 |
| 5 km straight | 22.6 / 86.3, 133 | 20.3 / 18.6, 2.0 | 24.6 / 12.8, 0.22 |
| exit step p95 | 2–106 m | 8–39 m | 3–5 m |
| car goes straight on at the diverge | follows the branch | 147–439 m off at the portal | "main" in 500/500 (v0.5.10, 50 seeds × 10 scenarios) |

| Low-end IMU | 4 states | 10 states | 16 states (ESKF) |
|---|---|---|---|
| S1 | 11.2 / 12.2, 0.36 | 10.9 / 9.9, 0.30 | 16.7 / 10.5, 0.19 |
| S2 | 13.4 / 14.7, 0.86 | 12.0 / 11.3, 0.66 | 14.9 / 16.7, 0.94 |
| S3 | 5.7 / 18.6, 1.5 | 16.2 / 16.8, 0.24 | 22.6 / 17.9, 0.47 |
| S4 | 64.9 / 81.5, 16 | 64.2 / 93.8, 14 | 626 / 868, 162 |
| S5 | 17.4 / 10.8, 0.31 | 15.8 / 7.4, 0.19 | 29.8 / 16.6, 0.53 |
| S6 | 11.5 / 11.7, 0.32 | 11.2 / 9.0, 0.26 | 20.6 / 19.0, 0.58 |
| 5 km | 20.7 / 19.8, 0.48 | 17.6 / 16.0, 0.47 | 29.3 / 25.8, 0.49 |

These are v0.5.11 numbers; v0.5.12 gave the 10-state filter a curve landmark, its own branch decision
and a one-way speed constraint (§4.13: low-end S4 portal 94 m rms, 22 m without its 2 failures, in
300 seeds). The filter's mount pitched 2° wrong changes no filter by more than 2 m (lumped bias). The 4-state
filter with a map grade 10 % too steep or too shallow, S3, 50 seeds: portal 117 / 85 m,
(e/σ)² 710 / 290, and with the shallow grade every callout comes more than 1 s late
(`reports/0.5.11/ekf1d_grade_error_50.json`); without any grade, 3.4 km.

Readings:
- **For the callout, accuracy does not separate them on a mid-range phone.** At the diverge, where
  the callout is decided, the three agree within 2 m on S1, S2, S6 and 5 km. The 4-state filter is
  better on S3 and S5, the 10-state one on S4.
- **What the 16 states buy is independence from assumptions:**
  - from the map's grade (4 states: 3.4 km without it, ~100 m with a 10 % error);
  - from the vehicle model (4 states: suspension pitch, 94 m on S4; with the suspension pitch
    turned off in the truth, 4.9 m);
  - a σ that covers the error, which the k·σ callout rule needs (4 states: (e/σ)² up to 914);
  - a branch hypothesis, map errors as states and a capped exit (the other two jump at the exit and
    follow the wrong road when the car goes straight on).
- **On a low-end IMU the order reverses.** The ESKF trusts its gyro for pitch and breaks at crawling
  speed (§4.6); the smaller filters take pitch from the map (4 states) or hold it with the barometer
  and a scalar speed (10 states). The 10-state filter is best or equal in six of seven scenarios.
- **Why the 4-state filter loses in curves.** Its curvature-gyro update is local in s. In 5 km seed
  2 it reaches the exit curve 32 m ahead and expects the turn too early. The linearised update puts
  the mismatch into speed (−1.3 m/s), σ_s falls from 35 to 0.3 m, and the straight to the portal
  turns that into −64 m. Matching the map heading instead is global along the arc and corrects
  both ways; the ESKF's map layer and the 10-state filter do that.
- **How the 10 states were found.** The prototype started at 8 states and was over-confident
  ((e/σ)² 4–13). Each addition came from a trace:
  1. sideslip at the clothoids, read as position: a k·v³·κ′ term in R (S1 portal 8.5 → 3.4 m);
  2. the yaw-gyro scale error enters the speed from the curve: a state;
  3. a real car wanders in its lane: an R floor, the counterpart of the ESKF's 1° map-heading σ
     (5 km portal 23 → 7.5 m);
  4. a filter ahead of the car leaves the clothoid early and loses the position information: yaw
     and the global heading match (S4 portal 18.6 → 5.7 m, 4 seeds).
  None of these is in the 4-state design; each is something the 16-state filter had already shown.
- **The cost of the 16 states is not computation.** Order of magnitude [ASSUMED]: ESKF ~4·10⁵,
  10 states ~5·10⁴, 1D ~10⁴ flop/s, all well under 10⁻³ of a phone core, and all read the same
  sensors at the same rates. The cost is the tuning and verification surface: `EskfConfig` has 82
  fields, and most defects found in this work (gate lock, sideslip in the NHC, the barometer lag, the
  over-confident speed hold) are classes of error a 4-state filter cannot have.

**4.10 A pitch prior from the road for low-end phones** (owner 2026-10-25, A/B, off by default;
`EskfConfig.pitch_prior_*`, `zupt_trust_s`; `reports/0.5.11/pitch_prior_ab_30.json`).

Why it is needed: folding the fast gyro-bias terms into white noise (v0.5.7) kept the low-end filter
consistent, but it made the attitude process noise so large that pitch spreads ~3.6°/√s. At low
speed the NHC and the barometer see pitch only through v·δθ, so an outside source of pitch is
needed, and the road geometry is that source.

The prior is θ_b = γ(ŝ) ± σ, applied at 10 Hz below 10 m/s in the outage, with
∂θ_b/∂δθ = [u_y, −u_x, 0]/cos θ_b (checked by finite differences).

A trace of low-end S4 found a second mechanism. The speed is lost while crawling. When the car
stops, the filter still believes it moves at ~5 m/s, and the ZUPT's speed gate ("filter speed
< 2 m/s") then keeps the stop from correcting it: the gate relies on the very state it should
correct. A new gate source fixes that: a stop the detector has held for ≥ 2 s is applied whatever
the filter speed.

Low-end S4, 30 seeds, diverge / portal [m], (e/σ)², callouts > 1 s late:

| | low-end S4 |
|---|---|
| current | 638 / 894, 183, 100 % |
| trusted-stop gate only | 625 / 876, 203, 97 % |
| prior with an offset state δγ, σ 1° | 189 / 268, 50, 53 % |
| absolute prior, σ 1°, + trusted-stop gate | 73 / 88, 18, 30 % |
| 4-state 1D (reference) | 64 / 80, 15 |

Other results:
- **Other scenarios:** low-end S1, S2, S3, S5, S6 and 5 km are bit-identical in every variant (the
  prior never fires above 10 m/s).
- **Note10 S4:** the gate source changes nothing; with the prior, 33.7 / 4.7 → 36.1 / 6.7 m.
- **Graded stop-and-go** (3 %, 30 km/h, 60 s stop): 1507 m today. With the absolute prior, 73 m with
  the grade in the map and 648 m without it.
- **Mount pitched 1° / 3° wrong:** the absolute prior gives 32 / 102 m.

Readings:
- **The absolute prior works** and brings the ESKF to the 1D baseline's level on low-end S4.
- **It is not consistent** ((e/σ)² 18). The remaining over-confidence comes from the map layer's
  curve matching at the exit, where σ_s collapses while the error is ~100 m: the 4-state filter's
  curve mechanism again.
- **It needs the map grade,** which the ESKF otherwise avoids.
- **The offset state did not work as intended.** It restarts at each outage, and at low speed a slow
  gyro drift and a constant offset are the same unobservable direction, so the drift goes into the
  offset.
- **Status:** the prior stays off. Next steps: learn the offset at the portal, where pitch is still
  known from GNSS, and an innovation gate on the map rows (§5).

**4.11 A failure found on new seeds, and the map re-association that fixed it** (owner
2026-10-25; `EskfConfig.map_nis_gate`, `map_reassoc`; `tools/eval_sets.py`, `tools/failure_cases.py`,
`tools/perseed.py`; `reports/0.5.11/`).

*The failure.* On seeds 50–99, S4 ended 694 m and 571 m off at the portal in 2 runs (seeds 63, 72),
with (e/σ)² 44 for the set. The trace of seed 63:
- The filter reaches the diverge 94 m behind the car, consistently (σ_s 81 m).
- When the car turns into the branch, ŝ is still on the straight before it, where κ(ŝ) = 0.
- The heading row then carries no position information and writes the whole 12–20° innovation
  into yaw, and the cross-track row pulls the car sideways: −478 m along, 100 m across, 12° in
  heading.

This is the same mechanism as the 4-state filter's curve problem (§4.9): a point linearisation
at a wrong ŝ.

*Four forms, 200 seeds each* (S4 Note10 unless noted; thresholds fixed before the runs):

| | S4 runs > 100 m | S4 portal rms, per set | S4 worst (e/σ)² | other scenarios | straight-on control |
|---|---|---|---|---|---|
| none (v0.5.10) | 2 (−694, −571 m) | 5.8–127 m | 44 | – | "main" 400/400 |
| NIS gate, χ² 99.9 % on the map rows | 3 (+104, +117, +121 m) | 5.2–33 m | 1.2 | unchanged | unchanged |
| gate + branch deferral (σ_s > clothoid length) | 0 | 31–35 m | 0.5 | unchanged | unchanged; branch decided in 18 % |
| re-association, v1 (also before the decision) | 0 | 4.9–5.9 m | 0.17 | better | 92–170 m, 242 runs > 100 m |
| re-association on the decided branch, v2 | 0 | 4.9–5.9 m | 0.17 | better (S5 3.6–4.2 m) | unchanged |
| **v2 + the map length-error state, v3 (default)** | **0** | **4.7–6.0 m** | **0.13** | equal or better | unchanged |

How the gate and the forms behave:
- **The gate alone fixed the two runs and broke three others,** as predicted in review: a large
  innovation is sometimes the useful one. In seeds 103, 113 and 114 the filter enters the exit
  curve ~70 m behind. Without the gate the curve matching corrects that at once. With the gate,
  the large first innovations are rejected and the later small ones accepted, a biased subset that
  overshoots to +110 m.
- **The branch deferral switched the map layer off through the exit curve** and lost its
  along-track information.
- **Re-association** (the owner's proposal) evaluates candidates s_c in ŝ ± 3σ_s at 1 Hz:
  J(s_c) = wrap(ψ̂ − ψ_map(s_c))²/(σ_ψ² + P_ψψ) + (s_c − ŝ)²/σ_s².
  If J(ŝ) − min J > χ²₁ 99.9 %, a position pseudo-measurement pulls ŝ to the best candidate, with
  R = max(5 m, σ_ψ,eff/|κ|)². In a curve the gyro-held yaw says where in the curve the car is;
  a linearisation at a wrong ŝ cannot. With it on, the gate no longer fires on nominal runs and
  stays as a safety net. Two simplifications err conservative:
  - the prior enters twice, once in J and again through P in the update;
  - the 5 m R floor stands in for the sideslip shift k·v² (3.4 m at 22 m/s), which a faster car
    would exceed.
- **v1 ran before the branch decision.** A car going straight on, matched to the branch
  hypothesis, was pulled back to the straight before the diverge every second; the straight-on
  control found it.
- **v2 trusted the map's geometry too much:** with twice the assumed map error, S5 went from 15 to
  28 m with (e/σ)² 13. **v3** adds the map length-error state to the re-association row, as the
  other map rows have it: R then includes the map's uncertainty.

*Known failures, seeds 0–99* (`failure_cases_100.json`; portal rms, (e/σ)²):

| | none | gate | v3 | v3, supervisor with the majority monitor |
|---|---|---|---|---|
| pressure gradient 30 Pa/km, S1 (wrong branch in 100 %) | 309 m, 196 | 101 m, 12.6 | 101 m, 12.6 (no re-association: decision "main") | (e/σ)² 1.8, alarm in 25 % of runs |
| pressure gradient 30 Pa/km, S3 | 510 m, 945 | 103 m, 8.1 | 103 m, 8.1 | 1.05, alarm 61 % |
| map error 2× (4 m, +1 %), S1 | 19 m, 1.6 | 19 m, 1.6 | 20 m, 1.7 | 1.5 |
| map error 2×, S4 | 66 m, 5.3 | 39 m, 2.9 | 27 m, 2.4 | 1.5 |
| map error 2×, S5 | 15 m, 1.1 | 16 m, 1.2 | 18 m, 1.6 | 1.3 |

Readings:
- **The gate is a real safety net where the geometry is wrong:** 3400–3800 rejections cut the
  pressure-gradient error threefold and the over-confidence 15–100 fold.
- **Re-association does not make that failure look safer,** because it only runs on a decided
  branch.

*Per seed, 200 seeds, none → v3* (`perseed_base_vs_gate-reassoc.json`, v2 form):
- **Most worsened seed per scenario:** S4 seed 1, 1.1 → 12.5 m (e/σ 1.2); S3 seed 82,
  4.6 → 11.6 m; S5 seed 87, 0.7 → 7.9 m.
- **Seeds worse by > 5 m:** 0–4 per scenario.
- **Seeds better by > 5 m:** 0–71 per scenario (S5 71, S4 30).

*Seed discipline.*
- This feature was designed on seeds the development set did not contain.
- What it says about held-out performance comes from the validation tier (§3.1).
- The lesson is the same one the callout claim taught ("1 in 500" on 0–49, 0–10 % per set over
  200): tune on one set and report on another.

**4.12 A supervisor over the three filters** (owner 2026-10-25; `tunnelsim/supervisor.py`,
`tools/supervisor_eval.py`; `reports/0.5.11/supervisor_{dev,heldout}.json`;
`reports/0.5.12/`; since v0.5.13 `reports/0.5.13/supervisor_sets_{0-99,100-199,200-299}.json`,
`supervisor_validation.json`). All three filters run on the same phone data; the
supervisor decides what the app shows. It sees only estimator-side outputs.

*Selection, decided once before the tunnel.*
- **Class rule (approved):** ESKF on a Note10-class IMU, 10-state on a low-end one.
- **Open-sky rehearsals (measured, rejected):** shadow copies of each filter ran without GNSS for
  30 s, three times on the approach, and were scored against the next fix.
  - The three filters scored alike (rms ~4 m, (e/σ)² ~0.4). Dead reckoning differs only after
    ~100 s, and 30 s of it is hidden in the GNSS's own 3–4 m.
  - A rule picking the best rehearsal chose the 1D filter in 55–60 % of runs: Note10 S4 portal
    66–92 m, 5 km 29–86 m (held-out seeds).
  - The rehearsals may still serve as a health check for gross faults (a wrong mount, a broken
    IMU); that was not measured.

*Display.* Inside the tunnel the selected filter. From the first GNSS fix of any kind after the
outage, a follower that moves with the selected filter's speed and closes the gap to its re-locked
ŝ at ≤ 2 m/s: exit step ≤ 1.2 m p95 on nominal runs. Without it, the smaller filters' re-lock on
the first Doppler fix moved ŝ by up to 25 m. The branch decision shown comes from the selected
filter: the ESKF's, or since v0.5.12 the 10-state filter's own (§4.13). On low-end S4 the shown
decision is right in 98–100 % of runs per set; the ESKF's, shown until v0.5.11, in 2–28 %.

*Disagreement monitor* (does not switch filters):
- σ_disp² = σ_i² + max(0, agg_j[(ŝ_i − ŝ_j)² − σ_i² − σ_j²]), alarm while agg_j d_ij > 9.
- "**any**" (agg = max) is captive to the worst filter. On Note10 S4 it raised an alarm in 76 % of
  runs, because of the 1D filter's error, and made 100 % of low-end callouts > 5 s early.
- "**majority**" (agg = min, the default) needs the selected filter to disagree with both others.

*Class rule with the majority monitor, six sets (0–299), v0.5.13* (estimated mount, §4.14), app's
along-track error; time to 5 m is the median per set from the first fix after the outage:

| | diverge rms | portal rms | (e/σ)² worst set, without / with monitor | > 1 s late | > 5 s early | runs with an alarm | to 5 m [s] |
|---|---|---|---|---|---|---|---|
| Note10, S1–S6, 5 km | 4.2–48.6 m | 4.2–17.1 m | ≤ 0.41 / ≤ 0.39 | ≤ 12 % (S5) | ≤ 8 % (S4) | ≤ 10 % | 12–19 |
| low-end, S1, S2, S3, S5, S6, 5 km | 10.9–30.8 m | 7.9–22.8 m | ≤ 0.53 / ≤ 0.52 | ≤ 4 % | 0 % | 0 % | 17–28 |
| low-end S4 | 54–126 m | 16–21 m in five sets, 225 m in one | 25 / 0.60 | ≤ 2 % | 56–76 % (mean 6–7 s early) | 28–52 % | 22–27 |
| low-end S4, v0.5.12 | 54–139 m | 20–24 m in five sets, 225 m in one | 24 / 0.70 | ≤ 2 % | 60–74 % | 26–46 % | – |
| low-end S4, v0.5.11 (0–299) | 40–229 m | 89–335 m | 59 / 12.9 | 6 % | ≤ 86 % | 36 % | – |
| pressure 30 Pa/km, S1 / S3 | 105–154 m | 96–106 m | 13.4 / 2.1, 8.5 / 1.06 | 0 % | 4 % / 100 % | 28 % / 72 % | 49–52 / 36–38 |

- **Against v0.5.12** the rows moved by the estimated mount and the 10-state filter's new rows
  (§4.14): low-end S4 portal 20.4–23.9 → 16.1–20.5 m in the five good sets, low-end S2 9.9–11.6 →
  9.0–9.7 m, Note10 S4 4.7–6.1 → 4.5–7.8 m; the other rows within ±1 m. The set at 225 m is seed 232
  again (§4.14).
- **Prompts** (`prompts` in the JSON, §4.14): on low-end S4 every 200 m and "now" prompt is shown
  without a distance (2σ > 100 m), and "now" comes a mean 121–145 m early; on the Note10 the vague
  share is ≤ 58 % (S4 200 m prompt) and "now" is missed in ≤ 2 % of runs.
- **Exit step** ≤ 1.1 m p95 on the Note10, ≤ 0.34 m on low-end phones, 7.0 m with the pressure
  gradient on S3.

On low-end S4 the "err early" rule has a measured price: two callouts in three come more than 5 s
early. At crawling speed 6 s is ~15 m, so in practice harmless, but it is what a σ that covers the
error costs there.

The straight-on control was run at filter level (ESKF `eval_sets`, 10-state `ekf10_sets`), not
through the supervisor. The selection is by IMU class and each filter's branch decision is its
own, so no difference is expected; it was not measured.

*Validation, seeds 400–499* (`reports/0.5.13/supervisor_validation.json`, run once): the same
picture. (The v0.5.12 validation on 300–399 is in `reports/0.5.12/`.)
- Note10: portal 5.4–16.1 m, (e/σ)² ≤ 0.39.
- Low-end except S4: 7.8–21.9 m, (e/σ)² ≤ 0.48.
- Low-end S4: 14.4 / 20.0 m, (e/σ)² 0.48 → 0.24 and 1.21 → 0.35 with the monitor (the two sets);
  callouts more than 5 s early in 62 % and 72 %.
- Pressure gradient: (e/σ)² 7.6–12.1 → 1.03–2.03.
- Exit step ≤ 1.0 m except the pressure gradient on S3 (7.3 m). Time to 5 m: median 13–29 s per set
  (pressure 38–53 s).

Readings:
- **Low-end S4 in the product configuration** went from 89–335 m (v0.5.11) to 19–24 m in seven of
  eight sets (v0.5.12, §4.13: the curve landmark, the 10-state filter's own branch decision and the
  one-way speed constraint) and to 14–21 m in seven of eight in v0.5.13 (§4.14). The set at 225 m
  holds seed 232, now partly explained (§4.14).
- **Why the monitor works where it does: the filters are not independent.**

  | shared input | 1D | 10-state | ESKF |
  |---|---|---|---|
  | IMU, stop detector, GNSS | yes | yes | yes |
  | barometer | – | yes | yes |
  | map heading along the route | – (curvature only) | yes | yes |
  | map grade | yes | – | – |

  "Majority" is in practice two correlated filters plus one. The monitor catches failures where the
  models differ:
  - a pressure gradient (the 1D filter ignores the barometer);
  - suspension pitch (the 1D filter alone);
  - one filter's map-layer failure.

  It cannot catch failures in a shared part: a false stop, an IMU scale error. It is a consistency
  warning, not a vote of three independent sources and not an accuracy guarantee: raising σ_disp
  does not move the estimate. When σ stays large, telling the driver so ("position uncertain") is
  a product decision this work did not make.
- **The pressure gradient shows the limit.** For the first two minutes the ESKF and the 10-state
  filter drift together (+100 m at t+120 s) and the monitor is silent. It fires only when they
  split, the ESKF taking the wrong branch (−120 m) and the 10-state continuing (+200 m). The
  monitor then makes σ honest ((e/σ)² ~1–2), late.

**4.13 v0.5.12: a branch window that grows with σ_s, a curve landmark and a branch decision for
the 10-state filter, and a one-way speed constraint** (owner 2026-10-25/26; `EskfConfig.map_pre_sigma_k`,
`no_reverse*`; `Ekf10Config.curve_landmark`, `no_reverse*`; `tools/eval_sets.py`,
`tools/ekf10_sets.py`; `reports/0.5.12/`). Every number below is over seeds 0–299 (six sets of 50)
unless it says otherwise. 20-seed runs were a first look only: on low-end S4 they showed 35 → 0
failures where 300 seeds gave 35 → 6, the third time a small sample was optimistic, so the method
is now "20 seeds to look, 300 for the number".

*Branch window* (`eval_sets_300_win2.json`, `eval_sets_300_win0_*.json`; measured before the speed
constraint was added). The decision window
starts max(100 m, 2σ_s) before the diverge instead of 100 m.
- **S4 seed 247** (§3.1): −110.1 → −3.4 m, the branch decided correctly. S4 200–249 rms 16.7 →
  6.1 m; runs > 100 m on the branch route 1 → 0 in 300.
- **Everything else:** other S4 sets within 0.1 m, S1-5km 0–49 12.78 → 12.94 m, all other
  scenarios bit-identical.
- **Straight-on control, unchanged with the window** (k = 0 and k = 2 give the same table):

  | | S1 straight on | S4 straight on |
  |---|---|---|
  | runs > 100 m off at the portal (of 300) | 0 | 38 (3, 7, 8, 10, 3, 7 per set) |
  | "not yet" after the diverge, mean / p95 | 5.1 / 6.1 s | 12.9 / 20.5 s |
  | while "not yet": the shown point's distance from the road the car is on, largest per run, mean / p95 | 11.6 / 14.2 m | 38 / 58 m |
  | runs where it exceeds 10 m | 235 | 300 |
  | undecided at the portal | 0 | 1 (the same run as v0.5.11) |

  While "not yet", the screen projects the estimate onto the route polyline, i.e. the branch. A
  car that goes straight on has left the route and the app will re-route; 13 s at 38 m is the
  measured cost of that state, recorded and not engineered away. A car that takes the branch is
  "not yet" for 2.8 s (S1) and 3.7 s (S4), with the shown point ≤ 0.5 m from the road.

  The table is the ESKF's (Note10). **On a low-end phone the cost is larger.** The 10-state
  filter's own decision (below) stays "not yet" through the whole tunnel in 299 of 300
  straight-on S4 runs, so the screen draws the car on the branch until GNSS returns, and the
  s-guard lets s run free meanwhile: at the portal the shown point is 302–439 m rms from the car
  per set (p95 ~580 m; validation 342 / 390 m), against 193–381 m in v0.5.11. The landmark never
  fires there. The costs are asymmetric: a car that takes the branch gets its decision in seconds,
  one that goes straight on a low-end phone never does. Showing the car on the main tunnel instead
  while "not yet" would trade them; not measured (§5).

*The 10-state filter's open failure, low-end stop-and-go.* Two attempts that did not fire, then
what worked.
- **Re-association as in the ESKF (yaw ψ̂ against ψ_map(s_c)) never fires** (ΔJ ≤ 0.4 against
  10.83). The ESKF's re-association relies on ψ̂; on a low-end gyro the 10-state yaw is pulled to
  the map within a second by the map-heading row, so the information is in the change of heading,
  not in the heading.
- **Matching the yaw rate (w̄ against v·κ(s_c)) never fires either.** That is an artefact of the
  profile, not physics: the low-end gyro_noise of 0.063 rad/√s folds the τ ≤ 50 s bias terms into
  white noise (v0.5.7). That is right for 300 s of consistency and wrong for a 10–15 s curve, over
  which the bias is nearly constant. With the folded value in R (0.064 rad/s for a 1 s mean) the
  curve rate (~0.06 rad/s) was buried. A gyro-only heading drifting ~90° in 300 s does not matter
  either: a curve needs 12 s, not 300.
- **The actual mechanism.** With σ_s larger than the curve, the linearised heading and
  curvature-gyro rows lock s to a wrong point: σ_s 350 → 20 m at −160 m error. Either row alone
  does it.
- **(a) the curve s-guard:** while σ_s exceeds the route's shortest clothoid, those rows do not
  correct s (H_s = 0), as in the ESKF's branch deferral.
- **(b) the curve landmark (owner's form):** the yaw change over the last 12 s,
  Δψ_g(t_j) = ∫(w_z − b̂_gz)dt/(1 + ŝ_g), against ψ_map(c − d_T + d_j) − ψ_map(c − d_T) for
  candidates c in ŝ ± 3σ_s (d_j = ∫v̂ dt); full covariance
  C = ARW²·min(t_i, t_j) + Σ_GM(τ ≤ 50 s) + P_bgz·t_i t_j + diag(sideslip k·v·|w|, map heading)².
  The gyro terms are the sensor's [MEAS] short-term ones (worst axis: ARW 0.00625 rad/√s, GM
  τ = 5 s σ 0.0066 rad/s, τ = 50 s σ 0.0056 rad/s); over 12 s that is ~5°, more conservative than
  the owner's ARW-only 2°, because the τ ≤ 50 s terms are not constant over 12 s. It fires on a
  turn of more than 5° and, if the profile fits (χ²ₙ 99.9 %) and beats ŝ by χ²₁, updates s with
  R from the curvature of J (≥ 5 m), at most once per 12 s. (a) and (b) are one switch: (a)
  alone is consistent and worse (S1-5km Note10: 39 runs > 100 m in 300).
- **Its own branch decision (owner's form (2)).** The same profile against the same candidates,
  once with the branch geometry (J_kol) and once with the main tunnel (J_ana): min J_ana − min
  J_kol > χ²₁ → "branch" and the landmark runs; the reverse → "main", the map rows switch to the
  main tunnel and the landmark stays off; otherwise "not yet". It is the batch form of the ESKF's
  r_b / r_m test. A first form gated the landmark on the ESKF's decision instead; on a low-end
  phone the ESKF is the broken filter (~900 m on S4), its "branch" came late or never, only (a)
  was left, and the product's low-end S4 got worse (178–376 m against 89–335 m;
  `supervisor_sets_*_eskfgate.json`). A gate on "a turn was seen" alone (form (1)) assumes a straight
  main tunnel after the diverge, which most Istanbul tunnels are not; with it the landmark fired 176
  times in 300 straight-on S4 runs, because the 12 s gyro noise (~5°) is the size of the 5° threshold.
- **One-way speed constraint** (owner, [ASSUMED] "a car does not reverse in a one-way tunnel"):
  while v̂ < 0 in an outage, z = 0 − v̂. The owner's form, R = zupt_sigma² = (0.02 m/s)², made
  things worse (below), because v̂ wobbles around zero at stops and the tight one-sided update
  biases it upward; the default is R = (0.5 m/s)² [ASSUMED], in both filters. It is a limit, not a
  cure: the crawl pitch error that drives v̂ negative stays (§4.10, §5).

Low-end S4, 10-state filter alone (route given), 300 seeds (`ekf10_sets_300_v2.json`):

| | rms | rms without runs > 100 m | runs > 100 m | p95 | (e/σ)² |
|---|---|---|---|---|---|
| v0.5.11 | 224 m | 29.1 m | 35 | 354 m | 26.1 |
| (a) only (v1 run) | 307 m | 44.1 m | 84 | 585 m | 8.0 |
| (a) + (b), gate (1): a turn seen | 167 m | 21.8 m | 5 | 50 m | 8.4 |
| (a) + (b), gate (2): own decision | 167 m | 21.9 m | 5 | 50 m | 8.5 |
| (2) + no reverse, R = (0.02 m/s)² | 187 m | 25.7 m | 12 | 60 m | 10.3 |
| **(2) + no reverse, R = (0.5 m/s)² (default)** | **94 m** | **22.0 m** | **2** | **47 m** | **5.1** |

- **(1) and (2) on a car that takes the branch:** 260 of 300 low-end S4 runs identical, the largest
  difference 23.8 m, the same failures; Note10 298 of 300. The decision comes a few seconds after the
  turn passes 5°, so (2) sometimes misses one landmark.
- **Straight on:** (2) never fires the landmark (low-end S1 and S4, Note10). It decides "main" in 300
  of 300 on S1 and in 1 of 300 on low-end S4, where σ_s covers the straight before the diverge and
  the two hypotheses look the same ("not yet").
- **Seed 219** (a wrong lock with (1): σ 132 → 17 m at 83 m error) is 31 m with (2). A χ² margin
  between the best and the best distant candidate was tried and made 20 seeds worse (0 → 3 runs >
  100 m); it stays off (`landmark_margin`).
- **Other scenarios:** Note10 S4 25.5 → 16.5 m (3 → 1 runs > 100 m), Note10 5 km 22.3 → 9.4 m
  (2 → 0), Note10 S5 5.5 → 3.2 m; low-end 5 km 16.0 → 18.0 m; the rest within 1 m. Note10 S1 is
  5.5 m with (1) and 5.7 m with (2): the gate also keeps the landmark off on curves before the
  diverge, a small cost of the rule.
- **The no-reverse constraint, the two forms:** on low-end S4 the tight form gives 12 runs > 100 m,
  the soft one 2; on Note10 S4 the tight form gives 13.8 m and the soft one 16.5 m. In the ESKF the
  tight form fired 1437 times on 10 Note10 S4 runs and moved the portal from 5.0 to 6.5 m; the soft
  one leaves 5.0 m. The ESKF on low-end S4 stays ~900 m with either: its failure is not reversing.
- **What remains on low-end S4:** seed 232 (−1649 → −1566 m) and seed 200, which the constraint
  makes worse (66 → 229 m; v0.5.11 186 m). The constraint turns seeds 113 (−2063 m), 125 (−516 m)
  and 255 (−996 m) into runs under 100 m, more than "a limit" promised; why the crawl goes negative
  is still the pitch prior's question (§4.10).
- **The same landmark could serve the ESKF;** it was not needed there, because the Note10 gyro keeps
  ψ̂ informative.

*R floor k·v² for the re-association* (`eval_sets_300_win2-slip_*.json`, off): S1, S4 and 5 km, 18
sets: every per-seed error identical. At scenario speeds k·v² ≤ 3.4 m stays under the 5 m floor.

**4.14 v0.5.13: the mount estimated in open sky, a lateral-force speed row, recovery and prompt
metrics** (owner 2026-10-29/30, from a comparison with another case-study repository;
`tunnelsim/mount_init.py`, `magnetometer.py`, `navmetrics.py`; `EskfConfig.mount_yaw_sigma`;
`Ekf10Config.lateral_speed`, `mount_yaw_sigma`; `tools/eval_sets.py` variants `known`, `cup`,
`noexc`, `cup-known`, `noexc-known`; `reports/0.5.13/`). Every number is over seeds 0–299 unless
it says otherwise.

*Mount alignment (owner's design).* Until v0.5.12 every filter was given the true mount C_bp
[ASSUMED]. Now it is estimated in the first 90 s of open sky [ASSUMED] from the phone's own data:
1. **Tilt from gravity** ("complementary init"): where the GNSS acceleration is under 0.1 m/s², the
   gyro is quiet and v > 5 m/s, the 1 s mean specific force is gravity in the phone frame; the car's
   own pitch there is the road grade from GNSS.
2. **Yaw first from the magnetometer:** the phone attitude from gravity and the field (TRIAD), against
   the car attitude from the GNSS course and grade, averaged over those epochs. The magnetometer is
   new to the model and read only in open sky: Earth field 47 µT, inclination 56°, declination 6°
   (Istanbul), the car's own field 3 µT fixed in the body, soft iron 2 %, noise 0.4 µT at 25 Hz, all
   [ASSUMED] (`magnetometer.py`). Inside the tunnel it is never read (DESIGN §2).
3. **Then yaw from the velocity direction:** where the car accelerates or turns, the change of the
   specific force against the GNSS acceleration in the phone frame, through C_bp(δ) = R_z(δ) C_bp; the
   best δ on a 0.25° grid replaces the magnetometer's yaw once Σ|a_h|² ≥ 3 (m/s²)² [ASSUMED]. The
   difference against the still-epoch mean removes the accelerometer bias.

The remaining yaw error enters both filters as a state (owner's option (a) for the 10-state
filter): ψ_m in the lateral NHC row (H = v_bx) and the map-heading row (H = −1) of the ESKF, and in
the 10-state propagation (a = f_x − ψ_m f_y − …) and lateral row. Its prior is 6° when only the
magnetometer set the yaw (the measured rms below) and 1° after step 3 [ASSUMED].

| mount, S1 / S4 (300 seeds each) | error rms per set, roll / pitch / yaw | yaw from | S1 diverge / portal | S1 (e/σ)² | S4 diverge / portal | S4 (e/σ)² | S4 branch |
|---|---|---|---|---|---|---|---|
| true (v0.5.12) | – | – | 11.4–16.9 / 5.5–6.9 m | 0.20 | 34.9–50.3 / 4.7–6.1 m | 0.13 | 300/300 |
| estimated (default) | ≤ 0.6 / 0.7 / 0.7° | mag+accel 600/600 | 11.5–16.9 / 5.6–7.1 m | 0.22 | 35.0–48.4 / 4.5–7.9 m | 0.14 | 299/300 |
| cup holder (10°, −20°, 25°), estimated | ≤ 0.7 / 0.7 / 0.7° | mag+accel 600/600 | 10.6–16.1 / 6.0–7.7 m | 0.25 | 35.2–54.4 / 4.5–7.0 m | 0.14 | 298/300 |
| cup holder, true | – | – | 10.6–16.0 / 5.5–7.3 m | 0.23 | 35.0–61.9 / 4.6–6.0 m | 0.13 | 299/300 |
| no approach manoeuvre, estimated | ≤ 0.6 / 0.7 / 7.4° (5.8° overall) | mag 600/600 | 11.3–16.9 / **16.6–25.4 m** | **1.69** | 36.3–47.8 / 6.1–8.2 m | 0.22 | 300/300 |
| no approach manoeuvre, true | – | – | 11.2–17.1 / 5.1–6.9 m | 0.21 | 36.2–48.9 / 5.0–6.6 m | 0.15 | 300/300 |

- **With an approach manoeuvre the estimate costs nothing measurable** on S1 and S4: within the
  set-to-set spread on every row, and no run over 100 m. Against v0.5.12 the other scenarios move
  by ≤ 0.7 m, except the S-bend S2 diverge error, 3.5–4.6 → 4.2–5.2 m; the A/B with the true mount
  was not run on S2, so the residual mount yaw in its bends is the likely but unmeasured reason. The one S4 branch miss is seed 271, decided
  "main" (8.3 → 41.1 m at the portal).
- **Without a manoeuvre the yaw rests on the magnetometer (5.8° rms)** and the S1 portal is 3–4 times
  worse and inconsistent ((e/σ)² 1.69 in the worst set): open. The diverge error does not change;
  the portal, after the exit curve, does. That fits a yaw error acting through the lateral NHC row in
  the curve; not traced yet. On 10 seeds the mount-yaw state had given 0.89; the 300
  seeds give 1.69, the fourth optimistic small sample (§6.3).

*The 10-state filter with the estimated mount* (`ekf10_sets_300_v3*.json`), portal rms / without
runs > 100 m, failures, (e/σ)²:

| | low-end S4 | Note10 S2 | Note10 S1 | low-end 5 km |
|---|---|---|---|---|
| v0.5.12 (true mount, no lateral row) | 94 / 22 m, 2, 5.1 | 3.5 m, 1.13 | 5.6 m | 16.0 m |
| + lateral row, true mount | 27.9 / 19.9 m, 1, 1.15 | 3.5 m, 1.13 | 5.6 m | 16.2 m |
| + estimated mount, no mount-yaw state | 93.3 / 19.6 m, 1, 5.08 | 6.0 m, 3.29 | 6.2 m, 1.05 | 18.0 m |
| **+ mount-yaw state (v0.5.13)** | **93.3 / 19.6 m, 1, 5.03** | **4.4 m, 0.98** | **6.1 m, 0.70** | **18.0 m** |

- **Lateral-force speed row (owner's item 4):** in a turn the phone's lateral specific force is
  v·ω, so z = f̄_y − b̂_y − (v(ω̄ − b_gz) − ψ_m f̄_x) observes the speed where the low-end phone loses
  it. b̂_y is a 30 s mean of f_y over straight driving. First form, a fixed |ω| > 0.02 rad/s
  threshold: on low-end S4, 710 m with 12 failures in 20 seeds, because gyro noise peaks were taken
  as turns. Final form: |ω| > max(0.02, 4σ_ω,1s), with σ_ω,1s from the gyro's [MEAS] short-term
  terms, and (v·σ_ω,1s)² in R. With the true mount it fixed seed 200 (229 m) and moved seed 232
  from −1566 to −340 m.
- **The estimated mount undid that on seed 232** (−1581 m): a low-end tilt error of ~2° leaks
  gravity into the speed during the crawl. Every other low-end S4 run stays under 100 m.
- **Without a mount-yaw state** the Note10 S2 was 6.0 m with (e/σ)² 3.3: a yaw error the filter did
  not know about. The state brings it to 4.4 m and 0.98.

*Seed 232, the mechanism as far as it is understood* (low-end S4). After the crawl the speed drifts
low (16.7 → 8.9 m/s, the pitch gravity leak of §4.6) and σ_s grows honestly to 713 m. In the curve
the error is pulled from −561 to −273 m while σ_s collapses to 67 m: a lock to a wrong point at ~4σ,
the failure the s-guard of §4.13 is there to prevent. Why the guard let it through is not yet known.

*Recovery and prompt metrics (owner's items 1 and 2;* `navmetrics.py`). Recovery: from the first
fix of any kind after the outage, the time until the shown along-track error is under 10 m and 5 m
for the rest of a 60 s window, and the largest error shown meanwhile. Prompts at 1000, 500 and 200 m
and "now" (D = max(30 m, 2 s·v̂) [ASSUMED]), each when ŝ + kσ ≥ s_gore − D, as the callout: the
true distance minus D when it fires, late by more than 1 s, "now" missed (fired after the gore or
never), and vague (2σ > 100 m [ASSUMED]: worded "Exit A ahead, keep right" without a distance).
Numbers in §3 and §4.12. The ESKF gets under 5 m in a median of 10–18 s; 12–34 runs of 300 per
scenario do not within 60 s. Through the supervisor's follower a low-end phone takes 17–28 s, and the
pressure gradient 36–52 s.

*Simulator (owner's item 6).* The page state is in the address (`#tab=…&t=…&show=…&nav=…&k=…`), so a
view can be shared as a link. The callout wording follows σ: without a distance when 2σ > 100 m. The
navigation view draws the other filters and the constant-speed baseline as dots, by the same
checkboxes as the map.

*Not taken:* BLE beacons in the tunnel. The owner rejected the assumption that a tunnel has them.

## 5. Limitations and next steps

- **Simulation only.** The IMU noise terms are measured. The time shape of the noise, vibration,
  sideslip, barometer, map and GNSS behaviour are [ASSUMED].
- **Simplified map errors.** Map errors are a lateral offset and a length scale. Shape errors,
  missing curves and a wrong diverge position are not simulated. The truth car drives on the lane
  centre.
- **The stop detector is causal since v0.5.10, and deliberately conservative.** The
  forward-backward (offline) version used before saw a little of the future. One pass of the same
  filters rejects the ~25 Hz engine tone less, so idle and crawling vibration stay slightly above
  the band threshold. The detector then catches 80 % of the stop time instead of 96 %, and never
  holds "stopped" for more than 0.5 s while the car moves. Filter order A/B on S4
  (`reports/0.5.10/zupt_order_ab.json`; 50 seeds for the detector, 30 for the ESKF):

  | Causal band-pass order | Stop time caught | Seeds with > 0.5 s false "stopped" | ESKF diverge / portal, (e/σ)², > 1 s late |
  |---|---|---|---|
  | 2 | 2 % | 0 | 192 / 334 m, 31, 13 % |
  | 3 | 24 % | 0 | 30.4 / 4.4 m, 0.07, 0 % |
  | **4 (default)** | **80 %** | **0** | **33.7 / 4.7 m, 0.08, 0 %** |
  | 8, the offline rejection | 95 % | 3 | 51.9 / 13.3 m, 0.64, 7 % |
  | offline (reference) | 96 % | 0 | 30.3 / 4.8 m, 0.08, 0 % |

  - **The asymmetry:** a missed stop costs little, because the ZUPT still fires for most of it;
    a false stop at 3 m/s cost hundreds of metres.
  - **Why not order 3:** it scores slightly better, but it sits next to the cliff at order 2.
  - **What it still rests on:** the margin comes from the [ASSUMED] vibration and engine model,
    so the thresholds must be re-set on recordings.
- **Narrow setting.** Two IMU classes, a rigid phone mount (estimated since v0.5.13, §4.14), a
  single lane, one car. A second car
  would only be another noise realisation of the same sensors; the 50 seeds cover that.
- **Next, with real devices:**
  - static Allan runs to check §3 of `DESIGN.md`;
  - recordings in Istanbul tunnels, with the portals as checkpoints;
  - measuring the map's polyline error and the tunnels' pressure gradient;
  - the IMU class from the sensor name, and a low-end profile from a real phone's Allan run.
- **Next, in the filter, for low-end IMUs** (not measured):
  - the barometer at 25 Hz with its own correlation model instead of the 1 s mean, to hold pitch
    more often;
  - the gyro bias as a sum of Gauss–Markov terms (τ 5/50/500 s, as in the truth model) instead of
    a random walk;
  - the map-constrained filters of §4.9, selected by the supervisor of §4.12 (done: 16–21 m on
    low-end S4 in five of six sets in v0.5.13), or the pitch prior of §4.10 for the ESKF;
  - Android's calibrated gyro stream, which removes the turn-on bias at stops but not the drift in
    the tunnel.
- **A smaller filter** (built as a prototype, §4.9). The 10-state filter matches the ESKF at the
  diverge on a mid-range phone and is the better filter on a low-end one. Since v0.5.12 it has its
  own branch decision (§4.13). Before it could replace the ESKF it still needs:
  - a capped re-acquisition at the exit (8–39 m steps; the supervisor's follower covers it today);
  - consistency in stop-and-go ((e/σ)² 5.1 on low-end S4, 4.8 on Note10 S4 in validation);
  - the map-error stress of §4.4.
- **Low-end stop-and-go, what is left** (§4.13, §4.14): 1 run of 300 more than 100 m off in the
  10-state filter, seed 232 (−1581 m). Seed 200 was fixed by the lateral row. The trace shows the
  crawl speed loss and then a wrong lock at ~4σ in the curve; why the s-guard let it through is
  open, and the estimated mount's tilt error (~2° on that phone) makes it worse. "Low-end works"
  is not claimed until it is understood.
- **The mount without an approach manoeuvre** (§4.14): the yaw rests on the magnetometer (5.8° rms)
  and the S1 portal is 16.6–25.4 m with (e/σ)² 1.69, inconsistent. A larger mount-yaw prior, or
  holding the NHC rows until a turn has refined the yaw, are the next things to try; not measured.
- **The magnetometer model is [ASSUMED]** (field, the car's own field, soft iron, noise). A real
  car's field at a dashboard mount can be several times larger; recordings would set it.
- **Low-end, a car that goes straight on** (§4.13): the 10-state decision never leaves "not yet"
  and the car is drawn on the branch through the tunnel, 302–439 m off at the portal. Next step,
  not measured: while "not yet", show the car on the main tunnel instead of the route.
- **Recovery is now measured as time** (§4.14), not only as the largest step. The follower closes
  a gap at ≤ 2 m/s, so the pressure gradient's 100 m takes ~50 s to show right. Whether 10–28 s to
  5 m is acceptable for guidance after the exit is a product question.
- **Coverage, not only the mean of (e/σ)².** (e/σ)² is the along-track scalar, not the full-state
  NEES, and a small mean does not show that rare large errors are covered. At the portal 100 % of
  Note10 runs are within 2σ (20 seeds); P(|e| ≤ σ) and P(|e| ≤ 2σ) over time before the diverge,
  and separately for the nominal, 2× map error and pressure-gradient cases and for the
  supervisor's σ_disp, are the next check.
- **The one-way assumption** (§4.13): a car does not reverse in a one-way tunnel [ASSUMED]. It is
  weaker than the hard map constraint, but a two-way tunnel or a car backing out of a jam breaks it.
- **The main tunnel after the diverge is straight in every scenario** and straight in the map object.
  The 10-state filter's branch test compares against the main tunnel's geometry, so a curved main
  tunnel is handled by design, but it needs the main polyline in the map, and it was not simulated
  or measured. Most Istanbul tunnels curve after a branch.
- **The 12 s gyro noise on a low-end phone (~5°) is the size of the 5° turn threshold.** The branch
  gate keeps false turns from moving s; on its own the threshold would not.
- **The validation tier is one set of 100 seeds per version** (§3.1, 400–499 for v0.5.13); claims on
  rarer failures (1 in 300 in §4.14) need more.
- **A car that stays in the main tunnel has no curve to correct it.** On S4, 11 % of such runs are
  more than 100 m off at the portal (§3). Only GNSS at the exit, through the follower, fixes it;
  a map with features in the main tunnel (or a curve) would be needed.
- **The branch decision's window** grows with σ_s since v0.5.12 (max(100 m, 2σ_s), §4.13). The
  measured cost is in the straight-on control: "not yet" for 13 s on S4, with the car shown on the
  branch up to 38 m from its road, until the app re-routes.
- **Next, for the pitch prior** (§4.10):
  - learn the offset at the portal, where pitch is still known from GNSS, instead of restarting it
    with σ 1°;
  - the map re-association of §4.11, now in the ESKF, removes the exit-curve collapse that made
    the prior inconsistent; the A/B should be repeated with it.

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
| Scope and rules | Set the assumptions: fixed mount; no BLE, OBD or camera, and no magnetometer in the tunnel (open sky only since v0.5.13); map available; Suvorkin IMU profiles. Set the working rules: tags, truth/estimator split, consistency as acceptance, robust over optimal, a snapshot per version | DESIGN §2, §12 |
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
| Baseline (§4.9) | Asked for a 1D map-constrained filter as a measured reference against the ESKF. A comparison drafted with another AI model gave the design and the questions; the agent re-implemented it from that description and re-measured every number | CHANGELOG v0.5.9 |
| Filter size (§4.9) | Read the 4-vs-16 comparison as "the 16 states buy independence from assumptions, not accuracy" and asked for the middle size. The 10-state filter was built in a separate AI session under his review (8 → 10 states, each addition from a trace); the agent brought it into the repository and re-measured it with 50 seeds, the exit step and the straight-on control | CHANGELOG v0.5.11 |
| Stop detector (§5) | Chose causal and the conservative filter order 4 after the order A/B (order 3 sits next to a cliff) | CHANGELOG v0.5.10 |
| Seed discipline (§3, §4.11) | Read the 200-seed result as a selection effect, not luck: every tuning was done on 0–49. Asked for failure counts next to rms, for the most-worsened seed, and for an untouched validation tier (200–299) run once | CHANGELOG v0.5.11 |
| Map re-association (§4.11) | Read seed 63's mechanism, predicted that the NIS gate would also reject useful innovations (it did), and proposed re-association along the arc. Required the failure cases (wrong branch, map error 2×) before it became the default | CHANGELOG v0.5.11 |
| Pitch prior (§4.10) | Proposed the road-grade prior for low-end phones. Diagnosed the ZUPT speed gate as circular and moved its source to the detector's own confidence. Proposed the offset state, which the measurement then rejected | CHANGELOG v0.5.11 |
| Supervisor (§4.12) | Proposed a layer that selects among the three filters, and deciding before the tunnel while GNSS is good. Approved the class rule and the open-sky rehearsals | CHANGELOG v0.5.11 |
| Branch window (§4.13) | Proposed the σ-scaled window from seed 247. Asked for the "not yet" cost on the straight-on control and decided to record it rather than engineer it away (the app re-routes). Demoted 200–299 to a design tier and set 300–399 as the validation tier | CHANGELOG v0.5.12 |
| 10-state landmark (§4.13) | **Corrected the agent's reading** that the yaw rate is buried in gyro noise: a profile artefact (the folded 0.063 rad/√s), not physics. Designed the curve landmark on the yaw change over 10–15 s and the s-guard before it, as one switch | CHANGELOG v0.5.12 |
| 10-state branch decision (§4.13) | **Rejected the agent's proposal** to gate the landmark on "a turn was seen": it assumes a straight main tunnel, which Istanbul tunnels are not. Designed the batch J_kol / J_ana test instead | CHANGELOG v0.5.12 |
| Speed constraint (§4.13) | Proposed "a car does not reverse in a one-way tunnel" as an assumption, with the expectation "a limit, not a cure". Chose the soft form after the measurement showed his tight form harmful | CHANGELOG v0.5.12 |
| Method (§3) | Read the third optimistic small-sample result as a method: 20 seeds to look, 300 for the number | CHANGELOG v0.5.12 |
| Comparison (§4.14) | Compared the repository with another case study and chose what to take: recovery and prompt metrics, the mount init, the lateral-force row, the permalink and σ-dependent wording. **Rejected BLE beacons**: a tunnel with beacons is not a fair assumption | CHANGELOG v0.5.13 |
| Mount init (§4.14) | Designed it: tilt from the gravity mean ("complementary init"), yaw first from the magnetometer, then from the velocity and acceleration direction; noted that heading needs the car's direction, so the phone-to-car difference is the mount, and proposed the acceleration direction once the car moves | CHANGELOG v0.5.13 |
| Mount-yaw state (§4.14) | Chose option (a), a mount-yaw state in the 10-state filter as well, after the measurement showed the estimated mount inconsistent there (Note10 S2 (e/σ)² 3.3) | CHANGELOG v0.5.13 |
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
| Headline results used the offline (forward-backward) stop detector; the 0.5.3 results file recorded k = 1.65 in its main callout block while the report quoted its k = 1 row | an external AI comparison; measured, switched and re-run at v0.5.10 |
| The causal stop detector at twice the order (to restore the offline engine rejection) took a 3 m/s crawl as a stop in 3 of 50 seeds: S4 portal 91 m | the agent's 50-seed re-run; the order A/B and the choice of the conservative order were the author's |
| "Removing the ZUPT gate costs the Note10 6 m": a 30-seed number compared with a 5-seed one | the agent, on re-measuring the same seeds (the gate changes nothing on the Note10) |
| The display handed over to the ESKF after the exit, which on a low-end phone is the broken filter (253 m step) | the agent's first supervisor run |
| `MapView.nearest` failed when an estimate ran past the end of the map; the same in the 10-state and 1D route grids | the pitch-prior A/B; a supervisor run |
| "One callout in 500 more than 1 s late" (k = 1), measured on the development seeds 0–49; 11 in 2000 over 0–199 | the owner's request to re-run on new seeds (a selection effect) |
| "One straight-on run in 100 reported the branch" (§3.1 draft); the run was undecided, none reported the branch | review v8 |
| Straight-on S4 runs > 100 m off (28 of 200) left out of the report: the failure column counted the branch route only | review v8 |
| Re-association v1 ran before the branch decision and pulled a car going straight on back to the straight (92–170 m) | the straight-on negative control |
| Re-association v2 left the map's length error out of its R (S5 (e/σ)² 13 with a 2× map error); the first fix had the wrong sign and made it worse | the owner's failure-case request; the sign by comparison with the cross-track row |
| The supervisor display started its follower at the first position fix, after the smaller filters had already re-locked on Doppler (25 m step) | a trace requested by the owner |
| 1D baseline: re-lock after the tunnel started only on position fixes, so the gate locked out GNSS at the exit | the agent's event counts (230 rejections in 5 seeds) |
| 10-state re-association by yaw rate: R took the profile's folded 0.063 rad/√s, which buries the curve rate; "a gyro-only heading drifts 90° in 300 s" argued against a landmark that needs 12 s | the author |
| The 10-state landmark gated on the ESKF's branch decision: on a low-end phone the ESKF is the broken filter, its decision came late or never, and the product's low-end S4 got worse (178–376 m) | the agent's supervisor run over 0–299 |
| "Low-end S4: 35 → 0 failures" from 20 seeds; 35 → 6 on 300 | the 300-seed run; the method rule was the author's |
| The lateral-force row with a fixed 0.02 rad/s turn threshold took low-end gyro noise peaks as turns: 710 m, 12 failures in 20 seeds | the agent's 20-seed look before the 300-seed run |
| The no-excitation mount case looked consistent with the mount-yaw state on 10 seeds ((e/σ)² 0.89); 300 seeds give 1.69 | the 300-seed run |
| Two tests of the v0.5.11 mechanisms failed once the mount became estimated; they were testing with the true mount and are pinned to it | the test suite |
| A statement on the supervisor table that seed 232 fails because the speed goes negative before the constraint acts; the trace shows the speed held at zero while the car moves | the agent's trace before the report went out |

**Process controls,** used because generated code can be confidently wrong:
- parameter tags;
- a test that enforces the truth/estimator split;
- (e/σ)² as an acceptance criterion;
- a CHANGELOG entry and a metrics snapshot per version, with every changed metric explained;
- decisions and reviews kept as commits;
- release tags set by the author.
