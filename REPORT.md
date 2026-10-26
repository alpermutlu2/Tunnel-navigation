# Through the Tunnel: write-up

A two-page version is in [`SUMMARY.md`](SUMMARY.md).

Pointr case study. A phone in a car estimates its position through a GNSS outage in a road
tunnel. The navigation app has to keep showing the car, call out an exit inside the tunnel at the
right moment, and recover cleanly when the car comes out.

All results come from simulation. The main results are four independent sets of 50 seeds
(`tools/eval_sets.py`, `reports/0.5.11/`) plus a held-out validation set; §3 explains the three
seed tiers. Earlier sections quote 30–50 seeds where they say so. Every parameter is tagged [MEAS] (measured,
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

**Seeds, in three tiers.**
- **Development, 0–49:** every tuning in this repository was chosen on these.
- **Feature design, 50–199:** the map re-association of §4.11 was designed on seeds 63, 72, 103,
  113 and 114, and its first form's error was found on all of 0–199. These are no longer held out
  for that feature.
- **Validation, 200–299:** run once on the final configuration and not used to change anything
  (§3.1).

The ranges below are over four independent sets of 50 seeds (0–199), final configuration
(`reports/0.5.11/eval_sets_200_gate-reassoc.json`). The development set gave the low end of most
ranges: the S1 diverge error was 12.1 m on 0–49 and 13.0–16.9 m on the other three sets.

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | S1-1km | S1-5km |
|---|---|---|---|---|---|---|---|---|---|---|
| | 3 km straight | S-bend | V-profile | stop-and-go | braking | wander ±1 km/h | ±2 km/h | ±4 km/h | 1 km | 5 km |
| along-track at diverge, rms [m] | 12.1–16.9 | 3.5–4.6 | 12.1–15.2 | 34.7–50.2 | 25.0–31.2 | 13.4–17.6 | 13.3–18.3 | 13.2–18.1 | 4.5–5.0 | 24.6–31.6 |
| along-track at portal, rms [m] | 5.5–6.7 | 4.1–5.4 | 8.2–11.2 | 4.7–6.0 | 6.5–8.1 | 5.8–7.3 | 6.1–7.8 | 5.6–7.8 | 2.5–2.9 | 12.3–16.1 |
| portal p95, worst set [m] | 13 | 10 | 21 | 11 | 15 | 15 | 15 | 17 | 6 | 29 |
| runs > 100 m off at the portal (of 200) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| mean (e/σ)² at portal, worst set | 0.19 | 0.39 | 0.26 | 0.13 | 0.32 | 0.23 | 0.25 | 0.25 | 0.29 | 0.34 |
| callout > 1 s late, worst set: k = 0 / 1 / 1.3 / 1.65 | 12 / 0 / 0 / 0 % | 0 / 0 / 0 / 0 % | 10 / 0 / 0 / 0 % | 50 / 2 / 0 / 0 % | 56 / 10 / 0 / 0 % | 10 / 0 / 0 / 0 % | 14 / 0 / 0 / 0 % | 10 / 0 / 0 / 0 % | 0 / 0 / 0 / 0 % | 34 / 0 / 0 / 0 % |
| callout mean, k = 1 [s] (− early) | -0.6…-0.8 | -0.2…-0.3 | -0.8…-0.9 | -2.0…-2.4 | -0.4…-0.8 | -0.6…-0.8 | -0.6…-0.8 | -0.6…-0.9 | -0.1…-0.2 | -1.3…-1.6 |
| largest exit step p95, worst set [m] | 4.3 | 6.0 | 4.2 | 4.0 | 4.6 | 4.1 | 4.1 | 4.4 | 4.9 | 4.3 |

Cross-track error at the portal is 1.3–3 m (v0.5.10, 50 seeds), the map's own lateral uncertainty.
Branch decision: the branch is reported in 800 of 800 runs that take it. When the car goes
straight on, "main" is reported in 400 of 400 on 0–199 and in 199 of 200 in validation; one run
stayed undecided and none reported the branch.

- **Showing the car.**
  - Along the road, in a straight tunnel at steady speed: 12–18 m after 1.5–2 min, 25–50 m with
    braking or stops.
  - Across the road: 1.3–3 m, which is the map's own lateral uncertainty; the filter cannot do
    better than the map here. Without the map layer, the cross-track error was 26 m on S1 and
    106 m at 5 km, because heading is not observable in a straight tunnel.
  - The exit curve brings the portal error down to 2.5–16 m.
  - The filter is consistent: mean (e/σ)² ≤ 0.39 in every set.
- **Callout.** "The right moment" is read as *never late, and only as early as the uncertainty
  requires*: a late callout can make the driver miss the exit.
  - **Without the rule (k = 0):** 10–56 % of callouts come more than 1 s late (worst set).
  - **With k = 1:** 11 of 2000 callouts are more than 1 s late (10 on S5, 1 on S4), and on average
    they come 0.1–2.4 s early. The development set alone gave "1 in 500"; over 200 seeds S5 is at
    0–10 % per set.
  - **With k = 1.3 or 1.65:** none of 2000 are late on 0–199, but 3 of 100 on S5 in the
    validation tier (§3.1). The cost is 0.3–0.9 s more advance (S4: 2.2 → 3.0 → 4.0 s early on
    average).
  - **Why k = 1 stays:** σ_s is about twice the real error ((e/σ)² 0.1–0.4), so k = 1 is in effect
    ~2σ. The k = 1.3 row is the measured alternative; it halves the late callouts.
  - k has to be chosen again once σ is calibrated on real data. The voice says the estimated
    distance rounded to 50 m.
- **Branch.**
  - Car takes the branch: the in-loop decision reports it 2–3.5 s after the diverge.
  - Car goes straight on (negative control, map with the branch): "main" in 400 of 400. The
    portal error then stays at the no-map value: S1 18–28 m rms, S4 51–78 m rms, because there is
    no exit curve.
  - **On S4, 28 of 200 straight-on runs are more than 100 m off at the portal** (3, 7, 8 and 10
    per set; 10 of 100 in validation). One stop-and-go drive in seven stays in the main tunnel
    and is shown more than 100 m off when it comes out. This is the measured cost of "the map's
    along-track gain comes only from curves": with no curve before the exit, nothing corrects the
    dead reckoning until GNSS returns, and then the exit follower.
- **Recovery.** The largest step at the exit is ≤ 6 m (p95, worst set).

**3.1 Validation on untouched seeds** (200–299, final configuration, run once;
`reports/0.5.11/validation_200-299.json`). Nothing was changed after this run.

| | S1 | S2 | S3 | S4 | S5 | S6s | S6 | S6L | 1 km | 5 km |
|---|---|---|---|---|---|---|---|---|---|---|
| diverge rms, two sets [m] | 11.4 / 14.6 | 4.0 / 4.6 | 12.2 / 15.0 | 44.9 / 41.5 | 29.1 / 35.7 | 11.7 / 14.7 | 11.4 / 14.9 | 11.1 / 15.3 | 4.1 / 5.4 | 28.8 / 38.2 |
| portal rms, two sets [m] | 6.8 / 6.9 | 4.1 / 4.9 | 9.4 / 11.1 | 16.7 / 5.1 | 6.1 / 8.6 | 7.2 / 7.3 | 7.6 / 7.6 | 7.4 / 7.4 | 2.6 / 3.1 | 14.4 / 16.2 |
| runs > 100 m off (of 100) | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |
| worst (e/σ)² | 0.20 | 0.34 | 0.25 | 0.20 | 0.36 | 0.22 | 0.24 | 0.22 | 0.33 | 0.35 |
| > 1 s late of 100, k = 1 / 1.3 | 0 / 0 | 0 / 0 | 0 / 0 | 2 / 2 | 7 / 3 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 2 / 0 |

- **Inside the ranges of §3.** Most numbers fall there. The 5 km diverge (38 m in one set) is
  above them.
- **One new failure, S4 seed 247.**
  - The crawl leaves an unusually large dead-reckoning error: −220 m at the diverge region, with
    σ_s 93 m (2.4σ).
  - The car turns while ŝ is still more than 100 m before the diverge, outside the branch
    decision's window, so the branch is decided "main".
  - Re-association, which runs only on a decided branch, never starts. The portal error is
    −110 m with e/σ 2.0.
  - The same seed going straight on is 323 m off with "main" reported correctly.
  - Straight-on control in validation: "main" in 199 of 200, one run undecided, none reporting
    the branch. 10 of the 100 S4 straight-on runs are more than 100 m off at the portal (§3).
  - The structural limit: the along-track error can exceed the branch window. A window that grows
    with σ_s is the next step (§5).
- **k = 1.3 is not "never late".** It gave 0 in 2000 on 0–199, but 3 of 100 on S5 here (k = 1:
  7). The claim is "k = 1.3 halves the late callouts", not "removes them".

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

The filter's mount pitched 2° wrong changes no filter by more than 2 m (lumped bias). The 4-state
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
`tools/supervisor_eval.py`; `reports/0.5.11/supervisor_{dev,heldout}.json`,
`supervisor_sets_{0-99,100-199}.json`). All three filters run on the same phone data; the
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
ŝ at ≤ 2 m/s: exit step ≤ 1.1 m p95 on nominal runs. Without it, the smaller filters' re-lock on
the first Doppler fix moved ŝ by up to 25 m. The branch decision always comes from the ESKF.

*Disagreement monitor* (does not switch filters):
- σ_disp² = σ_i² + max(0, agg_j[(ŝ_i − ŝ_j)² − σ_i² − σ_j²]), alarm while agg_j d_ij > 9.
- "**any**" (agg = max) is captive to the worst filter. On Note10 S4 it raised an alarm in 76 % of
  runs, because of the 1D filter's error, and made 100 % of low-end callouts > 5 s early.
- "**majority**" (agg = min, the default) needs the selected filter to disagree with both others.

*Class rule with the majority monitor, four sets (0–199)*, app's along-track error:

| | diverge rms | portal rms | (e/σ)² worst set, without / with monitor | > 1 s late | runs with an alarm |
|---|---|---|---|---|---|
| Note10, S1–S6, 1 and 5 km | 3.5–31.8 m | 4.3–16.4 m | ≤ 0.43 / ≤ 0.38 | ≤ 10 % (S5) | ≤ 10 % |
| low-end, S1, S2, S3, S5, S6, 5 km | 10.9–25.1 m | 7.4–23.8 m | ≤ 0.80 / ≤ 0.80 | ≤ 2 % | 0 % |
| low-end S4 | 40–229 m | 89–335 m | 59 / 10.4 | 6 % | 36 % |
| pressure 30 Pa/km, S1 / S3 | 105–153 m | 96–106 m | 13.3 / 1.9, 8.5 / 1.05 | 0 % | 32 % / 68 % |

*Validation, seeds 200–299* (`supervisor_sets_200-299.json`, run once): the same picture.
- Note10: portal 4.1–16.5 m, (e/σ)² ≤ 0.37.
- Low-end except S4: 8.8–19.8 m, (e/σ)² ≤ 0.70.
- Low-end S4: 177–249 m, (e/σ)² 28 → 12.9 with the monitor.
- Pressure gradient: (e/σ)² 8.5–12.7 → 1.05–2.05.
- Exit step ≤ 1.2 m except the pressure gradient on S3 (6.8 m).

Readings:
- **Low-end S4 stays broken in the product configuration** (the low-end IMU; §4.9's 10-state S4 row,
  22.7 / 25.2 m, is the Note10). The 10-state filter has the
  point-linearised heading update that re-association replaced in the ESKF, and fails in some
  sets.
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

  It cannot catch failures in a shared part: a false stop, an IMU scale error.
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
- **Narrow setting.** Two IMU classes, a rigid phone mount, a single lane, one car. A second car
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
  - the map-constrained filters of §4.9 (4 or 10 states, 80–94 m on low-end S4 against the ESKF's
    868–894 m), selected by the supervisor of §4.12, or the pitch prior of §4.10;
  - Android's calibrated gyro stream, which removes the turn-on bias at stops but not the drift in
    the tunnel.
- **A smaller filter** (built as a prototype, §4.9). The 10-state filter matches the ESKF at the
  diverge on a mid-range phone and is the better filter on a low-end one. Before it could replace
  the ESKF it needs:
  - a second hypothesis at the diverge (today it follows the branch when the car goes straight on);
  - a capped re-acquisition at the exit (8–39 m steps);
  - consistency in stop-and-go ((e/σ)² 8.8 on S4), where the ESKF's speed-wander prior is the first
    candidate;
  - the map-error stress of §4.4.
- **The product configuration's open failure is low-end stop-and-go** (§4.12: 89–335 m). The
  10-state filter still has the point-linearised heading update; giving it the ESKF's
  re-association (§4.11) is the first step, then the pitch prior (§4.10).
- **The validation tier is one set of 100 seeds** (§3.1); claims on rarer failures (2 in 200 in
  §4.11) need more.
- **A car that stays in the main tunnel has no curve to correct it.** On S4, 14 % of such runs are
  more than 100 m off at the portal (§3). Only GNSS at the exit, through the follower, fixes it;
  a map with features in the main tunnel (or a curve) would be needed.
- **The branch decision's window** is fixed at 100 m before the diverge. A dead-reckoning error
  larger than that (S4, 1 in 100 validation runs) puts the turn outside it and decides "main";
  the window should grow with σ_s.
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
| Baseline (§4.9) | Asked for a 1D map-constrained filter as a measured reference against the ESKF. A comparison drafted with another AI model gave the design and the questions; the agent re-implemented it from that description and re-measured every number | CHANGELOG v0.5.9 |
| Filter size (§4.9) | Read the 4-vs-16 comparison as "the 16 states buy independence from assumptions, not accuracy" and asked for the middle size. The 10-state filter was built in a separate AI session under his review (8 → 10 states, each addition from a trace); the agent brought it into the repository and re-measured it with 50 seeds, the exit step and the straight-on control | CHANGELOG v0.5.11 |
| Stop detector (§5) | Chose causal and the conservative filter order 4 after the order A/B (order 3 sits next to a cliff) | CHANGELOG v0.5.10 |
| Seed discipline (§3, §4.11) | Read the 200-seed result as a selection effect, not luck: every tuning was done on 0–49. Asked for failure counts next to rms, for the most-worsened seed, and for an untouched validation tier (200–299) run once | CHANGELOG v0.5.11 |
| Map re-association (§4.11) | Read seed 63's mechanism, predicted that the NIS gate would also reject useful innovations (it did), and proposed re-association along the arc. Required the failure cases (wrong branch, map error 2×) before it became the default | CHANGELOG v0.5.11 |
| Pitch prior (§4.10) | Proposed the road-grade prior for low-end phones. Diagnosed the ZUPT speed gate as circular and moved its source to the detector's own confidence. Proposed the offset state, which the measurement then rejected | CHANGELOG v0.5.11 |
| Supervisor (§4.12) | Proposed a layer that selects among the three filters, and deciding before the tunnel while GNSS is good. Approved the class rule and the open-sky rehearsals | CHANGELOG v0.5.11 |
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

**Process controls,** used because generated code can be confidently wrong:
- parameter tags;
- a test that enforces the truth/estimator split;
- (e/σ)² as an acceptance criterion;
- a CHANGELOG entry and a metrics snapshot per version, with every changed metric explained;
- decisions and reviews kept as commits;
- release tags set by the author.
