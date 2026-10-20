# Review 6 (owner, 2026-10-20): cruise speed model, exit branch, barometer

Summary in English of the owner's review (original in Turkish). Decisions:

| Topic | Decision |
|---|---|
| Cruise (b) on by default | Yes; re-validate with the corrected (barometer) results |
| S3 barometer fix (dda405e) | Accepted, subject to a residual-error and R check |
| Exit branch on the route in every scenario | Accepted (conditions below) |
| Branch confirmation from the gyro | Accepted as a secondary feature |
| Gauss–Markov speed-wander state | Implement experimentally |
| Declare S6 solved | Not yet |
| Spreading to new algorithms | Not now |

Order: (1) finalise S1–S6 after the barometer fix; (2) add the GM speed state and compare it
directly with (b); (3) add the exit-branch geometry to all scenarios and re-evaluate callout and
recovery.

## 1. Exit branch geometry
- Every scenario: a diverge inside the tunnel and an exit branch on the followed route; the
  callout is given before the diverge. 20° over 150 m (R ≈ 430 m) [ASSUMED] as a synthetic start.
- S1 keeps its character: the curve must not artificially improve straight-tunnel
  observability. Report results before and after the diverge separately.
- Same total tunnel distance for comparability, but state that the last part's dynamics changed.
- Evaluate branch confirmation independently of the navigation algorithm: map matching along the
  route does not by itself prove the right branch was taken. Compare the measured heading change
  with both map candidates, r_ψ,j = Δψ̂ − Δψ_map,j (j = main, branch); 5° is only a simple
  reference detector, later normalised by the yaw uncertainty. Do not spend much time here.

## 2. GM speed-wander state: accepted as an idea, the claim corrected
- Model v = v_ref + d, ḋ = −d/τ_d + w, σ_d = 0.5 m/s, τ_d = 60 s as a starting point.
- The earlier claim "driver wander is bounded, bias grows, so they separate in ~50 s" is wrong as
  an observability statement: a GM process is not hard-bounded, and real small acceleration and
  accelerometer bias still enter the same measurement (ã = a + b_a + n). The GM adds a
  statistical prior on driver behaviour, not a sensor measurement. Correct claim: *assuming
  cruise speed changes follow a stated statistical model, the estimate is more robust to
  bias-driven drift during the outage.*
- 0 = v_∥ − v_ref − d as a kinematic identity with small R; speed-change uncertainty belongs in
  the process model of d, not in an inflated R. Discrete: φ = e^(−Δt/τ_d), Q_d = σ_d²(1 − φ²).
- Keep the cross-covariance between the cloned v_ref and the current speed.
- Acceptance test: NEES/NIS and coverage, not only rms. If S6 improves but (e/σ)² returns to
  4–5, reject.

## 3. Parameters and cases
| | σ_d | τ_d |
|---|---|---|
| stiffer | 0.3 m/s | 30 s |
| nominal | 0.5 m/s | 60 s |
| looser | 1 m/s | 120 s |

Cases at least: constant speed (S1), slow ramps (S6), accelerations slightly above the detector
threshold; also other wander amplitudes. Cruise detector unchanged.

## 4. Barometer check
First-order lag correction is right; check the second-order term ½ḧΔt² on S3 and that the filter
R matches the 1 s average (variance and time correlation). Close the barometer model if both are
reasonable.

## Reporting
The exit callout is now one of the main success metrics: report its timing, its uncertainty and
the late-callout rate, not only position error. Keep what the sensors measure separate from what
we assume about the driver.
