# Review 7 (owner, 2026-10-20): NHC turn gate, checks before closing v0.5

Summary in English of the owner's review (original in Turkish).

| Item | Decision |
|---|---|
| GM cruise default | Keep |
| Lateral NHC gate in turns | On: `nhc_turn_gate = 0.3 m/s²`, `nhc_turn_k = 0` |
| Gate threshold sensitivity | Short check: 0.2 / 0.3 / 0.5 m/s² on S1 and S2; keep 0.3 if close, otherwise the value is tuned to the simulation |
| Curve without sideslip | Short check: same curves with β = 0; the gate must not cost noticeable accuracy (information loss) |
| Negative branch test | Add: car goes straight at the diverge; the detector must not report the branch |
| Final S1–S6L run | Do it |
| New IMM / acceleration model | Not now |
| S4/S5 late callouts (30 % / 42 % > 1 s) | Limitation and future work; discuss late vs early callout consequences in the report. Possible later: uncertainty-aware early callout s_call = s_exit − d_warn − k·σ_s |

Notes:
- Lateral NHC assumes v_y^b = 0; with sideslip v_y^b = v·sin β ≈ 0.23 m/s (β ≈ 0.6°, v ≈ 22 m/s),
  2.3× the NHC σ, same sign over the whole curve, so a systematic error. The gate uses
  a_lat ≈ |ω_z|·v from the gyro and the estimated speed (no truth) and keeps the vertical row.
- R_yy = 10⁶ removes the row only approximately; skipping the row is cleaner (not required for v0.5).
- 0.23 m/s for 3 s is under 1 m of direct displacement: the ~20 m along-track transient must come
  from the wrong NHC updates spreading into attitude, velocity and bias states. Show it on an
  example trace.
- C (assumption removal) short, four dimensions: GNSS entry quality, phone mount error, bias
  model, sideslip magnitude; nominal vs stressing only.
- D must not be squeezed: working visual simulator, clear README, a few pages of report, AI-usage
  section are direct deliverables and come before a few more metres of accuracy.
