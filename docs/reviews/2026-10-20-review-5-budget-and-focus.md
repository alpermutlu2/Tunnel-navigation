# Review 5 (2026-10-20): error budget results and project focus

Review of `91f10bd` (budget, DESIGN 4.3), plus the owner's direction on focus.

## Technical points (checked)

| Point | Check | Status |
|---|---|---|
| Long-time instantaneous bias error: none σ², held 2σ², GM prediction σ² | correct for t ≫ τ | agreed |
| The 15.8 m "GM prediction" assumes each GM component's entry value known; from one total b(0) the conditional mean is b(0)·Σσᵢ²e^(−t/τᵢ)/Σσᵢ² | computed: total-known 21.6 m (accel), 703 m (gyro); components-known 15.8 / 507 m; single GM 11.6 m | **corrected** in DESIGN 4.3 (15.8 m labelled oracle lower bound) |
| "No calibration" row is BI-only, not an uncalibrated phone | correct | row meaning stated in DESIGN 4.3 |
| Suvorkin: no temperature characterisation for the Note10; Allan "bending" on the vertical z axis, same pattern on whichever axis was vertical, authors say coefficients unaffected | confirmed from the paper | z-axis B sensitivity noted: held-bias error 13.0 m with x/y-level B vs 30.9 m |
| Temperature not the cause of the ~30 m (off in nominal); adding datasheet drift on top of GM risks double counting | agreed | stress scenario only, [ASSUMED] |
| 3-GM nominal, single GM shown with every key result; later cross-test truth model × estimator model | agreed | |
| Bias error grows ~t² (30 m at 180 s → 120 m at 360 s for a constant residual); S4 needs ZUPT; S1 favours CV | agreed | |
| First ESKF: one lumped slow bias state, GM structure only if tests demand it | agreed | for v0.5 |

## Focus (owner)

The case study says: "Where something is unspecified, make a reasonable assumption and write it
down." So start from a scenario in which the effective bias **can** be estimated (fixed mount,
180 s of GNSS before the tunnel with some accelerations and braking, flat or known grade before
the tunnel), build a navigation algorithm that works there with simulated measurements (oracle
results only as upper bounds), then remove assumptions one at a time and show where and why it
breaks.
