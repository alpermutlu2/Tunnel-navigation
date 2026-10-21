# Review 10 (2026-10-21): third repository review at 7abf54e (Claude Fable 5.1, passed on by the owner)

Summary in English (original in Turkish).

Verdict: review 9's main finding closed overnight, implementation better than the prototype
(windowed matching, pause + in-loop branch decision, map-error states, explicit curve-matching
Jacobian); 176 tests, 50-seed evaluation, negative controls 500/500. Tags still not pushed.

New finding: the map quality assumption dominates the map layer's gain, and the report tests it
with one fixed +1σ stress. Reviewer's runs with per-seed random map errors (8 seeds): at the
default σ the filter is consistent ((e/σ)² 0.9–1.25) but the 5 km portal gain is 34 → 19 m, not
34 → 10 m; at twice the default σ (4 m, 1 %) it is overconfident ((e/σ)² 3.8–5.3), and widening
the prior does not fix it. A deterministic scale term in the cross-track row,
H[1, ε] = nᵀ(c − c_entry), improves S2 (1.6) but not 5 km.

Actions: (1) push tags; (2) random 1×/2× stress rows under "map quality", say the σ are [ASSUMED]
and must be measured on a real map (OSM tunnel geometry is traced from imagery; 1 % over 5 km is
plausible); (3) optional scale Jacobian; (4) branch metric from the in-loop decision, or explain the
offline check; (5) negative-control portal numbers as evidence that the gain comes only from curves.
Also: the 1.5 m cross-track equals the map's lateral prior (unobservable offset), and footnote the
seed-count difference in the 5 km comparison.
