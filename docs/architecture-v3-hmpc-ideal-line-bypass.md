# Architecture — v3 HMPC ideal-line bypass (direct CSV reference for the inner)

**Spec:** `dev-planning/hmpc-ideal-line-bypass/spec.md` (2026-05-27, planned with Buddy).
**Branch:** `feature/sc-71955/lap-simulation`.

**Predecessors:**
- `docs/architecture-v3-hmpc-outer-line-following.md` — soft line-follow; "Option B
  hard-pin" left unbuilt; curvature-the-chassis-sees conclusion (motivated Arm B).
- `docs/architecture-v3-hmpc-dp-along-ref.md` — `a_long = v·dv/ds` derivation + sign
  conventions (the `_dp_reference.py` pattern this work mirrors for the CSV speed column).
- `docs/architecture-v3-hmpc-inner-asymmetric-pedals.md` — v3.7 inner weights,
  ellipse-soft + Lever-3 Phase-2 headline (143.04 s), brake-ceiling diagnosis.

---

## What the code does (one paragraph)

This work adds a `reference_source = "ideal_csv"` bypass mode to `HMPCController`
that **deletes the outer NLP planner from the per-tick loop entirely** and builds the
inner CasADi tracker's full four-channel reference directly from the offline-optimised
ideal-line CSV, once, at controller construction. The lateral channels (`n_ref`,
`psi_e_ref`) come from the existing `_ideal_line_loader.IdealLineFrenet` projection of
the CSV onto the *plant* `ReferencePath`; the longitudinal channels come from a new
`_ideal_line_speed.IdealLineSpeedReference` that re-indexes the CSV `speed_ms` column
onto the same plant-`s` grid and derives `a_long(s) = v·dv/ds`. The inner tracker
(`hmpc_inner_casadi.py`) is **completely unchanged** — only what fills its reference
sequences changes. The default `reference_source = "nlp"` path is untouched and the
outer NLP still fires per cadence/overrun for every existing caller.

---

## Why this architecture

### Bypass, not an outer-side lever
Every prior outer/DP lever bottomed out at 143–150 s on Sprint A. The diagnosis
(`dp-along-ref` doc, `line-following` doc) was that the *speed reference* the inner is
asked to track is conservative — produced by the DP planner under `safety_margin` /
`chicane_safety_mult`, or by the ~1 s outer horizon. The ideal-line CSV's own
`speed_ms` column integrates to **110.88 s** (only +3.32 s over Tomas 107.56), and it
is *not* run through any safety machinery. The cheapest way to put a near-Tomas-pace
reference in front of the inner — and read off whether the inner can track it — is to
delete the outer NLP and feed the CSV speed directly. This is strictly more decisive
than the unbuilt "Option B hard-pin" (which would still run the NLP).

### One mechanism distinguishes the two arms: the plant curvature κ
The bypass data flow is identical for both arms. The only difference is the **plant
track CSV** (the positional `track` argument), which sets the `ReferencePath` curvature
`kappa_seq` the inner sees:
- **Arm A — centerline plant** (`layout_sprint_a.csv`): the inner carries the CSV's
  near-apex speed through the *centerline* curvature; `n_ref`/`psi_e_ref` are the
  non-zero ideal-line offsets.
- **Arm B — ideal-line plant** (`layout_sprint_a_ideal_line.csv` directly): the chassis
  sees the wider-apex ideal-line κ; the loader self-projects the CSV onto a path built
  from itself, so `n_ref ≈ 0`. This is the R6-mitigation arm.

### Speed reference must live on plant `s` (load-bearing, spec §7.2)
The CSV `distance_m` is the ideal line's own arc-length, but the inner stage grid
`s_seq = s_now + k·ds_stage` lives on plant `s`. `IdealLineSpeedReference` therefore
re-runs the *identical* Frenet projection + row dedup that `IdealLineFrenet` performs,
carries `speed_ms` through the same dedup, and derives `a_long = v·dv/ds` on the
resulting plant-`s` grid via `np.gradient` (handles non-uniform spacing). A
construction-time assertion confirms the speed grid is byte-identical to
`IdealLineFrenet.s`, so all four channels are consistent at every `s_seq`. On Arm B the
projection is a near-identity, so the re-index is a no-op — the same code path serves
both arms with no branch.

### No DP cap leaks into the CSV `v_ref` (spec §5.1 / R3)
In bypass mode the Tier-0 `v_ref` is the raw CSV `speed_ms` (clamped only to
`vx_min + 0.1` for QP well-posedness). The DP plan is still built — it remains the
Tier-1 fallback reference — but its `chicane_safety_mult`/`safety_margin` cap never
touches the Tier-0 channel. Verified: Tier-0 `v_ref(475 m) ≈ 57.5 m/s` (the CSV value),
not the DP-capped chicane speed.

---

## Data flow

### Construction (once)
```
plant CSV ─► build_reference_path ─► self.ref (plant Frenet frame + κ)
ideal CSV ─┬─► load_ideal_line_frenet(ideal, self.ref) ─► IdealLineFrenet  (n_ref, psi_e_ref on plant s)
           └─► load_ideal_line_speed (ideal, self.ref) ─► IdealLineSpeedReference (v_ref, a_long on plant s)
                 (asserts grid == IdealLineFrenet.s; runs a_long sign-check)
```

### Per inner tick — `_resolve_ideal_csv` (Tier-0)
```
_resolve(state, t):
  if reference_source == "ideal_csv": _resolve_ideal_csv(...) ; return   # outer NEVER fired
    s_seq = s_now + arange(N)·ds_stage          (plant s, clipped)
    kappa_seq, v_dp_seq, _ = sample_seq(s_seq, self.ref)     # κ + DP plan (Tier-1 only)
    v_ref_seq      = IdealLineSpeedReference.v_seq(s_seq)    # CSV speed_ms  (NOT DP-capped)
    a_long_ref_seq = IdealLineSpeedReference.a_long_seq(s_seq)
    n_ref_seq      = IdealLineFrenet.n_seq(s_seq)
    psi_e_ref_seq  = IdealLineFrenet.psi_seq(s_seq)
    inner.solve(x0, kappa_seq, v_ref_seq, n_ref_seq, psi_e_ref_seq, u_prev, a_long_ref_seq)
```

### Fallback ladder (spec §8 — there is no outer to fall back to)
- **Tier-0:** inner tracks the CSV reference (all four channels CSV-sourced). Normal path.
- **Tier-1:** on inner infeasibility, retry `inner.solve` with `v_ref_seq = v_dp_seq`,
  `n_ref/psi_e/a_long = None` — the existing DP-plan "safe slow" retry, unchanged.
- **Tier-2:** reactive sub-controller on chassis-divergence (`|n|>4 m` / `|e_psi|>20°`)
  or two consecutive inner-infeasible ticks. Unchanged.
- `self._reference` (the outer `ReferenceTrajectory`) stays `None` in bypass mode; every
  dereference is guarded (the `_emit` PI-trim error and the debug-trace accessors now
  branch on `reference_source == "ideal_csv"` and use the CSV reference — R5).

---

## File inventory

### Created
| File | Purpose |
|---|---|
| `src/lap_estimator/dynamics/_ideal_line_speed.py` | `IdealLineSpeedReference` (~215 lines incl. docstrings). Reads `speed_ms`, re-indexes onto the plant `s` grid via the same projection+dedup as `_ideal_line_loader`, derives `a_long = v·dv/ds`, exposes `v_at/a_long_at/v_seq/a_long_seq`, and the integrated `∫ds/v` lap estimate. |
| `docs/architecture-v3-hmpc-ideal-line-bypass.md` | This document. |
| `.tmp/hmpc_ideal_bypass_sweep.py` | Sweep + MC harness, parameterised on the plant CSV (`--arm A|B`). Modes: `smoke`/`wv`/`wa_es`/`asym`/`mc`. Reuses `.tmp/hmpc_inner_dp_along_sweep.py` patterns. |
| `.tmp/ideal_bypass_rung1.py` | Rung-1 reference-sanity probe (construction-only; no sim). |

### Edited
| File | Change |
|---|---|
| `src/lap_estimator/dynamics/hmpc_controller.py` | New `REFERENCE_SOURCES`/`DEFAULT_REFERENCE_SOURCE` constants; two ctor kwargs (`reference_source`, `ideal_line_csv`) resolved kwarg > driver-JSON > default; `_build_ideal_csv_references` + `_log_ideal_csv_sign_check`; `_resolve` early-returns into `_resolve_ideal_csv` in bypass mode (outer never fires); `_emit` PI-trim branch guards `self._reference`; build-log line surfaces source/CSV/integrated-estimate/residual. |
| `src/lap_estimator/dynamics/slip_simulator.py` | Threaded `hmpc_reference_source` + `hmpc_ideal_line_csv` through `simulate_slip` → `_run_single`/`_run_monte_carlo` → `_make_controller` → `HMPCController`. Dedicated plumbing, independent of the line-follow keys. |
| `lap.py` | New CLI `--hmpc-reference-source {nlp,ideal_csv}` (default `nlp`) and `--hmpc-ideal-line-csv <PATH>`. Dedicated flags — do not reuse `--hmpc-outer-line-follow`. |

### Untouched (hard constraints)
`hmpc_inner_casadi.py` (zero edits), `_ideal_line_loader.py` (reused as-is; private
helpers imported by `_ideal_line_speed`), `_dp_reference.py`, `hmpc_outer.py`, and all
`mpc_*`/`mpcc_*`/reactive/v2/`hmpc_inner_cost.py` code.

---

## How to enable

```bash
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \   # Arm B plant
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi \
  --hmpc-reference-source ideal_csv \
  --hmpc-ideal-line-csv tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv
```
Arm A = pass `layout_sprint_a.csv` as the positional track instead. Driver-JSON
equivalent: `control_params.hmpc.reference_source` + `.ideal_line_csv`.

---

## Results

All runs: Sprint A, BMW 1M (`inertia_zz=2400`, `boost_steady≈0.73`, `Cd≈0.32`,
`brake_torque_mult≈1.80`), Tomas, `inner_solver=casadi`, `chicane_safety_mult=0.95`,
reference = `layout_sprint_a_ideal_line.csv`. Single deterministic seed for smoke
variants. Harness: `.tmp/hmpc_ideal_bypass_sweep.py`.

### Rung 1 — reference sanity (construction only) — PASS, both arms

| Arm | plant | integrated ∫ds/v | n_ref p95 / max | psi_e p95 | grids equal | a_long sign-check |
|---|---|---:|---:|---:|:--:|:--:|
| A | centerline | **110.59 s** | 1.840 / 2.898 m | 0.054 rad | yes | OK |
| B | ideal-line | **110.88 s** | **0.004 / 0.007 m** | 0.0001 rad | yes | OK |

Both within ~1 s of the 110.88 s ceiling. Arm B's `n_ref` p95 ≈ 0 confirms the
near-identity self-projection (no frame/dedup bug). Chicane Tier-0 `v_ref(475 m) ≈
57.5 m/s` = the CSV value (not DP-capped) — R3 mitigated.

### Rung 2 — `nlp`-mode regression — PASS (no behavioral change when off)
Default `--hmpc-reference-source nlp` on the centerline plant: outer NLP fires
(`HMPC outer: n=10, infeas=0`), tier mix `0=89.1% / 1=2.8% / 2=8.1%` — structurally
identical before and after the edit (the bypass code is fully gated behind
`reference_source == "ideal_csv"`). The nlp lap itself aborts at the chicane (s=640) —
that is the **pre-existing working-tree state** of this branch, unchanged by this work.

### Rungs 3–5 — bypass smoke + sweeps — BOTH ARMS ABORT AT THE CHICANE

Arm B — `inner_w_v` sweep (rung 4):
| `inner_w_v` | finished | brake commit s | brake peak | p95 thresh | tier-0 frac | abort s |
|---:|:--:|---:|---:|---:|---:|---:|
| 10 | no | 432.9 | 1.00 | 0.79 | 1.00 | 623.4 |
| 20 | no | 22.2 | 1.00 | 0.81 | 1.00 | 625.2 |
| 40 | no | 60.0 | 1.00 | 0.99 | 1.00 | 630.7 |
| 80 | no | 207.7 | 1.00 | 1.00 | 1.00 | 623.9 |

Arm B — `inner_w_a` × `inner_w_ellipse_soft` grid (rung 5, wv=10):
| wa | es | finished | brake peak | p95 thresh | tier-0 frac | abort s |
|---:|---:|:--:|---:|---:|---:|---:|
| 3 | 500 | no | 1.00 | 0.77 | 1.00 | 622.7 |
| 3 | 1000 | no | 1.00 | 0.62 | 1.00 | 621.7 |
| 5 | 500 | no | 1.00 | 0.79 | 1.00 | 623.4 |
| 5 | 1000 | no | 1.00 | 0.64 | 1.00 | 620.0 |
| 8 | 500 | no | 1.00 | 0.72 | 1.00 | 621.3 |
| 8 | 1000 | no | 0.25 | — | — | **279.3** (early over-brake) |

Arm A — `inner_w_a` × `inner_w_ellipse_soft` grid (rung 5, wv=10):
| wa | es | finished | brake peak | p95 thresh | tier-0 frac | abort s |
|---:|---:|:--:|---:|---:|---:|---:|
| 3 | 500 | no | 1.00 | 0.71 | 1.00 | 624.7 |
| 3 | 1000 | no | 1.00 | 0.63 | 1.00 | 625.0 |
| 5 | 500 | no | 0.45 | — | — | **266.8** (early over-brake) |
| 5 | 1000 | no | 1.00 | 0.63 | 1.00 | 623.4 |
| 8 | 500 | no | 0.98 | 0.72 | 1.00 | 623.2 |
| 8 | 1000 | no | 1.00 | 0.63 | 1.00 | 622.6 |

**No config in either arm finishes a lap.** Every stable variant aborts off-track at
s ≈ 620–640 m (the chicane), with brake peak saturated at 1.00 and tier-0 fraction
1.00 (the inner *is* tracking the CSV reference — the lap is not carried by fallback).
A handful of over-aggressive variants instead abort early on the lead-in (s ≈ 266–279)
from destabilising over-brake. Rungs 6–8 (asym touch-up, chicane-cap isolation, MC)
were not run: there is no finishing config to seat them on, and MC on a deterministic
abort would only re-confirm 0/10.

### The wall — friction-envelope ceiling (quantified)
Trace of Arm B `wa5_es500` through the chicane brake zone `s ∈ [470, 640] m`:

| Quantity | Value |
|---|---:|
| CSV-demanded `a_long` (peak / mean) | **−15.2 / −9.3 m/s²** |
| Plant-achieved decel (peak / mean) | **+7.1 / +5.8 m/s²** |
| Chassis `v` vs CSV `v_ref` at s=470 | 63.0 vs 58.7 m/s |
| Chassis `v` vs CSV `v_ref` at s=574 | 52.9 vs 33.6 m/s (**+19 m/s over**) |
| Chassis `v` vs CSV `v_ref` at s=623 | 47.1 vs 14.8 m/s (**+32 m/s over**, off-track) |

The inner commits brake at a reasonable s (≈438 m, brake 0.63) and saturates to 1.00,
but the fitted Tomas + BMW 1M Pacejka envelope delivers **less than half** the
deceleration the CSV racing-line profile assumes (≈7 m/s² achievable vs ≈15 m/s²
demanded). The chassis cannot bleed the speed in the available distance and arrives at
the chicane apex ~32 m/s too fast. Higher `inner_w_v` (up to 80) only saturates the
brake harder and moves the abort by metres, not past the corner.

---

## Verdict (honest)

**The <3 s-of-Tomas gate was NOT cleared, on either arm.** Both Arm A (centerline κ)
and Arm B (ideal-line κ) abort at the chicane. Because **both** stall — including
Arm B, which sees the wider-apex ideal-line curvature — the binding constraint is
**not** R6 (chassis-curvature) and **not** the speed plan's conservatism (the bypass
removed that, and rung 1 confirmed the CSV reference reaches the inner uncapped). The
wall is the **inner's friction envelope at near-Tomas pace**: the fitted Pacejka
deceleration ceiling (~7 m/s²) is roughly half what the ideal-line CSV's speed profile
demands at the chicane (~15 m/s²). No controller-weight lever can manufacture grip the
plant model does not have.

This matches the spec §9 worst-case branch precisely: *"if both stall, the inner's
friction envelope is binding even at ideal-line curvature, ruling the inner out at
near-Tomas pace."* The CSV `speed_ms` column was generated for a higher-grip envelope
than Tomas + BMW 1M's fitted Pacejka (D_lat ≈ 1.03), so its 110.88 s ceiling is not
physically reachable by *this* plant regardless of how the reference is delivered.

### What the bypass mode delivers (still valuable)
- A clean, tested, bit-identical-when-off mode that puts the most aggressive possible
  reference in front of the inner with the DP/outer conservatism fully removed.
- The decisive measurement that closes a long line of HMPC speculation: the gap to
  Tomas on this plant is **grip-envelope**, not reference conservatism, not outer
  horizon, not chassis curvature.

### Next step (out of scope here; needs a new spec)
To pursue the ideal-line speed on this stack, the lever is the **plant friction
envelope**, not the controller: either re-fit the Pacejka against a higher-grip
calibration (matching the grip the ideal-line CSV was optimised for) or generate an
ideal-line speed profile *against the fitted Tomas envelope* (so the demanded decel is
physically achievable). Both are vehicle-model changes, not HMPC changes, and the spec
explicitly fixed the plant calibration — so this is the honest stopping point for the
bypass campaign.

---

## Notes for the next maintainer
- `_ideal_line_speed.load_ideal_line_speed` imports three private helpers from
  `_ideal_line_loader` (`_read_ideal_line_xy`, `_compute_local_tangent`,
  `_dedup_monotone`) to guarantee its `s_plant` grid is byte-identical to
  `IdealLineFrenet.s`. If you refactor the loader's dedup, the construction-time
  `np.array_equal` assertion in `_build_ideal_csv_references` will warn on drift.
- The CSV `a_long` sign-check (`_log_ideal_csv_sign_check`) reuses the Sprint A
  brake/straight zones from the DP check; on other tracks eyeball the logged means.
- Bypass mode keeps building the DP plan (for the Tier-1 fallback) and runs the
  pre-existing DP `a_long` sign-check — the WARNING you may see on the **centerline**
  plant (`straight mean = -0.000`) is the long-standing DP check on Arm A's plant, not
  the CSV path, which logs OK.
