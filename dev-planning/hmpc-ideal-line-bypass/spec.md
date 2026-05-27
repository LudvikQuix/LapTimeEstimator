# HMPC ideal-line bypass — direct CSV reference for the inner tracker

**Status:** Draft
**Project:** LapTimeEstimator
**Created:** 2026-05-27
**Branch:** `feature/sc-71955/lap-simulation`
**Planned with:** Buddy

---

## 1. Motivation

Every prior HMPC lever (line-follow soft pin, DP-sourced `a_long_ref`, asymmetric
pedals, ellipse-relax + Lever-3) bottomed out around **143–150 s** on Sprint A and
never closed the gap to reactive (126.00 s) or Tomas (107.56 s). The diagnoses
converge on one root cause: the speed/accel reference the inner is asked to track
is **conservative**, because it is produced by the DP planner under
`safety_margin` + `chicane_safety_mult`, and/or by the outer NLP whose ~1 s
horizon under-asks for brake.

The decisive number: the ideal-line CSV
(`tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv`) carries its **own**
`speed_ms` column, and `∫ ds/v` over that column integrates to **110.88 s** —
only **+3.32 s** off Tomas's real 107.56 s lap. That column is *not* run through
the DP planner's safety machinery; it is the offline-optimised racing-line speed
profile directly. The DP plan, by contrast, yields ~126 s (reactive) / 143–150 s
(HMPC) — i.e. the DP planner's conservatism is discarding **15–30 s of pace**.

This spec bypasses the HMPC outer NLP planner **entirely**. Instead of solving an
online trajectory-optimization NLP every outer tick, we build the inner tracker's
full reference (`n_ref`, `psi_e_ref`, `v_ref`, `a_long_ref`) directly from the
offline ideal-line CSV, once, at controller construction. The inner CasADi tracker
(unchanged) pure-tracks this fixed reference.

The campaign runs **two plant arms in parallel** (see §6):

- **Arm A (centerline plant) — leads the ladder.** Plant on the **centerline**
  CSV (`layout_sprint_a.csv`) — same plant trick as the existing line-follow mode;
  only the reference *source* changes. `n_ref`/`psi_e_ref` are the non-zero
  ideal-line Frenet offsets against the centerline; the inner is asked to carry
  the CSV's near-apex speed through the *centerline* curvature.
- **Arm B (ideal-line plant) — R6-mitigation arm, first-class.** Plant on the
  **ideal-line** CSV (`layout_sprint_a_ideal_line.csv`) directly — the same plant
  trick reactive already uses. There the centerline *is* the ideal line, so the
  chassis sees the ideal-line curvature κ, `n_ref ≈ 0`, `psi_e_ref ≈ 0`, and the
  `_ideal_line_loader` Frenet projection is a near-identity (self-projection).
  `v_ref` is still the CSV `speed_ms` and `a_long_ref` is still `v·dv/ds`. Per the
  line-follow doc's conclusion (`docs/architecture-v3-hmpc-outer-line-following.md`),
  the chassis-sees-centerline-κ vs ideal-line-κ gap (Risk R6) is the likely wall,
  so the <3 s-of-Tomas target almost certainly requires Arm B — it is the arm
  **most likely to hit the gate**, not a "maybe follow-up".

In both arms the outer NLP is deleted from the loop and the *speed* reference is
CSV-sourced; only the plant track CSV (and hence the Frenet residual
`n_ref`/`psi_e_ref` the inner sees) differs between arms.

The research question: **can the inner track a near-Tomas-pace reference?** The
theoretical ceiling here is 110.88 s and there is essentially **zero tracking
slack**. The inner has historically been the binding constraint (aborts at the
chicane on aggressive brake/slip demands). This is ambitious by design; the make-
or-break risk is the inner's brake/slip tracking of an aggressive `v_ref` — under
Arm A also through the centerline curvature it has historically choked on, which
is exactly why Arm B carries the campaign's load.

---

## 2. Architecture

### 2.1 Data flow — today (`reference_source = "nlp"`, unchanged)

```
centerline CSV ──► build_reference_path ──► ReferencePath (s, kappa, half_w, v_dp)
                                                   │
plant on centerline CSV                            ▼
                          HMPCController._resolve (per inner tick):
                            (a) cadence/overrun → outer.solve(state) → ReferenceTrajectory
                            (b) sample v_ref/n_ref/psi_e_ref/a_long_ref from the
                                frozen ReferenceTrajectory at the inner stage grid
                            (c)(d) inner.solve(...) tracks it
                            Tier-1 = DP-plan fallback; Tier-2 = reactive
```

### 2.2 Data flow — bypass (`reference_source = "ideal_csv"`, NEW)

The bypass data flow is identical for both arms; the **plant track CSV** is the
only difference (it is the positional `track` argument to `lap.py`, not a bypass
flag). The `--hmpc-ideal-line-csv` reference source is the *same* CSV in both arms.

```
plant CSV ──────► build_reference_path ──► ReferencePath  (PLANT + Frenet frame; unchanged)
  (Arm A: layout_sprint_a.csv                      │
   Arm B: layout_sprint_a_ideal_line.csv)          │
                                                   │
ideal-line CSV ──┬─► IdealLineFrenet (REUSE _ideal_line_loader)   → n_ref(s), psi_e_ref(s)
  (--hmpc-       └─► IdealLineSpeedReference (NEW, ~80 lines)      → v_ref(s), a_long_ref(s)
   ideal-line-csv)      (v_ref = CSV speed_ms; a_long = v·dv/ds via np.gradient,
                         pattern REUSED from _dp_reference.DPLongitudinalReference)
                                                   │
plant on plant CSV                                 ▼
                          HMPCController._resolve (per inner tick):
                            (a) OUTER NOT FIRED — no NLP solve, ever
                            (b) sample v_ref/n_ref/psi_e_ref/a_long_ref DIRECTLY from
                                the cached CSV-backed references at the inner stage grid
                            (c)(d) inner.solve(...) tracks it  (inner UNCHANGED)
                            Tier-1 = DP-plan fallback (still available); Tier-2 = reactive
```

Key points:
- The outer NLP (`OuterPlanner.solve`) is **never invoked** in bypass mode. There
  is no `ReferenceTrajectory`, no IPOPT, no outer cadence/overrun logic.
- All four inner reference channels come from CSV-derived, construction-time-cached
  callables sampled per-tick via `np.interp` on the inner stage grid `s_seq`.
- `n_ref` / `psi_e_ref` are in the **plant** Frenet frame (so they are
  consistent with the inner's `x0 = [n_now, e_psi_now, ...]` against `self.ref`),
  exactly as `_ideal_line_loader` already produces them for line-follow.
  - **Arm A:** plant = centerline → `n_ref`/`psi_e_ref` are the (non-zero)
    ideal-line offsets against the centerline frame.
  - **Arm B:** plant = ideal line → the loader projects the ideal-line CSV onto a
    `ReferencePath` built from the *same* ideal-line CSV, so `n_ref ≈ 0`,
    `psi_e_ref ≈ 0` to within projection/dedup tolerance (a near-identity
    self-projection). ArchDev should log the residual and confirm it is small;
    a large residual indicates a frame/dedup bug, not a real offset.
- `v_ref` and `a_long_ref` are on the **plant** arc-length `s` grid too — the
  speed reference must be re-indexed onto plant `s` (see §7.2), because the
  inner stage grid `s_seq = s_now + k·ds_stage` lives on plant `s`. In Arm B
  the plant `s` and the ideal-line CSV's own `s` coincide, so the re-index is a
  near-identity; in Arm A it is the real centerline-`s` re-index.

### 2.3 Why bypass (not Option B hard-pin)

`docs/architecture-v3-hmpc-outer-line-following.md` left "Option B (hard pin —
replace `n` with `n_ideal` as a constant in the NLP)" unbuilt and concluded that
soft line-follow does NOT help because the gap is *the curvature the chassis sees*,
not the lateral plan. Option B would still run the NLP (just with `n` fixed). This
bypass goes further: it deletes the NLP from the loop and replaces the *speed*
reference too — which is the channel that prior work showed actually moves lap time
(the DP/outer speed plan is the conservatism source). It is the cheapest possible
way to put a near-Tomas-pace reference in front of the inner and read off whether
the inner can track it. Arm B additionally addresses the doc's *actual* conclusion
— that the binding gap is the chassis curvature, fixed by planting the chassis on
the ideal-line CSV (the reactive trick), not by any outer-side lateral plan.

---

## 3. File inventory

### Created
| File | Purpose |
|---|---|
| `src/lap_estimator/dynamics/_ideal_line_speed.py` | `IdealLineSpeedReference` (~80 lines). Reads `distance_m`/`speed_ms` from the ideal-line CSV, re-indexes onto the plant `s` grid (§7.2), derives `a_long(s)=v·dv/ds` via `np.gradient` (pattern from `_dp_reference.py`). Provides `v_at(s)`, `a_long_at(s)`, `v_seq(s_seq)`, `a_long_seq(s_seq)`. Sign convention: decel negative. |
| `docs/architecture-v3-hmpc-ideal-line-bypass.md` | Post-implementation architecture/results doc (ArchDev fills results after the tuning campaign; must report **both arms** separately). |
| `.tmp/hmpc_ideal_bypass_sweep.py` | Sweep + MC harness (modes per §6). Reuse the patterns from `.tmp/hmpc_inner_ellipse_soft_sweep.py`. Parameterised on plant CSV so the same rungs run for Arm A and Arm B. |

### Edited
| File | Change |
|---|---|
| `src/lap_estimator/dynamics/hmpc_controller.py` | New `reference_source` resolution (kwarg > `control_params.hmpc.reference_source` > default `"nlp"`). In `"ideal_csv"` mode: build `IdealLineFrenet` (REUSE `_ideal_line_loader.load_ideal_line_frenet`) + `IdealLineSpeedReference` at construction; in `_resolve`, branch step (a)/(b) to skip the outer and fill the four reference channels from the cached CSV references; force `_outer_disable`-equivalent behavior for the outer cadence block. Build-log line surfaces `reference_source` + the CSV path + integrated lap-time estimate + the `n_ref`/`psi_e_ref` residual p95 (so Arm B's near-identity is verifiable in the log). |
| `src/lap_estimator/dynamics/slip_simulator.py` | Thread `reference_source` and the ideal-line CSV path (the dedicated `--hmpc-ideal-line-csv` plumbing — **do NOT overload** the existing `--hmpc-outer-line-follow` path) through `simulate_slip` → `_run_single`/`_run_monte_carlo` → `_make_controller` → `HMPCController.__init__`. The plant CSV is unchanged plumbing (the existing positional `track` argument); selecting Arm A vs Arm B is purely which track CSV is passed. |
| `lap.py` | New CLI: `--hmpc-reference-source {nlp,ideal_csv}` (default `nlp`) and `--hmpc-ideal-line-csv <PATH>` (the speed+line source). **Dedicated flag — does NOT reuse the line-follow plumbing** (per Q2 decision). |

### Untouched (HARD constraints)
- `src/lap_estimator/dynamics/hmpc_inner_casadi.py` — the inner already accepts
  `(n_ref_seq, psi_e_ref_seq, v_ref_seq, a_long_ref_seq)`; we only change what fills
  them. **Zero edits.**
- `src/lap_estimator/dynamics/_ideal_line_loader.py` — REUSE as-is for `n_ref`/`psi_e_ref`.
- `src/lap_estimator/dynamics/_dp_reference.py` — REUSE the `v·dv/ds` pattern (do not
  edit; the new `_ideal_line_speed.py` mirrors it for the CSV speed column).
- `src/lap_estimator/dynamics/hmpc_outer.py` — outer NLP unchanged (just not called
  in bypass mode).
- `mpc_*`, `mpcc_*`, reactive (`driver_controller.py`), v2 (`car.py`,
  `simulator.py`), `hmpc_inner_cost.py`, `mpc_qp.py` — untouched.

---

## 4. Plant calibration (FIXED inputs — do not sweep)

Already established; pass through unchanged:
- `boost_steady ≈ 0.73`
- `Cd ≈ 0.32`
- `brake_torque_mult ≈ 1.80`
- `inertia_zz = 2400`

These are plant/vehicle calibration, not controller knobs. The tuning campaign
(§6) sweeps controller weights only. Both arms use the same calibration.

---

## 5. Tuning knobs (the "whole chain")

### 5.1 chicane_safety_mult / DP chicane cap interaction (IMPORTANT)
In bypass mode `v_ref` comes from the **CSV `speed_ms` column**, NOT from the DP
plan. Therefore the DP planner's `chicane_safety_mult` and `safety_margin` **must
NOT clip `v_ref`** — that would re-introduce the exact conservatism we are
bypassing. Spec decision:
- The DP plan is still built (the controller needs it for the **Tier-1 fallback**
  reference and the existing `_dp_along_ref` sign-check build). But in bypass mode
  the Tier-0 `v_ref`/`a_long_ref` channels are CSV-sourced and the DP cap does not
  touch them.
- `chicane_safety_mult` therefore only affects the **Tier-1 fallback** speed in
  bypass mode. It should be left at its prior value (e.g. 0.95) so that *if* the
  inner drops to Tier-1 at the chicane it still has a sane (conservative) target.
- ArchDev must verify in code review that no DP cap is applied to the CSV `v_ref`
  in the bypass path. Add an assertion/log that the Tier-0 `v_ref` at the chicane
  apex matches the CSV value (within interp tolerance), not the DP-capped value.

### 5.2 Inner MPC weights (the primary campaign)
- `inner_w_v` — v-tracking gain. Must be high enough that the inner actually chases
  the aggressive CSV `v_ref` (prior finding: w_v=0.5 left the brake commit at the
  DP baseline; bumped to 10.0 to bite). Likely needs to go **higher** here.
- `inner_w_a` — Lever 3 (`(a_long − a_long_ref)²`). Now fed the CSV-derived
  `a_long`, which is a *whole-lap, near-Tomas* decel profile — exactly the
  long-horizon signal Lever 3 wanted, without the DP conservatism. Prior stable
  window was [3, 5] under relaxed ellipse; re-sweep here.
- `inner_w_ellipse_soft` — Phase-2 finding: relaxing 5000 → 500 unlocks brake
  authority but only pays off paired with Lever 3. Re-sweep paired with the CSV
  reference.
- v3.7 asym-pedal weights: `inner_w_du_brake` (≈0.5), `inner_w_du_throttle` (≈10),
  `inner_w_brake_double_well` (≈50), `inner_w_brake_throttle_overlap` (≈50). Start
  at the Phase-2 headline defaults; only sweep if brake commit is the binding
  failure.

The same weight knobs are swept on both arms. The best weights need not coincide
between arms (Arm B's wider apex curvature may admit higher `inner_w_v` before the
ellipse fires); tune each arm independently from the shared starting config.

### 5.3 Inner tick rate / horizon
Default inner = 30 m / 15 stages. The CSV `a_long_ref` is whole-lap, so the inner
already gets long-horizon decel info via Lever 3 even at a short horizon. Only widen
the inner horizon if the brake-commit-too-late failure persists after the weight
campaign — and note that widening costs inner solve time.

---

## 6. Tuning campaign plan (ordered ladder)

Two plant arms, run as **parallel first-class arms**. Both on Sprint A, BMW 1M
(`inertia_zz=2400`), Tomas, `inner_solver=casadi`, reference =
`layout_sprint_a_ideal_line.csv`. The arms differ only in the **plant track CSV**:

- **Arm A — centerline plant.** Plant = `layout_sprint_a.csv`, reference = ideal-line
  Frenet (n_ref, psi_ref ≠ 0). Honors the originally-chosen scope; **leads the ladder**.
- **Arm B — ideal-line plant.** Plant = `layout_sprint_a_ideal_line.csv` directly →
  `n_ref ≈ 0`, `psi_ref ≈ 0`, chassis sees ideal-line κ. `v_ref` still = CSV
  `speed_ms`; `a_long` still = `v·dv/ds`. This is the R6-mitigation arm and the one
  **most likely to hit the target**. Not a follow-up — run it as part of this campaign.

Run order: complete the ladder on **Arm A first** (rungs 1–8), then run the same
ladder on **Arm B**. If Arm A stalls below the gate at the chicane (the expected
R6 outcome), Arm B is the answer, not a future experiment — do not defer it.
Single deterministic seed per smoke variant; 10-seed MC only at each arm's best
stable config. Each rung gates the next *within* an arm.

1. **Reference sanity (no sweep).** Build the controller in `ideal_csv` mode and
   log: integrated `∫ds/v` of the CSV `v_ref` on the plant `s` grid (expect
   ≈110.88 s ± re-indexing error), `a_long` sign-check (decel<0 at chicane,
   ≥0 on straight), `n_ref` p95/max from `IdealLineFrenet`. **Gate:** integrated
   estimate within ~1 s of 110.88; sign-check OK. For Arm B additionally assert
   `n_ref`/`psi_e_ref` residual p95 ≈ 0 (near-identity self-projection); a large
   residual is a frame/dedup bug. If not, fix re-indexing (§7.2) before tuning.

2. **Bit-identity guard.** Run `--hmpc-reference-source nlp` (default) and confirm
   the lap is byte-identical to the current branch baseline (no behavioral change
   when off). **Gate:** identical. (Run once; arm-independent.)

3. **First bypass smoke.** `ideal_csv` at the Phase-2 headline inner config
   (es=500, wa=5, asym defaults, `inner_w_v=10`). Read: finished?, lap time, brake
   commit `s`, brake peak, p95-thresh, abort `s` if any, tier-0 fraction. This
   establishes whether the inner even survives the aggressive reference.

4. **`inner_w_v` sweep** ∈ {10, 20, 40, 80}. The CSV `v_ref` is the whole point;
   find the gain where the inner actually tracks it without destabilising. Pick the
   best finisher.

5. **`inner_w_a` × `inner_w_ellipse_soft` grid** at the best `w_v`:
   `w_a ∈ {3, 5, 8}` × `es ∈ {500, 1000}`. Prior work shows a narrow stability
   window; the CSV `a_long` may shift it. Pick the best finisher.

6. **Asym-pedal touch-up** (only if brake commit is the binding failure):
   `inner_w_brake_double_well ∈ {50, 100}`, `inner_w_du_brake ∈ {0.5, 2}`.

7. **chicane interaction check.** Confirm (per §5.1) that the Tier-0 `v_ref` at the
   chicane is the CSV value, and that varying `chicane_safety_mult` does NOT move
   the Tier-0 lap time (only Tier-1 fallback). Document.

8. **MC verification** (10 seeds) at the single best stable config. **Gate:**
   ≥7/10 finishes (ideally 10/10) with median lap ≤ 110.56 s.

Then repeat rungs 1, 3–8 on **Arm B** (rung 2 is arm-independent). Report Arm A
and Arm B results side by side in the results doc; the headline finding is which
arm (if either) clears the §9 gate, and whether Arm B clears it where Arm A does
not (which would confirm R6 as the wall).

### 6.x Run commands (Bash, never PowerShell)

```bash
# Rung 1/3 — Arm A bypass smoke (centerline plant, single seed):
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi \
  --hmpc-reference-source ideal_csv \
  --hmpc-ideal-line-csv tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv

# Rung 1/3 — Arm B bypass smoke (ideal-line plant, single seed):
#   ONLY the positional track CSV changes (plant = ideal-line CSV);
#   the --hmpc-ideal-line-csv reference flag is identical.
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv \
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi \
  --hmpc-reference-source ideal_csv \
  --hmpc-ideal-line-csv tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv

# Rung 2 — bit-identity guard (default; must equal current baseline; arm-independent):
python lap.py cars_csv/bmw_1m \
  tracks_csv/ks_nurburgring/layout_sprint_a.csv \
  drivers/tomas.json \
  --model slip --controller hmpc --single-lap --no-plot \
  --inertia-zz 2400 --chicane-safety-mult 0.95 \
  --hmpc-inner-solver casadi \
  --hmpc-reference-source nlp

# Weight sweeps via driver-JSON control_params.hmpc block, driven by
# (harness takes the plant CSV as an argument so the same rungs run per arm):
python .tmp/hmpc_ideal_bypass_sweep.py smoke  --arm A   # rung 3, Arm A
python .tmp/hmpc_ideal_bypass_sweep.py wv     --arm A   # rung 4
python .tmp/hmpc_ideal_bypass_sweep.py wa_es  --arm A   # rung 5
python .tmp/hmpc_ideal_bypass_sweep.py asym   --arm A   # rung 6
python .tmp/hmpc_ideal_bypass_sweep.py mc     --arm A   # rung 8
python .tmp/hmpc_ideal_bypass_sweep.py smoke  --arm B   # rung 3, Arm B
# ... same ladder for --arm B ...
```

Driver-JSON block for `ideal_csv` mode (identical for both arms; the arm is the
plant track CSV passed on the command line, not a JSON key):
```json
"control_params": {
  "hmpc": {
    "inner_solver": "casadi",
    "reference_source": "ideal_csv",
    "ideal_line_csv": "tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv",
    "inner_w_v": 20.0,
    "inner_w_a": 5.0,
    "inner_w_ellipse_soft": 500.0,
    "inner_w_du_brake": 0.5,
    "inner_w_du_throttle": 10.0,
    "inner_w_brake_double_well": 50.0,
    "inner_w_brake_throttle_overlap": 50.0
  }
}
```

---

## 7. Data & interface contracts

### 7.1 `reference_source` switch
- `control_params.hmpc.reference_source ∈ {"nlp", "ideal_csv"}`, default `"nlp"`.
- `control_params.hmpc.ideal_line_csv` (JSON) / `--hmpc-ideal-line-csv` (CLI) carry
  the ideal-line CSV path. **Dedicated keys — do NOT overload the existing
  line-follow `path`/`--hmpc-outer-line-follow` plumbing** (Q2 decision).
- `"nlp"` is **bit-identical to today** — the outer NLP path is untouched.
- `"ideal_csv"` requires a resolvable ideal-line CSV path. If the path is
  missing/unresolvable, raise at construction (do NOT silently fall back to `nlp`).
- The plant track CSV (Arm A vs Arm B) is the existing positional `track` argument;
  it is orthogonal to `reference_source` and to `ideal_line_csv`.

### 7.2 Speed-reference re-indexing onto plant `s` (load-bearing)
The CSV `distance_m`/`speed_ms` are on the **ideal-line's own** arc-length. The
inner stage grid is on the **plant** `s` (centerline `s` for Arm A; ideal-line `s`
for Arm B). The `IdealLineFrenet` loader already computes, per ideal-line sample,
its projected plant `s` (`IdealLineFrenet.s`). `IdealLineSpeedReference` MUST map
the CSV speed onto the same plant `s` axis:
- Build `s_plant` per ideal-line CSV row by reusing the loader's projection (or by
  consuming `IdealLineFrenet.s` directly, since it is already row-aligned and
  monotone-deduplicated — preferred, to avoid re-projecting).
- Resample `speed_ms` onto the dedup-monotone `s_plant` grid, then derive
  `a_long = v·dv/ds` on that grid via `np.gradient` (handles non-uniform spacing).
- `v_seq(s_seq)` / `a_long_seq(s_seq)` then `np.interp` against `s_plant`, clipped
  at the grid edges (same convention as `_dp_reference` and `IdealLineFrenet`).
- **Contract:** the `s_plant` grid used for speed must be identical to the
  `IdealLineFrenet.s` grid used for `n_ref`/`psi_e_ref`, so all four channels are
  consistent at every `s_seq`.
- **Arm B note:** when the plant CSV *is* the ideal-line CSV, the loader projects
  the ideal line onto a reference built from itself, so `s_plant` ≈ the ideal-line's
  own `distance_m` and the re-index is a near-identity. The same code path serves
  both arms; no Arm-B-specific branch is needed.

### 7.3 Sign convention (per `architecture-v3-hmpc-dp-along-ref.md`)
- `a_long_ref < 0` in brake zones (decel), `≥ 0` on straights/accel.
- `v·dv/ds` with `dv/ds<0` in brake ⟹ `a_long<0`. Matches outer NLP and inner
  chassis-frame conventions. Reuse `_log_dp_along_sign_check`'s zone logic to verify
  the CSV-derived `a_long` on build (log WARNING on violation, no abort).

### 7.4 Inner solve call (UNCHANGED signature)
`inner.solve(x0, kappa_seq, v_ref_seq, n_ref_seq, psi_e_ref_seq, u_prev,
a_long_ref_seq)` — same as today. In bypass mode all four ref sequences are
non-None and CSV-sourced on the Tier-0 path. (`kappa_seq` comes from the plant
`ReferencePath`, so it carries centerline κ in Arm A and ideal-line κ in Arm B —
this is the single mechanism by which the arms differ in what the inner sees.)

---

## 8. Tier / fallback behavior (must-spec — there is no outer to fall back to)

In `nlp` mode the tiers are: Tier-0 HMPC (outer+inner), Tier-1 DP-plan inner
fallback (on outer-ref rejection or inner infeasibility), Tier-2 reactive. In
`ideal_csv` mode there is **no outer**, so the fallback ladder changes (identically
for both arms):

- **Tier-0 (bypass):** inner tracks the CSV reference (all four channels CSV-sourced).
  This is the normal path.
- **Tier-1 (DP-plan inner fallback):** UNCHANGED mechanism and STILL used. When the
  inner returns `infeasible`, retry `inner.solve` with `v_ref_seq = v_dp_seq`
  (the DP plan), `n_ref_seq=None`, `psi_e_ref_seq=None`, `a_long_ref_seq=None` —
  exactly the existing Tier-1 retry at `hmpc_controller.py:966`. The DP plan is the
  natural "safe slow" reference; this is why we still build it in bypass mode. Note
  this drops the chassis back to plant-centerline-tracking at DP pace for that tick.
- **Tier-2 (reactive):** UNCHANGED. Entered on chassis-divergence
  (`|n|>4 m` or `|e_psi|>20°`) or two consecutive inner-infeasible ticks, with the
  same hysteresis-exit window. The reactive sub-controller is built regardless of
  `reference_source`.
- **No "stale outer reference" path:** in `nlp` mode an infeasible outer keeps the
  stale `ReferenceTrajectory`. In bypass mode there is no outer ref to go stale —
  the CSV reference is static and always valid, so that branch is simply not
  exercised. The controller must not assume `self._reference` exists in bypass mode
  (guard the debug-trace `v_ref_at(s_now)` accessors, which currently dereference
  `self._reference`).
- **Acceptance-gate consequence:** report Tier-0 fraction. If the lap only "meets"
  the gate because it spends meaningful time in Tier-1/Tier-2 (DP/reactive pace),
  that is NOT a pass — the question is whether the *inner* tracks the CSV reference.
  Require Tier-0 ≥ ~95% in the brake-threshold zone (s∈[475,585]) at the headline
  config, mirroring the Phase-2 honesty bar.

---

## 9. Acceptance gates

Gates apply **per arm**; report both. Clearing the primary gate on *either* arm
is a project-level success, but the results doc must state which arm cleared it.

| Gate | Target |
|---|---|
| **Primary lap time** (achieved HMPC lap on the CSV reference) | **≤ 110.56 s** (within 3 s of Tomas 107.56) |
| MC stability | **≥ 7/10** finishes (ideally 10/10) at the headline config |
| Tier-0 fraction in brake-threshold zone (s∈[475,585]) | ≥ ~95% (lap not carried by fallback) |
| `a_long_ref` sign-check | OK (decel<0 at chicane, ≥0 on straight) |
| Re-indexed CSV `v_ref` integrated estimate | within ~1 s of 110.88 s |
| Arm B `n_ref`/`psi_e_ref` residual p95 | ≈ 0 (near-identity self-projection) |
| `nlp`-mode regression | bit-identical to current branch baseline |

The theoretical ceiling is **110.88 s** (the CSV's own integrated time), so the
primary gate leaves ~0.3 s of tracking slack above the floor and 3 s above Tomas.
Be honest in the results doc: if neither arm tracks within ~3 s, record the
failure mode (where/why it aborts or bleeds time) for **each arm** — that is itself
the research deliverable. In particular, if Arm A stalls at the chicane but Arm B
clears the gate, that confirms R6 (chassis curvature, not speed plan, was the wall);
if *both* stall, the inner's friction envelope is binding even at ideal-line
curvature, ruling the inner out at near-Tomas pace.

---

## 10. Risks

- **R1 — Inner cannot track the aggressive `v_ref` (PRIMARY, make-or-break).** Every
  prior lever aborted at the chicane on aggressive brake/slip. The CSV reference is
  *more* aggressive than anything tried (near-Tomas, no DP conservatism). The
  friction-ellipse soft penalty at the chicane apex (documented in
  asym-pedals doc §"binding constraint") may fire hard and either abort or force
  Tier-2. Mitigation: the `inner_w_v` / `inner_w_a` / `inner_w_ellipse_soft` ladder
  (§6) on **both arms**; accept that an honest "inner cannot track" is a valid outcome.
- **R2 — Re-indexing error (§7.2).** If the CSV speed is left on ideal-line `s`
  instead of plant `s`, `v_ref` is misaligned with the inner stage grid and the
  brake commit lands at the wrong `s`. Gate rung 1 catches this (integrated estimate
  + sign-check). (Near-identity in Arm B, but the rung-1 gate still runs.)
- **R3 — DP cap leaks into CSV `v_ref` (§5.1).** If any existing code path clips
  `v_ref` by `chicane_safety_mult`/`safety_margin`, the bypass silently re-imports
  the conservatism it was built to remove. Mitigation: explicit assertion/log that
  Tier-0 chicane `v_ref` == CSV value.
- **R4 — Fallback masks the result (§8).** A "passing" lap that is actually carried
  by Tier-1/Tier-2 pace would be a false positive. Mitigation: Tier-0-fraction gate.
- **R5 — `self._reference` dereferences in bypass mode.** Debug-trace and any code
  assuming an outer `ReferenceTrajectory` exists must be guarded.
- **R6 — Curvature-the-chassis-sees gap (from line-follow doc) — NOW ADDRESSED BY
  ARM B.** The line-follow doc concluded the 22-s gap lived in centerline-κ vs
  ideal-line-κ, not the lateral plan. **Arm A** runs the chassis on the centerline,
  so the inner still carries the CSV's near-apex speed through the *centerline*
  curvature — this may be the wall (Arm A's expected failure mode). **Arm B**
  mitigates R6 directly: it plants the chassis on the ideal-line CSV (the reactive
  trick), so the chassis sees the wider-apex ideal-line κ. Arm B is the primary
  hedge against R6 and is run as a first-class arm, not a follow-up. Residual risk:
  even at ideal-line curvature the inner's friction envelope may still bind at
  near-Tomas pace (the chase-Tomas doc found the outer-NLP variant of this lands at
  ~2:28 because L3 destabilises on that track too); if Arm B also stalls, that is
  the honest "inner is the binding constraint" finding.

---

## 11. Resolved decisions

- **Q1 — Plant arms: RESOLVED.** Run **both** plant configs as parallel first-class
  arms (see §6). Arm A (centerline plant) leads the ladder to honor the chosen
  scope; Arm B (ideal-line plant directly — the reactive plant trick) is a
  first-class part of the campaign, not a follow-up, and is the R6-mitigation arm
  most likely to hit the <3 s-of-Tomas target. On Arm B the centerline IS the ideal
  line, so the `_ideal_line_loader` Frenet projection is a near-identity and
  `n_ref ≈ psi_e_ref ≈ 0`. The single mechanism distinguishing the arms is the
  plant `ReferencePath` κ the inner sees (centerline vs ideal-line); everything
  else (speed source, `a_long`, fallback ladder, weights to sweep) is shared.
- **Q2 — CLI surface: RESOLVED.** Use a **dedicated** `--hmpc-ideal-line-csv <PATH>`
  CLI flag plus `control_params.hmpc.reference_source: "ideal_csv"` and
  `control_params.hmpc.ideal_line_csv` JSON keys. **Do NOT overload** the existing
  `--hmpc-outer-line-follow` / `control_params.hmpc.line_follow.path` plumbing —
  line-follow and bypass are distinct modes and must stay independently selectable.

---

## 12. References

- `docs/architecture-v3-hmpc-outer-line-following.md` — soft line-follow; "Option B
  hard-pin" left unbuilt; curvature-the-chassis-sees conclusion (motivates Arm B).
- `docs/architecture-v3-hmpc-dp-along-ref.md` — `a_long=v·dv/ds` derivation + sign
  conventions.
- `docs/architecture-v3-hmpc-inner-asymmetric-pedals.md` — v3.7 inner weights,
  ellipse-soft + Lever-3 Phase-2 headline (143.04 s), brake-ceiling diagnosis.
- `docs/architecture-v3-hmpc-casadi-outer.md` — the outer NLP being bypassed +
  `ReferenceTrajectory` shape.
- `src/lap_estimator/dynamics/hmpc_controller.py` — `_resolve` (outer fire + inner
  ref build), `_resolve_a_long_ref`, Tier-1/Tier-2 ladder (lines ~829–1037).
- `src/lap_estimator/dynamics/_ideal_line_loader.py` — `IdealLineFrenet` (REUSE).
- `src/lap_estimator/dynamics/_dp_reference.py` — `v·dv/ds` pattern (REUSE).
- `tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv` — speed+line source
  (`speed_ms` column; integrates to 110.88 s); also the **Arm B plant** track.
