"""Inverse-PSI solver: recommend cold pressures for a target wear at a target lap.

Spec §21.5 / §11.29.

Algorithm:
  1. Coarse seed scan over {22, 27, 32, 37, 42, 47} psi (uniform across all wheels).
  2. Bracket the target wear between two adjacent seeds.
  3. Per-wheel greedy coordinate-descent bisection (4 wheels x <=12 iters each),
     OR uniform bisection if `--uniform-pressure`.
  4. Convergence: |observed - target| <= 0.01 OR |psi_hi - psi_lo| <= 0.5.

`target_wheel` selects the aggregate reduction:
  - "max" -> most-worn wheel (lowest wear_pct).
  - "min" -> least-worn wheel (highest wear_pct).
  - "avg" -> mean across the four wheels.
  - "FL"/"FR"/"RL"/"RR" -> that specific wheel.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .setup import Setup
from .simulator import simulate_stint
from .tyre_state import TyreCalibration, WHEELS

SEED_PSI = (22.0, 27.0, 32.0, 37.0, 42.0, 47.0)
PSI_RANGE = (20.0, 50.0)
WEAR_TOL_PCT = 0.01  # 1% absolute (target is in 0..1, so 0.01 here)
PSI_TOL = 0.5
MAX_ITERS_PER_WHEEL = 12


@dataclass
class SolveResult:
    recommended_psi: dict  # {"FL": ..., "FR": ..., "RL": ..., "RR": ...}
    verification_stint: object  # StintResult re-run with recommended_psi
    target_wear: float
    target_lap: int
    target_wheel: str
    target_wheel_resolved: str  # "max"/"min"/"avg" resolved to FL/.../"avg" tag
    observed_wear: float
    converged: bool
    iterations: int
    uniform: bool
    seed_scan: list = field(default_factory=list)  # [(psi, observed_wear), ...]


def _aggregate(wear_pct_dict: dict, target_wheel: str) -> tuple[float, str]:
    """Reduce per-wheel wear to a single number per `target_wheel`.

    Returns `(wear_value, resolved_wheel_label)` where the second element is
    the wheel name when `target_wheel` is "max" or "min", or the input wheel
    when it's an explicit wheel name, or "avg" when averaging.
    """
    if target_wheel == "avg":
        return sum(wear_pct_dict.values()) / 4.0 / 100.0, "avg"
    if target_wheel == "max":
        # Most-worn = lowest wear_pct.
        w = min(wear_pct_dict, key=lambda k: wear_pct_dict[k])
        return wear_pct_dict[w] / 100.0, w
    if target_wheel == "min":
        # Least-worn = highest wear_pct.
        w = max(wear_pct_dict, key=lambda k: wear_pct_dict[k])
        return wear_pct_dict[w] / 100.0, w
    if target_wheel in WHEELS:
        return wear_pct_dict[target_wheel] / 100.0, target_wheel
    raise ValueError(f"Unknown target_wheel '{target_wheel}'")


def _run_stint(car, track, driver, *, n_laps, pressures, ambient_temp_C, calibration, ds,
               compound=None):
    """Single point evaluation: returns the StintResult."""
    setup = Setup(
        pressures_psi=dict(pressures),
        ambient_temp_C=ambient_temp_C,
        source="solver",
        name="solver-candidate",
        compound=compound.name if compound is not None else None,
    )
    setup.validate()
    return simulate_stint(
        car, track, driver,
        n_laps=n_laps,
        setup=setup,
        calibration=calibration,
        compound=compound,
        ds=ds,
    )


def _wear_at_lap(stint, lap_idx: int, target_wheel: str) -> tuple[float, str]:
    """Get observed wear at end of `lap_idx` (1-based) for `target_wheel`."""
    end = stint.tyre_state_history[lap_idx]  # history[k] = end-of-lap-k
    return _aggregate(end.wear_pct, target_wheel)


def solve_pressure_for_wear(
    car, track, driver, *,
    target_wear: float,
    target_lap: int,
    target_wheel: str = "max",
    uniform: bool = False,
    calibration: TyreCalibration | None = None,
    ambient_temp_C: float = 25.0,
    ds: float = 2.0,
    n_laps_max: int | None = None,
    compound=None,
) -> SolveResult:
    """Bisect cold PSI to hit `target_wear` at `target_lap` (spec §21.5).

    Args:
        car, track, driver: standard sim inputs.
        target_wear: target wear in [0.0, 1.0] (0.50 = 50% worn -> 50% remaining
            tread, i.e. wear_pct = 50).
        target_lap: lap index (1-based) at which wear should hit the target.
        target_wheel: "max"|"min"|"avg"|"FL"|"FR"|"RL"|"RR" (default "max").
        uniform: if True, bisect a single PSI applied to all four wheels.
        calibration: TyreCalibration (default = hand-defaults, measured=False).
        ambient_temp_C: ambient & T_cold reference.
        ds: distance step (m).
        n_laps_max: stint length (defaults to `target_lap`).
    """
    if not 0.0 <= target_wear <= 1.0:
        raise ValueError(f"target_wear must be in [0, 1], got {target_wear}")
    if target_lap < 1:
        raise ValueError(f"target_lap must be >= 1, got {target_lap}")
    if calibration is None:
        calibration = TyreCalibration()
    n_laps_max = n_laps_max if n_laps_max is not None else target_lap

    # Target is in 0..1 ("0.50 = 50% wear"). Convert to wear-percent-remaining
    # for comparison: 50% wear = 50% tread remaining = wear_pct == 50.
    # `_aggregate` returns wear_pct/100 (so 0.50 == 50% remaining). For the
    # solver, "target_wear = 0.50" means the user wants 50% remaining; that's
    # exactly `aggregate(state) == 0.50`. Bisect against `observed - target`.

    # --- 1. Seed scan ---
    seed_scan: list = []
    seed_states: dict = {}
    for psi in SEED_PSI:
        stint = _run_stint(
            car, track, driver,
            n_laps=n_laps_max,
            pressures={w: psi for w in WHEELS},
            ambient_temp_C=ambient_temp_C,
            calibration=calibration,
            ds=ds,
            compound=compound,
        )
        observed, resolved = _wear_at_lap(stint, target_lap, target_wheel)
        seed_scan.append((psi, observed))
        seed_states[psi] = stint

    # --- 2. Bracket ---
    lo_psi, hi_psi = _find_bracket(seed_scan, target_wear)
    if lo_psi is None or hi_psi is None:
        msg = (
            f"could not find a setup hitting {target_wear * 100:.1f}% wear at "
            f"lap {target_lap} in PSI range {PSI_RANGE}. Seed scan: "
            + ", ".join(f"{p:.0f}psi->{w * 100:.1f}%" for p, w in seed_scan)
        )
        raise ValueError(msg)

    # --- 3. Bisect ---
    iters_total = 0
    if uniform:
        psi_lo, psi_hi = lo_psi, hi_psi
        best_psi = 0.5 * (psi_lo + psi_hi)
        for _ in range(MAX_ITERS_PER_WHEEL):
            iters_total += 1
            mid = 0.5 * (psi_lo + psi_hi)
            stint = _run_stint(
                car, track, driver,
                n_laps=n_laps_max,
                pressures={w: mid for w in WHEELS},
                ambient_temp_C=ambient_temp_C,
                calibration=calibration,
                ds=ds,
            )
            observed, _ = _wear_at_lap(stint, target_lap, target_wheel)
            best_psi = mid
            if abs(observed - target_wear) <= WEAR_TOL_PCT:
                break
            if (psi_hi - psi_lo) <= PSI_TOL:
                break
            # Monotonic assumption: which direction makes wear move toward target?
            # Re-bracket using the previous seed observations to decide direction.
            psi_lo, psi_hi = _refine_bracket(seed_scan, target_wear, mid, observed,
                                             psi_lo, psi_hi)
        recommended = {w: best_psi for w in WHEELS}
    else:
        # Per-wheel greedy coordinate descent: seed all 4 wheels at the midpoint
        # of the bracket, then bisect each wheel in turn while holding the others.
        recommended = {w: 0.5 * (lo_psi + hi_psi) for w in WHEELS}
        for w in WHEELS:
            psi_lo, psi_hi = lo_psi, hi_psi
            mid = 0.5 * (psi_lo + psi_hi)
            for _ in range(MAX_ITERS_PER_WHEEL):
                iters_total += 1
                mid = 0.5 * (psi_lo + psi_hi)
                trial = dict(recommended)
                trial[w] = mid
                stint = _run_stint(
                    car, track, driver,
                    n_laps=n_laps_max,
                    pressures=trial,
                    ambient_temp_C=ambient_temp_C,
                    calibration=calibration,
                    ds=ds,
                )
                observed, _ = _wear_at_lap(stint, target_lap, target_wheel)
                if abs(observed - target_wear) <= WEAR_TOL_PCT:
                    recommended[w] = mid
                    break
                if (psi_hi - psi_lo) <= PSI_TOL:
                    recommended[w] = mid
                    break
                psi_lo, psi_hi = _refine_bracket(
                    seed_scan, target_wear, mid, observed, psi_lo, psi_hi
                )
            recommended[w] = mid

    # --- 4. Verification stint ---
    verification = _run_stint(
        car, track, driver,
        n_laps=n_laps_max,
        pressures=recommended,
        ambient_temp_C=ambient_temp_C,
        calibration=calibration,
        ds=ds,
    )
    obs_final, resolved = _wear_at_lap(verification, target_lap, target_wheel)
    converged = abs(obs_final - target_wear) <= WEAR_TOL_PCT

    return SolveResult(
        recommended_psi=recommended,
        verification_stint=verification,
        target_wear=target_wear,
        target_lap=target_lap,
        target_wheel=target_wheel,
        target_wheel_resolved=resolved,
        observed_wear=obs_final,
        converged=converged,
        iterations=iters_total,
        uniform=uniform,
        seed_scan=seed_scan,
    )


def _find_bracket(seed_scan, target_wear):
    """Find adjacent (lo, hi) PSI values whose observed wear straddles target."""
    pairs = sorted(seed_scan, key=lambda t: t[0])
    for i in range(len(pairs) - 1):
        p_lo, w_lo = pairs[i]
        p_hi, w_hi = pairs[i + 1]
        if (w_lo - target_wear) * (w_hi - target_wear) <= 0:
            return p_lo, p_hi
    # No bracket: report the bound straddle status for the error message.
    return None, None


def _refine_bracket(seed_scan, target_wear, mid_psi, mid_observed,
                    psi_lo, psi_hi):
    """Update (lo, hi) using the per-bracket monotonic assumption.

    We treat observed_wear as a function of PSI. The seed scan tells us whether
    that function is increasing or decreasing in this region.
    """
    pairs = sorted(seed_scan, key=lambda t: t[0])
    # Determine monotonic direction over the seed scan as a whole.
    # If observed wear at higher PSI is generally lower (over-inflation reduces
    # contact patch -> lower lat-g -> less wear) then "increasing PSI -> decreasing wear".
    # But across the bracket the local trend wins; use the bracket endpoints.
    # Find seeds nearest psi_lo and psi_hi.
    seed_near_lo = min(pairs, key=lambda t: abs(t[0] - psi_lo))
    seed_near_hi = min(pairs, key=lambda t: abs(t[0] - psi_hi))
    w_lo = seed_near_lo[1]
    w_hi = seed_near_hi[1]
    if w_lo == w_hi:
        # Degenerate; just narrow toward the mid.
        if mid_observed > target_wear:
            return mid_psi, psi_hi
        return psi_lo, mid_psi
    # Direction: True if observed wear increases with PSI.
    increasing = w_hi > w_lo
    if increasing:
        # observed > target -> shrink top half
        if mid_observed > target_wear:
            return psi_lo, mid_psi
        return mid_psi, psi_hi
    # decreasing in PSI
    if mid_observed > target_wear:
        return mid_psi, psi_hi
    return psi_lo, mid_psi
