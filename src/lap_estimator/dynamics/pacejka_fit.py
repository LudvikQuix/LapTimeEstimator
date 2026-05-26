"""Fit Pacejka coefficients from lake telemetry (spec §23.3, §23.7).

Five-stage algorithm (spec §23.7.2):

A. Invert per-wheel slip-angle ``alpha`` and slip-ratio ``kappa`` from
   chassis kinematics + per-wheel geometry. See
   :mod:`._slip_inversion.stage_a`.
B. Invert per-wheel forces ``(Fx[w], Fy[w])`` from chassis acceleration
   + yaw moment + drivetrain priors. See :mod:`._slip_inversion.stage_b`.
C. Per-axle Magic Formula coefficient fit via a hand-rolled
   Levenberg-Marquardt with bounded parameters (see ``_fit_helpers``).
D. Combined-slip friction-ellipse exponent. Phase 2 default: 2.0
   (true ellipse). v3.1 fits this from combined-slip samples.
E. Hold-out cross-validation on the newest lap (Phase 2 thresholds:
   RMSE Fy <= 15%, RMSE Fx <= 20%).

Output: ``pacejka_calibration`` block written into the existing
``drivers/<name>.json`` (sibling to ``tyre_calibration``).
"""

from __future__ import annotations

import datetime as _dt
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

from ._fit_helpers import lm_least_squares
from ._slip_inversion import (
    WHEELS,
    build_car_geom,
    has_v3_channels,
    stage_a,
    stage_b,
)
from .pacejka import pacejka_lateral, pacejka_longitudinal

# Per-axle bounds (§23.7.2 Stage C, Phase 2 brief widens C upper to 2.5).
# E bounds tightened 2026-05-15 from [-2.0, +1.0] to [-1.0, +0.5] (sc-71955
# improvement 1): Pacejka book values for real tyres have |E| <= 0.5; the
# wider bound let the optimizer chase noise to the rail on sparse fronts.
#
# D_per_Fz LAT bounds tightened 2026-05-18 from [0.5, 2.5] to [0.95, 1.40]
# (sc-71955 refit-2): the previous lower bound let the LM fit settle at
# D=0.838 (a "soft tyre" solution) which costs +46 s/lap in v3. Real BMW M1
# semislicks (Kunos DY0, compound idx 1) deliver ~1.17 g lateral peak; the
# floor 0.95 forbids the underfit, and the ceiling 1.40 leaves head-room
# for hot/sticky-tyre samples without permitting unphysical 2.0+.
_BOUNDS_LAT_LO = np.array([3.0, 1.0, 0.95, -1.0])
_BOUNDS_LAT_HI = np.array([20.0, 2.5, 1.40, 0.5])
# Longitudinal C floor raised from 1.0 to 1.30 (sc-71955 refit-2): with C<=1
# the Magic Formula is monotone-increasing in kappa (no peak), so the v3
# driver controller can't find a stable slip-ratio target and the rear wheels
# spin out on throttle application -> ghost-fallbacks at corner exits.
# D_long floor raised from 0.5 to 0.95 for the same compound-realism reason
# as the lateral floor: BMW M1 semislicks long-mu peak is ~1.1.
_BOUNDS_LONG_LO = np.array([3.0, 1.30, 0.95, -1.0])
_BOUNDS_LONG_HI = np.array([20.0, 2.5, 1.50, 0.5])
_GUESS_LAT = np.array([10.0, 1.30, 1.17, -0.20])
_GUESS_LONG = np.array([10.0, 1.65, 1.10, 0.30])


# ---------------------------------------------------------------------------
# Stage C — Magic Formula fit per axle, per direction
# ---------------------------------------------------------------------------


def _fit_axle(slip: np.ndarray,
              Fz: np.ndarray,
              F_obs: np.ndarray,
              *,
              guess: np.ndarray,
              lower: np.ndarray,
              upper: np.ndarray,
              direction_fn) -> dict[str, float]:
    """Generic Magic-Formula axle fit (lateral + longitudinal share this).

    Sample weighting: normalised against ``1.5 * Fz`` (a proxy for peak
    grip × Fz) so high-load and light-load samples contribute on the
    same fractional-of-peak-grip scale.
    """
    Fz_safe = np.maximum(Fz, 1000.0)
    ref = 1.5 * Fz_safe

    def resid(p):
        B, C, D, E = p
        return (direction_fn(slip, Fz, B, C, D, E) - F_obs) / ref

    res = lm_least_squares(resid, guess, lower=lower, upper=upper, max_iter=200)
    B, C, D, E = res["x"]
    return {"B": float(B), "C": float(C), "D_per_Fz": float(D), "E": float(E)}


def _fit_lateral_axle(alpha: np.ndarray,
                      Fz: np.ndarray,
                      Fy: np.ndarray) -> dict[str, float]:
    return _fit_axle(
        alpha, Fz, Fy,
        guess=_GUESS_LAT, lower=_BOUNDS_LAT_LO, upper=_BOUNDS_LAT_HI,
        direction_fn=pacejka_lateral,
    )


def _fit_longitudinal_axle(kappa: np.ndarray,
                           Fz: np.ndarray,
                           Fx: np.ndarray) -> dict[str, float]:
    return _fit_axle(
        kappa, Fz, Fx,
        guess=_GUESS_LONG, lower=_BOUNDS_LONG_LO, upper=_BOUNDS_LONG_HI,
        direction_fn=pacejka_longitudinal,
    )


# ---------------------------------------------------------------------------
# Stage E — cross-validation
# ---------------------------------------------------------------------------


def _rmse_pct(pred: np.ndarray, obs: np.ndarray) -> float:
    """RMSE as percent of mean(|obs|). Returns inf if mean(|obs|) is zero."""
    mean_abs = float(np.mean(np.abs(obs)))
    if mean_abs < 1e-9:
        return float("inf")
    rmse = float(np.sqrt(np.mean((pred - obs) ** 2)))
    return rmse / mean_abs * 100.0


def _predict_axle_lat(alpha, Fz, c):
    return pacejka_lateral(alpha, Fz, c["B"], c["C"], c["D_per_Fz"], c["E"])


def _predict_axle_long(kappa, Fz, c):
    return pacejka_longitudinal(kappa, Fz, c["B"], c["C"], c["D_per_Fz"], c["E"])


def _per_axle_diag(pred: np.ndarray, obs: np.ndarray) -> dict:
    """Per-axle fit diagnostics for the ``measured`` field (sc-71955 refit-2).

    Returns absolute RMSE (Newtons), residual p95 (Newtons), sample count, and
    the relative RMSE (percent of mean(|obs|)). The latter matches what
    ``source`` already reports so we can sanity-check the two views agree.
    """
    finite = np.isfinite(pred) & np.isfinite(obs)
    pred = pred[finite]
    obs = obs[finite]
    n = int(pred.size)
    if n == 0:
        return {"rmse_N": float("nan"), "residual_p95_N": float("nan"),
                "rmse_pct": float("nan"), "n_samples": 0}
    residual = pred - obs
    rmse_N = float(np.sqrt(np.mean(residual ** 2)))
    p95 = float(np.percentile(np.abs(residual), 95.0))
    return {
        "rmse_N": round(rmse_N, 2),
        "residual_p95_N": round(p95, 2),
        "rmse_pct": round(_rmse_pct(pred, obs), 3),
        "n_samples": n,
    }


def _cross_validate(holdout_a, holdout_b,
                    front_lat, rear_lat, front_long, rear_long):
    """Compute RMSE percent on the held-out lap for both axles + directions.

    Returns a dict with both the legacy aggregated/per-axle percent metrics
    (consumed by ``source``) and a ``per_axle`` dict of richer per-axle
    diagnostics (consumed by ``measured``).
    """
    def collect(axle, ka, kb):
        a = np.concatenate([holdout_a[w][ka] for w in axle])
        Fz = np.concatenate([holdout_b[w]["Fz"] for w in axle])
        obs = np.concatenate([holdout_b[w][kb] for w in axle])
        return a, Fz, obs

    a_f, Fz_f, Fy_f = collect(("FL", "FR"), "alpha", "Fy")
    a_r, Fz_r, Fy_r = collect(("RL", "RR"), "alpha", "Fy")
    k_f, Fz_kf, Fx_f = collect(("FL", "FR"), "kappa", "Fx")
    k_r, Fz_kr, Fx_r = collect(("RL", "RR"), "kappa", "Fx")

    pred_Fy_f = _predict_axle_lat(a_f, Fz_f, front_lat)
    pred_Fy_r = _predict_axle_lat(a_r, Fz_r, rear_lat)
    pred_Fx_f = _predict_axle_long(k_f, Fz_kf, front_long)
    pred_Fx_r = _predict_axle_long(k_r, Fz_kr, rear_long)
    return {
        "rmse_fy_pct": _rmse_pct(
            np.concatenate([pred_Fy_f, pred_Fy_r]),
            np.concatenate([Fy_f, Fy_r])
        ),
        "rmse_fx_pct": _rmse_pct(
            np.concatenate([pred_Fx_f, pred_Fx_r]),
            np.concatenate([Fx_f, Fx_r])
        ),
        "rmse_fy_front_pct": _rmse_pct(pred_Fy_f, Fy_f),
        "rmse_fy_rear_pct": _rmse_pct(pred_Fy_r, Fy_r),
        "rmse_fx_front_pct": _rmse_pct(pred_Fx_f, Fx_f),
        "rmse_fx_rear_pct": _rmse_pct(pred_Fx_r, Fx_r),
        "per_axle": {
            "front": {
                "lateral": _per_axle_diag(pred_Fy_f, Fy_f),
                "longitudinal": _per_axle_diag(pred_Fx_f, Fx_f),
            },
            "rear": {
                "lateral": _per_axle_diag(pred_Fy_r, Fy_r),
                "longitudinal": _per_axle_diag(pred_Fx_r, Fx_r),
            },
        },
    }


# ---------------------------------------------------------------------------
# Top-level orchestrator
# ---------------------------------------------------------------------------


def _filter_v3_laps(laps: list[dict]) -> list[dict]:
    """Keep only laps with all v3 channels; print skipped ones to stderr."""
    keep = []
    for i, lap in enumerate(laps):
        if not has_v3_channels(lap):
            missing = [c for c in (
                "wheelLoadFL", "wheelAngularSpeedFL",
                "localVelocity_x", "accG_x"
            ) if c not in lap]
            print(f"  fit_pacejka: lap {i} missing v3 channels {missing}; skipped",
                  file=sys.stderr)
            continue
        keep.append(lap)
    return keep


def _pool_axle(stage_a_list, stage_b_list, axle, ka, kb,
               *, load_frac_lo: float = 0.0, load_frac_hi: float = 1.0):
    """Pool per-wheel ``(slip, Fz, F_obs)`` samples across laps for one axle.

    sc-71955 improvement 3: when ``load_frac_lo``/``load_frac_hi`` are
    set, drop samples where the wheel's Fz is outside
    ``[load_frac_lo, load_frac_hi]`` of the axle's total Fz at that
    timestep. Extreme weight-transfer events (one wheel near-lifting)
    are dominated by suspension limits, not by tyre Pacejka, and
    poison the lateral fit on the front axle where Stage B already
    has more noise than the rear.
    """
    pieces_slip, pieces_Fz, pieces_obs = [], [], []
    for sa, sb in zip(stage_a_list, stage_b_list):
        axle_Fz_total = np.maximum(
            sum(sb[ww]["Fz"] for ww in axle), 1.0
        )
        for w in axle:
            slip = sa[w][ka]
            Fz_w = sb[w]["Fz"]
            obs = sb[w][kb]
            frac = Fz_w / axle_Fz_total
            mask = (frac >= load_frac_lo) & (frac <= load_frac_hi)
            pieces_slip.append(slip[mask])
            pieces_Fz.append(Fz_w[mask])
            pieces_obs.append(obs[mask])
    return (np.concatenate(pieces_slip),
            np.concatenate(pieces_Fz),
            np.concatenate(pieces_obs))


def _clean(arrays, v_long_filter=None):
    """Drop NaN/inf rows; optionally drop v_long below 5 m/s."""
    n = len(arrays[0])
    mask = np.ones(n, dtype=bool)
    for arr in arrays:
        mask &= np.isfinite(arr)
    if v_long_filter is not None:
        mask &= v_long_filter > 5.0
    return [a[mask] for a in arrays]


def _resolve_compound(laps: list[dict], override: str | None) -> str | None:
    if override is not None:
        return override
    for lap in laps:
        comp = lap.get("tyreCompound")
        if comp is None or len(comp) == 0:
            continue
        first = comp[0] if hasattr(comp, "__len__") else comp
        if isinstance(first, bytes):
            first = first.decode("utf-8", "ignore")
        return str(first)
    return None


def fit_pacejka_from_laps(laps: list[dict],
                          *,
                          car_data_dir,
                          compound_name: str | None = None) -> dict[str, Any]:
    """Run the 5-stage Pacejka fit, return a JSON-ready ``pacejka_calibration`` block.

    Each ``lap`` is a dict-of-arrays (the shape produced by
    ``telemetry.merge_with_track`` + the v3 channels appended by
    ``lake_loader``). Last lap is held out for cross-validation.
    """
    if len(laps) < 2:
        raise ValueError(
            f"fit_pacejka_from_laps: need >= 2 laps "
            f"(N-1 train + 1 holdout), got {len(laps)}"
        )
    keep = _filter_v3_laps(laps)
    if len(keep) < 2:
        raise ValueError(
            f"fit_pacejka_from_laps: need >=2 laps with full v3 channels, "
            f"only {len(keep)} found"
        )
    geom = build_car_geom(car_data_dir)

    train_laps = keep[:-1]
    holdout = keep[-1]

    train_a = [stage_a(lap, geom) for lap in train_laps]
    train_b = [stage_b(lap, geom, sa) for lap, sa in zip(train_laps, train_a)]

    # Pool per-axle clouds. sc-71955 improvement 3: drop samples where the
    # wheel carries <30% or >70% of its axle's load -- extreme weight-transfer
    # events dominated by suspension, not Pacejka.
    #
    # sc-71955 refit-2 (2026-05-18): widened to [0.10, 0.90]. The [0.30, 0.70]
    # filter excluded the peak-grip samples (outer wheel ~75-85% of axle load
    # at apex), which forced the LM fit onto the D_per_Fz lower rail. Real
    # telemetry has per-wheel mu_y p95 = 1.07 and lat_g max = 1.92 -- the
    # high-mu evidence lives in the tail the old filter discarded.
    LOAD_FRAC_LO, LOAD_FRAC_HI = 0.10, 0.90
    a_f, Fz_f, Fy_f = _pool_axle(
        train_a, train_b, ("FL", "FR"), "alpha", "Fy",
        load_frac_lo=LOAD_FRAC_LO, load_frac_hi=LOAD_FRAC_HI,
    )
    a_r, Fz_r, Fy_r = _pool_axle(
        train_a, train_b, ("RL", "RR"), "alpha", "Fy",
        load_frac_lo=LOAD_FRAC_LO, load_frac_hi=LOAD_FRAC_HI,
    )
    k_f, Fz_kf, Fx_f = _pool_axle(
        train_a, train_b, ("FL", "FR"), "kappa", "Fx",
        load_frac_lo=LOAD_FRAC_LO, load_frac_hi=LOAD_FRAC_HI,
    )
    k_r, Fz_kr, Fx_r = _pool_axle(
        train_a, train_b, ("RL", "RR"), "kappa", "Fx",
        load_frac_lo=LOAD_FRAC_LO, load_frac_hi=LOAD_FRAC_HI,
    )

    # Reject samples with non-finite values.
    a_f, Fz_f, Fy_f = _clean([a_f, Fz_f, Fy_f])
    a_r, Fz_r, Fy_r = _clean([a_r, Fz_r, Fy_r])
    k_f, Fz_kf, Fx_f = _clean([k_f, Fz_kf, Fx_f])
    k_r, Fz_kr, Fx_r = _clean([k_r, Fz_kr, Fx_r])

    # Stage C — pooled lateral + pooled longitudinal (sc-71955 improvement 2).
    # Tomas's BMW M1 runs Semislicks on all four wheels (same compound, same
    # construction): the physical (B, C, D_per_Fz, E) for the LATERAL direction
    # is identical on front and rear. Per-wheel variation comes only from Fz
    # (already accounted for by D_per_Fz * Fz inside the formula). Pooling
    # FL+FR+RL+RR for the lateral fit lets the rear samples (clean) dominate
    # while front samples (noisier from Stage B's Fy decomposition) just add
    # data points -- principled, not a workaround.
    #
    # For longitudinal: only the driven axle (rear for RWD) carries meaningful
    # Fx, so we pool the driven-axle samples and apply the result to both
    # axles. The non-driven axle's Fx is dominated by rolling resistance
    # noise and would corrupt the fit.
    a_all = np.concatenate([a_f, a_r])
    Fz_all = np.concatenate([Fz_f, Fz_r])
    Fy_all = np.concatenate([Fy_f, Fy_r])
    pooled_lat = _fit_lateral_axle(a_all, Fz_all, Fy_all)
    front_lat = dict(pooled_lat)
    rear_lat = dict(pooled_lat)

    drive = geom.drive_type.upper()
    if drive == "FWD":
        k_drive, Fz_drive, Fx_drive = k_f, Fz_kf, Fx_f
    elif drive == "AWD":
        # All four are driven; pool all four.
        k_drive = np.concatenate([k_f, k_r])
        Fz_drive = np.concatenate([Fz_kf, Fz_kr])
        Fx_drive = np.concatenate([Fx_f, Fx_r])
    else:
        # RWD (default).
        k_drive, Fz_drive, Fx_drive = k_r, Fz_kr, Fx_r
    pooled_long = _fit_longitudinal_axle(k_drive, Fz_drive, Fx_drive)
    front_long = dict(pooled_long)
    rear_long = dict(pooled_long)

    # Stage D — Phase 2: hand-default ellipse exponent.
    ellipse_exponent = 2.0

    # Stage E — cross-validation on holdout.
    holdout_a = stage_a(holdout, geom)
    holdout_b = stage_b(holdout, geom, holdout_a)
    rmse = _cross_validate(
        holdout_a, holdout_b,
        front_lat, rear_lat, front_long, rear_long,
    )
    cv_passed = (rmse["rmse_fy_pct"] <= 15.0
                 and rmse["rmse_fx_pct"] <= 20.0)

    compound = _resolve_compound(keep, compound_name)
    n_samples = int(sum(len(sb["FL"]["Fz"]) for sb in train_b))
    now_iso = _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    return {
        "front": {"lateral": front_lat, "longitudinal": front_long},
        "rear":  {"lateral": rear_lat,  "longitudinal": rear_long},
        "friction_ellipse_exponent": ellipse_exponent,
        # sc-71955 refit-2: ``measured`` is now a dict of per-axle fit
        # diagnostics, not a bool. Consumers that need a truthy "is this
        # fitted?" check should test `pacejka_calibration.measured.get(...)`
        # presence or fall back to "measured": False on legacy JSON.
        "measured": rmse["per_axle"],
        "source": {
            "compound": compound,
            "fit_version": "v3.0",
            "fit_date": now_iso,
            "n_laps_train": len(train_laps),
            "n_laps_holdout": 1,
            "n_samples": n_samples,
            "rmse_lat_pct": round(rmse["rmse_fy_pct"], 3),
            "rmse_long_pct": round(rmse["rmse_fx_pct"], 3),
            "rmse_lat_front_pct": round(rmse["rmse_fy_front_pct"], 3),
            "rmse_lat_rear_pct": round(rmse["rmse_fy_rear_pct"], 3),
            "rmse_long_front_pct": round(rmse["rmse_fx_front_pct"], 3),
            "rmse_long_rear_pct": round(rmse["rmse_fx_rear_pct"], 3),
            "cv_passed": bool(cv_passed),
            "fitted_at": now_iso,
        },
    }


def merge_into_driver_json(driver_json_path,
                           pacejka_block: dict[str, Any]) -> None:
    """Merge ``pacejka_block`` into an existing driver JSON, preserving fields.

    The block is written under the top-level key ``pacejka_calibration``;
    every other top-level field (skill_pct, tyre_calibration, profile,
    source, etc.) is preserved unchanged.
    """
    p = Path(driver_json_path)
    if p.exists():
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        data = {}
    data["pacejka_calibration"] = pacejka_block
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def fit_pacejka_from_lake(lake_csv_paths,
                          *,
                          car_data_dir,
                          compound_name: str | None = None,
                          output_driver_json=None,
                          require_cv_pass: bool = False) -> dict[str, Any]:
    """Phase-2 entry: read AC log CSVs from disk, fit Pacejka, optionally write.

    This entrypoint is the path used when telemetry sits as local CSVs.
    For the lake path, see ``fit_slip.py`` which calls
    :func:`fit_pacejka_from_laps` directly via ``load_laps_from_lake``.
    """
    from ..telemetry import read_ac_log
    laps = []
    for path in lake_csv_paths:
        telem = read_ac_log(str(path))
        laps.append(telem)
    block = fit_pacejka_from_laps(
        laps, car_data_dir=car_data_dir, compound_name=compound_name,
    )
    if require_cv_pass and not block["source"]["cv_passed"]:
        raise RuntimeError(
            f"Pacejka fit failed cross-validation: "
            f"RMSE Fy={block['source']['rmse_lat_pct']:.1f}% "
            f"RMSE Fx={block['source']['rmse_long_pct']:.1f}%"
        )
    if output_driver_json is not None:
        merge_into_driver_json(output_driver_json, block)
    return block
