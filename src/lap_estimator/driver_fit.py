"""Telemetry-driven driver fitting (v1.1 multi-lap + v1.2 profile dynamics + v2 tyre calibration).

Given merged per-lap telemetry frames and a Car physics model, derive
`skill_pct` and `consistency_sigma` from the pooled cornering samples and
measure the v1.2 dynamic profile fields.

Algorithm (spec §13.5 + §13.14):
  0. (v2, §13.14) If per-wheel state channels present, fit tyre-calibration
     knobs BEFORE skill_pct so calibrated f_temp/f_wear/f_pressure feed
     into lat_g_max (decouples tyre-state confounds from driver skill).
     Optimization is NumPy-only coordinate descent in log10 space (no SciPy
     dependency); see `fit_tyre_calibration`.
  1. (Per lap) Out-lap trim happens at the CLI layer before merge.
  2. Per-lap lat_g_obs = v^2 / R / g, exclude straights (R >= 500 m).
  3. Per-lap lat_g_max = car.tyre_grip_lateral(v) * (1 + downforce / (m*g)).
     (v2 with measured calibration: also scaled by f_temp · f_wear · f_pressure
      evaluated at the sample's per-wheel-state values.)
  4. Per-lap util = lat_g_obs / lat_g_max, clipped to [0, 1.2].
  5. Pool every lap's cornering-sample util array; take the 85th percentile
     for skill_pct, scaled stdev for consistency_sigma_seconds.
  5b. (v1.2) `profile_dynamics.measure_dynamics(merged_frames)` -> dynamic
      profile fields, plumbed into `FitResult.profile`.
  6+ Caller writes JSON and runs the validation sim.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .profile_dynamics import ProfileDynamics, measure_dynamics
from .telemetry import has_per_wheel_state
from .tyre_state import (
    SegmentInfo,
    TyreCalibration,
    TyreState,
    WHEELS,
    build_car_tyre_model,
    derive_radius_sign,
    f_pressure,
    f_temp,
    f_wear,
    update_per_segment,
)

STRAIGHT_THRESHOLD_M = 500.0
G = 9.81
# A lap is "finished" when its post-trim normalizedCarPosition span >= this.
FINISHED_NCP_SPAN = 0.9


@dataclass
class FitResult:
    skill_pct: float
    consistency_sigma: float
    util_p85: float
    util_stdev: float
    n_corner_samples: int
    n_over_unity: int
    n_laps: int = 1
    n_finished_laps: int = 0
    real_lap_times_s: list = field(default_factory=list)
    real_lap_time_s: float | None = None
    pooled_sample_count: int = 0
    profile: ProfileDynamics | None = None
    # v2 (§13.14): tyre-calibration block. `measured=False` if per-wheel state
    # channels were absent from the input telemetry.
    tyre_calibration: TyreCalibration | None = None
    tyre_calibration_rmse: dict = field(default_factory=dict)
    # v2 (§21.11): resolved compound for the fit. Records the compound the
    # fitter calibrated against, which downstream callers can echo into the
    # driver JSON's `tyre_calibration.source.compound`.
    compound_name: str | None = None
    compound_source: str | None = None  # "telemetry" | "car-default"


def fit_driver(
    car,
    merged_frames,
    *,
    straight_threshold_m: float = STRAIGHT_THRESHOLD_M,
    util_percentile: float = 85.0,
    track=None,
) -> FitResult:
    """Fit a driver from a list of per-lap merged telemetry frames.

    Backward compatibility shim: a single dict argument (legacy v1 callers)
    is wrapped into a one-element list and treated as a single-lap fit. The
    multi-lap guard (`>=2 laps required`) lives at the CLI layer where the
    user-facing error message can be emitted.

    v2: when per-wheel state channels are present on every frame, fits the
    four tyre-calibration knobs before computing skill_pct. The calibrated
    grip envelope (f_temp · f_wear · f_pressure) is then woven into the
    per-sample lat_g_max so skill_pct does not absorb tyre-state confounds.
    """
    frames = _normalise_frames(merged_frames)

    # --- v2 step -1 (§21.11): resolve active compound from telemetry. ---
    compound, compound_source = _resolve_compound_from_frames(car, frames)

    # --- v2 step 0: tyre-calibration fit (spec §13.14). ---
    tyre_calibration, tyre_rmse = _try_fit_tyre_calibration(
        car, frames, track, compound,
    )

    pooled_util: list[np.ndarray] = []
    n_over_unity_total = 0
    n_corner_total = 0
    real_lap_times: list[float] = []
    n_finished = 0

    # v2 grip-envelope helper: if calibration is measured + the frame has per-
    # wheel state channels, fold the per-sample (g_FL,g_FR,g_RL,g_RR) into
    # lat_g_max so util is decoupled from tyre-state confounds.
    tyre_model_for_envelope = None
    if tyre_calibration is not None and tyre_calibration.measured:
        tyre_model_for_envelope = build_car_tyre_model(car, compound)

    for merged in frames:
        v = merged["speed_ms"]
        r = merged["radius_m"]
        corner_mask = (r < straight_threshold_m) & (v > 1.0)
        if not corner_mask.any():
            # This lap contributes nothing to the skill pool but still counts
            # toward `n_laps`. Skip its util calc.
            pass
        else:
            v_c = v[corner_mask]
            r_c = r[corner_mask]
            lat_g_obs = (v_c ** 2) / np.clip(r_c, 1.0, None) / G
            lat_g_max = np.array([
                car.tyre_grip_lateral(float(vv))
                * (1.0 + car.downforce(float(vv)) / (car.total_mass * G))
                for vv in v_c
            ])
            if tyre_model_for_envelope is not None and has_per_wheel_state(merged):
                envelope = _per_sample_grip_envelope(
                    merged, corner_mask, tyre_model_for_envelope
                )
                lat_g_max = lat_g_max * envelope
            lat_g_max = np.clip(lat_g_max, 1e-3, None)
            util = lat_g_obs / lat_g_max
            util_clipped = np.clip(util, 0.0, 1.2)
            pooled_util.append(util_clipped)
            n_over_unity_total += int((util > 1.0).sum())
            n_corner_total += int(corner_mask.sum())

        ncp = merged.get("normalizedCarPosition")
        if ncp is not None and len(ncp) > 0:
            ncp_span = float(np.max(ncp) - np.min(ncp))
        else:
            ncp_span = 0.0
        if ncp_span >= FINISHED_NCP_SPAN:
            ts = merged.get("timestamp_ms")
            if ts is not None and len(ts) >= 2:
                real_lap_times.append(float((ts.max() - ts.min()) / 1000.0))
                n_finished += 1

    if not pooled_util:
        raise ValueError("No cornering samples across any lap (all radii above straight threshold).")

    pool = np.concatenate(pooled_util)
    p85 = float(np.percentile(pool, util_percentile))
    skill = float(np.clip(p85, 0.05, 1.0))
    stdev = float(np.std(pool, ddof=1)) if len(pool) > 1 else 0.0
    sigma_s = float(np.clip(stdev / 0.03, 0.0, 1.5))

    profile = measure_dynamics(frames)

    real_lap_time_s = (
        float(np.mean(real_lap_times)) if real_lap_times else None
    )

    return FitResult(
        skill_pct=skill,
        consistency_sigma=sigma_s,
        util_p85=p85,
        util_stdev=stdev,
        n_corner_samples=n_corner_total,
        n_over_unity=n_over_unity_total,
        n_laps=len(frames),
        n_finished_laps=n_finished,
        real_lap_times_s=[round(t, 3) for t in real_lap_times],
        real_lap_time_s=round(real_lap_time_s, 3) if real_lap_time_s is not None else None,
        pooled_sample_count=int(len(pool)),
        profile=profile,
        tyre_calibration=tyre_calibration,
        tyre_calibration_rmse=tyre_rmse,
        compound_name=compound.name if compound is not None else None,
        compound_source=compound_source,
    )


def _per_sample_grip_envelope(merged, corner_mask, model):
    """Per-sample reduced grip envelope using measured per-wheel state.

    Uses the same `0.5·(min(g_FL,g_FR) + min(g_RL,g_RR))` rule the simulator
    applies (`tyre_state.combined_grip_envelope`). Returns an array of length
    `corner_mask.sum()`.
    """
    idx = np.where(corner_mask)[0]
    out = np.ones(len(idx), dtype=float)
    for j, i in enumerate(idx):
        g = {}
        for w in WHEELS:
            is_front = w[0] == "F"
            T = float(merged[f"tyreTemp{w}"][i])
            W = float(merged[f"tyreWear{w}"][i])
            P = float(merged[f"wheelsPressure{w}"][i])
            g[w] = (
                f_temp(model, T, front=is_front)
                * f_wear(model, W, front=is_front)
                * f_pressure(model, P, front=is_front)
            )
        g_front = min(g["FL"], g["FR"])
        g_rear = min(g["RL"], g["RR"])
        out[j] = 0.5 * (g_front + g_rear)
    return out


def _try_fit_tyre_calibration(car, frames, track, compound=None):
    """Wrap `fit_tyre_calibration` with the per-wheel-state availability check.

    Returns `(TyreCalibration, rmse_dict)`. Defaults with `measured=False` when
    any frame lacks the per-wheel-state channels. `compound` is the active
    `Compound` (§21.11); when None, falls back to `car.default_compound` inside
    `build_car_tyre_model`.
    """
    have_all = all(has_per_wheel_state(f) for f in frames)
    if not have_all:
        return TyreCalibration(measured=False), {}
    if track is None:
        # Calibration needs signed-radius info; track CSV is required.
        return TyreCalibration(measured=False), {}
    try:
        cal, rmse = fit_tyre_calibration(car, frames, track, compound=compound)
    except Exception as e:  # pragma: no cover - defensive: bad LUT, etc.
        return TyreCalibration(measured=False, source={"error": repr(e)}), {}
    return cal, rmse


def _resolve_compound_from_frames(car, frames):
    """Pick the most-common `tyreCompound` value across frames; resolve it.

    Returns `(Compound, source_tag)` where `source_tag` is `"telemetry"` on
    match, `"car-default"` on miss (warns) or absence. Spec §13.14 / §21.11.
    """
    counts: dict = {}
    for frame in frames:
        col = frame.get("tyreCompound")
        if col is None:
            continue
        for v in col:
            if v is None:
                continue
            s = str(v).strip()
            if not s:
                continue
            counts[s] = counts.get(s, 0) + 1
    if not counts:
        return car.default_compound, "car-default"
    most_common = max(counts.items(), key=lambda kv: kv[1])[0]
    resolved = car.find_compound(most_common)
    if resolved is None:
        default = car.default_compound
        print(
            f'  warn: tyre compound "{most_common}" not found in car.compounds '
            f'-- falling back to default "{default.name}" (spec §13.14)'
        )
        return default, "car-default"
    return resolved, "telemetry"


def fit_tyre_calibration(car, merged_frames, track, *, compound=None):
    """Fit (k_friction, h, C_thermal, k_wear) against measured per-wheel state.

    Spec §13.14. NumPy-only optimisation: coordinate descent in log10 space
    with two refinement passes. Returns `(TyreCalibration(measured=True), rmse_dict)`.

    Loss: w_T·RMSE(temp) + w_W·RMSE(wear) + w_P·RMSE(pressure); weights 1.0/5.0/0.5.

    `compound` is the active `Compound` (spec §21.11). When None,
    falls back to `car.default_compound`.
    """
    frames = _normalise_frames(merged_frames)

    # Pre-compute per-sample simulator inputs for each frame (no re-derivation
    # inside the loss). For each frame, build SegmentInfo lists keyed by index.
    model = build_car_tyre_model(car, compound)
    cooked_frames = [_cook_frame_for_calibration(car, f, track) for f in frames]

    # Loss in log10 space over 4 knobs. Bounds enforced by clipping.
    LOG_BOUNDS = {
        "k_friction": (np.log10(0.1), np.log10(10.0)),
        "h": (np.log10(5.0), np.log10(500.0)),
        "C_thermal": (np.log10(500.0), np.log10(50000.0)),
        "k_wear": (np.log10(1e-9), np.log10(1e-4)),
    }
    W_T, W_W, W_P = 1.0, 5.0, 0.5

    def _loss(log_knobs):
        k_friction = 10 ** float(np.clip(log_knobs[0], *LOG_BOUNDS["k_friction"]))
        h = 10 ** float(np.clip(log_knobs[1], *LOG_BOUNDS["h"]))
        C_thermal = 10 ** float(np.clip(log_knobs[2], *LOG_BOUNDS["C_thermal"]))
        k_wear = 10 ** float(np.clip(log_knobs[3], *LOG_BOUNDS["k_wear"]))
        calib = TyreCalibration(
            k_friction=k_friction, h=h, C_thermal=C_thermal, k_wear=k_wear,
            measured=True,
        )
        sq_T, sq_W, sq_P, n = 0.0, 0.0, 0.0, 0
        for cooked in cooked_frames:
            err_T, err_W, err_P, k = _frame_rmse(cooked, car, calib, model)
            sq_T += err_T
            sq_W += err_W
            sq_P += err_P
            n += k
        if n == 0:
            return float("inf"), 0.0, 0.0, 0.0
        rmse_T = float(np.sqrt(sq_T / n))
        rmse_W = float(np.sqrt(sq_W / n))
        rmse_P = float(np.sqrt(sq_P / n))
        return W_T * rmse_T + W_W * rmse_W + W_P * rmse_P, rmse_T, rmse_W, rmse_P

    # Initial guess: defaults (k_friction=1, h=50, C_thermal=5000, k_wear=1e-7).
    x = np.array([np.log10(1.0), np.log10(50.0), np.log10(5000.0), np.log10(1.0e-7)])
    best_loss, best_T, best_W, best_P = _loss(x)
    # Coordinate descent: cycle through each knob, line-search 5 candidates +/- step.
    for outer in range(8):  # up to 8 outer sweeps; usually converges in 3-4.
        improved = False
        for i in range(4):
            lo, hi = list(LOG_BOUNDS.values())[i]
            step = max(0.4 / (1 + outer), 0.05)
            candidates = [x[i] + step * d for d in (-2, -1, 0, 1, 2)]
            best_candidate = x[i]
            for c in candidates:
                c_clip = float(np.clip(c, lo, hi))
                trial = x.copy()
                trial[i] = c_clip
                lv, T, W, P = _loss(trial)
                if lv < best_loss - 1e-6:
                    best_loss = lv
                    best_T, best_W, best_P = T, W, P
                    best_candidate = c_clip
                    improved = True
            x[i] = best_candidate
        if not improved:
            break

    k_friction = 10 ** float(np.clip(x[0], *LOG_BOUNDS["k_friction"]))
    h = 10 ** float(np.clip(x[1], *LOG_BOUNDS["h"]))
    C_thermal = 10 ** float(np.clip(x[2], *LOG_BOUNDS["C_thermal"]))
    k_wear = 10 ** float(np.clip(x[3], *LOG_BOUNDS["k_wear"]))
    return (
        TyreCalibration(
            k_friction=k_friction, h=h, C_thermal=C_thermal, k_wear=k_wear,
            measured=True,
        ),
        {"rmse_temp_C": best_T, "rmse_wear_pct": best_W, "rmse_pressure_psi": best_P},
    )


def _cook_frame_for_calibration(car, merged, track):
    """Pre-compute per-segment inputs for the calibration loss.

    Returns a dict with arrays needed by `_frame_rmse`:
      distances, speeds, radii (unsigned), radius_signs, labels (synthesised
      from gas/brake), measured_temp[wheel], measured_wear[wheel],
      measured_pressure[wheel], initial_state.
    """
    distances = np.asarray(merged["distance_m"], dtype=float)
    speeds = np.asarray(merged["speed_ms"], dtype=float)
    radii = np.asarray(merged["radius_m"], dtype=float)
    # Re-derive sign from track CSV's x,y for each sample distance.
    if getattr(track, "is_csv_backed", False):
        signs = derive_radius_sign(distances, track.csv_data)
    else:
        signs = np.zeros(len(distances), dtype=int)
    gas = np.asarray(merged["gas"], dtype=float)
    brake = np.asarray(merged["brake"], dtype=float)
    labels = np.where(brake > 0.5, "brake",
                      np.where((gas > 0.5) & (radii > 500.0), "accel",
                               np.where(gas > 0.5, "accel", "corner")))
    measured_T = {w: np.asarray(merged[f"tyreTemp{w}"], dtype=float) for w in WHEELS}
    measured_W = {w: np.asarray(merged[f"tyreWear{w}"], dtype=float) for w in WHEELS}
    measured_P = {w: np.asarray(merged[f"wheelsPressure{w}"], dtype=float) for w in WHEELS}

    # Initial state at sample 0 from measurements (warm tyres).
    first_T = {w: float(measured_T[w][0]) for w in WHEELS}
    first_W = {w: float(measured_W[w][0]) for w in WHEELS}
    first_P = {w: float(measured_P[w][0]) for w in WHEELS}
    avg_T = float(np.nanmean(list(first_T.values())))
    initial = TyreState(
        temp_C=first_T,
        wear_pct=first_W,
        pressure_psi=first_P,
        pressure_cold_psi=dict(first_P),
        T_cold_K=avg_T + 273.15,
        ambient_temp_C=avg_T,
        cumulative_slip_energy_J={w: 0.0 for w in WHEELS},
    )

    return {
        "distances": distances,
        "speeds": speeds,
        "radii": radii,
        "radius_signs": signs,
        "labels": labels,
        "measured_T": measured_T,
        "measured_W": measured_W,
        "measured_P": measured_P,
        "initial_state": initial,
    }


def _frame_rmse(cooked, car, calib, model):
    """Simulate per-wheel state across one frame; return sum-squared errors.

    Returns `(sum_sq_T, sum_sq_W, sum_sq_P, n_samples)` summed across wheels
    and samples. Caller averages across frames to get RMSE.
    """
    state = cooked["initial_state"]
    # Walk: at each step, advance state with a synthesised SegmentInfo.
    distances = cooked["distances"]
    speeds = cooked["speeds"]
    radii = cooked["radii"]
    signs = cooked["radius_signs"]
    labels = cooked["labels"]
    measured_T = cooked["measured_T"]
    measured_W = cooked["measured_W"]
    measured_P = cooked["measured_P"]
    n = len(distances)
    sq_T = sq_W = sq_P = 0.0
    samples = 0
    # Reset state at the start of this frame (each frame is one lap).
    cur_state = TyreState(
        temp_C=dict(state.temp_C),
        wear_pct=dict(state.wear_pct),
        pressure_psi=dict(state.pressure_psi),
        pressure_cold_psi=dict(state.pressure_cold_psi),
        T_cold_K=state.T_cold_K,
        ambient_temp_C=state.ambient_temp_C,
        cumulative_slip_energy_J={w: 0.0 for w in WHEELS},
    )
    for i in range(n - 1):
        seg_len = float(distances[i + 1] - distances[i])
        if seg_len <= 0:
            continue
        v_mid = max(0.5 * (float(speeds[i]) + float(speeds[i + 1])), 1.0)
        seg = SegmentInfo(
            distance_m=float(distances[i]),
            segment_length_m=seg_len,
            v_ms=v_mid,
            radius_m=float(radii[i]),
            radius_sign=int(signs[i]),
            binding_label=str(labels[i]),
        )
        update_per_segment(cur_state, seg, car, calib, model)
        # Compare to measurement at i+1.
        for w in WHEELS:
            T_m = float(measured_T[w][i + 1])
            W_m = float(measured_W[w][i + 1])
            P_m = float(measured_P[w][i + 1])
            sq_T += (cur_state.temp_C[w] - T_m) ** 2
            sq_W += (cur_state.wear_pct[w] - W_m) ** 2
            sq_P += (cur_state.pressure_psi[w] - P_m) ** 2
            samples += 1
    return sq_T, sq_W, sq_P, samples


def _normalise_frames(merged_frames):
    """Accept either a single merged dict or a list of merged dicts."""
    if isinstance(merged_frames, dict):
        return [merged_frames]
    return list(merged_frames)
