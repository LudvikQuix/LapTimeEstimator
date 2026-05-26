"""Single-shot longitudinal DP plan over the Pacejka envelope (spec §23.10.6).

Phase 4.2 of the v3.1 controller upgrade. Replaces the v2 friction-circle
plan (mu_v2 ~ 1.20) — which is geometrically infeasible for the fitted
Pacejka tyre (D_lat ~ 1.03) and causes Phase 4.1 to abort with
OffTrackError at the Sprint A chicane — with a plan built against the
*measured* Pacejka envelope. The controller's lookup machinery is
untouched: the output is a ``(distances, speeds)`` pair shaped exactly
like the v2 plan it replaces.

Structurally identical to v2's 3-pass solver
(:func:`simulator._three_pass`):

1. Backward 1 — per-point max corner speed from ``v_max[i] = sqrt(D_lat * g / |kappa_i|)``.
2. Backward 2 — brake-feasibility walk i=N-1 -> 0 with combined-slip-aware
   ``a_brake`` available given the lateral demand at ``v_max[i+1]``.
3. Forward — throttle-feasibility walk i=0 -> N-1 with combined-slip-aware
   ``a_throttle`` available given the lateral demand at ``v_plan[i]``.

A documented small ``safety_margin`` is applied on top. Phase 5.0.1
(spec §23.2-5.0.1.5) bumps the default from 0.97 -> 0.94 — the 3 %
headroom that pre-Phase-5.0.1 used was geometrically tight at the
Sprint A chicane (BMW 1M at the Pacejka grip limit, 5 % apex margin
on the DP plan) and the MPC controller had no feasibility room
above-and-beyond the plan. 6 % headroom inflates Sprint A lap time
by ~1 s on the plan side while giving the controller measurable
feasibility margin at the chicane apex; the trade-off is documented
in §23.2-5.0.1.5. Pre-Phase-5.0.1 callers can pin the historical
0.97 by passing ``safety_margin=0.97`` explicitly (CLI override on
``lap.py --dp-safety-margin``).

Per spec §23.10.6.2 the planner uses **static Fz** (no weight transfer).
Transients are the simulator's job; the planner produces a reference the
controller can track.

Phase 5.0.2 (chicane fallback): an additional ``chicane_config`` knob
applies a localised speed cap on tight-curvature segments
(default ``radius<60 m`` -> ``v_corner *= 0.80`` with a 5-segment
ramp-in / ramp-out). The cap is applied *between* Pass 1 and Pass 2,
so the backward / forward feasibility sweeps walk a smooth valley
around the chicane rather than a step discontinuity. See
:mod:`chicane_safety` and the architecture doc at
``docs/architecture-slip-model-phase5_0_2-chicane-fallback.md``.

v3 longitudinal-physics-fix follow-up (2026-05-23): the forward /
backward feasibility passes now consume the same six terms the v3
slip plant carries -- twin-turbo curve, ``DRIVETRAIN_EFFICIENCY``
(``η=0.87``) on engine traction, engine-brake coast drag (linear
in RPM, see ``[COAST_REF]``), gravity along ``gradient_pct``,
rolling resistance, and effective chassis mass
``m_eff = m + I_engine * (gear*final)^2 / r_drive^2``. The friction
envelope at corner apex (Pass 1) is untouched -- lateral physics is
the same. The *approach* and *exit* speeds change because the
longitudinal headroom now matches the plant. Without this fix the
planner asks the controllers for speeds the corrected plant cannot
hold; see ``docs/architecture-v3-longitudinal-physics-fix.md`` for
the regression history.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .chicane_safety import (
    ChicaneSafetyConfig,
    ChicaneSafetyReport,
    apply_chicane_cap,
)
from .vehicle import DRIVETRAIN_EFFICIENCY

if TYPE_CHECKING:
    from ..car import Car
    from ..track import Track
    from .vehicle import PacejkaCalibration


G = 9.81
V_MIN = 5.0  # m/s — floor mirrors v2 _three_pass.
KAPPA_MIN = 1e-6  # rad/m — straight-line cap to avoid div-by-zero.


@dataclass(frozen=True)
class LongitudinalPlan:
    """Public output of :func:`plan_longitudinal`.

    Attributes
    ----------
    distances : np.ndarray
        Per-point distance along the ideal line (m, monotonic).
    speeds : np.ndarray
        Per-point reference speed (m/s, ``safety_margin``-scaled).
    chicane_report : ChicaneSafetyReport | None
        Diagnostics from the Phase 5.0.2 chicane-safety cap. ``None``
        when the cap is disabled.
    """

    distances: np.ndarray
    speeds: np.ndarray
    chicane_report: ChicaneSafetyReport | None = None


def plan_longitudinal(
    car: "Car",
    track: "Track",
    calib: "PacejkaCalibration",
    *,
    safety_margin: float = 0.94,
    chicane_config: ChicaneSafetyConfig | None = None,
    log_chicane: bool = True,
) -> LongitudinalPlan:
    """Solve a minimum-time longitudinal plan against the Pacejka envelope.

    Single forward-backward DP over the existing track CSV ideal-line
    samples — same algorithm as v2's :func:`simulator._three_pass`, applied
    to the fitted Pacejka grip surface instead of v2's scalar friction
    circle. Per spec §23.10.6 the planner uses static Fz and the *front*
    axle's ``D_lat`` / ``D_long`` (the binding axle on the BMW 1M setup
    and the one the controller measures slip against). Weight transfer is
    deferred to the simulator.

    Parameters
    ----------
    car : Car
        Parsed AC car. Provides ``total_mass``, top-speed cap, aero drag.
    track : Track
        CSV-backed track; the ideal-line samples come from
        ``track.csv_data['distance_m']`` and ``radius_m``.
    calib : PacejkaCalibration
        Per-axle Magic-Formula coefficients + ellipse exponent. The front
        axle's ``D`` peaks drive the grip envelope.
    safety_margin : float, optional
        Multiplied into the final speeds. Default 0.94 (Phase 5.0.1,
        spec §23.2-5.0.1.5) — 6 % headroom for transient overshoot,
        loosened from 0.97 to give the MPC measurable feasibility margin
        at the Sprint A chicane. Pre-Phase-5.0.1 callers wanting the
        historical 0.97 plan can pass it explicitly (CLI:
        ``lap.py --dp-safety-margin 0.97``).
    chicane_config : ChicaneSafetyConfig | None, optional
        Phase 5.0.2 chicane-safety cap (see
        :mod:`chicane_safety`). When ``None`` (default) a
        :class:`ChicaneSafetyConfig` with stock defaults is used
        (``radius_thresh_m=60.0``, ``safety_mult=0.80``). Pass an
        explicit config (e.g. ``safety_mult=1.0``) to disable.
    log_chicane : bool, optional
        Print the chicane-safety summary line at construction. Default
        True. Set False for tests / quiet bulk runs.

    Returns
    -------
    LongitudinalPlan
    """
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("plan_longitudinal requires a CSV-backed track.")

    distances, kappa, grade_sin = _sample_line(track)
    D_lat = float(calib.front.lateral.D)
    D_long = float(calib.front.longitudinal.D)
    ellipse_n = float(calib.ellipse_exponent)
    v_top = float(car.top_speed())
    # AC load-sensitivity knobs (per-axle FZ0 + per-direction LS_EXP*).
    # Defaults preserve linear-load behaviour. We carry both front and
    # rear values so the brake-pass headroom (which sees an Fz_total that
    # includes the rear axle) can use each axle's own reference load.
    fz0_f = getattr(calib.front, "Fz0", None)
    fz0_r = getattr(calib.rear, "Fz0", None)
    ls_lat_f = getattr(calib.front, "ls_exp_lat", None)
    ls_long_f = getattr(calib.front, "ls_exp_long", None)
    ls_long_r = getattr(calib.rear, "ls_exp_long", None)

    # Pass 1 — per-point max corner speed from the lateral envelope.
    # v_max[i] = sqrt(D_lat_eff * g / |kappa_i|), with the AC load-
    # sensitivity factor evaluated at the *static* front-axle load (which
    # is FZ0 by construction, so the factor equals 1.0 at low speed and
    # drifts only with aero downforce). The chicane-apex grip win from
    # the inner-vs-outer wheel asymmetry shows up in the per-wheel truth
    # model, not in the per-axle planner — see arch doc for the
    # planner/plant accounting split.
    kappa_safe = np.maximum(np.abs(kappa), KAPPA_MIN)
    v_corner = np.sqrt(D_lat * G / kappa_safe)
    v_corner = np.clip(v_corner, V_MIN, v_top)

    # Phase 5.0.2 — chicane-safety cap (spec §23.2-5.0.2). Apply BEFORE
    # the backward / forward sweeps so the brake-feasibility walk into
    # the chicane and the throttle-feasibility walk out of it both see
    # the ramped cap as a smooth valley rather than a step. This is a
    # planner-side, NOT a controller-side, fix: the goal is to give any
    # reasonable controller enough margin to survive the Sprint A
    # chicane's ~27 m apex, where combined-slip leaves the controller
    # almost no longitudinal headroom for transient correction.
    cfg = chicane_config if chicane_config is not None else ChicaneSafetyConfig()
    v_corner, chicane_report = apply_chicane_cap(distances, v_corner, kappa_safe, cfg)
    if log_chicane:
        print(chicane_report.fmt_line())

    # Pass 2 — backward brake-feasibility walk. v3 long-physics fix
    # (2026-05-23): ``_available_long_decel`` now also accounts for engine
    # brake (coast drag, linear in RPM) and the body-x gravity coefficient
    # at the *next* sample (uphill = helps brake, downhill = hurts).
    v_plan = v_corner.copy()
    n = len(distances)
    for i in range(n - 2, -1, -1):
        v_next = v_plan[i + 1]
        ds = float(distances[i + 1] - distances[i])
        if ds <= 0.0:
            continue
        # Lateral demand a_y at the *next* point governs the brake headroom
        # we have walking back into it (we need to be slow enough that we
        # can carry only that much lateral g and still brake).
        a_y = (v_next ** 2) * float(kappa_safe[i + 1])
        a_brake = _available_long_decel(
            car, v_next, a_y, D_lat, D_long, ellipse_n,
            grade_sin=float(grade_sin[i + 1]),
            fz0_front=fz0_f, fz0_rear=fz0_r,
            ls_exp_lat=ls_lat_f, ls_exp_long_front=ls_long_f,
            ls_exp_long_rear=ls_long_r,
        )
        v_brake = float(np.sqrt(max(v_next ** 2 + 2.0 * a_brake * ds, V_MIN ** 2)))
        v_plan[i] = min(v_plan[i], v_brake)

    # Pass 3 — forward throttle-feasibility walk. v3 long-physics fix
    # (2026-05-23): ``_available_long_accel`` now applies
    # ``DRIVETRAIN_EFFICIENCY``, the twin-turbo curve via
    # ``car.engine_torque`` -> ``car.turbo_boost_at_rpm``, subtracts the
    # body-x gravity coefficient at this sample, and divides by
    # ``m_eff = m + I_engine * (gear*final)^2 / r_drive^2`` so the
    # acceleration estimate matches what the v3 ODE produces.
    for i in range(n - 1):
        v_now = v_plan[i]
        ds = float(distances[i + 1] - distances[i])
        if ds <= 0.0:
            continue
        # Lateral demand here governs how much longitudinal headroom we
        # have for throttle. At the apex (v_corner binding), this drives
        # a_throttle -> 0, exactly as it should.
        a_y = (v_now ** 2) * float(kappa_safe[i])
        a_thr = _available_long_accel(
            car, v_now, a_y, D_lat, D_long, ellipse_n,
            grade_sin=float(grade_sin[i]),
            fz0_front=fz0_f, fz0_rear=fz0_r,
            ls_exp_lat=ls_lat_f, ls_exp_long_front=ls_long_f,
            ls_exp_long_rear=ls_long_r,
        )
        v_thr = float(np.sqrt(max(v_now ** 2 + 2.0 * a_thr * ds, V_MIN ** 2)))
        v_plan[i + 1] = min(v_plan[i + 1], v_thr)

    speeds = np.clip(v_plan * float(safety_margin), V_MIN, v_top)
    return LongitudinalPlan(
        distances=distances.copy(),
        speeds=speeds,
        chicane_report=chicane_report,
    )


# ---------------------------------------------------------------------------
# Helpers — friction-ellipse decomposition + axle load.
# ---------------------------------------------------------------------------


def _sample_line(track: "Track") -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return ``(distances, kappa, grade_sin)`` on the existing CSV samples.

    ``kappa`` is the curvature ``1 / |radius_m|`` per-sample. Straight
    sections are clamped to ``KAPPA_MIN`` to keep ``v_max`` finite at the
    car's top speed. We deliberately consume the CSV's own samples rather
    than resampling on a uniform grid — spec §23.10.6.2 step 1 calls for
    "the existing track CSV ideal-line distance samples (~700 pts on
    Sprint A)" and matches the controller's interp lookup grid.

    ``grade_sin = sin(atan(gradient_pct/100))`` is the body-x gravity
    coefficient at every track sample, computed the same way the v3 ODE
    builds its per-step ``gravity_a_x = -g * grade_sin`` (see
    :func:`solver._build_grade_lookup`). Positive grade = uphill =
    decelerating. Returns an all-zero array if the CSV has no
    ``gradient_pct`` column (legacy tracks; planner treats as flat).
    """
    d = np.asarray(track.csv_data["distance_m"], dtype=float)
    r = np.asarray(track.csv_data["radius_m"], dtype=float)
    kappa = 1.0 / np.clip(np.abs(r), 1e-3, None)
    if "gradient_pct" in track.csv_data:
        grad_pct = np.asarray(track.csv_data["gradient_pct"], dtype=float)
        grade_sin = np.sin(np.arctan(grad_pct / 100.0))
    else:
        grade_sin = np.zeros_like(d)
    return d, kappa, grade_sin


def _static_axle_load(car: "Car", v_x: float, axle: str = "front") -> float:
    """Static per-axle normal load with aero downforce (single axle, two wheels).

    Per spec §23.10.6.2 the planner uses static Fz — no longitudinal /
    lateral weight transfer. Aero downforce is split front/rear by the
    static CG distribution.
    """
    m = float(car.total_mass)
    W = m * G
    if axle == "front":
        share = 1.0 - float(car.cg_front)
    else:
        share = float(car.cg_front)
    F_down = float(car.downforce(max(v_x, 0.0)))
    return max(W * share + F_down * share, 100.0)


def _engine_force_at_v(car: "Car", v_x: float) -> float:
    """Net engine drive force at the rear axle, η-corrected (full throttle).

    Mirrors the v3 plant's :func:`vehicle._engine_torque_at_wheel` for the
    full-throttle / no-coast case the planner needs (Pass 3 assumes the
    driver is on the throttle). Picks the optimal gear at ``v_x``, reads
    engine torque off the LUT (which includes ``turbo_boost_at_rpm``),
    multiplies through gear * final ratio, applies
    ``DRIVETRAIN_EFFICIENCY``, then converts torque -> force via the
    drive-axle radius.

    Used in place of ``car.max_traction_force`` because the latter does
    NOT apply η — it returns the *uncorrected* wheel force the planner
    assumed pre-fix.
    """
    v = max(float(v_x), 1.0)
    gear = car.optimal_gear(v)
    rpm = car.rpm_from_speed(v, gear)
    rpm = float(np.clip(rpm, car.power_rpm[0], car.rev_limit))
    drive_t = float(max(0.0, car.wheel_torque(rpm, gear)))
    drive_t *= DRIVETRAIN_EFFICIENCY
    drive_type = str(getattr(car, "drive_type", "RWD") or "RWD").upper()
    if drive_type == "FWD":
        r_drive = float(car.tyre_radius_f)
    elif drive_type == "AWD":
        r_drive = 0.5 * (float(car.tyre_radius_f) + float(car.tyre_radius_r))
    else:
        r_drive = float(car.tyre_radius_r)
    return drive_t / max(r_drive, 1e-6)


def _engine_brake_force_at_v(car: "Car", v_x: float) -> float:
    """Magnitude of engine coast drag at the wheel (positive = decel).

    Mirrors the coast path in :func:`vehicle._engine_torque_at_wheel` with
    throttle = 0 (full lift), so the full COAST_REF curve applies:

        T_coast(rpm) = - coast_ref_torque * (rpm / coast_ref_rpm)
        T_wheel_coast = T_coast * gear * final * η
        F_wheel = |T_wheel_coast| / r_drive

    For the BMW 1M (75 Nm @ 7200 RPM, 4th gear at ~50 m/s) this is
    ~150-200 N (~0.1 g of decel). Falls back to 0 if the ini did not
    declare ``[COAST_REF]``.
    """
    v = max(float(v_x), 1.0)
    gear = car.optimal_gear(v)
    rpm = car.rpm_from_speed(v, gear)
    rpm = float(np.clip(rpm, car.power_rpm[0], car.rev_limit))
    coast_ref_rpm = float(getattr(car, "coast_ref_rpm", 7000.0))
    coast_ref_torque = float(getattr(car, "coast_ref_torque", 0.0))
    if coast_ref_rpm <= 1.0 or coast_ref_torque <= 0.0:
        return 0.0
    engine_brake_t = coast_ref_torque * (rpm / coast_ref_rpm)
    coast_ratio = float(car.gear_ratios[gear]) * float(car.final_ratio)
    wheel_t = engine_brake_t * coast_ratio * DRIVETRAIN_EFFICIENCY
    drive_type = str(getattr(car, "drive_type", "RWD") or "RWD").upper()
    if drive_type == "FWD":
        r_drive = float(car.tyre_radius_f)
    elif drive_type == "AWD":
        r_drive = 0.5 * (float(car.tyre_radius_f) + float(car.tyre_radius_r))
    else:
        r_drive = float(car.tyre_radius_r)
    return wheel_t / max(r_drive, 1e-6)


def _m_eff_at_v(car: "Car", v_x: float) -> float:
    """Effective longitudinal mass at this speed.

    Mirrors :func:`vehicle.compute_derivatives` step 10:
    ``m_eff = m + I_engine * (gear*final)^2 / r_drive^2``. Picks the same
    optimal gear the plant would. In 1st (gear=4.11) this adds ~120 kg
    of equivalent mass for the BMW 1M; in 5th-6th ~5 kg.
    """
    v = max(float(v_x), 1.0)
    gear = car.optimal_gear(v)
    gear_ratio = float(car.gear_ratios[gear]) * float(car.final_ratio)
    drive_type = str(getattr(car, "drive_type", "RWD") or "RWD").upper()
    if drive_type == "FWD":
        r_drive = float(car.tyre_radius_f)
    elif drive_type == "AWD":
        r_drive = 0.5 * (float(car.tyre_radius_f) + float(car.tyre_radius_r))
    else:
        r_drive = float(car.tyre_radius_r)
    I_engine = float(getattr(car, "engine_inertia", 0.0))
    return float(car.total_mass) + I_engine * (gear_ratio ** 2) / max(r_drive ** 2, 1e-6)


def _ls_factor(Fz: float, Fz0: float | None, ls_exp: float | None) -> float:
    """Return ``(Fz/Fz0)**(ls_exp - 1)`` or ``1.0`` if either knob is unset.

    Local scalar mirror of :func:`pacejka._load_sens_factor` — the planner
    operates on scalars per-stage so a pure-Python helper is cheaper than
    routing every call through NumPy. Defaults preserve linear-load
    behaviour exactly.
    """
    if Fz0 is None or ls_exp is None or Fz0 <= 0.0:
        return 1.0
    if abs(float(ls_exp) - 1.0) < 1e-12:
        return 1.0
    return float(max(Fz, 1.0) / float(Fz0)) ** (float(ls_exp) - 1.0)


def _available_long_accel(
    car: "Car",
    v_x: float,
    a_y: float,
    D_lat: float,
    D_long: float,
    ellipse_n: float,
    *,
    grade_sin: float = 0.0,
    fz0_front: float | None = None,
    fz0_rear: float | None = None,
    ls_exp_lat: float | None = None,
    ls_exp_long_front: float | None = None,
    ls_exp_long_rear: float | None = None,
) -> float:
    """Combined-slip-aware longitudinal acceleration available at this point.

    The friction ellipse (per the fitted ellipse exponent) is what closes
    the model: with lateral demand ``a_y`` we have only ``a_x_avail`` long
    headroom. Engine traction and aero drag both clamp the result —
    drive-axle Fx cannot exceed the ellipse, and the engine cannot deliver
    more torque than its peak.

    v3 long-physics fix (2026-05-23): the engine traction term now uses
    :func:`_engine_force_at_v` (twin-turbo curve + ``DRIVETRAIN_EFFICIENCY``,
    same path as the v3 plant), the net body-x force is divided by
    :func:`_m_eff_at_v` (engine-side inertia reflected through the gearbox),
    and the body-x gravity component
    ``F_grav_x = - m * g * grade_sin`` (positive grade = uphill =
    decelerating) is subtracted. ``grade_sin`` defaults to 0 so existing
    flat-track callers are unaffected.
    """
    # Friction-ellipse headroom: (a_x / (D_long_eff*g))^n + (a_y / (D_lat_eff*g))^n <= 1.
    # AC ``LS_EXPY`` / ``LS_EXPX`` make the peak ``D(Fz)`` sub-linear in Fz
    # via ``D_eff = D_ref · (Fz_wheel/FZ0)**(ls_exp - 1)``. The JSON keeps
    # ``FZ0`` as the *per-wheel* reference load (matching AC's tyres.ini
    # convention); the planner works in per-axle Fz, so divide by 2 to
    # convert to per-wheel before computing the factor.
    Fz_front_static = _static_axle_load(car, v_x, axle="front")
    Fz_rear_static = _static_axle_load(car, v_x, axle="rear")
    D_lat_eff = D_lat * _ls_factor(0.5 * Fz_front_static, fz0_front, ls_exp_lat)
    drive_axle = "rear" if (car.drive_type or "RWD").upper() == "RWD" else "front"
    if drive_axle == "rear":
        Fz_drive = Fz_rear_static
        ls_drive = ls_exp_long_rear
        fz0_drive = fz0_rear
    else:
        Fz_drive = Fz_front_static
        ls_drive = ls_exp_long_front
        fz0_drive = fz0_front
    D_long_drive_eff = D_long * _ls_factor(0.5 * Fz_drive, fz0_drive, ls_drive)
    a_y_norm = abs(a_y) / max(D_lat_eff * G, 1e-6)
    a_y_norm = min(a_y_norm, 1.0 - 1e-6)
    n = float(ellipse_n)
    headroom = (1.0 - a_y_norm ** n) ** (1.0 / n)
    # Grip cap on the drive axle. `_static_axle_load` already returns the
    # per-axle (two-wheel) normal sum, so no *2.0 fudge.
    F_grip = D_long_drive_eff * Fz_drive * headroom
    # Engine traction cap, η-corrected. Clamped by the drive-axle Fx envelope.
    F_engine = _engine_force_at_v(car, v_x)
    F_drive = min(F_grip, F_engine)
    # Resisting forces (all body-x):
    F_drag = float(car.drag_force(v_x))
    F_roll = float(car.rolling_resistance(v_x))
    m = float(car.total_mass)
    F_grav = m * G * float(grade_sin)  # positive grade = uphill = resists motion
    # m_eff for the divide -- engine inertia reflected through the gearbox.
    m_eff = _m_eff_at_v(car, v_x)
    a_x_avail = (F_drive - F_drag - F_roll - F_grav) / m_eff
    return max(a_x_avail, 0.1)


def _available_long_decel(
    car: "Car",
    v_x: float,
    a_y: float,
    D_lat: float,
    D_long: float,
    ellipse_n: float,
    *,
    grade_sin: float = 0.0,
    fz0_front: float | None = None,
    fz0_rear: float | None = None,
    ls_exp_lat: float | None = None,
    ls_exp_long_front: float | None = None,
    ls_exp_long_rear: float | None = None,
) -> float:
    """Combined-slip-aware longitudinal braking decel available at this point.

    Same friction-ellipse decomposition as :func:`_available_long_accel`,
    but on all four wheels (brakes act on every axle, unlike traction on
    RWD/FWD). Aero drag *helps* braking and is added to the result.

    v3 long-physics fix (2026-05-23): also adds engine-brake coast drag
    (``_engine_brake_force_at_v``) -- ~0.1 g for the BMW 1M -- and the
    body-x gravity coefficient. On an uphill segment gravity helps the
    car decelerate; on a downhill segment it works against the brakes.
    The result is divided by ``m_eff`` so the deceleration matches the
    plant's response to the same net force.
    """
    Fz_front = _static_axle_load(car, v_x, "front")
    Fz_rear = _static_axle_load(car, v_x, "rear")
    # FZ0 in the JSON is per-wheel; halve axle Fz to convert.
    D_lat_eff = D_lat * _ls_factor(0.5 * Fz_front, fz0_front, ls_exp_lat)
    a_y_norm = abs(a_y) / max(D_lat_eff * G, 1e-6)
    a_y_norm = min(a_y_norm, 1.0 - 1e-6)
    n = float(ellipse_n)
    headroom = (1.0 - a_y_norm ** n) ** (1.0 / n)
    # Braking can use all four wheels' longitudinal envelope, but each
    # axle's peak D scales with its own (Fz_wheel/FZ0)**(LS_EXPX-1) —
    # under forward braking the front wheels are loaded (factor < 1)
    # and the rear unloaded (factor > 1). The two terms partially
    # cancel; net effect at static Fz is exactly the linear-D case.
    F_brake_grip = (
        D_long * _ls_factor(0.5 * Fz_front, fz0_front, ls_exp_long_front) * Fz_front
        + D_long * _ls_factor(0.5 * Fz_rear, fz0_rear, ls_exp_long_rear) * Fz_rear
    ) * headroom
    # Assisting / opposing terms (all body-x, magnitudes positive when they
    # assist braking).
    F_drag = float(car.drag_force(v_x))
    F_roll = float(car.rolling_resistance(v_x))
    F_engine_brake = _engine_brake_force_at_v(car, v_x)
    m = float(car.total_mass)
    F_grav = m * G * float(grade_sin)  # uphill (grade>0) helps brake; downhill hurts
    m_eff = _m_eff_at_v(car, v_x)
    a_brake = (F_brake_grip + F_drag + F_roll + F_engine_brake + F_grav) / m_eff
    return max(a_brake, 0.1)
