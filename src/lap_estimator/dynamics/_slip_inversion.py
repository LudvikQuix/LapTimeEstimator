"""Stage A + B helpers for the Pacejka fit (spec §23.7.2).

Stage A — invert per-wheel slip-angle (alpha) and slip-ratio (kappa) from
chassis kinematics + per-wheel geometry.

Stage B — invert per-wheel (Fx, Fy) from chassis dynamics (accG + yaw
moment) plus drivetrain priors. See the architecture doc
``docs/architecture-slip-model-phase2.md`` for the design rationale; in
short, axle-level Fy is closed-form from F_y_chassis + M_z, then split
within axle by Fz fraction.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np

from ..car import Car
from ._fit_helpers import central_diff, low_pass_iir

WHEELS = ("FL", "FR", "RL", "RR")


# ---------------------------------------------------------------------------
# Car geometry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CarGeom:
    """Geometry pack needed by Stages A/B."""

    mass: float           # kg
    wheelbase: float      # m
    track_front: float    # m
    track_rear: float     # m
    cg_front: float       # fraction; 0.5 = perfectly balanced
    tyre_radius_f: float  # m
    tyre_radius_r: float  # m
    drive_type: str       # "RWD" / "FWD" / "AWD"
    brake_front_share: float
    I_zz: float           # kg*m^2; from estimate if not given


def build_car_geom(car_data_dir) -> CarGeom:
    """Read car/suspensions/drivetrain/brakes/tyres → CarGeom."""
    car = Car(str(car_data_dir))
    # AC suspensions.ini has separate FRONT.TRACK and REAR.TRACK; the
    # Car wrapper currently doesn't expose them as separate attributes,
    # so re-parse the ini directly.
    import configparser as _cp
    susp = _cp.ConfigParser(inline_comment_prefixes=(";",), strict=False)
    susp.read(os.path.join(str(car_data_dir), "suspensions.ini"))
    track_f = float(susp["FRONT"].get("TRACK", "1.5"))
    track_r = float(susp["REAR"].get("TRACK", "1.5"))
    # I_zz estimate: m * ((wheelbase/2)^2) * 1.2 per Phase 2 brief.
    I_zz_est = car.total_mass * (car.wheelbase / 2.0) ** 2 * 1.2
    return CarGeom(
        mass=car.total_mass,
        wheelbase=car.wheelbase,
        track_front=track_f,
        track_rear=track_r,
        cg_front=car.cg_front,
        tyre_radius_f=car.tyre_radius_f,
        tyre_radius_r=car.tyre_radius_r,
        drive_type=car.drive_type,
        brake_front_share=car.brake_front_share,
        I_zz=I_zz_est,
    )


def wheel_offsets(geom: CarGeom) -> dict[str, tuple[float, float]]:
    """Wheel (x, y) position relative to chassis CG in body frame.

    x is longitudinal (+ forward), y is lateral (+ left).
    """
    a = geom.wheelbase * (1.0 - geom.cg_front)  # CG-to-front distance
    b = geom.wheelbase * geom.cg_front          # CG-to-rear distance
    half_f = geom.track_front / 2.0
    half_r = geom.track_rear / 2.0
    return {
        "FL": (+a, +half_f),
        "FR": (+a, -half_f),
        "RL": (-b, +half_r),
        "RR": (-b, -half_r),
    }


# ---------------------------------------------------------------------------
# Common helpers
# ---------------------------------------------------------------------------


def resolve_field(lap: dict, *aliases: str) -> np.ndarray | None:
    """Return the first array in ``lap`` matching any alias, else None."""
    for a in aliases:
        if a in lap:
            return np.asarray(lap[a], dtype=float)
    return None


def has_v3_channels(lap: dict) -> bool:
    """Return True iff ``lap`` carries every channel Stages A+B need."""
    needed = (
        "wheelLoadFL", "wheelLoadFR", "wheelLoadRL", "wheelLoadRR",
        "wheelAngularSpeedFL", "wheelAngularSpeedFR",
        "wheelAngularSpeedRL", "wheelAngularSpeedRR",
        "localVelocity_x", "localVelocity_z",
        "accG_x", "accG_z",
    )
    return all(c in lap for c in needed)


def estimate_dt(lap: dict) -> float:
    """Median timestep in seconds from a lap dict."""
    t = lap["timestamp_ms"] / 1000.0
    if len(t) < 2:
        return 0.01
    dt = np.median(np.diff(t))
    return max(float(dt), 1e-4)


def _wheel_steer(lap: dict, geom: CarGeom) -> dict[str, np.ndarray]:
    """Per-wheel steer angle (rad) in chassis body frame.

    Preferred source: ``tyreContactHeading{w}_x/y`` (a body-frame vector;
    angle = atan2(y, x)). Falls back to ``steerAngle / steer_ratio`` for
    the front wheels and zero for rear.

    Phase 2 simplification: hard-coded steering ratio 13:1 when only
    ``steerAngle`` is logged. v3.1 reads STEER_LOCK + linkage geometry.
    """
    n = len(lap["timestamp_ms"])
    zeros = np.zeros(n, dtype=float)
    # NOTE: tyreContactHeading{w}_* is in WORLD frame (sample shows
    # `_z = -0.9999` for a car heading along -Z world axis). Using it as
    # body-frame steer angle gave garbage. Fall through to steerAngle/13.
    # v3.1: implement proper world->body transform via `heading` channel.
    steer = lap.get("steerAngle")
    if steer is None:
        return {w: zeros for w in WHEELS}
    # AC's `steerAngle` is normalized [-1, +1] of STEER_LOCK, NOT radians.
    # Verified by Ackermann comparison vs estimated corner radius (2026-05-17):
    # interpretation D (degrees) gave scale 0.04, B (radians/15) gave 0.14,
    # but normalized × STEER_LOCK / STEER_RATIO gave |scale| = 1.10. So:
    #   delta_wheel_rad = -steerAngle × STEER_LOCK_RAD / STEER_RATIO
    # Sign-flip: AC's steerAngle is opposite-signed to yaw and lateral G.
    import math
    STEER_LOCK_RAD = math.radians(450.0)   # car.ini [CONTROLS] STEER_LOCK = 450
    steer_ratio = 15.0                     # car.ini [CONTROLS] STEER_RATIO = 15
    delta_avg = -steer * STEER_LOCK_RAD / steer_ratio
    return {
        "FL": delta_avg, "FR": delta_avg,
        "RL": zeros, "RR": zeros,
    }


# ---------------------------------------------------------------------------
# Stage A — slip-angle / slip-ratio inversion
# ---------------------------------------------------------------------------


def stage_a(lap: dict, geom: CarGeom) -> dict[str, dict[str, np.ndarray]]:
    """Invert per-wheel alpha (rad) and kappa (dimensionless) from kinematics.

    Returns ``{wheel: {"alpha": arr, "kappa": arr, "v_long": arr}}``.
    """
    # AC body-frame axes: X = lateral, Y = vertical, Z = longitudinal.
    v_x = lap["localVelocity_z"]   # longitudinal
    v_y = lap["localVelocity_x"]   # lateral
    omega = resolve_field(lap, "localAngularVel_y", "worldRotationYaw")
    if omega is None:
        t = lap["timestamp_ms"] / 1000.0
        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
        if "worldPositionYaw" in lap:
            omega = central_diff(lap["worldPositionYaw"], dt)
        else:
            omega = np.zeros_like(v_x)
    omega = low_pass_iir(omega, dt_s=estimate_dt(lap), cutoff_hz=10.0)
    steer_per_wheel = _wheel_steer(lap, geom)
    offsets = wheel_offsets(geom)
    out: dict[str, dict[str, np.ndarray]] = {}
    for w in WHEELS:
        ox, oy = offsets[w]
        v_w_x = v_x - omega * oy
        v_w_y = v_y + omega * ox
        delta = steer_per_wheel[w]
        cos_d = np.cos(delta)
        sin_d = np.sin(delta)
        v_long = v_w_x * cos_d + v_w_y * sin_d
        v_lat = -v_w_x * sin_d + v_w_y * cos_d
        alpha = np.arctan2(-v_lat, np.maximum(np.abs(v_long), 0.5))
        alpha = np.clip(alpha, -np.deg2rad(15.0), np.deg2rad(15.0))
        R_tyre = geom.tyre_radius_f if w[0] == "F" else geom.tyre_radius_r
        omega_w = lap[f"wheelAngularSpeed{w}"]
        wheel_linear_v = omega_w * R_tyre
        denom = np.maximum(np.abs(v_long), 1.0)
        kappa = (wheel_linear_v - v_long) / denom
        kappa = np.clip(kappa, -0.3, 0.3)
        out[w] = {"alpha": alpha, "kappa": kappa, "v_long": v_long}
    return out


# ---------------------------------------------------------------------------
# Stage B — per-wheel force decomposition
# ---------------------------------------------------------------------------


def stage_b(lap: dict,
            geom: CarGeom,
            stage_a_out: dict[str, dict[str, np.ndarray]]) -> dict[str, dict[str, np.ndarray]]:
    """Invert per-wheel (Fx, Fy) from chassis dynamics + load distribution.

    Two-step strategy (per §23.7.2 + Phase 2 brief):

    1. **Axle-level Fy from chassis dynamics.** ``sum(Fy) = m a_y`` and
       ``Fy_front * a - Fy_rear * b = M_z`` is a fully determined 2x2
       system per timestep — solved analytically.
    2. **Within-axle split by Fz fraction.** Outer wheel takes a share
       proportional to its load.
    3. **Per-wheel Fx by drivetrain + brake share.** Acceleration goes
       through the drive axle (RWD/FWD/AWD); braking by ``FRONT_SHARE``.
       Within-axle split by Fz again.
    """
    del stage_a_out  # Stage B is independent of Stage A; param kept for symmetry.
    n = len(lap["timestamp_ms"])
    g = 9.81
    # AC body-frame axes: X = lateral, Y = vertical, Z = longitudinal.
    a_x = lap["accG_z"] * g   # longitudinal
    a_y = lap["accG_x"] * g   # lateral
    F_x_chassis = geom.mass * a_x
    F_y_chassis = geom.mass * a_y

    # Per-wheel Fy decomposition: split chassis F_y_chassis directly by the
    # MEASURED instantaneous Fz fraction per wheel. This skips the M_z-based
    # front/rear split that was causing a bimodal scatter in the front-axle
    # Pacejka cloud (the M_z = I_zz * omega_dot term is INERTIAL, not tyre
    # grip — during corner exit it flips Fy_front's sign even though the
    # tyres are still cornering. See scratch_diag.py output 2026-05-17.)
    Fz = {w: lap[f"wheelLoad{w}"] for w in WHEELS}
    Fz_sum = np.maximum(Fz["FL"] + Fz["FR"] + Fz["RL"] + Fz["RR"], 1.0)
    Fy = {w: F_y_chassis * Fz[w] / Fz_sum for w in WHEELS}

    brake = lap.get("brake", np.zeros(n))
    is_braking = brake > 0.05
    drive = geom.drive_type.upper()
    if drive == "FWD":
        drive_axle = {"FL": 1.0, "FR": 1.0, "RL": 0.0, "RR": 0.0}
    elif drive == "AWD":
        drive_axle = {"FL": 0.5, "FR": 0.5, "RL": 0.5, "RR": 0.5}
    else:
        drive_axle = {"FL": 0.0, "FR": 0.0, "RL": 1.0, "RR": 1.0}
    front_brake = geom.brake_front_share
    rear_brake = 1.0 - front_brake
    brake_axle = {
        "FL": front_brake, "FR": front_brake,
        "RL": rear_brake,  "RR": rear_brake,
    }
    Fx = {}
    for w in WHEELS:
        same_axle = ("FL", "FR") if w[0] == "F" else ("RL", "RR")
        axle_Fz = sum(Fz[ww] for ww in same_axle)
        load_frac = Fz[w] / np.maximum(axle_Fz, 1.0)
        drive_total = drive_axle[w] * F_x_chassis
        brake_total = brake_axle[w] * F_x_chassis
        Fx_drive_w = drive_total * load_frac
        Fx_brake_w = brake_total * load_frac
        Fx[w] = np.where(is_braking, Fx_brake_w, Fx_drive_w)

    out = {}
    for w in WHEELS:
        out[w] = {"Fx": Fx[w], "Fy": Fy[w], "Fz": Fz[w]}
    return out
