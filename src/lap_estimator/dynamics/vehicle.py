"""Vehicle state + per-step derivative computation (spec §23.3, §23.6).

10-element ``VehicleState`` and ``compute_derivatives(...)`` — the
right-hand-side of the ODE the solver integrates. Also packages the
quasi-static weight-transfer model, Ackermann steering geometry, and the
per-wheel slip-angle / slip-ratio kinematics.

State vector layout (10 variables, spec §23.6.1):

    state = [x, y, psi, v_x, v_y, omega_yaw, omega_FL, omega_FR, omega_RL, omega_RR]

Phase 3: implements §23.6.2 force model in full. Per-wheel ``TyreState`` is
held constant inside the lap (passed via ``tyre_state_snapshot``); thermal
evolution is layered on by the simulator across laps, not inside the ODE.

Numerical guards: all per-wheel longitudinal velocities are floored at 0.5
m/s for slip-angle / slip-ratio denominators (spec §23.6.5).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ._chassis_geometry import CarDynamics, load_car_dynamics
from .pacejka import PacejkaCoeffs, combined_friction_ellipse, pacejka_fx, pacejka_fy

if TYPE_CHECKING:
    from ..car import Car
    from ..tyre_state import Compound


__all__ = [
    "CarDynamics",
    "Controls",
    "PacejkaCalibration",
    "AxleCoeffs",
    "VehicleState",
    "compute_derivatives",
    "load_car_dynamics",
]


WHEELS = ("FL", "FR", "RL", "RR")
G = 9.81
RHO = 1.225  # air density kg/m^3
V_FLOOR = 0.5  # spec §23.6.2 step 4 — denominator floor for slip-ratio formula.

# v3 longitudinal-physics fix (2026-05-23): constant drivetrain efficiency.
# Empirical LS-fit against Tomas's QuixLake rich lap data (`.tmp/tomas_force_v2.csv`)
# yielded F_obs/F_v3_real_boost slope = 0.871 — i.e. v3 over-predicts by 12.9 % when
# turbo is correctly modelled. Constant across gears 2-4 (~3 % spread). So η = 0.87.
# v3.1 may swap for a per-gear LUT if higher-fidelity telemetry suggests it.
DRIVETRAIN_EFFICIENCY = 0.87


@dataclass
class VehicleState:
    """10-DOF planar chassis state (spec §23.6.1).

    All quantities in SI units. ``(x, y)`` is the chassis CG in the
    track frame, ``psi`` the chassis heading. ``(v_x, v_y)`` are body-frame
    velocity components; ``omega_yaw`` is the yaw rate. ``omega_FL..RR``
    are the four wheel angular speeds.
    """

    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    v_x: float = 0.0
    v_y: float = 0.0
    omega_yaw: float = 0.0
    omega_FL: float = 0.0
    omega_FR: float = 0.0
    omega_RL: float = 0.0
    omega_RR: float = 0.0

    def to_array(self) -> np.ndarray:
        """Return state as a length-10 NumPy array (for the ODE solver)."""
        return np.array(
            [
                self.x, self.y, self.psi,
                self.v_x, self.v_y, self.omega_yaw,
                self.omega_FL, self.omega_FR, self.omega_RL, self.omega_RR,
            ],
            dtype=float,
        )

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "VehicleState":
        """Build a ``VehicleState`` from a length-10 NumPy array."""
        return cls(
            x=float(arr[0]), y=float(arr[1]), psi=float(arr[2]),
            v_x=float(arr[3]), v_y=float(arr[4]), omega_yaw=float(arr[5]),
            omega_FL=float(arr[6]), omega_FR=float(arr[7]),
            omega_RL=float(arr[8]), omega_RR=float(arr[9]),
        )


@dataclass(frozen=True)
class Controls:
    """Driver-controller output: per-step actuator commands.

    ``steer_rad`` is the averaged front-axle steer angle in radians
    (Ackermann splitting happens inside :func:`compute_derivatives`).
    Throttle and brake are pedal fractions in [0, 1].
    """

    steer_rad: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


@dataclass(frozen=True)
class AxleCoeffs:
    """Pacejka coefficient pair (lateral + longitudinal) for one axle.

    Optional AC non-linear load knobs (``LS_EXPY`` / ``LS_EXPX``,
    ``FZ0``) come in as ``None`` by default, in which case the Magic
    Formula keeps its legacy linear-load form. When set, the per-axle
    static-axle reference load ``FZ0`` and the per-direction
    ``ls_exp_*`` exponents drive ``D(Fz) = D_ref · (Fz/FZ0)**(ls_exp-1)``.
    See :func:`pacejka._magic_formula` for the math.
    """
    lateral: PacejkaCoeffs
    longitudinal: PacejkaCoeffs
    # Reference vertical load for AC's load-sensitivity formula (per
    # axle, not per direction — AC keeps a single ``FZ0`` per axle).
    Fz0: float | None = None
    ls_exp_lat: float | None = None
    ls_exp_long: float | None = None


@dataclass(frozen=True)
class PacejkaCalibration:
    """Per-axle Magic-Formula coefficients + ellipse exponent (spec §23.5.1).

    AC load-sensitivity / post-peak-falloff knobs live on the
    :class:`AxleCoeffs` (per-axle ``FZ0`` / ``ls_exp_*``) and on this
    dataclass (``falloff_level`` is shared across axles, mirroring AC
    where the value is normally identical front/rear). All defaults
    keep the legacy linear-load behaviour.
    """
    front: AxleCoeffs
    rear: AxleCoeffs
    ellipse_exponent: float = 2.0
    falloff_level: float | None = None


def _ackermann_steer(steer_avg_rad: float, wheelbase: float, track_f: float) -> tuple[float, float]:
    """Per-front-wheel Ackermann steering angles.

    For small steer_avg, the linearised result is steer_avg ± 0 — for larger
    angles we use the geometric Ackermann formula. Sign convention: positive
    ``steer_avg`` = left turn; inner wheel = left = ``delta_FL``.
    """
    if abs(steer_avg_rad) < 1e-4:
        return steer_avg_rad, steer_avg_rad
    L = wheelbase
    t = track_f * 0.5
    # tan(delta_avg) defines the turn radius R = L / tan(delta_avg).
    R = L / np.tan(steer_avg_rad)
    # Inner wheel (smaller radius) and outer wheel (larger radius). For a
    # left turn (positive steer), the inner wheel is the left (FL).
    if steer_avg_rad > 0:  # left turn
        delta_inner = float(np.arctan(L / (R - t)))  # FL
        delta_outer = float(np.arctan(L / (R + t)))  # FR
        return delta_inner, delta_outer
    # right turn
    delta_outer = float(np.arctan(L / (R - t)))  # FL (outer)
    delta_inner = float(np.arctan(L / (R + t)))  # FR (inner)
    return delta_outer, delta_inner


def _wheel_offsets(dyn: CarDynamics) -> dict[str, tuple[float, float]]:
    """Per-wheel (x_offset, y_offset) from CG, body frame.

    ``x_offset`` positive forward; ``y_offset`` positive to the left.
    """
    a = dyn.wheelbase * (1.0 - dyn.cg_front)  # distance from CG to front axle
    b = dyn.wheelbase * dyn.cg_front          # distance from CG to rear axle
    return {
        "FL": (a, dyn.track_f * 0.5),
        "FR": (a, -dyn.track_f * 0.5),
        "RL": (-b, dyn.track_r * 0.5),
        "RR": (-b, -dyn.track_r * 0.5),
    }


def _wheel_omegas(state: VehicleState) -> dict[str, float]:
    return {
        "FL": state.omega_FL,
        "FR": state.omega_FR,
        "RL": state.omega_RL,
        "RR": state.omega_RR,
    }


def _tyre_radii(car: "Car") -> dict[str, float]:
    return {
        "FL": car.tyre_radius_f, "FR": car.tyre_radius_f,
        "RL": car.tyre_radius_r, "RR": car.tyre_radius_r,
    }


def _drive_split(car: "Car") -> dict[str, float]:
    """Fraction of engine torque sent to each wheel."""
    dtype = (car.drive_type or "RWD").upper()
    if dtype == "FWD":
        return {"FL": 0.5, "FR": 0.5, "RL": 0.0, "RR": 0.0}
    if dtype == "AWD":
        return {"FL": 0.25, "FR": 0.25, "RL": 0.25, "RR": 0.25}
    # RWD default.
    return {"FL": 0.0, "FR": 0.0, "RL": 0.5, "RR": 0.5}


def _brake_split(car: "Car") -> dict[str, float]:
    """Brake-torque share per wheel from ``brakes.ini`` ``FRONT_SHARE``."""
    front = float(getattr(car, "brake_front_share", 0.6))
    return {
        "FL": front * 0.5, "FR": front * 0.5,
        "RL": (1.0 - front) * 0.5, "RR": (1.0 - front) * 0.5,
    }


def _engine_torque_at_wheel(car: "Car", v_x: float, throttle: float) -> float:
    """Total wheel torque (sum across driven wheels) for current speed + throttle.

    Picks the optimal gear at this speed, reads engine torque off the LUT,
    applies throttle fraction, multiplies by gear * final ratio AND by the
    constant ``DRIVETRAIN_EFFICIENCY`` (v3 longitudinal-physics fix). Engine
    coast drag (`throttle < 1`) is added via a separate negative term scaled
    by ``car.coast_ref_torque / coast_ref_rpm`` (parsed from ``[COAST_REF]``):

        T_coast(rpm) = - coast_ref_torque * (rpm / coast_ref_rpm)
        T_wheel_coast = T_coast * gear_ratio_total * eta
                        * (1 - clip(throttle))

    Coast losses pass through the same drivetrain on the way to the wheel,
    so eta multiplies them too (the drivetrain still consumes mechanical
    work). The `(1 - throttle)` scaling models the smooth blend AC does
    internally: at full throttle there is no coast drag; at zero throttle
    the full COAST_REF curve applies.
    """
    v = max(float(v_x), 1.0)
    gear = car.optimal_gear(v)
    rpm = car.rpm_from_speed(v, gear)
    rpm = float(np.clip(rpm, car.power_rpm[0], car.rev_limit))
    drive_t = float(max(0.0, car.wheel_torque(rpm, gear)))
    drive_t *= float(np.clip(throttle, 0.0, 1.0)) * DRIVETRAIN_EFFICIENCY
    # Engine coast drag (negative wheel torque). Multiplied through the gear
    # ratio and final drive just like drive torque. Sign: negative torque
    # opposes wheel rotation in the forward-driving case.
    coast_ratio = float(car.gear_ratios[gear]) * float(car.final_ratio)
    coast_ref_rpm = float(getattr(car, 'coast_ref_rpm', 7000.0))
    coast_ref_torque = float(getattr(car, 'coast_ref_torque', 0.0))
    coast_blend = 1.0 - float(np.clip(throttle, 0.0, 1.0))
    if coast_ref_rpm > 1.0 and coast_ref_torque > 0.0:
        engine_brake_t = -coast_ref_torque * (rpm / coast_ref_rpm)
        coast_wheel_t = (
            engine_brake_t * coast_ratio * DRIVETRAIN_EFFICIENCY * coast_blend
        )
    else:
        coast_wheel_t = 0.0
    return drive_t + coast_wheel_t


def _weight_transfer(
    car: "Car",
    dyn: CarDynamics,
    v_x: float,
    a_x: float,
    a_y: float,
) -> dict[str, float]:
    """Quasi-static per-wheel normal load (spec §23.6.2 step 5).

    Inputs are chassis-frame accelerations (m/s^2). Positive ``a_x`` = forward
    accel; positive ``a_y`` = leftward accel. Aero downforce is added
    proportionally to the front/rear weight split.
    """
    m = car.total_mass
    g = G
    W = m * g
    # Static.
    Fz_front = W * (1.0 - dyn.cg_front)
    Fz_rear = W * dyn.cg_front
    # Longitudinal transfer: forward accel takes load off front, adds to rear.
    dFz_long = m * a_x * dyn.h_cg / max(dyn.wheelbase, 1e-3)
    Fz_front_pair = Fz_front - dFz_long
    Fz_rear_pair = Fz_rear + dFz_long
    # Lateral transfer per axle (assume front/rear roll stiffness equal in
    # v3.0 — better split is v3.1).
    dFz_lat_f = m * a_y * dyn.h_cg / max(dyn.track_f, 1e-3) * 0.5 * (1.0 - dyn.cg_front)
    dFz_lat_r = m * a_y * dyn.h_cg / max(dyn.track_r, 1e-3) * 0.5 * dyn.cg_front
    # Positive a_y = leftward accel => load transfers to the RIGHT side.
    # FL/RL lose load; FR/RR gain it.
    Fz = {
        "FL": Fz_front_pair * 0.5 - dFz_lat_f,
        "FR": Fz_front_pair * 0.5 + dFz_lat_f,
        "RL": Fz_rear_pair * 0.5 - dFz_lat_r,
        "RR": Fz_rear_pair * 0.5 + dFz_lat_r,
    }
    # Aero downforce.
    F_down = car.downforce(max(v_x, 0.0))
    Fz["FL"] += F_down * (1.0 - dyn.cg_front) * 0.5
    Fz["FR"] += F_down * (1.0 - dyn.cg_front) * 0.5
    Fz["RL"] += F_down * dyn.cg_front * 0.5
    Fz["RR"] += F_down * dyn.cg_front * 0.5
    # Floor at 100 N (effectively airborne below that).
    return {w: max(100.0, Fz[w]) for w in WHEELS}


def _mu_scale_from_tyre_state(
    tyre_state: "TyreState | None",
    car_tyre_model: object | None,
    wheel: str,
) -> float:
    """Per-wheel grip multiplier from the TyreState (compound-aware).

    Returns 1.0 if either input is None (Phase 3 default — TyreState plumbing
    layered in via simulator across laps, not inside the ODE).
    """
    if tyre_state is None or car_tyre_model is None:
        return 1.0
    from ..tyre_state import f_pressure_grip, f_temp, f_wear
    front = wheel.startswith("F")
    T = float(tyre_state.temp_C[wheel])
    W = float(tyre_state.wear_pct[wheel])
    p = float(tyre_state.pressure_psi[wheel])
    return float(
        f_pressure_grip(car_tyre_model, p, front=front)
        * f_temp(car_tyre_model, T, front=front)
        * f_wear(car_tyre_model, W, front=front)
    )


def compute_derivatives(
    state: VehicleState,
    controls: Controls,
    car: "Car",
    compound: "Compound | None",
    pacejka_calib: PacejkaCalibration,
    dyn: CarDynamics,
    *,
    tyre_state_snapshot: "TyreState | None" = None,
    car_tyre_model: object | None = None,
    drag_scale: float = 1.0,
    gravity_a_x: float = 0.0,
    record: dict | None = None,
) -> np.ndarray:
    """Return ``d(state)/dt`` for the 10-DOF chassis (spec §23.6.2).

    Parameters
    ----------
    state : VehicleState
        Current chassis state.
    controls : Controls
        Averaged front steer (rad), throttle/brake fractions.
    car : Car
        Parsed AC car (uses ``total_mass``, ``cg_front``, ``wheelbase``,
        ``brake_torque``, etc.).
    compound : Compound or None
        Active compound — currently only used by :func:`_mu_scale_from_tyre_state`
        via ``car_tyre_model``. Phase 3 keeps it loose; Phase 4 may consume.
    pacejka_calib : PacejkaCalibration
        Per-axle Magic-Formula coefficients + ellipse exponent.
    dyn : CarDynamics
        Extra chassis geometry (track widths, CG height, I_zz, I_wheel).
    tyre_state_snapshot : TyreState, optional
        Current per-wheel TyreState (held constant inside the ODE).
    car_tyre_model : CarTyreModel, optional
        Cached LUTs for the active compound; consumed by mu-scaling.
    drag_scale : float
        Per-step aero-drag scale factor (e.g. ``f_pressure_drag`` aggregate).
    gravity_a_x : float
        Body-frame longitudinal component of gravitational acceleration
        (m/s^2), signed. Positive = downhill (accelerating); negative =
        uphill (decelerating). v3 longitudinal-physics fix (2026-05-23):
        ``solver`` interpolates ``gradient_pct`` against arclength ``s``,
        converts to ``-g*sin(atan(gradient/100))``, and passes the value
        here. Default 0.0 treats the lap as flat.
    record : dict, optional
        If given, the function records per-wheel diagnostics into it (one
        scalar entry per WHEEL key under "alpha_rad", "kappa", "Fz", "Fx",
        "Fy"). Used by the solver to gather telemetry without re-deriving.

    Returns
    -------
    dstate_dt : np.ndarray, shape (10,)
    """
    s = state
    m = float(car.total_mass)
    I_zz = float(dyn.I_zz)
    I_w = float(dyn.I_wheel)

    # 1. Per-wheel steering angle (Ackermann).
    delta_FL, delta_FR = _ackermann_steer(controls.steer_rad, dyn.wheelbase, dyn.track_f)
    delta = {"FL": delta_FL, "FR": delta_FR, "RL": 0.0, "RR": 0.0}

    # 2. Per-wheel velocity at contact patch (body frame).
    offsets = _wheel_offsets(dyn)
    v_w_x_body: dict[str, float] = {}
    v_w_y_body: dict[str, float] = {}
    for w in WHEELS:
        ox, oy = offsets[w]
        v_w_x_body[w] = s.v_x - s.omega_yaw * oy
        v_w_y_body[w] = s.v_y + s.omega_yaw * ox

    # 3 & 4. Project onto tyre frame, slip-angle alpha and slip-ratio kappa.
    R_tyre = _tyre_radii(car)
    omegas = _wheel_omegas(state)
    v_long_w: dict[str, float] = {}
    v_lat_w: dict[str, float] = {}
    alpha: dict[str, float] = {}
    kappa: dict[str, float] = {}
    for w in WHEELS:
        c = np.cos(delta[w])
        sn = np.sin(delta[w])
        # Project body-frame velocity onto wheel frame.
        v_long = c * v_w_x_body[w] + sn * v_w_y_body[w]
        v_lat = -sn * v_w_x_body[w] + c * v_w_y_body[w]
        v_long_w[w] = float(v_long)
        v_lat_w[w] = float(v_lat)
        denom = max(abs(v_long), V_FLOOR)
        # Slip-angle convention (spec §23.6.2 step 3): alpha = atan2(-v_lat, |v_long|).
        alpha[w] = float(np.arctan2(-v_lat, denom))
        # Slip-ratio (spec §23.6.2 step 4), clamped to [-1.5, 1.5] to avoid
        # the low-speed blow-up where omega*R >> v_long produces non-physical
        # kappa (Pacejka was never meant to evaluate at kappa=30). Real
        # tyres reach peak Fx around kappa=0.1-0.2; saturate beyond that.
        kappa_raw = (omegas[w] * R_tyre[w] - v_long) / denom
        kappa[w] = float(np.clip(kappa_raw, -1.5, 1.5))

    # 5. Quasi-static weight transfer. We need chassis-frame inertial
    # acceleration of the CG (what an accelerometer on the car reads),
    # which is `F_total/m`. We don't know F_total yet because Fz feeds into
    # F. Phase 3 simplification: estimate (a_x, a_y) from the kinematic
    # steady-turn formula (the centripetal terms). In a steady left turn
    # (omega>0, v_x>0), lateral measured accel = v_x*omega (leftward).
    # Longitudinal measured accel ~ 0 in steady state — use 0 here. The
    # transient term `dv/dt - omega*v_perp` is what we're missing; for the
    # weight-transfer purposes this is good enough at small dt.
    a_x_est = 0.0                   # transient longitudinal — neglected in Phase 3
    a_y_est = s.v_x * s.omega_yaw   # steady-turn lateral accel of CG
    Fz = _weight_transfer(car, dyn, s.v_x, a_x_est, a_y_est)

    # 6. Pacejka per wheel + combined-slip friction ellipse. Per-axle AC
    # load-sensitivity (``LS_EXPY`` / ``LS_EXPX`` with ``FZ0``) and the
    # post-peak ``FALLOFF_LEVEL`` floor flow through both the per-channel
    # Magic-Formula evaluation and the ellipse semi-axes so the two stay
    # consistent: the ellipse cap is the same ``D(Fz) · Fz`` curve the
    # uncombined call would have peaked at.
    Fx_tyre: dict[str, float] = {}
    Fy_tyre: dict[str, float] = {}
    falloff = pacejka_calib.falloff_level
    for w in WHEELS:
        axle = "front" if w in ("FL", "FR") else "rear"
        coeffs = pacejka_calib.front if axle == "front" else pacejka_calib.rear
        mu = _mu_scale_from_tyre_state(tyre_state_snapshot, car_tyre_model, w)
        fy = float(pacejka_fy(
            alpha[w], Fz[w], coeffs.lateral,
            mu_scale=mu,
            Fz0=coeffs.Fz0, ls_exp=coeffs.ls_exp_lat,
            falloff_level=falloff,
        ))
        fx = float(pacejka_fx(
            kappa[w], Fz[w], coeffs.longitudinal,
            mu_scale=mu,
            Fz0=coeffs.Fz0, ls_exp=coeffs.ls_exp_long,
            falloff_level=falloff,
        ))
        # Combined-slip ellipse clamp.
        fx_c, fy_c = combined_friction_ellipse(
            fx, fy, Fz[w],
            D_x=float(coeffs.longitudinal.D) * mu,
            D_y=float(coeffs.lateral.D) * mu,
            ellipse_exponent=pacejka_calib.ellipse_exponent,
            Fz0_x=coeffs.Fz0, ls_exp_x=coeffs.ls_exp_long,
            Fz0_y=coeffs.Fz0, ls_exp_y=coeffs.ls_exp_lat,
        )
        Fx_tyre[w] = float(fx_c)
        Fy_tyre[w] = float(fy_c)

    # 7. Rotate per-wheel tyre forces back into chassis body frame.
    Fx_body: dict[str, float] = {}
    Fy_body: dict[str, float] = {}
    for w in WHEELS:
        c = np.cos(delta[w])
        sn = np.sin(delta[w])
        Fx_body[w] = float(c * Fx_tyre[w] - sn * Fy_tyre[w])
        Fy_body[w] = float(sn * Fx_tyre[w] + c * Fy_tyre[w])

    # 8. Sum forces + yaw moment. v3 longitudinal-physics fix (2026-05-23)
    # adds three previously missing terms to ``F_x_total``:
    #
    #   - Aero drag (already present): ``-0.5 * rho * v^2 * Cd * A``.
    #   - Rolling resistance: ``-Crr_model(v_x)``. Uses
    #     ``car.rolling_resistance()`` (same helper the v2 path consumes at
    #     `simulator.py:196-197`), signed against the body-x velocity so a
    #     reversing car would see it correctly. ~50 N at 60 m/s for the
    #     BMW 1M.
    #   - Gravity along grade: ``+m * gravity_a_x``. ``gravity_a_x`` is
    #     pre-projected to the body-x axis by the solver
    #     (``-g * sin(atan(gradient_pct/100))``); positive value =
    #     downhill = accelerating. We add it as a *force* via `m *`
    #     for consistency with the rest of the F_x_total sum, then divide
    #     by `m_eff` below. This means the gravity contribution to `dvx`
    #     is `gravity_a_x * (m / m_eff)`, which is correct (the
    #     reflected-inertia mass increase from the gearbox does NOT
    #     change the gravitational acceleration component, but it does
    #     blunt the chassis's response to a given gravitational *force*
    #     because some of that force is going into spinning up the
    #     engine on a descent. The first-order textbook form skips this
    #     and writes `+gravity_a_x` directly; the discrepancy is at most
    #     a few percent in 1st gear and ~0 in top gear).
    F_drag = 0.5 * RHO * (s.v_x ** 2) * car.aero_cd * car.frontal_area * float(drag_scale)
    F_roll = float(car.rolling_resistance(abs(s.v_x)))
    sign_vx = 1.0 if s.v_x >= 0.0 else -1.0
    F_grav_x = float(m * gravity_a_x)
    F_x_total = (
        sum(Fx_body.values())
        - F_drag
        - sign_vx * F_roll
        + F_grav_x
    )
    F_y_total = sum(Fy_body.values())
    a = dyn.wheelbase * (1.0 - dyn.cg_front)
    b = dyn.wheelbase * dyn.cg_front
    M_z_total = (
        (Fy_body["FL"] + Fy_body["FR"]) * a
        - (Fy_body["RL"] + Fy_body["RR"]) * b
        + (Fx_body["FR"] - Fx_body["FL"]) * (dyn.track_f * 0.5)
        + (Fx_body["RR"] - Fx_body["RL"]) * (dyn.track_r * 0.5)
    )

    # 9. Wheel torques (engine + brake) and wheel angular accel.
    wheel_torque_total = _engine_torque_at_wheel(car, s.v_x, controls.throttle)
    drive_split = _drive_split(car)
    brake_split = _brake_split(car)
    max_brake_per_wheel = float(getattr(car, "brake_torque", 2500.0))
    tau_w: dict[str, float] = {}
    for w in WHEELS:
        tau_drive = wheel_torque_total * drive_split[w]
        tau_brake = max_brake_per_wheel * brake_split[w] * float(np.clip(controls.brake, 0.0, 1.0))
        # Sign: braking opposes rotation.
        sign_omega = 1.0 if omegas[w] >= 0.0 else -1.0
        tau_net = tau_drive - sign_omega * tau_brake - Fx_tyre[w] * R_tyre[w]
        tau_w[w] = float(tau_net)

    # 10. Effective chassis mass (v3 longitudinal-physics fix). The textbook
    # form reflects the engine-side rotational inertia through the gear ratio
    # squared. Wheel-side inertia is already represented implicitly in the
    # per-wheel omega ODE (`d_omega_w = tau_w / I_w`), so we only need the
    # engine-side contribution here:
    #
    #   m_eff = m + I_engine * (gear * final)^2 / r_wheel^2
    #
    # The wheel radius used is the drive-axle radius (rear for RWD, front for
    # FWD, average for AWD). In 1st gear at the BMW 1M (gear=4.11, final=3.15,
    # I_engine=0.16 kg.m^2, r=0.334 m) this adds ~120 kg of equivalent mass;
    # in 6th gear (gear=0.846) it adds only ~5 kg. The lateral and yaw DOFs
    # are unaffected (no engine-side rotation coupled to v_y or omega_yaw).
    v_for_gear = max(float(s.v_x), 1.0)
    cur_gear = car.optimal_gear(v_for_gear)
    gear_ratio = float(car.gear_ratios[cur_gear]) * float(car.final_ratio)
    if str(getattr(car, 'drive_type', 'RWD')).upper() == 'FWD':
        r_drive = float(car.tyre_radius_f)
    elif str(getattr(car, 'drive_type', 'RWD')).upper() == 'AWD':
        r_drive = 0.5 * (float(car.tyre_radius_f) + float(car.tyre_radius_r))
    else:
        r_drive = float(car.tyre_radius_r)
    m_eff = m + float(car.engine_inertia) * (gear_ratio ** 2) / max(r_drive ** 2, 1e-6)

    # 11. State derivatives.
    dx = s.v_x * np.cos(s.psi) - s.v_y * np.sin(s.psi)
    dy = s.v_x * np.sin(s.psi) + s.v_y * np.cos(s.psi)
    dpsi = s.omega_yaw
    dvx = F_x_total / m_eff + s.v_y * s.omega_yaw
    dvy = F_y_total / m - s.v_x * s.omega_yaw
    domega = M_z_total / max(I_zz, 1e-3)
    d_omega_FL = tau_w["FL"] / I_w
    d_omega_FR = tau_w["FR"] / I_w
    d_omega_RL = tau_w["RL"] / I_w
    d_omega_RR = tau_w["RR"] / I_w

    if record is not None:
        record["alpha_rad"] = dict(alpha)
        record["kappa"] = dict(kappa)
        record["Fz"] = dict(Fz)
        record["Fx"] = dict(Fx_tyre)
        record["Fy"] = dict(Fy_tyre)
        record["delta"] = dict(delta)

    return np.array([dx, dy, dpsi, dvx, dvy, domega,
                     d_omega_FL, d_omega_FR, d_omega_RL, d_omega_RR],
                    dtype=float)
