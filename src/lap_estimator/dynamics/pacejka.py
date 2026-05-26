"""Pacejka Magic Formula tyre force calculator (spec §23.3, §23.6.2 step 6).

Pure, NumPy-vectorised functions. Given per-wheel slip angle / slip ratio,
normal load, and per-axle Magic Formula coefficients ``(B, C, D, E)``, return
the lateral / longitudinal tyre force. A combined-slip friction-ellipse
clamp closes the model when both slip channels are loaded simultaneously.

Reference: Hans Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., Ch. 4.

Phase 2 status: implemented per §23.6.2 step 6. Coefficient "D" is treated
as the peak-grip coefficient (mu); the actual peak force at a given Fz is
``D * Fz``. ``mu_scale`` further modulates D per the per-wheel TyreState
(``f_pressure_grip * f_temp * f_wear``) — applied before the sine.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PacejkaCoeffs:
    """Magic Formula coefficients for a single (axle, direction) pair.

    Attributes
    ----------
    B : float
        Stiffness factor (typical road tyre 8-12).
    C : float
        Shape factor (typical 1.3 lat, 1.65 long).
    D : float
        Peak factor / mu (typical 1.0-1.6). Scales linearly with the
        per-wheel grip multiplier from ``TyreState`` (compound-aware).
    E : float
        Curvature factor (typical -0.2 to 0.5).
    """

    B: float
    C: float
    D: float
    E: float


# Phase 2 spec-explicit aliases. Some call sites (fitter, future
# vehicle.py) want a more direction-explicit name. They wrap the same
# four-tuple so all the math stays unified.
LateralCoeffs = PacejkaCoeffs
LongitudinalCoeffs = PacejkaCoeffs


def _load_sens_factor(
    Fz_arr: np.ndarray,
    Fz0: float | None,
    ls_exp: float | None,
) -> np.ndarray | float:
    """Return ``(Fz/Fz0)**(LS_EXP - 1)`` or ``1.0`` if either knob is unset.

    AC's brush-model load sensitivity (``tyres.ini`` keys ``LS_EXPY`` /
    ``LS_EXPX`` with reference load ``FZ0``) scales the Pacejka peak ``D``
    sub-linearly with ``Fz``: at ``Fz = FZ0`` the factor is exactly 1, so
    this matches the legacy linear-D fit at the operating point; below
    ``FZ0`` the inner / unloaded wheel gains grip, above ``FZ0`` the
    outer / loaded wheel loses grip. With ``LS_EXPY ~ 0.83`` this is
    ~+10 % grip at the inside wheel in a hard chicane apex.
    """
    if Fz0 is None or ls_exp is None or Fz0 <= 0.0:
        return 1.0
    if abs(float(ls_exp) - 1.0) < 1e-12:
        return 1.0
    # Floor Fz at 1 N defensively (matches per-axle floors elsewhere).
    base = np.maximum(Fz_arr, 1.0) / float(Fz0)
    return np.power(base, float(ls_exp) - 1.0)


def _magic_formula(slip: np.ndarray | float,
                   Fz: np.ndarray | float,
                   coeffs: PacejkaCoeffs,
                   mu_scale: float = 1.0,
                   *,
                   Fz0: float | None = None,
                   ls_exp: float | None = None,
                   falloff_level: float | None = None) -> np.ndarray | float:
    """Core Magic Formula: ``F = D_eff * Fz * sin(...)``.

    Shared by :func:`pacejka_fy` and :func:`pacejka_fx`; the slip variable
    is interpreted as slip-angle (rad) or slip-ratio (dimensionless)
    depending on which axle pack is supplied.

    Non-linear load sensitivity (AC ``LS_EXPY`` / ``LS_EXPX``)
    -----------------------------------------------------------
    With ``Fz0`` and ``ls_exp`` provided the peak ``D(Fz)`` becomes

        D(Fz) = D_ref · (Fz / Fz0)**(ls_exp - 1)

    which matches AC's brush-model peak-load curve. At ``Fz = Fz0`` the
    factor reduces to ``1.0``; the legacy linear-D fit calibrated at the
    static axle load remains exact at that point. ``ls_exp = None``
    keeps the historical ``D · Fz`` behaviour.

    Post-peak falloff (AC ``FALLOFF_LEVEL``)
    -----------------------------------------
    Past the slip-angle peak the Magic Formula's ``sin(C·atan(...))``
    drifts toward ``D · sin(C · pi/2)``. AC keeps a residual grip floor
    so the tail doesn't collapse: when ``falloff_level`` is set we clip
    ``|sin(C·atan(...))|`` from below at ``falloff_level · sin(C · pi/2)``.
    Sign is preserved so the model remains odd in slip.
    """
    B = coeffs.B
    C = coeffs.C
    D = coeffs.D * float(mu_scale)
    E = coeffs.E
    Bx = B * np.asarray(slip, dtype=float)
    inner = Bx - E * (Bx - np.arctan(Bx))
    Fz_arr = np.asarray(Fz, dtype=float)
    sin_term = np.sin(C * np.arctan(inner))
    if falloff_level is not None and falloff_level > 0.0:
        # |sin| at the Magic Formula's peak slip is sin(C · pi/2). Clip the
        # *magnitude* of the sin term to ``falloff_level · peak_sin`` while
        # keeping its sign so the model stays odd in slip and the
        # transition through alpha=0 is smooth.
        peak_sin = float(np.sin(C * np.pi * 0.5))
        floor_mag = float(falloff_level) * peak_sin
        # Only the "deep" tail benefits — protect the central region where
        # |sin_term| is naturally below the floor (small slip = small Fy).
        # Use sign(sin_term) * max(|sin_term|, floor_mag * step) where
        # ``step`` masks the central region. We approximate that mask via
        # ``tanh(|inner|)``: ~0 near alpha=0, ~1 once we're well into the
        # Magic Formula's saturation regime. Matches AC's intent (a floor
        # past peak, not a global lift).
        gate = np.tanh(np.abs(inner) * 0.5)
        floor_signed = np.sign(sin_term) * floor_mag * gate
        sin_term = np.where(
            np.abs(sin_term) >= np.abs(floor_signed),
            sin_term,
            floor_signed,
        )
    load_factor = _load_sens_factor(Fz_arr, Fz0, ls_exp)
    return D * load_factor * Fz_arr * sin_term


def pacejka_fy(
    alpha: np.ndarray | float,
    Fz: np.ndarray | float,
    coeffs: PacejkaCoeffs,
    *,
    mu_scale: float = 1.0,
    Fz0: float | None = None,
    ls_exp: float | None = None,
    falloff_level: float | None = None,
) -> np.ndarray | float:
    """Return lateral tyre force ``Fy`` from slip angle ``alpha`` and load ``Fz``.

    ``Fy = D(Fz) * Fz * sin(C * atan(B*alpha - E*(B*alpha - atan(B*alpha))))``
    where ``D(Fz) = D_ref · (Fz / Fz0)**(ls_exp − 1)`` when ``Fz0`` and
    ``ls_exp`` are supplied (AC ``LS_EXPY``), else ``D(Fz) = D_ref``
    (legacy linear-load behaviour).

    Parameters
    ----------
    alpha : array-like, radians
        Slip angle (driver-axes convention: ``alpha = steer - heading_vel``).
    Fz : array-like, N
        Vertical (normal) load on the contact patch. Clamped >= 100 N
        upstream — see §23.6.2 step 5.
    coeffs : PacejkaCoeffs
        Per-axle lateral coefficients.
    mu_scale : float
        Per-wheel grip multiplier from ``TyreState``
        (``f_pressure_grip * f_temp * f_wear``). Applied as
        ``D_eff = D * mu_scale``.
    Fz0, ls_exp, falloff_level : optional
        AC non-linear-load / falloff parameters. See :func:`_magic_formula`.

    Returns
    -------
    Fy : array-like, N
        Lateral tyre force at the contact patch.
    """
    return _magic_formula(
        alpha, Fz, coeffs,
        mu_scale=mu_scale,
        Fz0=Fz0, ls_exp=ls_exp, falloff_level=falloff_level,
    )


def pacejka_fx(
    kappa: np.ndarray | float,
    Fz: np.ndarray | float,
    coeffs: PacejkaCoeffs,
    *,
    mu_scale: float = 1.0,
    Fz0: float | None = None,
    ls_exp: float | None = None,
    falloff_level: float | None = None,
) -> np.ndarray | float:
    """Return longitudinal tyre force ``Fx`` from slip ratio ``kappa`` and load ``Fz``.

    Same Magic Formula shape as :func:`pacejka_fy` with the longitudinal
    coefficient triplet ``(B_long, C_long, D_long, E_long)``. ``kappa`` is
    dimensionless and follows the ``omega*R / v_x - 1`` convention.

    AC ``LS_EXPX`` non-linear-load scaling is supported via ``Fz0`` /
    ``ls_exp``; see :func:`pacejka_fy`.
    """
    return _magic_formula(
        kappa, Fz, coeffs,
        mu_scale=mu_scale,
        Fz0=Fz0, ls_exp=ls_exp, falloff_level=falloff_level,
    )


# Phase 2 spec aliases — `pacejka_lateral` / `pacejka_longitudinal` are the
# names referenced in the §23.7 Phase 2 brief; we keep `pacejka_fy/fx` as
# canonical and provide thin wrappers so both call sites read naturally.
def pacejka_lateral(
    alpha_rad: np.ndarray | float,
    Fz_N: np.ndarray | float,
    B: float,
    C: float,
    D: float,
    E: float,
) -> np.ndarray | float:
    """Magic Formula lateral force from raw scalar coefficients (Phase 2 fitter).

    Thin wrapper around :func:`pacejka_fy` that takes B/C/D/E directly —
    suitable as a curve-fit objective with explicit free parameters.
    """
    return _magic_formula(alpha_rad, Fz_N, PacejkaCoeffs(B, C, D, E))


def pacejka_longitudinal(
    kappa: np.ndarray | float,
    Fz_N: np.ndarray | float,
    B: float,
    C: float,
    D: float,
    E: float,
) -> np.ndarray | float:
    """Magic Formula longitudinal force from raw scalar coefficients."""
    return _magic_formula(kappa, Fz_N, PacejkaCoeffs(B, C, D, E))


def combined_friction_ellipse(
    Fx: np.ndarray | float,
    Fy: np.ndarray | float,
    Fz: np.ndarray | float,
    D_x: float,
    D_y: float,
    *,
    ellipse_exponent: float = 2.0,
    Fz0_x: float | None = None,
    ls_exp_x: float | None = None,
    Fz0_y: float | None = None,
    ls_exp_y: float | None = None,
) -> tuple[np.ndarray | float, np.ndarray | float]:
    """Clamp ``(Fx, Fy)`` onto the friction ellipse boundary.

    Ellipse semi-axes are ``(D_x · LS_x(Fz) · Fz, D_y · LS_y(Fz) · Fz)``
    where ``LS_*(Fz) = (Fz / Fz0)**(ls_exp - 1)`` when the AC non-linear
    load knobs are provided, else ``1.0``. This keeps the ellipse
    consistent with the per-direction Pacejka peaks: at ``Fz = Fz0`` the
    semi-axes reduce to the legacy ``D · Fz`` form and the linear-load
    fit is recovered exactly.

    If ``((Fx/(D_x*LS_x*Fz))^n + (Fy/(D_y*LS_y*Fz))^n) > 1`` the pair is
    scaled proportionally to land on the boundary. ``n = ellipse_exponent``;
    ``n = 2`` is a true ellipse, real tyres often fit 2.2-2.5 (blunter,
    more grip near combined max).
    """
    Fx_arr = np.asarray(Fx, dtype=float)
    Fy_arr = np.asarray(Fy, dtype=float)
    Fz_arr = np.asarray(Fz, dtype=float)
    # Floor Fz to avoid div-by-zero on airborne wheels (Fz clamped to 100 N
    # upstream per spec §23.6.2 step 5, but be defensive).
    Fz_safe = np.maximum(Fz_arr, 1.0)
    load_x = _load_sens_factor(Fz_safe, Fz0_x, ls_exp_x)
    load_y = _load_sens_factor(Fz_safe, Fz0_y, ls_exp_y)
    Fx_max = D_x * load_x * Fz_safe
    Fy_max = D_y * load_y * Fz_safe
    n = float(ellipse_exponent)
    # Normalised force magnitudes; only clamp when ratio > 1.
    nx = np.abs(Fx_arr / Fx_max) ** n
    ny = np.abs(Fy_arr / Fy_max) ** n
    ratio = nx + ny
    # Scale factor s such that ((s*Fx)/Fx_max)^n + ((s*Fy)/Fy_max)^n = 1.
    # s = ratio^(-1/n) when ratio > 1, else 1.
    over = ratio > 1.0
    scale = np.where(over, np.power(np.maximum(ratio, 1e-12), -1.0 / n), 1.0)
    return Fx_arr * scale, Fy_arr * scale


def combined_slip_force(
    alpha_rad: np.ndarray | float,
    kappa: np.ndarray | float,
    Fz_N: np.ndarray | float,
    coeffs_lat: PacejkaCoeffs,
    coeffs_long: PacejkaCoeffs,
    mu: float = 1.0,
    *,
    ellipse_exponent: float = 2.0,
    Fz0_lat: float | None = None,
    ls_exp_lat: float | None = None,
    Fz0_long: float | None = None,
    ls_exp_long: float | None = None,
    falloff_level: float | None = None,
) -> tuple[np.ndarray | float, np.ndarray | float]:
    """Return ``(Fx, Fy)`` under combined slip with friction-ellipse coupling.

    Computes the independent Pacejka forces in each direction (with the
    AC non-linear load knobs applied per direction), then projects onto
    the friction ellipse with semi-axes
    ``(D_long · mu · LS_long(Fz) · Fz, D_lat · mu · LS_lat(Fz) · Fz)``.
    Both components scale proportionally when the demanded force exceeds
    the ellipse boundary.
    """
    Fy = pacejka_fy(
        alpha_rad, Fz_N, coeffs_lat,
        mu_scale=mu, Fz0=Fz0_lat, ls_exp=ls_exp_lat,
        falloff_level=falloff_level,
    )
    Fx = pacejka_fx(
        kappa, Fz_N, coeffs_long,
        mu_scale=mu, Fz0=Fz0_long, ls_exp=ls_exp_long,
        falloff_level=falloff_level,
    )
    return combined_friction_ellipse(
        Fx, Fy, Fz_N,
        D_x=coeffs_long.D * mu,
        D_y=coeffs_lat.D * mu,
        ellipse_exponent=ellipse_exponent,
        Fz0_x=Fz0_long, ls_exp_x=ls_exp_long,
        Fz0_y=Fz0_lat, ls_exp_y=ls_exp_lat,
    )
