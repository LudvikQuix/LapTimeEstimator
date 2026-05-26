"""MPCC reference-path builder + curvilinear coordinate helpers (spec §23.3.6.1).

The MPCC controller plans in **curvilinear (s, n, ψ_e)** coordinates along a
fixed reference path. This module owns the construction of that path from the
track CSV + DP plan, plus the forward/inverse coordinate transforms that map
between chassis (x, z, ψ) and curvilinear (s, n, ψ_e).

Sign convention (matches the rest of the v3 stack):

- ``n > 0`` ⇒ chassis to the **LEFT** of the reference path (Frenet).
- ``ψ_e`` wrapped to ``[-π, π]``.
- ``s`` modulo ``track_length`` (set by caller; helpers don't wrap implicitly).

The reference path is uniformly resampled on a fine ``ds_ref = 0.5 m`` grid.
Curvature is clamped to ``|κ| ≤ kappa_clamp`` (default 0.2 rad/m → radius ≥ 5 m)
so the curvilinear Jacobian ``ds/dt = (v_x cos ψ_e − v_y sin ψ_e) / (1 − n·κ)``
stays well-conditioned at the chicane apex (spec §23.3.11 risk #1).

Track-edge constraint source (build-time decision §23.3.10.2):

- If ``track.csv_data['width_total_m']`` is present, the per-sample
  half-width is ``0.5 * width_total_m`` (Sprint A is in the 11–14 m range).
- Otherwise, ``track_half_width_default`` is broadcast (caller's call; the
  controller passes ``5.0`` for a sane fallback).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


DEFAULT_DS_REF = 0.5  # m — fine uniform s-grid for the reference resample.
DEFAULT_KAPPA_CLAMP = 0.2  # rad/m → radius ≥ 5 m at the apex.
DEFAULT_TRACK_HALF_WIDTH = 5.0  # m — fallback when track CSV lacks width column.
DEFAULT_EDGE_SAFETY = 0.3  # m — buffer inside the track edge (spec §23.3.5).


@dataclass(frozen=True)
class ReferencePath:
    """Pre-computed reference path on a uniform s-grid (spec §23.3.7.1).

    Attributes
    ----------
    s_grid : np.ndarray
        Uniform along-path distance grid (m). Length ``M``.
    xs, zs : np.ndarray
        Reference path coordinates in chassis-frame axes (the track CSV uses
        ``x`` and ``z`` as the two planar axes; we keep the same convention
        here so the controller stays drop-in with ``MPCController``).
    psi_ref : np.ndarray
        Path tangent angle (rad), length ``M``.
    kappa_ref : np.ndarray
        Signed curvature (rad/m), clamped to ``[-kappa_clamp, +kappa_clamp]``.
    v_ref : np.ndarray
        DP-plan speed (m/s) interpolated onto ``s_grid``.
    track_half_width : np.ndarray
        Per-sample track half-width (m). Constant array when CSV lacks the
        ``width_total_m`` column.
    total_length : float
        ``s_grid[-1]``; convenience for wrap-around computations.
    """

    s_grid: np.ndarray
    xs: np.ndarray
    zs: np.ndarray
    psi_ref: np.ndarray
    kappa_ref: np.ndarray
    v_ref: np.ndarray
    track_half_width: np.ndarray
    total_length: float


def build_reference_path(
    track,
    plan,
    *,
    ds_ref: float = DEFAULT_DS_REF,
    kappa_clamp: float = DEFAULT_KAPPA_CLAMP,
    track_half_width_default: float = DEFAULT_TRACK_HALF_WIDTH,
    line_override: tuple[np.ndarray, np.ndarray] | None = None,
) -> ReferencePath:
    """Build a uniform-s reference path from the track CSV + DP plan.

    Parameters
    ----------
    track : Track
        Must be CSV-backed; we read ``x``, ``z``, ``distance_m``, and
        ``radius_m`` (required). Optionally ``width_total_m``.
    plan : LongitudinalPlan
        Provides ``distances`` and ``speeds`` arrays for the v_ref column.
    ds_ref : float
        Uniform spacing of the resampled grid (m). Default 0.5 m.
    kappa_clamp : float
        Maximum |κ| (rad/m). Default 0.2 (radius ≥ 5 m).
    track_half_width_default : float
        Fallback half-width when the track CSV lacks the ``width_total_m``
        column. Default 5.0 m.
    line_override : tuple of (xs, zs), optional
        Replace the centreline ``(x, z)`` with a user-supplied racing
        line. The arrays MUST be parameterised against the track's
        ``distance_m`` column (same length as ``track.csv_data['distance_m']``).
        Used by the Tomas-line experiment (2026-05-24) so the MPCC tracks
        Tomas's recorded line instead of the centreline. The tangent
        (``psi_ref``) is recomputed from the override; the curvature
        magnitude continues to come from the centreline's ``radius_m``
        column (the override line's true curvature would need to be
        computed from the integrated trajectory, which is the
        Phase-5.1-line-fit backlog item). The track-half-width
        constraint is left at the centreline width — Tomas's line is
        within the existing track edges per the empirical reconstruction.

    Returns
    -------
    ReferencePath
    """
    data = track.csv_data
    ds_raw = np.asarray(data["distance_m"], dtype=float)
    radius_raw = np.asarray(data["radius_m"], dtype=float)
    if line_override is not None:
        xs_raw = np.asarray(line_override[0], dtype=float)
        zs_raw = np.asarray(line_override[1], dtype=float)
        if len(xs_raw) != len(ds_raw) or len(zs_raw) != len(ds_raw):
            raise ValueError(
                f"build_reference_path line_override length mismatch: "
                f"xs={len(xs_raw)}, zs={len(zs_raw)}, "
                f"distance_m={len(ds_raw)}"
            )
    else:
        xs_raw = np.asarray(data["x"], dtype=float)
        zs_raw = np.asarray(data["z"], dtype=float)

    total_length = float(ds_raw[-1])
    if total_length <= ds_ref:
        raise ValueError(
            f"Track length {total_length:.2f} m is too short for ds_ref={ds_ref}"
        )

    # Build the uniform s-grid (inclusive of 0; stops before total_length so
    # the lookup never lands on the wrap-around).
    n_samples = int(math.floor(total_length / ds_ref)) + 1
    s_grid = np.arange(n_samples, dtype=float) * ds_ref
    s_grid = np.clip(s_grid, 0.0, total_length - 1e-6)

    # Interpolate position onto the uniform grid.
    xs = np.interp(s_grid, ds_raw, xs_raw)
    zs = np.interp(s_grid, ds_raw, zs_raw)

    # Tangent angle: per-sample central difference of the resampled (x, z).
    # The first/last samples use forward/backward differences. Result is in
    # radians; sign convention atan2(dz, dx) matches Track and the v2 stack.
    psi_ref = _compute_tangent(xs, zs)

    # Signed curvature derived from the track's radius_m column. The CSV
    # stores |radius|; we re-sign from the resampled tangent angle's local
    # rotational sense (positive κ = left-turn).
    kappa_ref = _compute_signed_kappa(
        s_grid, ds_raw, radius_raw, psi_ref,
        kappa_clamp=kappa_clamp,
    )

    # v_ref on the uniform grid.
    v_ref = np.interp(
        s_grid,
        np.asarray(plan.distances, dtype=float),
        np.asarray(plan.speeds, dtype=float),
    )
    v_ref = np.maximum(v_ref, 1.0)  # floor; controller has its own V_FLOOR.

    # Track half-width.
    if "width_total_m" in data:
        w_total = np.asarray(data["width_total_m"], dtype=float)
        # Per-sample half-width; floor at 2 m so a CSV anomaly never collapses
        # the constraint completely.
        half_w_raw = np.maximum(0.5 * w_total, 2.0)
        track_half_width = np.interp(s_grid, ds_raw, half_w_raw)
    else:
        track_half_width = np.full_like(s_grid, float(track_half_width_default))

    return ReferencePath(
        s_grid=s_grid,
        xs=xs,
        zs=zs,
        psi_ref=psi_ref,
        kappa_ref=kappa_ref,
        v_ref=v_ref,
        track_half_width=track_half_width,
        total_length=float(s_grid[-1]),
    )


def _compute_tangent(xs: np.ndarray, zs: np.ndarray) -> np.ndarray:
    """Per-sample tangent angle ``ψ_ref(s) = atan2(dz/ds, dx/ds)``."""
    n = len(xs)
    psi = np.zeros(n)
    if n < 2:
        return psi
    # Interior central differences.
    dx = np.zeros(n)
    dz = np.zeros(n)
    dx[1:-1] = xs[2:] - xs[:-2]
    dz[1:-1] = zs[2:] - zs[:-2]
    # Edges: one-sided differences.
    dx[0] = xs[1] - xs[0]
    dz[0] = zs[1] - zs[0]
    dx[-1] = xs[-1] - xs[-2]
    dz[-1] = zs[-1] - zs[-2]
    psi = np.arctan2(dz, dx)
    # Unwrap so successive samples don't jump by ±2π (matters for the
    # `nearest_s` projection's Newton refinement and for downstream
    # `s - θ` shortest-path arithmetic).
    psi = np.unwrap(psi)
    return psi


def _compute_signed_kappa(
    s_grid: np.ndarray,
    ds_raw: np.ndarray,
    radius_raw: np.ndarray,
    psi_ref: np.ndarray,
    *,
    kappa_clamp: float,
) -> np.ndarray:
    """Return signed κ(s) on the uniform grid (positive = left turn).

    Magnitude comes from interpolating ``1/radius_m``; sign comes from the
    local tangent's rotational sense. Below the straight-line threshold
    (radius ≥ 500 m → |κ| ≤ 2e-3) the value is zeroed so the linearised
    plant doesn't chase phantom yaw demand at the chicane.
    """
    STRAIGHT_RADIUS_M = 500.0
    # Magnitude.
    radius = np.interp(s_grid, ds_raw, np.abs(radius_raw))
    active = radius < STRAIGHT_RADIUS_M
    kappa_mag = np.where(active, 1.0 / np.maximum(radius, 1.0), 0.0)
    # Sign from the tangent's local rate of rotation. Central difference of
    # the (already-unwrapped) psi_ref gives dpsi/ds directly = κ. We use it
    # as the sign source but cap magnitude at ``1/radius`` (the CSV is the
    # source of truth for magnitude — the tangent rate is noisier).
    n = len(s_grid)
    if n < 3:
        return np.clip(kappa_mag, -kappa_clamp, kappa_clamp)
    ds_grid = float(s_grid[1] - s_grid[0])
    dpsi = np.zeros(n)
    dpsi[1:-1] = (psi_ref[2:] - psi_ref[:-2]) / (2.0 * ds_grid)
    dpsi[0] = (psi_ref[1] - psi_ref[0]) / max(ds_grid, 1e-6)
    dpsi[-1] = (psi_ref[-1] - psi_ref[-2]) / max(ds_grid, 1e-6)
    sign_arr = np.where(dpsi >= 0, 1.0, -1.0)
    kappa_signed = sign_arr * kappa_mag
    return np.clip(kappa_signed, -kappa_clamp, kappa_clamp)


# ----------------------------------------------------------------------
# Forward / inverse transforms (chassis ⇄ curvilinear).
# ----------------------------------------------------------------------


def to_curvilinear(
    x: float, z: float, psi: float,
    ref: ReferencePath,
    hint_idx: int = 0,
    *,
    newton_steps: int = 1,
) -> tuple[float, float, float, int]:
    """Project chassis ``(x, z, ψ)`` onto the reference, return ``(s, n, ψ_e, idx_new)``.

    Uses ``hint_idx`` to seed a windowed nearest-point scan (cheap; same
    pattern as :func:`mpc_controller_geom.nearest_index`). One Newton step
    refines ``s`` to sub-sample precision; matches the canonical Frenet
    projection used in Liniger MPCC and TUMFTM.

    Sign of ``n``: positive when the chassis is left of the path's tangent
    direction (Frenet convention).
    """
    xs = ref.xs
    zs = ref.zs
    n_total = len(xs)
    # Windowed nearest-index search around the hint.
    lo = max(0, int(hint_idx) - 5)
    hi = min(n_total, int(hint_idx) + 200)
    if hi <= lo:
        hi = min(n_total, lo + 200)
    seg_x = xs[lo:hi]
    seg_z = zs[lo:hi]
    d2 = (seg_x - x) ** 2 + (seg_z - z) ** 2
    j = int(np.argmin(d2))
    idx = lo + j
    # Sub-sample refinement via one Newton step against the tangent.
    s_idx = float(ref.s_grid[idx])
    psi_idx = float(ref.psi_ref[idx])
    # The reference is parameterised by s, so locally
    # p_ref(s + δ) ≈ p_ref(s) + δ · (cos ψ, sin ψ).
    # Solve δ = (p − p_ref) · (cos ψ, sin ψ).
    dx_local = float(x) - float(xs[idx])
    dz_local = float(z) - float(zs[idx])
    for _ in range(max(0, int(newton_steps))):
        delta = dx_local * math.cos(psi_idx) + dz_local * math.sin(psi_idx)
        s_idx = s_idx + delta
        # Re-interpolate at the refined s. Bounded inside [0, total_length].
        s_idx = float(np.clip(s_idx, 0.0, ref.total_length))
        x_ref = float(np.interp(s_idx, ref.s_grid, ref.xs))
        z_ref = float(np.interp(s_idx, ref.s_grid, ref.zs))
        psi_idx = float(np.interp(s_idx, ref.s_grid, ref.psi_ref))
        dx_local = float(x) - x_ref
        dz_local = float(z) - z_ref
    # Signed perpendicular distance. normal = (-sin ψ, cos ψ); positive n
    # means chassis is left of the path tangent.
    n_signed = -dx_local * math.sin(psi_idx) + dz_local * math.cos(psi_idx)
    # Heading error wrapped to [-π, π].
    psi_e = math.atan2(
        math.sin(float(psi) - psi_idx),
        math.cos(float(psi) - psi_idx),
    )
    return float(s_idx), float(n_signed), float(psi_e), int(idx)


def to_chassis(s: float, n: float, psi_e: float, ref: ReferencePath) -> tuple[float, float, float]:
    """Inverse transform: ``(s, n, ψ_e) → (x_chassis, z_chassis, ψ_chassis)``."""
    s = float(np.clip(s, 0.0, ref.total_length))
    x_ref = float(np.interp(s, ref.s_grid, ref.xs))
    z_ref = float(np.interp(s, ref.s_grid, ref.zs))
    psi_ref_s = float(np.interp(s, ref.s_grid, ref.psi_ref))
    x_chassis = x_ref - n * math.sin(psi_ref_s)
    z_chassis = z_ref + n * math.cos(psi_ref_s)
    psi_chassis = psi_ref_s + psi_e
    return x_chassis, z_chassis, psi_chassis


# ----------------------------------------------------------------------
# Wrap-aware helpers (spec §23.3.11 risk #7).
# ----------------------------------------------------------------------


def signed_shortest(delta_s: float, total_length: float) -> float:
    """Shortest-path signed s-distance modulo ``total_length``.

    Used by the lag-cost residual ``(s − θ)`` to handle the start/finish line
    wraparound: a chassis that is ahead of the reference by ε then suddenly
    behind by ``total_length − ε`` after the wrap should still see a small
    residual.
    """
    L = float(total_length)
    if L <= 0:
        return float(delta_s)
    half = 0.5 * L
    d = float(delta_s) % L
    if d > half:
        d -= L
    return float(d)


def kappa_at(s: float, ref: ReferencePath) -> float:
    """Interpolated reference curvature κ(s)."""
    return float(np.interp(
        float(np.clip(s, 0.0, ref.total_length)),
        ref.s_grid, ref.kappa_ref,
    ))


def v_ref_at(s: float, ref: ReferencePath) -> float:
    """Interpolated reference speed v_ref(s)."""
    return float(np.interp(
        float(np.clip(s, 0.0, ref.total_length)),
        ref.s_grid, ref.v_ref,
    ))


def half_width_at(s: float, ref: ReferencePath) -> float:
    """Interpolated track half-width at the given s (m)."""
    return float(np.interp(
        float(np.clip(s, 0.0, ref.total_length)),
        ref.s_grid, ref.track_half_width,
    ))


def sample_seq(
    s_seq: np.ndarray,
    ref: ReferencePath,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorised ``(kappa, v_ref, half_width)`` lookup for stage gridding."""
    s_clipped = np.clip(np.asarray(s_seq, dtype=float), 0.0, ref.total_length)
    kappa = np.interp(s_clipped, ref.s_grid, ref.kappa_ref)
    v = np.interp(s_clipped, ref.s_grid, ref.v_ref)
    w = np.interp(s_clipped, ref.s_grid, ref.track_half_width)
    return kappa, v, w


__all__ = [
    "DEFAULT_DS_REF",
    "DEFAULT_KAPPA_CLAMP",
    "DEFAULT_TRACK_HALF_WIDTH",
    "DEFAULT_EDGE_SAFETY",
    "ReferencePath",
    "build_reference_path",
    "to_curvilinear",
    "to_chassis",
    "signed_shortest",
    "kappa_at",
    "v_ref_at",
    "half_width_at",
    "sample_seq",
]
