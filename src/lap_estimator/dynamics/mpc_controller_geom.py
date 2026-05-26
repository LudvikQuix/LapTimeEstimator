"""Track-line geometry helpers for the MPC controller.

Extracted from :mod:`mpc_controller` to keep the main controller module
under the 500-line soft cap (CLAUDE.md). These are pure projection /
curvature helpers shared between the MPC's reference-trajectory build
(``_resolve_mpc``) and the chassis-divergence check (``controls``).

All helpers operate on per-controller cached arrays (``xs``, ``ys``,
``ds``, ``radius_m``) passed in by the caller. The ``GeometryState``
dataclass holds the rolling index hint so successive nearest-point
projections stay cheap (O(window) instead of O(n_samples)).
"""

from __future__ import annotations

import math

import numpy as np


class GeometryState:
    """Mutable cache for the nearest-point projection hint.

    Lightweight: a single int. The MPCController owns one instance and
    threads it through every call so successive projections only scan
    a ~200-sample window around the prior result.
    """

    __slots__ = ("idx_hint",)

    def __init__(self) -> None:
        self.idx_hint: int = 0


def nearest_index(
    xs: np.ndarray,
    ys: np.ndarray,
    x: float, y: float,
    state: GeometryState,
) -> int:
    """Return the index of the racing-line sample closest to ``(x, y)``.

    Uses ``state.idx_hint`` as the centre of the search window so
    successive calls cost O(window) ~ 200 samples instead of O(N) ~
    3000 samples per step. Updates the hint as a side effect.
    """
    n = len(xs)
    lo = max(0, state.idx_hint - 5)
    hi = min(n, state.idx_hint + 200)
    seg_xs = xs[lo:hi]
    seg_ys = ys[lo:hi]
    d2 = (seg_xs - x) ** 2 + (seg_ys - y) ** 2
    j = int(np.argmin(d2))
    idx = lo + j
    state.idx_hint = idx
    return idx


def line_tangent(xs: np.ndarray, ys: np.ndarray, idx: int) -> float:
    """Local tangent direction (rad) at racing-line sample ``idx``."""
    n = len(xs)
    i0 = max(0, idx - 2)
    i1 = min(n - 1, idx + 2)
    if i1 <= i0:
        i1 = min(n - 1, i0 + 1)
    return float(np.arctan2(ys[i1] - ys[i0], xs[i1] - xs[i0]))


def kappa_at(
    s_seq: np.ndarray,
    ds: np.ndarray,
    radius_m: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
) -> np.ndarray:
    """Return signed curvature 1/r at each distance-along-track in ``s_seq``.

    Sign convention: positive kappa = left turn (positive yaw rate
    required to follow). The CSV stores ``radius_m`` as unsigned; we
    sign it from the local tangent's rotational sense over a 30 m
    stencil — a tighter stencil produces noisy sign flips on near-
    straight samples (verified empirically: at 10 m stencil the sign
    flips ~every 20 m on the Sprint A 2000 m-radius "straight").

    Below a magnitude threshold (radius > 500 m → kappa < 2e-3) we
    zero out the curvature entirely so the linearised plant doesn't
    chase phantom yaw demand. The MPC uses cross-track error to absorb
    the missing micro-curvature.
    """
    radius = np.interp(s_seq, ds, radius_m)
    STRAIGHT_RADIUS_M = 500.0
    active = np.abs(radius) < STRAIGHT_RADIUS_M
    kappa_mag = np.where(active, 1.0 / np.maximum(np.abs(radius), 1.0), 0.0)
    sign_arr = np.ones_like(s_seq)
    n = len(ds)
    ds_avg = float(ds[-1] - ds[0]) / max(n - 1, 1)
    i_step = max(2, int(round(15.0 / ds_avg)))
    for k, s in enumerate(s_seq):
        if not active[k]:
            continue
        i = int(np.searchsorted(ds, s))
        i = max(i_step, min(n - 1 - i_step, i))
        t0 = line_tangent(xs, ys, i - i_step)
        t1 = line_tangent(xs, ys, i + i_step)
        d = math.atan2(math.sin(t1 - t0), math.cos(t1 - t0))
        sign_arr[k] = 1.0 if d >= 0 else -1.0
    return sign_arr * kappa_mag


__all__ = [
    "GeometryState",
    "nearest_index",
    "line_tangent",
    "kappa_at",
]
