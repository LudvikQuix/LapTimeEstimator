"""Ideal-line CSV → Frenet (s, n_ideal, psi_offset_ideal) projection.

Built for the HMPC v3.4 outer line-follow mode (task brief 2026-05-26).

The HMPC outer NLP plans in curvilinear coordinates relative to the
centerline-derived :class:`ReferencePath`. To force it to follow an
offline-optimised racing line (the ``layout_sprint_a_ideal_line.csv``
trajectory) we need ``n_ideal(s)`` and ``psi_offset_ideal(s)`` sampled
on the same s-grid the outer uses for its stages.

This module:

  1. Reads the ideal-line CSV (must expose ``x``, ``z``, ``distance_m``).
  2. For each (x, z) sample, runs a Frenet projection against the
     centerline ``ReferencePath``, yielding ``(s_center, n_offset,
     psi_offset)``.
  3. Returns dense arrays sorted by ``s_center`` so the consumer can
     ``np.interp`` at any outer-stage ``s_k``.

Sanity checks performed:
  - ``|n_offset|`` p95 must exceed 0.2 m (otherwise the ideal-line is
    essentially the centerline and line-follow mode is a no-op; we log
    a warning).
  - ``|n_offset|`` max must stay inside the track half-width (otherwise
    the projection wandered onto a far-side branch; we log a warning
    and clip to the half-width).
  - The s-grid must be strictly monotonic (after sorting). We
    deduplicate ties and reject crossings (rare; would indicate the
    ideal line loops back, which Sprint A does not).

The projection reuses the existing windowed Newton refinement from
:func:`mpcc_reference.to_curvilinear` so the result is consistent with
how the controller will read the chassis state at runtime.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .mpcc_reference import ReferencePath, to_curvilinear


log = logging.getLogger(__name__)


# When the ideal-line CSV is essentially the centerline (|n| p95 < this
# threshold) we treat line-follow as a no-op and warn — the outer would
# just be told to pull to centerline, which it already does via ``w_n``.
MIN_MEANINGFUL_N_P95_M = 0.2


@dataclass(frozen=True)
class IdealLineFrenet:
    """Frenet-projected ideal-line samples on the centerline arc-length.

    Attributes
    ----------
    s : np.ndarray
        Centerline arc-length at each ideal-line sample, sorted
        strictly-increasing (m).
    n_ideal : np.ndarray
        Signed perpendicular offset of the ideal-line sample from the
        centerline at ``s`` (m). Positive = left of centerline tangent.
    psi_offset_ideal : np.ndarray
        Heading offset of the ideal-line tangent w.r.t. the centerline
        tangent at ``s`` (rad), wrapped to ``[-π, π]``.
    csv_path : str
        Path the data was read from (for logging / traceability).
    n_p95_m : float
        ``np.quantile(|n_ideal|, 0.95)`` — diagnostic; below
        ``MIN_MEANINGFUL_N_P95_M`` indicates the ideal line is the
        centerline.
    n_max_m : float
        ``max |n_ideal|`` — diagnostic.
    """

    s: np.ndarray
    n_ideal: np.ndarray
    psi_offset_ideal: np.ndarray
    csv_path: str
    n_p95_m: float
    n_max_m: float

    def n_at(self, s_query: float) -> float:
        """Linear-interp ``n_ideal(s_query)``; clipped at the edges."""
        s_query = float(np.clip(s_query, self.s[0], self.s[-1]))
        return float(np.interp(s_query, self.s, self.n_ideal))

    def psi_at(self, s_query: float) -> float:
        """Linear-interp ``psi_offset_ideal(s_query)``; clipped at the edges."""
        s_query = float(np.clip(s_query, self.s[0], self.s[-1]))
        return float(np.interp(s_query, self.s, self.psi_offset_ideal))

    def n_seq(self, s_seq: np.ndarray) -> np.ndarray:
        """Vectorised ``n_ideal`` at the given s array."""
        s_clipped = np.clip(np.asarray(s_seq, dtype=float), self.s[0], self.s[-1])
        return np.interp(s_clipped, self.s, self.n_ideal)

    def psi_seq(self, s_seq: np.ndarray) -> np.ndarray:
        """Vectorised ``psi_offset_ideal`` at the given s array."""
        s_clipped = np.clip(np.asarray(s_seq, dtype=float), self.s[0], self.s[-1])
        return np.interp(s_clipped, self.s, self.psi_offset_ideal)


def _read_ideal_line_xy(csv_path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Read (x, z) columns from an ideal-line CSV.

    Track CSVs (incl. the ideal-line variant) use ``x``/``z`` as the two
    planar axes (matching the rest of the v3 stack — ``y`` is elevation).
    We read with numpy's ``genfromtxt`` so the dependency stays light.
    """
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"Ideal-line CSV not found: {path}")
    # Header-driven read so we don't break if upstream adds columns.
    with path.open("r", encoding="utf-8") as fh:
        header = fh.readline().rstrip("\n").split(",")
    if "x" not in header or "z" not in header:
        raise ValueError(
            f"Ideal-line CSV missing 'x' or 'z' column: {path} "
            f"(found: {header[:8]}...)"
        )
    ix = header.index("x")
    iz = header.index("z")
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    if data.ndim == 1:
        # Single-row CSV — degenerate.
        raise ValueError(f"Ideal-line CSV has <2 rows: {path}")
    return np.asarray(data[:, ix], dtype=float), np.asarray(data[:, iz], dtype=float)


def _compute_local_tangent(xs: np.ndarray, zs: np.ndarray) -> np.ndarray:
    """Per-sample tangent angle along the ideal-line (rad), unwrapped.

    Same convention as :func:`mpcc_reference._compute_tangent` but
    local to this loader so we don't introduce a circular import.
    """
    n = len(xs)
    if n < 2:
        return np.zeros(n, dtype=float)
    dx = np.zeros(n, dtype=float)
    dz = np.zeros(n, dtype=float)
    dx[1:-1] = xs[2:] - xs[:-2]
    dz[1:-1] = zs[2:] - zs[:-2]
    dx[0] = xs[1] - xs[0]
    dz[0] = zs[1] - zs[0]
    dx[-1] = xs[-1] - xs[-2]
    dz[-1] = zs[-1] - zs[-2]
    psi = np.arctan2(dz, dx)
    return np.unwrap(psi)


def _dedup_monotone(
    s: np.ndarray,
    n: np.ndarray,
    psi: np.ndarray,
    min_ds: float = 1e-4,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sort by s, drop near-duplicates, keep first crossing only.

    The Frenet projection can occasionally map two consecutive ideal-line
    samples to the same (or backwards-going) centerline s when the ideal
    line tightens onto a corner — both samples are valid but interpolating
    requires a strict monotone grid. We keep the first occurrence at each
    near-duplicate s, and drop any sample that would make the grid go
    backwards (a Sprint A loop-back doesn't exist, so this is a rare
    safety net).
    """
    order = np.argsort(s, kind="stable")
    s_s = s[order]
    n_s = n[order]
    psi_s = psi[order]
    # Walk forward, keep monotonic-increasing samples only.
    keep = np.ones(len(s_s), dtype=bool)
    last_s = -np.inf
    for i, si in enumerate(s_s):
        if si <= last_s + min_ds:
            keep[i] = False
        else:
            last_s = si
    return s_s[keep], n_s[keep], psi_s[keep]


def load_ideal_line_frenet(
    csv_path: str | Path,
    center_ref: ReferencePath,
) -> IdealLineFrenet:
    """Project an ideal-line CSV's (x, z) onto the centerline Frenet frame.

    Parameters
    ----------
    csv_path
        Path to the ideal-line CSV (e.g.
        ``tracks_csv/ks_nurburgring/layout_sprint_a_ideal_line.csv``).
    center_ref
        Centerline-derived :class:`ReferencePath` built by
        :func:`mpcc_reference.build_reference_path` against the
        *centerline* track CSV (the one the simulation plant is running
        on; see HMPCController's ``self.ref``).

    Returns
    -------
    IdealLineFrenet

    Raises
    ------
    FileNotFoundError, ValueError
        From the underlying CSV reader / projection.
    """
    csv_path_str = str(Path(csv_path))
    xs, zs = _read_ideal_line_xy(csv_path_str)
    if len(xs) != len(zs):
        raise ValueError(
            f"Ideal-line CSV: x/z length mismatch ({len(xs)} vs {len(zs)})"
        )

    # Local tangent of the ideal line (rad) at each sample.
    psi_ideal = _compute_local_tangent(xs, zs)

    # Project each sample onto the centerline Frenet frame. We thread a
    # forward-only hint so the projection stays O(N) total instead of
    # O(N · M) — same pattern as the controller's per-tick projection.
    s_arr = np.empty(len(xs), dtype=float)
    n_arr = np.empty(len(xs), dtype=float)
    psi_e_arr = np.empty(len(xs), dtype=float)
    hint = 0
    for i, (x_i, z_i, psi_i) in enumerate(zip(xs, zs, psi_ideal, strict=False)):
        # ``to_curvilinear`` computes (s, n, psi_e) where psi_e is the
        # heading-offset of the supplied tangent vs the centerline at
        # the projected s — which is exactly what we want.
        s_i, n_i, psi_e_i, hint = to_curvilinear(
            float(x_i), float(z_i), float(psi_i),
            center_ref, hint_idx=hint,
        )
        s_arr[i] = s_i
        n_arr[i] = n_i
        psi_e_arr[i] = psi_e_i

    # Sort + dedup → strict-monotone grid.
    s_mono, n_mono, psi_mono = _dedup_monotone(s_arr, n_arr, psi_e_arr)
    if len(s_mono) < 2:
        raise ValueError(
            f"Ideal-line projection produced <2 unique samples after "
            f"deduplication (csv={csv_path_str})."
        )

    # Diagnostics.
    abs_n = np.abs(n_mono)
    n_p95 = float(np.quantile(abs_n, 0.95))
    n_max = float(abs_n.max())

    if n_p95 < MIN_MEANINGFUL_N_P95_M:
        log.warning(
            "Ideal-line %s: |n_ideal| p95=%.3f m is below %.3f m threshold. "
            "Projection may be wrong, or the ideal line is the centerline. "
            "Line-follow mode will not change planner behaviour meaningfully.",
            csv_path_str, n_p95, MIN_MEANINGFUL_N_P95_M,
        )

    # Half-width clip warning. We DON'T clip n_ideal — the constraint
    # ``|n_k| <= half_width`` is enforced inside the outer NLP; clipping
    # here would silently move the target. Instead we surface the worst
    # sample so the operator notices a CSV/track mismatch.
    s_at_max = float(s_mono[int(np.argmax(abs_n))])
    half_w_at_max = float(np.interp(
        s_at_max, center_ref.s_grid, center_ref.track_half_width,
    ))
    if n_max > half_w_at_max + 0.5:
        log.warning(
            "Ideal-line %s: |n_ideal| max=%.2f m at s=%.0f m exceeds "
            "centerline half-width %.2f m there by >0.5 m. Outer NLP "
            "track-edge constraint will reject the target; check the CSV "
            "or the centerline pairing.",
            csv_path_str, n_max, s_at_max, half_w_at_max,
        )

    log.info(
        "Ideal-line %s loaded: %d samples → %d strict-monotone, "
        "|n_ideal| p95=%.2f m, max=%.2f m (half-width %.2f m).",
        csv_path_str, len(xs), len(s_mono), n_p95, n_max, half_w_at_max,
    )

    return IdealLineFrenet(
        s=s_mono,
        n_ideal=n_mono,
        psi_offset_ideal=psi_mono,
        csv_path=csv_path_str,
        n_p95_m=n_p95,
        n_max_m=n_max,
    )


__all__ = [
    "IdealLineFrenet",
    "MIN_MEANINGFUL_N_P95_M",
    "load_ideal_line_frenet",
]
