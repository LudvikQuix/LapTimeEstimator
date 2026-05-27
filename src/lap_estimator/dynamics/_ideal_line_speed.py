"""Ideal-line CSV → speed / longitudinal-accel reference on the plant ``s`` grid.

Built for the HMPC ideal-line bypass mode (spec
``dev-planning/hmpc-ideal-line-bypass/spec.md``, 2026-05-27).

The bypass mode deletes the HMPC outer NLP from the loop and feeds the inner
tracker a reference built directly from the offline-optimised ideal-line CSV.
:mod:`_ideal_line_loader` already supplies the lateral channels
(``n_ref``/``psi_e_ref``) by projecting the ideal line onto the *plant*
:class:`ReferencePath` Frenet frame. This module supplies the longitudinal
channels:

  - ``v_ref(s)``  — the CSV ``speed_ms`` column, and
  - ``a_long_ref(s) = v · dv/ds`` — derived once via ``np.gradient`` (the same
    kinematic identity and sign convention as
    :class:`_dp_reference.DPLongitudinalReference`; decel is negative).

The load-bearing requirement (spec §7.2) is that the speed reference lives on
the **plant** arc-length ``s`` grid, not the ideal line's own ``distance_m``,
because the inner stage grid ``s_seq = s_now + k·ds_stage`` is on plant ``s``.
We obtain ``s_plant`` per CSV row by reusing the exact projection + dedup that
:func:`_ideal_line_loader.load_ideal_line_frenet` performs, carrying the
``speed_ms`` column through the identical row dedup so the resulting
``s_plant`` grid is byte-identical to ``IdealLineFrenet.s`` and all four inner
reference channels are consistent at every ``s_seq``.

On Arm B (plant CSV *is* the ideal-line CSV) the projection is a near-identity
self-projection, so ``s_plant`` ≈ the ideal line's own ``distance_m`` and the
re-index is a no-op; the same code path serves both arms with no branch.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ._ideal_line_loader import (
    _compute_local_tangent,
    _dedup_monotone,
    _read_ideal_line_xy,
)
from .mpcc_reference import ReferencePath, to_curvilinear

log = logging.getLogger(__name__)

__all__ = ["IdealLineSpeedReference"]


def _read_speed_ms(csv_path: str | Path) -> np.ndarray:
    """Read the ``speed_ms`` column from an ideal-line CSV (m/s)."""
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"Ideal-line CSV not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        header = fh.readline().rstrip("\n").split(",")
    if "speed_ms" not in header:
        raise ValueError(
            f"Ideal-line CSV missing 'speed_ms' column: {path} "
            f"(found: {header[:8]}...)"
        )
    isp = header.index("speed_ms")
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    if data.ndim == 1:
        raise ValueError(f"Ideal-line CSV has <2 rows: {path}")
    return np.asarray(data[:, isp], dtype=float)


@dataclass(frozen=True)
class IdealLineSpeedReference:
    """Cached ``v_ref(s)`` / ``a_long_ref(s)`` from the ideal-line CSV speed column.

    Both arrays are on the **plant** arc-length grid ``s_plant`` — the same grid
    :class:`_ideal_line_loader.IdealLineFrenet.s` produces, so the longitudinal
    and lateral references are sampled consistently.

    Attributes
    ----------
    s_plant : np.ndarray
        Plant-frame arc-length at each surviving ideal-line sample (m, strictly
        increasing). Identical to ``IdealLineFrenet.s`` for the same CSV/plant.
    v_ref : np.ndarray
        CSV ``speed_ms`` re-indexed onto ``s_plant`` (m/s).
    a_long : np.ndarray
        ``v · dv/ds`` on ``s_plant`` (m/s²). Decel is negative.
    csv_path : str
        Source CSV path (for logging / traceability).
    integrated_lap_s : float
        ``∫ ds/v`` over ``s_plant`` — the CSV's own integrated lap time (s).
        Diagnostic; expected ≈ 110.88 s for Sprint A.
    """

    s_plant: np.ndarray
    v_ref: np.ndarray
    a_long: np.ndarray
    csv_path: str
    integrated_lap_s: float

    # -- vectorised accessors (np.interp, clipped at the grid edges) --

    def v_at(self, s: float) -> float:
        """Linear-interp ``v_ref(s)``; clipped at the grid edges."""
        return float(np.interp(
            float(np.clip(s, self.s_plant[0], self.s_plant[-1])),
            self.s_plant, self.v_ref,
        ))

    def a_long_at(self, s: float) -> float:
        """Linear-interp ``a_long(s)``; clipped at the grid edges."""
        return float(np.interp(
            float(np.clip(s, self.s_plant[0], self.s_plant[-1])),
            self.s_plant, self.a_long,
        ))

    def v_seq(self, s_seq: np.ndarray) -> np.ndarray:
        """Vectorised ``v_ref`` at the given s array (clipped at the edges)."""
        s_clipped = np.clip(
            np.asarray(s_seq, dtype=float), self.s_plant[0], self.s_plant[-1],
        )
        return np.interp(s_clipped, self.s_plant, self.v_ref)

    def a_long_seq(self, s_seq: np.ndarray) -> np.ndarray:
        """Vectorised ``a_long`` at the given s array (clipped at the edges)."""
        s_clipped = np.clip(
            np.asarray(s_seq, dtype=float), self.s_plant[0], self.s_plant[-1],
        )
        return np.interp(s_clipped, self.s_plant, self.a_long)


def load_ideal_line_speed(
    csv_path: str | Path,
    plant_ref: ReferencePath,
) -> IdealLineSpeedReference:
    """Build an :class:`IdealLineSpeedReference` from an ideal-line CSV.

    Re-runs the same Frenet projection + row dedup as
    :func:`_ideal_line_loader.load_ideal_line_frenet` so the resulting
    ``s_plant`` grid is identical to ``IdealLineFrenet.s``, then carries the
    CSV ``speed_ms`` column through that dedup and derives
    ``a_long = v · dv/ds`` on the dedup-monotone plant grid.

    Parameters
    ----------
    csv_path
        Ideal-line CSV path (must expose ``x``, ``z``, ``speed_ms``).
    plant_ref
        The plant :class:`ReferencePath` (centerline for Arm A, ideal line for
        Arm B). The same object passed to ``load_ideal_line_frenet``.

    Raises
    ------
    FileNotFoundError, ValueError
        From the CSV reader / projection.
    """
    csv_path_str = str(Path(csv_path))
    xs, zs = _read_ideal_line_xy(csv_path_str)
    speed = _read_speed_ms(csv_path_str)
    if not (len(xs) == len(zs) == len(speed)):
        raise ValueError(
            f"Ideal-line CSV: x/z/speed_ms length mismatch "
            f"({len(xs)}/{len(zs)}/{len(speed)}) in {csv_path_str}"
        )

    psi_ideal = _compute_local_tangent(xs, zs)

    # Project each sample onto the plant Frenet frame (same forward-only hint
    # as the loader) to get its plant-s. We carry the row index into the
    # ``psi`` slot of _dedup_monotone purely so dedup keeps speed row-aligned;
    # to avoid abusing that, we dedup (s, speed) ourselves with the identical
    # rule the loader uses.
    s_arr = np.empty(len(xs), dtype=float)
    hint = 0
    for i, (x_i, z_i, psi_i) in enumerate(zip(xs, zs, psi_ideal, strict=False)):
        s_i, _n_i, _psi_e_i, hint = to_curvilinear(
            float(x_i), float(z_i), float(psi_i), plant_ref, hint_idx=hint,
        )
        s_arr[i] = s_i

    # Dedup speed against the identical monotone rule used for n/psi. We pass
    # ``speed`` in both trailing slots so the returned arrays are the
    # row-deduped (s_plant, speed); the second copy is discarded.
    s_plant, v_plant, _ = _dedup_monotone(s_arr, speed, speed)
    if len(s_plant) < 2:
        raise ValueError(
            f"Ideal-line speed projection produced <2 unique samples after "
            f"dedup (csv={csv_path_str})."
        )

    v_plant = np.maximum(v_plant, 1e-3)  # guard ∫ds/v
    dv_ds = np.gradient(v_plant, s_plant)
    a_long = v_plant * dv_ds

    # Integrated lap-time estimate ∫ ds/v over the plant grid (trapezoid on 1/v).
    ds = np.diff(s_plant)
    inv_v_mid = 0.5 * (1.0 / v_plant[:-1] + 1.0 / v_plant[1:])
    integrated_lap_s = float(np.sum(ds * inv_v_mid))

    log.info(
        "Ideal-line speed %s loaded: %d samples -> %d strict-monotone on plant "
        "s (v in [%.1f, %.1f] m/s); integrated lap est = %.2f s.",
        csv_path_str, len(xs), len(s_plant),
        float(v_plant.min()), float(v_plant.max()), integrated_lap_s,
    )

    return IdealLineSpeedReference(
        s_plant=np.asarray(s_plant, dtype=float),
        v_ref=np.asarray(v_plant, dtype=float),
        a_long=np.asarray(a_long, dtype=float),
        csv_path=csv_path_str,
        integrated_lap_s=integrated_lap_s,
    )
