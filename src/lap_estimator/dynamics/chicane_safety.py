"""Chicane-safety speed cap for the v3 longitudinal DP planner.

Phase 5.0.2: a localised, conservative speed cap applied to ``v_max[i]``
in tight curvature regions (default: radius < 60 m). This is a
*planner-side* fallback for the Sprint A chicane, which every controller
iteration to date (Phase 4.2 reactive, Phase 5.0 / 5.0.1 MPC) has
failed at. The DP plan already builds against the fitted Pacejka
envelope — what's been missing is a small extra margin in segments
where the controller has the least feasibility room (tight radius =>
high lateral demand => combined-slip leaves almost no longitudinal
headroom for transient correction).

This is intentionally dumb. We do *not* try to solve the controller
problem here. We give the reference plan enough margin in chicane
segments so any reasonable controller can survive them, then claw back
lap time later (lower ``safety_mult``, structural MPC work).

Design choices
--------------

- **Detection** is per-segment on ``|kappa|`` (equivalently 1/radius).
  Threshold defaults to ``radius < 60.0 m`` — Sprint A's chicane sits
  at ~27 m so a 60 m cutoff catches it with room. Configurable.
- **Multiplier** defaults to ``0.80`` — 20 % below the friction-envelope
  ``v_corner``. Applied to ``v_corner`` *before* the backward/forward
  passes in :func:`plan_longitudinal`, so the brake-feasibility and
  throttle-feasibility sweeps walk a continuous ramp around the cap
  rather than a step discontinuity.
- **Ramp-in / ramp-out** of 5 segments on either side of every flagged
  cluster. The effective multiplier on segment ``i`` is the *minimum*
  over a small triangular window: 1.0 outside the flag zone, ramping
  linearly to ``safety_mult`` at the centre. Picking ramp-then-clamp
  (rather than clamp-then-smooth) keeps the result deterministic and
  easy to reason about — segment-by-segment, we know the cap is
  ``v_corner[i] * effective_mult[i]``.

Config precedence: ``ChicaneSafetyConfig.resolve(cli, driver_json)`` —
explicit CLI override > driver JSON ``control_params.chicane`` block >
:class:`ChicaneSafetyConfig` defaults. Backwards-compat: driver JSONs
without the ``chicane`` block use defaults; passing ``None`` for the
config to :func:`plan_longitudinal` disables the cap entirely (for
regression / A-B comparison).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from ..driver import Driver


# Defaults (spec §23.2-5.0.2). The radius threshold is generous on
# Sprint A — chicane is ~27 m, threshold is 60 m — to give a wide ramp
# rather than a sharp clamp. The 0.80 multiplier is a 20 % conservative
# margin on top of the DP plan's existing ``safety_margin`` (0.94 default).
DEFAULT_RADIUS_THRESH_M = 60.0
DEFAULT_SAFETY_MULT = 0.80
RAMP_SEGMENTS = 5  # one-sided ramp width (in CSV-grid samples)


@dataclass(frozen=True)
class ChicaneSafetyConfig:
    """Tunable knobs for the chicane-safety cap.

    Attributes
    ----------
    radius_thresh_m : float
        Segments with ``radius_m < radius_thresh_m`` are flagged as
        "tight" and receive the safety multiplier. Default 60.0.
    safety_mult : float
        Conservative multiplier applied to ``v_corner[i]`` on flagged
        segments (with linear ramp-in / ramp-out on either side).
        Default 0.80 (20 % margin).
    ramp_segments : int
        One-sided ramp width in CSV-grid samples. Default 5.
    """

    radius_thresh_m: float = DEFAULT_RADIUS_THRESH_M
    safety_mult: float = DEFAULT_SAFETY_MULT
    ramp_segments: int = RAMP_SEGMENTS

    @classmethod
    def from_driver(cls, driver: "Driver | None") -> "ChicaneSafetyConfig":
        """Read ``control_params.chicane`` from a driver JSON; defaults otherwise."""
        if driver is None:
            return cls()
        raw = getattr(driver, "raw", {}) or {}
        cp = raw.get("control_params") or {}
        block = cp.get("chicane") if isinstance(cp, dict) else None
        if not isinstance(block, dict):
            return cls()
        try:
            r_thresh = float(block.get("radius_thresh_m", cls.radius_thresh_m))
        except (TypeError, ValueError):
            r_thresh = cls.radius_thresh_m
        try:
            mult = float(block.get("safety_mult", cls.safety_mult))
        except (TypeError, ValueError):
            mult = cls.safety_mult
        try:
            ramp = int(block.get("ramp_segments", cls.ramp_segments))
        except (TypeError, ValueError):
            ramp = cls.ramp_segments
        return cls(
            radius_thresh_m=max(0.0, r_thresh),
            safety_mult=float(np.clip(mult, 0.1, 1.0)),
            ramp_segments=max(0, ramp),
        )

    @classmethod
    def resolve(
        cls,
        driver: "Driver | None",
        *,
        cli_radius_thresh_m: float | None = None,
        cli_safety_mult: float | None = None,
    ) -> "ChicaneSafetyConfig":
        """Compose defaults < driver JSON < CLI overrides (highest wins)."""
        base = cls.from_driver(driver)
        return cls(
            radius_thresh_m=(
                float(cli_radius_thresh_m)
                if cli_radius_thresh_m is not None
                else base.radius_thresh_m
            ),
            safety_mult=(
                float(np.clip(cli_safety_mult, 0.1, 1.0))
                if cli_safety_mult is not None
                else base.safety_mult
            ),
            ramp_segments=base.ramp_segments,
        )


@dataclass(frozen=True)
class ChicaneSafetyReport:
    """One-shot summary of what got flagged. For logging only."""

    n_flagged: int
    flagged_ranges_m: list[tuple[float, float]]
    v_cap_min_mps: float
    config: ChicaneSafetyConfig

    def fmt_line(self) -> str:
        """One-line human-readable summary."""
        if self.n_flagged == 0:
            return (
                f"Chicane-safety: 0 segments flagged "
                f"(radius_thresh={self.config.radius_thresh_m:.1f} m, "
                f"safety_mult={self.config.safety_mult:.2f})."
            )
        ranges = ", ".join(
            f"[{lo:.0f}..{hi:.0f} m]" for lo, hi in self.flagged_ranges_m
        )
        return (
            f"Chicane-safety: {self.n_flagged} segments flagged "
            f"(s={ranges}), safety_mult={self.config.safety_mult:.2f}, "
            f"v_max cap min = {self.v_cap_min_mps:.1f} m/s."
        )


def apply_chicane_cap(
    distances: np.ndarray,
    v_corner: np.ndarray,
    kappa: np.ndarray,
    config: ChicaneSafetyConfig,
) -> tuple[np.ndarray, ChicaneSafetyReport]:
    """Apply the chicane-safety multiplier to ``v_corner`` and report flagged segments.

    Builds a per-segment effective multiplier ``mult[i]`` that is 1.0
    outside flagged zones, ramps linearly down to ``config.safety_mult``
    at the centre of each tight cluster, and ramps back up. The ramp
    uses a one-sided distance-transform style construction so that two
    nearby clusters merge into a single conservative valley (rather
    than producing a notched profile the controller will fight).

    Parameters
    ----------
    distances : np.ndarray
        Per-sample distances (m). Unused by this function directly but
        carried for the flag-range reporting.
    v_corner : np.ndarray
        Per-sample corner-limit speeds from the friction envelope
        (m/s). The cap is applied multiplicatively to this array.
    kappa : np.ndarray
        Per-sample curvature magnitude ``|1/r|`` (1/m).
    config : ChicaneSafetyConfig

    Returns
    -------
    (v_capped, report) : tuple[np.ndarray, ChicaneSafetyReport]
        ``v_capped`` is a NEW array (input is not mutated).
    """
    n = len(v_corner)
    if n == 0 or config.safety_mult >= 1.0 or config.radius_thresh_m <= 0.0:
        return v_corner.copy(), ChicaneSafetyReport(
            n_flagged=0,
            flagged_ranges_m=[],
            v_cap_min_mps=float(np.min(v_corner)) if n else 0.0,
            config=config,
        )

    kappa_thresh = 1.0 / max(config.radius_thresh_m, 1e-6)
    flagged = np.abs(kappa) >= kappa_thresh

    # Build a one-sided ramp distance: for each sample, how many samples
    # away is the nearest flagged sample? Clipped at ramp_segments+1.
    ramp = int(config.ramp_segments)
    if ramp <= 0:
        # No ramp -> straight clamp on flagged samples.
        mult = np.where(flagged, config.safety_mult, 1.0)
    else:
        # Distance (in samples) to nearest flagged sample, in both
        # directions. Two forward/backward passes give a 1-D distance
        # transform. ``BIG`` is any value >= ramp+1 (means "out of ramp").
        BIG = ramp + 1
        dist = np.where(flagged, 0, BIG).astype(np.int32)
        # Forward sweep.
        for i in range(1, n):
            if dist[i - 1] + 1 < dist[i]:
                dist[i] = dist[i - 1] + 1
        # Backward sweep.
        for i in range(n - 2, -1, -1):
            if dist[i + 1] + 1 < dist[i]:
                dist[i] = dist[i + 1] + 1
        # Linear ramp: dist=0 -> safety_mult, dist>=ramp -> 1.0.
        dist_f = np.minimum(dist.astype(float), ramp) / max(ramp, 1)
        mult = config.safety_mult + (1.0 - config.safety_mult) * dist_f

    v_capped = v_corner * mult

    # Build flagged-range report (contiguous runs of `flagged==True`).
    flagged_ranges: list[tuple[float, float]] = []
    in_run = False
    run_start_d = 0.0
    for i in range(n):
        if flagged[i] and not in_run:
            in_run = True
            run_start_d = float(distances[i])
        elif not flagged[i] and in_run:
            in_run = False
            flagged_ranges.append((run_start_d, float(distances[i - 1])))
    if in_run:
        flagged_ranges.append((run_start_d, float(distances[-1])))

    n_flagged = int(np.count_nonzero(flagged))
    v_cap_min = float(np.min(v_capped[flagged])) if n_flagged > 0 else float(np.min(v_capped))
    return v_capped, ChicaneSafetyReport(
        n_flagged=n_flagged,
        flagged_ranges_m=flagged_ranges,
        v_cap_min_mps=v_cap_min,
        config=config,
    )
