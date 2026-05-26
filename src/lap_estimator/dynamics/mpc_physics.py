"""Phase 5.0.4 dynamic-Fz config + helpers (spec §23.2-5.0.4.8).

A small, JSON-serialisable dataclass that captures the per-driver
overrides for the Phase 5.0.4 dynamic-Fz plant. Loaded from the
``control_params.mpc`` block alongside the existing horizon / weights
knobs; defaults reproduce the static-Fz Phase 5.0.3 behaviour when the
driver JSON omits the block.

Kept in a sibling module so :mod:`mpc_controller` stays close to its
500-line soft cap.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MpcPhysicsConfig:
    """Per-driver MPC plant-model overrides (Phase 5.0.4)."""

    dynamic_fz_enabled: bool = True
    cg_height_m: float | None = None       # None -> CarDynamics.h_cg
    track_width_f_m: float | None = None   # None -> CarDynamics.track_f
    track_width_r_m: float | None = None   # None -> CarDynamics.track_r

    @classmethod
    def from_mpc_block(cls, block: dict | None) -> "MpcPhysicsConfig":
        """Pull ``mpc.dynamic_fz_enabled``, ``mpc.cg_height_m``, ``mpc.track_width_*_m``.

        ``None`` / missing fields keep the dataclass defaults. ``None``
        on the geometry fields means "fall through to ``CarDynamics``"
        and is preserved through the dataclass (not coerced to a number
        until the controller does the merge against ``dyn``).
        """
        if not isinstance(block, dict):
            return cls()
        def _opt_float(name: str) -> float | None:
            val = block.get(name)
            if val is None:
                return None
            try:
                return float(val)
            except (TypeError, ValueError):
                return None
        return cls(
            dynamic_fz_enabled=bool(block.get("dynamic_fz_enabled", True)),
            cg_height_m=_opt_float("cg_height_m"),
            track_width_f_m=_opt_float("track_width_f_m"),
            track_width_r_m=_opt_float("track_width_r_m"),
        )

    def force_static(self) -> "MpcPhysicsConfig":
        """Return a copy with ``dynamic_fz_enabled=False`` (CLI ``--static-fz``)."""
        return MpcPhysicsConfig(
            dynamic_fz_enabled=False,
            cg_height_m=self.cg_height_m,
            track_width_f_m=self.track_width_f_m,
            track_width_r_m=self.track_width_r_m,
        )


__all__ = ["MpcPhysicsConfig"]
