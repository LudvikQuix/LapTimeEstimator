"""HMPC PI-trim layer (spec §23.4.6.4).

Bounded proportional-integral correction on cross-track (``e_n``) and
longitudinal-speed (``e_vx``) error against the outer's reference. The
trim's authority is hard-capped at ±15 % of the steering envelope and
±10 % of the pedal envelope so it cannot override the inner MPC's
commit — it only nudges. Anti-windup on the integrators when the
output saturates.

This module is intentionally light (no controller dependencies) so it
can be re-used by future hierarchical-MPC variants without dragging the
v3.2 stack along.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Defaults — spec §23.4.6.4.
DEFAULT_PI_KP_N = 0.05
DEFAULT_PI_KI_N = 0.01
DEFAULT_PI_KP_VX = 0.02
DEFAULT_PI_KI_VX = 0.005
DEFAULT_PI_BOUND_STEER_FRAC = 0.15
DEFAULT_PI_BOUND_PEDAL_ABS = 0.10


@dataclass
class PITrimConfig:
    """Tunables for the PI trim (spec §23.4.6.4 defaults)."""

    K_p_n: float = DEFAULT_PI_KP_N
    K_i_n: float = DEFAULT_PI_KI_N
    K_p_vx: float = DEFAULT_PI_KP_VX
    K_i_vx: float = DEFAULT_PI_KI_VX
    bound_steer_frac: float = DEFAULT_PI_BOUND_STEER_FRAC
    bound_pedal_abs: float = DEFAULT_PI_BOUND_PEDAL_ABS


class PITrim:
    """Bounded PI correction on cross-track and v_x error.

    Inputs are the curvilinear errors ``e_n = n_actual − n_ref`` and
    ``e_vx = v_x_actual − v_ref``. Outputs are additive trims:

      δ_trim    = clip(K_p_n  · e_n  + K_i_n  · ∫e_n  dt, ±bound_steer)
      thr_trim  = clip(−K_p_vx · e_vx − K_i_vx · ∫e_vx dt, ±bound_pedal)
      brk_trim  = clip(+K_p_vx · e_vx + K_i_vx · ∫e_vx dt, ±bound_pedal)

    Anti-windup: the integral is rolled back the moment its
    proportional + integral sum saturates at the bound.

    Per-tick |trim| histories are accumulated so the controller can
    surface p95 metrics to ``SlipSimResult``.
    """

    def __init__(
        self,
        cfg: PITrimConfig,
        *,
        delta_max: float,
        tick_period: float,
    ) -> None:
        self.cfg = cfg
        self.delta_max = float(delta_max)
        self.tick_period = float(tick_period)
        self._i_n = 0.0
        self._i_vx = 0.0
        self.steer_trim_hist: list[float] = []
        self.thr_trim_hist: list[float] = []
        self.brk_trim_hist: list[float] = []

    def apply(
        self, *, e_n: float, e_vx: float,
    ) -> tuple[float, float, float]:
        """Return ``(steer_trim_rad, thr_trim, brk_trim)``."""
        dt = self.tick_period
        cfg = self.cfg
        self._i_n += float(e_n) * dt
        self._i_vx += float(e_vx) * dt
        steer_bound = cfg.bound_steer_frac * self.delta_max
        steer_raw = cfg.K_p_n * float(e_n) + cfg.K_i_n * self._i_n
        steer_trim = float(np.clip(steer_raw, -steer_bound, steer_bound))
        # Anti-windup steering: roll the integrator back when we're at
        # the bound AND the raw command agrees in sign (so the next
        # tick's e_n would push further into the bound).
        if abs(steer_trim) >= steer_bound - 1e-9 and steer_trim * steer_raw > 0:
            self._i_n -= float(e_n) * dt
        thr_raw = -(cfg.K_p_vx * float(e_vx) + cfg.K_i_vx * self._i_vx)
        brk_raw = +(cfg.K_p_vx * float(e_vx) + cfg.K_i_vx * self._i_vx)
        thr_trim = float(np.clip(thr_raw, -cfg.bound_pedal_abs, cfg.bound_pedal_abs))
        brk_trim = float(np.clip(brk_raw, -cfg.bound_pedal_abs, cfg.bound_pedal_abs))
        if (abs(thr_trim) >= cfg.bound_pedal_abs - 1e-9
                or abs(brk_trim) >= cfg.bound_pedal_abs - 1e-9):
            self._i_vx -= float(e_vx) * dt
        self.steer_trim_hist.append(abs(steer_trim))
        self.thr_trim_hist.append(abs(thr_trim))
        self.brk_trim_hist.append(abs(brk_trim))
        return steer_trim, thr_trim, brk_trim

    def p95(self) -> dict[str, float]:
        """Return p95 |trim| values; zeros when no history."""
        out = {"steer": 0.0, "throttle": 0.0, "brake": 0.0}
        if self.steer_trim_hist:
            out["steer"] = float(np.quantile(self.steer_trim_hist, 0.95))
        if self.thr_trim_hist:
            out["throttle"] = float(np.quantile(self.thr_trim_hist, 0.95))
        if self.brk_trim_hist:
            out["brake"] = float(np.quantile(self.brk_trim_hist, 0.95))
        return out


__all__ = [
    "DEFAULT_PI_KP_N",
    "DEFAULT_PI_KI_N",
    "DEFAULT_PI_KP_VX",
    "DEFAULT_PI_KI_VX",
    "DEFAULT_PI_BOUND_STEER_FRAC",
    "DEFAULT_PI_BOUND_PEDAL_ABS",
    "PITrim",
    "PITrimConfig",
]
