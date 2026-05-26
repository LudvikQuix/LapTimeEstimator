"""Phase-3 ghost driver — preview-line follower with Stanley steering.

Kept reachable in Phase 4 as (a) a regression-test target reachable via
:func:`simulate_slip(use_ghost=True)`, and (b) the in-controller safety
fallback used by :class:`driver_controller.DriverController` for any
single timestep where the Phase-4 commands would push the abort guards.

The ghost is intentionally separate from the Phase-4 :class:`ControlParams`
because the two have different parameter spaces — the ghost has its own
hand-tuned Stanley constants (rate limit, cross-track gain, max steer)
that the spec §23.5.2 JSON schema does not describe.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .vehicle import Controls, VehicleState

if TYPE_CHECKING:
    from ..track import Track


@dataclass(frozen=True)
class GhostControlParams:
    """Phase-3 GhostDriver tuning constants (kept for back-compat).

    Distinct from the spec §23.5.2 :class:`ControlParams` so a driver
    JSON's ``control_params`` block doesn't accidentally retune the
    ghost driver.
    """

    preview_distance_m: float = 18.0
    steering_p_gain: float = 1.2
    throttle_p_gain: float = 0.5
    brake_p_gain: float = 0.6
    slip_target_lat_deg: float = 6.0
    consistency_noise_std_steer_deg: float = 0.3
    consistency_noise_std_throttle_pct: float = 1.5
    measured: bool = False


class GhostDriver:
    """Phase-3 ghost driver: preview-line follower with Stanley + P controllers.

    Steering is Stanley-style (heading-err + cross-track / (v + soft))
    and rate-limited. Throttle / brake are P-controllers on
    (target_speed - v_x), with target speed taken as the minimum upcoming
    racing-line speed inside a braking-distance window.
    """

    def __init__(
        self,
        track: "Track",
        *,
        preview_distance_m: float = 10.0,
        steering_p_gain: float = 2.5,
        throttle_p_gain: float = 0.3,
        brake_p_gain: float = 0.35,
        max_steer_rad: float | None = None,
        max_steer_rate_rad_s: float = 8.0,
        stanley_k_cross: float = 0.5,
        target_speed_scale: float = 1.0,
        target_speed_ds: np.ndarray | None = None,
        target_speeds: np.ndarray | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("GhostDriver needs a CSV-backed track (Phase 3).")
        data = track.csv_data
        # AC coords: (x, z) = horizontal plane, y = elevation. Solver matches.
        self._xs = np.asarray(data["x"], dtype=float)
        self._ys = np.asarray(data["z"], dtype=float)
        self._ds = np.asarray(data["distance_m"], dtype=float)
        if target_speed_ds is not None and target_speeds is not None:
            self._speed = np.interp(
                self._ds,
                np.asarray(target_speed_ds, dtype=float),
                np.asarray(target_speeds, dtype=float),
            )
        else:
            self._speed = np.asarray(data["speed_ms"], dtype=float)
        self._total_len = float(self._ds[-1])
        self.preview_distance_m = float(preview_distance_m)
        self.steering_p_gain = float(steering_p_gain)
        self.throttle_p_gain = float(throttle_p_gain)
        self.brake_p_gain = float(brake_p_gain)
        self.max_steer_rad = float(
            max_steer_rad if max_steer_rad is not None else np.deg2rad(15.0)
        )
        self.max_steer_rate_rad_s = float(max_steer_rate_rad_s)
        self.stanley_k_cross = float(stanley_k_cross)
        self.target_speed_scale = float(target_speed_scale)
        self._idx_hint = 0
        self._last_steer = 0.0
        self._last_t = 0.0

    def _nearest_index(self, x: float, y: float) -> int:
        n = len(self._xs)
        lo = max(0, self._idx_hint - 5)
        hi = min(n, self._idx_hint + 200)
        seg_xs = self._xs[lo:hi]
        seg_ys = self._ys[lo:hi]
        d2 = (seg_xs - x) ** 2 + (seg_ys - y) ** 2
        j = int(np.argmin(d2))
        idx = lo + j
        self._idx_hint = idx
        return idx

    def controls(self, state: VehicleState, t: float,
                 track: "Track | None" = None) -> Controls:  # noqa: ARG002
        idx = self._nearest_index(state.x, state.y)
        v = float(state.v_x)

        lookahead_idx = self._lookahead_idx(idx, self.preview_distance_m)
        tangent_psi = self._line_tangent(lookahead_idx)
        line_x = float(self._xs[idx])
        line_y = float(self._ys[idx])
        dx = state.x - line_x
        dy = state.y - line_y
        cross = float(dx * np.sin(tangent_psi) - dy * np.cos(tangent_psi))

        psi_err = float(np.arctan2(np.sin(tangent_psi - state.psi),
                                   np.cos(tangent_psi - state.psi)))

        softening = 3.0
        steer_target = psi_err + float(np.arctan2(
            self.stanley_k_cross * cross, v + softening))
        if v < 5.0:
            steer_target *= v / 5.0
        steer_target = float(np.clip(steer_target,
                                     -self.max_steer_rad, self.max_steer_rad))
        dt_ctrl = max(1e-3, t - self._last_t)
        max_step = self.max_steer_rate_rad_s * dt_ctrl
        delta_steer = steer_target - self._last_steer
        if delta_steer > max_step:
            steer_rad = self._last_steer + max_step
        elif delta_steer < -max_step:
            steer_rad = self._last_steer - max_step
        else:
            steer_rad = steer_target
        self._last_steer = steer_rad
        self._last_t = t

        brake_lookahead_m = max(20.0, v * v / (2.0 * 5.0))
        v_target = self._min_speed_within(idx, brake_lookahead_m)

        e_v = v_target - v
        throttle = float(np.clip(self.throttle_p_gain * max(e_v, 0.0), 0.0, 1.0))
        brake = float(np.clip(self.brake_p_gain * max(-e_v, 0.0), 0.0, 1.0))
        body_slip_rad = float(np.arctan2(state.v_y, max(abs(state.v_x), 1.0)))
        if abs(body_slip_rad) > np.deg2rad(15.0):
            throttle = 0.0
            brake = 1.0
        if v < 1.0 and v_target > 2.0:
            throttle = 1.0
            brake = 0.0
        return Controls(steer_rad=steer_rad, throttle=throttle, brake=brake)

    def _line_tangent(self, idx: int) -> float:
        n = len(self._xs)
        i0 = max(0, idx - 2)
        i1 = min(n - 1, idx + 2)
        if i1 <= i0:
            i1 = min(n - 1, i0 + 1)
        return float(np.arctan2(self._ys[i1] - self._ys[i0],
                                self._xs[i1] - self._xs[i0]))

    def _lookahead_idx(self, idx: int, dist_m: float) -> int:
        s_here = float(self._ds[idx])
        s_target = min(s_here + dist_m, self._total_len - 1e-3)
        j = int(np.searchsorted(self._ds, s_target))
        return max(idx, min(j, len(self._ds) - 1))

    def _min_speed_within(self, idx: int, lookahead_m: float) -> float:
        s_here = float(self._ds[idx])
        s_end = min(s_here + lookahead_m, self._total_len - 1e-3)
        end_idx = int(np.searchsorted(self._ds, s_end))
        end_idx = max(idx + 1, min(end_idx, len(self._ds) - 1))
        return float(self._speed[idx:end_idx + 1].min()) * self.target_speed_scale
