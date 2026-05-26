"""Deliberately-conservative "just finish the lap" PI baseline.

Goal: complete a lap on every Nuerburgring CSV layout without aborting.
**Speed is secondary; survival is primary.** This is the floor against
which any future v3 controller (reactive / mpc / future) is judged.

Design (per spec):

1. **Lateral reference = track centerline.** No ideal-line lookup, just the
   ``track.csv_data["x"] / [z]`` columns. Same nearest-point projection
   precedent as :mod:`pi_controller` / :mod:`driver_controller`.

2. **Longitudinal reference = 0.7 * v_max_DP.** The v3 DP planner's
   ``LongitudinalPlan.speeds`` (the post-feasibility-sweep+safety-margin
   output) is scaled by 0.7 to leave huge margin in the friction ellipse
   so combined-slip coupling can't kill us.

3. **Two independent PI loops** with very conservative gains:

   - Lateral PI on signed cross-track error ``e_lat`` -> ``delta_target``.
     Default ``Kp_lat = 0.03 rad/m``, ``Ki_lat = 0.003 rad/(m*s)``.
   - Longitudinal PI on speed error ``e_v = v_setpoint - v_actual`` ->
     a single signed ``u_lon`` that routes positive to throttle and
     negative to brake. Default ``Kp_lon = 0.10``, ``Ki_lon = 0.02``.

4. **Slew limiters on every actuator** (the key part). PI emits a *target*;
   the controller's actually-emitted value walks toward that target with
   a per-second rate cap:

   - Steering: 2.0 rad/s
   - Throttle: 2.0 /s
   - Brake: 3.0 /s

5. **Anti-windup.** Standard back-calculation on the steering integrator
   (clamp to the value that just fails to saturate); freeze-during-
   saturation on the longitudinal integrator (simpler, equally effective
   for a survival controller).

6. **Diagnostic CSV** on request: per-tick ``s, v_actual, v_setpoint, e_v,
   e_lat, delta_target, delta_emitted, throttle_target, throttle_emitted,
   brake_target, brake_emitted, util``.

All gains and slew rates are tunable via environment variables so the
gain sweep can search without recompilation. The committed defaults are
the conservative "just finish" choices; do NOT tune them to optimise
lap time.

The controller carries ``slip_target_rad`` for the simulator's util_p85
plumbing but does not consume it -- this is a pure tracking controller,
not a slip-aware one.
"""

from __future__ import annotations

import logging
import math
import os
from typing import TYPE_CHECKING

import numpy as np

from .vehicle import Controls, VehicleState

if TYPE_CHECKING:
    from ..driver import Driver
    from ..track import Track
    from .longitudinal_planner import LongitudinalPlan

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Tunable parameters with environment-variable overrides. The committed
# defaults are deliberately conservative (per spec: "speed second, survival
# first").
# ---------------------------------------------------------------------------
def _env_float(name: str, default: float) -> float:
    """Read a float from an env var; fall back to ``default`` on absence
    or parse failure."""
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


# PI gains.
KP_LAT = _env_float("SAFE_KP_LAT", 0.03)        # rad/m
KI_LAT = _env_float("SAFE_KI_LAT", 0.003)       # rad/(m*s)
KP_LON = _env_float("SAFE_KP_LON", 0.10)        # 1/(m/s)
KI_LON = _env_float("SAFE_KI_LON", 0.02)        # 1/((m/s)*s)

# Setpoint scale: longitudinal reference = SETPOINT_SCALE * v_max_DP.
SETPOINT_SCALE = _env_float("SAFE_SETPOINT_SCALE", 0.70)

# Actuator slew rates (per second). The whole point of this controller.
STEER_SLEW_RAD_S = _env_float("SAFE_STEER_SLEW", 2.0)
THROTTLE_SLEW_PER_S = _env_float("SAFE_THROTTLE_SLEW", 2.0)
BRAKE_SLEW_PER_S = _env_float("SAFE_BRAKE_SLEW", 3.0)

# Hard limits.
MAX_STEER_RAD = math.radians(20.0)  # front-axle averaged command
# Integrator clamps. Looser than pi_controller because gains are smaller;
# kept finite so the integrator can't run away after a saturation event.
I_LAT_MAX = 20.0   # m*s
I_LON_MAX = 100.0  # (m/s)*s


class SafePIController:
    """Conservative "just finish the lap" cascade PI controller.

    Same public surface as :class:`DriverController`, :class:`PIController`,
    :class:`FFPIController`:

    - ``__init__(driver, track, *, plan, ...)``.
    - ``.controls(state, t, track=None) -> Controls`` once per ODE step.

    Carries ``slip_target_rad`` for the util_p85 plumbing inside
    :func:`_run_single`. The controller itself does NOT use slip.
    """

    def __init__(
        self,
        driver: "Driver",
        track: "Track",
        *,
        plan: "LongitudinalPlan",
        rng_seed: int | None = None,  # noqa: ARG002 -- deterministic; kept for API parity
        slip_target_rad_override: float | None = None,
        log_csv_path: str | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("SafePIController needs a CSV-backed track.")
        self.driver = driver

        # Slip target carried for util_p85 plumbing only; the controller does
        # NOT modulate on slip.
        if slip_target_rad_override is not None:
            self.slip_target_rad = float(slip_target_rad_override)
        else:
            try:
                self.slip_target_rad = math.radians(
                    float(driver.derived_slip_target_deg())
                )
            except Exception:
                self.slip_target_rad = math.radians(6.0)

        # Centerline geometry (same convention as pi_controller).
        data = track.csv_data
        self._xs = np.asarray(data["x"], dtype=float)
        self._ys = np.asarray(data["z"], dtype=float)
        self._ds = np.asarray(data["distance_m"], dtype=float)
        self._total_len = float(self._ds[-1])

        # Speed setpoint: 0.7 * v_max_DP (the DP planner's post-feasibility
        # plan.speeds) resampled onto the line samples.
        v_plan = np.interp(
            self._ds,
            np.asarray(plan.distances, dtype=float),
            np.asarray(plan.speeds, dtype=float),
        )
        self._v_setpoint = SETPOINT_SCALE * v_plan

        # Sliding-window cursor for nearest-index search.
        self._idx_hint = 0

        # PI integrator state.
        self._i_lat = 0.0   # m*s
        self._i_lon = 0.0   # (m/s)*s

        # Slew-limited actuator state (rate-limited from PI targets).
        self._steer_emitted = 0.0
        self._throttle_emitted = 0.0
        self._brake_emitted = 0.0
        self._last_t = 0.0

        # Diagnostic CSV.
        self._log_csv_path = log_csv_path
        self._csv_handle = None
        if log_csv_path:
            os.makedirs(
                os.path.dirname(os.path.abspath(log_csv_path)), exist_ok=True
            )
            self._csv_handle = open(log_csv_path, "w", encoding="utf-8")
            self._csv_handle.write(
                "t,s,v_actual,v_setpoint,e_v,e_lat,"
                "delta_target,delta_emitted,"
                "throttle_target,throttle_emitted,"
                "brake_target,brake_emitted,util\n"
            )

    # ------------------------------------------------------------------
    # Public surface
    # ------------------------------------------------------------------

    def controls(
        self, state: VehicleState, t: float,
        track: "Track | None" = None,  # noqa: ARG002 -- API parity with siblings
    ) -> Controls:
        v = max(0.0, float(state.v_x))

        # Project to nearest centerline point.
        idx = self._nearest_index(state.x, state.y)
        ln_x = float(self._xs[idx])
        ln_y = float(self._ys[idx])
        s_here = float(self._ds[idx])
        tangent = self._line_tangent(idx)

        # Cross-track error: positive => car to the right of the centerline.
        # Same sign convention as driver_controller.py:244 working precedent.
        body_dx = float(state.x) - ln_x
        body_dy = float(state.y) - ln_y
        e_lat = body_dx * math.sin(tangent) - body_dy * math.cos(tangent)

        # Speed error against the 0.7 * v_max_DP setpoint.
        v_setpoint = float(self._v_setpoint[idx])
        e_v = v_setpoint - v

        # dt for integrators and slew limiters.
        if self._last_t > 0.0:
            dt = max(0.0, float(t) - self._last_t)
        else:
            dt = 0.0
        dt_slew = max(1e-3, float(t) - self._last_t)

        # ---------- lateral PI -> delta_target ----------
        self._i_lat += e_lat * dt
        # Hard-clamp integrator (cheap anti-windup floor).
        if self._i_lat > I_LAT_MAX:
            self._i_lat = I_LAT_MAX
        elif self._i_lat < -I_LAT_MAX:
            self._i_lat = -I_LAT_MAX

        delta_unclipped = KP_LAT * e_lat + KI_LAT * self._i_lat
        # Damp at standing start (matches pi_controller / driver_controller).
        if v < 5.0:
            delta_unclipped *= max(0.0, v / 5.0)

        delta_target = float(
            np.clip(delta_unclipped, -MAX_STEER_RAD, MAX_STEER_RAD)
        )

        # Back-calculation anti-windup: if delta_target saturated against the
        # steer clip, project the integrator back to the value that just
        # fails to saturate.
        if delta_target != delta_unclipped and KI_LAT > 0:
            saturated_room = (delta_target - KP_LAT * e_lat) / KI_LAT
            self._i_lat = float(
                np.clip(saturated_room, -I_LAT_MAX, I_LAT_MAX)
            )

        # ---------- longitudinal PI -> u_lon_target ----------
        self._i_lon += e_v * dt
        if self._i_lon > I_LON_MAX:
            self._i_lon = I_LON_MAX
        elif self._i_lon < -I_LON_MAX:
            self._i_lon = -I_LON_MAX

        u_lon_unclipped = KP_LON * e_v + KI_LON * self._i_lon
        # Single signed channel: positive => throttle, negative => brake.
        if u_lon_unclipped >= 0:
            throttle_target = float(np.clip(u_lon_unclipped, 0.0, 1.0))
            brake_target = 0.0
            sat_high = u_lon_unclipped > 1.0
            sat_low = False
        else:
            throttle_target = 0.0
            brake_target = float(np.clip(-u_lon_unclipped, 0.0, 1.0))
            sat_high = False
            sat_low = u_lon_unclipped < -1.0

        # Anti-windup (freeze-on-saturation): if we saturated in the
        # direction of the current error, undo this step's integration so the
        # integrator doesn't keep winding against a closed valve.
        if (sat_high and e_v > 0) or (sat_low and e_v < 0):
            self._i_lon -= e_v * dt

        # Standing-start bypass: without this, v3 burns 5-10 s pulling away
        # from rest because PI alone can't move the pedal off the floor.
        if v < 1.0 and v_setpoint > 2.0:
            throttle_target = 1.0
            brake_target = 0.0

        # ---------- slew-limit the emitted actuator values ----------
        delta_emitted = _slew(
            self._steer_emitted, delta_target,
            STEER_SLEW_RAD_S * dt_slew,
        )
        throttle_emitted = _slew(
            self._throttle_emitted, throttle_target,
            THROTTLE_SLEW_PER_S * dt_slew,
        )
        brake_emitted = _slew(
            self._brake_emitted, brake_target,
            BRAKE_SLEW_PER_S * dt_slew,
        )

        # Clamp emitted values to physically valid ranges. The slew step can
        # nudge throttle/brake a hair past [0,1] in degenerate dt cases.
        throttle_emitted = float(np.clip(throttle_emitted, 0.0, 1.0))
        brake_emitted = float(np.clip(brake_emitted, 0.0, 1.0))
        delta_emitted = float(np.clip(delta_emitted, -MAX_STEER_RAD, MAX_STEER_RAD))

        # ---------- diagnostic logging ----------
        if self._csv_handle is not None:
            util = (
                abs(e_lat) / max(1.0, abs(v_setpoint))
                if v_setpoint > 0 else 0.0
            )
            self._csv_handle.write(
                f"{t:.4f},{s_here:.3f},{v:.3f},{v_setpoint:.3f},"
                f"{e_v:.4f},{e_lat:.4f},"
                f"{delta_target:.5f},{delta_emitted:.5f},"
                f"{throttle_target:.4f},{throttle_emitted:.4f},"
                f"{brake_target:.4f},{brake_emitted:.4f},"
                f"{util:.4f}\n"
            )

        # Bookkeeping.
        self._steer_emitted = delta_emitted
        self._throttle_emitted = throttle_emitted
        self._brake_emitted = brake_emitted
        self._last_t = float(t)

        return Controls(
            steer_rad=delta_emitted,
            throttle=throttle_emitted,
            brake=brake_emitted,
        )

    def close(self) -> None:
        """Close the diagnostic CSV handle, if one was opened."""
        if self._csv_handle is not None:
            try:
                self._csv_handle.close()
            finally:
                self._csv_handle = None

    def __del__(self) -> None:
        # Best-effort cleanup; ignore failures during interpreter shutdown.
        try:
            self.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _nearest_index(self, x: float, y: float) -> int:
        """Sliding-window nearest-point search (matches pi_controller)."""
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

    def _line_tangent(self, idx: int) -> float:
        """Central-difference tangent at line index ``idx``."""
        n = len(self._xs)
        i0 = max(0, idx - 2)
        i1 = min(n - 1, idx + 2)
        if i1 <= i0:
            i1 = min(n - 1, i0 + 1)
        return float(np.arctan2(
            self._ys[i1] - self._ys[i0],
            self._xs[i1] - self._xs[i0],
        ))


def _slew(prev: float, target: float, max_step: float) -> float:
    """Walk ``prev`` toward ``target`` by at most ``max_step`` (absolute)."""
    if max_step <= 0:
        return prev
    delta = target - prev
    if delta > max_step:
        return prev + max_step
    if delta < -max_step:
        return prev - max_step
    return target


__all__ = [
    "SafePIController",
    "KP_LAT", "KI_LAT", "KP_LON", "KI_LON",
    "SETPOINT_SCALE",
    "STEER_SLEW_RAD_S", "THROTTLE_SLEW_PER_S", "BRAKE_SLEW_PER_S",
]
