"""Deliberately-dumb PI baseline controller for the v3 slip-based simulator.

Purpose: **reference smoke-test** for the physics envelope. After 7 phases of
MPC iteration failing at the Sprint A chicane, we need a control-side
baseline so simple it can't possibly be the bug. If a pure cascade PI
controller (no preview, no slip-awareness, no Stanley, no MPC) completes
the lap at the DP-planned speeds, then the physics envelope IS feasible and
the MPC failures are MPC-side bugs. If PI also dies at the chicane, the
envelope is infeasible and we need a Pacejka refit or a different track.

Design:

1. **Lateral (steering)** -- PI on cross-track error vs centerline / ideal
   line. Error ``e_lat`` is the signed perpendicular distance from car to
   the nearest line point. Sign matches the existing
   :class:`DriverController` cross convention (positive ``cross`` ->
   positive steer correction) so the PI law is
   ``steer = +(Kp_lat * e_lat + Ki_lat * integral)``. The spec writes a
   leading minus, but that's relative to "right of line = positive"; in
   this codebase ``cross > 0`` already needs a positive corrective steer.
   See :mod:`driver_controller` line ~258 for the working precedent.

2. **Longitudinal (throttle/brake)** -- PI on speed error
   ``e_lon = v_target(s) - v_actual``, where ``v_target`` comes from the
   v3 DP plan (the SAME plan the MPC consumes). Output ``u_lon`` is split
   into throttle (positive) or brake (negative). Anti-windup clamps the
   integrator on saturation.

3. **No coupling between loops.** Steering knows nothing about speed.
   Throttle/brake knows nothing about cross-track. No slip awareness, no
   feedforward, no fallback, no ghost path. If the QP / MPC pipeline can
   do better than this, the *control* delta is real. If they can't, the
   problem is physics or the plan.

4. **Diagnostic CSV.** On request (``log_csv_path=...``), every tick is
   appended to ``.tmp/pi_diag_<feature>.csv`` with columns
   ``[t, s, x, y, v, v_target, e_lat, e_lon, steer, throttle, brake,
   i_lat, i_lon]`` so we can tune gains offline without re-running the sim.
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
# Default gains. Hand-picked starting point; tune empirically per diagnostic
# run, do NOT tune-to-pass an acceptance gate (this controller is a
# diagnostic instrument, not a production target).
#
# Lateral: a cross-track error of 1 m at v=30 m/s should produce ~0.05 rad
# (~3 deg) of steer command. Integrator adds another 0.01 rad/m per second
# of sustained offset (will saturate quickly on a real corner, but the
# anti-windup clamp keeps it sane).
#
# Longitudinal: 10 m/s speed error commits 1.0 throttle (saturates) when
# accelerating; 5 m/s overspeed commits 1.0 brake (saturates) when slowing.
# Integrator gain 0.05 means ~1 s of sustained 1 m/s error saturates the
# pedal. Anti-windup on saturation.
# ---------------------------------------------------------------------------
def _env_float(name: str, default: float) -> float:
    """Read a float from an env var; fall back to ``default`` on absence
    or parse failure. Lets the diagnostic sweep tune gains without editing
    source.
    """
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


# Spec defaults. Env-var overrides (PI_KP_LAT etc.) let the gain sweep
# adjust them without recompilation; they are still constants for the
# lifetime of any one process.
KP_LAT = _env_float("PI_KP_LAT", 0.05)            # rad/m
KI_LAT = _env_float("PI_KI_LAT", 0.01)            # rad/(m*s)
KP_THROTTLE = _env_float("PI_KP_THROTTLE", 0.1)   # 1/(m/s)
KP_BRAKE = _env_float("PI_KP_BRAKE", 0.2)         # 1/(m/s)
KI_LON = _env_float("PI_KI_LON", 0.05)            # 1/((m/s)*s)

# Hard limits.
MAX_STEER_RAD = math.radians(20.0)  # front-axle averaged command
MAX_STEER_RATE_RAD_S = 10.0         # rad/s slew limit
# Integrator clamps (anti-windup).
I_LAT_MAX = 5.0  # m*s -- caps the lateral integrator contribution
I_LON_MAX = 50.0  # (m/s)*s -- caps the longitudinal integrator contribution


class PIController:
    """Cascade PI controller -- diagnostic reference baseline.

    Same public surface as :class:`DriverController` and
    :class:`MPCController`:

    - ``__init__`` consumes a :class:`Driver`, :class:`Track`, and the v3
      longitudinal plan (``plan.distances``, ``plan.speeds``).
    - ``.controls(state, t, track=None) -> Controls`` -- called once per
      ODE step.

    Carries ``slip_target_rad`` for :func:`_run_single` 's util_p85
    plumbing (we still want that metric in the diagnostic output even
    though the controller itself doesn't use it).
    """

    def __init__(
        self,
        driver: "Driver",
        track: "Track",
        *,
        plan: "LongitudinalPlan",
        rng_seed: int | None = None,  # noqa: ARG002 -- unused; PI is deterministic
        slip_target_rad_override: float | None = None,
        log_csv_path: str | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("PIController needs a CSV-backed track.")
        self.driver = driver

        # Slip target carried for util_p85 plumbing only; the controller
        # does NOT modulate on slip. Picks the driver's derived value
        # unless overridden (MC runs).
        if slip_target_rad_override is not None:
            self.slip_target_rad = float(slip_target_rad_override)
        else:
            try:
                self.slip_target_rad = math.radians(
                    float(driver.derived_slip_target_deg())
                )
            except Exception:
                self.slip_target_rad = math.radians(6.0)

        # Line geometry.
        data = track.csv_data
        self._xs = np.asarray(data["x"], dtype=float)
        self._ys = np.asarray(data["z"], dtype=float)
        self._ds = np.asarray(data["distance_m"], dtype=float)
        self._total_len = float(self._ds[-1])

        # Target speed plan interpolated onto the line samples (matches
        # DriverController convention).
        self._speed = np.interp(
            self._ds,
            np.asarray(plan.distances, dtype=float),
            np.asarray(plan.speeds, dtype=float),
        )

        # Sliding-window cursor for nearest-index search (same trick as
        # DriverController so we don't pay O(N) per step).
        self._idx_hint = 0

        # PI integrator state.
        self._i_lat = 0.0   # m*s
        self._i_lon = 0.0   # (m/s)*s

        # Rate-limit / continuity state.
        self._last_steer = 0.0
        self._last_t = 0.0

        # Diagnostic CSV.
        self._log_csv_path = log_csv_path
        self._csv_handle = None
        if log_csv_path:
            # Ensure parent directory exists. Caller is expected to point
            # at .tmp/ per the project's scratch hygiene rules.
            os.makedirs(os.path.dirname(os.path.abspath(log_csv_path)), exist_ok=True)
            self._csv_handle = open(log_csv_path, "w", encoding="utf-8")
            self._csv_handle.write(
                "t,s,x,y,v,v_target,e_lat,e_lon,steer,throttle,brake,i_lat,i_lon\n"
            )

    # ------------------------------------------------------------------
    # Public surface
    # ------------------------------------------------------------------

    def controls(self, state: VehicleState, t: float,
                 track: "Track | None" = None) -> Controls:  # noqa: ARG002
        v = max(0.0, float(state.v_x))

        # Project to nearest line point.
        idx = self._nearest_index(state.x, state.y)
        ln_x = float(self._xs[idx])
        ln_y = float(self._ys[idx])
        s_here = float(self._ds[idx])
        tangent = self._line_tangent(idx)

        # Cross-track error: positive => car to the *right* of the line in
        # the existing codebase's convention. Same formula as
        # DriverController._controls() so the sign is consistent with
        # working precedent.
        body_dx = float(state.x) - ln_x
        body_dy = float(state.y) - ln_y
        e_lat = body_dx * math.sin(tangent) - body_dy * math.cos(tangent)

        # Speed error.
        v_target = float(self._speed[idx])
        e_lon = v_target - v

        # dt for the integrator. First call: skip integration (t=0).
        dt = max(0.0, float(t) - self._last_t) if self._last_t > 0 else 0.0

        # --- lateral PI ---------------------------------------------------
        # Integrate first (forward Euler), then form output, then anti-windup
        # clamp on saturation.
        self._i_lat += e_lat * dt
        # Hard-clamp the integrator BEFORE applying it -- this is the
        # simplest anti-windup that still respects the gain. Symmetric.
        if self._i_lat > I_LAT_MAX:
            self._i_lat = I_LAT_MAX
        elif self._i_lat < -I_LAT_MAX:
            self._i_lat = -I_LAT_MAX

        steer_unclipped = KP_LAT * e_lat + KI_LAT * self._i_lat
        # Damp at standing start so the integrator doesn't kick at t=0 from
        # any random startup transient.
        if v < 5.0:
            steer_unclipped *= max(0.0, v / 5.0)

        steer_cmd = float(np.clip(steer_unclipped, -MAX_STEER_RAD, MAX_STEER_RAD))

        # If we saturated, bleed the integrator back so it doesn't keep
        # winding up against a closed clamp.
        if steer_cmd != steer_unclipped and abs(self._i_lat) > 0:
            # Pure linear bleed: project the integrator back to whatever
            # value would just fail to saturate.
            saturated_room = (steer_cmd - KP_LAT * e_lat) / max(KI_LAT, 1e-9)
            self._i_lat = float(np.clip(saturated_room, -I_LAT_MAX, I_LAT_MAX))

        # Steering rate limit.
        dt_ctrl = max(1e-3, float(t) - self._last_t)
        max_step = MAX_STEER_RATE_RAD_S * dt_ctrl
        d_steer = steer_cmd - self._last_steer
        if d_steer > max_step:
            steer_cmd = self._last_steer + max_step
        elif d_steer < -max_step:
            steer_cmd = self._last_steer - max_step

        # --- longitudinal PI ---------------------------------------------
        self._i_lon += e_lon * dt
        if self._i_lon > I_LON_MAX:
            self._i_lon = I_LON_MAX
        elif self._i_lon < -I_LON_MAX:
            self._i_lon = -I_LON_MAX

        # Combined PI output. Positive => want throttle; negative => brake.
        # The integrator gain is the same on both sides (KI_LON) so a
        # sustained speed deficit can saturate either pedal.
        u_lon_p = KP_THROTTLE * e_lon
        u_lon_b = KP_BRAKE * e_lon  # negative when overspeed; we flip sign below
        i_term = KI_LON * self._i_lon

        if e_lon >= 0:
            # Accelerating: throttle = clip(Kp_throttle*e + Ki*i, 0, 1).
            u = u_lon_p + i_term
            throttle = float(np.clip(u, 0.0, 1.0))
            brake = 0.0
            saturated_high = u > 1.0
            saturated_low = u < 0.0
        else:
            # Braking: brake = clip(-Kp_brake*e - Ki*i, 0, 1). The integrator
            # is negative under overspeed, so -Ki*i is positive.
            u = -u_lon_b - i_term
            brake = float(np.clip(u, 0.0, 1.0))
            throttle = 0.0
            saturated_high = u > 1.0
            saturated_low = u < 0.0

        # Anti-windup on longitudinal: if we saturated, freeze the integrator
        # in the direction that would worsen the saturation. Simple back-
        # calculation would be more elegant; freeze is good enough for a
        # diagnostic baseline.
        if (saturated_high and e_lon > 0) or (saturated_low and e_lon < 0):
            # Undo this step's integration -- we'd just keep winding the
            # integrator against a closed valve.
            self._i_lon -= e_lon * dt

        # Soft start: standing-start dead zone bypass (matches
        # DriverController). Without this v3 burns 5-10 s pulling away from
        # rest because the integrator can't move the pedal off the floor.
        if v < 1.0 and v_target > 2.0:
            throttle = 1.0
            brake = 0.0

        # Diagnostic logging.
        if self._csv_handle is not None:
            self._csv_handle.write(
                f"{t:.4f},{s_here:.3f},{state.x:.3f},{state.y:.3f},"
                f"{v:.3f},{v_target:.3f},{e_lat:.4f},{e_lon:.4f},"
                f"{steer_cmd:.5f},{throttle:.4f},{brake:.4f},"
                f"{self._i_lat:.4f},{self._i_lon:.4f}\n"
            )

        # Bookkeeping.
        self._last_steer = float(steer_cmd)
        self._last_t = float(t)

        return Controls(
            steer_rad=float(steer_cmd),
            throttle=float(throttle),
            brake=float(brake),
        )

    def close(self) -> None:
        """Close the diagnostic CSV handle, if one was opened.

        Caller can invoke this after a sim run to flush the file. Also
        called implicitly via the finaliser, but explicit is better for
        deterministic IO order in batch runs.
        """
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
        """Sliding-window nearest-point search; matches DriverController."""
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


__all__ = ["PIController", "KP_LAT", "KI_LAT", "KP_THROTTLE", "KP_BRAKE", "KI_LON"]
