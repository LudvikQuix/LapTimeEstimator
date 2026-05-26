"""Feedforward-dominant + bounded-PI controller for the v3 slip-based simulator.

Classic racing-control pattern: precompute the bulk of the control signal from
track geometry (steering FF from Ackermann + curvature; throttle/brake FF from
the planned ``a_x_plan(s)``) and let a small PI trim correction handle the
residual cross-track + speed error. The PI delta is clipped to ``+/- 30 %`` of
the FF magnitude (with a small absolute floor on straights), so the controller
can never "fight" the plan beyond a known envelope.

Why this exists
---------------

Phase 5.0.x MPC iterations failed at the Sprint A chicane; the deliberately-
dumb cascade :class:`PIController` aborted earlier still (s ~ 638 m, util
~ 0.13 -- it never even *tried* to reach the envelope). The question this
controller answers, decisively:

* If FF+PI completes Sprint A at a reasonable time, the physics envelope is
  fine and the previous MPC complexity was unjustified.
* If FF+PI aborts at the chicane, the envelope is genuinely tight there
  (Pacejka refit or track switch needed).
* If FF+PI aborts elsewhere, the DP plan has a separate aggressiveness
  problem at that location.

Diagnostic CSV (``log_csv_path``) captures per-tick
``s, v, v_target, e_lat, delta_FF, dDelta_PI, delta, throttle_FF, brake_FF,
dU_PI, throttle, brake, util`` so the verdict can be checked from the trace
without re-running.

Sign / convention notes
-----------------------

* Cross-track sign matches :mod:`driver_controller` and :mod:`pi_controller`:
  positive ``e_lat`` ⇒ car right of line ⇒ positive corrective steer
  (see ``driver_controller.py`` ~ line 244).
* Signed curvature is derived from the centreline tangent: ``kappa =
  d psi_line / ds`` where ``psi_line = arctan2(dz, dx)``. This produces a
  ``delta_FF`` whose sign matches the PI's cross-track sign convention,
  so the two channels reinforce rather than fight. If the diagnostic CSV
  ever shows them fighting on a straight + steady-state corner, the FF
  sign is wrong and must be flipped here (this module only -- never touch
  the truth model).

Gains and envelope
------------------

* Default gains (env-var overridable): ``Kp_lat=0.05``, ``Ki_lat=0.01``,
  ``Kp_lon=0.20``, ``Ki_lon=0.05`` per spec.
* Bound: ``|dDelta| <= max(0.3 * |delta_FF|, delta_min_floor=0.02 rad)``;
  ``|dU| <= max(0.3 * max(|throttle_FF|, |brake_FF|), u_min_floor=0.05)``.
* Anti-windup: integrators are clamped when the bounded output saturates.
"""

from __future__ import annotations

import logging
import math
import os
from typing import TYPE_CHECKING

import numpy as np

from .vehicle import Controls, VehicleState

if TYPE_CHECKING:
    from ..car import Car
    from ..driver import Driver
    from ..track import Track
    from .longitudinal_planner import LongitudinalPlan

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Gains -- env-var overridable. Same trick as :class:`PIController` so a
# CLI sweep can tune without source edits.
# ---------------------------------------------------------------------------
def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


KP_LAT = _env_float("FFPI_KP_LAT", 0.05)
KI_LAT = _env_float("FFPI_KI_LAT", 0.01)
KP_LON = _env_float("FFPI_KP_LON", 0.20)
KI_LON = _env_float("FFPI_KI_LON", 0.05)

# Envelope of the PI delta as a fraction of |FF|, plus an absolute floor so
# the PI can still act on straights (where FF ~ 0).
PI_BOUND_FRAC = 0.30
DELTA_MIN_FLOOR_RAD = 0.02
U_MIN_FLOOR = 0.05

# FF inverse-model parameters. ``a_x_max_drive`` is the M1 RWD-limited
# longitudinal accel ceiling; ``a_x_max_brake`` is the sticky-Pacejka
# braking ceiling. Both are scalar approximations -- the PI trim covers
# residual model error.
G = 9.81
A_X_MAX_DRIVE = 0.4 * G
A_X_MAX_BRAKE = 1.1 * G

# Hard limits / rate limit -- match :class:`PIController` so cross-controller
# comparisons are apples-to-apples.
MAX_STEER_RAD = math.radians(20.0)
MAX_STEER_RATE_RAD_S = 10.0
I_LAT_MAX = 5.0
I_LON_MAX = 50.0


class FFPIController:
    """Feedforward + bounded-PI controller (sibling of :class:`PIController`).

    Same public surface as :class:`DriverController`, :class:`MPCController`,
    :class:`PIController`: ``controls(state, t, track=None)`` returns a
    :class:`Controls`. Carries ``slip_target_rad`` for the ``util_p85``
    plumbing in :func:`_run_single`.
    """

    def __init__(
        self,
        driver: "Driver",
        track: "Track",
        car: "Car",
        *,
        plan: "LongitudinalPlan",
        rng_seed: int | None = None,  # noqa: ARG002 -- deterministic
        slip_target_rad_override: float | None = None,
        log_csv_path: str | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("FFPIController needs a CSV-backed track.")
        self.driver = driver

        # Slip target carried for util_p85 plumbing only; the controller
        # does NOT modulate on slip (FF closes that loop implicitly through
        # the DP plan, PI never sees a slip signal).
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
        self._radius = np.asarray(data["radius_m"], dtype=float)
        self._total_len = float(self._ds[-1])

        # Target speed (from DP plan) interpolated onto the line samples.
        self._speed = np.interp(
            self._ds,
            np.asarray(plan.distances, dtype=float),
            np.asarray(plan.speeds, dtype=float),
        )

        # Precompute the three FF arrays (steer, throttle, brake) per line
        # sample. They are constants for the lifetime of this controller.
        L = float(getattr(car, "wheelbase", 2.66))
        self._wheelbase = L
        self._car = car
        self._delta_ff = self._compute_steer_ff(L)
        self._a_x_plan = self._compute_a_x_plan()
        self._throttle_ff, self._brake_ff = self._compute_lon_ff(self._a_x_plan)

        # Sliding-window cursor (matches DriverController / PIController).
        self._idx_hint = 0

        # PI integrator + continuity state.
        self._i_lat = 0.0
        self._i_lon = 0.0
        self._last_steer = 0.0
        self._last_t = 0.0

        # Diagnostic CSV.
        self._log_csv_path = log_csv_path
        self._csv_handle = None
        if log_csv_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_csv_path)), exist_ok=True)
            self._csv_handle = open(log_csv_path, "w", encoding="utf-8")
            self._csv_handle.write(
                "t,s,v,v_target,e_lat,e_lon,delta_ff,d_delta_pi,delta,"
                "throttle_ff,brake_ff,d_u_pi,throttle,brake\n"
            )

    # ------------------------------------------------------------------
    # FF precomputation
    # ------------------------------------------------------------------

    def _compute_steer_ff(self, wheelbase_m: float) -> np.ndarray:
        """Geometric Ackermann FF: ``delta_FF(s) = arctan(L * kappa_signed(s))``.

        ``kappa_signed`` is the centreline curvature derived from the
        unwrapped tangent angle ``psi_line = arctan2(dz, dx)``:
        ``kappa = d psi_line / ds``. This captures even very gentle
        (x, z) curvature that the CSV's ``radius_m`` column masks with
        the 2000 m "straight" sentinel -- and gentle curves matter
        because the 30 %-of-FF PI envelope cannot generate enough
        authority by itself to follow them.

        Sign convention: positive ``kappa_signed`` matches the cross-
        track PI's "positive cross => positive corrective steer" so the
        two channels reinforce rather than fight. To suppress
        per-sample noise in ``psi_line``, we smooth it with a 5-sample
        moving average before differentiating.
        """
        n = len(self._xs)
        if n < 3:
            return np.zeros(n)
        psi_line = np.unwrap(np.arctan2(
            np.gradient(self._ys), np.gradient(self._xs),
        ))
        # Light smoothing on psi_line to suppress per-sample noise; the
        # CSV is sampled at ~1.6 m so a 5-sample window is ~8 m, much
        # shorter than any real corner.
        kernel = np.ones(5) / 5.0
        psi_smooth = np.convolve(psi_line, kernel, mode="same")
        ds = np.gradient(self._ds)
        ds_safe = np.where(np.abs(ds) > 1e-6, ds, 1e-6)
        kappa_signed = np.gradient(psi_smooth) / ds_safe
        # Cap kappa at the CSV's own radius magnitude so genuinely sharp
        # corners (where the CSV radius is the ground truth) aren't
        # over-driven by tangent-noise. The CSV radius is always >=
        # 1.0 m on these tracks.
        kappa_csv_cap = 1.0 / np.clip(np.abs(self._radius), 1.0, None)
        kappa_signed = np.sign(kappa_signed) * np.minimum(
            np.abs(kappa_signed), kappa_csv_cap,
        )
        delta = np.arctan(wheelbase_m * kappa_signed)
        return np.clip(delta, -MAX_STEER_RAD, MAX_STEER_RAD)

    def _compute_a_x_plan(self) -> np.ndarray:
        """Compute planned longitudinal accel ``a_x_plan(s) = v * dv/ds``."""
        v = self._speed
        ds = np.gradient(self._ds)
        ds_safe = np.where(np.abs(ds) > 1e-6, ds, 1e-6)
        dv_ds = np.gradient(v) / ds_safe
        return v * dv_ds

    def _compute_lon_ff(self, a_x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Map planned ``a_x`` to (throttle_FF, brake_FF) in [0, 1].

        Physics-aware inverse model -- the scalar ``a_x / 0.4g`` mapping
        from the spec literal under-commands the pedal at high speed
        (the BMW 1M's full-throttle net accel is only ~0.05 g at 65 m/s
        because of gearing + aero drag), which leaves PI with nothing
        to trim against. Instead:

        * Required tractive force per sample:
          ``F_req = m * a_x_plan + F_drag(v_plan) + F_rolling(v_plan)``.
        * ``throttle_FF = F_req / F_traction_max(v_plan)`` clipped to
          [0, 1]. PI absorbs any state-vs-plan-speed mismatch.
        * ``brake_FF = -a_x / A_X_MAX_BRAKE`` for negative ``a_x``. We
          intentionally don't discount brake_FF for drag (drag aids
          braking, so the FF runs slightly hot -- PI trims it back).

        All FF is precomputed per-line-sample at the planned speed, so
        per-tick lookup is a single ``self._throttle_ff[idx]`` with no
        closed-loop dependence on the actual state.
        """
        v_plan = self._speed
        m = float(getattr(self._car, "total_mass", 1500.0))
        F_drag = np.array([
            float(self._car.drag_force(max(float(v), 1.0))) for v in v_plan
        ])
        F_rr = np.array([
            float(self._car.rolling_resistance(max(float(v), 1.0))) for v in v_plan
        ])
        F_tract_max = np.array([
            float(self._car.max_traction_force(max(float(v), 1.0))) for v in v_plan
        ])
        F_req = m * a_x + F_drag + F_rr
        F_req = np.where(a_x >= 0, F_req, 0.0)
        throttle = np.clip(F_req / np.maximum(F_tract_max, 1.0), 0.0, 1.0)
        brake = np.clip(-a_x / A_X_MAX_BRAKE, 0.0, 1.0)
        # Mutually exclusive (defensive).
        both = (throttle > 0) & (brake > 0)
        throttle = np.where(both, 0.0, throttle)
        return throttle, brake

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
        tangent_now = self._line_tangent(idx)

        # Cross-track error (positive => car right of line; same convention
        # as DriverController / PIController).
        body_dx = float(state.x) - ln_x
        body_dy = float(state.y) - ln_y
        e_lat = body_dx * math.sin(tangent_now) - body_dy * math.cos(tangent_now)

        # Speed error.
        v_target = float(self._speed[idx])
        e_lon = v_target - v

        # FF lookups.
        delta_ff = float(self._delta_ff[idx])
        throttle_ff = float(self._throttle_ff[idx])
        brake_ff = float(self._brake_ff[idx])

        dt = max(0.0, float(t) - self._last_t) if self._last_t > 0 else 0.0

        # ----- lateral PI trim -------------------------------------------
        self._i_lat = float(np.clip(
            self._i_lat + e_lat * dt, -I_LAT_MAX, I_LAT_MAX,
        ))
        d_delta_raw = KP_LAT * e_lat + KI_LAT * self._i_lat
        # Damp at standing start.
        if v < 5.0:
            d_delta_raw *= max(0.0, v / 5.0)
        bound_lat = max(PI_BOUND_FRAC * abs(delta_ff), DELTA_MIN_FLOOR_RAD)
        d_delta = float(np.clip(d_delta_raw, -bound_lat, bound_lat))
        # Anti-windup: if we clipped, bleed the integrator back.
        if d_delta != d_delta_raw and KI_LAT > 1e-9:
            target_i = (d_delta - KP_LAT * e_lat) / KI_LAT
            self._i_lat = float(np.clip(target_i, -I_LAT_MAX, I_LAT_MAX))

        steer_unclipped = delta_ff + d_delta
        steer_cmd = float(np.clip(
            steer_unclipped, -MAX_STEER_RAD, MAX_STEER_RAD,
        ))

        # Steering rate limit.
        dt_ctrl = max(1e-3, float(t) - self._last_t)
        max_step = MAX_STEER_RATE_RAD_S * dt_ctrl
        d_steer = steer_cmd - self._last_steer
        if d_steer > max_step:
            steer_cmd = self._last_steer + max_step
        elif d_steer < -max_step:
            steer_cmd = self._last_steer - max_step

        # ----- longitudinal PI trim --------------------------------------
        self._i_lon = float(np.clip(
            self._i_lon + e_lon * dt, -I_LON_MAX, I_LON_MAX,
        ))
        d_u_raw = KP_LON * e_lon + KI_LON * self._i_lon
        bound_lon = max(
            PI_BOUND_FRAC * max(throttle_ff, brake_ff),
            U_MIN_FLOOR,
        )
        d_u = float(np.clip(d_u_raw, -bound_lon, bound_lon))
        # Anti-windup: if we clipped, bleed the integrator.
        if d_u != d_u_raw and KI_LON > 1e-9:
            target_i = (d_u - KP_LON * e_lon) / KI_LON
            self._i_lon = float(np.clip(target_i, -I_LON_MAX, I_LON_MAX))

        # Apply PI delta on top of FF, with bounded crossover at the
        # throttle/brake boundary: positive d_u extends throttle (or
        # cancels brake first); negative d_u extends brake (or cancels
        # throttle first). The +/- bound_lon envelope ensures we can
        # never command more than ~30 % beyond the FF in either pedal.
        net = throttle_ff - brake_ff + d_u  # signed pedal axis, +throttle/-brake
        if net >= 0:
            throttle = float(np.clip(net, 0.0, 1.0))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(np.clip(-net, 0.0, 1.0))

        # Soft start: standing-start dead-zone bypass (matches PIController).
        if v < 1.0 and v_target > 2.0:
            throttle = 1.0
            brake = 0.0

        # Diagnostic logging.
        if self._csv_handle is not None:
            self._csv_handle.write(
                f"{t:.4f},{s_here:.3f},{v:.3f},{v_target:.3f},"
                f"{e_lat:.4f},{e_lon:.4f},"
                f"{delta_ff:.5f},{d_delta:.5f},{steer_cmd:.5f},"
                f"{throttle_ff:.4f},{brake_ff:.4f},{d_u:.4f},"
                f"{throttle:.4f},{brake:.4f}\n"
            )

        self._last_steer = float(steer_cmd)
        self._last_t = float(t)

        return Controls(
            steer_rad=float(steer_cmd),
            throttle=float(throttle),
            brake=float(brake),
        )

    def close(self) -> None:
        """Close the diagnostic CSV handle if one was opened."""
        if self._csv_handle is not None:
            try:
                self._csv_handle.close()
            finally:
                self._csv_handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal helpers (mirror PIController for consistency).
    # ------------------------------------------------------------------

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

    def _line_tangent(self, idx: int) -> float:
        n = len(self._xs)
        i0 = max(0, idx - 2)
        i1 = min(n - 1, idx + 2)
        if i1 <= i0:
            i1 = min(n - 1, i0 + 1)
        return float(np.arctan2(
            self._ys[i1] - self._ys[i0],
            self._xs[i1] - self._xs[i0],
        ))


__all__ = [
    "FFPIController",
    "KP_LAT",
    "KI_LAT",
    "KP_LON",
    "KI_LON",
    "PI_BOUND_FRAC",
    "DELTA_MIN_FLOOR_RAD",
    "U_MIN_FLOOR",
    "A_X_MAX_DRIVE",
    "A_X_MAX_BRAKE",
]
