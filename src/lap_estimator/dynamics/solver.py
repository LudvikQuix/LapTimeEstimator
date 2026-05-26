"""Time-domain ODE integrator for the slip-based dynamics model (spec §23.3, §23.6.3).

Phase 3: hand-rolled RK4 at fixed ``dt = 5 ms``. The LSODA fallback path is
v3.1 (kept as an unimplemented hook here to preserve the Phase 1 surface).

Lap-completion detection: project ``(x, y)`` onto the track ideal-line each
step (nearest-point on the supplied ``track_xy`` polyline) → distance-along
track ``s``. The lap is complete when normalised ``s`` crosses 1.0 with
hysteresis (must have been below 0.5 within the last second).

Numerical-stability guards (spec §23.6.5):
- ``StalledError`` if ``v_x < 0.5 m/s`` for >2 s of integrated time.
- ``SpunError`` if ``|omega_yaw| > 5 rad/s``.
- ``NumericalError`` on NaN/Inf in any state component.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import TYPE_CHECKING, Callable

import numpy as np

from .vehicle import (
    CarDynamics,
    Controls,
    PacejkaCalibration,
    VehicleState,
    compute_derivatives,
)

if TYPE_CHECKING:
    from ..car import Car
    from ..track import Track
    from ..tyre_state import Compound


class StalledError(RuntimeError):
    """Vehicle stalled: ``v_x < 0.5 m/s`` for more than 2 s of integrated time."""


class SpunError(RuntimeError):
    """Vehicle spun: ``|omega_yaw| > 5 rad/s``. v3.0 does not yet recover."""


class NumericalError(RuntimeError):
    """NaN/Inf in state vector after an integration step."""


# Maximum yaw rate before we abort. Spec §23.6.5 says 5 rad/s.
SPIN_LIMIT_RAD_S = 5.0
STALL_VEL_FLOOR = 0.5  # m/s
STALL_TIME_LIMIT = 2.0  # seconds

# Off-track abort threshold (m). Phase 5.0.1 (spec §23.2-5.0.1.5)
# bumps the default 8 m -> 12 m as part of the chicane regression
# triage. Rationale: the §11.55.F gate stays at <= 6 m cross-track
# (Phase 5.0.1 gate D) as the controller-quality bar; the abort
# threshold is the safety net for unreasonable controllers and at
# 12 m it preserves observability (4 m above the cross-track target,
# giving 6 m of headroom before abort, which is what matters for
# transient excursions through the chicane). Historical 8 m
# (Phase 4.3 tightening, §23.10.12.6) and 50 m (pre-Phase-4.3) are
# both reachable via the env var LAP_OFFTRACK_ABORT_M.
import os as _os
OFFTRACK_ABORT_M = float(_os.environ.get("LAP_OFFTRACK_ABORT_M", "12.0"))
OFFTRACK_ABORT_M2 = OFFTRACK_ABORT_M ** 2


@dataclass
class LapTrace:
    """Per-step trace of an integrated lap.

    All arrays length-N where N is the number of integrator steps recorded
    before lap completion (or abort). Vectors are aligned by index.
    """
    times: np.ndarray = field(default_factory=lambda: np.zeros(0))
    distances: np.ndarray = field(default_factory=lambda: np.zeros(0))
    x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    psi: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v_x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v_y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    omega_yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Per-wheel arrays (dicts keyed FL/FR/RL/RR).
    omega_w: dict = field(default_factory=dict)
    alpha_rad: dict = field(default_factory=dict)
    kappa: dict = field(default_factory=dict)
    Fz_N: dict = field(default_factory=dict)
    Fx_N: dict = field(default_factory=dict)
    Fy_N: dict = field(default_factory=dict)
    # Controls.
    steer_rad: np.ndarray = field(default_factory=lambda: np.zeros(0))
    throttle: np.ndarray = field(default_factory=lambda: np.zeros(0))
    brake: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Reason for termination.
    finished: bool = False
    abort_reason: str = ""
    lap_time_s: float = 0.0
    wallclock_s: float = 0.0


def _build_track_xy(track: "Track") -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return ``(xs, ys, distances_m, total_length)`` for the racing line.

    AC coordinate convention: ``x`` and ``z`` form the horizontal plane,
    ``y`` is elevation. We project onto the horizontal plane as ``(x, z)``
    for the planar ODE — naming the second axis "y" inside the dynamics
    layer to keep the chassis-frame intuition (x = forward, y = left).
    """
    if not getattr(track, "is_csv_backed", False):
        raise ValueError("simulate_slip needs a CSV-backed track (Phase 3).")
    data = track.csv_data
    xs = np.asarray(data["x"], dtype=float)
    # AC `y` = elevation; `z` = the second horizontal axis.
    ys = np.asarray(data["z"], dtype=float)
    ds = np.asarray(data["distance_m"], dtype=float)
    return xs, ys, ds, float(ds[-1])


def _build_grade_lookup(track: "Track") -> np.ndarray | None:
    """Return per-sample ``sin(atan(gradient_pct/100))`` aligned with
    ``track.csv_data['distance_m']``.

    v3 longitudinal-physics fix (2026-05-23): pre-computes the body-x
    gravity coefficient at every track sample so the integration loop
    only does an ``np.interp`` against `s_along`. Returns ``None`` if the
    track CSV does not carry a `gradient_pct` column (legacy CSVs);
    callers treat ``None`` as "flat track, no gravity term".
    """
    data = getattr(track, "csv_data", None) or {}
    if "gradient_pct" not in data:
        return None
    grad_pct = np.asarray(data["gradient_pct"], dtype=float)
    # AC convention: positive `gradient_pct` = uphill = car decelerates.
    # Body-x gravity coefficient is therefore `-g * sin(atan(grad/100))`;
    # we store the unitless `sin(atan(...))` factor and multiply by `-g`
    # inside the integrator (or in the solver-side helper) so the sign
    # convention is co-located with the place that calls it.
    return np.sin(np.arctan(grad_pct / 100.0))


def _initial_heading(track: "Track") -> float:
    """Tangent angle of the racing line at the first sample."""
    data = track.csv_data
    x = np.asarray(data["x"], dtype=float)
    y = np.asarray(data["z"], dtype=float)  # horizontal plane = (x, z)
    # Use a forward difference over ~5 samples to smooth out noise.
    n = min(5, len(x) - 1)
    dx = float(x[n] - x[0])
    dy = float(y[n] - y[0])
    return float(np.arctan2(dy, dx))


def _project_to_track(x: float, y: float, xs: np.ndarray, ys: np.ndarray,
                      distances: np.ndarray, search_start_idx: int,
                      search_window: int = 200) -> tuple[int, float, float]:
    """Project ``(x, y)`` onto the racing-line polyline (monotonic).

    Forward-only sliding-window search starting at ``search_start_idx``.
    Allows small (<10 idx) backward jumps for projection wiggle but never
    teleports across the track — that prevents geometric coincidences
    (start/finish loop on a Nurburgring layout, for example) from
    short-circuiting the lap.

    Returns
    -------
    idx : int
        Index of the nearest sample (monotonically non-decreasing across
        calls, modulo a 10-index forgiveness band).
    s_along : float
        Distance along the line at that sample (m).
    d2_min : float
        Squared Euclidean distance to the nearest point (m^2).
    """
    n = len(xs)
    # Search forward only: allow a tiny backward look (5 samples) to
    # tolerate projection noise, but never the whole track.
    lo = max(0, search_start_idx - 5)
    hi = min(n, search_start_idx + search_window)
    seg_xs = xs[lo:hi]
    seg_ys = ys[lo:hi]
    dx = seg_xs - x
    dy = seg_ys - y
    d2 = dx * dx + dy * dy
    j = int(np.argmin(d2))
    d2min = float(d2[j])
    idx = lo + j
    return idx, float(distances[idx]), d2min


def integrate(state_arr: np.ndarray,
              derivatives_fn: Callable[[np.ndarray], np.ndarray],
              dt: float) -> np.ndarray:
    """Classical 4-stage Runge-Kutta integration step.

    ``derivatives_fn(state_arr) -> dstate/dt`` is called four times per step.
    Pure NumPy; no SciPy dependency.
    """
    k1 = derivatives_fn(state_arr)
    k2 = derivatives_fn(state_arr + 0.5 * dt * k1)
    k3 = derivatives_fn(state_arr + 0.5 * dt * k2)
    k4 = derivatives_fn(state_arr + dt * k3)
    return state_arr + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_slip_lap(
    car: "Car",
    track: "Track",
    compound: "Compound | None",
    pacejka_calib: PacejkaCalibration,
    dyn: CarDynamics,
    driver_controls_fn: Callable[[VehicleState, float], Controls],
    *,
    initial_state: VehicleState | None = None,
    dt: float = 0.01,
    max_time: float = 300.0,
    tyre_state_snapshot: object | None = None,
    car_tyre_model: object | None = None,
    drag_scale: float = 1.0,
    record_every: int = 1,
) -> LapTrace:
    """Integrate one lap with RK4 + per-step driver controller call.

    Termination conditions (first to fire wins):
    - Lap complete: ``normalised_position`` crossed 1.0 with hysteresis.
    - Stalled: ``v_x < 0.5 m/s`` for >2 s.
    - Spun: ``|omega_yaw| > 5 rad/s``.
    - Diverged: NaN/Inf in state.
    - Hard ceiling: ``t > max_time``.

    Parameters
    ----------
    car, track, compound : standard lap-estimator inputs.
    pacejka_calib : PacejkaCalibration
    dyn : CarDynamics
    driver_controls_fn : Callable
        Called once per integrator step with ``(state, t)`` -> ``Controls``.
        Phase 3: a ghost driver (preview-line follower); Phase 4: a real
        :class:`DriverController`.
    initial_state : VehicleState, optional
        Starting chassis state. Default = first track point with v_x=0.
    dt : float
        Integrator step (s). Spec default 5 ms; Phase 3 brief uses 10 ms.
    max_time : float
        Hard ceiling on integrated time (s).
    tyre_state_snapshot, car_tyre_model : optional
        Held constant inside the ODE (Phase 3: no mid-lap thermal evolution).
    drag_scale : float
    record_every : int
        Down-sample factor for the trace (1 = record every step).

    Returns
    -------
    LapTrace
    """
    xs, ys, ds, total_len = _build_track_xy(track)
    # v3 longitudinal-physics fix (2026-05-23): pre-compute `sin(grade)` at
    # every track sample for fast per-step interpolation. ``None`` => the
    # track CSV has no `gradient_pct` column (legacy); we then pass 0 to
    # `compute_derivatives`'s `gravity_a_x` and the lap stays flat.
    grade_sin_arr = _build_grade_lookup(track)
    if initial_state is None:
        initial_state = VehicleState(
            x=float(xs[0]), y=float(ys[0]),
            psi=_initial_heading(track),
            v_x=0.0, v_y=0.0, omega_yaw=0.0,
            omega_FL=0.0, omega_FR=0.0, omega_RL=0.0, omega_RR=0.0,
        )

    state = initial_state.to_array()
    t = 0.0
    t0 = perf_counter()

    # Trace buffers — growable lists, packed to ndarrays at the end.
    buf_t: list[float] = []
    buf_x: list[float] = []
    buf_y: list[float] = []
    buf_psi: list[float] = []
    buf_vx: list[float] = []
    buf_vy: list[float] = []
    buf_omega: list[float] = []
    buf_s: list[float] = []
    buf_omega_w = {w: [] for w in ("FL", "FR", "RL", "RR")}
    buf_alpha = {w: [] for w in ("FL", "FR", "RL", "RR")}
    buf_kappa = {w: [] for w in ("FL", "FR", "RL", "RR")}
    buf_Fz = {w: [] for w in ("FL", "FR", "RL", "RR")}
    buf_Fx = {w: [] for w in ("FL", "FR", "RL", "RR")}
    buf_Fy = {w: [] for w in ("FL", "FR", "RL", "RR")}
    buf_steer: list[float] = []
    buf_throttle: list[float] = []
    buf_brake: list[float] = []

    stall_clock = 0.0
    track_idx = 0
    crossed_half = False
    step_count = 0
    abort_reason = ""
    finished = False
    record: dict = {}
    # Stuck-in-place guard: if `track_idx` hasn't advanced more than 5 over
    # the last 2 seconds (and we're past the first 5 seconds), the car is
    # going in circles. Abort.
    max_idx_seen = 0
    last_idx_advance_time = 0.0

    while t <= max_time:
        vs = VehicleState.from_array(state)

        # Position projection (lap-completion + stall reset).
        track_idx, s_along, d2_off = _project_to_track(
            vs.x, vs.y, xs, ys, ds, track_idx, search_window=200,
        )
        norm_pos = s_along / total_len if total_len > 0 else 0.0
        if norm_pos > 0.5:
            crossed_half = True
        # Lap complete: norm_pos crossed 1.0 from <1.0 to ~0, OR projection
        # wrapped to a near-zero index after having been past 50%.
        if crossed_half and track_idx >= len(xs) - 5:
            finished = True
            break
        # Off-track abort: nearest racing-line point is >OFFTRACK_ABORT_M
        # away. Phase 5.0.1 (spec §23.2-5.0.1.5) default is 12 m, loosened
        # from the Phase 4.3 8 m to give the MPC controller honest room
        # at the Sprint A chicane apex while the §11.55-5.0.1.D cross-track
        # gate (≤ 6 m) stays as the controller-quality bar. Override via
        # the LAP_OFFTRACK_ABORT_M env var.
        if d2_off > OFFTRACK_ABORT_M2:
            abort_reason = (
                f"OffTrackError at t={t:.3f}s: chassis {np.sqrt(d2_off):.1f} m "
                f"from racing line (s={s_along:.0f} m)"
            )
            break
        # Progress guard: track_idx must advance over the last 5 s. If it
        # stays within 5 indices of its max for >5 s, we're stuck (likely
        # going in circles).
        if track_idx > max_idx_seen + 2:
            max_idx_seen = track_idx
            last_idx_advance_time = t
        if t > 10.0 and (t - last_idx_advance_time) > 5.0:
            abort_reason = (
                f"StuckError at t={t:.3f}s: no track-index progress for 5 s "
                f"(s={s_along:.0f} m, idx={track_idx})"
            )
            break

        # Driver controls.
        controls = driver_controls_fn(vs, t)

        # v3 longitudinal-physics fix (2026-05-23): interpolate `gradient_pct`
        # along the track CSV at the current `s_along`, convert to body-x
        # gravity acceleration (`-g * sin(atan(grad/100))`). Held constant
        # across the RK4 stages -- the grade varies on a much longer length
        # scale than the dt=5 ms substep distance (<= 1 m vs ~10 m CSV
        # spacing), so the within-step approximation is safe.
        if grade_sin_arr is not None:
            grade_sin = float(np.interp(s_along, ds, grade_sin_arr))
            gravity_a_x_step = -9.81 * grade_sin
        else:
            gravity_a_x_step = 0.0

        # Build closure for RK4 derivative-fn.
        def dfn(arr: np.ndarray, _ga=gravity_a_x_step) -> np.ndarray:
            return compute_derivatives(
                VehicleState.from_array(arr), controls, car, compound,
                pacejka_calib, dyn,
                tyre_state_snapshot=tyre_state_snapshot,
                car_tyre_model=car_tyre_model,
                drag_scale=drag_scale,
                gravity_a_x=_ga,
                record=None,
            )

        # Record for the trace (one derivative call with record=True to
        # gather alpha/kappa/Fz/Fx/Fy at the current state).
        record.clear()
        compute_derivatives(
            vs, controls, car, compound, pacejka_calib, dyn,
            tyre_state_snapshot=tyre_state_snapshot,
            car_tyre_model=car_tyre_model,
            drag_scale=drag_scale,
            gravity_a_x=gravity_a_x_step,
            record=record,
        )

        # Trace before stepping.
        if step_count % record_every == 0:
            buf_t.append(t)
            buf_x.append(vs.x)
            buf_y.append(vs.y)
            buf_psi.append(vs.psi)
            buf_vx.append(vs.v_x)
            buf_vy.append(vs.v_y)
            buf_omega.append(vs.omega_yaw)
            buf_s.append(s_along)
            for w in ("FL", "FR", "RL", "RR"):
                buf_omega_w[w].append(getattr(vs, f"omega_{w}"))
                buf_alpha[w].append(record["alpha_rad"][w])
                buf_kappa[w].append(record["kappa"][w])
                buf_Fz[w].append(record["Fz"][w])
                buf_Fx[w].append(record["Fx"][w])
                buf_Fy[w].append(record["Fy"][w])
            buf_steer.append(controls.steer_rad)
            buf_throttle.append(controls.throttle)
            buf_brake.append(controls.brake)

        # RK4 step.
        new_state = integrate(state, dfn, dt)
        # Guard: NaN / Inf.
        if not np.all(np.isfinite(new_state)):
            abort_reason = f"NumericalError at t={t:.3f}s: NaN/Inf in state"
            break
        # Clamp wheel omega at 0 to avoid runaway-negative under heavy brake.
        for i in (6, 7, 8, 9):
            if new_state[i] < 0.0:
                new_state[i] = 0.0
        # Guard: spin.
        if abs(new_state[5]) > SPIN_LIMIT_RAD_S:
            abort_reason = f"SpunError at t={t:.3f}s: |omega_yaw|={abs(new_state[5]):.2f} rad/s"
            break
        # Guard: stall.
        if new_state[3] < STALL_VEL_FLOOR:
            stall_clock += dt
            if stall_clock > STALL_TIME_LIMIT:
                abort_reason = (
                    f"StalledError at t={t:.3f}s: v_x<{STALL_VEL_FLOOR} m/s "
                    f"for >{STALL_TIME_LIMIT} s"
                )
                break
        else:
            stall_clock = 0.0

        state = new_state
        t += dt
        step_count += 1

    if not finished and not abort_reason:
        abort_reason = f"max_time {max_time:.1f}s reached without lap completion"

    # Pack outputs.
    times = np.array(buf_t)
    distances = np.array(buf_s)
    speeds = np.array(buf_vx)  # body-frame forward speed
    omega_w = {w: np.array(buf_omega_w[w]) for w in ("FL", "FR", "RL", "RR")}
    alpha = {w: np.array(buf_alpha[w]) for w in ("FL", "FR", "RL", "RR")}
    kappa = {w: np.array(buf_kappa[w]) for w in ("FL", "FR", "RL", "RR")}
    Fz = {w: np.array(buf_Fz[w]) for w in ("FL", "FR", "RL", "RR")}
    Fx = {w: np.array(buf_Fx[w]) for w in ("FL", "FR", "RL", "RR")}
    Fy = {w: np.array(buf_Fy[w]) for w in ("FL", "FR", "RL", "RR")}

    trace = LapTrace(
        times=times,
        distances=distances,
        x=np.array(buf_x), y=np.array(buf_y), psi=np.array(buf_psi),
        v_x=speeds, v_y=np.array(buf_vy), omega_yaw=np.array(buf_omega),
        omega_w=omega_w, alpha_rad=alpha, kappa=kappa,
        Fz_N=Fz, Fx_N=Fx, Fy_N=Fy,
        steer_rad=np.array(buf_steer),
        throttle=np.array(buf_throttle),
        brake=np.array(buf_brake),
        finished=finished,
        abort_reason=abort_reason,
        lap_time_s=float(t) if finished else 0.0,
        wallclock_s=perf_counter() - t0,
    )
    return trace


def integrate_lap(  # noqa: D401 — preserved for back-compat with Phase 1 surface
    initial_state: VehicleState,
    car: "Car",
    track: "Track",
    compound: "Compound | None",
    controller,
    *,
    pacejka_calib: PacejkaCalibration,
    dyn: CarDynamics,
    dt: float = 0.01,
    max_sim_time_s: float = 300.0,
    tyre_state_snapshot: object | None = None,
    car_tyre_model: object | None = None,
    drag_scale: float = 1.0,
) -> LapTrace:
    """Phase 1 compat wrapper: integrate one lap with a controller object.

    The controller is expected to expose ``.controls(state, t, track) ->
    Controls`` (ghost driver, Phase 3) or ``.step(state, t) -> Controls``
    (Phase 4 :class:`DriverController`). We probe for both.
    """
    if hasattr(controller, "controls"):
        def fn(state: VehicleState, t: float) -> Controls:
            return controller.controls(state, t, track)
    elif hasattr(controller, "step"):
        def fn(state: VehicleState, t: float) -> Controls:
            return controller.step(state, t)
    else:
        raise TypeError("controller must expose .controls(...) or .step(...)")

    return simulate_slip_lap(
        car, track, compound, pacejka_calib, dyn, fn,
        initial_state=initial_state, dt=dt, max_time=max_sim_time_s,
        tyre_state_snapshot=tyre_state_snapshot, car_tyre_model=car_tyre_model,
        drag_scale=drag_scale,
    )
