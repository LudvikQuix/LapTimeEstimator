"""Slip-sim result dataclass + trace-to-result packing (spec §23.3).

Pulled out of :mod:`slip_simulator` to keep that module under the 500-
line soft cap. The contents (``SlipSimResult``, ``_fmt_time``,
``_load_pacejka_calibration``, ``_trace_to_result``) are pure data-
shape machinery — no controller or solver logic lives here.

Module-level :data:`log` is provided so the rear-axle-fallback warning
in ``_load_pacejka_calibration`` keeps its current logger name.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from .pacejka import PacejkaCoeffs
from .solver import LapTrace
from .vehicle import AxleCoeffs, PacejkaCalibration

if TYPE_CHECKING:
    from ..driver import Driver
    from ..track import Track

log = logging.getLogger(__name__)


@dataclass
class SlipSimResult:
    """Output of a slip-based single-lap or stint run.

    Drop-in compatible with the v2 :class:`SimResult` writers via the
    legacy field set: ``lap_time``, ``distances``, ``speeds``, ``times``,
    ``ai_speeds``, ``lap_id``, ``two_lap``, ``lap1_time``, ``lap2_time``.
    """

    # v2 compat surface.
    lap_time: float = 0.0
    distances: np.ndarray = field(default_factory=lambda: np.zeros(0))
    speeds: np.ndarray = field(default_factory=lambda: np.zeros(0))
    times: np.ndarray = field(default_factory=lambda: np.zeros(0))
    ai_speeds: np.ndarray | None = None
    limit_label: np.ndarray | None = None
    sectors: list = field(default_factory=list)
    lap_id: np.ndarray | None = None
    lap1_time: float = 0.0
    lap2_time: float = 0.0
    two_lap: bool = False

    # v3-only extras.
    lap_times_s: list[float] = field(default_factory=list)
    n_laps: int = 1
    x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    psi: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v_x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v_y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    omega_yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slip_angle_rad: dict[str, np.ndarray] = field(default_factory=dict)
    slip_ratio: dict[str, np.ndarray] = field(default_factory=dict)
    Fz_N: dict[str, np.ndarray] = field(default_factory=dict)
    Fx_N: dict[str, np.ndarray] = field(default_factory=dict)
    Fy_N: dict[str, np.ndarray] = field(default_factory=dict)
    rng_seed: int | None = None
    solver: str = "rk4"
    dt_s: float = 0.01

    fallback_front_to_rear: bool = False
    abort_reason: str = ""
    finished: bool = False
    wallclock_s: float = 0.0

    # Phase 4 metrics.
    util_p85: float = 0.0
    """85th percentile of |alpha_front_avg| / slip_target_rad across the lap.

    Headline acceptance metric (spec §11.55): values >1.0 indicate the
    driver is exceeding the tyre's slip-peak window. The Phase-4 target
    is util_p85 <= 1.0.
    """
    slip_target_rad: float = 0.0
    used_ghost: bool = False
    """True if the simulation was executed with the Phase-3 GhostDriver."""

    # Monte-Carlo extras (when consistency_sigma > 0).
    mc_lap_times_s: list[float] = field(default_factory=list)
    mc_n_runs: int = 0
    mc_sigma_s: float = 0.0

    # Phase 5.0 (v3.2 MPC, spec §23.2): solver diagnostics.
    mpc_solve_times_s: list[float] = field(default_factory=list)
    """Per-tick MPC solve time (s). Used for §11.55.I real-time check."""
    mpc_ghost_steps: int = 0
    """Count of ghost-fallback steps inside the MPC's tier-2 recovery.

    Phase 5.0.3: now an alias for ``mpc_tier_counts[2]`` (the reactive
    sub-controller fallback). The historical ghost-driver path is
    deprecated per spec §23.2-5.0.3.5; the field stays for back-compat
    plot / CSV readers. Marked for removal in Phase 5.1.
    """

    # Phase 5.0.3 (v3.2 MPC, spec §23.2-5.0.3.7): fallback-tier diagnostics.
    mpc_tier_counts: dict = field(
        default_factory=lambda: {0: 0, 1: 0, 2: 0}
    )
    """Per-ODE-step tier counts. Tier 0 = clean MPC, Tier 1 = ellipse-
    saturation feedforward, Tier 2 = reactive sub-controller. Used for
    §11.55-5.0.3 gates A/B/C."""

    mpc_tier1_episodes: int = 0
    """Number of distinct Tier-1 episodes (contiguous runs). Diagnostic."""

    mpc_tier1_max_consecutive_steps: int = 0
    """Longest Tier-1 episode in steps. Diagnostic."""

    mpc_tier2_episodes: int = 0
    """Number of distinct Tier-2 episodes (contiguous runs)."""

    mpc_qp_status_counts: dict = field(default_factory=dict)
    """Raw OSQP status string -> count. Helps post-mortem classify which
    infeasibility class (primal vs dual vs max_iter) dominates."""

    mpc_post_solve_ellipse_violation_p95: float = 0.0
    """95th percentile of max-stage post-solve ellipse residual across
    all clean Tier-0 ticks. >0.15 ⇒ the tangent half-space
    approximation is biting and detection 4 is firing often.
    Diagnostic; informs whether to tighten the ellipse linearisation."""

    # v3.3 MPCC diagnostics (spec §23.3.6.7 / §23.3.7.7). Populated when
    # ``--controller mpcc``; default 0.0 otherwise.
    mpcc_contour_p95: float = 0.0
    """95th-percentile |n| (perpendicular distance from the reference path)
    over the run. Reads as "within a car-width of the reference." MPCC gate:
    ≤ 1.5 m."""

    mpcc_lag_p95: float = 0.0
    """95th-percentile |s − θ| (virtual-progress lag) over the run. Reads as
    "the virtual progress θ is tracking the actual progress s." MPCC gate:
    ≤ 3.0 m."""

    mpcc_progress_mean: float = 0.0
    """Mean ``V_θ`` (rate of virtual progress) over committed MPCC ticks.
    Confidence check: should sit within ±15 % of mean ``v_ref``."""

    # v3.4 Hierarchical MPC diagnostics (spec §23.4.6.5). Populated only
    # for ``--controller hmpc``; default zeros otherwise.
    hmpc_outer_solve_times_s: list[float] = field(default_factory=list)
    """Per-call outer planner wall-clock solve time (s)."""
    hmpc_inner_solve_times_s: list[float] = field(default_factory=list)
    """Per-tick inner tracker wall-clock solve time (s). Alias for
    ``mpc_solve_times_s`` so v3.2 dashboards keep working."""
    hmpc_outer_solve_count: int = 0
    hmpc_inner_solve_count: int = 0
    hmpc_outer_staleness_ticks_p95: float = 0.0
    """95th pct ticks elapsed since the last outer solve. Should sit at
    or below ``Δt_outer / Δt_inner`` (= 50 at 1 Hz outer / 50 Hz inner)."""
    hmpc_outer_infeas_count: int = 0
    hmpc_outer_vs_inner_vref_p95: float = 0.0
    """95th pct |v_ref_outer(s) − v_dp_plan(s)| over inner stages. Large
    values mean outer is doing real work; small values mean the outer
    is barely deviating from the DP plan."""
    hmpc_pi_trim_steer_p95: float = 0.0
    """95th pct |steer_trim| (rad). Saturated at the bound for >0.5 s
    indicates the outer's reference is infeasible for the inner — see
    §23.4.9.3."""
    hmpc_pi_trim_throttle_p95: float = 0.0
    hmpc_pi_trim_brake_p95: float = 0.0
    hmpc_tier_counts: dict = field(
        default_factory=lambda: {0: 0, 1: 0, 2: 0}
    )
    """Per-ODE-step HMPC tier counts. 0 = clean (outer + inner + PI),
    1 = DP-plan-inner fallback, 2 = reactive sub-controller."""

    @property
    def lap_time_str(self) -> str:
        return _fmt_time(self.lap_time)

    @property
    def lap1_time_str(self) -> str:
        return _fmt_time(self.lap1_time)

    @property
    def lap2_time_str(self) -> str:
        return _fmt_time(self.lap2_time)

    @property
    def max_speed_kph(self) -> float:
        return float(np.max(self.speeds) * 3.6) if len(self.speeds) else 0.0

    @property
    def min_speed_kph(self) -> float:
        return float(np.min(self.speeds) * 3.6) if len(self.speeds) else 0.0

    @property
    def avg_speed_kph(self) -> float:
        if not len(self.distances) or self.lap_time <= 0:
            return 0.0
        d = float(self.distances[-1] - self.distances[0])
        return (d / self.lap_time) * 3.6


def _fmt_time(seconds: float) -> str:
    m = int(seconds // 60)
    s = seconds - m * 60
    return f"{m}:{s:06.3f}"


def _load_pacejka_calibration(driver: "Driver") -> tuple[PacejkaCalibration, bool]:
    """Build a :class:`PacejkaCalibration` from the driver's ``pacejka_calibration`` block.

    Returns
    -------
    calib : PacejkaCalibration
    fallback : bool
        ``True`` if the front-axle E coefficient is rail-clamped at -2.0 and
        we copied the rear axle's coefficients to the front. The Phase 2
        fitter sometimes produces clamped fronts when the calibration cloud
        is sparse on the front axle; spec §23 Phase 3 brief recommends this
        rear→front fallback for the Phase-3 test bed.
    """
    raw = getattr(driver, "raw", {}) or {}
    block = raw.get("pacejka_calibration")
    if not isinstance(block, dict):
        raise ValueError(
            "Driver JSON has no `pacejka_calibration` block. Run "
            "`fit_driver.py --fit-pacejka` first (Phase 2)."
        )
    front = block.get("front") or {}
    rear = block.get("rear") or {}
    if not front or not rear:
        raise ValueError(
            "pacejka_calibration: both 'front' and 'rear' axles required."
        )

    def axle(d: dict) -> AxleCoeffs:
        lat = d.get("lateral") or {}
        lng = d.get("longitudinal") or {}
        # The fitter stores D as `D_per_Fz` (peak grip / Fz, i.e. mu peak).
        lat_d = float(lat.get("D", lat.get("D_per_Fz", 1.5)))
        lng_d = float(lng.get("D", lng.get("D_per_Fz", 1.4)))
        # AC load-sensitivity knobs (per axle): ``FZ0`` and ``LS_EXPY`` /
        # ``LS_EXPX``. Top-level axle dict may supply ``FZ0`` and
        # ``LS_EXPY`` / ``LS_EXPX`` directly (mirrors ``tyres.ini`` layout);
        # per-direction overrides are accepted via
        # ``lateral.LS_EXPY`` / ``longitudinal.LS_EXPX`` for forward
        # compatibility. Missing values keep the legacy linear-load form.
        fz0_raw = d.get("FZ0")
        fz0 = float(fz0_raw) if fz0_raw is not None else None
        ls_lat_raw = lat.get("LS_EXPY", d.get("LS_EXPY"))
        ls_lng_raw = lng.get("LS_EXPX", d.get("LS_EXPX"))
        ls_lat = float(ls_lat_raw) if ls_lat_raw is not None else None
        ls_lng = float(ls_lng_raw) if ls_lng_raw is not None else None
        return AxleCoeffs(
            lateral=PacejkaCoeffs(
                B=float(lat.get("B", 10.0)),
                C=float(lat.get("C", 1.3)),
                D=lat_d,
                E=float(lat.get("E", -0.2)),
            ),
            longitudinal=PacejkaCoeffs(
                B=float(lng.get("B", 10.0)),
                C=float(lng.get("C", 1.65)),
                D=lng_d,
                E=float(lng.get("E", 0.3)),
            ),
            Fz0=fz0,
            ls_exp_lat=ls_lat,
            ls_exp_long=ls_lng,
        )

    front_axle = axle(front)
    rear_axle = axle(rear)

    # Phase 3 fallback: if either front coefficient set has E rail-clamped
    # at the lower bound, the fit failed for that axle. Use rear coeffs for
    # all four wheels as a clean test bed for the ODE integrator.
    # sc-71955: bounds tightened to E >= -1.0 on fit side; legacy fits at
    # -2.0 still hit this path. With improvement 2 (pooled lateral fit)
    # front/rear lateral coefficients are now identical by construction,
    # so this fallback typically becomes a no-op.
    fallback = (
        abs(front_axle.lateral.E + 2.0) < 1e-6
        or abs(front_axle.longitudinal.E + 2.0) < 1e-6
        or abs(front_axle.lateral.E + 1.0) < 1e-3
        or abs(front_axle.longitudinal.E + 1.0) < 1e-3
    )
    if fallback:
        log.warning(
            "Front Pacejka E rail-clamped at lower bound -> using rear-axle "
            "coefficients for all four wheels (Phase 3 test-bed fallback)."
        )
        front_axle = rear_axle

    falloff_raw = block.get("falloff_level")
    falloff = float(falloff_raw) if falloff_raw is not None else None
    return PacejkaCalibration(
        front=front_axle,
        rear=rear_axle,
        ellipse_exponent=float(block.get("friction_ellipse_exponent", 2.0)),
        falloff_level=falloff,
    ), fallback


def _trace_to_result(
    traces: list[LapTrace],
    track: "Track",
    *,
    dt: float,
    fallback: bool,
) -> SlipSimResult:
    """Pack one or more lap traces into a v2-compat :class:`SlipSimResult`."""
    lap_times = [float(tr.lap_time_s) for tr in traces]
    times_parts: list[np.ndarray] = []
    distances_parts: list[np.ndarray] = []
    speeds_parts: list[np.ndarray] = []
    lap_id_parts: list[np.ndarray] = []
    x_parts: list[np.ndarray] = []
    y_parts: list[np.ndarray] = []
    psi_parts: list[np.ndarray] = []
    vy_parts: list[np.ndarray] = []
    omega_parts: list[np.ndarray] = []
    slip_a = {w: [] for w in ("FL", "FR", "RL", "RR")}
    slip_k = {w: [] for w in ("FL", "FR", "RL", "RR")}
    Fz = {w: [] for w in ("FL", "FR", "RL", "RR")}
    Fx = {w: [] for w in ("FL", "FR", "RL", "RR")}
    Fy = {w: [] for w in ("FL", "FR", "RL", "RR")}
    t_offset = 0.0
    for k, tr in enumerate(traces, start=1):
        n = len(tr.times)
        if n == 0:
            continue
        times_parts.append(tr.times + t_offset)
        distances_parts.append(tr.distances)
        speeds_parts.append(tr.v_x)
        lap_id_parts.append(np.full(n, k, dtype=int))
        x_parts.append(tr.x)
        y_parts.append(tr.y)
        psi_parts.append(tr.psi)
        vy_parts.append(tr.v_y)
        omega_parts.append(tr.omega_yaw)
        for w in ("FL", "FR", "RL", "RR"):
            slip_a[w].append(tr.alpha_rad[w])
            slip_k[w].append(tr.kappa[w])
            Fz[w].append(tr.Fz_N[w])
            Fx[w].append(tr.Fx_N[w])
            Fy[w].append(tr.Fy_N[w])
        t_offset += float(tr.lap_time_s)

    if not times_parts:
        return SlipSimResult(
            lap_time=0.0,
            n_laps=len(traces),
            lap_times_s=lap_times,
            fallback_front_to_rear=fallback,
            abort_reason=traces[-1].abort_reason if traces else "no traces",
            finished=False,
            wallclock_s=sum(tr.wallclock_s for tr in traces),
            dt_s=dt,
        )

    ai_parts = []
    for d_arr in distances_parts:
        v_src = track.csv_data["speed_ms"]
        d_src = track.csv_data["distance_m"]
        ai_parts.append(np.interp(d_arr, d_src, v_src))

    result = SlipSimResult(
        lap_time=float(lap_times[-1]) if lap_times else 0.0,
        distances=np.concatenate(distances_parts),
        speeds=np.concatenate(speeds_parts),
        times=np.concatenate(times_parts),
        ai_speeds=np.concatenate(ai_parts),
        lap_id=np.concatenate(lap_id_parts),
        lap1_time=float(lap_times[0]) if lap_times else 0.0,
        lap2_time=float(lap_times[1]) if len(lap_times) >= 2 else 0.0,
        two_lap=len(traces) >= 2,
        lap_times_s=lap_times,
        n_laps=len(traces),
        x=np.concatenate(x_parts),
        y=np.concatenate(y_parts),
        psi=np.concatenate(psi_parts),
        v_x=np.concatenate(speeds_parts),
        v_y=np.concatenate(vy_parts),
        omega_yaw=np.concatenate(omega_parts),
        slip_angle_rad={w: np.concatenate(slip_a[w]) for w in ("FL", "FR", "RL", "RR")},
        slip_ratio={w: np.concatenate(slip_k[w]) for w in ("FL", "FR", "RL", "RR")},
        Fz_N={w: np.concatenate(Fz[w]) for w in ("FL", "FR", "RL", "RR")},
        Fx_N={w: np.concatenate(Fx[w]) for w in ("FL", "FR", "RL", "RR")},
        Fy_N={w: np.concatenate(Fy[w]) for w in ("FL", "FR", "RL", "RR")},
        dt_s=dt,
        fallback_front_to_rear=fallback,
        finished=all(tr.finished for tr in traces),
        wallclock_s=sum(tr.wallclock_s for tr in traces),
        abort_reason="; ".join(tr.abort_reason for tr in traces if tr.abort_reason),
    )
    return result
