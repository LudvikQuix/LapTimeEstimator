"""Driver-controller parameter block (spec §23.5.2, Phase 4).

A small, JSON-serialisable dataclass that captures every knob the
Phase 4 :class:`DriverController` needs. Loaded from the driver JSON
``control_params`` block when present; defaults otherwise.

Two parameter blocks coexist:

- The **Phase-3 GhostDriver** carries its own hand-tuned constants and
  uses :class:`GhostControlParams` (Stanley-style; see
  :mod:`driver_controller`).
- The **Phase-4 DriverController** uses :class:`ControlParams` (this
  module) and reads ``control_params`` from the driver JSON.

We keep them separate so the Phase 3 ghost-driver remains reachable for
regression testing.

Spec hand-defaults (§23.5.2):

    preview_distance_m         18.0
    preview_time_s              1.5   (Phase 4.1, §23.10.5.1)
    steering_p_gain             1.2
    throttle_p_gain             0.5
    brake_p_gain                0.6
    throttle_rate_limit_pct_s   None  (Phase 4.1; resolves to
                                       profile.dynamic.throttle_ramp_pct_s
                                       else 1000 -> effectively unlimited)
    slip_target_deg             None  (derived from skill_pct + Pacejka)
    consistency_noise_std_*     0.3, 1.5, 1.5
    steering_softener_engage    1.49  (Phase 5.0.1, §23.2-5.0.1.6; kill switch)
    steering_softener_full      1.50  (Phase 5.0.1, §23.2-5.0.1.6; engage<full)

Phase 5.0 (v3.2 MPC, spec §23.2): an optional nested ``MPCParams`` block
carries the MPC horizon / weights / SQP iteration cap. Read via
:meth:`MPCParams.from_driver`; the MPC controller pulls it lazily so
non-MPC users pay zero cost.

Phase 4 brief overrides preview / gains downward (15 / 1.5 / 0.5 / 0.3)
based on practical stability tuning on Sprint A; the spec's 18 / 1.2 /
0.5 / 0.6 are the documented schema and remain the JSON defaults so a
hand-authored driver JSON has the same numbers as the spec.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..driver import Driver


@dataclass(frozen=True)
class ControlParams:
    """Phase-4 :class:`DriverController` tunable parameters.

    Hand defaults match spec §23.5.2 verbatim. The Phase 4 brief used
    slightly tighter numbers for in-code defaults; we honour the spec
    here and let drivers override per-channel via JSON.
    """

    preview_distance_m: float = 18.0
    preview_time_s: float = 1.5  # Phase 4.1 longitudinal preview horizon (§23.10.5.1)
    steering_p_gain: float = 1.2
    throttle_p_gain: float = 0.5
    brake_p_gain: float = 0.6
    # Phase 4.1: throttle rate-limit (%/s). When None, resolved at controller
    # construction from `driver.profile.dynamic.throttle_ramp_pct_s`; if that
    # is also absent, falls back to 1000 (effectively unlimited).
    throttle_rate_limit_pct_s: float | None = None
    slip_target_deg: float | None = None  # if set, overrides skill-derived
    consistency_noise_std_steer_deg: float = 0.3
    consistency_noise_std_throttle_pct: float = 1.5
    consistency_noise_std_brake_pct: float = 1.5
    # Phase 4.3 (§23.10.12): slip-aware steering softener. The softener
    # multiplies the post-Stanley δ by a linear ramp that is 1.0 below
    # `engage * α_peak_front`, decays to 0.0 at `full * α_peak_front`, and
    # saturates at 0.0 above. Setting `engage` >= `full` effectively
    # disables the softener.
    # Phase 4.3 softener is empirically dead (parent §23.10.12; Phase 5.0.1
    # spec §23.2-5.0.1.6). Defaults are the kill-switch band 1.49 < 1.50
    # (no values of |alpha_norm| in [0, 1] hit the engagement band). Set
    # explicit values in driver JSON to re-enable for A/B comparison.
    steering_softener_engage: float = 1.49
    steering_softener_full: float = 1.5
    # Phase 5.0.5-reactive (2026-05-23): first-order LPF smoother on the
    # reactive controller's post-softener steer command, addressing the
    # chronic Sprint A s≈1087 m chatter abort (sustained-radius corner,
    # util_p85 ≈ 0.5 — NOT envelope-limited; the failure is steering
    # chatter, not grip). Spec offered FIR or LPF and said "pick whichever
    # is simpler"; the LPF avoids the FIR's stochastic-noise re-injection
    # problem (the Tier-1 FIR worked on bounded unit vectors but on raw
    # radian δ with per-tick consistency noise the recursive emit-FIR
    # becomes an IIR with too much phase lag).
    #
    # The discrete LPF is δ_smoothed = α·δ + (1-α)·δ_prev with
    # α = 1 / `stanley_blend_window_ticks`. At the simulator's 20 ms
    # tick (dt=0.02 s, 50 Hz), the time constant τ = N · dt. The default
    # is **N=3 (τ=60 ms)** — directly matching the spec's stated
    # alternative τ ≈ 60 ms. Empirically (Tomas / Sprint A / reactive,
    # 2026-05-23 longitudinal-physics head):
    #   - N=2: too aggressive — aborts at first corner cluster (s≈160 m).
    #   - N=3 (τ=60 ms, default): clears the chicane, util_p85 ≈ 1.08,
    #     abort moves to s=1086 (the chronic chatter corner — within
    #     1 m of historical s=1087).
    #   - N=4..6: chicane abort returns with rising util_p85 (1.86..1.38).
    #   - N=8+: stalls.
    # Set to 0 or 1 to bypass the LPF entirely.
    stanley_blend_window_ticks: int = 3
    # Phase 5.0.6-reactive (2026-05-23): Stanley cross-track gain. The
    # classical Stanley formulation is
    #   δ_cross = atan2(k_cross · e_cross, v + soft)
    # which drops the effective lateral bandwidth as speed rises (k_cross /
    # (v+soft)) to avoid high-speed oscillation. The trade-off: in
    # sustained-radius high-speed corners e_cross accumulates monotonically
    # until the off-track guard fires. Pre-fix value was hardcoded 0.5
    # which gave an effective gain of 0.5/(40+3) ≈ 0.012 in the chronic
    # Sprint A s=1086 m corner (v≈40 m/s, 40 m sustained radius). Empirical
    # sweep (Tomas / Sprint A / single lap / inertia-zz 2400, MC×10):
    #   - k_cross=0.5  (pre-fix):    0/10 finish; 8/10 abort s=1086 m.
    #   - k_cross=0.75 (× 1.5, this default): 7/10 finish, mean 2:09.12,
    #     σ=0.047; remaining 3 aborts are the chicane envelope problem
    #     (s≈665 m), NOT the chatter station — chronic s=1086 m abort
    #     fully resolved.
    #   - k_cross=1.0  (× 2.0):      7/10 finish, mean 2:10.24, σ=0.050.
    #   - k_cross=1.5  (× 3.0):      7/10 finish, mean 2:11.72, σ=0.066.
    # The ×1.5 default is the cheapest gain that clears the chronic corner
    # without destabilising turn-in (verified visually: early-lap steering
    # remains smooth, no oscillation through s<300 m corner cluster). Set
    # per driver via `control_params.stanley.k_cross`.
    stanley_k_cross: float = 0.75
    measured: bool = False

    @classmethod
    def from_driver(cls, driver: "Driver") -> "ControlParams":
        """Read ``control_params`` from a driver's JSON, falling back to defaults.

        Recognises both the spec key (``slip_target_deg``) and the legacy
        Phase-3 key (``slip_target_lat_deg``) for the slip override.
        """
        raw = getattr(driver, "raw", {}) or {}
        block = raw.get("control_params") or {}
        if not isinstance(block, dict):
            block = {}
        slip = block.get("slip_target_deg")
        if slip is None:
            slip = block.get("slip_target_lat_deg")
        try:
            slip_val = float(slip) if slip is not None else None
        except (TypeError, ValueError):
            slip_val = None
        rate_limit_raw = block.get("throttle_rate_limit_pct_s")
        try:
            rate_limit = float(rate_limit_raw) if rate_limit_raw is not None else None
        except (TypeError, ValueError):
            rate_limit = None
        # Phase 4.3 (§23.10.12.4): softener band. Validate at load time —
        # out-of-range values are rejected here with a clear message.
        engage = float(block.get(
            "steering_softener_engage", cls.steering_softener_engage))
        full = float(block.get(
            "steering_softener_full", cls.steering_softener_full))
        if not (0.5 <= engage < full <= 1.5):
            raise ValueError(
                "control_params: steering_softener band invalid "
                f"(engage={engage}, full={full}); require "
                "0.5 <= engage < full <= 1.5"
            )
        # Phase 5.0.5-reactive: stanley blend window. Read from the nested
        # `stanley` block if present; otherwise default. The nested-block
        # location mirrors the `mpc.tier1` block used by the Tier-1 FIR fix
        # — a separate namespace for reactive-only knobs.
        stanley_block = block.get("stanley") if isinstance(block, dict) else None
        if not isinstance(stanley_block, dict):
            stanley_block = {}
        stanley_window_raw = stanley_block.get(
            "blend_window_ticks", cls.stanley_blend_window_ticks,
        )
        try:
            stanley_window = int(stanley_window_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "control_params.stanley.blend_window_ticks must be an int "
                f"(got {stanley_window_raw!r})"
            ) from exc
        if stanley_window < 0:
            raise ValueError(
                "control_params.stanley.blend_window_ticks must be >= 0 "
                f"(got {stanley_window})"
            )
        # Phase 5.0.6-reactive: Stanley cross-track gain. Shares the
        # `control_params.stanley` namespace with the LPF blend-window
        # knob. Validation: must be > 0 (negative or zero gain disables
        # cross-track correction, which would cause silent off-track
        # drift on any layout).
        stanley_k_cross_raw = stanley_block.get(
            "k_cross", cls.stanley_k_cross,
        )
        try:
            stanley_k_cross = float(stanley_k_cross_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "control_params.stanley.k_cross must be a float "
                f"(got {stanley_k_cross_raw!r})"
            ) from exc
        if stanley_k_cross <= 0:
            raise ValueError(
                "control_params.stanley.k_cross must be > 0 "
                f"(got {stanley_k_cross})"
            )
        return cls(
            preview_distance_m=float(block.get(
                "preview_distance_m", cls.preview_distance_m)),
            preview_time_s=float(block.get(
                "preview_time_s", cls.preview_time_s)),
            steering_p_gain=float(block.get(
                "steering_p_gain", cls.steering_p_gain)),
            throttle_p_gain=float(block.get(
                "throttle_p_gain", cls.throttle_p_gain)),
            brake_p_gain=float(block.get(
                "brake_p_gain", cls.brake_p_gain)),
            throttle_rate_limit_pct_s=rate_limit,
            slip_target_deg=slip_val,
            consistency_noise_std_steer_deg=float(block.get(
                "consistency_noise_std_steer_deg",
                cls.consistency_noise_std_steer_deg)),
            consistency_noise_std_throttle_pct=float(block.get(
                "consistency_noise_std_throttle_pct",
                cls.consistency_noise_std_throttle_pct)),
            consistency_noise_std_brake_pct=float(block.get(
                "consistency_noise_std_brake_pct",
                cls.consistency_noise_std_brake_pct)),
            steering_softener_engage=engage,
            steering_softener_full=full,
            stanley_blend_window_ticks=stanley_window,
            stanley_k_cross=stanley_k_cross,
            measured=bool(block.get("measured", False)),
        )

    def with_slip_target(self, slip_target_deg: float | None) -> "ControlParams":
        """Return a copy with ``slip_target_deg`` replaced (for Monte Carlo)."""
        return ControlParams(
            preview_distance_m=self.preview_distance_m,
            preview_time_s=self.preview_time_s,
            steering_p_gain=self.steering_p_gain,
            throttle_p_gain=self.throttle_p_gain,
            brake_p_gain=self.brake_p_gain,
            throttle_rate_limit_pct_s=self.throttle_rate_limit_pct_s,
            slip_target_deg=slip_target_deg,
            consistency_noise_std_steer_deg=self.consistency_noise_std_steer_deg,
            consistency_noise_std_throttle_pct=self.consistency_noise_std_throttle_pct,
            consistency_noise_std_brake_pct=self.consistency_noise_std_brake_pct,
            steering_softener_engage=self.steering_softener_engage,
            steering_softener_full=self.steering_softener_full,
            stanley_blend_window_ticks=self.stanley_blend_window_ticks,
            stanley_k_cross=self.stanley_k_cross,
            measured=self.measured,
        )
