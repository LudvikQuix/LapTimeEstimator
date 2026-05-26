"""Driver-as-control-loop (spec §23.3, §23.8).

Phase 4: introduces the **real :class:`DriverController`** — a
preview-target steering controller with a slip-aware speed loop.
:class:`GhostDriver` (Phase 3 stub: Stanley + min-speed-in-window
P-controller targeting the v2 plan at 0.85x scale) stays in place so it
remains reachable for regression testing and as a safety fallback when
the new controller spins.

Phase 4 / 4.1 controller — design (§23.8, §23.10.5):

1.  Preview-target steering. Project the chassis onto the racing line,
    look ahead by ``preview_distance_m`` to a preview point. Steering is
    Stanley (heading-err + cross-track / (v + soft)). Unchanged in 4.1.
2.  Phase 4.1 longitudinal preview. Look ahead **in time** by
    ``preview_time_s`` (default 1.5 s; horizon = ``max(30 m,
    v_preview·t_preview)``) and track the **minimum** target speed in
    that window — i.e. pre-brake before slow corners. This replaces the
    v3.0 reactive ``v > v_target_local`` brake trigger that only fired
    after the brake point had passed.
3.  Throttle and brake P-controllers on ``(v_target_min - v_x)``. A
    throttle rate-limit (from ``control_params.throttle_rate_limit_pct_s``
    or ``driver.profile.dynamic.throttle_ramp_pct_s``) is applied
    BEFORE the slip-band modulators (§23.10.9 risk #3).
4.  Slip-target enforcement (post-hoc safety, §23.10.5.1 step 5). If
    ``|alpha_front_avg|`` is **below** ``0.9 * slip_target_rad`` and we
    want more speed, push throttle a notch. **Above** ``1.1x``: ease off
    15%. **Above** ``1.3x``: cut throttle 30% and add trail brake. The
    v3.0 feedforward speed-target clamp at slip_ratio > 0.85 is REMOVED
    — pre-braking handles the planning role honestly.
5.  α_peak-derived slip target (§23.10.5.2). ``slip_target_deg =
    α_peak_front_deg * (0.5 + 0.5 * skill_pct)`` when the Pacejka block
    is present (computed lazily; cached). Skill=1 → 100% of peak, skill=0
    → 50% of peak. Fallback to the v3.0 ``6·skill + 1·(1-skill)`` when
    no measured fit is on the driver JSON.
6.  Consistency noise. When the driver has ``consistency_sigma > 0``, the
    controller's slip_target is perturbed per Monte-Carlo run (±5% of
    target), and small Gaussian noise is added to the steer / throttle /
    brake commands (channel stds from ``control_params``).

The skill-derived ``slip_target`` (1 deg at skill=0 -> 6 deg at skill=1)
is what gives the controller its "skill" character. A low-skill driver
*intentionally* uses less of the tyre slip envelope, which translates
through the Pacejka curve into less cornering force and therefore a
slower lap.

When the new controller produces commands that put the car into a state
the abort guards would catch (spin: ``|omega_yaw| > 4 rad/s``;
deep over-slip: ``|alpha_front_avg| > 3 * slip_target``), we hand
control to the embedded :class:`GhostDriver` for that single timestep
and log a one-shot warning. Subsequent steps return to the Phase 4
controller. This keeps the abort guards from firing on transient
controller missteps that the Stanley controller can absorb.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

import numpy as np

from ._control_params import ControlParams
from ._ghost_driver import GhostControlParams, GhostDriver
from .vehicle import Controls, VehicleState

if TYPE_CHECKING:
    from ..driver import Driver
    from ..track import Track

log = logging.getLogger(__name__)


class DriverController:
    """Phase-4 / 4.1 preview-target controller with slip-aware speed loop.

    Reads the v2 racing-line speed plan unscaled (we trust the v2 plan as
    the *target* speed because v2 is already on the friction circle; the
    job of v3 is to track that plan honestly through Pacejka physics).
    A skill-modulated, α_peak-derived slip-angle target drives a throttle
    gain that pushes harder when the car is under the tyre's peak and
    eases off when it is over it.

    Phase 4.1 (§23.10.5):
    - Longitudinal preview is **time-based** (``preview_time_s``) and
      tracks the **min target speed** in the window — pre-braking.
    - Throttle rate-limit from v1.2-measured pedal slew applied before
      slip-band modulators.
    - α_peak-derived slip target replaces the v3.0 ``6·skill + 1`` formula.
    - The ``target_speed_scale`` kwarg is DeprecatedKwarg (always 1.0
      from ``_make_controller``); retained for ABI compat. Removal in v3.2.

    Interface contract (matches :class:`GhostDriver`):

    - ``.controls(state, t, track=None) -> Controls``

    Falls back to an internal :class:`GhostDriver` instance for any step
    where the controller would produce a divergent command (spin
    forming, deep over-slip). Logs once per fallback band.
    """

    # Slip-target thresholds, multiples of ``slip_target_rad`` (spec §23.8
    # plus Phase 4 brief).
    _SLIP_UNDER_BAND = 0.9    # below 0.9x: push harder
    _SLIP_OVER_BAND = 1.1     # above 1.1x: ease off
    _SLIP_HARD_CAP = 1.3      # above 1.3x: cut throttle 30%
    _SLIP_PANIC = 3.0         # above 3.0x: hand off to ghost for this step

    # Spin-fallback threshold. The solver's hard abort is 5 rad/s; we
    # hand off to the ghost driver earlier so the abort never fires
    # mid-lap on a recoverable yaw excursion.
    _SPIN_FALLBACK_RAD_S = 4.0

    def __init__(
        self,
        driver: "Driver",
        track: "Track",
        params: ControlParams | None = None,
        *,
        rng_seed: int | None = None,
        target_speed_ds: np.ndarray | None = None,
        target_speeds: np.ndarray | None = None,
        target_speed_scale: float = 1.0,  # DeprecatedKwarg (v3.1): always 1.0 in _make_controller
        slip_target_rad_override: float | None = None,
        line_xs: np.ndarray | None = None,
        line_ys: np.ndarray | None = None,
    ) -> None:
        if not getattr(track, "is_csv_backed", False):
            raise ValueError("DriverController needs a CSV-backed track.")
        self.driver = driver
        self.params = params if params is not None else ControlParams.from_driver(driver)
        # Skill-derived slip target with optional override (Monte Carlo).
        if slip_target_rad_override is not None:
            self.slip_target_rad = float(slip_target_rad_override)
        elif self.params.slip_target_deg is not None:
            self.slip_target_rad = math.radians(float(self.params.slip_target_deg))
        else:
            self.slip_target_rad = math.radians(float(driver.derived_slip_target_deg()))

        # Racing-line geometry: centreline by default, optional override
        # for the Tomas-line experiment (2026-05-24). When ``line_xs`` /
        # ``line_ys`` are supplied they MUST be the same length and
        # parameterised against ``track.csv_data['distance_m']`` (the
        # ``ds_c`` grid) so the Stanley projection, preview look-ahead, and
        # min-speed window all stay valid. The solver's off-track abort
        # measures against ``track.csv_data['x','z']`` independently and
        # is unaffected.
        data = track.csv_data
        if line_xs is not None and line_ys is not None:
            self._xs = np.asarray(line_xs, dtype=float)
            self._ys = np.asarray(line_ys, dtype=float)
            if (len(self._xs) != len(data["distance_m"])
                    or len(self._ys) != len(data["distance_m"])):
                raise ValueError(
                    f"DriverController line override length mismatch: "
                    f"line_xs={len(self._xs)}, line_ys={len(self._ys)}, "
                    f"track distance_m={len(data['distance_m'])}"
                )
        else:
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
        self._speed = self._speed * float(target_speed_scale)
        self._total_len = float(self._ds[-1])

        # Sliding-window cursor for nearest-index search.
        self._idx_hint = 0
        # Rate-limit state.
        self._last_steer = 0.0
        self._last_throttle = 0.0
        self._last_t = 0.0
        # Front-axle steering hard limit. The ODE's Ackermann splitter
        # handles per-wheel; this caps the averaged command.
        self._max_steer_rad = math.radians(20.0)
        self._max_steer_rate_rad_s = 10.0

        # Phase 4.1 throttle rate-limit resolution (spec §23.10.5.1 step 3).
        # Precedence: control_params.throttle_rate_limit_pct_s ->
        # driver.profile.dynamic.throttle_ramp_pct_s -> 1000 (unlimited).
        # The rate limit is in **fraction-per-second** units inside the
        # controller (the public spec name uses %/s; divide by 100 here).
        rate_pct_s: float | None = self.params.throttle_rate_limit_pct_s
        if rate_pct_s is None:
            raw = getattr(driver, "raw", {}) or {}
            profile = raw.get("profile") if isinstance(raw, dict) else None
            dynamic = profile.get("dynamic") if isinstance(profile, dict) else None
            if isinstance(dynamic, dict):
                val = dynamic.get("throttle_ramp_pct_s")
                if val is not None:
                    try:
                        rate_pct_s = float(val)
                    except (TypeError, ValueError):
                        rate_pct_s = None
        if rate_pct_s is None or rate_pct_s <= 0:
            rate_pct_s = 1000.0  # effectively unlimited (10x full pedal/s)
        self._throttle_rate_per_s = float(rate_pct_s) / 100.0

        # Phase 4.3 (§23.10.12): slip-aware steering softener. Resolve
        # `alpha_peak_front_rad` once at controller construction. If the
        # Pacejka block is absent (no measured fit), the softener is
        # disabled by storing a non-positive sentinel — the runtime check
        # below short-circuits and the legacy controller behaviour is
        # preserved.
        apf_deg = None
        try:
            apf_deg = driver._alpha_peak_front_deg()
        except Exception:
            apf_deg = None
        if apf_deg is not None and apf_deg > 0:
            # Same clamp as derived_slip_target_deg (spec §23.10.9 risk #2).
            apf_clamped = max(4.0, min(10.0, float(apf_deg)))
            self._alpha_peak_front_rad = math.radians(apf_clamped)
        else:
            self._alpha_peak_front_rad = -1.0  # sentinel: softener disabled
        self._softener_engage = float(self.params.steering_softener_engage)
        self._softener_full = float(self.params.steering_softener_full)
        # Previous-step measured α (front, average). The softener fires on
        # the *measured* α from the prior ODE step (spec §23.10.12.1).
        self._last_alpha_front_avg = 0.0

        # Phase 5.0.5-reactive (2026-05-23): first-order low-pass
        # smoother on the post-softener steer command. Spec offered two
        # alternatives ("Pick whichever is simpler"); we ship the LPF —
        # it has no buffer / no bootstrap / no per-tick reduction and
        # matches the Tier-1 emit-FIR's dominant time constant without
        # the Tier-1 pattern's stochastic-noise re-injection problem
        # (the Tier-1 emit-FIR was OK for bounded unit-vector signals
        # but on a raw radian δ with per-tick consistency-noise
        # injection it builds up into an IIR with too much phase lag —
        # documented empirically in the verification subsection of
        # docs/architecture-slip-model-phase5_0_5-v32-reactive-chatter.md).
        #
        # Discrete first-order LPF (matching the spec literally):
        #     δ_smoothed[k] = α · δ_target[k] + (1-α) · δ_smoothed[k-1]
        # with α = dt / τ and τ = N · dt — i.e. the same
        # `stanley_blend_window_ticks` knob translates to a time
        # constant of N tick-periods (60 ms at N=3 / dt=20 ms). The
        # LPF is applied to the pre-clip raw post-softener δ; the clip
        # / rate-limit / noise stages downstream act on the smoothed
        # value, so noise is added once at emit and never re-injected
        # back into the smoother state.
        #
        # Stanley has no integrator term, so there is no anti-windup
        # state to clear during a smoothed flat output; the rate-limit
        # below remains the lateral safety floor.
        self._steer_blend_window = max(
            0, int(self.params.stanley_blend_window_ticks),
        )
        # LPF state: None until first call seeds it. On ghost-fallback
        # the state is reset so the next normal step doesn't average
        # the recovered ghost emit into the LPF.
        self._steer_lpf_prev: float | None = None

        # Soft state for fallback bookkeeping.
        self._ghost: GhostDriver | None = None
        self._ghost_step_count = 0
        # RNG for consistency noise.
        if rng_seed is None:
            self._rng = np.random.default_rng()
        else:
            self._rng = np.random.default_rng(int(rng_seed))
        self._rng_seed = rng_seed
        # Cache: did we already warn about ghost fallback?
        self._warned_fallback = False

    # --- public surface ---------------------------------------------------

    def controls(self, state: VehicleState, t: float,
                 track: "Track | None" = None) -> Controls:  # noqa: ARG002
        # Project to the racing line.
        idx = self._nearest_index(state.x, state.y)
        v = max(0.0, float(state.v_x))

        # --- preview-target Stanley-style steering ------------------------
        # Aim at the racing-line tangent ``preview_distance_m`` ahead, with
        # a cross-track correction at the *current* line point scaled by
        # 1/(v + soft). Stanley is what Phase 3 used; it is proven stable
        # on Sprint A. The Phase-4 distinction lives in the slip-aware
        # throttle/brake loop below, not in the steering channel.
        pv_idx = self._distance_ahead_idx(idx, self.params.preview_distance_m)
        tangent_preview = self._line_tangent(pv_idx)

        ln_x = float(self._xs[idx])
        ln_y = float(self._ys[idx])
        body_dx = float(state.x) - ln_x
        body_dy = float(state.y) - ln_y
        # Cross-track: positive => car to the *right* of the line.
        tangent_now = self._line_tangent(idx)
        cross = body_dx * math.sin(tangent_now) - body_dy * math.cos(tangent_now)

        # Heading error to the preview-tangent direction.
        heading_err = math.atan2(
            math.sin(tangent_preview - float(state.psi)),
            math.cos(tangent_preview - float(state.psi)),
        )

        # Stanley control law: steer = heading_err + atan(k_cross * cross /
        # (v + soft)). The ``steering_p_gain`` from ControlParams scales the
        # heading-err leg (the spec calls it "P-controller on preview
        # heading error"); k_cross is `control_params.stanley.k_cross`
        # (default 1.0; pre-Phase 5.0.6 was 0.5 which gave an effective
        # lateral bandwidth of 0.5/(40+3) ≈ 0.012 in the chronic Sprint A
        # s=1086 m corner — too low to null sustained-radius cross-track
        # error; see _control_params.stanley_k_cross docstring for the
        # sensitivity sweep).
        softening = 3.0
        k_cross = float(self.params.stanley_k_cross)
        steer_cmd = (
            self.params.steering_p_gain * heading_err
            + math.atan2(k_cross * cross, v + softening)
        )
        # Damp at standing start so psi_err doesn't run away.
        if v < 5.0:
            steer_cmd *= max(0.0, v / 5.0)

        # --- Phase 4.3 (§23.10.12) slip-aware steering softener ----------
        # Apply BEFORE the max_steer_rad clip and BEFORE the steering
        # rate-limit (spec §23.10.12.2 insertion point). Fires on the
        # *previous-step* measured front α (spec §23.10.12.1 — same
        # denominator-independent path as the slip-band throttle
        # modulators, which fire on α / slip_target; the softener fires
        # on α / α_peak — see §23.10.12.5 for the independence rationale).
        # When the Pacejka block is absent (sentinel < 0), the softener
        # is disabled and steer_cmd passes through unchanged.
        if self._alpha_peak_front_rad > 0.0:
            alpha_norm = abs(self._last_alpha_front_avg) / self._alpha_peak_front_rad
            if alpha_norm <= self._softener_engage:
                soften = 1.0
            elif alpha_norm >= self._softener_full:
                soften = 0.0
            else:
                soften = (
                    (self._softener_full - alpha_norm)
                    / (self._softener_full - self._softener_engage)
                )
            steer_cmd *= soften

        # --- Phase 5.0.5-reactive LPF smoother (2026-05-23) ---------------
        # First-order discrete low-pass on the raw post-softener steer,
        # applied BEFORE the magnitude clip / rate-limit / noise so
        # those stages still act as the final safety floor on the
        # smoothed signal AND so noise is never re-injected into the
        # smoother state.
        #
        # α = dt / τ with τ = N · dt → α = 1 / N. At the default N=3
        # (knob: `stanley_blend_window_ticks`) the time constant is
        # 60 ms — matching the spec's stated LPF τ ≈ 60 ms. N=0 or
        # 1 disables the smoother (pass-through). The LPF mirrors the
        # Tier-1 FIR's intent without the recursive emit-FIR's
        # stochastic-noise build-up (see `ControlParams.stanley_blend_
        # window_ticks` docstring for the empirical sweep).
        if self._steer_blend_window > 1:
            n = float(self._steer_blend_window)
            alpha = 1.0 / n
            if self._steer_lpf_prev is None:
                self._steer_lpf_prev = float(steer_cmd)  # seed on first tick
            steer_cmd = (
                alpha * float(steer_cmd)
                + (1.0 - alpha) * self._steer_lpf_prev
            )
            self._steer_lpf_prev = float(steer_cmd)

        steer_cmd = float(np.clip(
            steer_cmd, -self._max_steer_rad, self._max_steer_rad,
        ))

        # Rate-limit so the controller can't snap the front wheels.
        dt_ctrl = max(1e-3, t - self._last_t)
        max_step = self._max_steer_rate_rad_s * dt_ctrl
        d_steer = steer_cmd - self._last_steer
        if d_steer > max_step:
            steer_cmd = self._last_steer + max_step
        elif d_steer < -max_step:
            steer_cmd = self._last_steer - max_step

        # --- preview-target speed: min over a time-based window -----------
        # Phase 4.1 (spec §23.10.5.1 step 1): the lookahead horizon scales
        # with ``v_preview * t_preview`` so the controller looks ~1.5 s
        # ahead in time, not a v²/2a brake-distance proxy. Using
        # ``max(v, target_v_local)`` ensures the window grows in fast
        # sections even before the controller has built speed.
        t_preview = float(self.params.preview_time_s)
        # Use the instantaneous-line speed at idx (not the preview-window
        # min) to seed v_preview, so the horizon scales with the speed
        # we are about to hit, not the speed we're afraid of.
        v_target_local = float(self._speed[idx])
        v_preview = max(v, v_target_local)
        brake_lookahead_m = max(30.0, v_preview * t_preview)
        # Phase 4.1 step 2: brake whenever ``v > v_target_min`` over the
        # preview window (pre-braking). This replaces the v3.0 reactive
        # ``v > v_target_local`` brake trigger that only fired after the
        # car had already passed the brake point. Throttle continues to
        # target the **local** line speed so we still accelerate up to
        # apex speed in fast sections — but throttle is gated off when
        # the preview min is below current v (don't push throttle into
        # a corner we know we have to brake for).
        v_target_min = self._min_speed_within(idx, brake_lookahead_m)
        e_v_throttle = v_target_local - v
        e_v_brake = v_target_min - v
        v_target = v_target_min  # "effective" target used by downstream slip-band logic

        # --- throttle / brake P-controllers -------------------------------
        if e_v_brake >= 0:
            throttle = float(np.clip(
                self.params.throttle_p_gain * max(e_v_throttle, 0.0), 0.0, 1.0))
        else:
            throttle = 0.0
        brake = float(np.clip(
            self.params.brake_p_gain * max(-e_v_brake, 0.0), 0.0, 1.0))
        e_v = e_v_brake  # downstream slip-band soft-start uses the brake error

        # Soft start: when nearly stopped, full throttle to break the
        # quasi-static integrator's dead zone.
        if v < 1.0 and v_target > 2.0:
            throttle = 1.0
            brake = 0.0

        # --- throttle rate limit (spec §23.10.5.1 step 3) -----------------
        # Applied FIRST so the slip-band modulators below operate on the
        # rate-limited output (spec §23.10.9 risk #3: order-dependent
        # interaction otherwise).
        dt_throttle = max(1e-3, t - self._last_t)
        max_throttle_step = self._throttle_rate_per_s * dt_throttle
        d_throttle = throttle - self._last_throttle
        if d_throttle > max_throttle_step:
            throttle = self._last_throttle + max_throttle_step
        elif d_throttle < -max_throttle_step:
            throttle = self._last_throttle - max_throttle_step
        throttle = float(np.clip(throttle, 0.0, 1.0))

        # --- slip-target enforcement --------------------------------------
        # Compute the current average front-wheel slip angle from the
        # body-frame velocity at the front-axle contact patch. The exact
        # per-wheel alpha is computed inside compute_derivatives, but
        # since we only need the *average* (used as a throttle modulator)
        # we approximate from chassis state — body slip + yaw-rate
        # geometry.
        wb = 0.0
        # Best-effort wheelbase pull-back; if unavailable, fall back to
        # a sane default.
        try:
            wb_attr = getattr(self.driver, "_wheelbase_cache", None)
            wb = float(wb_attr) if wb_attr is not None else 2.66
        except Exception:
            wb = 2.66
        a_front = wb * 0.5  # rough — proper offset requires CarDynamics
        # Lateral velocity at the front axle, body frame.
        vy_front = float(state.v_y) + float(state.omega_yaw) * a_front
        denom = max(abs(float(state.v_x)), 0.5)
        # Approximation: alpha_front ~= steer_cmd - atan2(vy_front, v_x).
        # The actual per-wheel alpha includes the Ackermann split, but
        # for the *average* the simple linearisation is good enough.
        body_slip_front = math.atan2(vy_front, denom)
        alpha_front_avg = steer_cmd - body_slip_front

        slip_ratio = abs(alpha_front_avg) / max(self.slip_target_rad, 1e-3)

        # Phase 4.1 (spec §23.10.5.1 step 5): the in-controller
        # ``slip_ratio > 0.85`` feedforward speed-target clamp is REMOVED.
        # With proper pre-braking via the preview window above, the
        # controller doesn't need a band-aid clamp; the slip-band
        # modulators below remain as post-hoc safety.
        if slip_ratio < self._SLIP_UNDER_BAND and e_v > 0 and throttle < 1.0:
            # Under-using the tyre and we want more speed: small boost.
            throttle = float(min(1.0, throttle + 0.05))
        elif slip_ratio > self._SLIP_HARD_CAP:
            # Deep over-slip — soft cap throttle by 30%.
            throttle = float(throttle * 0.70)
            # And nudge a touch of trail-brake to bleed speed.
            if abs(steer_cmd) > 0.1:
                brake = float(min(1.0, brake + 0.05))
        elif slip_ratio > self._SLIP_OVER_BAND:
            # Ease off, but not as hard as the hard cap.
            throttle = float(throttle * 0.85)

        # Trail-braking: gradually release brake as steer angle increases.
        # Linear taper between 0 and ~12 deg of steer; at 12 deg, brake
        # is halved. This avoids locking the inside front in a deep
        # turn-in.
        steer_taper_threshold = math.radians(12.0)
        if brake > 0 and abs(steer_cmd) > 0.01:
            taper = 1.0 - 0.5 * min(1.0, abs(steer_cmd) / steer_taper_threshold)
            brake = float(brake * taper)

        # --- spin / over-slip fallback to ghost driver --------------------
        # If the controller is about to put the car somewhere the abort
        # guards would catch, hand control to GhostDriver for this step
        # so the lap can recover.
        spinning = abs(float(state.omega_yaw)) > self._SPIN_FALLBACK_RAD_S
        deep_over_slip = slip_ratio > self._SLIP_PANIC
        if spinning or deep_over_slip:
            ghost_cmd = self._ghost_fallback().controls(state, t)
            self._ghost_step_count += 1
            if not self._warned_fallback:
                log.warning(
                    "DriverController -> ghost-fallback at t=%.2fs "
                    "(spin=%s, over_slip=%s, omega=%.2f, slip_ratio=%.2f)",
                    t, spinning, deep_over_slip,
                    float(state.omega_yaw), slip_ratio,
                )
                self._warned_fallback = True
            self._last_steer = float(ghost_cmd.steer_rad)
            self._last_throttle = float(ghost_cmd.throttle)
            self._last_t = float(t)
            self._last_alpha_front_avg = float(alpha_front_avg)
            # Sync the LPF state with what we actually emitted so the
            # next normal step blends FROM the ghost's recovered steer
            # rather than from the pre-fallback Stanley command.
            self._steer_lpf_prev = float(ghost_cmd.steer_rad)
            return ghost_cmd

        # --- consistency noise -------------------------------------------
        # Per-channel Gaussian noise. Only when the driver has
        # consistency_sigma > 0 (so deterministic skill-only runs are
        # bit-stable).
        sigma = float(getattr(self.driver, "consistency_sigma", 0.0))
        if sigma > 0:
            steer_noise_rad = math.radians(
                self.params.consistency_noise_std_steer_deg
            ) * float(self._rng.standard_normal())
            throttle_noise = (
                self.params.consistency_noise_std_throttle_pct / 100.0
            ) * float(self._rng.standard_normal())
            brake_noise = (
                self.params.consistency_noise_std_brake_pct / 100.0
            ) * float(self._rng.standard_normal())
            steer_cmd = float(np.clip(
                steer_cmd + steer_noise_rad,
                -self._max_steer_rad, self._max_steer_rad,
            ))
            throttle = float(np.clip(throttle + throttle_noise, 0.0, 1.0))
            brake = float(np.clip(brake + brake_noise, 0.0, 1.0))

        # Bookkeeping.
        self._last_steer = float(steer_cmd)
        self._last_throttle = float(throttle)
        self._last_t = float(t)
        # Phase 4.3: cache this step's measured α_front_avg so the next
        # call's softener block can read it. Same approximation used by
        # the slip-band throttle modulator above (§23.10.12.1 note —
        # this is the controller's existing measured-α path, no new
        # tyre-state plumbing).
        self._last_alpha_front_avg = float(alpha_front_avg)
        # Phase 5.0.5-reactive LPF: state already updated at the
        # smoother insertion point with the pre-clip smoothed value.
        # The post-clip / post-noise emitted δ is NOT folded back in
        # — that would create a recursive emit-LPF identical in
        # pathology to the Tier-1 pattern we deliberately rejected.

        return Controls(steer_rad=float(steer_cmd),
                        throttle=float(throttle),
                        brake=float(brake))

    # --- helpers ----------------------------------------------------------

    def _ghost_fallback(self) -> GhostDriver:
        """Lazy-create the embedded ghost driver used for transient fallback."""
        if self._ghost is None:
            # Build a fresh GhostDriver pointing at the same speed plan we
            # are using ourselves. Same scale (1.0) — the ghost would
            # normally be 0.85 in Phase 3, but here it is only the safety
            # net for a single timestep.
            class _FakeTrack:
                is_csv_backed = True
                csv_data = {
                    "x": self._xs, "z": self._ys,
                    "distance_m": self._ds, "speed_ms": self._speed,
                }
            self._ghost = GhostDriver(
                _FakeTrack(),  # type: ignore[arg-type]
                preview_distance_m=10.0,
                target_speed_scale=1.0,
            )
        return self._ghost

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

    def _distance_ahead_idx(self, idx: int, dist_m: float) -> int:
        s_here = float(self._ds[idx])
        s_target = min(s_here + max(0.0, dist_m), self._total_len - 1e-3)
        j = int(np.searchsorted(self._ds, s_target))
        return max(idx, min(j, len(self._ds) - 1))

    def _line_tangent(self, idx: int) -> float:
        n = len(self._xs)
        i0 = max(0, idx - 2)
        i1 = min(n - 1, idx + 2)
        if i1 <= i0:
            i1 = min(n - 1, i0 + 1)
        return float(np.arctan2(self._ys[i1] - self._ys[i0],
                                self._xs[i1] - self._xs[i0]))

    def _min_speed_within(self, idx: int, lookahead_m: float) -> float:
        s_here = float(self._ds[idx])
        s_end = min(s_here + lookahead_m, self._total_len - 1e-3)
        end_idx = int(np.searchsorted(self._ds, s_end))
        end_idx = max(idx + 1, min(end_idx, len(self._ds) - 1))
        return float(self._speed[idx:end_idx + 1].min())


# Backwards-compat re-export: the legacy ``ControlParams`` import path is
# preserved so callers that did `from .driver_controller import ControlParams`
# continue to work after the Phase 4 split.
__all__ = ["ControlParams", "DriverController", "GhostDriver"]
