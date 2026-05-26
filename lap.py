#!/usr/bin/env python3
"""Lap Time Estimator - simulator CLI (car / track / driver).

v2 (spec §21): multi-lap stint mode (`--laps N`, default 2 for back-compat),
inverse-PSI solver (`--solve-pressure-for-wear`), per-wheel cold-pressure setup
files (`--setup`, `--pressure`, `--ambient-temp-c`). Legacy single-lap and
two-lap behaviour is preserved (§11.30, §11.31).
"""
from __future__ import annotations

import _bootstrap  # noqa: F401  (puts src/ on sys.path)

import argparse
import os
import sys

from lap_estimator.car import Car
from lap_estimator.driver import Driver
from lap_estimator.report import (
    build_output_stem,
    plot_speed_overlay,
    print_per_lap_block,
    write_comparison_plot,
    write_stint_summary_csv,
    write_trace_csv,
)
from lap_estimator.setup import resolve_compound, resolve_setup
from lap_estimator.sim_telemetry import write_synthetic_log
from lap_estimator.simulator import (
    print_report,
    simulate,
    simulate_monte_carlo,
    simulate_stint,
)
from lap_estimator.solve_setup import solve_pressure_for_wear
from lap_estimator.track import BUILTIN_TRACKS, Track
from lap_estimator.validate import validate_lap, write_bins_csv


def find_car_data(car_path):
    if os.path.isfile(os.path.join(car_path, "engine.ini")):
        return car_path
    data_dir = os.path.join(car_path, "data")
    if os.path.isdir(data_dir) and os.path.isfile(os.path.join(data_dir, "engine.ini")):
        return data_dir
    raise FileNotFoundError(f"No car data (engine.ini) found in {car_path}")


def resolve_track(arg):
    """Return (track_obj, source_kind, source_path)."""
    if os.path.isfile(arg) and arg.lower().endswith(".csv"):
        return Track.from_csv(arg), "csv", arg
    if os.path.isfile(arg) and arg.lower().endswith(".json"):
        return Track.from_json(arg), "json", arg
    if arg in BUILTIN_TRACKS:
        return BUILTIN_TRACKS[arg](), "builtin", arg
    raise ValueError(
        f"Track '{arg}' not found. Provide a .csv, .json, or one of: "
        f"{', '.join(BUILTIN_TRACKS)}"
    )


def main():
    parser = argparse.ArgumentParser(description="Lap Time Estimator")
    parser.add_argument("car", help="Path to car data directory")
    parser.add_argument("track", help="Track CSV path, JSON path, or built-in name")
    parser.add_argument("driver", help="Path to driver JSON (v1.1: JSON only, no YAML)")
    parser.add_argument("--ds", type=float, default=2.0)
    parser.add_argument("--all-tracks", action="store_true",
                        help="Run on all built-in tracks (ignores positional track)")
    parser.add_argument("--single-lap", action="store_true",
                        help="Alias for --laps 1. Mutually exclusive with --laps.")
    parser.add_argument("--laps", type=int, default=None,
                        help="Number of laps in the stint (1..50, default 2). "
                             "--laps 1 is back-compat single-lap; --laps 2 matches "
                             "the v1.1 two-lap default byte-for-byte (§11.31); "
                             "--laps N for N>=3 engages stint mode (§21.4).")
    parser.add_argument("--setup", default=None,
                        help="Path to a setup JSON (see setups/<car>_*.json).")
    parser.add_argument("--pressure", default=None,
                        help="Per-wheel cold pressure override: "
                             "FL=31,FR=31,RL=29,RR=29 (subset allowed).")
    parser.add_argument("--ambient-temp-c", type=float, default=None,
                        help="Ambient temperature (°C). Overrides setup file. Default 25.")
    parser.add_argument("--compound", default=None,
                        help="Active tyre compound name or short-name (e.g. Semislicks or SM). "
                             "Overrides setup-JSON `compound`. Case-insensitive; trailing "
                             "parenthetical short-name is stripped (so 'Semislicks (SM)' "
                             "matches). Unknown name -> argparse error.")
    parser.add_argument("--solve-pressure-for-wear", type=float, default=None,
                        help="Inverse-solver mode: target wear (0..1) at --at-lap.")
    parser.add_argument("--at-lap", type=int, default=None,
                        help="Target lap for --solve-pressure-for-wear (required with that flag).")
    parser.add_argument("--target-wheel", default="max",
                        choices=("max", "min", "avg", "FL", "FR", "RL", "RR"),
                        help="Aggregator for --solve-pressure-for-wear (default max).")
    parser.add_argument("--uniform-pressure", action="store_true",
                        help="Solve a single PSI applied to all 4 wheels.")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument("--telemetry-dt-ms", type=int, default=10,
                        help="Cadence (ms) of the synthetic telemetry CSV. "
                             "v1.2 default is 10 (was 100); pass 100 to revert.")
    parser.add_argument("--validate-against", default=None,
                        help="Path to real AC telemetry CSV for cross-track validation.")
    parser.add_argument("--bin-m", type=int, default=100)
    parser.add_argument("--per-corner", action="store_true")
    parser.add_argument("--model", default="point-mass",
                        choices=("point-mass", "slip"),
                        help="Simulator model. 'point-mass' (default) is the v2 "
                             "kinematic 3-pass simulator. 'slip' is the v3 "
                             "slip-based dynamics model (Pacejka + RK4 ODE + "
                             "driver controller) — Phase 1 scaffolding only.")
    parser.add_argument("--plan-source", default="v3_dp",
                        choices=("v2", "v3_dp", "tomas"),
                        help="Target speed plan source for the v3 slip controller "
                             "(spec §23.10.6). 'v3_dp' (default) runs a forward-"
                             "backward DP against the fitted Pacejka envelope. "
                             "'v2' uses the legacy point-mass plan (regression "
                             "path; known to abort on Sprint A with measured "
                             "Pacejka). 'tomas' (experimental, 2026-05-24) uses "
                             "Tomas's recorded lap-5 v(s) directly as the "
                             "controller reference -- no DP, no safety_margin, "
                             "no chicane cap; the recorded trajectory IS the "
                             "plan. CSV path overridable via --tomas-csv. See "
                             "docs/architecture-v3-tomas-trajectory-injection.md. "
                             "Ignored when --model point-mass.")
    parser.add_argument("--tomas-csv", default=None,
                        help="Override the input telemetry CSV consumed by "
                             "--plan-source tomas. Default: "
                             ".tmp/tomas_lap5_rich.csv. The CSV must have at "
                             "minimum 'distanceTraveled' and 'speedKmh' columns "
                             "and cover exactly one lap of the active track.")
    parser.add_argument("--line-source", default="center",
                        choices=("center", "tomas"),
                        help="Racing-line source for the v3 slip controllers' "
                             "Frenet projection (2026-05-24 experiment). "
                             "'center' (default) is the centreline from "
                             "the track CSV. 'tomas' reconstructs Tomas's "
                             "recorded (x, z) trajectory by integrating his "
                             "body-frame velocity rotated by his world "
                             "heading (the canonical lap-5 CSV has no "
                             "carCoordinates_* channels). Solver off-track "
                             "abort still measures against the centreline. "
                             "Test matrix: --line-source tomas --plan-source "
                             "tomas (line + speed), or --line-source tomas "
                             "--plan-source v3_dp (line only). See "
                             "docs/architecture-v3-tomas-trajectory-injection.md "
                             "'Line override' section.")
    parser.add_argument("--controller", default="reactive",
                        choices=("reactive", "mpc", "mpcc", "hmpc",
                                 "pi", "ffpi", "safe_pi"),
                        help="Slip-model controller (spec §23.2.12, Phase 5.0). "
                             "'reactive' (default during Phase 5.0 dev) is the "
                             "Phase 4.x preview-Stanley + slip-band P-loop. "
                             "'mpc' is the Phase 5.0 receding-horizon MPC with "
                             "OSQP inner solver + 3-iter SQP outer loop. "
                             "'mpcc' is the v3.3 Model Predictive Contouring "
                             "Controller (spec §23.3): curvilinear (s, n, ψ_e) "
                             "OCP that rewards progress along a reference path "
                             "instead of tracking v_max(s); steering-only "
                             "commit, throttle/brake delegated to the reactive "
                             "sub-controller. "
                             "'pi' is the deliberately-dumb cascade PI baseline "
                             "(physics-envelope smoke test; emits per-tick "
                             "diagnostic CSV to .tmp/pi_diag_sprint_a.csv). "
                             "'ffpi' is the feedforward-dominant + bounded-PI "
                             "baseline (Ackermann steer FF + DP-derived "
                             "throttle/brake FF + PI trim clipped to a "
                             "bounded envelope; emits "
                             ".tmp/ffpi_diag_sprint_a.csv). "
                             "'safe_pi' is the deliberately-conservative "
                             "'just finish the lap' baseline (centerline + "
                             "0.7 * v_max_DP, two independent PI loops, slew "
                             "limits on every actuator; emits "
                             ".tmp/safe_pi_diag_sprint_a.csv). "
                             "Ignored when --model point-mass.")
    parser.add_argument("--mpc-horizon-m", type=float, default=None,
                        help="MPC horizon length in metres (spec §23.2.3, default 30). "
                             "Only consumed with --controller mpc.")
    parser.add_argument("--dp-safety-margin", type=float, default=None,
                        help="DP planner safety_margin override (spec §23.2-5.0.1.5, "
                             "Phase 5.0.1 default 0.94; pre-Phase-5.0.1 historical "
                             "value was 0.97). Only applies when --model slip and "
                             "--plan-source v3_dp.")
    parser.add_argument("--chicane-safety-mult", type=float, default=None,
                        help="Phase 5.0.2 chicane-safety multiplier (spec "
                             "§23.2-5.0.2). Applied to v_corner on segments with "
                             "radius < --chicane-radius-thresh, with a 5-segment "
                             "ramp-in / ramp-out. Default 0.80 (20% margin). "
                             "Pass 1.0 to disable. Overrides driver JSON "
                             "control_params.chicane.safety_mult. Only applies "
                             "when --model slip and --plan-source v3_dp.")
    parser.add_argument("--chicane-radius-thresh", type=float, default=None,
                        help="Phase 5.0.2 chicane-safety radius threshold in metres "
                             "(spec §23.2-5.0.2). Segments with radius_m below this "
                             "are flagged 'tight'. Default 60.0 (Sprint A chicane "
                             "apex is ~27 m). Overrides driver JSON "
                             "control_params.chicane.radius_thresh_m.")
    parser.add_argument("--mpc-tier1-disable", action="store_true",
                        help="Phase 5.0.3 (spec §23.2-5.0.3.9): force the MPC "
                             "Tier-1 ellipse-saturation feedforward off. With "
                             "tier1 disabled, the controller falls straight to "
                             "the reactive sub-controller (Tier 2) on QP "
                             "infeasibility — useful for A/B regression "
                             "measurement against Phase 5.0.2 byte-for-byte. "
                             "Only applies with --model slip --controller mpc.")
    parser.add_argument("--mpc-tier1-max-consecutive", type=int, default=None,
                        help="Phase 5.0.3 (spec §23.2-5.0.3.9): override the "
                             "per-driver Tier-1 consecutive-tick cap before "
                             "escalating to Tier 2. Default 10 (50 Hz × 200 ms). "
                             "Overrides driver JSON "
                             "control_params.mpc.tier1.max_consecutive_ticks.")
    parser.add_argument("--static-fz", action="store_true",
                        help="Phase 5.0.4 (spec §23.2-5.0.4.8): force the MPC "
                             "plant to use STATIC per-axle Fz, disabling the "
                             "Phase 5.0.4 dynamic-Fz refresh. Regression A/B "
                             "knob against Phase 5.0.3. Only applies with "
                             "--model slip --controller mpc.")
    parser.add_argument("--mpc-emit-source", type=str, default="qp",
                        choices=["qp", "sub"],
                        help="Phase 5.0.8 (spec OP-4): MPC Tier-0 emit source "
                             "for throttle/brake. 'qp' (default) emits the "
                             "QP-solved (throttle, brake) directly — "
                             "first-class longitudinal channel, closes the "
                             "Phase 5.0.7 weight-retune diagnosis of why "
                             "weights alone could not move v_x at the "
                             "chicane. 'sub' reverts to the pre-5.0.8 "
                             "delegated behaviour where the reactive "
                             "sub-controller's throttle/brake is committed; "
                             "use for A/B regression against 5.0.7. Tier 1 "
                             "and Tier 2 emit paths are unaffected. Only "
                             "applies with --model slip --controller mpc.")
    parser.add_argument("--cimpcc-weight", type=float, default=None,
                        help="Phase 5.0.8 (CiMPCC, arXiv:2502.03695): add a "
                             "curvature-weighted velocity hinge cost "
                             "w_kappa * max(0, v_x - v_kappa_safe(kappa))^2 "
                             "to the MPC stage cost. v_kappa_safe = "
                             "sqrt(D_lat * g / |kappa| * safety). Default None "
                             "(overlay DISABLED — bit-identical to pre-5.0.8). "
                             "Pass e.g. 500 to enable; ~500 is the spec seed. "
                             "Compare to w_v ~ 0.5 — CiMPCC is intentionally "
                             "much larger because it fires asymmetrically. "
                             "Only applies with --model slip --controller mpc.")
    parser.add_argument("--cimpcc-safety", type=float, default=None,
                        help="Phase 5.0.8 CiMPCC safety multiplier on "
                             "v_kappa_safe. Default 0.95 (5%% margin below "
                             "the geometric friction limit). Pass 1.0 to ride "
                             "the limit exactly. Only meaningful when "
                             "--cimpcc-weight > 0.")
    # v3.3 MPCC CLI flags (spec §23.3.6.8). All optional; defaults applied
    # inside MPCCController.__init__ after consulting driver JSON
    # ``control_params.mpcc``.
    parser.add_argument("--mpcc-horizon-m", type=float, default=None,
                        help="MPCC horizon length in metres (spec §23.3.6.4, "
                             "default 50 m). Stages are uniform in dθ. Sweep "
                             "30 / 40 / 50 / 60 to pick the smallest that "
                             "passes the lap-time gate. Only consumed with "
                             "--controller mpcc.")
    parser.add_argument("--mpcc-n-stages", type=int, default=None,
                        help="MPCC number of stages (spec §23.3.6.4, default 25). "
                             "dθ_stage = horizon / n_stages. Only consumed with "
                             "--controller mpcc.")
    parser.add_argument("--mpcc-tick-hz", type=float, default=None,
                        help="MPCC tick rate (Hz, spec §23.3.10.1, default 10). "
                             "Liniger default; the v3.2 MPC uses 50 Hz to "
                             "stabilise the LTV linearisation but MPCC's "
                             "progress-cost should not need it. Raise if "
                             "chassis drift between ticks exceeds 1 m. "
                             "Only consumed with --controller mpcc.")
    parser.add_argument("--mpcc-w-contour", type=float, default=None,
                        help="MPCC contouring weight (spec §23.3.7.3, default "
                             "100.0). Penalises perpendicular path error n². "
                             "Only consumed with --controller mpcc.")
    parser.add_argument("--mpcc-w-lag", type=float, default=None,
                        help="MPCC lag weight (spec §23.3.7.3, default 1000.0). "
                             "Penalises (s − θ)² mismatch. Higher than contour "
                             "by design — keeps virtual progress glued to "
                             "actual progress. Only consumed with --controller mpcc.")
    parser.add_argument("--mpcc-w-progress", type=float, default=None,
                        help="MPCC progress reward weight (spec §23.3.7.3, "
                             "default 2.0). Stored positive; the cost subtracts "
                             "w_progress · V_θ_k so larger values push for more "
                             "aggression. Sweep 2 → 10 if the lap-time gate "
                             "isn't met. Only consumed with --controller mpcc.")
    parser.add_argument("--mpcc-w-du", type=float, default=None,
                        help="MPCC input-rate smoothness weight (spec "
                             "§23.3.7.3, default 1.0). Penalises ‖u_k − u_{k-1}‖². "
                             "Only consumed with --controller mpcc.")
    parser.add_argument("--mpcc-force-static-fz", action="store_true",
                        help="Force the MPCC plant to STATIC per-axle Fz "
                             "(disabling the Phase 5.0.4 dynamic-Fz refresh). "
                             "Regression A/B knob; only applies with "
                             "--model slip --controller mpcc.")
    # ----- v3.4 HMPC CLI flags (spec §23.4.6.7). All optional; defaults
    # applied inside HMPCController.__init__ after consulting driver JSON
    # `control_params.hmpc`.
    parser.add_argument("--hmpc-outer-horizon-m", type=float, default=None,
                        help="HMPC outer planner horizon length in metres "
                             "(spec §23.4.6.2 amendment 2026-05-24, default 500). "
                             "Stages are uniform in ds_outer = horizon / n_stages. "
                             "Only consumed with --controller hmpc.")
    parser.add_argument("--hmpc-outer-n-stages", type=int, default=None,
                        help="HMPC outer planner stage count (default 50; at "
                             "the 500 m horizon default = 10 m per stage). "
                             "Only consumed with --controller hmpc.")
    parser.add_argument("--hmpc-outer-ds-m", type=float, default=None,
                        help="HMPC outer planner per-stage length (m). "
                             "Convenience: if set, takes precedence over the "
                             "horizon/n_stages combination by adjusting "
                             "n_stages = round(horizon_m / ds_m). Only "
                             "consumed with --controller hmpc.")
    parser.add_argument("--hmpc-outer-rate-hz", type=float, default=None,
                        help="HMPC outer planner cadence (Hz, default 1.0 — "
                             "spec §23.4.6.2 amendment, longer reference "
                             "window justifies slower re-solve). Inner ticks "
                             "between outer fires consume the frozen "
                             "reference. Only consumed with --controller hmpc.")
    parser.add_argument("--hmpc-outer-mu-circle", type=float, default=None,
                        help="HMPC outer planner friction-circle mu (unit-Fz). "
                             "Default None → 0.85 · min(D_lat, D_long) per "
                             "spec §23.4.11 Risk 1. Sweep 0.85 → 0.75 → 0.65 "
                             "if PI trim saturates. Only consumed with "
                             "--controller hmpc.")
    parser.add_argument("--hmpc-outer-w-progress", type=float, default=None,
                        help="HMPC outer progress weight (default 1.0). "
                             "Pushes outer v_ref up. Sweep 1 → 5 once "
                             "completion gate is met (spec §23.4.12 step 5). "
                             "Only consumed with --controller hmpc.")
    parser.add_argument("--hmpc-outer-w-n", type=float, default=None,
                        help="HMPC outer cross-track weight (default 5.0). "
                             "Only consumed with --controller hmpc.")
    parser.add_argument("--hmpc-outer-w-du", type=float, default=None,
                        help="HMPC outer rate weight (default 0.5). Smooths "
                             "(a_long, a_lat) between stages. Only consumed "
                             "with --controller hmpc.")
    parser.add_argument("--hmpc-inner-horizon-m", type=float, default=None,
                        help="HMPC inner tracker horizon length in metres "
                             "(default 30 — v3.2 MPC parity). Only consumed "
                             "with --controller hmpc.")
    parser.add_argument("--hmpc-inner-n-stages", type=int, default=None,
                        help="HMPC inner tracker stage count (default 15 — "
                             "v3.2 MPC parity). Only consumed with "
                             "--controller hmpc.")
    parser.add_argument("--hmpc-inner-tick-hz", type=float, default=None,
                        help="HMPC inner tick rate (Hz, default 50.0 — v3.2 "
                             "MPC parity). Only consumed with --controller "
                             "hmpc.")
    parser.add_argument("--hmpc-pi-kp-n", type=float, default=None,
                        help="HMPC PI trim proportional gain on cross-track "
                             "(default 0.05 rad/m). Only consumed with "
                             "--controller hmpc.")
    parser.add_argument("--hmpc-pi-ki-n", type=float, default=None,
                        help="HMPC PI trim integral gain on cross-track "
                             "(default 0.01 rad/(m·s)).")
    parser.add_argument("--hmpc-pi-kp-vx", type=float, default=None,
                        help="HMPC PI trim proportional gain on v_x error "
                             "(default 0.02 per (m/s)).")
    parser.add_argument("--hmpc-pi-ki-vx", type=float, default=None,
                        help="HMPC PI trim integral gain on v_x error "
                             "(default 0.005 per (m/s·s)).")
    parser.add_argument("--hmpc-pi-trim-bound-steer-pct", type=float, default=None,
                        help="HMPC PI trim steering bound as fraction of "
                             "delta_max (default 0.15 — spec §23.4.6.4). "
                             "Hard-caps the trim's authority so it cannot "
                             "override the inner MPC.")
    parser.add_argument("--hmpc-pi-trim-bound-pedal-pct", type=float, default=None,
                        help="HMPC PI trim throttle/brake bound in absolute "
                             "[0, 1] units (default 0.10 — spec §23.4.6.4). "
                             "Same authority cap on the longitudinal channel.")
    parser.add_argument("--hmpc-outer-disable", action="store_true",
                        help="Disable the HMPC outer planner. The inner "
                             "tracks the DP plan directly (effectively "
                             "`--controller mpc`); the A/B harness for "
                             "isolating outer contribution (spec §23.4.6.6).")
    parser.add_argument("--hmpc-force-static-fz", action="store_true",
                        help="Force the HMPC inner plant to STATIC per-axle "
                             "Fz. Regression A/B knob.")
    parser.add_argument("--hmpc-emit-source", type=str, default="qp",
                        choices=["qp", "sub"],
                        help="HMPC tier-0 pedal emit source. 'qp' (default) "
                             "commits the inner's first-stage throttle/brake "
                             "plus the PI trim (delivers the brake-"
                             "anticipation mechanism by letting the outer's "
                             "v_ref reach the chassis through the inner's "
                             "w_v cost). 'sub' commits the reactive sub-"
                             "controller's pedals plus the PI trim only "
                             "(literal-spec emit; A/B regression).")
    parser.add_argument("--hmpc-inner-solver", type=str, default="osqp",
                        choices=["osqp", "casadi"],
                        help="HMPC inner-tracker solver class. 'osqp' "
                             "(default) is the v3.2 condensed LTV-bicycle "
                             "QP solved by OSQP+SQP. 'casadi' is the "
                             "Phase 5.3 nonlinear inner: full bicycle "
                             "dynamics + per-axle friction-ellipse "
                             "constraint solved natively by IPOPT. "
                             "Only consumed with --controller hmpc.")
    parser.add_argument("--hmpc-debug-trace", action="store_true",
                        help="Write a per-tick HMPC trace CSV to "
                             ".tmp/hmpc_diag.csv (spec §23.4.6.5). Off by "
                             "default.")
    # Powertrain calibration overrides (BMW 1M straight-line shortfall, 2026-05-22).
    # See docs/architecture-bmw1m-powertrain-calibration.md.
    parser.add_argument("--boost-steady", type=float, default=None,
                        help="Override [TURBO_0].WASTEGATE (steady-state turbo "
                             "boost multiplier). The engine torque model is "
                             "`T * (1 + boost)`, so 0.55 -> +55%% steady boost. "
                             "Default: None (use the ini WASTEGATE value, "
                             "0.46 for the BMW 1M). MAX_BOOST in the ini is "
                             "0.85 (instantaneous peak before the wastegate "
                             "vents).")
    parser.add_argument("--cd-override", type=float, default=None,
                        help="Override the body drag coefficient sampled from "
                             "WING_0.LUT_AOA_CD at AOA=0 (BMW 1M default 0.34). "
                             "Pass e.g. 0.32 to model a leaner aero pack.")
    parser.add_argument("--brake-torque-mult", type=float, default=None,
                        help="Multiply the brake_torque (brakes.ini MAX_TORQUE) "
                             "by this factor. v3 longitudinal-physics fix "
                             "(2026-05-23): the BMW 1M ini's 3200 Nm budget "
                             "is ~1.80x lower than peak ~17-22 kN brake force "
                             "in real-Tomas telemetry; pass 1.80 to match. "
                             "Per-driver override -- NOT baked into the ini.")
    parser.add_argument("--inertia-zz", type=float, default=None,
                        help="Override chassis yaw moment of inertia I_zz "
                             "(kg.m^2). v3 lateral yaw-inertia fix "
                             "(2026-05-23): the prior plate-formula default "
                             "(~1240 for the BMW 1M) under-estimated I_zz by "
                             "~2x vs the manufacturer/Wikipedia 2300-2500 "
                             "range; the default now reads AC's "
                             "[BASIC].INERTIA box dims via "
                             "m*(w^2+l^2)/12 (~3050 for the BMW 1M). Use this "
                             "flag to force a specific value (e.g. 2400 to "
                             "mid-range the manufacturer band). Per-driver "
                             "override -- NOT baked into the ini.")
    args = parser.parse_args()

    # v3 slip-model dispatch (spec §23.4.1). Phase 3: real implementation
    # wired in — ghost driver + RK4 ODE + per-wheel Pacejka.
    if args.model == "slip":
        _run_slip_model(args, parser)
        return

    if args.single_lap and args.laps is not None:
        parser.error("--single-lap is an alias for --laps 1; do not pass both.")
    if args.single_lap:
        n_laps = 1
    elif args.laps is not None:
        n_laps = int(args.laps)
        if not 1 <= n_laps <= 50:
            parser.error("--laps must be in [1, 50].")
    else:
        n_laps = 2  # v1.1 default (back-compat).

    data_dir = find_car_data(args.car)
    car = Car(
        data_dir,
        boost_steady_override=args.boost_steady,
        cd_override=args.cd_override,
        brake_torque_mult=args.brake_torque_mult,
        inertia_zz_override=args.inertia_zz,
    )
    driver = Driver.load(args.driver)
    calibration = driver.get_tyre_calibration()

    print(f"\nLoaded: {car}")
    print(
        f"Driver: {driver.name} "
        f"(skill={driver.skill_pct:.2f}, sigma={driver.consistency_sigma:.2f}, "
        f"tau={driver.driver_tau_s:.2f}s, trail={driver.trail_brake_m:.0f}m, "
        f"ramp={driver.throttle_ramp_m:.0f}m)"
    )
    if calibration.measured:
        print(
            f"Tyre calibration: MEASURED (k_friction={calibration.k_friction:.3f}, "
            f"h={calibration.h:.1f}, C_thermal={calibration.C_thermal:.0f}, "
            f"k_wear={calibration.k_wear:.2e})"
        )
    else:
        print("Tyre calibration: defaults (measured=false; "
              "per-wheel state channels absent from fit telemetry)")

    # Compound resolution (spec §21.11). Two-phase: load any setup-JSON peek
    # for its `compound` field, then resolve via CLI > setup > car-default.
    setup_for_compound_peek = None
    if args.setup:
        from lap_estimator.setup import Setup as _Setup
        try:
            setup_for_compound_peek = _Setup.load(args.setup)
        except Exception as e:  # pragma: no cover -- bad path / json surfaces below
            parser.error(f"Could not load --setup {args.setup}: {e}")
    try:
        compound, compound_source = resolve_compound(
            car,
            cli_name=args.compound,
            setup=setup_for_compound_peek,
        )
    except ValueError as e:
        parser.error(str(e))
    print(f"Compound: {compound.name} (idx {compound.index}) | source: {compound_source}")

    # Solve mode dispatch.
    if args.solve_pressure_for_wear is not None:
        if args.at_lap is None:
            parser.error("--solve-pressure-for-wear requires --at-lap")
        _run_solver(args, car, data_dir, driver, calibration, compound)
        return

    # Resolve setup (only used in stint mode; legacy single-lap path is unaffected).
    setup = resolve_setup(car, args.setup, args.pressure, args.ambient_temp_c,
                          compound=compound)
    print(setup.fmt_line())

    if args.all_tracks:
        tracks = [(name, BUILTIN_TRACKS[name](), "builtin", name) for name in BUILTIN_TRACKS]
    else:
        track, kind, src = resolve_track(args.track)
        tracks = [(track.name, track, kind, src)]

    for _name, track, kind, src in tracks:
        _run_one_track(args, car, driver, calibration, setup, track, kind, src,
                       n_laps=n_laps, compound=compound)


def _run_one_track(args, car, driver, calibration, setup, track, kind, src,
                   *, n_laps, compound=None):
    label = "single-lap" if n_laps == 1 else (
        "two-lap" if n_laps == 2 and not calibration.measured else f"stint ({n_laps} laps)"
    )
    print(f"\nSimulating {track.name} ({label})...")

    # Engage stint mode for N>=3 OR calibration.measured (so per-wheel state evolves).
    # Also engage when the user supplied an explicit setup/compound/pressure -- those
    # only have an effect through the stint loop's per-segment state update. (Spec
    # §11.35: --compound + --pressure at --laps 1 must affect lap-1 grip envelope.)
    user_supplied_state_overrides = bool(
        getattr(args, "setup", None)
        or getattr(args, "pressure", None)
        or getattr(args, "compound", None)
        or getattr(args, "ambient_temp_c", None)
    )
    use_stint = (
        n_laps >= 3
        or calibration.measured
        or n_laps == 2
        or user_supplied_state_overrides
    )

    if use_stint:
        stint = simulate_stint(
            car, track, driver,
            n_laps=n_laps, setup=setup, calibration=calibration,
            compound=compound, ds=args.ds,
        )
        # For lap-time stdout + plot, use the LAST lap's SimResult as the headline.
        result = stint.per_lap_sim_results[-1]
        # n_laps==2 back-compat: result is the v1.1 two-lap SimResult.
        if n_laps == 1:
            print(f"  Lap 1: {_fmt(stint.lap_times_s[0])}")
        elif n_laps == 2 and not calibration.measured:
            # Legacy print path -- back-compat with v1.2.1.
            print_report(car, track, result, driver=driver)
        else:
            # Per-lap block (spec §11.33).
            print_per_lap_block(stint)
    else:
        # Single-lap legacy path.
        stint = None
        mc = None
        if driver.consistency_sigma > 0:
            mc = simulate_monte_carlo(car, track, driver, ds=args.ds, two_lap=False)
            result = mc.representative
        else:
            result = simulate(car, track, driver, ds=args.ds, two_lap=False)
        print_report(car, track, result, driver=driver, mc=mc)

    stem = build_output_stem(src if kind == "csv" else track.name,
                             driver.name,
                             is_csv_track=(kind == "csv"))
    trace_path = f"{stem}_sim_trace.csv"
    write_trace_csv(result, trace_path)
    print(f"  Wrote trace: {trace_path}")

    if stint is not None and n_laps != 1 and not (n_laps == 2 and not calibration.measured):
        summary_path = f"{stem}_stint_summary.csv"
        write_stint_summary_csv(stint, summary_path)
        print(f"  Wrote stint summary: {summary_path}")

    if not args.no_plot:
        plot_path = f"{stem}_sim_vs_ai.png"
        lap_label = result.lap2_time_str if result.two_lap else result.lap_time_str
        ok = write_comparison_plot(
            result, track.name, driver.name, plot_path,
            lap_time_label=lap_label,
        )
        if ok:
            print(f"  Wrote plot:  {plot_path}")

    if not args.no_telemetry:
        tel_path = f"{stem}_sim_telemetry.csv"
        if stint is not None:
            write_synthetic_log(
                stint, car, driver, track.total_length_m, tel_path,
                telemetry_dt_ms=args.telemetry_dt_ms,
            )
        else:
            write_synthetic_log(
                result, car, driver, track.total_length_m, tel_path,
                telemetry_dt_ms=args.telemetry_dt_ms,
            )
        print(f"  Wrote telemetry: {tel_path}")

    if args.validate_against:
        if kind != "csv":
            print("ERROR: --validate-against requires a CSV-backed track.",
                  file=sys.stderr)
            sys.exit(2)
        target_lap = 2 if result.two_lap else 1
        if not result.two_lap:
            print("  warn: single-lap result; comparing real (flying) lap against "
                  "sim lap 1 (standing).")
        vr = validate_lap(
            car, track, result, args.validate_against,
            bin_m=args.bin_m, per_corner=args.per_corner, target_lap=target_lap,
        )
        _print_validation(vr, src, args.driver)
        bins_path = f"{stem}_validation_bins.csv"
        write_bins_csv(vr, bins_path)
        print(f"  Wrote bins: {bins_path}")
        if not args.no_plot:
            _plot_validation_overlay(
                car, track, result, args.validate_against,
                f"{stem}_validation_overlay.png", driver.name,
                target_lap=target_lap,
            )


def _run_slip_model(args, parser):
    """v3 slip-model entry point (spec §23, Phase 3).

    Loads the same car/track/driver/compound that the point-mass path uses,
    dispatches to ``dynamics.simulate_slip``, and writes output artifacts
    with the ``_slip`` suffix (spec §23.4.1).
    """
    from lap_estimator.dynamics import simulate_slip

    data_dir = find_car_data(args.car)
    car = Car(
        data_dir,
        boost_steady_override=args.boost_steady,
        cd_override=args.cd_override,
        brake_torque_mult=args.brake_torque_mult,
        inertia_zz_override=args.inertia_zz,
    )
    driver = Driver.load(args.driver)
    track, kind, src = resolve_track(args.track)
    if kind != "csv":
        print("ERROR: --model slip requires a CSV-backed track.", file=sys.stderr)
        sys.exit(2)

    try:
        compound, compound_source = resolve_compound(car, cli_name=args.compound)
    except ValueError as e:
        parser.error(str(e))

    if args.single_lap and args.laps is not None:
        parser.error("--single-lap is an alias for --laps 1; do not pass both.")
    if args.single_lap:
        n_laps = 1
    elif args.laps is not None:
        n_laps = int(args.laps)
    else:
        n_laps = 1

    print(f"\nLoaded: {car}")
    print(f"Driver: {driver.name} (skill={driver.skill_pct:.2f})")
    print(f"Compound: {compound.name} (idx {compound.index}) | source: {compound_source}")
    print(f"Chassis I_zz: {car.inertia_zz:.1f} kg.m^2 "
          f"(source: {car.inertia_zz_source})")
    print(f"\nSimulating {track.name} (slip model, {n_laps} lap{'s' if n_laps != 1 else ''}) ...")

    # Phase 5.0.2 chicane-safety: build config from CLI overrides + driver JSON.
    from lap_estimator.dynamics.chicane_safety import ChicaneSafetyConfig
    chicane_config = ChicaneSafetyConfig.resolve(
        driver,
        cli_radius_thresh_m=args.chicane_radius_thresh,
        cli_safety_mult=args.chicane_safety_mult,
    )
    # HMPC: resolve --hmpc-outer-ds-m into n_stages if provided.
    hmpc_outer_horizon_m = args.hmpc_outer_horizon_m
    hmpc_outer_n_stages = args.hmpc_outer_n_stages
    if args.hmpc_outer_ds_m is not None:
        from lap_estimator.dynamics.hmpc_outer import DEFAULT_HORIZON_M as _DEF_OUT_H
        eff_horizon = (
            float(hmpc_outer_horizon_m)
            if hmpc_outer_horizon_m is not None else float(_DEF_OUT_H)
        )
        hmpc_outer_n_stages = max(1, int(round(eff_horizon / float(args.hmpc_outer_ds_m))))
    hmpc_debug_path = ".tmp/hmpc_diag.csv" if bool(args.hmpc_debug_trace) else None

    try:
        result = simulate_slip(
            car, track, driver,
            compound=compound, n_laps=n_laps,
            plan_source=args.plan_source,
            controller=args.controller,
            dp_safety_margin=args.dp_safety_margin,
            chicane_config=chicane_config,
            mpc_tier1_disable=bool(args.mpc_tier1_disable),
            mpc_tier1_max_consecutive=args.mpc_tier1_max_consecutive,
            mpc_force_static_fz=bool(args.static_fz),
            mpc_cimpcc_weight=args.cimpcc_weight,
            mpc_cimpcc_safety=args.cimpcc_safety,
            mpc_emit_source=args.mpc_emit_source,
            mpcc_horizon_m=args.mpcc_horizon_m,
            mpcc_n_stages=args.mpcc_n_stages,
            mpcc_tick_hz=args.mpcc_tick_hz,
            mpcc_w_contour=args.mpcc_w_contour,
            mpcc_w_lag=args.mpcc_w_lag,
            mpcc_w_progress=args.mpcc_w_progress,
            mpcc_w_du=args.mpcc_w_du,
            mpcc_force_static_fz=bool(args.mpcc_force_static_fz),
            hmpc_outer_horizon_m=hmpc_outer_horizon_m,
            hmpc_outer_n_stages=hmpc_outer_n_stages,
            hmpc_outer_rate_hz=args.hmpc_outer_rate_hz,
            hmpc_outer_mu_circle=args.hmpc_outer_mu_circle,
            hmpc_outer_w_progress=args.hmpc_outer_w_progress,
            hmpc_outer_w_n=args.hmpc_outer_w_n,
            hmpc_outer_w_du=args.hmpc_outer_w_du,
            hmpc_inner_horizon_m=args.hmpc_inner_horizon_m,
            hmpc_inner_n_stages=args.hmpc_inner_n_stages,
            hmpc_inner_tick_hz=args.hmpc_inner_tick_hz,
            hmpc_pi_kp_n=args.hmpc_pi_kp_n,
            hmpc_pi_ki_n=args.hmpc_pi_ki_n,
            hmpc_pi_kp_vx=args.hmpc_pi_kp_vx,
            hmpc_pi_ki_vx=args.hmpc_pi_ki_vx,
            hmpc_pi_bound_steer=args.hmpc_pi_trim_bound_steer_pct,
            hmpc_pi_bound_pedal=args.hmpc_pi_trim_bound_pedal_pct,
            hmpc_outer_disable=bool(args.hmpc_outer_disable),
            hmpc_force_static_fz=bool(args.hmpc_force_static_fz),
            hmpc_emit_source=args.hmpc_emit_source,
            hmpc_inner_solver=args.hmpc_inner_solver,
            hmpc_debug_trace_path=hmpc_debug_path,
            tomas_csv_path=args.tomas_csv,
            line_source=args.line_source,
        )
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    # Headline output.
    if result.fallback_front_to_rear:
        print("  note: front Pacejka coefficients rail-clamped -> using rear "
              "coefficients on all four wheels (Phase 3 fallback).")
    for i, lt in enumerate(result.lap_times_s, start=1):
        print(f"  Lap {i}: {_fmt(lt)}")
    if not result.finished:
        print(f"  WARN: simulation did not complete cleanly: {result.abort_reason}")
    print(f"  Wallclock: {result.wallclock_s:.2f}s")
    # v3 Phase 4: slip-utilisation telemetry (spec §11.55).
    if result.slip_target_rad > 0:
        import math as _math
        slip_deg = _math.degrees(result.slip_target_rad)
        verdict = "within tyre peak" if result.util_p85 <= 1.0 else (
            f"{(result.util_p85 - 1) * 100:.0f}% over peak"
        )
        print(f"  util_p85 = {result.util_p85:.3f} (slip_target = {slip_deg:.2f} deg, {verdict})")
    if result.mc_n_runs > 1:
        print(f"  Monte Carlo: {result.mc_n_runs} runs, sigma = {result.mc_sigma_s:.3f}s")
    # Phase 5.0 (v3.2 MPC) diagnostics — only present when --controller mpc.
    if result.mpc_solve_times_s:
        import numpy as _np
        sts = _np.asarray(result.mpc_solve_times_s)
        mean_ms = float(_np.mean(sts) * 1000.0)
        p95_ms = float(_np.quantile(sts, 0.95) * 1000.0)
        max_ms = float(_np.max(sts) * 1000.0)
        print(
            f"  MPC solve: n={len(sts)} ticks; "
            f"mean={mean_ms:.2f} ms, p95={p95_ms:.2f} ms, max={max_ms:.2f} ms; "
            f"ghost-fallbacks={result.mpc_ghost_steps}"
        )
        # Phase 5.0.3 tier-distribution summary (spec §23.2-5.0.3.7).
        tc = result.mpc_tier_counts or {}
        total_steps = sum(int(v) for v in tc.values())
        if total_steps > 0:
            tier_parts = []
            for tier_id, tier_name in ((0, "0"), (1, "1"), (2, "2")):
                cnt = int(tc.get(tier_id, 0))
                if cnt > 0:
                    pct = 100.0 * cnt / total_steps
                    tier_parts.append(f"{tier_name}={cnt} ({pct:.1f}%)")
            ep1 = result.mpc_tier1_episodes
            mx1 = result.mpc_tier1_max_consecutive_steps
            ep2 = result.mpc_tier2_episodes
            extra = []
            if ep1 > 0 or mx1 > 0:
                extra.append(f"tier1 episodes={ep1}, max={mx1} steps")
            if ep2 > 0:
                extra.append(f"tier2 episodes={ep2}")
            extra_str = f"  [{', '.join(extra)}]" if extra else ""
            print(f"  MPC tiers: {' | '.join(tier_parts)}{extra_str}")
        viol_p95 = result.mpc_post_solve_ellipse_violation_p95
        if viol_p95 > 0:
            print(
                f"  MPC post-solve ellipse violation p95 = {viol_p95:.3f}"
                f" (> 0.15 => detection 4 firing often)"
            )
    # v3.3 MPCC diagnostics (spec §23.3.6.7). The MPCC reuses the
    # ``mpc_*`` SlipSimResult slots for solve-time / tier-share — these
    # already print above when present — and adds three MPCC-specific
    # terminal metrics (contour_p95, lag_p95, progress_mean).
    if args.controller == "mpcc" and result.mpc_solve_times_s:
        print(
            f"  MPCC diagnostics: contour_p95={result.mpcc_contour_p95:.2f} m, "
            f"lag_p95={result.mpcc_lag_p95:.2f} m, "
            f"V_theta_mean={result.mpcc_progress_mean:.2f} m/s"
        )
    # v3.4 HMPC diagnostics (spec §23.4.6.5). Outer + inner layer
    # solve-time split, staleness, PI-trim p95, outer-vs-inner v_ref
    # disagreement.
    if args.controller == "hmpc":
        import numpy as _np
        if result.hmpc_outer_solve_times_s:
            outs = _np.asarray(result.hmpc_outer_solve_times_s)
            o_mean = float(_np.mean(outs) * 1000.0)
            o_p99 = float(_np.quantile(outs, 0.99) * 1000.0)
            print(
                f"  HMPC outer: n={result.hmpc_outer_solve_count}, "
                f"mean={o_mean:.2f} ms, p99={o_p99:.2f} ms, "
                f"infeas={result.hmpc_outer_infeas_count}"
            )
        if result.hmpc_inner_solve_times_s:
            ins = _np.asarray(result.hmpc_inner_solve_times_s)
            i_mean = float(_np.mean(ins) * 1000.0)
            i_p95 = float(_np.quantile(ins, 0.95) * 1000.0)
            print(
                f"  HMPC inner: n={result.hmpc_inner_solve_count}, "
                f"mean={i_mean:.2f} ms, p95={i_p95:.2f} ms"
            )
        print(
            f"  HMPC PI trim p95: steer={result.hmpc_pi_trim_steer_p95:.4f} rad, "
            f"thr={result.hmpc_pi_trim_throttle_p95:.3f}, "
            f"brk={result.hmpc_pi_trim_brake_p95:.3f}"
        )
        print(
            f"  HMPC outer staleness p95={result.hmpc_outer_staleness_ticks_p95:.0f} ticks, "
            f"outer-vs-DP v_ref p95={result.hmpc_outer_vs_inner_vref_p95:.2f} m/s"
        )

    stem = build_output_stem(src, driver.name, is_csv_track=True)
    trace_path = f"{stem}_sim_trace_slip.csv"
    from lap_estimator.report import write_trace_csv as _write_trace
    _write_trace(result, trace_path)
    print(f"  Wrote trace: {trace_path}")

    if not args.no_plot:
        from lap_estimator.report import write_comparison_plot as _plot
        plot_path = f"{stem}_sim_vs_ai_slip.png"
        ok = _plot(
            result, track.name, driver.name, plot_path,
            lap_time_label=result.lap_time_str,
        )
        if ok:
            print(f"  Wrote plot:  {plot_path}")


def _run_solver(args, car, data_dir, driver, calibration, compound=None):
    """Inverse-PSI solver entry point (spec §21.5)."""
    track, kind, _src = resolve_track(args.track)
    target_wear = float(args.solve_pressure_for_wear)
    if not 0.0 <= target_wear <= 1.0:
        print("ERROR: --solve-pressure-for-wear must be in [0.0, 1.0].", file=sys.stderr)
        sys.exit(2)
    ambient = args.ambient_temp_c if args.ambient_temp_c is not None else 25.0
    print(
        f"\nSolver: target {target_wear * 100:.1f}% wear at lap {args.at_lap} "
        f"on {track.name} (wheel={args.target_wheel}, "
        f"{'uniform' if args.uniform_pressure else 'per-wheel'}) ..."
    )
    try:
        sr = solve_pressure_for_wear(
            car, track, driver,
            target_wear=target_wear,
            target_lap=int(args.at_lap),
            target_wheel=args.target_wheel,
            uniform=bool(args.uniform_pressure),
            calibration=calibration,
            ambient_temp_C=ambient,
            ds=args.ds,
            compound=compound,
        )
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)

    print(f"\nRecommended setup for {target_wear * 100:.1f}% wear at lap "
          f"{args.at_lap} on {track.name}:")
    for w in ("FL", "FR", "RL", "RR"):
        marker = "  <-- target wheel" if w == sr.target_wheel_resolved else ""
        print(f"  {w} = {sr.recommended_psi[w]:.1f} psi  (cold){marker}")
    print()
    print("Verification - predicted state under recommended setup:")
    print(f"  {'Lap':>3} {'Time':>9}  {'Wear FL/FR/RL/RR (%)':30}  "
          f"{'Temp avg':>9}  {'Pressure avg':>12}")
    print(f"  {'-' * 3} {'-' * 9}  {'-' * 30}  {'-' * 9}  {'-' * 12}")
    for k in range(sr.verification_stint.n_laps):
        end = sr.verification_stint.tyre_state_history[k + 1]
        lap_time = sr.verification_stint.lap_times_s[k]
        wears = f"{end.wear_pct['FL']:.0f}/{end.wear_pct['FR']:.0f}/{end.wear_pct['RL']:.0f}/{end.wear_pct['RR']:.0f}"
        print(
            f"  {k + 1:>3} {_fmt(lap_time):>9}  {wears:>30}  "
            f"{end.avg_temp_C():>6.1f} °C  {end.avg_pressure_psi():>8.1f} psi"
        )
    print()
    obs_pct = sr.observed_wear * 100.0
    target_pct = sr.target_wear * 100.0
    verdict = "target hit" if sr.converged else "target NOT hit (within tolerance)"
    print(f"Verdict: {verdict} "
          f"({sr.target_wheel_resolved}={obs_pct:.1f}% vs target {target_pct:.1f}%, "
          f"delta {obs_pct - target_pct:+.1f}%).")


def _print_validation(vr, track_path, driver_path):
    print("\n--- VALIDATION ---")
    print(f"Track:     {track_path}")
    print(f"Driver:    {driver_path}")
    print(f"Real lap:  {_fmt(vr.real_lap_time_s)}")
    label = f"Sim lap {vr.target_lap}" + (
        " (flying)" if vr.target_lap == 2 else " (standing)"
    )
    print(f"{label}: {_fmt(vr.sim_lap_time_s)}  (predicted)")
    print(f"Delta:     {_signed(vr.delta_s)} s  ({_signed_pct(vr.delta_pct)})")
    print(f"Verdict:   {vr.verdict}")


def _plot_validation_overlay(car, track, sim_result, real_telem_path, output_path,
                             driver_name, *, target_lap=2):
    from lap_estimator.telemetry import merge_with_track, read_ac_log
    import numpy as np
    telem = read_ac_log(real_telem_path)
    merged = merge_with_track(telem, track)
    if sim_result.lap_id is not None and sim_result.two_lap:
        mask = sim_result.lap_id == target_lap
        d = sim_result.distances[mask]
        sim_kmh = sim_result.speeds[mask] * 3.6
    else:
        d = sim_result.distances
        sim_kmh = sim_result.speeds * 3.6
    real_kmh = np.interp(d, merged["distance_m"], merged["speedKmh"])
    series = {"sim": sim_kmh, "real": real_kmh}
    plot_speed_overlay(
        d, series,
        title=f"{track.name} - {driver_name} (validation, sim lap {target_lap})",
        output_path=output_path,
    )
    print(f"  Wrote overlay: {output_path}")


def _fmt(seconds):
    m = int(seconds // 60)
    s = seconds - m * 60
    return f"{m}:{s:06.3f}"


def _signed(x):
    return f"{x:+.3f}"


def _signed_pct(x):
    return f"{x:+.2f}%"


if __name__ == "__main__":
    main()
