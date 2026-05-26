"""Curvilinear LTV bicycle plant + linearisation for v3.3 MPCC (spec §23.3.6.2).

This is the curvilinear (s, n, ψ_e) reformulation of ``mpc_model.f_continuous``
with two extra states for the MPCC formulation:

- ``θ`` — virtual progress variable; the optimiser increments it via its
  rate-control ``V_θ`` and the cost penalises ``(s − θ)²``.
- (no change to chassis dynamics — the chassis sees the same Pacejka /
  longitudinal / yaw equations as v3.2, just expressed against a different
  spatial basis.)

State vector (NX_C = 10):

    x = [s, n, ψ_e, v_x, v_y, ω, δ, throttle, brake, θ]

Control vector (NU_C = 4):

    u = [δ_dot, throttle_dot, brake_dot, V_θ]

Continuous-time dynamics (spec §23.3.5):

    ds/dt    = (v_x cos ψ_e − v_y sin ψ_e) / (1 − n · κ(θ))
    dn/dt    = v_x sin ψ_e + v_y cos ψ_e
    dψ_e/dt  = ω − κ(θ) · ds/dt
    dv_x/dt  = (Fx_front cos δ − Fy_front sin δ + Fx_rear − F_drag) / m + v_y ω
    dv_y/dt  = (Fx_front sin δ + Fy_front cos δ + Fy_rear) / m − v_x ω
    dω/dt    = (a_f · (Fx_front sin δ + Fy_front cos δ) − a_r · Fy_rear) / I_zz
    dδ/dt        = u0
    dthrottle/dt = u1
    dbrake/dt    = u2
    dθ/dt        = u3 = V_θ

The denominator ``(1 − n · κ)`` is floored at ``DENOM_FLOOR`` to keep the
Jacobian well-conditioned at the apex (spec §23.3.11 risk #1).

Tyre / longitudinal force expressions are byte-for-byte copied from
``mpc_model.f_continuous`` so the friction-ellipse hard constraint and the
controller's overall envelope view remain consistent across v3.2 and v3.3.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .mpc_model import (
    G,
    V_FLOOR,
    PlantConstants,
)
from .mpcc_reference import ReferencePath, kappa_at


# State vector layout (curvilinear MPCC).
NX_C = 10
NU_C = 4
IDX_S = 0
IDX_N = 1
IDX_E_PSI = 2
IDX_VX = 3
IDX_VY = 4
IDX_OMEGA = 5
IDX_DELTA = 6
IDX_THR = 7
IDX_BRK = 8
IDX_THETA = 9

IDX_DELTA_DOT = 0
IDX_THR_DOT = 1
IDX_BRK_DOT = 2
IDX_V_THETA = 3

# Denominator floor on (1 − n · κ); same as DENOM clamp in spec §23.3.11.
# With kappa_clamp = 0.2 rad/m and |n| ≤ ~6 m, the un-clamped denominator can
# touch zero. Floor at 0.3 keeps the curvilinear Jacobian well-conditioned
# (geometric factor ≥ 0.3 ⇒ effective radius ≥ 1.5 m at the apex).
DENOM_FLOOR = 0.3


@dataclass(frozen=True)
class StageLinearisationC:
    """Discrete-time linear model for one MPCC stage.

    ``x_{k+1} = A @ x_k + B @ u_k + c`` with ``x ∈ R^NX_C``, ``u ∈ R^NU_C``.
    """

    A: np.ndarray  # (NX_C, NX_C)
    B: np.ndarray  # (NX_C, NU_C)
    c: np.ndarray  # (NX_C,)
    x_lin: np.ndarray
    u_lin: np.ndarray
    ts: float
    kappa_ref: float
    v_ref: float


# ----------------------------------------------------------------------
# Chassis-frame axle forces (same as mpc_model, re-stated for clarity).
# ----------------------------------------------------------------------


def _axle_forces(x: np.ndarray, pc: PlantConstants) -> tuple[float, float, float, float, float, float]:
    """Return ``(Fx_f, Fy_f, Fx_r, Fy_r, cos_delta, sin_delta)`` at ``x``."""
    v_x = float(x[IDX_VX])
    v_y = float(x[IDX_VY])
    omega = float(x[IDX_OMEGA])
    delta = float(x[IDX_DELTA])
    throttle = float(x[IDX_THR])
    brake = float(x[IDX_BRK])
    vx_safe = max(abs(v_x), V_FLOOR)
    alpha_front = delta - math.atan2(v_y + pc.a_f * omega, vx_safe)
    alpha_rear = -math.atan2(v_y - pc.a_r * omega, vx_safe)
    Fy_front = pc.F_y_bias_front + pc.C_alpha_front * alpha_front
    Fy_rear = pc.F_y_bias_rear + pc.C_alpha_rear * alpha_rear
    Fy_front = float(np.clip(
        Fy_front,
        -pc.D_lat_front * pc.Fz_front,
        pc.D_lat_front * pc.Fz_front,
    ))
    Fy_rear = float(np.clip(
        Fy_rear,
        -pc.D_lat_rear * pc.Fz_rear,
        pc.D_lat_rear * pc.Fz_rear,
    ))
    Fx_rear = pc.k_throttle * throttle - pc.k_brake_rear * brake
    Fx_front = -pc.k_brake_front * brake
    return Fx_front, Fy_front, Fx_rear, Fy_rear, math.cos(delta), math.sin(delta)


def f_continuous_c(
    x: np.ndarray,
    u: np.ndarray,
    *,
    ref: ReferencePath,
    pc: PlantConstants,
) -> np.ndarray:
    """Continuous-time curvilinear dynamics ``dx/dt = f(x, u; ref)``.

    Reads ``κ(θ)`` from ``ref`` at every call so the linearisation tracks
    the reference path through the horizon.
    """
    s = float(x[IDX_S])
    n = float(x[IDX_N])
    e_psi = float(x[IDX_E_PSI])
    v_x = float(x[IDX_VX])
    v_y = float(x[IDX_VY])
    omega = float(x[IDX_OMEGA])
    theta = float(x[IDX_THETA])
    V_theta = float(u[IDX_V_THETA])

    kappa_t = kappa_at(theta, ref)
    cos_e = math.cos(e_psi)
    sin_e = math.sin(e_psi)
    denom = 1.0 - n * kappa_t
    if denom < DENOM_FLOOR:
        denom = DENOM_FLOOR
    s_dot = (v_x * cos_e - v_y * sin_e) / denom
    n_dot = v_x * sin_e + v_y * cos_e
    e_psi_dot = omega - kappa_t * s_dot

    Fx_f, Fy_f, Fx_r, Fy_r, cd, sd = _axle_forces(x, pc)
    F_drag = pc.drag_coeff * v_x * abs(v_x)
    vx_dot = (Fx_f * cd - Fy_f * sd + Fx_r - F_drag) / pc.mass + v_y * omega
    vy_dot = (Fx_f * sd + Fy_f * cd + Fy_r) / pc.mass - v_x * omega
    omega_dot = (
        pc.a_f * (Fx_f * sd + Fy_f * cd) - pc.a_r * Fy_r
    ) / max(pc.I_zz, 1e-3)

    # s, n, e_psi, v_x, v_y, omega, delta, throttle, brake, theta
    return np.array([
        s_dot,
        n_dot,
        e_psi_dot,
        vx_dot,
        vy_dot,
        omega_dot,
        float(u[IDX_DELTA_DOT]),
        float(u[IDX_THR_DOT]),
        float(u[IDX_BRK_DOT]),
        V_theta,
    ], dtype=float)


def linearise_stage_c(
    x_lin: np.ndarray,
    u_lin: np.ndarray,
    *,
    ref: ReferencePath,
    pc: PlantConstants,
    ds_stage: float,
    v_ref_stage: float | None = None,
) -> StageLinearisationC:
    """Build discrete-time ``(A, B, c)`` for one MPCC stage by central differences.

    Stage time is ``ts = ds_stage / max(V_θ, v_ref_stage, V_FLOOR)`` where
    ``V_θ`` is read from ``u_lin``; we fall back to ``v_ref_stage`` (an
    explicit caller hint) when the linearisation point's V_θ is small, so
    the discrete-time step never explodes near a stop.

    Spec §23.3.6.4: the MPCC grids in ``dθ`` (virtual progress), and the
    stage time follows from that.
    """
    V_theta = float(u_lin[IDX_V_THETA])
    if v_ref_stage is None:
        v_ref_stage = float(ref.v_ref[0]) if len(ref.v_ref) else V_FLOOR
    v_for_ts = max(V_theta, float(v_ref_stage), V_FLOOR)
    ts = float(ds_stage) / v_for_ts

    f0 = f_continuous_c(x_lin, u_lin, ref=ref, pc=pc)

    # Step sizes per state component. Chosen ~few-percent of typical operating
    # magnitude (matches mpc_model.linearise_stage).
    eps_x = np.array([
        1.0,    # s        (m; scale: 100+)
        1e-2,   # n        (m; scale: 0.1-3)
        1e-3,   # e_psi    (rad)
        1e-2,   # v_x      (m/s)
        1e-2,   # v_y      (m/s)
        1e-3,   # omega    (rad/s)
        1e-3,   # delta    (rad)
        1e-3,   # throttle
        1e-3,   # brake
        1.0,    # theta    (m)
    ])
    eps_u = np.array([1e-2, 1e-2, 1e-2, 1e-2])

    A_cont = np.zeros((NX_C, NX_C))
    for i in range(NX_C):
        dx = np.zeros(NX_C)
        dx[i] = eps_x[i]
        fp = f_continuous_c(x_lin + dx, u_lin, ref=ref, pc=pc)
        fm = f_continuous_c(x_lin - dx, u_lin, ref=ref, pc=pc)
        A_cont[:, i] = (fp - fm) / (2.0 * eps_x[i])

    B_cont = np.zeros((NX_C, NU_C))
    for j in range(NU_C):
        du = np.zeros(NU_C)
        du[j] = eps_u[j]
        fp = f_continuous_c(x_lin, u_lin + du, ref=ref, pc=pc)
        fm = f_continuous_c(x_lin, u_lin - du, ref=ref, pc=pc)
        B_cont[:, j] = (fp - fm) / (2.0 * eps_u[j])

    # Explicit-Euler discretisation. Same shape as mpc_model.linearise_stage.
    A = np.eye(NX_C) + ts * A_cont
    B = ts * B_cont
    c = ts * (f0 - A_cont @ x_lin - B_cont @ u_lin)

    kappa_t = kappa_at(float(x_lin[IDX_THETA]), ref)
    return StageLinearisationC(
        A=A, B=B, c=c,
        x_lin=x_lin.copy(), u_lin=u_lin.copy(),
        ts=ts, kappa_ref=float(kappa_t),
        v_ref=float(v_ref_stage),
    )


def integrate_reference_c(
    x0: np.ndarray,
    u_seq: np.ndarray,
    *,
    ref: ReferencePath,
    pc: PlantConstants,
    ds_stage: float,
    v_ref_seq: np.ndarray,
) -> np.ndarray:
    """Forward-roll the curvilinear plant over the horizon (explicit Euler).

    Stage gridding in ``dθ`` (spec §23.3.6.4). ``ds_stage`` is the
    fixed virtual-progress increment; the stage time per stage uses the
    larger of ``V_θ_k`` (from ``u_seq``) and ``v_ref_seq[k]``.

    Returns ``x_seq`` of shape ``(N+1, NX_C)`` with ``x_seq[0] = x0``.
    """
    n = len(u_seq)
    x_seq = np.zeros((n + 1, NX_C))
    x_seq[0] = x0
    for k in range(n):
        V_theta = float(u_seq[k, IDX_V_THETA])
        v_for_ts = max(V_theta, float(v_ref_seq[k]), V_FLOOR)
        ts = float(ds_stage) / v_for_ts
        f = f_continuous_c(x_seq[k], u_seq[k], ref=ref, pc=pc)
        x_seq[k + 1] = x_seq[k] + ts * f
    return x_seq


def compute_axle_force_from_state_c(
    x_vec: np.ndarray,
    pc: PlantConstants,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return ``((Fx_f, Fy_f), (Fx_r, Fy_r))`` (N) at a curvilinear state.

    Mirrors :func:`mpc_model.compute_axle_force_from_state` (which reads the
    v3.2 state-vector indices). Provided here so downstream diagnostics can
    interrogate planned forces without dispatching on plant flavour.
    """
    Fx_f, Fy_f, Fx_r, Fy_r, _cd, _sd = _axle_forces(x_vec, pc)
    return (float(Fx_f), float(Fy_f)), (float(Fx_r), float(Fy_r))


def axle_accelerations_from_trajectory_c(
    x_seq: np.ndarray,
    *,
    v_ref_seq: np.ndarray,
    u_seq: np.ndarray,
    ds_stage: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Recover ``(a_x_k, a_y_k)`` per stage from the rolled curvilinear trajectory.

    Mirrors :func:`mpc_model.axle_accelerations_from_trajectory`; uses
    ``V_θ`` (from ``u_seq``) and ``v_ref_seq`` for the stage-time denominator,
    consistent with :func:`integrate_reference_c`. Returns length-``N`` arrays.
    """
    x_arr = np.asarray(x_seq, dtype=float)
    n = x_arr.shape[0] - 1
    if n <= 0:
        return np.zeros(0), np.zeros(0)
    V_theta = np.asarray(u_seq[:n, IDX_V_THETA], dtype=float)
    v_ref = np.asarray(v_ref_seq[:n], dtype=float)
    v_for_ts = np.maximum.reduce([V_theta, v_ref, np.full(n, V_FLOOR)])
    ts = float(ds_stage) / np.maximum(v_for_ts, V_FLOOR)
    v_x_k = x_arr[:n, IDX_VX]
    v_x_next = x_arr[1:n + 1, IDX_VX]
    a_x = (v_x_next - v_x_k) / np.maximum(ts, 1e-3)
    omega_k = x_arr[:n, IDX_OMEGA]
    a_y = v_x_k * omega_k
    return a_x, a_y


__all__ = [
    "NX_C", "NU_C",
    "IDX_S", "IDX_N", "IDX_E_PSI",
    "IDX_VX", "IDX_VY", "IDX_OMEGA",
    "IDX_DELTA", "IDX_THR", "IDX_BRK", "IDX_THETA",
    "IDX_DELTA_DOT", "IDX_THR_DOT", "IDX_BRK_DOT", "IDX_V_THETA",
    "DENOM_FLOOR",
    "G", "V_FLOOR",
    "StageLinearisationC",
    "f_continuous_c",
    "linearise_stage_c",
    "integrate_reference_c",
    "compute_axle_force_from_state_c",
    "axle_accelerations_from_trajectory_c",
]
