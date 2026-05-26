"""Numerical helpers for ``pacejka_fit`` (spec §23.7).

Kept in a sibling module so ``pacejka_fit.py`` stays close to the §23.7
algorithm narrative. Two things live here:

1. A hand-rolled Levenberg-Marquardt with parameter bounds (logistic
   transform). We deliberately don't depend on SciPy — the rest of the
   library is numpy-only, and ``curve_fit`` would be the only SciPy use.
2. A low-pass IIR filter and a central-differences helper, both used by
   the chassis-dynamics inversion in Stage B.
"""

from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------------------
# Low-pass / differentiation
# ---------------------------------------------------------------------------


def low_pass_iir(x: np.ndarray, dt_s: float, cutoff_hz: float) -> np.ndarray:
    """One-pole IIR low-pass. ``y[k] = a*y[k-1] + (1-a)*x[k]``.

    The cutoff is approximate (sets the -3 dB frequency); good enough for
    suppressing yaw-rate noise before central-differencing. NaNs are
    forward-filled before filtering, then masked back out.
    """
    if cutoff_hz <= 0 or len(x) == 0:
        return np.asarray(x, dtype=float).copy()
    x = np.asarray(x, dtype=float)
    nan_mask = ~np.isfinite(x)
    if nan_mask.any():
        # Forward-fill NaNs for the filter; restore as NaN at the end.
        good = np.where(~nan_mask)[0]
        if len(good) == 0:
            return x.copy()
        x = x.copy()
        last = x[good[0]]
        for i in range(len(x)):
            if not np.isfinite(x[i]):
                x[i] = last
            else:
                last = x[i]
    rc = 1.0 / (2.0 * np.pi * cutoff_hz)
    a = rc / (rc + dt_s)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = a * y[i - 1] + (1.0 - a) * x[i]
    if nan_mask.any():
        y[nan_mask] = np.nan
    return y


def central_diff(y: np.ndarray, dt_s: np.ndarray | float) -> np.ndarray:
    """Central-difference derivative ``dy/dt``. Endpoints use forward/backward.

    ``dt_s`` may be scalar (uniform sampling) or per-step (variable rate);
    if variable, we use the local timestep around each interior point.
    """
    y = np.asarray(y, dtype=float)
    n = len(y)
    if n < 2:
        return np.zeros_like(y)
    out = np.empty_like(y)
    if np.isscalar(dt_s):
        dt = float(dt_s)
        out[1:-1] = (y[2:] - y[:-2]) / (2.0 * dt)
        out[0] = (y[1] - y[0]) / dt
        out[-1] = (y[-1] - y[-2]) / dt
    else:
        dts = np.asarray(dt_s, dtype=float)
        # dts[i] = t[i+1] - t[i]; for central diff on index k use
        # (y[k+1] - y[k-1]) / (t[k+1] - t[k-1]).
        out[1:-1] = (y[2:] - y[:-2]) / np.maximum(dts[1:] + dts[:-1], 1e-9)[: n - 2]
        out[0] = (y[1] - y[0]) / max(dts[0], 1e-9)
        out[-1] = (y[-1] - y[-2]) / max(dts[-2], 1e-9) if n >= 2 else 0.0
    return out


# ---------------------------------------------------------------------------
# Bounded nonlinear least-squares (Levenberg-Marquardt + logistic clamp)
# ---------------------------------------------------------------------------


def _logistic_transform(p_free: np.ndarray,
                        lower: np.ndarray,
                        upper: np.ndarray) -> np.ndarray:
    """Map unconstrained ``p_free`` to bounded ``p`` via the logistic.

    ``p = lower + (upper - lower) * sigmoid(p_free)``. Smooth, differentiable,
    and avoids hitting hard limits during finite-difference Jacobians.
    """
    # Clip the free-space input to avoid exp overflow.
    p_clipped = np.clip(p_free, -50.0, 50.0)
    s = 1.0 / (1.0 + np.exp(-p_clipped))
    return lower + (upper - lower) * s


def _logistic_inverse(p: np.ndarray,
                      lower: np.ndarray,
                      upper: np.ndarray) -> np.ndarray:
    """Inverse of :func:`_logistic_transform`, clipped to avoid logit blow-up."""
    eps = 1e-4
    s = (p - lower) / np.maximum(upper - lower, 1e-12)
    s = np.clip(s, eps, 1.0 - eps)
    return np.log(s / (1.0 - s))


def lm_least_squares(residual_fn,
                     p0: np.ndarray,
                     *,
                     lower: np.ndarray,
                     upper: np.ndarray,
                     max_iter: int = 80,
                     tol: float = 1e-8,
                     fd_step: float = 1e-5) -> dict:
    """Levenberg-Marquardt with logistic bound-transform. Returns dict.

    Parameters
    ----------
    residual_fn : callable
        ``residual_fn(p) -> np.ndarray`` of residuals (predicted - observed).
    p0 : np.ndarray
        Initial parameter guess in *bounded* space.
    lower, upper : np.ndarray
        Per-parameter lower/upper bounds (same shape as ``p0``).
    max_iter : int
        Maximum LM iterations (default 80).
    tol : float
        Stop when ``|cost - cost_prev| < tol * (1 + cost)``.
    fd_step : float
        Relative finite-difference step for the Jacobian.

    Returns
    -------
    result : dict
        Keys: ``"x"`` (best bounded params), ``"cost"`` (final 0.5*||r||^2),
        ``"residual"`` (final residual vector), ``"n_iter"``, ``"success"``.
    """
    p0 = np.asarray(p0, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    # Clip p0 inside bounds (with margin) before inverting.
    p0_clipped = np.clip(p0, lower + 1e-6, upper - 1e-6)
    q = _logistic_inverse(p0_clipped, lower, upper)

    def _r(q_vec):
        p = _logistic_transform(q_vec, lower, upper)
        return residual_fn(p)

    r = _r(q)
    cost = 0.5 * float(np.dot(r, r))
    lam = 1e-3  # LM damping
    success = False
    last_cost = cost
    n_iter = 0
    for n_iter in range(1, max_iter + 1):
        # Finite-difference Jacobian in q-space.
        J = np.empty((len(r), len(q)), dtype=float)
        for j in range(len(q)):
            step = fd_step * (abs(q[j]) + 1.0)
            q_p = q.copy()
            q_p[j] += step
            r_p = _r(q_p)
            J[:, j] = (r_p - r) / step
        # Damped normal equations: (J^T J + lam diag(J^T J)) dq = -J^T r
        JTJ = J.T @ J
        diag = np.diag(JTJ).copy()
        diag[diag < 1e-12] = 1e-12
        try:
            dq = np.linalg.solve(JTJ + lam * np.diag(diag), -J.T @ r)
        except np.linalg.LinAlgError:
            lam *= 10.0
            continue
        q_new = q + dq
        r_new = _r(q_new)
        cost_new = 0.5 * float(np.dot(r_new, r_new))
        if cost_new < cost:
            # Accept step, reduce damping.
            q = q_new
            r = r_new
            last_cost = cost
            cost = cost_new
            lam = max(lam / 3.0, 1e-9)
            if abs(last_cost - cost) < tol * (1.0 + cost):
                success = True
                break
        else:
            # Reject, increase damping.
            lam *= 5.0
            if lam > 1e9:
                break
    p_final = _logistic_transform(q, lower, upper)
    return {
        "x": p_final,
        "cost": cost,
        "residual": r,
        "n_iter": n_iter,
        "success": success,
    }
