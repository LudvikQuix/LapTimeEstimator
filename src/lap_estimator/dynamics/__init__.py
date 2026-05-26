"""Slip-based dynamics model (v3 parallel track).

Public surface for the v3 model. The v2 point-mass model lives in
``src/lap_estimator/simulator.py`` and is unaffected by anything in here.

Imports stay one-directional (per spec §23.3):
``slip_simulator`` -> ``solver`` -> ``vehicle`` -> ``pacejka``. The driver
controller is consumed by ``solver``; ``tyre_state`` is imported by
``slip_simulator`` only.

Phase 1 status: package skeleton only. All public entry points raise
``NotImplementedError``.
"""

from .slip_simulator import SlipSimResult, simulate_slip, simulate_stint_slip

__all__ = ["SlipSimResult", "simulate_slip", "simulate_stint_slip"]
