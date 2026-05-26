"""Static chassis geometry the ODE needs but ``Car`` doesn't expose.

Read once from the car ini files; held constant for the lap. Kept in a
sibling module so :mod:`vehicle` stays under the 500-line soft cap.
"""

from __future__ import annotations

import os
from configparser import ConfigParser
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..car import Car


@dataclass(frozen=True)
class CarDynamics:
    """Extra chassis geometry needed by the ODE.

    Notes
    -----
    - ``track_f`` / ``track_r`` come from ``suspensions.ini``
      ``[FRONT].TRACK`` and ``[REAR].TRACK``.
    - ``h_cg`` is hand-defaulted to 0.45 m. AC's ``BASEY`` field has a
      car-specific sign convention; computed naïvely it gives unphysical
      values for the BMW 1M (0.17 m). Phase 3 keeps a single safe scalar.
    - ``I_zz`` is read off ``car.inertia_zz``, which resolves from (in
      priority order) ``--inertia-zz`` CLI override, AC ``[BASIC].INERTIA``
      box dims via ``m * (w^2 + l^2) / 12``, or a ``2.0 * plate``
      backstop. v3 lateral yaw-inertia fix (2026-05-23) -- the prior
      direct plate-formula here was ~2x too low.
    - ``I_wheel`` is the rotational inertia of one wheel; hand-default
      1.0 kg·m² per the Phase 3 brief.
    """
    wheelbase: float
    cg_front: float  # fraction of mass on front axle (0..1)
    track_f: float
    track_r: float
    h_cg: float
    I_zz: float
    I_wheel: float = 1.0


def load_car_dynamics(car: "Car") -> CarDynamics:
    """Parse extra chassis geometry from the car's ini files.

    Falls back to sane defaults if a file or key is missing — never
    raises.
    """
    track_f = 1.55
    track_r = 1.55
    # Phase 3: hand-default CG height of 0.45 m (typical road car). The AC
    # `BASEY` field is documented as "distance from centre of wheel" but
    # the sign convention is car-specific; pulling 0.17 m from BASEY on the
    # BMW 1M yields unphysical weight transfer. Phase 3 keeps a safe
    # scalar; v3.1 may parse a per-car CG-height override.
    h_cg = 0.45
    susp_path = os.path.join(car.data_dir, "suspensions.ini")
    if os.path.isfile(susp_path):
        ini = ConfigParser(inline_comment_prefixes=(";",), strict=False)
        ini.read(susp_path)
        if ini.has_option("FRONT", "TRACK"):
            track_f = float(ini["FRONT"]["TRACK"])
        if ini.has_option("REAR", "TRACK"):
            track_r = float(ini["REAR"]["TRACK"])

    # v3 lateral yaw-inertia fix (2026-05-23). Was:
    #   I_zz = car.total_mass * (car.wheelbase ** 2 + track_f ** 2) / 12.0
    # That plate-formula estimate is ~1240 kg.m^2 for the BMW 1M with the
    # numbers parsed from suspensions.ini, which is ~2x lower than the
    # manufacturer/Wikipedia reference of 2300-2500. The lateral empirical
    # diagnostic ``M_z_obs / M_z_v3`` slope (0.131) confirms the model is
    # under-resisting yaw torque from a second angle. ``Car`` now exposes
    # ``inertia_zz`` directly, sourced from (in priority order) an explicit
    # CLI override, AC's box-formula derived from ``car.ini`` ``[BASIC].INERTIA``
    # ``(w, h, l)`` dims, or a 2x scaled plate-formula backstop. See
    # ``docs/architecture-v3-lateral-yaw-inertia-fix.md`` for the audit.
    I_zz = float(getattr(car, 'inertia_zz', None) or car.total_mass * (
        car.wheelbase ** 2 + track_f ** 2) / 12.0)
    # v3 longitudinal-physics fix (2026-05-23): use per-axle ANGULAR_INERTIA
    # from `tyres.ini` (parsed by `car.py`) instead of the prior 1.0 kg.m^2
    # hardcoded fallback. CarDynamics still exposes a single scalar — average
    # the two axles so the per-wheel ODE (`d_omega = tau / I_w`) gets a
    # plant-faithful value. The engine-side reflected inertia in
    # `vehicle.compute_derivatives` reads `wheel_inertia_f`/`_r` directly
    # off the car for the full 4-wheel sum.
    I_wheel_avg = 0.5 * (
        float(getattr(car, 'wheel_inertia_f', 1.0))
        + float(getattr(car, 'wheel_inertia_r', 1.0))
    )
    return CarDynamics(
        wheelbase=float(car.wheelbase),
        cg_front=float(car.cg_front),
        track_f=float(track_f),
        track_r=float(track_r),
        h_cg=float(h_cg),
        I_zz=float(I_zz),
        I_wheel=float(I_wheel_avg),
    )
