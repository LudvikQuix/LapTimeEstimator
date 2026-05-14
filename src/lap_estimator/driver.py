"""Driver model: skill multiplier + consistency noise + v1.1 input-shape fields.

v1 fields (`skill_pct`, `consistency_sigma`) drive grip scaling and Monte-Carlo
noise. v1.1 adds sim-telemetry-only shape fields consumed by
`sim_telemetry.py` (not the grip-scaling path):
  - `trail_brake_m`   -- linear brake-taper distance on corner entry.
  - `throttle_ramp_m` -- linear throttle-ramp distance on corner exit.

`driver_tau_s` is retained on the dataclass for back-compat and as a measured
statistic (see `profile_dynamics.py`) but is **not consumed** by the v1.2.1
simulator -- the IIR low-pass it parameterised was removed per spec §14.3.

v1.2 adds the `profile.dynamic` block; when present, the measured values
override the top-level fields for the sim-consumed fields. The statistics-only
fields (`driver_tau_s`, `pedal_press_rate_per_s`,
`steering_aggression_deg_per_s`) are also loaded onto the dataclass but the
v1.2.1 simulator does not consume them.

Format: JSON only. No YAML fallback (v1.1 clean break -- see spec §6.2).
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

# Heuristic: seconds-of-jitter -> per-point grip sigma (fraction). v1 calibration.
SIGMA_SECONDS_TO_GRIP = 0.03

# v1.1 defaults (spec §6.2 / §7.2).
DEFAULT_DRIVER_TAU_S = 0.12
DEFAULT_TRAIL_BRAKE_M = 30.0
DEFAULT_THROTTLE_RAMP_M = 40.0


@dataclass
class Driver:
    name: str
    skill_pct: float
    consistency_sigma: float = 0.0
    # `driver_tau_s` is retained as a measured statistic but is NOT consumed by
    # the v1.2.1 simulator (the IIR low-pass was removed per spec §14.3).
    driver_tau_s: float = DEFAULT_DRIVER_TAU_S
    trail_brake_m: float = DEFAULT_TRAIL_BRAKE_M
    throttle_ramp_m: float = DEFAULT_THROTTLE_RAMP_M
    # v1.2 statistics-only fields. Not wired into the simulator yet.
    pedal_press_rate_per_s: float | None = None
    steering_aggression_deg_per_s: float | None = None
    # v2: tyre calibration block (spec §7.2). Defaults injected when absent.
    tyre_calibration: dict = field(default_factory=lambda: {
        "k_friction": 1.0, "h": 50.0, "C_thermal": 5000.0,
        "k_wear": 1.0e-7, "measured": False,
        "source": {
            "telemetry_csvs": [],
            "fit_rmse_temp_C": None,
            "fit_rmse_wear_pct": None,
            "fit_rmse_pressure_psi": None,
            "fitted_at": None,
        },
    })
    source: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: str) -> "Driver":
        with open(path) as f:
            raw = json.load(f)
        if not isinstance(raw, dict):
            raise ValueError(f"Driver JSON {path}: top-level must be an object")
        if "skill_pct" not in raw:
            raise ValueError(f"Driver JSON {path} missing required field 'skill_pct'")
        skill = float(raw["skill_pct"])
        if not (0.0 < skill <= 1.0):
            raise ValueError(
                f"Driver JSON {path}: skill_pct must be in (0, 1], got {skill}"
            )
        sigma = float(raw.get("consistency_sigma", 0.0))
        if sigma < 0:
            raise ValueError(
                f"Driver JSON {path}: consistency_sigma must be >= 0, got {sigma}"
            )

        profile = raw.get("profile") or {}
        dynamic = profile.get("dynamic") if isinstance(profile, dict) else None
        if not isinstance(dynamic, dict):
            dynamic = {}

        # v1.2 precedence: profile.dynamic.<field> > top-level <field> > default.
        tau = _resolve_field(
            path, "driver_tau_s", dynamic, raw, DEFAULT_DRIVER_TAU_S,
            min_value=0.0, fail_on_dynamic_max=1.0,
        )
        trail = _resolve_field(
            path, "trail_brake_m", dynamic, raw, DEFAULT_TRAIL_BRAKE_M,
            min_value=0.0,
        )
        ramp = _resolve_field(
            path, "throttle_ramp_m", dynamic, raw, DEFAULT_THROTTLE_RAMP_M,
            min_value=0.0,
        )
        # Statistics-only; warn-don't-fail per spec §7.2.
        press_rate = _resolve_optional(dynamic, "pedal_press_rate_per_s")
        steer_aggr = _resolve_optional(dynamic, "steering_aggression_deg_per_s")

        name = raw.get("name") or os.path.splitext(os.path.basename(path))[0]
        source = raw.get("source") or {}
        tyre_cal = _load_tyre_calibration(raw)
        return cls(
            name=str(name),
            skill_pct=skill,
            consistency_sigma=sigma,
            driver_tau_s=tau,
            trail_brake_m=trail,
            throttle_ramp_m=ramp,
            pedal_press_rate_per_s=press_rate,
            steering_aggression_deg_per_s=steer_aggr,
            tyre_calibration=tyre_cal,
            source=source if isinstance(source, dict) else {},
            raw=raw,
        )

    def get_tyre_calibration(self):
        """Return a `tyre_state.TyreCalibration` derived from this driver's block.

        Local import to avoid a circular dependency at module-import time.
        """
        from .tyre_state import TyreCalibration
        tc = self.tyre_calibration or {}
        return TyreCalibration(
            k_friction=float(tc.get("k_friction", 1.0)),
            h=float(tc.get("h", 50.0)),
            C_thermal=float(tc.get("C_thermal", 5000.0)),
            k_wear=float(tc.get("k_wear", 1.0e-7)),
            measured=bool(tc.get("measured", False)),
            source=tc.get("source") if isinstance(tc.get("source"), dict) else {},
        )

    @property
    def grip_sigma(self) -> float:
        """Per-point Gaussian sigma applied to grip during Monte-Carlo runs.

        Clipped to [0, 0.1] to avoid pathological grip excursions.
        """
        return float(max(0.0, min(0.1, self.consistency_sigma * SIGMA_SECONDS_TO_GRIP)))

    def wrap(self, car, *, rng=None, noise=False):
        """Return a thin grip-scaled facade over `car`.

        Scaling is uniform across lateral + longitudinal grip. When `noise=True`
        and `rng` is provided, each grip query is perturbed by N(0, grip_sigma)
        clamped to a sane band.
        """
        return _DriverScaledCar(car, self, rng=rng, noise=noise)


def _resolve_field(
    path: str,
    key: str,
    dynamic: dict,
    raw: dict,
    default: float,
    *,
    min_value: float,
    fail_on_dynamic_max: float | None = None,
) -> float:
    """Resolve a driver field with v1.2 precedence + validation.

    `profile.dynamic.<key>` wins; falls back to `raw[<key>]`; falls back to
    `default`. Top-level negative values fail fast (legacy v1 behaviour).
    `profile.dynamic.<key>` fail-fast bounds per spec §7.2.
    """
    if key in dynamic:
        value = float(dynamic[key])
        if value < 0.0:
            raise ValueError(
                f"Driver JSON {path}: profile.dynamic.{key} must be >= 0, got {value}"
            )
        if fail_on_dynamic_max is not None and not (0.0 <= value <= fail_on_dynamic_max):
            raise ValueError(
                f"Driver JSON {path}: profile.dynamic.{key} must be in "
                f"[0.0, {fail_on_dynamic_max}], got {value}"
            )
        return value
    value = float(raw.get(key, default))
    if value < min_value:
        raise ValueError(
            f"Driver JSON {path}: {key} must be >= {min_value}, got {value}"
        )
    return value


def _load_tyre_calibration(raw: dict) -> dict:
    """Build the v2 tyre_calibration block onto the Driver dataclass.

    Defaults (spec §7.2):
      k_friction=1.0, h=50.0, C_thermal=5000.0, k_wear=1.0e-7, measured=False.
    Absent block -> defaults with measured=False. Partial block -> per-field
    defaulting for missing keys.
    """
    block = raw.get("tyre_calibration") or {}
    if not isinstance(block, dict):
        block = {}
    defaults = {
        "k_friction": 1.0,
        "h": 50.0,
        "C_thermal": 5000.0,
        "k_wear": 1.0e-7,
        "measured": False,
        "source": {
            "telemetry_csvs": [],
            "fit_rmse_temp_C": None,
            "fit_rmse_wear_pct": None,
            "fit_rmse_pressure_psi": None,
            "fitted_at": None,
        },
    }
    out = dict(defaults)
    for k, v in block.items():
        out[k] = v
    # Force bool on `measured`.
    out["measured"] = bool(out.get("measured", False))
    return out


def _resolve_optional(dynamic: dict, key: str) -> float | None:
    """Optional statistic field; returns `None` if absent or `null`."""
    if key not in dynamic:
        return None
    value = dynamic[key]
    if value is None:
        return None
    return float(value)


class _DriverScaledCar:
    """Composition wrapper. Delegates everything to `car`, overrides the
    grip-limited methods to apply `skill_pct` (and optional per-point noise).
    """

    def __init__(self, car, driver: Driver, *, rng=None, noise=False):
        self._car = car
        self._driver = driver
        self._rng = rng
        self._noise = bool(noise) and rng is not None and driver.grip_sigma > 0
        # Pre-pull constants the simulator reads
        self.drive_type = car.drive_type
        self.total_mass = car.total_mass
        self.power_rpm = car.power_rpm

    # ---- transparent delegation for non-grip methods ----
    def __getattr__(self, item):
        return getattr(self._car, item)

    def _grip_scale(self):
        s = self._driver.skill_pct
        if self._noise:
            s = s * (1.0 + float(self._rng.normal(0.0, self._driver.grip_sigma)))
            s = max(0.05, min(1.05, s))
        return s

    def tyre_grip_lateral(self, speed_ms):
        return self._car.tyre_grip_lateral(speed_ms) * self._grip_scale()

    def tyre_grip_longitudinal(self, speed_ms):
        return self._car.tyre_grip_longitudinal(speed_ms) * self._grip_scale()

    def max_cornering_speed(self, radius):
        # Re-derive iterative solve using the scaled lateral grip.
        import numpy as np
        if radius <= 0 or radius > 100000:
            return 999.0
        g = 9.81
        v = float(np.sqrt(self._car.tyre_dy0_f * g * radius))
        for _ in range(20):
            mu = self.tyre_grip_lateral(v)
            total_normal = self._car.total_mass * g + self._car.downforce(v)
            v_new = float(np.sqrt(mu * total_normal * radius / self._car.total_mass))
            if abs(v_new - v) < 0.01:
                break
            v = 0.5 * (v + v_new)
        return v

    def max_braking_decel(self, speed_ms):
        g = 9.81
        mu = self.tyre_grip_longitudinal(speed_ms)
        total_normal = self._car.total_mass * g + self._car.downforce(speed_ms)
        grip_force = mu * total_normal
        drag = self._car.drag_force(speed_ms)
        return (grip_force + drag) / self._car.total_mass

    def max_traction_force(self, speed_ms):
        gear = self._car.optimal_gear(speed_ms)
        rpm = self._car.rpm_from_speed(speed_ms, gear)
        rpm = max(self._car.power_rpm[0], min(self._car.rev_limit, rpm))
        wt = self._car.wheel_torque(rpm, gear)
        force = wt / self._car.tyre_radius_r
        grip = self.tyre_grip_longitudinal(speed_ms)
        weight_on_driven = self._car._driven_axle_load(speed_ms)
        max_grip_force = grip * weight_on_driven
        return min(force, max_grip_force)

    def max_accel(self, speed_ms):
        traction = self.max_traction_force(speed_ms)
        drag = self._car.drag_force(speed_ms)
        rr = self._car.rolling_resistance(speed_ms)
        return (traction - drag - rr) / self._car.total_mass

    def __repr__(self):
        return (
            f"<DriverScaledCar driver={self._driver.name} "
            f"skill={self._driver.skill_pct} car={self._car!r}>"
        )
