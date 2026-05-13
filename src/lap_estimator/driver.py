"""Driver model: skill multiplier + consistency noise.

v1 keeps it intentionally minimal — `skill_pct` uniformly scales the car's
effective grip (lateral + longitudinal), `consistency_sigma` (seconds-flavour)
maps to a small per-point grip sigma for Monte-Carlo runs.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml

# Heuristic: seconds-of-jitter -> per-point grip sigma (fraction). v1 calibration.
SIGMA_SECONDS_TO_GRIP = 0.03


@dataclass
class Driver:
    name: str
    skill_pct: float
    consistency_sigma: float = 0.0
    source: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: str) -> "Driver":
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        if "skill_pct" not in raw:
            raise ValueError(f"Driver YAML {path} missing required field 'skill_pct'")
        skill = float(raw["skill_pct"])
        if not (0.0 < skill <= 1.0):
            raise ValueError(
                f"Driver YAML {path}: skill_pct must be in (0, 1], got {skill}"
            )
        sigma = float(raw.get("consistency_sigma", 0.0))
        if sigma < 0:
            raise ValueError(
                f"Driver YAML {path}: consistency_sigma must be >= 0, got {sigma}"
            )
        name = raw.get("name") or os.path.splitext(os.path.basename(path))[0]
        source = raw.get("source") or {}
        return cls(
            name=str(name),
            skill_pct=skill,
            consistency_sigma=sigma,
            source=source if isinstance(source, dict) else {},
            raw=raw,
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
        return f"<DriverScaledCar driver={self._driver.name} skill={self._driver.skill_pct} car={self._car!r}>"
