"""Parse Assetto Corsa car data into a physics model.

v2 (spec §6.14, §21.11): multi-compound tyres.ini parsing. The `Car` exposes
`compounds: list[Compound]` (one entry per `[FRONT_n]/[REAR_n]/[THERMAL_FRONT_n]
/[THERMAL_REAR_n]` quadruple; un-suffixed sections = compound 0) and
`default_compound_index: int` (honours `[COMPOUND_DEFAULT].INDEX` if present,
else 0). Each `Compound` carries per-axle `pressure_ideal`, `pressure_static`,
`pressure_d_gain`, `dy0/dy1/dx0/dx1/dy_ref/dx_ref`, `wear_curve_{front,rear}`
LUT paths, and `thermal_lut_{front,rear}` LUT paths. The legacy direct
attributes (`car.tyre_dy0_f`, `car.tyre_speed_sens_f`, etc.) are still
populated from the default compound for back-compat with the v1.2.1 simulator
single-grip-envelope codepath. Active-compound resolution happens at the
sim/fitter entry points (spec §21.11).
"""
import configparser
import os
import re
from dataclasses import dataclass

import numpy as np


def parse_lut(filepath):
    """Parse a lookup table file (RPM|value format) into numpy arrays."""
    x, y = [], []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            parts = line.split('|')
            if len(parts) == 2:
                try:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))
                except ValueError:
                    continue
    return np.array(x), np.array(y)


def parse_ini(filepath):
    """Parse an AC ini file, stripping inline comments."""
    config = configparser.ConfigParser(inline_comment_prefixes=(';',), strict=False)
    config.read(filepath)
    return config


@dataclass(frozen=True)
class Compound:
    """One tyre compound parsed from `tyres.ini` (spec §21.11).

    `wear_curve_*` and `thermal_lut_*` are file paths (relative to the car data
    dir) of the corresponding `.lut` files; loading is deferred to
    `tyre_state.build_car_tyre_model(car, compound)`.
    """
    index: int
    name: str
    short_name: str
    # Per-axle cold + ideal pressures (psi).
    pressure_ideal_front: float
    pressure_ideal_rear: float
    pressure_static_front: float
    pressure_static_rear: float
    # Pressure-falloff quadratic gain (spec §21.3 step 5c). Shared front/rear in
    # AC for the BMW M1; if a future car splits, extend per-axle.
    pressure_d_gain: float
    # Grip coefficients per axle (DY = lateral, DX = longitudinal, DY1/DX1 =
    # load sensitivity slopes, *_ref = reference values).
    dy0_front: float
    dy1_front: float
    dx0_front: float
    dx1_front: float
    dy_ref_front: float
    dx_ref_front: float
    dy0_rear: float
    dy1_rear: float
    dx0_rear: float
    dx1_rear: float
    dy_ref_rear: float
    dx_ref_rear: float
    # Speed sensitivities (per-axle).
    speed_sens_front: float
    speed_sens_rear: float
    # LUT filenames (resolved against `car.data_dir` at load time downstream).
    wear_curve_front: str
    wear_curve_rear: str
    thermal_lut_front: str
    thermal_lut_rear: str
    # v1.3 (spec §21.3, §21.11): asymmetric pressure-drag constants. NOT parsed
    # from `tyres.ini` -- AC has no source for them. Hand-defaults, compound-
    # agnostic in v1.3 (both Street and Semislicks share 0.5 / 0.10); per-
    # compound calibration is a v1.4 candidate (spec §21.10).
    k_drag: float = 0.5
    k_drag_reduction: float = 0.10


_COMPOUND_FAMILIES = ("THERMAL_FRONT", "THERMAL_REAR", "FRONT", "REAR")
_TRAILING_PAREN_RE = re.compile(r"\s*\([^)]+\)\s*$")
_SUFFIX_RE = re.compile(r"_(\d+)$")


def _classify_section(section: str) -> "tuple[str, int] | None":
    """Return `(family, index)` if `section` is a compound section, else None.

    Order matters: "THERMAL_FRONT" must be tested before "FRONT" so that
    `THERMAL_FRONT_1` isn't mis-classified as `FRONT` with suffix `T_1`.
    """
    for family in _COMPOUND_FAMILIES:
        if section == family:
            return family, 0
        if section.startswith(family + "_"):
            m = _SUFFIX_RE.match(section[len(family):])
            if m:
                return family, int(m.group(1))
    return None


def _discover_compound_indices(ini: configparser.ConfigParser) -> list:
    """Scan all section names; group by compound index (suffix `_n`, un-suffixed = 0).

    Returns a sorted list of int indices for which all four families exist.
    Raises a ValueError if any family is missing for a given index.
    """
    by_index: dict = {}
    for section in ini.sections():
        hit = _classify_section(section)
        if hit is None:
            continue
        family, idx = hit
        by_index.setdefault(idx, set()).add(family)

    # Validate: each discovered index must have all four families.
    complete = []
    for idx, families in sorted(by_index.items()):
        missing = set(_COMPOUND_FAMILIES) - families
        if missing:
            sec_names = ", ".join(
                f"[{f}{'' if idx == 0 else f'_{idx}'}]" for f in sorted(missing)
            )
            raise ValueError(
                f"tyres.ini compound index {idx}: missing section(s) {sec_names}"
            )
        complete.append(idx)
    if not complete:
        raise ValueError(
            "tyres.ini: no compound sections found "
            "(need [FRONT]/[REAR]/[THERMAL_FRONT]/[THERMAL_REAR])"
        )
    return complete


def _section_name(family: str, index: int) -> str:
    return family if index == 0 else f"{family}_{index}"


def _parse_compound(ini: configparser.ConfigParser, index: int) -> Compound:
    """Build a `Compound` for the given compound index from the ini sections."""
    fs = ini[_section_name("FRONT", index)]
    rs = ini[_section_name("REAR", index)]
    tfs = ini[_section_name("THERMAL_FRONT", index)]
    trs = ini[_section_name("THERMAL_REAR", index)]

    # NAME/SHORT_NAME live on the FRONT section in AC's schema; SHORT_NAME may
    # be absent on user-modded cars -- fall back to NAME.
    name = str(fs.get("NAME", f"compound_{index}")).strip()
    short = str(fs.get("SHORT_NAME", name)).strip()

    # PRESSURE_D_GAIN is per-section in the ini but shared front/rear for the
    # BMW M1; we take the FRONT value (REAR is the same).
    pressure_d_gain = float(fs.get("PRESSURE_D_GAIN", "0.004"))

    return Compound(
        index=index,
        name=name,
        short_name=short,
        pressure_ideal_front=float(fs.get("PRESSURE_IDEAL", "30")),
        pressure_ideal_rear=float(rs.get("PRESSURE_IDEAL", "30")),
        pressure_static_front=float(fs.get("PRESSURE_STATIC", "30")),
        pressure_static_rear=float(rs.get("PRESSURE_STATIC", "30")),
        pressure_d_gain=pressure_d_gain,
        dy0_front=float(fs.get("DY0", "1.0")),
        dy1_front=float(fs.get("DY1", "0.0")),
        dx0_front=float(fs.get("DX0", "1.0")),
        dx1_front=float(fs.get("DX1", "0.0")),
        dy_ref_front=float(fs.get("DY_REF", fs.get("DY0", "1.0"))),
        dx_ref_front=float(fs.get("DX_REF", fs.get("DX0", "1.0"))),
        dy0_rear=float(rs.get("DY0", "1.0")),
        dy1_rear=float(rs.get("DY1", "0.0")),
        dx0_rear=float(rs.get("DX0", "1.0")),
        dx1_rear=float(rs.get("DX1", "0.0")),
        dy_ref_rear=float(rs.get("DY_REF", rs.get("DY0", "1.0"))),
        dx_ref_rear=float(rs.get("DX_REF", rs.get("DX0", "1.0"))),
        speed_sens_front=float(fs.get("SPEED_SENSITIVITY", "0.0")),
        speed_sens_rear=float(rs.get("SPEED_SENSITIVITY", "0.0")),
        wear_curve_front=str(fs.get("WEAR_CURVE", "street_front.lut")).strip(),
        wear_curve_rear=str(rs.get("WEAR_CURVE", "street_rear.lut")).strip(),
        thermal_lut_front=str(tfs.get("PERFORMANCE_CURVE", "tcurve_street.lut")).strip(),
        thermal_lut_rear=str(trs.get("PERFORMANCE_CURVE", "tcurve_street.lut")).strip(),
    )


def _parse_compounds(ini: configparser.ConfigParser) -> tuple:
    """Return `(compounds_sorted_by_index, default_index)` from a parsed tyres.ini."""
    indices = _discover_compound_indices(ini)
    compounds = [_parse_compound(ini, i) for i in indices]
    default = 0
    if ini.has_section("COMPOUND_DEFAULT"):
        try:
            default = int(ini["COMPOUND_DEFAULT"].get("INDEX", "0"))
        except (TypeError, ValueError):
            default = 0
    if default not in indices:
        default = indices[0]
    return compounds, default


class Car:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self._load(data_dir)

    def _load(self, d):
        # Engine
        eng = parse_ini(os.path.join(d, 'engine.ini'))
        self.rev_limit = float(eng['ENGINE_DATA']['LIMITER'])
        self.engine_inertia = float(eng['ENGINE_DATA']['INERTIA'])

        # Power curve: RPM -> torque in Nm
        rpm, power = parse_lut(os.path.join(d, 'power.lut'))
        self.power_rpm = rpm
        self.power_values = power  # these are torque values in Nm

        # Turbo
        self.turbo_max_boost = 0.0
        if eng.has_section('TURBO_0'):
            self.turbo_max_boost = float(eng['TURBO_0']['WASTEGATE'])

        # Drivetrain
        dt = parse_ini(os.path.join(d, 'drivetrain.ini'))
        self.drive_type = dt['TRACTION']['TYPE'].strip()
        n_gears = int(dt['GEARS']['COUNT'])
        self.gear_ratios = [float(dt['GEARS'][f'GEAR_{i+1}']) for i in range(n_gears)]
        self.final_ratio = float(dt['GEARS']['FINAL'])

        # Car basics
        car = parse_ini(os.path.join(d, 'car.ini'))
        self.mass = float(car['BASIC']['TOTALMASS'])
        self.steer_lock = float(car['CONTROLS']['STEER_LOCK'])
        self.fuel_default = float(car['FUEL']['FUEL'])

        # Suspensions
        susp = parse_ini(os.path.join(d, 'suspensions.ini'))
        self.wheelbase = float(susp['BASIC']['WHEELBASE'])
        self.cg_front = float(susp['BASIC']['CG_LOCATION'])

        # Tyres - parse ALL compounds (spec §6.14, §21.11).
        tyres = parse_ini(os.path.join(d, 'tyres.ini'))
        self.compounds, self.default_compound_index = _parse_compounds(tyres)
        # Tyre radius is compound-independent in the BMW M1 ini but read it
        # from the FRONT/REAR sections (un-suffixed = compound 0) for safety.
        self.tyre_radius_f = float(tyres['FRONT']['RADIUS'])
        self.tyre_radius_r = float(tyres['REAR']['RADIUS'])
        # Legacy direct attributes -- pinned to compound 0 (un-suffixed
        # FRONT/REAR sections) for v1.2.1 back-compat with the single-lap
        # simulator and `--laps 2` byte-equivalence test (§11.30, §11.31).
        # Stint mode (n_laps >= 3 or measured calibration) uses the active
        # compound's dy0/dx0 via `GripScaledCar(compound=...)`.
        compound_zero = self.compounds[0]
        self.tyre_dy0_f = compound_zero.dy0_front
        self.tyre_dy0_r = compound_zero.dy0_rear
        self.tyre_dx0_f = compound_zero.dx0_front
        self.tyre_dx0_r = compound_zero.dx0_rear
        self.tyre_speed_sens_f = compound_zero.speed_sens_front
        self.tyre_speed_sens_r = compound_zero.speed_sens_rear

        # Brakes
        brakes = parse_ini(os.path.join(d, 'brakes.ini'))
        self.brake_torque = float(brakes['DATA']['MAX_TORQUE'])
        self.brake_front_share = float(brakes['DATA']['FRONT_SHARE'])

        # Aero - body drag (main component)
        aero = parse_ini(os.path.join(d, 'aero.ini'))
        self.aero_cd = self._calc_aero_cd(aero, d)
        self.aero_cl = self._calc_aero_cl(aero, d)
        chord = float(aero['WING_0']['CHORD'])
        span = float(aero['WING_0']['SPAN'])
        self.frontal_area = chord * span  # approximate frontal area

        # Total mass with fuel
        self.total_mass = self.mass + self.fuel_default * 0.75  # fuel density ~0.75 kg/L

    @classmethod
    def from_dir(cls, data_dir: str) -> "Car":
        """Construct from a car data directory. Equivalent to `Car(data_dir)`.

        Provided for spec-compatible naming (§11.34 acceptance criterion).
        """
        return cls(data_dir)

    @property
    def default_compound(self) -> Compound:
        return self.compounds[self.default_compound_index]

    def find_compound(self, query) -> "Compound | None":
        """Resolve a name/short-name to a `Compound`. Case-insensitive.

        Accepts strings like `"Semislicks"`, `"SM"`, or telemetry's
        `"Semislicks (SM)"` (the trailing parenthetical is stripped).
        Returns `None` on miss.
        """
        if query is None:
            return None
        q = str(query).strip()
        if not q:
            return None
        q = _TRAILING_PAREN_RE.sub("", q).strip()
        q_lower = q.lower()
        for c in self.compounds:
            if c.name.lower() == q_lower or c.short_name.lower() == q_lower:
                return c
        return None

    def _calc_aero_cd(self, aero, d):
        """Get drag coefficient at 0 degrees AOA from body wing."""
        lut_file = os.path.join(d, aero['WING_0']['LUT_AOA_CD'].strip())
        if os.path.exists(lut_file):
            aoa, cd = parse_lut(lut_file)
            # interpolate at AOA=0
            return float(np.interp(0, aoa, cd))
        return 0.34  # default

    def _calc_aero_cl(self, aero, d):
        """Get lift coefficient at 0 degrees AOA from body wing."""
        lut_file = os.path.join(d, aero['WING_0']['LUT_AOA_CL'].strip())
        if os.path.exists(lut_file):
            aoa, cl = parse_lut(lut_file)
            return float(np.interp(0, aoa, cl))
        return -0.08  # default (negative = downforce)

    def engine_torque(self, rpm):
        """Interpolate engine torque at given RPM, including turbo."""
        base_torque = np.interp(rpm, self.power_rpm, self.power_values)
        return base_torque * (1.0 + self.turbo_max_boost)

    def wheel_torque(self, rpm, gear_idx):
        """Calculate torque at the driven wheels for a given gear (0-indexed)."""
        if gear_idx < 0 or gear_idx >= len(self.gear_ratios):
            return 0.0
        ratio = self.gear_ratios[gear_idx] * self.final_ratio
        return self.engine_torque(rpm) * ratio

    def rpm_from_speed(self, speed_ms, gear_idx):
        """Calculate engine RPM from vehicle speed and gear."""
        if gear_idx < 0 or gear_idx >= len(self.gear_ratios):
            return 0.0
        ratio = self.gear_ratios[gear_idx] * self.final_ratio
        wheel_rps = speed_ms / self.tyre_radius_r
        return wheel_rps * ratio * 60.0 / (2 * np.pi)

    def speed_from_rpm(self, rpm, gear_idx):
        """Calculate vehicle speed from RPM and gear."""
        if gear_idx < 0 or gear_idx >= len(self.gear_ratios):
            return 0.0
        ratio = self.gear_ratios[gear_idx] * self.final_ratio
        wheel_rps = rpm / ratio / 60.0 * (2 * np.pi)
        return wheel_rps * self.tyre_radius_r

    def optimal_gear(self, speed_ms):
        """Find the gear that gives maximum wheel torque at the given speed."""
        best_gear = 0
        best_torque = -1
        for g in range(len(self.gear_ratios)):
            rpm = self.rpm_from_speed(speed_ms, g)
            if rpm < 900 or rpm > self.rev_limit:
                continue
            t = self.wheel_torque(rpm, g)
            if t > best_torque:
                best_torque = t
                best_gear = g
        return best_gear

    def max_traction_force(self, speed_ms):
        """Maximum forward traction force at given speed."""
        gear = self.optimal_gear(speed_ms)
        rpm = self.rpm_from_speed(speed_ms, gear)
        rpm = np.clip(rpm, self.power_rpm[0], self.rev_limit)
        wt = self.wheel_torque(rpm, gear)
        force = wt / self.tyre_radius_r

        # Limit by tyre grip
        grip = self.tyre_grip_longitudinal(speed_ms)
        weight_on_driven = self._driven_axle_load(speed_ms)
        max_grip_force = grip * weight_on_driven
        return min(force, max_grip_force)

    def drag_force(self, speed_ms):
        """Aerodynamic drag force."""
        rho = 1.225  # air density kg/m^3
        return 0.5 * rho * self.aero_cd * self.frontal_area * speed_ms ** 2

    def downforce(self, speed_ms):
        """Aerodynamic downforce (positive = pushes car down)."""
        rho = 1.225
        # CL is negative for downforce in AC convention
        return -0.5 * rho * self.aero_cl * self.frontal_area * speed_ms ** 2

    def rolling_resistance(self, speed_ms):
        """Rolling resistance force."""
        return 10 * 4 + 0.001 * 4 * speed_ms ** 2  # simplified from tyre data

    def tyre_grip_lateral(self, speed_ms):
        """Effective lateral grip coefficient (mu_y) accounting for speed sensitivity."""
        mu_f = self.tyre_dy0_f / (1 + self.tyre_speed_sens_f * speed_ms)
        mu_r = self.tyre_dy0_r / (1 + self.tyre_speed_sens_r * speed_ms)
        return min(mu_f, mu_r)

    def tyre_grip_longitudinal(self, speed_ms):
        """Effective longitudinal grip coefficient (mu_x)."""
        mu_f = self.tyre_dx0_f / (1 + self.tyre_speed_sens_f * speed_ms)
        mu_r = self.tyre_dx0_r / (1 + self.tyre_speed_sens_r * speed_ms)
        if self.drive_type == 'RWD':
            return mu_r
        elif self.drive_type == 'FWD':
            return mu_f
        return min(mu_f, mu_r)

    def _driven_axle_load(self, speed_ms):
        """Normal load on the driven axle in Newtons."""
        g = 9.81
        total_weight = self.total_mass * g + self.downforce(speed_ms)
        if self.drive_type == 'RWD':
            return total_weight * (1 - self.cg_front)
        elif self.drive_type == 'FWD':
            return total_weight * self.cg_front
        return total_weight

    def max_cornering_speed(self, radius):
        """Maximum speed through a corner of given radius using iterative solve."""
        if radius <= 0 or radius > 100000:
            return 999.0  # straight

        g = 9.81
        # Iterative: v = sqrt(mu * (m*g + downforce(v)) * R / m)
        v = np.sqrt(self.tyre_dy0_f * g * radius)  # initial guess
        for _ in range(20):
            mu = self.tyre_grip_lateral(v)
            total_normal = self.total_mass * g + self.downforce(v)
            v_new = np.sqrt(mu * total_normal * radius / self.total_mass)
            if abs(v_new - v) < 0.01:
                break
            v = 0.5 * (v + v_new)
        return v

    def max_braking_decel(self, speed_ms):
        """Maximum braking deceleration in m/s^2."""
        g = 9.81
        mu = self.tyre_grip_longitudinal(speed_ms)
        total_normal = self.total_mass * g + self.downforce(speed_ms)
        grip_force = mu * total_normal
        drag = self.drag_force(speed_ms)
        return (grip_force + drag) / self.total_mass

    def max_accel(self, speed_ms):
        """Maximum forward acceleration in m/s^2."""
        traction = self.max_traction_force(speed_ms)
        drag = self.drag_force(speed_ms)
        rr = self.rolling_resistance(speed_ms)
        return (traction - drag - rr) / self.total_mass

    def top_speed(self):
        """Estimate top speed where drag equals max traction in top gear."""
        for v in np.arange(10, 120, 0.5):
            if self.max_accel(v) <= 0:
                return v
        return 120.0

    def __repr__(self):
        return (f"Car(mass={self.total_mass:.0f}kg, {self.drive_type}, "
                f"{len(self.gear_ratios)}spd, turbo={self.turbo_max_boost:.0%}, "
                f"Cd={self.aero_cd:.3f}, grip_y={self.tyre_dy0_f:.3f}/{self.tyre_dy0_r:.3f})")
