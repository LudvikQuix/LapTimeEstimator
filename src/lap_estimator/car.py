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
    def __init__(
        self,
        data_dir,
        *,
        boost_steady_override: float | None = None,
        cd_override: float | None = None,
        brake_torque_mult: float | None = None,
        inertia_zz_override: float | None = None,
    ):
        """Load a car from an AC car data directory.

        Optional calibration overrides (v3 powertrain calibration, spec
        addendum 2026-05-22): ``boost_steady_override`` replaces the
        ``[TURBO_0].WASTEGATE`` steady-state turbo multiplier used by
        ``engine_torque`` (and therefore by the v3 ``_engine_torque_at_wheel``
        path). ``cd_override`` replaces the body-drag coefficient
        interpolated from ``WING_0.LUT_AOA_CD`` at AOA=0. ``brake_torque_mult``
        (v3 longitudinal-physics fix, 2026-05-23) multiplies ``brake_torque``
        (per-axle-sum ``MAX_TORQUE`` from ``brakes.ini``); used when the AC ini
        value undershoots the real brake budget observed in the lake telemetry
        (BMW 1M needs ~1.80x). ``inertia_zz_override`` (v3 lateral yaw-inertia
        fix, 2026-05-23) replaces the auto-computed chassis yaw inertia
        ``I_zz`` (kg.m^2) used by the v3 ODE; see ``_chassis_geometry`` and
        ``docs/architecture-v3-lateral-yaw-inertia-fix.md``. Per-driver
        override only -- NOT baked into the ini, so other recorded laps stay
        untouched. All overrides default to ``None`` (= no override; load
        values from the ini files unchanged).
        """
        self.data_dir = data_dir
        self.boost_steady_override = boost_steady_override
        self.cd_override = cd_override
        self.brake_torque_mult = brake_torque_mult
        self.inertia_zz_override = inertia_zz_override
        self._load(data_dir)

    def _load(self, d):
        # Engine
        eng = parse_ini(os.path.join(d, 'engine.ini'))
        self.rev_limit = float(eng['ENGINE_DATA']['LIMITER'])
        self.engine_inertia = float(eng['ENGINE_DATA']['INERTIA'])
        # Coast / engine-brake reference (v3 longitudinal-physics fix). AC's
        # `[COAST_REF]` declares a linear coast curve: at `RPM` the engine
        # spins down with `TORQUE` Nm of (negative) crankshaft torque. We
        # model it as ``T_coast(rpm) = -coast_ref_torque * rpm / coast_ref_rpm``
        # (linear; NON_LINEARITY=0 in the BMW 1M ini). Falls back to 0 (no
        # engine brake) if the section is absent.
        if eng.has_section('COAST_REF'):
            self.coast_ref_rpm = float(eng['COAST_REF'].get('RPM', '7000'))
            self.coast_ref_torque = float(eng['COAST_REF'].get('TORQUE', '0'))
        else:
            self.coast_ref_rpm = 7000.0
            self.coast_ref_torque = 0.0

        # Power curve: RPM -> torque in Nm
        rpm, power = parse_lut(os.path.join(d, 'power.lut'))
        self.power_rpm = rpm
        self.power_values = power  # these are torque values in Nm

        # Turbo. v3 powertrain calibration (2026-05-23 follow-up): replace the
        # flat ``WASTEGATE`` multiplier with a proper steady-state per-RPM
        # boost curve assembled from one or more ``[TURBO_n]`` sections.
        #
        # AC steady-state formula per turbo (no lag):
        #   ratio(rpm) = (rpm / REFERENCE_RPM)^(1 / GAMMA)
        #   single_boost(rpm) = min(ratio, MAX_BOOST, WASTEGATE)
        # For multi-turbo cars the contributions add. The BMW 1M (N54)
        # declares two identical ``[TURBO_0]/[TURBO_1]`` sections so the
        # combined cap is ``2 * WASTEGATE = 0.92`` -- exactly what the
        # lake's ``turboBoost`` channel saturates at on Tomas Lap5.
        #
        # Empirical fit (`.tmp/diag_turbo_curve.py` on Tomas Lap5):
        #   - Single-turbo formula vs lake:  rmse = 0.458, bias = -0.458
        #     (predicts 0.46 cap; data sits at 0.92 -- factor-of-2 short.)
        #   - Twin-turbo additive vs lake:   rmse = 0.014, bias = +0.002
        #     (matches the data to within 1.4 % rms.)
        #
        # ``turbo_specs`` stores ``(MAX_BOOST, WASTEGATE, REFERENCE_RPM,
        # GAMMA)`` per declared turbo. ``turbo_max_boost`` is kept as a
        # back-compat alias for the *peak* steady-state multiplier (sum of
        # WASTEGATEs across all turbos), used by the v2 codepath and the
        # ``__repr__``. ``boost_steady_override`` retains its meaning: a
        # flat scalar that bypasses the per-RPM curve entirely.
        self.turbo_specs: list[tuple[float, float, float, float]] = []
        idx = 0
        while True:
            sec = f'TURBO_{idx}'
            if not eng.has_section(sec):
                break
            s = eng[sec]
            self.turbo_specs.append((
                float(s.get('MAX_BOOST', '0.0')),
                float(s.get('WASTEGATE', '0.0')),
                float(s.get('REFERENCE_RPM', '1.0')),
                float(s.get('GAMMA', '1.0')),
            ))
            idx += 1
        # Peak / fallback flat multiplier = sum of WASTEGATEs (twin = 0.92).
        # If the ini exposes only TURBO_0 (or none), this collapses to the
        # legacy single-WASTEGATE value (or 0.0). Used as the v2 flat boost
        # AND as the value reported by ``__repr__``.
        self.turbo_max_boost = sum(wg for (_mb, wg, _r, _g) in self.turbo_specs)
        if self.boost_steady_override is not None:
            self.turbo_max_boost = float(self.boost_steady_override)

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
        # v3 lateral yaw-inertia fix (2026-05-23). AC's ``[BASIC].INERTIA``
        # field is "width, height, length" of a uniform-box equivalent
        # (NOT the inertia tensor itself; see the in-ini comment and the
        # AC modding docs). AC then computes I_zz internally as
        # ``m * (w^2 + l^2) / 12`` (uniform solid box about the vertical
        # axis). We mirror that here so the v3 dynamics get a
        # plant-faithful yaw inertia. We also keep the raw box dims for
        # diagnostics. Field is optional -- fall back to ``None`` (the
        # chassis-geometry loader then uses the plate * 2.0 backstop).
        self.car_box_dims_m: tuple[float, float, float] | None = None
        if car.has_option('BASIC', 'INERTIA'):
            raw = str(car['BASIC']['INERTIA']).strip()
            parts = [p.strip() for p in raw.split(',') if p.strip()]
            if len(parts) >= 3:
                try:
                    w = float(parts[0])
                    h = float(parts[1])
                    el = float(parts[2])
                    self.car_box_dims_m = (w, h, el)
                except ValueError:
                    self.car_box_dims_m = None

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
        # Per-wheel rotational inertia (v3 longitudinal-physics fix). AC's
        # `tyres.ini` declares `ANGULAR_INERTIA` per axle (rim + tyre + brake
        # disc, kg.m^2). Falls back to 1.0 (the prior hardcoded default in
        # `_chassis_geometry.py`) if the field is absent. Used by the v3 ODE
        # for both per-wheel omega dynamics AND the engine-side reflected
        # inertia of the effective chassis mass.
        self.wheel_inertia_f = float(tyres['FRONT'].get('ANGULAR_INERTIA', '1.0'))
        self.wheel_inertia_r = float(tyres['REAR'].get('ANGULAR_INERTIA', '1.0'))
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
        # v3 longitudinal-physics fix (2026-05-23): allow a per-driver
        # multiplier on the brake torque budget. The BMW 1M's brakes.ini
        # MAX_TORQUE (3200 Nm) is ~1.80x lower than the peak ~17-22 kN brake
        # force seen in real Tomas lake telemetry. We do NOT bake this into
        # the ini (would silently corrupt other recorded laps); apply as a
        # ctor-time override instead.
        if self.brake_torque_mult is not None:
            self.brake_torque = self.brake_torque * float(self.brake_torque_mult)

        # Aero - body drag (main component). `cd_override` short-circuits
        # the WING_0 LUT lookup; the LUT-driven AOA dependence is not part
        # of the v2/v3 plant (we only ever sample AOA=0), so a scalar swap
        # is faithful.
        aero = parse_ini(os.path.join(d, 'aero.ini'))
        if self.cd_override is not None:
            self.aero_cd = float(self.cd_override)
        else:
            self.aero_cd = self._calc_aero_cd(aero, d)
        self.aero_cl = self._calc_aero_cl(aero, d)
        chord = float(aero['WING_0']['CHORD'])
        span = float(aero['WING_0']['SPAN'])
        self.frontal_area = chord * span  # approximate frontal area

        # Total mass with fuel
        self.total_mass = self.mass + self.fuel_default * 0.75  # fuel density ~0.75 kg/L

        # Yaw moment of inertia (kg.m^2). v3 lateral yaw-inertia fix
        # (2026-05-23). Resolution order, highest priority first:
        #   1. ``inertia_zz_override`` ctor arg (CLI flag ``--inertia-zz``).
        #   2. AC box formula from ``car.ini`` ``[BASIC].INERTIA``
        #      ``(w, h, l)`` dims:
        #         I_zz = m * (w^2 + l^2) / 12
        #      This is what AC's runtime physics uses internally and is
        #      the right plant-faithful default. For the BMW 1M with
        #      ``INERTIA=1.60,1.40,4.52`` and m~1592 kg, that gives
        #      I_zz ~ 3050 kg.m^2, which is in family with the
        #      manufacturer/Wikipedia 2300-2500 range and well above
        #      the prior plate-formula estimate (~1240, off by ~2x).
        #   3. Backstop: wheelbase plate formula scaled by 2.0
        #      (the prior bug; the 2x correction matches the lateral
        #      empirical-diagnostic slope of 0.131 -> ~7-8x suggested,
        #      but a flat 2x at least closes most of the gap). Only
        #      used when both 1 and 2 are unavailable.
        if self.inertia_zz_override is not None:
            self.inertia_zz = float(self.inertia_zz_override)
            self.inertia_zz_source = "override"
        elif self.car_box_dims_m is not None:
            w, _h, el = self.car_box_dims_m
            self.inertia_zz = self.total_mass * (w * w + el * el) / 12.0
            self.inertia_zz_source = "car.ini box formula"
        else:
            # Plate * 2.0 backstop. ``wheelbase`` is on hand; ``track_f``
            # is parsed by ``_chassis_geometry`` so we approximate with
            # 1.55 here (BMW 1M).
            track_f_approx = 1.55
            plate = self.total_mass * (self.wheelbase ** 2 + track_f_approx ** 2) / 12.0
            self.inertia_zz = 2.0 * plate
            self.inertia_zz_source = "plate * 2.0 backstop"

    @classmethod
    def from_dir(
        cls,
        data_dir: str,
        *,
        boost_steady_override: float | None = None,
        cd_override: float | None = None,
        brake_torque_mult: float | None = None,
        inertia_zz_override: float | None = None,
    ) -> "Car":
        """Construct from a car data directory. Equivalent to `Car(data_dir)`.

        Provided for spec-compatible naming (§11.34 acceptance criterion).
        Forwards optional calibration overrides (see ``Car.__init__``).
        """
        return cls(
            data_dir,
            boost_steady_override=boost_steady_override,
            cd_override=cd_override,
            brake_torque_mult=brake_torque_mult,
            inertia_zz_override=inertia_zz_override,
        )

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

    def turbo_boost_at_rpm(self, rpm):
        """Steady-state turbo boost multiplier at this RPM (no lag).

        Sums the per-turbo contributions described in ``_load``::

            ratio_i = (rpm / REFERENCE_RPM_i) ^ (1 / GAMMA_i)
            single_i = min(ratio_i, MAX_BOOST_i, WASTEGATE_i)
            boost(rpm) = sum_i single_i

        When ``boost_steady_override`` is set on the ctor, the override
        wins regardless of RPM (back-compat: old calibrations pinned to a
        flat boost value still behave the same way).

        Accepts a scalar or a numpy array; returns the same kind.
        """
        if self.boost_steady_override is not None:
            return float(self.boost_steady_override)
        if not self.turbo_specs:
            return 0.0
        r = np.maximum(np.asarray(rpm, dtype=float), 1.0)
        total = np.zeros_like(r, dtype=float)
        for mb, wg, ref_rpm, gamma in self.turbo_specs:
            ref = max(ref_rpm, 1.0)
            g = max(gamma, 1e-3)
            ratio = np.power(r / ref, 1.0 / g)
            single = np.minimum(ratio, mb)
            if wg > 0.0:
                single = np.minimum(single, wg)
            total = total + single
        # Preserve scalar-in / scalar-out for the common call path.
        if np.ndim(rpm) == 0:
            return float(total)
        return total

    def engine_torque(self, rpm):
        """Interpolate engine torque at given RPM, including turbo.

        Uses the per-RPM steady-state turbo curve (``turbo_boost_at_rpm``)
        rather than a flat multiplier. When ``boost_steady_override`` is
        set, the override bypasses the curve and behaves as before.
        """
        base_torque = np.interp(rpm, self.power_rpm, self.power_values)
        boost = self.turbo_boost_at_rpm(rpm)
        return base_torque * (1.0 + boost)

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
