"""Per-wheel tyre state evolution: temperature, wear, pressure (spec §21.3).

State per wheel `w ∈ {FL, FR, RL, RR}`:
  - `temp_C[w]`        -- bulk tyre temperature (Celsius).
  - `wear_pct[w]`      -- 0..100, 100 = fresh, 0 = bald.
  - `pressure_psi[w]`  -- current hot pressure.

Architectural posture: option 2.5 (spec §21.1). The existing 3-pass simulator
and single-scalar grip envelope are retained unchanged; per-wheel state is
computed OFFLINE between laps and reduced to scalars
`(mu_x_scale, mu_y_scale, drag_scale)` for the next lap's solver pass.

Per-segment update flow (spec §21.3):
  1. Slip-energy attribution from `v², R, binding_label` + load-transfer.
  2. Thermal Euler step: `dT = (P_heat - h·(T - T_amb)) / C_thermal · dt`.
  3. Wear: `dwear = k_wear · dE/dt · f_temp_penalty · dt`.
  4. Pressure: ideal-gas `p_hot = p_cold · (T+273.15) / T_cold_K`.

Scalar reduction (spec §21.3 step 5, v1.3 asymmetric pressure model):
  - Grip: `g[w] = f_temp(T) · f_wear(wear%) · f_pressure_grip(p)`.
    Penalty active only ABOVE PRESSURE_IDEAL (over-pressure -> smaller patch).
  - Drag: `d[w] = f_pressure_drag(p)`.
    Penalty active only BELOW PRESSURE_IDEAL (under-pressure -> sidewall flex
    + rolling resistance); small benefit above IDEAL.
  - `g_combined = 0.5 · (min(g_FL, g_FR) + min(g_RL, g_RR))`.
  - `drag_scale = mean(d_FL, d_FR, d_RL, d_RR)`.
  - Returned as `(mu_x_scale, mu_y_scale, drag_scale)` — single-grip envelope
    plus a separate scalar that multiplies the total drag force (aero + rolling
    resistance) in the 3-pass solver. `drag_scale == 1.0` at IDEAL pressure
    preserves single-lap regression (§11.30).

Hand-coded constants (per §21.3):
  - `k_load = 0.30` -- lateral load-transfer coefficient.
  - `k_long = 0.20` -- longitudinal load-transfer coefficient.
  - `STRAIGHT_THRESHOLD_M = 500.0` -- lat_g = 0 above this radius.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

WHEELS = ("FL", "FR", "RL", "RR")
G = 9.81
STRAIGHT_THRESHOLD_M = 500.0

# Hand-coded load-transfer coefficients (spec §21.3 / Decisions item 23).
K_LOAD = 0.30
K_LONG = 0.20

# Pressure-falloff clamp (spec §21.3 step 5).
PRESSURE_CLAMP_LO = 0.3
PRESSURE_CLAMP_HI = 1.0

# v1.3 drag-scale clamp (spec §21.3 step 5b).
DRAG_CLAMP_LO = 0.85
DRAG_CLAMP_HI = 1.50


@dataclass
class TyreCalibration:
    """Four scalar knobs that govern the per-wheel state evolution.

    Bounds (spec §7.2):
      k_friction  in [0.1, 10.0]
      h           in [5.0, 500.0]   (Watts per Kelvin per tyre)
      C_thermal   in [500.0, 50000.0] (Joules per Kelvin per tyre)
      k_wear      in [1e-9, 1e-4]   (pct per Joule per second, modulated by f_temp_penalty)
    """
    k_friction: float = 1.0
    h: float = 50.0
    C_thermal: float = 5000.0
    k_wear: float = 1.0e-7
    measured: bool = False
    source: dict = field(default_factory=dict)

    def clamped(self) -> "TyreCalibration":
        return TyreCalibration(
            k_friction=float(np.clip(self.k_friction, 0.1, 10.0)),
            h=float(np.clip(self.h, 5.0, 500.0)),
            C_thermal=float(np.clip(self.C_thermal, 500.0, 50000.0)),
            k_wear=float(np.clip(self.k_wear, 1.0e-9, 1.0e-4)),
            measured=self.measured,
            source=dict(self.source),
        )


@dataclass
class TyreState:
    """Per-wheel state. All four wheels evolve in lockstep per segment."""
    temp_C: dict  # {"FL": float, ...}
    wear_pct: dict
    pressure_psi: dict  # current hot pressure
    pressure_cold_psi: dict  # initial cold pressure (set once at stint start)
    T_cold_K: float  # ambient + 273.15 (set once at stint start)
    ambient_temp_C: float
    cumulative_slip_energy_J: dict = field(
        default_factory=lambda: {w: 0.0 for w in WHEELS}
    )

    @classmethod
    def from_setup(cls, setup) -> "TyreState":
        """Build initial state from a `Setup`. Tyres start at ambient temp, fresh."""
        amb = float(setup.ambient_temp_C)
        return cls(
            temp_C={w: amb for w in WHEELS},
            wear_pct={w: 100.0 for w in WHEELS},
            pressure_psi={w: float(setup.pressures_psi[w]) for w in WHEELS},
            pressure_cold_psi={w: float(setup.pressures_psi[w]) for w in WHEELS},
            T_cold_K=amb + 273.15,
            ambient_temp_C=amb,
            cumulative_slip_energy_J={w: 0.0 for w in WHEELS},
        )

    def copy(self) -> "TyreState":
        return TyreState(
            temp_C=dict(self.temp_C),
            wear_pct=dict(self.wear_pct),
            pressure_psi=dict(self.pressure_psi),
            pressure_cold_psi=dict(self.pressure_cold_psi),
            T_cold_K=self.T_cold_K,
            ambient_temp_C=self.ambient_temp_C,
            cumulative_slip_energy_J=dict(self.cumulative_slip_energy_J),
        )

    def avg_temp_C(self) -> float:
        return float(np.mean(list(self.temp_C.values())))

    def avg_pressure_psi(self) -> float:
        return float(np.mean(list(self.pressure_psi.values())))


# ---------------------------------------------------------------------------
# LUT loaders (cached at module level so repeated `simulate_stint` calls don't
# re-parse the disk-resident lookups).
# ---------------------------------------------------------------------------

_LUT_CACHE: dict = {}


def _parse_lut(path: str) -> tuple[np.ndarray, np.ndarray]:
    xs, ys = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(";"):
                continue
            if "|" in line:
                a, b = line.split("|", 1)
                try:
                    xs.append(float(a))
                    ys.append(float(b))
                except ValueError:
                    continue
    return np.array(xs, dtype=float), np.array(ys, dtype=float)


def _load_lut_cached(path: str) -> tuple[np.ndarray, np.ndarray]:
    if path in _LUT_CACHE:
        return _LUT_CACHE[path]
    x, y = _parse_lut(path)
    _LUT_CACHE[path] = (x, y)
    return x, y


@dataclass
class CarTyreModel:
    """Distilled per-car tyre parameters needed for state evolution + grip envelope.

    Built once per car at the start of a stint sim (`build_car_tyre_model`) so we
    don't re-parse `tyres.ini` lookups every segment.
    """
    # Pressure-falloff curve (spec §21.3 step 5d): per-axle `(PRESSURE_IDEAL, PRESSURE_D_GAIN)`.
    pressure_ideal_front: float
    pressure_ideal_rear: float
    pressure_d_gain_front: float
    pressure_d_gain_rear: float
    # Temperature performance LUT (`tcurve_*.lut`) - x=°C, y=grip multiplier.
    tcurve_front_x: np.ndarray
    tcurve_front_y: np.ndarray
    tcurve_rear_x: np.ndarray
    tcurve_rear_y: np.ndarray
    # Wear performance LUT (`street_front.lut` / `street_rear.lut`) - x=virtual_km, y=grip%.
    # We map our wear_pct (0..100, 100=fresh) to an equivalent km along the wear LUT
    # by linear interpolation against the LUT's grip-percent column.
    wear_curve_front_x: np.ndarray
    wear_curve_front_y: np.ndarray
    wear_curve_rear_x: np.ndarray
    wear_curve_rear_y: np.ndarray
    # Per-axle base grip (for the wear LUT's y normalisation). We treat the LUT's
    # y=100 row as "fresh" and convert to a multiplier in `[0, 1]`.
    # The LUT also provides total virtual-km range used for `k_wear` axis mapping
    # in the inverse direction (we don't currently use this — `wear_pct` is the
    # state, the LUT shapes its grip contribution only).
    wear_km_max_front: float
    wear_km_max_rear: float
    # v1.3 (spec §21.3 step 5b): asymmetric pressure-drag constants, sourced
    # from the active `Compound`. Compound-agnostic in v1.3 (both Street and
    # Semislicks default to 0.5 / 0.10); per-compound calibration is v1.4.
    k_drag: float = 0.5
    k_drag_reduction: float = 0.10


def build_car_tyre_model(car, compound=None) -> CarTyreModel:
    """Parse all needed tyre LUT data for a given Car + active compound.

    Spec §21.11: per-compound `f_pressure`/`f_temp`/`f_wear` lookups. When
    `compound` is None, falls back to `car.default_compound` (back-compat for
    pre-v2.5 callers).

    Reads LUTs from `car.data_dir`:
      - Wear curves: `compound.wear_curve_{front,rear}` (e.g. `street_front.lut`
        for compound 0, `semislicks_front.lut` for compound 1).
      - Thermal performance curves: `compound.thermal_lut_{front,rear}`
        (e.g. `tcurve_street.lut` vs `tcurve_semis.lut`).
    """
    if compound is None:
        compound = car.default_compound
    d = car.data_dir
    tcurve_f_x, tcurve_f_y = _load_lut_cached(os.path.join(d, compound.thermal_lut_front))
    tcurve_r_x, tcurve_r_y = _load_lut_cached(os.path.join(d, compound.thermal_lut_rear))
    wear_f_x, wear_f_y = _load_lut_cached(os.path.join(d, compound.wear_curve_front))
    wear_r_x, wear_r_y = _load_lut_cached(os.path.join(d, compound.wear_curve_rear))

    return CarTyreModel(
        pressure_ideal_front=float(compound.pressure_ideal_front),
        pressure_ideal_rear=float(compound.pressure_ideal_rear),
        pressure_d_gain_front=float(compound.pressure_d_gain),
        pressure_d_gain_rear=float(compound.pressure_d_gain),
        tcurve_front_x=tcurve_f_x,
        tcurve_front_y=tcurve_f_y,
        tcurve_rear_x=tcurve_r_x,
        tcurve_rear_y=tcurve_r_y,
        wear_curve_front_x=wear_f_x,
        wear_curve_front_y=wear_f_y,
        wear_curve_rear_x=wear_r_x,
        wear_curve_rear_y=wear_r_y,
        wear_km_max_front=float(wear_f_x[-1]) if len(wear_f_x) else 50.0,
        wear_km_max_rear=float(wear_r_x[-1]) if len(wear_r_x) else 50.0,
        k_drag=float(getattr(compound, "k_drag", 0.5)),
        k_drag_reduction=float(getattr(compound, "k_drag_reduction", 0.10)),
    )


# ---------------------------------------------------------------------------
# Envelope helpers
# ---------------------------------------------------------------------------

def f_temp(model: CarTyreModel, temp_C: float, *, front: bool) -> float:
    """Grip multiplier at temperature `temp_C` (from `tcurve_*.lut`)."""
    x = model.tcurve_front_x if front else model.tcurve_rear_x
    y = model.tcurve_front_y if front else model.tcurve_rear_y
    if len(x) == 0:
        return 1.0
    val = float(np.interp(float(temp_C), x, y))
    return float(np.clip(val, 0.0, 1.5))


def f_wear(model: CarTyreModel, wear_pct: float, *, front: bool) -> float:
    """Grip multiplier at `wear_pct` (0..100, 100=fresh) from `WEAR_CURVE`.

    The AC `street_*.lut` is x=virtual_km, y=grip_percent (100=fresh, 80=bald).
    We treat `wear_pct` as the fresh-percent state itself: 100 -> first LUT row,
    0 -> last LUT row. The grip multiplier is `lut.y(km) / 100`.
    """
    x = model.wear_curve_front_x if front else model.wear_curve_rear_x
    y = model.wear_curve_front_y if front else model.wear_curve_rear_y
    if len(x) == 0:
        return 1.0
    # Map wear_pct (100=fresh) to km axis: km_max * (1 - wear_pct/100)
    km_max = float(x[-1])
    km = km_max * (1.0 - float(np.clip(wear_pct, 0.0, 100.0)) / 100.0)
    grip_pct = float(np.interp(km, x, y))
    # y is in percent (100=fresh); convert to multiplier.
    return float(np.clip(grip_pct / 100.0, 0.0, 1.5))


def f_pressure_grip(model: CarTyreModel, p_psi: float, *, front: bool) -> float:
    """Cornering-grip multiplier vs pressure (spec §21.3 step 5c, v1.3 asymmetric).

    Above `PRESSURE_IDEAL`: one-sided quadratic falloff (over-pressure -> smaller
    contact patch -> less grip), clamped to `[0.30, 1.0]`.
    Below `PRESSURE_IDEAL`: returns `1.0` (no grip penalty; real-world bonus
    from a larger contact patch is ignored in v1.3 — see spec §3 non-goals).
    """
    p_ideal = model.pressure_ideal_front if front else model.pressure_ideal_rear
    d_gain = model.pressure_d_gain_front if front else model.pressure_d_gain_rear
    p = float(p_psi)
    if p >= p_ideal:
        val = 1.0 - d_gain * (p - p_ideal) ** 2
        return float(np.clip(val, PRESSURE_CLAMP_LO, PRESSURE_CLAMP_HI))
    return 1.0


def f_pressure_drag(model: CarTyreModel, p_psi: float, *, front: bool) -> float:
    """Straight-line drag multiplier vs pressure (spec §21.3 step 5b, v1.3).

    Below `PRESSURE_IDEAL`: linear penalty `1 + k_drag · (IDEAL - p)/IDEAL`
    (under-pressure -> sidewall flex + rolling resistance -> more drag).
    Above `PRESSURE_IDEAL`: small linear benefit
    `1 - k_drag_reduction · (p - IDEAL)/IDEAL` (less rolling resistance from a
    stiffer, smaller patch).
    Final clamp `[0.85, 1.50]`. `k_drag` / `k_drag_reduction` are sourced from
    the active compound (hand-defaults `0.5` / `0.10` in v1.3).
    """
    p_ideal = model.pressure_ideal_front if front else model.pressure_ideal_rear
    p = float(p_psi)
    if p < p_ideal:
        penalty = model.k_drag * max((p_ideal - p) / max(p_ideal, 1e-6), 0.0)
        return float(np.clip(1.0 + penalty, DRAG_CLAMP_LO, DRAG_CLAMP_HI))
    benefit = model.k_drag_reduction * (p - p_ideal) / max(p_ideal, 1e-6)
    return float(np.clip(1.0 - benefit, DRAG_CLAMP_LO, DRAG_CLAMP_HI))


def f_pressure(model: CarTyreModel, p_psi: float, *, front: bool) -> float:
    """Back-compat alias -> `f_pressure_grip` (spec §21.3 step 5c).

    Pre-v1.3 callers in `driver_fit.py` query the cornering-grip envelope only;
    the asymmetric model preserves the same `lat_g_max` semantics on the
    over-pressure side and removes the (incorrect) under-pressure grip
    penalty. Drag is a separate plumbing path (`f_pressure_drag`).
    """
    return f_pressure_grip(model, p_psi, front=front)


def f_temp_penalty(model: CarTyreModel, temp_C: float, *, front: bool) -> float:
    """Wear-rate penalty as `1 / max(grip(T), 0.3)` (spec §21.3 step 3a)."""
    grip = f_temp(model, temp_C, front=front)
    return 1.0 / max(grip, 0.3)


def combined_grip_envelope(
    state: TyreState, model: CarTyreModel
) -> tuple[float, float, float]:
    """Reduce per-wheel state to `(mu_x_scale, mu_y_scale, drag_scale)` scalars.

    Spec §21.3 step 5 (v1.3 — return arity 3):
      - `g[w] = f_temp(T) · f_wear(wear%) · f_pressure_grip(p)` per wheel.
        `g_combined = 0.5 · (min(g_FL, g_FR) + min(g_RL, g_RR))`.
        `mu_x_scale = mu_y_scale = g_combined` (single-grip envelope).
      - `d[w] = f_pressure_drag(p)` per wheel.
        `drag_scale = mean(d_FL, d_FR, d_RL, d_RR)`.
    """
    def g_of(w: str) -> float:
        is_front = w[0] == "F"
        return (
            f_temp(model, state.temp_C[w], front=is_front)
            * f_wear(model, state.wear_pct[w], front=is_front)
            * f_pressure_grip(model, state.pressure_psi[w], front=is_front)
        )

    def d_of(w: str) -> float:
        is_front = w[0] == "F"
        return f_pressure_drag(model, state.pressure_psi[w], front=is_front)

    g_fl, g_fr, g_rl, g_rr = g_of("FL"), g_of("FR"), g_of("RL"), g_of("RR")
    g_front = min(g_fl, g_fr)
    g_rear = min(g_rl, g_rr)
    g_combined = 0.5 * (g_front + g_rear)
    # Numerical floor: never let combined grip fall below 0.30 (matches
    # f_pressure_grip's lower clamp). Prevents pathological lap-time explosion
    # when uncalibrated defaults drive temperatures into the tail of
    # `tcurve_*.lut`.
    g_combined = float(max(g_combined, 0.30))

    drag_scale = float(
        np.mean([d_of("FL"), d_of("FR"), d_of("RL"), d_of("RR")])
    )
    return g_combined, g_combined, drag_scale


# ---------------------------------------------------------------------------
# Per-segment update (spec §21.3 steps 1-4)
# ---------------------------------------------------------------------------

@dataclass
class SegmentInfo:
    """One segment's worth of input to the state update."""
    distance_m: float
    segment_length_m: float
    v_ms: float
    radius_m: float        # unsigned (from track CSV); used for lat_g magnitude.
    radius_sign: int       # +1 for right corners, -1 for left, 0 for straight.
    binding_label: str     # "corner" / "accel" / "brake"


def _load_share(radius_sign: int, lat_g: float, long_g: float, cg_front: float,
                accel_or_brake: str) -> dict:
    """Per-wheel normalised load share (sums to 1.0 across the four wheels).

    Combines static front-axle bias (`cg_front`), lateral load transfer
    (`k_load · |lat_g|`), and longitudinal load transfer (`k_long · long_g`).

    Sign convention:
      - `radius_sign = +1` => right-hander, outer wheels = LEFT (FL, RL).
      - `radius_sign = -1` => left-hander,  outer wheels = RIGHT (FR, RR).
      - `radius_sign = 0`  => straight, no lateral transfer.
      - `accel_or_brake = "accel"` => weight shifts rearward (front lighter).
      - `accel_or_brake = "brake"` => weight shifts forward (rear lighter).
      - `accel_or_brake = "corner"` => no longitudinal transfer.
    """
    # Base static share per axle, split evenly L/R.
    front_share = cg_front  # fraction of total weight on front axle (static).
    rear_share = 1.0 - cg_front
    # Static per-wheel:
    base = {"FL": front_share * 0.5, "FR": front_share * 0.5,
            "RL": rear_share * 0.5,  "RR": rear_share * 0.5}

    # Lateral transfer: outer wheels +k_load·|lat_g|, inner wheels -k_load·|lat_g|.
    abs_lat = abs(float(lat_g))
    if radius_sign > 0:
        # right-hander: outer = left side (FL, RL); inner = right (FR, RR)
        lat_factor = {"FL": 1.0 + K_LOAD * abs_lat, "FR": 1.0 - K_LOAD * abs_lat,
                      "RL": 1.0 + K_LOAD * abs_lat, "RR": 1.0 - K_LOAD * abs_lat}
    elif radius_sign < 0:
        lat_factor = {"FL": 1.0 - K_LOAD * abs_lat, "FR": 1.0 + K_LOAD * abs_lat,
                      "RL": 1.0 - K_LOAD * abs_lat, "RR": 1.0 + K_LOAD * abs_lat}
    else:
        lat_factor = {w: 1.0 for w in WHEELS}

    # Longitudinal transfer.
    abs_long = abs(float(long_g))
    if accel_or_brake == "accel":
        # weight shifts rearward
        long_factor = {"FL": 1.0 - K_LONG * abs_long, "FR": 1.0 - K_LONG * abs_long,
                       "RL": 1.0 + K_LONG * abs_long, "RR": 1.0 + K_LONG * abs_long}
    elif accel_or_brake == "brake":
        long_factor = {"FL": 1.0 + K_LONG * abs_long, "FR": 1.0 + K_LONG * abs_long,
                       "RL": 1.0 - K_LONG * abs_long, "RR": 1.0 - K_LONG * abs_long}
    else:
        long_factor = {w: 1.0 for w in WHEELS}

    raw = {w: base[w] * lat_factor[w] * long_factor[w] for w in WHEELS}
    total = sum(raw.values()) or 1.0
    return {w: raw[w] / total for w in WHEELS}


def _slip_energy_per_wheel(car, seg: SegmentInfo, calib: TyreCalibration,
                           state: TyreState) -> tuple[dict, float]:
    """Return `(dE[w], dt)` -- per-wheel slip energy (Joules) over the segment.

    Implements spec §21.3 step 1 ("slip-energy attribution"). All per-wheel
    energy is normalised by load-share; longitudinal energy further restricted
    to the driven axle (accel) or split by `brake_front_share` (brake).
    """
    v = max(float(seg.v_ms), 1e-3)
    dt = max(float(seg.segment_length_m) / v, 1e-6)

    # Lateral acceleration estimate (only for corners).
    if seg.radius_m < STRAIGHT_THRESHOLD_M and seg.radius_m > 1e-3:
        lat_g = (v ** 2) / (seg.radius_m * G)
    else:
        lat_g = 0.0

    # Longitudinal acceleration from binding label (best estimate -- the actual
    # solver pass is binding at this label, so this is the upper-bound demand).
    if seg.binding_label == "accel":
        long_g = float(car.max_accel(v)) / G
        accel_or_brake = "accel"
    elif seg.binding_label == "brake":
        long_g = -float(car.max_braking_decel(v)) / G
        accel_or_brake = "brake"
    else:
        long_g = 0.0
        accel_or_brake = "corner"

    cg_front = float(getattr(car, "cg_front", 0.5))
    share = _load_share(seg.radius_sign, lat_g, long_g, cg_front, accel_or_brake)

    # Total normal load with downforce.
    total_normal = car.total_mass * G + float(car.downforce(v))
    Fz = {w: total_normal * share[w] for w in WHEELS}

    # Per-wheel longitudinal acceleration (only driven/braked wheels get long.).
    drive_type = str(getattr(car, "drive_type", "RWD")).upper()
    brake_front_share = float(getattr(car, "brake_front_share", 0.6))
    long_share = {w: 0.0 for w in WHEELS}
    if accel_or_brake == "accel":
        # Driven axle absorbs all longitudinal energy.
        if drive_type == "FWD":
            long_share = {"FL": 0.5, "FR": 0.5, "RL": 0.0, "RR": 0.0}
        elif drive_type == "AWD":
            long_share = {w: 0.25 for w in WHEELS}
        else:  # RWD default
            long_share = {"FL": 0.0, "FR": 0.0, "RL": 0.5, "RR": 0.5}
    elif accel_or_brake == "brake":
        f_share = brake_front_share * 0.5
        r_share = (1.0 - brake_front_share) * 0.5
        long_share = {"FL": f_share, "FR": f_share, "RL": r_share, "RR": r_share}

    abs_a_long = abs(long_g * G)  # m/s^2
    abs_a_lat = abs(lat_g * G)

    dE: dict = {}
    for w in WHEELS:
        # Slip energy = friction work per axis. We use Fz · |a| · dt as the
        # surrogate for slip work (per spec §21.3 step 1, the `k_slip` factor
        # rolls into k_friction).
        long_energy = Fz[w] * abs_a_long * long_share[w] * dt
        lat_energy = Fz[w] * abs_a_lat * dt
        dE[w] = float(long_energy + lat_energy)

    return dE, dt


def update_per_segment(state: TyreState, seg: SegmentInfo, car,
                       calib: TyreCalibration, model: CarTyreModel) -> None:
    """Advance `state` in place for one segment (spec §21.3 steps 1-4).

    Modifies `state.temp_C`, `state.wear_pct`, `state.pressure_psi`,
    `state.cumulative_slip_energy_J` in place.
    """
    calib = calib.clamped()
    dE, dt = _slip_energy_per_wheel(car, seg, calib, state)

    for w in WHEELS:
        is_front = w[0] == "F"
        # Step 2: temperature.
        P_heat = calib.k_friction * dE[w] / max(dt, 1e-9)  # Watts
        dTdt = (P_heat - calib.h * (state.temp_C[w] - state.ambient_temp_C)) / max(calib.C_thermal, 1e-3)
        state.temp_C[w] += dTdt * dt

        # Step 3: wear.
        penalty = f_temp_penalty(model, state.temp_C[w], front=is_front)
        dwear_dt = calib.k_wear * (dE[w] / max(dt, 1e-9)) * penalty  # pct/s
        state.wear_pct[w] -= dwear_dt * dt
        state.wear_pct[w] = float(np.clip(state.wear_pct[w], 0.0, 100.0))

        # Step 4: pressure (ideal-gas, isovolumetric).
        state.pressure_psi[w] = state.pressure_cold_psi[w] * (
            (state.temp_C[w] + 273.15) / state.T_cold_K
        )

        # Diagnostic.
        state.cumulative_slip_energy_J[w] += dE[w]


def derive_radius_sign(d_query: np.ndarray, csv_data: dict) -> np.ndarray:
    """Compute signed-curvature sign at each `distance_m` query.

    Track CSVs only store unsigned radius. We infer sign from the (x, y)
    centerline via the cross product of consecutive tangents: positive
    cross -> left turn (counter-clockwise), negative -> right turn.

    Returns an int array of {-1, 0, +1}, length = `len(d_query)`.
    """
    n = len(d_query)
    out = np.zeros(n, dtype=int)
    x = csv_data.get("x")
    y = csv_data.get("y")
    d_src = csv_data.get("distance_m")
    if x is None or y is None or d_src is None or len(d_src) < 3:
        return out
    # Compute signed curvature on source grid via 3-point cross product.
    sign_src = np.zeros(len(d_src), dtype=int)
    for i in range(1, len(d_src) - 1):
        dx1 = x[i] - x[i - 1]
        dy1 = y[i] - y[i - 1]
        dx2 = x[i + 1] - x[i]
        dy2 = y[i + 1] - y[i]
        cross = dx1 * dy2 - dy1 * dx2
        if cross > 1e-9:
            sign_src[i] = -1  # left turn (CCW)
        elif cross < -1e-9:
            sign_src[i] = +1  # right turn (CW)
        else:
            sign_src[i] = 0
    sign_src[0] = sign_src[1]
    sign_src[-1] = sign_src[-2]
    # Nearest-neighbour interpolate sign onto the query grid.
    idx = np.searchsorted(d_src, d_query, side="left")
    idx = np.clip(idx, 0, len(d_src) - 1)
    out = sign_src[idx]
    return out


# ---------------------------------------------------------------------------
# Per-lap rollup (spec §21.4 step 2d)
# ---------------------------------------------------------------------------

def update_segments_in_place(
    state: TyreState,
    car,
    calib: TyreCalibration,
    model: CarTyreModel,
    *,
    distances: np.ndarray,
    speeds: np.ndarray,
    radii: np.ndarray,
    radius_signs: np.ndarray,
    labels: np.ndarray,
    on_segment=None,
) -> None:
    """Iterate per-segment and update state in place.

    Optional `on_segment(idx, state)` callback fires AFTER the per-segment update
    so callers (e.g. telemetry emission) can snapshot post-update state.
    """
    n = len(distances)
    if n < 2:
        return
    for i in range(n - 1):
        seg_len = float(distances[i + 1] - distances[i])
        if seg_len <= 0:
            if on_segment is not None:
                on_segment(i, state)
            continue
        v_mid = max(0.5 * (float(speeds[i]) + float(speeds[i + 1])), 1.0)
        seg = SegmentInfo(
            distance_m=float(distances[i]),
            segment_length_m=seg_len,
            v_ms=v_mid,
            radius_m=float(radii[i]),
            radius_sign=int(radius_signs[i]) if radius_signs is not None else 0,
            binding_label=str(labels[i]) if labels is not None else "accel",
        )
        update_per_segment(state, seg, car, calib, model)
        if on_segment is not None:
            on_segment(i, state)
    # Snapshot for the final point (no segment ahead -- just emit current state).
    if on_segment is not None:
        on_segment(n - 1, state)


# ---------------------------------------------------------------------------
# Car wrapper for grip-envelope rescaling (used by simulate_stint).
# ---------------------------------------------------------------------------

class GripScaledCar:
    """Thin wrapper that rescales lateral + longitudinal grip uniformly.

    Used by `simulate_stint` to apply the previous lap's `(mu_x_scale, mu_y_scale)`
    to the next lap's 3-pass solver pass (spec §21.4 step 2b). Identical to
    `_DriverScaledCar` but the scale comes from the tyre-state reduction.

    v2 (spec §21.11): when `compound` is provided, the baseline `dy0/dx0/
    speed_sens` used for the lateral/longitudinal grip queries comes from the
    active compound, NOT the car's default-compound attributes. This lets the
    same `Car` instance drive sims for any of its `car.compounds[*]`.
    """

    def __init__(self, car, mu_x_scale: float, mu_y_scale: float, compound=None):
        self._car = car
        self._mu_x = float(mu_x_scale)
        self._mu_y = float(mu_y_scale)
        self._compound = compound
        # Pre-pull the constants the simulator reads.
        self.drive_type = car.drive_type
        self.total_mass = car.total_mass
        self.power_rpm = car.power_rpm
        # Compound-aware baselines (fall back to car's default-compound attrs).
        if compound is not None:
            self.tyre_dy0_f = float(compound.dy0_front)
            self.tyre_dy0_r = float(compound.dy0_rear)
            self.tyre_dx0_f = float(compound.dx0_front)
            self.tyre_dx0_r = float(compound.dx0_rear)
            self.tyre_speed_sens_f = float(compound.speed_sens_front)
            self.tyre_speed_sens_r = float(compound.speed_sens_rear)
        else:
            self.tyre_dy0_f = car.tyre_dy0_f
            self.tyre_dy0_r = car.tyre_dy0_r
            self.tyre_dx0_f = car.tyre_dx0_f
            self.tyre_dx0_r = car.tyre_dx0_r
            self.tyre_speed_sens_f = car.tyre_speed_sens_f
            self.tyre_speed_sens_r = car.tyre_speed_sens_r

    def __getattr__(self, item):
        return getattr(self._car, item)

    def tyre_grip_lateral(self, speed_ms):
        mu_f = self.tyre_dy0_f / (1.0 + self.tyre_speed_sens_f * speed_ms)
        mu_r = self.tyre_dy0_r / (1.0 + self.tyre_speed_sens_r * speed_ms)
        return min(mu_f, mu_r) * self._mu_y

    def tyre_grip_longitudinal(self, speed_ms):
        mu_f = self.tyre_dx0_f / (1.0 + self.tyre_speed_sens_f * speed_ms)
        mu_r = self.tyre_dx0_r / (1.0 + self.tyre_speed_sens_r * speed_ms)
        drive = str(getattr(self._car, "drive_type", "RWD")).upper()
        if drive == "RWD":
            base = mu_r
        elif drive == "FWD":
            base = mu_f
        else:
            base = min(mu_f, mu_r)
        return base * self._mu_x

    def max_cornering_speed(self, radius):
        if radius <= 0 or radius > 100000:
            return 999.0
        v = float(math.sqrt(self.tyre_dy0_f * G * radius))
        for _ in range(20):
            mu = self.tyre_grip_lateral(v)
            total_normal = self._car.total_mass * G + self._car.downforce(v)
            v_new = float(math.sqrt(mu * total_normal * radius / self._car.total_mass))
            if abs(v_new - v) < 0.01:
                break
            v = 0.5 * (v + v_new)
        return v

    def max_braking_decel(self, speed_ms):
        mu = self.tyre_grip_longitudinal(speed_ms)
        total_normal = self._car.total_mass * G + self._car.downforce(speed_ms)
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
