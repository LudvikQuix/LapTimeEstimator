"""Setup loader: per-wheel cold pressures + ambient temperature + compound
(spec §7.10, §21.7, §21.11).

A `Setup` is the cold-state initial condition for a stint. It carries four
cold pressures (FL/FR/RL/RR, PSI), an ambient temperature (Celsius) used as
both the initial tyre temperature and the ideal-gas reference `T_cold`, and
an optional `compound` string that selects which entry from
`car.compounds[*]` the simulator should bind.

Pressure-resolution precedence (high to low, per §21.7):
  1. Explicit `--pressure FL=...,FR=...,RL=...,RR=...` (per-wheel override).
  2. `--setup <path>` (full setup JSON file).
  3. Active compound's `PRESSURE_STATIC` per axle (cold-pressure default,
     not `PRESSURE_IDEAL` which is the hot-grip target).

Compound-resolution precedence (high to low, per §21.11):
  1. `--compound <name>` CLI flag.
  2. Setup-JSON `compound` field.
  3. (Fitter only) Telemetry's most-common `tyreCompound`.
  4. `car.default_compound_index`.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

WHEELS = ("FL", "FR", "RL", "RR")
PSI_MIN = 20.0
PSI_MAX = 50.0
DEFAULT_AMBIENT_TEMP_C = 25.0


@dataclass
class Setup:
    pressures_psi: dict  # keys: FL/FR/RL/RR; values float in [PSI_MIN, PSI_MAX]
    ambient_temp_C: float = DEFAULT_AMBIENT_TEMP_C
    source: str = "default"  # short tag for stdout: "tyres.ini" / "setups/bmw_1m_default.json" / "cli"
    name: str = "default"
    car: str | None = None
    compound: str | None = None
    notes: str = ""
    raw: dict = field(default_factory=dict)

    def validate(self) -> None:
        for w in WHEELS:
            if w not in self.pressures_psi:
                raise ValueError(f"Setup missing pressure for wheel {w}")
            p = float(self.pressures_psi[w])
            if not (PSI_MIN <= p <= PSI_MAX):
                raise ValueError(
                    f"Setup pressure {w}={p} psi out of bounds [{PSI_MIN}, {PSI_MAX}]"
                )
        if self.ambient_temp_C < -50.0 or self.ambient_temp_C > 80.0:
            raise ValueError(f"ambient_temp_C={self.ambient_temp_C} out of plausible range")

    def with_overrides(self, pressure_overrides: dict | None,
                       ambient_override: float | None) -> "Setup":
        """Return a new Setup with per-wheel pressure overrides applied + ambient."""
        merged = dict(self.pressures_psi)
        if pressure_overrides:
            for w, p in pressure_overrides.items():
                if w not in WHEELS:
                    raise ValueError(f"Unknown wheel '{w}'; expected one of {WHEELS}")
                merged[w] = float(p)
        amb = float(ambient_override) if ambient_override is not None else self.ambient_temp_C
        out = Setup(
            pressures_psi=merged,
            ambient_temp_C=amb,
            source=self.source if not pressure_overrides else f"{self.source}+cli",
            name=self.name,
            car=self.car,
            compound=self.compound,
            notes=self.notes,
            raw=self.raw,
        )
        out.validate()
        return out

    def fmt_line(self) -> str:
        p = self.pressures_psi
        compound_tag = f" | compound={self.compound}" if self.compound else ""
        return (
            f"Setup: {self.source} | "
            f"FL={p['FL']:.1f} FR={p['FR']:.1f} RL={p['RL']:.1f} RR={p['RR']:.1f} | "
            f"ambient={self.ambient_temp_C:.1f}°C{compound_tag}"
        )

    @classmethod
    def load(cls, path: str) -> "Setup":
        """Load a setup JSON. `pressures_psi` is optional (v2 — §7.10).

        When absent, the returned Setup has an empty `pressures_psi` dict; the
        caller (typically `resolve_setup`) must fill it from the active
        compound's `PRESSURE_STATIC`.
        """
        with open(path) as f:
            raw = json.load(f)
        if not isinstance(raw, dict):
            raise ValueError(f"Setup JSON {path}: top-level must be an object")
        pressures = raw.get("pressures_psi")
        pressures_norm: dict = {}
        if pressures is not None:
            if not isinstance(pressures, dict):
                raise ValueError(f"Setup JSON {path}: pressures_psi must be an object")
            pressures_norm = {w: float(pressures[w]) for w in WHEELS if w in pressures}
            if set(pressures_norm.keys()) != set(WHEELS):
                missing = set(WHEELS) - set(pressures_norm.keys())
                raise ValueError(
                    f"Setup JSON {path}: pressures_psi missing wheels: {sorted(missing)}"
                )
        amb = float(raw.get("ambient_temp_C", DEFAULT_AMBIENT_TEMP_C))
        s = cls(
            pressures_psi=pressures_norm,
            ambient_temp_C=amb,
            source=os.path.relpath(path).replace("\\", "/"),
            name=str(raw.get("name", "default")),
            car=raw.get("car"),
            compound=raw.get("compound"),
            notes=str(raw.get("notes", "")),
            raw=raw,
        )
        # NB: do not call validate() here; pressures may be empty until the
        # caller fills them from the active compound's PRESSURE_STATIC.
        return s

    @classmethod
    def default_for_car(cls, car_data_dir, compound=None) -> "Setup":
        """Return a Setup whose pressures default to the compound's PRESSURE_STATIC.

        `car_data_dir` may be either a path (legacy single-compound caller) or
        a `Car` object (v2 caller). When `compound` is None, the car's default
        compound is used.
        """
        # Resolve car + compound. Support both (path, ...) and (car, compound)
        # calling shapes for back-compat with v2.0 single-compound code paths.
        car_obj = None
        if hasattr(car_data_dir, "compounds"):
            car_obj = car_data_dir
            data_dir = car_obj.data_dir
        else:
            data_dir = str(car_data_dir)
        if compound is None and car_obj is not None:
            compound = car_obj.default_compound
        if compound is None:
            # Legacy single-compound path (no Car object given): read tyres.ini
            # directly and use its un-suffixed [FRONT]/[REAR] PRESSURE_STATIC.
            from .car import parse_ini
            ini = parse_ini(os.path.join(data_dir, "tyres.ini"))
            if "FRONT" not in ini or "REAR" not in ini:
                raise ValueError(f"{data_dir}/tyres.ini: missing [FRONT] or [REAR] section")
            front_psi = float(ini["FRONT"].get("PRESSURE_STATIC", "30"))
            rear_psi = float(ini["REAR"].get("PRESSURE_STATIC", "30"))
            compound_name = None
        else:
            front_psi = float(compound.pressure_static_front)
            rear_psi = float(compound.pressure_static_rear)
            compound_name = compound.name
        pressures = {"FL": front_psi, "FR": front_psi, "RL": rear_psi, "RR": rear_psi}
        s = cls(
            pressures_psi=pressures,
            ambient_temp_C=DEFAULT_AMBIENT_TEMP_C,
            source="tyres.ini",
            name="tyres.ini-default",
            car=os.path.basename(os.path.normpath(data_dir)),
            compound=compound_name,
        )
        s.validate()
        return s


def parse_pressure_str(s: str) -> dict:
    """Parse "FL=31,FR=31,RL=29,RR=29" into {"FL":31.0, ...}. Partial is allowed.

    Wheels may appear in any order; unknown wheel names raise ValueError.
    """
    out: dict = {}
    if not s:
        return out
    for token in s.split(","):
        token = token.strip()
        if not token:
            continue
        if "=" not in token:
            raise ValueError(f"--pressure token '{token}' missing '=' (expected FL=31)")
        wheel, value = token.split("=", 1)
        wheel = wheel.strip().upper()
        if wheel not in WHEELS:
            raise ValueError(f"--pressure unknown wheel '{wheel}'; expected one of {WHEELS}")
        try:
            out[wheel] = float(value.strip())
        except ValueError as e:
            raise ValueError(f"--pressure '{token}': value must be numeric") from e
    return out


def resolve_compound(car, *, cli_name: str | None = None, setup=None,
                     telemetry_name: str | None = None):
    """Resolve the active `Compound` per the four-level precedence (§21.11).

    Returns `(compound, source_tag)` where `source_tag` is one of
    `"cli" | "setup" | "telemetry" | "car-default"`.

    Levels 1+2 (cli / setup) raise `ValueError` on unknown name listing
    available compounds. Level 3 (telemetry) is best-effort: unknown names
    warn-and-fall-through to the car default (caller responsibility — this
    helper just returns the car default with source="car-default").
    """
    # Level 1: CLI.
    if cli_name:
        c = car.find_compound(cli_name)
        if c is not None:
            return c, "cli"
        names = ", ".join(c.name for c in car.compounds)
        raise ValueError(
            f"--compound '{cli_name}' not found. Available compounds: {names}"
        )
    # Level 2: setup JSON.
    if setup is not None and getattr(setup, "compound", None):
        c = car.find_compound(setup.compound)
        if c is not None:
            return c, "setup"
        names = ", ".join(c.name for c in car.compounds)
        raise ValueError(
            f"setup '{setup.source}' compound '{setup.compound}' not found in car. "
            f"Available compounds: {names}"
        )
    # Level 3: telemetry (best-effort).
    if telemetry_name:
        c = car.find_compound(telemetry_name)
        if c is not None:
            return c, "telemetry"
        # fall through (caller should warn).
    # Level 4: car default.
    return car.default_compound, "car-default"


def resolve_setup(car_or_data_dir, setup_path: str | None,
                  pressure_str: str | None, ambient: float | None,
                  *, compound=None) -> Setup:
    """Centralised resolution: --pressure > --setup > compound PRESSURE_STATIC.

    `car_or_data_dir` may be either a `Car` object (v2 caller, recommended) or
    a path (legacy). When `compound` is None, the car's default compound is
    used as the cold-pressure fallback. Returns a validated Setup with
    `source` populated for the stdout log line.
    """
    # Resolve car + data_dir.
    car_obj = None
    if hasattr(car_or_data_dir, "compounds"):
        car_obj = car_or_data_dir

    if setup_path:
        base = Setup.load(setup_path)
    else:
        base = Setup(pressures_psi={}, source="tyres.ini")

    # If pressures_psi is empty (setup JSON omitted them or no setup file at all),
    # fill from the active compound's PRESSURE_STATIC.
    if not base.pressures_psi:
        default = Setup.default_for_car(
            car_obj if car_obj is not None else car_or_data_dir,
            compound,
        )
        base.pressures_psi = dict(default.pressures_psi)
        if base.compound is None and compound is not None:
            base.compound = compound.name
        if base.source in ("default", "tyres.ini"):
            base.source = default.source
    # Apply CLI overrides (per-wheel pressures + ambient).
    overrides = parse_pressure_str(pressure_str) if pressure_str else None
    return base.with_overrides(overrides, ambient)
