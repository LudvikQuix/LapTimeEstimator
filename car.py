"""Parse Assetto Corsa car data into a physics model."""
import configparser
import os
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

        # Tyres (use first compound - Street)
        tyres = parse_ini(os.path.join(d, 'tyres.ini'))
        self.tyre_radius_f = float(tyres['FRONT']['RADIUS'])
        self.tyre_radius_r = float(tyres['REAR']['RADIUS'])
        self.tyre_dy0_f = float(tyres['FRONT']['DY0'])
        self.tyre_dy0_r = float(tyres['REAR']['DY0'])
        self.tyre_dx0_f = float(tyres['FRONT']['DX0'])
        self.tyre_dx0_r = float(tyres['REAR']['DX0'])
        self.tyre_speed_sens_f = float(tyres['FRONT']['SPEED_SENSITIVITY'])
        self.tyre_speed_sens_r = float(tyres['REAR']['SPEED_SENSITIVITY'])

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
