"""
Atmosphere model: humidity-corrected air density and speed of sound.

Humid air is LESS dense than dry air at the same pressure and temperature,
because water vapor (M = 18.02 g/mol) is lighter than the dry-air average
(28.9647 g/mol). Ignoring humidity therefore biases drop/drift high in hot,
humid conditions. We compute vapor pressure with the Buck (1996) equation and
mix partial pressures the CIPM way.
"""

import math
from dataclasses import dataclass

# Molar masses (kg/mol) and universal gas constant (J/mol/K)
M_DRY = 0.0289647
M_VAPOR = 0.0180160
R_UNIV = 8.314462618

# ICAO sea-level standard, used as the reference for stability corrections.
STD_TEMP_C = 15.0
STD_PRESSURE_PA = 101325.0
STD_RH = 0.0


def saturation_vapor_pressure_pa(temp_c: float) -> float:
    """Buck (1996) saturation vapor pressure over liquid water, in pascals."""
    return 100.0 * 6.1121 * math.exp(
        (18.678 - temp_c / 234.5) * (temp_c / (257.14 + temp_c))
    )


def air_density(temp_c: float, pressure_pa: float, humidity: float = 0.0) -> float:
    """
    Moist-air mass density (kg/m^3).

    temp_c       : ambient temperature, Celsius
    pressure_pa  : ABSOLUTE (station) pressure, pascals -- not sea-level-corrected
    humidity     : relative humidity as a fraction 0..1
    """
    p_v = humidity * saturation_vapor_pressure_pa(temp_c)
    p_d = pressure_pa - p_v
    t_k = temp_c + 273.15
    return (p_d * M_DRY + p_v * M_VAPOR) / (R_UNIV * t_k)


def speed_of_sound(temp_c: float, pressure_pa: float, humidity: float = 0.0) -> float:
    """
    Speed of sound in moist air (m/s).

    Humidity raises c slightly (lighter mixture); the effect is a few tenths of a
    percent but shifts the Mach number a bullet sees near transonic, so we keep it.
    """
    p_v = humidity * saturation_vapor_pressure_pa(temp_c)
    x_v = p_v / pressure_pa                      # mole fraction water vapor
    m_mix = (1.0 - x_v) * M_DRY + x_v * M_VAPOR
    # Ratio of specific heats: mole-fraction blend of dry air (1.400) and vapor (1.330)
    gamma = (1.0 - x_v) * 1.400 + x_v * 1.330
    t_k = temp_c + 273.15
    return math.sqrt(gamma * R_UNIV * t_k / m_mix)


def standard_density() -> float:
    return air_density(STD_TEMP_C, STD_PRESSURE_PA, STD_RH)


@dataclass
class Atmosphere:
    temp_c: float = 15.0
    pressure_pa: float = 101325.0
    humidity: float = 0.0          # fraction 0..1

    @property
    def density(self) -> float:
        return air_density(self.temp_c, self.pressure_pa, self.humidity)

    @property
    def mach1(self) -> float:
        return speed_of_sound(self.temp_c, self.pressure_pa, self.humidity)
