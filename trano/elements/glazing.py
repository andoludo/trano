"""Solar-optical and thermal properties of a glazing system, derived from its layer data.

Solar optics are a port of ``Buildings.HeatTransfer.Windows.Functions`` (Buildings 13), which
follows Finlayson et al. (1993): the angular transmittance and reflectances of each pane come
from its normal-incidence values (Fresnel model for uncoated glass, polynomial correction for
coated glass), and the panes are combined by the net radiation method. Using the same algorithm
keeps the glazing seen by IDEAS and AixLib identical to the one Buildings computes itself from
the same YAML data.

The thermal properties follow EN 673 (gas gaps with long-wave radiation and natural convection,
U-value) and EN 410 (secondary heat transfer that, added to the solar transmittance, gives the
total solar energy transmittance or g-value).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]

N_ANGLES = 10
"""Incidence angles 0, 10, ..., 90 degrees: the grid of the Buildings window model and of IDEAS glazing tables."""
INCIDENCE_ANGLES: FloatArray = np.linspace(0.0, math.pi / 2, N_ANGLES)
HEMISPHERICAL = N_ANGLES
"""Index of the hemispherical (diffuse) value, stored after the angular values."""

UNCOATED_TOLERANCE = 0.005
"""Panes whose front and back reflectances differ less than this are treated as uncoated glass (as Buildings)."""
_SMALL = 1e-60  # Modelica.Constants.small
# Polynomial coefficients of the angular correction for coated glass (Buildings, Table A.2 in Wetter's thesis).
_COATED_COEFFICIENTS = np.array(
    [
        [-0.0015, 3.355, -3.840, 1.460, 0.0288],
        [0.999, -0.563, 2.043, -2.532, 1.054],
        [-0.002, 2.813, -2.341, -0.05725, 0.599],
        [0.997, -1.868, 6.513, -7.862, 3.225],
    ]
)

STEFAN_BOLTZMANN = 5.670374419e-8  # [W/(m2.K4)]
GRAVITY = 9.81  # [m/s2]
EN673_MEAN_TEMPERATURE = 283.0  # [K]
EN673_TEMPERATURE_DIFFERENCE = 15.0  # [K] across all the gaps of the glazing
EXTERIOR_SURFACE_COEFFICIENT = 23.0  # [W/(m2.K)] EN 673 and EN 410


def interior_surface_coefficient(emissivity: float) -> float:
    """Interior surface heat transfer coefficient [W/(m2.K)] of EN 673: 3.6 + 4.4 * emissivity / 0.837."""
    return 3.6 + 4.4 * emissivity / 0.837


class GasProperties(Protocol):
    @property
    def thermal_conductivity(self) -> float: ...  # [W/(m.K)]
    @property
    def density(self) -> float: ...  # [kg/m3]
    @property
    def specific_heat_capacity(self) -> float: ...  # [J/(kg.K)]
    @property
    def viscosity_coefficients(self) -> tuple[float, float]: ...  # viscosity a + b * T [Pa.s]


class PaneProperties(Protocol):
    """Glass data of the description: solar values at normal incidence, one per state (the first is used)."""

    @property
    def thermal_conductivity(self) -> float: ...  # [W/(m.K)]
    @property
    def solar_transmittance(self) -> list[float]: ...
    @property
    def solar_reflectance_outside_facing(self) -> list[float]: ...
    @property
    def solar_reflectance_room_facing(self) -> list[float]: ...
    @property
    def infrared_absorptivity_outside_facing(self) -> float: ...
    @property
    def infrared_absorptivity_room_facing(self) -> float: ...


class Pane(Protocol):
    @property
    def thickness(self) -> float: ...  # [m]
    @property
    def material(self) -> PaneProperties: ...


class Gap(Protocol):
    @property
    def thickness(self) -> float: ...  # [m]
    @property
    def material(self) -> GasProperties: ...


@dataclass(frozen=True)
class PaneOptics:
    """Angular and hemispherical solar properties of one pane.

    Each array holds the values at :data:`INCIDENCE_ANGLES` followed by the hemispherical value.
    The front side faces the outside, the back side the room.
    """

    transmittance: FloatArray
    reflectance_front: FloatArray
    reflectance_back: FloatArray

    @classmethod
    def from_normal_incidence(
        cls, transmittance: float, reflectance_front: float, reflectance_back: float, thickness: float
    ) -> PaneOptics:
        if abs(reflectance_front - reflectance_back) < UNCOATED_TOLERANCE:
            angular = _uncoated_angular(transmittance, reflectance_front, thickness)
        else:
            angular = _coated_angular(transmittance, reflectance_front, reflectance_back)
        angular[:, 0] = (transmittance, reflectance_front, reflectance_back)
        angular[:, N_ANGLES - 1] = (0.0, 1.0, 1.0)
        integrand = 2 * angular * np.cos(INCIDENCE_ANGLES) * np.sin(INCIDENCE_ANGLES)
        hemispherical = INCIDENCE_ANGLES[1] * (integrand.sum(axis=1) - (integrand[:, 0] + integrand[:, -1]) / 2)
        values = np.column_stack([angular, hemispherical])
        return cls(transmittance=values[0], reflectance_front=values[1], reflectance_back=values[2])


def _uncoated_angular(transmittance: float, reflectance: float, thickness: float) -> FloatArray:
    """Fresnel model of an uncoated pane (Buildings ``glassPropertyUncoated``, equations 1 to 15)."""
    beta = transmittance**2 - reflectance**2 + 2 * reflectance + 1
    discriminant = beta**2 - 4 * (2 - reflectance) * reflectance
    if discriminant < 0:
        raise ValueError(
            "Glass data inconsistent: no interface reflectivity matches the transmittance and reflectance."
        )
    rho0 = 0.5 * (beta - math.sqrt(discriminant)) / (2 - reflectance)
    attenuation = (reflectance - rho0) / (rho0 * transmittance)
    if rho0 <= 0 or attenuation <= 0:
        raise ValueError(
            "Glass data inconsistent: no extinction coefficient matches the transmittance and reflectance."
        )
    extinction = -math.log(attenuation) / thickness
    refraction_index = (1 + math.sqrt(rho0)) / (1 - math.sqrt(rho0))

    angular = np.zeros((3, N_ANGLES))
    angles = INCIDENCE_ANGLES[1 : N_ANGLES - 1]
    cos_air = np.cos(angles)
    cos_glass = np.cos(np.arcsin(np.sin(angles) / refraction_index))
    path = np.exp(-extinction * thickness / cos_glass)
    transmittances, reflectances = [], []
    for rho in (
        ((refraction_index * cos_air - cos_glass) / (refraction_index * cos_air + cos_glass)) ** 2,
        ((refraction_index * cos_glass - cos_air) / (refraction_index * cos_glass + cos_air)) ** 2,
    ):  # perpendicular and parallel polarization
        tau = (1 - rho) ** 2 * path / (1 - rho**2 * path**2)
        transmittances.append(tau)
        reflectances.append(rho * (1 + tau * path))
    angular[0, 1 : N_ANGLES - 1] = 0.5 * (transmittances[0] + transmittances[1])
    angular[1, 1 : N_ANGLES - 1] = 0.5 * (reflectances[0] + reflectances[1])
    angular[2, 1 : N_ANGLES - 1] = angular[1, 1 : N_ANGLES - 1]
    return angular


def _coated_angular(transmittance: float, reflectance_front: float, reflectance_back: float) -> FloatArray:
    """Polynomial angular correction of a coated pane (Buildings ``glassPropertyCoated``, equations A.4.68-69)."""
    rows = (0, 1) if transmittance > 0.645 else (2, 3)
    cos_air = np.cos(INCIDENCE_ANGLES[1 : N_ANGLES - 1])
    powers = np.vstack([cos_air**power for power in range(5)])
    angular_transmittance = _COATED_COEFFICIENTS[rows[0]] @ powers
    angular_reflectance = _COATED_COEFFICIENTS[rows[1]] @ powers - angular_transmittance
    angular = np.zeros((3, N_ANGLES))
    angular[0, 1 : N_ANGLES - 1] = transmittance * angular_transmittance
    angular[1, 1 : N_ANGLES - 1] = reflectance_front * (1 - angular_reflectance) + angular_reflectance
    angular[2, 1 : N_ANGLES - 1] = reflectance_back * (1 - angular_reflectance) + angular_reflectance
    return angular


@dataclass(frozen=True)
class GlazingOptics:
    """Solar properties of the glazing system for irradiation from the outside.

    ``transmittance`` and each row of ``absorptances`` (one row per pane, outside first) hold the
    values at :data:`INCIDENCE_ANGLES` followed by the hemispherical value.
    """

    transmittance: FloatArray
    absorptances: FloatArray

    @classmethod
    def from_panes(cls, panes: Sequence[PaneOptics]) -> GlazingOptics:
        """Net radiation method (Buildings ``glassTRExteriorIrradiationNoShading`` and ``glassAbs...NoShading``)."""
        n = len(panes)
        # [i][j] holds the property of the stack of panes i..j: transmittance and front reflectance
        # for i <= j, back reflectance stored at [j][i] as in Buildings.
        tra: list[list[FloatArray]] = [[np.zeros(N_ANGLES + 1) for _ in range(n)] for _ in range(n)]
        ref_front: list[list[FloatArray]] = [[np.zeros(N_ANGLES + 1) for _ in range(n)] for _ in range(n)]
        ref_back: list[list[FloatArray]] = [[np.zeros(N_ANGLES + 1) for _ in range(n)] for _ in range(n)]
        for j, pane in enumerate(panes):
            tra[j][j], ref_front[j][j], ref_back[j][j] = (
                pane.transmittance,
                pane.reflectance_front,
                pane.reflectance_back,
            )
        for i in range(n - 1):
            for j in range(i + 1, n):
                denominator = 1 - ref_front[j][j] * ref_back[j - 1][i]
                opaque = denominator < _SMALL
                inverse = np.where(opaque, 0.0, 1 / np.where(opaque, 1.0, denominator))
                tra[i][j] = np.where(opaque, 0.0, inverse * tra[i][j - 1] * tra[j][j])
                ref_front[i][j] = np.where(
                    opaque, 1.0, ref_front[i][j - 1] + inverse * tra[i][j - 1] ** 2 * ref_front[j][j]
                )
                ref_back[j][i] = np.where(opaque, 1.0, ref_back[j][j] + inverse * tra[j][j] ** 2 * ref_back[j - 1][i])

        absorptances = np.zeros((n, N_ANGLES + 1))
        for j, pane in enumerate(panes):
            front = 1 - pane.transmittance - pane.reflectance_front
            back = 1 - pane.transmittance - pane.reflectance_back
            if n == 1:
                absorptances[j] = front
                continue
            absorbed = np.zeros(N_ANGLES + 1)
            if j > 0:  # radiation reaching the pane from the outside
                denominator = 1 - ref_front[j][n - 1] * ref_back[j - 1][0]
                absorbed += _safe_ratio(front * tra[0][j - 1], denominator)
            if j < n - 1:  # radiation reflected back onto the pane by the panes behind it
                denominator = 1 - ref_back[j][0] * ref_front[j + 1][n - 1]
                absorbed += _safe_ratio(back * tra[0][j] * ref_front[j + 1][n - 1], denominator)
            if j == 0:
                absorbed += front
            absorptances[j] = absorbed
        return cls(transmittance=tra[0][n - 1], absorptances=absorptances)


def _safe_ratio(numerator: FloatArray, denominator: FloatArray) -> FloatArray:
    small = denominator < _SMALL
    return np.where(small, 0.0, numerator / np.where(small, 1.0, denominator))


def gap_conductance(
    thickness: float,
    gas: GasProperties,
    emissivity_outside_pane: float,
    emissivity_inside_pane: float,
    temperature_difference: float,
) -> float:
    """Thermal conductance of a vertical gas gap [W/(m2.K)] according to EN 673.

    Radiation between the two facing glass surfaces plus gas conduction and convection with the
    Nusselt correlation Nu = 0.035 (Gr Pr)^0.38 (at least 1), evaluated at the EN 673 mean temperature.
    """
    if min(emissivity_outside_pane, emissivity_inside_pane) > 0:
        radiation = (
            4
            * STEFAN_BOLTZMANN
            * EN673_MEAN_TEMPERATURE**3
            / (1 / emissivity_outside_pane + 1 / emissivity_inside_pane - 1)
        )
    else:
        radiation = 0.0
    a_mu, b_mu = gas.viscosity_coefficients
    viscosity = a_mu + b_mu * EN673_MEAN_TEMPERATURE
    grashof = GRAVITY * thickness**3 * temperature_difference * gas.density**2 / (EN673_MEAN_TEMPERATURE * viscosity**2)
    prandtl = viscosity * gas.specific_heat_capacity / gas.thermal_conductivity
    nusselt = max(1.0, 0.035 * math.pow(grashof * prandtl, 0.38))
    return radiation + nusselt * gas.thermal_conductivity / thickness


@dataclass(frozen=True)
class GlazingProperties:
    """Solar and thermal properties of a glazing system (center of glass, no frame)."""

    optics: GlazingOptics
    pane_resistances: tuple[float, ...]  # [m2.K/W] conduction through each pane, outside first
    gap_resistances: tuple[float, ...]  # [m2.K/W] each gas gap (EN 673)
    interior_coefficient: float  # [W/(m2.K)]
    u_value_override: float | None = None  # [W/(m2.K)] given in the description instead of computed
    g_value_override: float | None = None  # [1]

    @classmethod
    def from_layers(cls, panes: Sequence[Pane], gaps: Sequence[Gap]) -> GlazingProperties:
        """Properties of the panes and of the gas gaps between them, both listed from the outside."""
        if not panes or len(gaps) != len(panes) - 1:
            raise ValueError(
                f"A glazing has one gas gap between two panes, got {len(panes)} panes and {len(gaps)} gaps."
            )
        optics = GlazingOptics.from_panes(
            [
                PaneOptics.from_normal_incidence(
                    pane.material.solar_transmittance[0],
                    pane.material.solar_reflectance_outside_facing[0],
                    pane.material.solar_reflectance_room_facing[0],
                    pane.thickness,
                )
                for pane in panes
            ]
        )
        gap_temperature_difference = EN673_TEMPERATURE_DIFFERENCE / max(len(gaps), 1)
        gap_resistances = tuple(
            1
            / gap_conductance(
                gap.thickness,
                gap.material,
                panes[index].material.infrared_absorptivity_room_facing,
                panes[index + 1].material.infrared_absorptivity_outside_facing,
                gap_temperature_difference,
            )
            for index, gap in enumerate(gaps)
        )
        return cls(
            optics=optics,
            pane_resistances=tuple(pane.thickness / pane.material.thermal_conductivity for pane in panes),
            gap_resistances=gap_resistances,
            interior_coefficient=interior_surface_coefficient(panes[-1].material.infrared_absorptivity_room_facing),
        )

    @property
    def internal_resistance(self) -> float:
        """Resistance between the outer and inner glass surfaces [m2.K/W] (no surface coefficients)."""
        return sum(self.pane_resistances) + sum(self.gap_resistances)

    @property
    def u_value(self) -> float:
        if self.u_value_override is not None:
            return self.u_value_override
        """Center-of-glass U-value [W/(m2.K)] with the EN 673 surface coefficients."""
        return 1 / (1 / EXTERIOR_SURFACE_COEFFICIENT + self.internal_resistance + 1 / self.interior_coefficient)

    @property
    def solar_transmittance(self) -> float:
        """Direct solar transmittance at normal incidence [1]."""
        return float(self.optics.transmittance[0])

    @property
    def secondary_heat_transfer(self) -> float:
        """Share of the incident solar radiation absorbed by the panes and released to the inside (EN 410) [1].

        Each pane is an isothermal node: the heat it absorbs splits between outside and inside in
        inverse proportion to the resistances towards each side.
        """
        total = 1 / self.u_value
        resistance_to_outside = 1 / EXTERIOR_SURFACE_COEFFICIENT
        released = 0.0
        for index, absorptance in enumerate(self.optics.absorptances[:, 0]):
            resistance_to_outside += self.pane_resistances[index] / 2
            released += absorptance * resistance_to_outside / total
            resistance_to_outside += self.pane_resistances[index] / 2
            if index < len(self.gap_resistances):
                resistance_to_outside += self.gap_resistances[index]
        return released

    @property
    def g_value(self) -> float:
        """Total solar energy transmittance at normal incidence (EN 410) [1]."""
        if self.g_value_override is not None:
            return self.g_value_override
        return self.solar_transmittance + self.secondary_heat_transfer

    @property
    def layer_absorptances(self) -> FloatArray:
        """Absorptances of the alternating panes and gaps (zero), angular then hemispherical, outside first."""
        absorptances = np.zeros((2 * len(self.pane_resistances) - 1, N_ANGLES + 1))
        absorptances[::2] = self.optics.absorptances
        return absorptances

    @property
    def transmittance_table(self) -> list[list[float]]:
        """Rows of incidence angle [deg] and solar transmittance (IDEAS ``SwTrans``)."""
        return [
            [round(math.degrees(angle)), _round(value)]
            for angle, value in zip(INCIDENCE_ANGLES, self.optics.transmittance[:N_ANGLES], strict=True)
        ]

    @property
    def absorptance_table(self) -> list[list[float]]:
        """Rows of incidence angle [deg] and the absorptance of each layer (IDEAS ``SwAbs``)."""
        return [
            [round(math.degrees(angle)), *(_round(value) for value in self.layer_absorptances[:, index])]
            for index, angle in enumerate(INCIDENCE_ANGLES)
        ]

    @property
    def hemispherical_transmittance(self) -> float:
        return _round(self.optics.transmittance[HEMISPHERICAL])

    @property
    def hemispherical_absorptances(self) -> list[float]:
        return [_round(value) for value in self.layer_absorptances[:, HEMISPHERICAL]]


def _round(value: float) -> float:
    return round(float(value), 4)
