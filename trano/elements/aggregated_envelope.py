"""Envelope of a space aggregated per orientation and lumped into RC elements.

The AixLib reduced-order (VDI 6007) and ISO 13790 zones do not model each wall: they take the
opaque and window areas per orientation, lumped resistances [K/W] and capacitances [J/K] for the
exterior walls, the roof and the floor, U-values, and the weighting factors of the equivalent
outdoor temperature. This module derives them from the external boundaries of a space.

Each group of opaque elements is lumped into one 2R1C element: the capacitance sits between two
equal halves of the conduction resistance, the elements of a group are in parallel. The surface
heat transfer coefficients are not part of these resistances, the zone models add their own.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

from pydantic import BaseModel

from trano.elements.construction import Construction, Glass
from trano.elements.envelope import (
    GROUND_TEMPERATURE,
    BaseExternalWall,
    BaseFloorOnGround,
    BaseSimpleWall,
    BaseWindow,
    same_angle,
)
from trano.elements.types import TILT_MAPPING, Tilt

# Surface resistances of EN ISO 6946 [m2.K/W]. AixLib's defaults match them: 1/(2.7 + 5) inside, 1/(20 + 5) outside.
INTERIOR_RESISTANCE_WALL = 0.13
INTERIOR_RESISTANCE_ROOF = 0.10
INTERIOR_RESISTANCE_FLOOR = 0.17
EXTERIOR_RESISTANCE = 0.04

ROOF_TILTS = frozenset(
    {
        Tilt.ceiling,
        Tilt.pitched_roof_45,
        Tilt.pitched_roof_40,
        Tilt.pitched_roof_35,
        Tilt.pitched_roof_30,
        Tilt.pitched_roof_20,
    }
)
SIGNIFICANT_DIGITS = 6

# Values rendered for an empty group, where AixLib still requires a valid (unused) parameter.
EMPTY_RESISTANCE = 0.001  # [K/W]
EMPTY_CAPACITANCE = 10000.0  # [J/K]


def significant(value: float, digits: int = SIGNIFICANT_DIGITS) -> float:
    """Round to significant digits: resistances in K/W of large envelopes are well below 1e-3."""
    return float(f"{value:.{digits}g}")


def is_roof(element: BaseSimpleWall) -> bool:
    return isinstance(element, BaseExternalWall) and element.tilt in ROOF_TILTS


def tilt_radians(tilt: Tilt) -> float:
    return math.radians(TILT_MAPPING[tilt.value])


def _resistance(construction: Construction | Glass) -> float:
    """Conduction resistance [m2.K/W]: layers for opaque constructions, EN 673 glazing for windows."""
    if isinstance(construction, Glass):
        return construction.properties.internal_resistance
    return max(construction.total_thermal_resistance, 1e-6)


def _u_value(element: BaseSimpleWall, interior_resistance: float, exterior_resistance: float) -> float:
    """U-value [W/(m2.K)] with surface resistances (EN 673 for glazing, EN ISO 6946 otherwise)."""
    if isinstance(element.construction, Glass):
        return element.construction.properties.u_value
    return 1 / (interior_resistance + _resistance(element.construction) + exterior_resistance)


class Orientation(BaseModel):
    """Opaque and window surfaces facing one direction."""

    azimuth: float  # [rad]
    tilt: float  # [rad]
    opaque_area: float = 0.0  # [m2]
    window_area: float = 0.0  # [m2]
    opaque_conductance: float = 0.0  # [W/K] U-value times area, with surface resistances
    window_conductance: float = 0.0  # [W/K]

    def matches(self, element: BaseSimpleWall) -> bool:
        return same_angle(self.azimuth, element.azimuth) and math.isclose(
            self.tilt, tilt_radians(element.tilt), abs_tol=1e-9
        )


def group_by_orientation(
    opaque: Iterable[BaseSimpleWall], windows: Iterable[BaseSimpleWall], interior_resistance: float
) -> list[Orientation]:
    """Orientations of the opaque elements and windows, sorted by tilt then azimuth."""
    orientations: list[Orientation] = []
    for element, is_window in [(element, False) for element in opaque] + [(window, True) for window in windows]:
        orientation = next((orientation for orientation in orientations if orientation.matches(element)), None)
        if orientation is None:
            orientation = Orientation(azimuth=element.azimuth, tilt=tilt_radians(element.tilt))
            orientations.append(orientation)
        if is_window:
            orientation.window_area += element.surface
            orientation.window_conductance += element.surface * _u_value(
                element, interior_resistance, EXTERIOR_RESISTANCE
            )
        else:
            orientation.opaque_area += element.opaque_surface
            orientation.opaque_conductance += element.opaque_surface * _u_value(
                element, interior_resistance, EXTERIOR_RESISTANCE
            )
    return sorted(orientations, key=lambda orientation: (orientation.tilt, orientation.azimuth))


def weighting_factors(conductances: Sequence[float]) -> list[float]:
    """Shares of each orientation in the equivalent outdoor temperature (VDI 6007): they sum to 1.

    Without any conductance the factors are irrelevant (AixLib then disables the element); they
    are spread evenly so that the sum still equals 1.
    """
    total = sum(conductances)
    if total <= 0:
        return [significant(1 / len(conductances))] * len(conductances)
    return [significant(conductance / total) for conductance in conductances]


class LumpedElement(BaseModel):
    """Opaque elements lumped into one element: two equal resistances around one capacitance.

    Walls count without the windows cut out of them (``opaque_surface``).
    """

    area: float = 0.0  # [m2]
    resistance: float = EMPTY_RESISTANCE  # [K/W] interior surface to capacitance
    resistance_remaining: float = EMPTY_RESISTANCE  # [K/W] capacitance to exterior surface
    capacitance: float = EMPTY_CAPACITANCE  # [J/K]
    u_value: float = 0.0  # [W/(m2.K)] area-weighted, with surface resistances

    @classmethod
    def from_elements(
        cls, elements: Sequence[BaseSimpleWall], interior_resistance: float, exterior_resistance: float
    ) -> LumpedElement:
        area = sum(element.opaque_surface for element in elements)
        if area <= 0:
            return cls()
        # Each element's half resistance is r/2 per square metre: in parallel, 1/R = sum(A / (r/2)).
        half_conductance = sum(element.opaque_surface / (_resistance(element.construction) / 2) for element in elements)
        capacitance = sum(
            element.opaque_surface * element.construction.total_thermal_capacitance for element in elements
        )
        conductance = sum(
            element.opaque_surface * _u_value(element, interior_resistance, exterior_resistance) for element in elements
        )
        return cls(
            area=significant(area),
            resistance=significant(1 / half_conductance),
            resistance_remaining=significant(1 / half_conductance),
            capacitance=significant(capacitance),
            u_value=significant(conductance / area),
        )


class WindowGroup(BaseModel):
    """Windows lumped into one element."""

    area: float = 0.0  # [m2]
    resistance: float = EMPTY_RESISTANCE  # [K/W] between the inner and outer glass surfaces
    u_value: float = 0.0  # [W/(m2.K)] area-weighted, EN 673
    g_value: float = 0.0  # [1] area-weighted total solar energy transmittance, EN 410

    @classmethod
    def from_windows(cls, windows: Sequence[BaseSimpleWall]) -> WindowGroup:
        area = sum(window.surface for window in windows)
        if area <= 0:
            return cls()
        glazings = [window.construction.properties for window in windows]  # type: ignore[union-attr]
        return cls(
            area=significant(area),
            resistance=significant(
                1
                / sum(
                    window.surface / glazing.internal_resistance
                    for window, glazing in zip(windows, glazings, strict=True)
                )
            ),
            u_value=significant(
                sum(window.surface * glazing.u_value for window, glazing in zip(windows, glazings, strict=True)) / area
            ),
            g_value=significant(
                sum(window.surface * glazing.g_value for window, glazing in zip(windows, glazings, strict=True)) / area
            ),
        )


class AggregatedEnvelope(BaseModel):
    """Envelope of a space as seen by the AixLib reduced-order and ISO 13790 zones.

    ``orientations`` cover the exterior walls (everything opaque but roofs and floors on ground)
    and all windows, ``roof_orientations`` the roofs (flat and pitched).
    """

    orientations: list[Orientation]
    roof_orientations: list[Orientation]
    exterior_walls: LumpedElement
    roof: LumpedElement
    floor: LumpedElement
    windows: WindowGroup
    ground_temperature: float = GROUND_TEMPERATURE  # [K]

    @classmethod
    def from_boundaries(cls, boundaries: Sequence[BaseSimpleWall]) -> AggregatedEnvelope:
        windows = [boundary for boundary in boundaries if isinstance(boundary, BaseWindow)]
        floors = [boundary for boundary in boundaries if isinstance(boundary, BaseFloorOnGround)]
        roofs = [boundary for boundary in boundaries if is_roof(boundary)]
        walls = [
            boundary for boundary in boundaries if isinstance(boundary, BaseExternalWall) and not is_roof(boundary)
        ]
        orientations = group_by_orientation(walls, windows, INTERIOR_RESISTANCE_WALL) or [
            Orientation(azimuth=0.0, tilt=tilt_radians(Tilt.wall))
        ]
        roof_orientations = group_by_orientation(roofs, [], INTERIOR_RESISTANCE_ROOF) or [
            Orientation(azimuth=0.0, tilt=tilt_radians(Tilt.ceiling))
        ]
        floor_area = sum(floor.surface for floor in floors)
        ground_temperature = (
            sum(floor.surface * floor.ground_temperature for floor in floors) / floor_area
            if floor_area > 0
            else GROUND_TEMPERATURE
        )
        return cls(
            orientations=orientations,
            roof_orientations=roof_orientations,
            exterior_walls=LumpedElement.from_elements(walls, INTERIOR_RESISTANCE_WALL, EXTERIOR_RESISTANCE),
            roof=LumpedElement.from_elements(roofs, INTERIOR_RESISTANCE_ROOF, EXTERIOR_RESISTANCE),
            # Floors on ground: no exterior surface resistance, the ground is in contact with the slab.
            floor=LumpedElement.from_elements(floors, INTERIOR_RESISTANCE_FLOOR, 0.0),
            windows=WindowGroup.from_windows(windows),
            ground_temperature=significant(ground_temperature),
        )

    def floor_u_value(self, conditioned_floor_area: float) -> float:
        """Floor U-value [W/(m2.K)] for zones that multiply it by their conditioned floor area.

        The ISO 13790 zone computes its ground conductance as ``UFlo * AFlo``. Scaling the U-value of
        the floors on ground by their share of the conditioned floor area keeps the conductance of
        the description, also for zones over several storeys or above another zone (no ground loss).
        """
        if conditioned_floor_area <= 0:
            return self.floor.u_value
        return significant(self.floor.u_value * self.floor.area / conditioned_floor_area)

    @property
    def opaque_areas(self) -> list[float]:
        return [significant(orientation.opaque_area) for orientation in self.orientations]

    @property
    def window_areas(self) -> list[float]:
        return [significant(orientation.window_area) for orientation in self.orientations]

    @property
    def azimuths(self) -> list[float]:
        return [significant(orientation.azimuth) for orientation in self.orientations]

    @property
    def tilts(self) -> list[float]:
        return [significant(orientation.tilt) for orientation in self.orientations]

    @property
    def wall_weighting_factors(self) -> list[float]:
        return weighting_factors([orientation.opaque_conductance for orientation in self.orientations])

    @property
    def window_weighting_factors(self) -> list[float]:
        return weighting_factors([orientation.window_conductance for orientation in self.orientations])

    @property
    def roof_azimuths(self) -> list[float]:
        return [significant(orientation.azimuth) for orientation in self.roof_orientations]

    @property
    def roof_tilts(self) -> list[float]:
        return [significant(orientation.tilt) for orientation in self.roof_orientations]

    @property
    def roof_weighting_factors(self) -> list[float]:
        return weighting_factors([orientation.opaque_conductance for orientation in self.roof_orientations])
