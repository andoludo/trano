import logging
import math
from math import sqrt
from typing import TYPE_CHECKING, Dict, List, Optional, Type, Union

import numpy as np
from pydantic import BaseModel, model_validator, computed_field, Field

from trano.elements.base import BaseElement
from trano.elements.construction import Construction, Glass
from trano.elements.types import Azimuth, Tilt, ContainerTypes, TILT_MAPPING
from trano.exceptions import InvalidBuildingStructureError

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class BaseWall(BaseElement):
    container_type: ContainerTypes = "envelope"

    # @computed_field  # type: ignore
    @property
    def length(self) -> int:
        if hasattr(self, "surfaces"):
            return len(self.surfaces)
        return 1


class BaseSimpleWall(BaseWall):
    surface: float | int
    azimuth: float | int
    tilt: Tilt
    construction: Construction | Glass

    def get_tilt(self, space_name: str) -> Tilt:
        return self.tilt


class BaseInternalElement(BaseSimpleWall): ...


class BaseFloorOnGround(BaseSimpleWall): ...


class BaseExternalWall(BaseSimpleWall): ...


class BaseWindow(BaseSimpleWall):
    width: Optional[float] = None
    height: Optional[float] = None

    @model_validator(mode="after")
    def width_validator(self) -> "BaseWindow":
        if self.width is None and self.height is None:
            self.width = sqrt(self.surface)
            self.height = sqrt(self.surface)
        elif self.width is not None and self.height is None:
            self.height = self.surface / self.width
        elif self.width is None and self.height is not None:
            self.width = self.surface / self.height
        elif (
            self.width is not None
            and self.height is not None
            and int(self.width * self.height) != int(self.surface)
        ):
            raise InvalidBuildingStructureError(
                f"The surface does not match width * height for {self.name}."
            )
        else:
            ...

        return self


def _get_element(
    construction_type: str,
    base_walls: list[BaseExternalWall | BaseWindow | BaseFloorOnGround],
    construction: Construction | Glass,
) -> List[Union[BaseExternalWall | BaseWindow | BaseFloorOnGround]]:
    return [
        getattr(base_wall, construction_type)
        for base_wall in base_walls
        if base_wall.construction == construction
    ]


class MergedBaseWall(BaseWall):
    surfaces: List[float | int]
    azimuths: List[float | int]
    tilts: List[Tilt]
    constructions: List[Construction | Glass]
    include_in_layout: bool = False
    component_size: float = 3

    @classmethod
    def from_base_elements(
        cls, base_walls: list[BaseExternalWall | BaseWindow | BaseFloorOnGround]
    ) -> List["MergedBaseWall"]:
        merged_walls = []
        unique_constructions = {base_wall.construction for base_wall in base_walls}

        for construction in unique_constructions:
            data: Dict[
                str,
                list[BaseExternalWall | BaseWindow | BaseFloorOnGround],
            ] = {
                "azimuth": [],
                "tilt": [],
                "name": [],
                "surface": [],
            }
            for construction_type in data:
                data[construction_type] = _get_element(
                    construction_type, base_walls, construction
                )
            merged_wall = cls(
                name=f"merged_{'_'.join(data['name'])}",  # type: ignore
                surfaces=data["surface"],
                azimuths=data["azimuth"],
                tilts=data["tilt"],
                constructions=[construction],
            )
            merged_walls.append(merged_wall)
        return sorted(merged_walls, key=lambda x: x.name)  # type: ignore #TODO: what is the issue with this!!!


class MergedBaseWindow(MergedBaseWall): ...


class MergedBaseExternalWall(MergedBaseWall): ...


class ExternalDoor(BaseExternalWall): ...


class ExternalWall(ExternalDoor): ...


class FloorOnGround(BaseFloorOnGround):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor
    include_in_layout: bool = False
    component_size: float = 3


class SpaceTilt(BaseModel):
    space_name: str
    tilt: Optional[Tilt] = None


class InternalElement(BaseInternalElement):
    space_tilts: List[SpaceTilt] = Field(default_factory=list)

    def get_tilt(self, space_name: str) -> Tilt:
        for space_tilt in self.space_tilts:
            if space_tilt.tilt and space_tilt.space_name == space_name:
                return space_tilt.tilt
        return self.tilt


class MergedFloor(MergedBaseWall): ...


class MergedExternalWall(MergedBaseExternalWall): ...


class MergedWindows(MergedBaseWindow):
    widths: List[float | int]
    heights: List[float | int]

    @classmethod
    def from_base_windows(cls, base_walls: List["BaseWindow"]) -> List["MergedWindows"]:
        merged_windows = []
        unique_constructions = {base_wall.construction for base_wall in base_walls}

        for construction in unique_constructions:
            data: Dict[
                str, List[Union["ExternalWall", "FloorOnGround", "BaseWindow", str]]
            ] = {
                "azimuth": [],
                "tilt": [],
                "name": [],
                "surface": [],
                "width": [],
                "height": [],
            }
            for construction_type in data:
                data[construction_type] = _get_element(
                    construction_type, base_walls, construction  # type: ignore
                )
            merged_window = cls(
                name=f"merged_{'_'.join(data['name'])}",  # type: ignore
                surfaces=data["surface"],
                azimuths=data["azimuth"],
                tilts=data["tilt"],
                constructions=[construction],
                heights=data["height"],
                widths=data["width"],
            )
            merged_windows.append(merged_window)
        return sorted(merged_windows, key=lambda x: x.name)  # type: ignore


class Window(BaseWindow): ...


class WindowedWall(BaseSimpleWall): ...


def parallel_resistance(resistances: list[float]) -> float:
    return 1 / sum([1 / resistance for resistance in resistances])


class WallParameters(BaseModel):
    number: int
    surfaces: list[float]
    azimuths: list[float]
    u_values: list[float]
    layers: list[str]
    tilts: list[Tilt]
    window_area_by_orientation: list[float]
    type: str
    average_resistance_external: float = Field(default=0)
    average_resistance_external_remaining: float = Field(default=0)
    total_thermal_capacitance: float = Field(default=0)
    total_thermal_resistance: float = Field(default=0)

    def azimuths_to_radians(self) -> list[float]:
        return [math.radians(azimuth) for azimuth in self.azimuths]

    def tilts_to_radians(self) -> list[float]:
        return [math.radians(TILT_MAPPING[tilt.value]) for tilt in self.tilts]

    @computed_field
    def average_u_value(self) -> float:
        if not self.u_values:
            return 0
        return sum(self.u_values) / len(self.u_values)

    @classmethod
    def from_neighbors(  # noqa: PLR0913
        cls,
        space_name: str,
        neighbors: list["BaseElement"],
        wall: Type["BaseSimpleWall"],
        filter: Optional[list[str]] = None,
        suffix_type: Optional[str] = None,
    ) -> "WallParameters":
        constructions = [
            neighbor
            for neighbor in neighbors
            if isinstance(neighbor, wall)
            if neighbor.name not in (filter or [])
        ]
        window_area_by_orientation = []
        number = len(constructions)
        surfaces = [
            exterior_construction.surface for exterior_construction in constructions
        ]
        total_surface = sum(surfaces)
        azimuths = [
            exterior_construction.azimuth for exterior_construction in constructions
        ]
        layers = [
            exterior_construction.construction.name
            for exterior_construction in constructions
        ]
        u_values = [
            exterior_construction.construction.u_value
            for exterior_construction in constructions
        ]
        average_resistance_external = np.mean(
            [
                exterior_construction.construction.resistance_external
                for exterior_construction in constructions
            ]
        )
        average_resistance_external_remaining = np.mean(
            [
                exterior_construction.construction.resistance_external_remaining
                for exterior_construction in constructions
            ]
        )
        total_thermal_capacitance = total_surface * np.mean(
            [
                exterior_construction.construction.total_thermal_capacitance
                for exterior_construction in constructions
            ]
        )
        total_thermal_resistance = total_surface * np.mean(
            [
                exterior_construction.construction.total_thermal_resistance
                for exterior_construction in constructions
            ]
        )

        tilt = [
            exterior_construction.get_tilt(space_name)
            for exterior_construction in constructions
        ]
        type = wall.__name__ if not suffix_type else f"{wall.__name__}{suffix_type}"
        if issubclass(wall, BaseWindow):
            external_walls = [
                neighbor for neighbor in neighbors if isinstance(neighbor, ExternalWall)
            ]
            azimuth_surface = {
                construction.azimuth: construction.surface
                for construction in constructions
            }
            window_area_by_orientation = [
                (azimuth_surface.get(exterior_construction.azimuth, 0))
                for exterior_construction in external_walls
            ]
        return cls(
            number=number,
            surfaces=surfaces,
            azimuths=azimuths,
            layers=layers,
            tilts=tilt,
            type=type,
            u_values=u_values,
            window_area_by_orientation=window_area_by_orientation,
            average_resistance_external=average_resistance_external,
            average_resistance_external_remaining=average_resistance_external_remaining,
            total_thermal_capacitance=total_thermal_capacitance,
            total_thermal_resistance=total_thermal_resistance,
        )


class VerticalWallParameters(WallParameters):
    @classmethod
    def from_neighbors_(
        cls,
        space_name: str,
        neighbors: list["BaseElement"],
        wall: Type["BaseSimpleWall"],
    ) -> "VerticalWallParameters":
        neighbors = [n for n in neighbors if hasattr(n, "tilt") and n.tilt == Tilt.wall]
        return cls.from_neighbors(space_name, neighbors, wall, suffix_type="VerticalOnly")  # type: ignore


class RoofWallParameters(WallParameters):
    @classmethod
    def from_neighbors_(
        cls,
        space_name: str,
        neighbors: list["BaseElement"],
        wall: Type["BaseSimpleWall"],
    ) -> "RoofWallParameters":
        neighbors = [
            n for n in neighbors if hasattr(n, "tilt") and n.tilt == Tilt.ceiling
        ]
        return cls.from_neighbors(space_name, neighbors, wall, suffix_type="Roof")  # type: ignore


class ExternalWallParameters(WallParameters):
    @classmethod
    def from_neighbors_(
        cls,
        space_name: str,
        neighbors: list["BaseElement"],
        wall: Type["BaseSimpleWall"],
    ) -> "RoofWallParameters":
        return cls.from_neighbors(space_name, neighbors, wall, suffix_type="External")  # type: ignore


class WindowedWallParameters(WallParameters):
    window_layers: list[str]
    window_width: list[float]
    window_height: list[float]
    included_external_walls: list[str]

    @classmethod
    def from_neighbors(cls, neighbors: list["BaseElement"]) -> "WindowedWallParameters":  # type: ignore

        windows = [
            neighbor for neighbor in neighbors if isinstance(neighbor, BaseWindow)
        ]
        surfaces = []
        azimuths = []
        layers = []
        tilts = []
        window_layers = []
        window_width = []
        window_height = []
        included_external_walls = []
        u_values = []
        window_area_by_orientation: List[float] = []
        for window in windows:
            wall = get_common_wall_properties(neighbors, window)
            surfaces.append(wall.surface)
            azimuths.append(wall.azimuth)
            layers.append(wall.construction.name)
            tilts.append(wall.tilt)
            window_layers.append(window.construction.name)
            window_width.append(window.width)
            window_height.append(window.height)
            included_external_walls.append(wall.name)
            u_values.append(wall.construction.u_value)
        return cls(
            number=len(windows),
            surfaces=surfaces,
            azimuths=azimuths,
            layers=layers,
            u_values=u_values,
            tilts=tilts,
            type="WindowedWall",
            window_layers=window_layers,
            window_width=window_width,
            window_height=window_height,
            included_external_walls=included_external_walls,
            window_area_by_orientation=window_area_by_orientation,
        )


def get_common_wall_properties(
    neighbors: list["BaseElement"], window: BaseWindow
) -> BaseSimpleWall:
    walls = [
        neighbor
        for neighbor in neighbors
        if isinstance(neighbor, ExternalWall)
        and neighbor.azimuth == window.azimuth
        and Tilt.wall == neighbor.tilt
    ]
    similar_properties = (
        len({w.azimuth for w in walls}) == 1
        and len({w.tilt for w in walls}) == 1
        and len({w.construction.name for w in walls}) == 1
    )

    if not similar_properties:
        logger.warning(
            "The walls have different properties for the same azimuth. Using the one with the marges area."
        )
        walls = sorted(walls, key=lambda w: w.surface, reverse=True)[:1]
    if not walls:
        raise InvalidBuildingStructureError(
            "No walls found with the same azimuth and tilt as the window."
        )

    return BaseSimpleWall(
        surface=sum([w.surface for w in walls]),
        name=walls[0].name,
        tilt=walls[0].tilt,
        azimuth=walls[0].azimuth,
        construction=walls[0].construction,
    )
