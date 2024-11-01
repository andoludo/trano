from math import sqrt
from typing import TYPE_CHECKING, Dict, List, Optional, Type, Union

from pydantic import BaseModel, model_validator

from trano.elements.base import BaseElement
from trano.elements.construction import Construction, Glass
from trano.elements.types import Azimuth, Tilt

if TYPE_CHECKING:
    pass


class BaseWall(BaseElement):

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


class BaseInternalElement(BaseSimpleWall):
    ...


class BaseFloorOnGround(BaseSimpleWall):
    ...


class BaseExternalWall(BaseSimpleWall):
    ...


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


class MergedBaseWindow(MergedBaseWall):
    ...


class MergedBaseExternalWall(MergedBaseWall):
    ...


class ExternalDoor(BaseExternalWall):
    ...


class ExternalWall(ExternalDoor):
    ...


class FloorOnGround(BaseFloorOnGround):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor


class InternalElement(BaseInternalElement):
    ...


class MergedFloor(MergedBaseWall):
    ...


class MergedExternalWall(MergedBaseExternalWall):
    ...


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


class Window(BaseWindow):
    ...


class WindowedWall(BaseSimpleWall):
    ...


class WallParameters(BaseModel):
    number: int
    surfaces: list[float]
    azimuths: list[float]
    layers: list[str]
    tilts: list[Tilt]
    type: str

    @classmethod
    def from_neighbors(
        cls,
        neighbors: list["BaseElement"],
        wall: Type["BaseSimpleWall"],
        filter: Optional[list[str]] = None,
    ) -> "WallParameters":
        constructions = [
            neighbor
            for neighbor in neighbors
            if isinstance(neighbor, wall)
            if neighbor.name not in (filter or [])
        ]
        number = len(constructions)
        surfaces = [
            exterior_construction.surface for exterior_construction in constructions
        ]
        azimuths = [
            exterior_construction.azimuth for exterior_construction in constructions
        ]
        layers = [
            exterior_construction.construction.name
            for exterior_construction in constructions
        ]
        tilt = [exterior_construction.tilt for exterior_construction in constructions]
        type = wall.__name__
        return cls(
            number=number,
            surfaces=surfaces,
            azimuths=azimuths,
            layers=layers,
            tilts=tilt,
            type=type,
        )


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
        for window in windows:
            wall = get_Trano_wall_properties(neighbors, window)
            surfaces.append(wall.surface)
            azimuths.append(wall.azimuth)
            layers.append(wall.construction.name)
            tilts.append(wall.tilt)
            window_layers.append(window.construction.name)
            window_width.append(window.width)
            window_height.append(window.height)
            included_external_walls.append(wall.name)
        return cls(
            number=len(windows),
            surfaces=surfaces,
            azimuths=azimuths,
            layers=layers,
            tilts=tilts,
            type="WindowedWall",
            window_layers=window_layers,
            window_width=window_width,
            window_height=window_height,
            included_external_walls=included_external_walls,
        )


def get_Trano_wall_properties(
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
        raise NotImplementedError
    return BaseSimpleWall(
        surface=sum([w.surface for w in walls]),
        name=walls[0].name,
        tilt=walls[0].tilt,
        azimuth=walls[0].azimuth,
        construction=walls[0].construction,
    )
