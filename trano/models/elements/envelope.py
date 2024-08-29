from math import sqrt
from typing import TYPE_CHECKING, Dict, List, Optional, Union

from pydantic import model_validator

from trano.construction import Construction
from trano.glass import Glass
from trano.models.constants import Azimuth, Tilt
from trano.models.elements.base import BaseElement

if TYPE_CHECKING:
    pass


class BaseWall(BaseElement):
    ...

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
