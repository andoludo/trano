from typing import TYPE_CHECKING, Dict, List, Union

from pydantic import computed_field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.models.constants import Tilt
from neosim.models.elements.base import BaseElement

if TYPE_CHECKING:
    from neosim.models.elements.envelope.external_wall import ExternalWall
    from neosim.models.elements.envelope.floor_on_ground import FloorOnGround
    from neosim.models.elements.envelope.window import Window


class BaseWall(BaseElement):
    ...

    @computed_field  # type: ignore
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
    width: float | int
    height: float | int


def _get_element(
    construction_type: str,
    base_walls: List[Union["ExternalWall", "FloorOnGround", "Window"]],
    construction: Construction | Glass,
) -> List[Union["ExternalWall", "FloorOnGround", "Window"]]:
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
        cls, base_walls: List[Union["ExternalWall", "FloorOnGround", "Window"]]
    ) -> List["MergedBaseWall"]:
        merged_walls = []
        unique_constructions = {base_wall.construction for base_wall in base_walls}

        for construction in unique_constructions:
            data: Dict[
                str,
                Union[
                    List[Union["ExternalWall", "FloorOnGround", "Window"]] | List[str]
                ],
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
        return sorted(merged_walls, key=lambda x: x.name)


class MergedBaseWindow(MergedBaseWall):
    ...


class MergedBaseExternalWall(MergedBaseWall):
    ...
