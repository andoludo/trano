from typing import Type

from pydantic import BaseModel

from neosim.models.constants import Tilt
from neosim.models.elements.base import BaseElement
from neosim.models.elements.envelope.base import BaseSimpleWall, BaseWindow


class WallParameters(BaseModel):
    number: int
    surfaces: list[float]
    azimuths: list[float]
    layers: list[str]
    tilts: list[Tilt]
    type: str

    @classmethod
    def from_neighbors(
        cls, neighbors: list["BaseElement"], wall: Type["BaseSimpleWall"]
    ) -> "WallParameters":
        constructions = [
            neighbor for neighbor in neighbors if isinstance(neighbor, wall)
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
        for window in windows:
            wall = get_common_wall_properties(neighbors, window)
            surfaces.append(wall.surface)
            azimuths.append(wall.azimuth)
            layers.append(wall.construction.name)
            tilts.append(wall.tilt)
            window_layers.append(window.construction.name)
            window_width.append(window.width)
            window_height.append(window.height)
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
        )


def get_common_wall_properties(
    neighbors: list["BaseElement"], window: BaseWindow
) -> BaseSimpleWall:
    from neosim.models.elements.envelope.external_wall import ExternalWall

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
        name="temp",
        tilt=walls[0].tilt,
        azimuth=walls[0].azimuth,
        construction=walls[0].construction,
    )
