from typing import Optional, Type

from pydantic import BaseModel

from trano.models.constants import Tilt
from trano.models.elements.base import BaseElement
from trano.models.elements.envelope.envelope import BaseSimpleWall, BaseWindow, ExternalWall


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
            wall = get_common_wall_properties(neighbors, window)
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
        raise NotImplementedError
    return BaseSimpleWall(
        surface=sum([w.surface for w in walls]),
        name=walls[0].name,
        tilt=walls[0].tilt,
        azimuth=walls[0].azimuth,
        construction=walls[0].construction,
    )
