from enum import Enum
from typing import Union

from networkx import Graph
from pydantic import BaseModel


class WallParameters(BaseModel):
    number: int
    surfaces: list[float]
    azimuths: list[float]
    layers: list[str]
    tilts: list[float]
    type: str

    @classmethod
    def from_neighbors(cls, neighbors, wall):
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
            exterior_construction.layer_name for exterior_construction in constructions
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
    def from_neighbors(cls, neighbors):
        windows = [neighbor for neighbor in neighbors if isinstance(neighbor, Window)]
        surfaces = []
        azimuths = []
        layers = []
        tilts = []
        window_layers = []
        window_width = []
        window_height = []
        for window in windows:
            walls = [
                neighbor
                for neighbor in neighbors
                if isinstance(neighbor, ExternalWall)
                and neighbor.azimuth == window.azimuth
            ]
            if len(walls) !=1:
                raise not NotImplementedError
            wall = walls[0]
            surfaces.append(wall.surface)
            azimuths.append(wall.azimuth)
            layers.append(wall.layer_name)
            tilts.append(wall.tilt)
            window_layers.append(window.layer_name)
            window_width.append(window.width)
            window_height.append(window.height)
        return cls(
            number=len(windows),
            surfaces=surfaces,
            azimuths=azimuths,
            layers=layers,
            tilts=tilts,
            type="WindowedWall",
            window_layers = window_layers,
            window_width = window_width,
            window_height = window_height
        )


class Space(BaseModel):
    name: str
    volume: float | int
    height: float | int
    elevation: float | int
    external_boundaries: list[Union["ExternalWall", "Window"]]
    internal_elements: list["ExternalWall"] = None
    boundaries: list[WallParameters] = None

    def get_neighhors(self, graph: Graph) -> None:
        neighbors = list(graph.neighbors(self))
        self.boundaries = []
        for wall in [ExternalWall, Window, InternalElement]:
            self.boundaries.append(WallParameters.from_neighbors(neighbors, wall))
        self.boundaries +=[WindowedWallParameters.from_neighbors(neighbors)]

    def __hash__(self):
        return hash(f"{self.name}-{self.volume}")

    def __add__(self, other: "Space") -> "Space":
        self.name = (
            f"merge_{self.name.replace('merge','')}_{other.name.replace('merge','')}"
        )
        self.volume = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self


class BaseWall(BaseModel):
    name: str
    surface: float | int
    azimuth: float | int
    tilt: float | int
    layer_name: str

    def __hash__(self):
        return hash(f"{self.name}-{self.surface}")


class ExternalWall(BaseWall):
    ...


class Window(BaseWall):
    width: float | int
    height: float | int


class WindowedWall(BaseWall):
    ...


class InternalElement(BaseWall):
    ...


class Azimuth(Enum):
    north = 0
    south = 90
    east = 45
    west = 135
