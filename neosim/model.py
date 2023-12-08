from enum import Enum
from typing import Union, Optional

from networkx import Graph
from pydantic import BaseModel
from pydantic import BaseModel, Field, field_validator

from neosim.construction import Construction
from neosim.glass import Glass
class Tilt(Enum):
    wall = "wall"
    ceiling = "ceiling"
    floor = "floor"
class Azimuth(Enum):
    north = 0
    south = 90
    east = 45
    west = 135

class WallParameters(BaseModel):
    number: int
    surfaces: list[float]
    azimuths: list[float]
    layers: list[str]
    tilts: list[Tilt]
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
            exterior_construction.construction.name for exterior_construction in constructions
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
            if len(walls) != 1:
                raise not NotImplementedError
            wall = walls[0]
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


class Space(BaseModel):
    name: str
    volume: float | int
    height: float | int
    floor_area: float | int
    elevation: float | int
    external_boundaries: list[Union["ExternalWall", "Window", "FloorOnGround"]]
    internal_elements: list["ExternalWall"] = None
    boundaries: list[WallParameters] = None
    type: str = "space" # TODO: to replace
    position: Optional[list] = None


    def get_neighhors(self, graph: Graph) -> None:
        neighbors = list(graph.neighbors(self))
        self.boundaries = []
        for wall in [ExternalWall, Window, InternalElement, FloorOnGround]:
            self.boundaries.append(WallParameters.from_neighbors(neighbors, wall))
        self.boundaries += [WindowedWallParameters.from_neighbors(neighbors)]

    def __hash__(self):
        return hash(f"{self.name}-{self.volume}")

    def __add__(self, other: "Space") -> "Space":
        self.name = (
            f"merge_{self.name.replace('merge','')}_{other.name.replace('merge','')}"
        )
        self.volume = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self
    def get_position(self, layout):
        if not self.position:
            self.position = list(layout.get(self))


class System(BaseModel):
    name: str
    position: Optional[list] = None
    def get_position(self, layout):
        self.position = list(layout.get(self))
    @property
    def type(self):
        return type(self).__name__

    def __hash__(self):
        return hash(f"{self.name}-{type(self).__name__}")

class Occupancy(System):
    ...

class Weather(System):
    ...
class BaseWall(BaseModel):
    name: str
    surface: float | int
    azimuth: float | int
    tilt: Tilt
    construction: Construction | Glass
    position: Optional[list] = None
    @field_validator('construction', mode="before")
    @classmethod
    def _construction(cls,construction):
      return construction.value

    def __hash__(self):
        return hash(f"{self.name}-{self.surface}-{type(self).__name__}")

    def get_position(self, layout):
        if not self.position:
            self.position = list(layout.get(self))


    @property
    def type(self):
        return type(self).__name__


class ExternalWall(BaseWall):
    ...

class FloorOnGround(BaseWall):
    azimuth: float | int = Azimuth.south.value
    tilt: Tilt = Tilt.floor


class Window(BaseWall):
    width: float | int
    height: float | int


class WindowedWall(BaseWall):
    ...


class InternalElement(BaseWall):
    ...



