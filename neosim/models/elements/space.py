from typing import List, Optional, Union

from networkx import Graph
from pydantic import Field, computed_field

from neosim.models.elements.base import BaseElement
from neosim.models.elements.control import SpaceControl
from neosim.models.elements.merged_wall import (
    MergedBaseWall,
    MergedExternalWall,
    MergedWindows,
)
from neosim.models.elements.system import Emission, Occupancy, System
from neosim.models.elements.wall import (
    ExternalWall,
    FloorOnGround,
    InternalElement,
    Window,
)
from neosim.models.parameters import WallParameters, WindowedWallParameters

counter = 0


class Space(BaseElement):
    name: str
    volume: float | int
    height: float | int
    floor_area: float | int
    elevation: float | int
    external_boundaries: list[Union["ExternalWall", "Window", "FloorOnGround"]]
    internal_elements: List["InternalElement"] = Field(default=[])
    boundaries: Optional[List[WallParameters]] = None
    emissions: List[System] = Field(default=[])
    control: Optional[SpaceControl] = None
    occupancy: Optional[Occupancy] = None

    @computed_field  # type: ignore
    @property
    def number_merged_external_boundaries(self) -> int:
        return sum(
            [
                boundary.length
                for boundary in self.merged_external_boundaries + self.internal_elements
            ]
        )

    @computed_field  # type: ignore
    @property
    def merged_external_boundaries(
        self,
    ) -> List[Union["ExternalWall", "Window", "FloorOnGround", "MergedBaseWall"]]:
        external_walls = [
            boundary
            for boundary in self.external_boundaries
            if boundary.type == "ExternalWall"
        ]
        windows = [
            boundary
            for boundary in self.external_boundaries
            if boundary.type == "Window"
        ]
        merged_external_walls = MergedExternalWall.from_base_elements(external_walls)
        merged_windows = MergedWindows.from_base_windows(windows)  # type: ignore
        external_boundaries = (
            merged_external_walls
            + merged_windows
            + [
                boundary
                for boundary in self.external_boundaries
                if boundary.type not in ["ExternalWall", "Window"]
            ]
        )
        return external_boundaries

    def get_controllable_emission(self) -> Optional["System"]:
        cotrollable_emissions = []
        for emission in self.emissions:
            controllable_ports = emission.get_controllable_ports()
            if controllable_ports:
                cotrollable_emissions.append(emission)
        if len(cotrollable_emissions) > 1:
            raise NotImplementedError
        if not cotrollable_emissions:
            return None
        return cotrollable_emissions[0]

    def assign_position(self) -> None:
        global counter  # noqa: PLW0603
        self.position = [200 * counter, 50]
        counter += 1
        x = self.position[0]
        y = self.position[1]
        for i, emission in enumerate(self.emissions):
            emission.position = [x + i * 30, y - 75]
        if self.control:
            self.control.position = [x - 50, y - 50]
        if self.occupancy:
            self.occupancy.position = [x - 50, y]

    def find_emission(self) -> Optional["Emission"]:
        emissions = [
            emission for emission in self.emissions if isinstance(emission, Emission)
        ]
        if not emissions:
            return None
        if len(emissions) != 1:
            raise NotImplementedError
        return emissions[0]

    def first_emission(self) -> Optional["System"]:
        if self.emissions:
            return self.emissions[0]
        return None

    def last_emission(self) -> Optional["System"]:
        if self.emissions:
            return self.emissions[-1]
        return None

    def get_neighhors(self, graph: Graph) -> None:
        neighbors = list(graph.neighbors(self))  # type: ignore
        self.boundaries = []
        for wall in [ExternalWall, Window, InternalElement, FloorOnGround]:
            self.boundaries.append(WallParameters.from_neighbors(neighbors, wall))  # type: ignore
        self.boundaries += [WindowedWallParameters.from_neighbors(neighbors)]

    def __add__(self, other: "Space") -> "Space":
        self.name = (
            f"merge_{self.name.replace('merge', '')}_{other.name.replace('merge', '')}"
        )
        self.volume = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self
