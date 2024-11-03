from math import ceil
from typing import ClassVar, List, Optional, Union

from networkx import Graph
from pydantic import Field

from trano.elements.base import BaseElement
from trano.elements.envelope import (
    BaseExternalWall,
    BaseFloorOnGround,
    BaseInternalElement,
    BaseWindow,
    ExternalWall,
    FloorOnGround,
    InternalElement,
    MergedBaseWall,
    MergedExternalWall,
    MergedWindows,
    WallParameters,
    WindowedWallParameters,
)
from trano.elements.system import BaseOccupancy, Emission, System

MAX_X_SPACES = 3


def _get_controllable_element(elements: List[System]) -> Optional["System"]:
    controllable_elements = []
    for element in elements:
        controllable_ports = element.get_controllable_ports()
        if controllable_ports:
            controllable_elements.append(element)
    if len(controllable_elements) > 1:
        raise NotImplementedError
    if not controllable_elements:
        return None
    return controllable_elements[0]


class Space(BaseElement):
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-20, -20}, {20, 20}}
    {% endraw %})));"""
    counter: ClassVar[int] = 0
    name: str
    external_boundaries: list[
        Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround"]
    ]
    internal_elements: List["BaseInternalElement"] = Field(default=[])
    boundaries: Optional[List[WallParameters]] = None
    emissions: List[System] = Field(default=[])
    ventilation_inlets: List[System] = Field(default=[])
    ventilation_outlets: List[System] = Field(default=[])
    occupancy: Optional[BaseOccupancy] = None

    def model_post_init(self, __context) -> None:  # type: ignore # noqa: ANN001
        self._assign_space()

    def _assign_space(self) -> None:
        for emission in (
            self.emissions + self.ventilation_inlets + self.ventilation_outlets
        ):
            if emission.control:
                emission.control.space_name = self.name
        if self.occupancy:
            self.occupancy.space_name = self.name

    @property
    def number_merged_external_boundaries(self) -> int:
        return sum(
            [
                boundary.length
                for boundary in self.merged_external_boundaries + self.internal_elements
            ]
        )

    @property
    def number_ventilation_ports(self) -> int:
        return 2 + 1  # databus

    @property
    def merged_external_boundaries(
        self,
    ) -> List[
        Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround", "MergedBaseWall"]
    ]:

        external_walls = [
            boundary
            for boundary in self.external_boundaries
            if boundary.type in ["ExternalWall", "ExternalDoor"]
        ]
        windows = [
            boundary
            for boundary in self.external_boundaries
            if boundary.type == "Window"
        ]
        merged_external_walls = MergedExternalWall.from_base_elements(external_walls)
        merged_windows = MergedWindows.from_base_windows(windows)  # type: ignore
        external_boundaries: list[
            BaseExternalWall | BaseWindow | BaseFloorOnGround | MergedBaseWall
        ] = (
            merged_external_walls
            + merged_windows
            + [
                boundary
                for boundary in self.external_boundaries
                if boundary.type not in ["ExternalWall", "Window", "ExternalDoor"]
            ]
        )
        return external_boundaries

    def get_controllable_emission(self) -> Optional["System"]:
        return _get_controllable_element(self.emissions)

    def assign_position(self) -> None:
        self.position = [
            250 * (Space.counter % MAX_X_SPACES),
            150 * ceil(Space.counter / MAX_X_SPACES),
        ]
        Space.counter += 1
        x = self.position[0]
        y = self.position[1]
        for i, emission in enumerate(self.emissions):
            emission.position = [x + i * 30, y - 75]
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

    def get_ventilation_inlet(self) -> Optional["System"]:
        if self.ventilation_inlets:
            return self.ventilation_inlets[-1]
        return None

    def get_last_ventilation_inlet(self) -> Optional["System"]:
        if self.ventilation_inlets:
            return self.ventilation_inlets[0]
        return None

    def get_ventilation_outlet(self) -> Optional["System"]:
        if self.ventilation_outlets:
            return self.ventilation_outlets[0]
        return None

    def get_last_ventilation_outlet(self) -> Optional["System"]:
        if self.ventilation_outlets:
            return self.ventilation_outlets[-1]
        return None

    def get_neighhors(self, graph: Graph) -> None:

        neighbors = list(graph.neighbors(self))  # type: ignore
        self.boundaries = []
        windowed_wall_parameters = WindowedWallParameters.from_neighbors(neighbors)
        for wall in [ExternalWall, BaseWindow, InternalElement, FloorOnGround]:
            self.boundaries.append(
                WallParameters.from_neighbors(
                    neighbors,
                    wall,  # type: ignore
                    filter=windowed_wall_parameters.included_external_walls,
                )
            )
        self.boundaries += [windowed_wall_parameters]

    def __add__(self, other: "Space") -> "Space":
        self.name = (
            f"merge_{self.name.replace('merge', '')}_{other.name.replace('merge', '')}"
        )
        self.volume: float = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self
