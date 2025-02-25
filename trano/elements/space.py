from math import ceil
from typing import ClassVar, List, Optional, Union, TYPE_CHECKING

from networkx import Graph
from pydantic import Field, BaseModel


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
    VerticalWallParameters,
    RoofWallParameters,
    ExternalWallParameters,
)
from trano.elements.system import BaseOccupancy, Emission, System, AirHandlingUnit
from trano.elements.types import ContainerTypes

if TYPE_CHECKING:
    from trano.topology import Network

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


class BoundaryParameter(BaseModel):
    number_orientations: int = 0
    area_per_orientation: List[float] = Field(default_factory=lambda: [0.0])
    average_resistance_external: float = Field(0.001)
    average_resistance_external_remaining: float = Field(0.001)
    total_thermal_capacitance: float = Field(10000)
    tilts: List[float] = Field(default_factory=lambda: [0.0])
    azimuths: List[float] = Field(default_factory=lambda: [0.0])
    average_u_value: float = 0.001
    total_thermal_resistance: float = 0.001

    @classmethod
    def from_parameter(cls, parameter: WallParameters) -> "BoundaryParameter":
        if not parameter.number and parameter.type != "BaseWindow":
            return cls()
        return cls(
            number_orientations=parameter.number,
            area_per_orientation=(
                parameter.window_area_by_orientation
                if parameter.type == "BaseWindow"
                else parameter.surfaces
            ),
            average_resistance_external=parameter.average_resistance_external,
            average_resistance_external_remaining=parameter.average_resistance_external_remaining,
            total_thermal_capacitance=parameter.total_thermal_capacitance,
            tilts=parameter.tilts_to_radians(),
            azimuths=parameter.azimuths_to_radians(),
            average_u_value=parameter.average_u_value,
            tottal_thermal_resistance=parameter.total_thermal_resistance,
        )


class BoundaryParameters(BaseModel):
    roofs: BoundaryParameter = Field(default_factory=BoundaryParameter)
    external_boundaries: BoundaryParameter = Field(default_factory=BoundaryParameter)
    vertical_walls: BoundaryParameter = Field(default_factory=BoundaryParameter)
    windows: BoundaryParameter = Field(default_factory=BoundaryParameter)
    floors: BoundaryParameter = Field(default_factory=BoundaryParameter)

    @classmethod
    def from_boundaries(cls, parameters: List[WallParameters]) -> "BoundaryParameters":
        data = {}
        for p in parameters:
            if p.type == "ExternalWallRoof":
                data["roofs"] = BoundaryParameter.from_parameter(p)
            elif p.type == "ExternalWallVerticalOnly":
                data["vertical_walls"] = BoundaryParameter.from_parameter(p)
            elif p.type == "ExternalWallExternal":
                data["external_boundaries"] = BoundaryParameter.from_parameter(p)
            elif p.type == "FloorOnGround":
                data["floors"] = BoundaryParameter.from_parameter(p)
            elif p.type == "BaseWindow":
                data["windows"] = BoundaryParameter.from_parameter(p)
        return cls(**data)


class BaseSpace(BaseElement):
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
    container_type: ContainerTypes = "envelope"
    boundary_parameters: Optional[BoundaryParameters] = None

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
        x, y = [
            250 * (Space.counter % MAX_X_SPACES),
            150 * ceil(Space.counter / MAX_X_SPACES),
        ]
        self.position.set_global(x, y)
        Space.counter += 1

        for i, emission in enumerate(self.emissions):
            emission.position.set_global(x + i * 30, y - 75)
        if self.occupancy:
            self.occupancy.position.set_global(x - 50, y)

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
            if wall == ExternalWall:
                self.boundaries.append(
                    VerticalWallParameters.from_neighbors_(
                        neighbors,
                        wall,  # type: ignore
                    )
                )
                self.boundaries.append(
                    RoofWallParameters.from_neighbors_(
                        neighbors,
                        wall,  # type: ignore
                    )
                )
                self.boundaries.append(
                    ExternalWallParameters.from_neighbors_(
                        neighbors,
                        wall,  # type: ignore
                    )
                )

            self.boundaries.append(
                WallParameters.from_neighbors(
                    neighbors,
                    wall,  # type: ignore
                    filter=windowed_wall_parameters.included_external_walls,
                )
            )
        self.boundaries += [windowed_wall_parameters]
        self.boundary_parameters = BoundaryParameters.from_boundaries(self.boundaries)

    def __add__(self, other: "BaseSpace") -> "BaseSpace":
        self.name = (
            f"merge_{self.name.replace('merge', '')}_{other.name.replace('merge', '')}"
        )
        self.volume: float = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self


class Space(BaseSpace):
    def add_to_network(self, network: "Network") -> None:
        network.add_node(self)
        if network.library.merged_external_boundaries:
            external_boundaries = self.merged_external_boundaries
        else:
            external_boundaries = self.external_boundaries  # type: ignore
        for boundary in external_boundaries:
            network.add_node(boundary)
            network.graph.add_edge(
                self,
                boundary,
            )
        emission = self.find_emission()
        if emission:
            network.add_node(emission)
            network.graph.add_edge(
                self,
                emission,
            )
            network._add_subsequent_systems(self.emissions)
        if self.occupancy:
            network.add_node(self.occupancy)
            network.connect_system(self, self.occupancy)
        # Assumption: first element always the one connected to the space.
        if self.get_ventilation_inlet():
            network.add_node(self.get_ventilation_inlet())  # type: ignore
            network.graph.add_edge(self.get_ventilation_inlet(), self)
        if self.get_ventilation_outlet():
            network.add_node(self.get_ventilation_outlet())  # type: ignore
            network.graph.add_edge(self, self.get_ventilation_outlet())
        # The rest is connected to each other
        network._add_subsequent_systems(self.ventilation_outlets)
        network._add_subsequent_systems(self.ventilation_inlets)
        self.assign_position()

    def processing(self, network: "Network") -> None:
        from trano.elements import VAVControl

        _neighbors = []
        if self.get_last_ventilation_inlet():
            _neighbors += list(
                network.graph.predecessors(self.get_last_ventilation_inlet())  # type: ignore
            )
        if self.get_last_ventilation_outlet():
            _neighbors += list(
                network.graph.predecessors(self.get_last_ventilation_outlet())  # type: ignore
            )
        neighbors = list(set(_neighbors))
        controllable_ventilation_elements = list(
            filter(
                None,
                [
                    _get_controllable_element(self.ventilation_inlets),
                    _get_controllable_element(self.ventilation_outlets),
                ],
            )
        )
        for controllable_element in controllable_ventilation_elements:
            if controllable_element.control and isinstance(
                controllable_element.control, VAVControl
            ):
                controllable_element.control.ahu = next(
                    (n for n in neighbors if isinstance(n, AirHandlingUnit)), None
                )

        self.get_neighhors(network.graph)
