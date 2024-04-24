from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Type, Union

from networkx import Graph
from pydantic import BaseModel, Field, computed_field, field_validator

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.library.buildings.buildings import buildings_ports


class Tilt(Enum):
    wall = "wall"
    ceiling = "ceiling"
    floor = "floor"


class Azimuth:
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
                raise NotImplementedError
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


class Flow(Enum):
    inlet = "inlet"
    outlet = "outlet"
    inlet_or_outlet = "inlet_or_outlet"
    undirected = "undirected"


class PartialConnection(BaseModel):
    equation: str
    position: List[float]


class Connection(BaseModel):
    right: PartialConnection
    left: PartialConnection

    @property
    def path(self) -> List[List[float] | Tuple[float, float]]:
        if self.left.position[0] < self.right.position[0]:
            mid_path = (self.right.position[0] - self.left.position[0]) / 2
            return [
                self.left.position,
                (self.left.position[0] + mid_path, self.left.position[1]),
                (self.right.position[0] - mid_path, self.right.position[1]),
                self.right.position,
            ]

        else:
            mid_path = (self.left.position[0] - self.right.position[0]) / 2
            return [
                self.left.position,
                (self.left.position[0] - mid_path, self.left.position[1]),
                (self.right.position[0] + mid_path, self.right.position[1]),
                self.right.position,
            ]


class Port(BaseModel):
    names: list[str]
    target: Optional[Any] = None
    available: bool = True
    flow: Flow = Field(default=Flow.undirected)
    multi_connection: bool = False
    multi_object: bool = False
    bus_connection: bool = False
    use_counter: bool = True
    counter: int = Field(default=1)

    def is_available(self) -> bool:
        return self.multi_connection or self.available

    def is_controllable(self) -> bool:
        return self.target is not None and self.target == Control

    def link(
        self, node: "BaseElement", connected_node: "BaseElement"
    ) -> list[PartialConnection]:
        self.available = False
        partial_connections = []
        merged_number = 1
        if isinstance(node, MergedBaseWall):
            merged_number = len(node.surfaces)

        if isinstance(connected_node, MergedBaseWall):
            merged_number = len(connected_node.surfaces)
        for name in self.names:
            if self.multi_connection and self.bus_connection:
                first_counter = self.counter
                last_counter = self.counter + merged_number - 1
                if first_counter == last_counter:
                    counter = f"{first_counter}"
                else:
                    counter = f"{first_counter}:{last_counter}"
                if self.multi_object:
                    equation = f"{node.name}[{counter}].{name}"
                else:
                    equation = f"{node.name}.{name}[{counter}]"
                self.counter = last_counter + 1
            elif self.multi_connection and self.use_counter:
                equation = f"{node.name}.{name}[{self.counter}]"
                self.counter += 1
            else:
                equation = f"{node.name}.{name}"
            partial_connections.append(
                PartialConnection(equation=equation, position=node.position)
            )

        return partial_connections


def connect(edge: Tuple["BaseElement", "BaseElement"]) -> list[Connection]:
    connections = []
    edge_first = edge[0]
    edge_second = edge[1]
    current_port = edge_first._get_target_compatible_port(edge_second, Flow.outlet)
    other_port = edge_second._get_target_compatible_port(edge_first, Flow.inlet)
    if any(port is None for port in [current_port, other_port]):
        return []
    for left, right in zip(
        current_port.link(edge_first, edge_second),  # type: ignore
        other_port.link(edge_second, edge_first),  # type: ignore
    ):
        connections.append(Connection(left=left, right=right))
    return connections


def _has_inlet_or_outlet(target: "BaseElement") -> bool:
    return bool([port for port in target.ports if port.flow == Flow.inlet_or_outlet])


class BaseElement(BaseModel):
    name: str
    position: Optional[List[float]] = None
    ports: list[Port] = Field(default=[], validate_default=True)

    def get_position(self, layout: Dict["BaseElement", Any]) -> None:
        if not self.position:
            self.position = list(layout.get(self))  # type: ignore

    def get_controllable_ports(self) -> List[Port]:
        return [port for port in self.ports if port.is_controllable()]

    @property
    def type(self) -> str:
        return type(self).__name__

    def _get_target_compatible_port(
        self, target: "BaseElement", flow: Flow
    ) -> Optional[Port]:
        ports = [
            port
            for port in self.ports
            if port.target and isinstance(target, port.target) and port.is_available()
        ]
        if ports:
            return ports[0]
        ports = [
            port
            for port in self.ports
            if not port.target
            and port.is_available()
            and port.flow == Flow.inlet_or_outlet
            and _has_inlet_or_outlet(target)
        ]
        if ports:
            if len(ports) > 1:
                raise NotImplementedError
            return ports[0]
        ports = [
            port
            for port in self.ports
            if not port.target and port.is_available() and port.flow == flow
        ]
        if ports:
            if len(ports) > 1:
                raise NotImplementedError
            return ports[0]

        return None

    def __hash__(self) -> int:
        return hash(f"{self.name}-{type(self).__name__}")

    @field_validator("ports")
    @classmethod
    def ports_validator(cls, ports: list[Port]) -> list[Port]:
        return buildings_ports().get(cls.__name__, [])


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
    emissions: List["System"] = Field(default=[])
    control: Optional["SpaceControl"] = None
    occupancy: Optional["Occupancy"] = None

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
        external_boundaries = [merged_external_walls, merged_windows] + [
            boundary
            for boundary in self.external_boundaries
            if boundary.type not in ["ExternalWall", "Window"]
        ]
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


class Control(BaseElement):
    name: str
    position: Optional[List[float]] = None


class SpaceControl(Control):
    ...


class System(BaseElement):
    name: str
    position: Optional[List[float]] = None
    control: Optional["Control"] = None


class Emission(System):
    ...


class Valve(System):
    ...


class SplitValve(System):
    ...


class ThreeWayValve(System):
    ...


class Pump(System):
    ...


class Boiler(System):
    ...


class Occupancy(System):
    ...


class Weather(System):
    ...


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


class ExternalWall(BaseSimpleWall):
    ...


class FloorOnGround(BaseSimpleWall):
    azimuth: float | int = Azimuth.south
    tilt: Tilt = Tilt.floor


class Window(BaseSimpleWall):
    width: float | int
    height: float | int


class WindowedWall(BaseSimpleWall):
    ...


class InternalElement(BaseSimpleWall):
    ...


class MergedBaseWall(BaseWall):
    surfaces: List[float | int]
    azimuths: List[float | int]
    tilts: List[Tilt]
    constructions: List[Construction | Glass]

    @classmethod
    def from_base_elements(
        cls, base_walls: List[ExternalWall | Window | FloorOnGround]
    ) -> "MergedBaseWall":
        surfaces = [base_wall.surface for base_wall in base_walls]
        azimuths = [base_wall.azimuth for base_wall in base_walls]
        tilts = [base_wall.tilt for base_wall in base_walls]
        constructions = [base_wall.construction for base_wall in base_walls]
        names = [base_wall.name for base_wall in base_walls]
        return cls(
            name=f"merged_{'_'.join(names)}",
            surfaces=surfaces,
            azimuths=azimuths,
            tilts=tilts,
            constructions=constructions,
        )


class MergedExternalWall(MergedBaseWall):
    ...


class MergedFloor(MergedBaseWall):
    ...


class MergedWindows(MergedBaseWall):
    widths: List[float | int]
    heights: List[float | int]

    @classmethod
    def from_base_windows(cls, base_walls: List[Window]) -> "MergedWindows":
        surfaces = [base_wall.surface for base_wall in base_walls]
        azimuths = [base_wall.azimuth for base_wall in base_walls]
        tilts = [base_wall.tilt for base_wall in base_walls]
        constructions = [base_wall.construction for base_wall in base_walls]
        names = [base_wall.name for base_wall in base_walls]
        widths = [base_wall.width for base_wall in base_walls]
        heights = [base_wall.height for base_wall in base_walls]
        return cls(
            name=f"merged_{'_'.join(names)}",
            surfaces=surfaces,
            azimuths=azimuths,
            tilts=tilts,
            constructions=constructions,
            heights=heights,
            widths=widths,
        )
