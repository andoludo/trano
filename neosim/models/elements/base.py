from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

from pydantic import BaseModel, Field

from neosim.models.constants import Flow

if TYPE_CHECKING:
    pass


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
        from neosim.models.elements.control import Control

        return self.target is not None and self.target == Control

    def link(
        self, node: "BaseElement", connected_node: "BaseElement"
    ) -> list[PartialConnection]:

        from neosim.models.elements.merged_wall import MergedBaseWall

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
    ) -> Optional["Port"]:
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