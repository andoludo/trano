from typing import TYPE_CHECKING, Any, List, Optional, Tuple, Type

from pydantic import BaseModel, Field, field_validator, model_validator

from trano import elements
from trano.elements.common_base import BaseElementPosition, BasePosition

from trano.elements.types import (
    ConnectionView,
    Flow,
    Medium,
    ContainerTypes,
)
from trano.exceptions import IncompatiblePortsError, ConnectionLimitReached

if TYPE_CHECKING:
    from trano.elements import BaseElement, ThreeWayValve
    from trano.elements.base import ElementPort
    from trano.elements.containers import Containers

INCOMPATIBLE_PORTS = [sorted(["dataBus", "y"])]


class Port(BaseModel):
    names: list[str]
    targets: List[Any] = Field(default_factory=list)
    connected: bool = False
    flow: Flow
    medium: Medium
    multi_connection: bool = False
    multi_object: bool = False
    bus_connection: bool = False
    use_counter: bool = True
    same_counter_per_name: bool = False
    counter: int = Field(default=1)
    connection_counter: int = Field(default=0)

    def set_connected(self) ->None:
        self.connected = True
        self.connection_counter += 1

    def get_compatible_flow(self, flow: Flow) -> bool:
        if self.medium == Medium.fluid:
            if self.flow == Flow.inlet:
                return flow in [Flow.outlet, Flow.inlet_or_outlet]
            if self.flow == Flow.outlet:
                return flow in [Flow.inlet, Flow.inlet_or_outlet]
            if self.flow == Flow.inlet_or_outlet:
                return flow in [Flow.inlet, Flow.outlet, Flow.inlet_or_outlet]
        if self.medium == Medium.data:
            return True
        return self.flow == flow

    def similar_flow(self, port: "Port") -> bool:
        if self.medium == Medium.fluid:
            return self.flow == port.flow or any([f == Flow.inlet_or_outlet for f in [self.flow, port.flow]])
        return self.flow == port.flow

    def is_complementary(self, port: "Port") -> bool:
        return self.medium == port.medium and self.get_compatible_flow(port.flow)

    def is_inlet(self) -> bool:
        return self.flow in [Flow.inlet, Flow.inlet_or_outlet] and self.medium == Medium.fluid

    def is_outlet(self) -> bool:
        return self.flow in [Flow.outlet, Flow.inlet_or_outlet] and self.medium == Medium.fluid

    def with_directed_flow(self) -> bool:
        return self.flow in [Flow.inlet, Flow.outlet, Flow.inlet_or_outlet]

    def without_targets(self) -> "Port":
        return Port.model_validate(self.model_dump(exclude={"targets"}))

    def disconnect(self) -> "Port":
        return Port.model_validate(self.model_dump(exclude={"connected"}))

    @field_validator("targets")
    @classmethod
    def validate_targets(cls, values: List[str]) -> List[Type["BaseElement"]]:
        from trano.elements import BaseElement

        targets: List[Type[BaseElement]] = []
        for value in values:
            if isinstance(value, str):
                if hasattr(elements, value):
                    targets.append(getattr(elements, value))
                else:
                    raise ValueError(f"Target {value} not found")
            else:
                targets.append(value)
        return targets

    def is_available(self, available_ports: List["Port"]) -> bool:
        if self.multi_connection and self.connected:
            return not any((p.medium==self.medium and self.similar_flow(p)) for p in available_ports)
        else:
            return not self.connected


    def is_controllable(self) -> bool:
        from trano.elements.base import Control

        return self.targets is not None and any(
            target == Control for target in self.targets
        )

    def base_equation(
        self,
        merged_number: int,
        element_name: Optional[str] = None,
        element_position: Optional[BasePosition] = None,
    ) -> List["BasePartialConnection"]:
        element_position = element_position or BasePosition()
        partial_connections = []
        for sub_port_number, name in enumerate(self.names):
            if self.multi_connection and self.bus_connection:
                first_counter = self.counter
                last_counter = self.counter + merged_number - 1
                counter = (
                    f"{first_counter}"
                    if first_counter == last_counter
                    else f"{first_counter}:{last_counter}"
                )
                if element_name:
                    equation = (
                        f"{element_name}[{counter}].{name}"
                        if self.multi_object
                        else f"{element_name}.{name}[{counter}]"
                    )
                else:
                    equation = f"{name}[{counter}]" if self.multi_object else f"{name}"
                self.counter = last_counter + 1
            elif self.multi_connection and self.use_counter:
                if element_name:
                    equation = f"{element_name}.{name}[{self.counter}]"
                else:
                    equation = f"{name}[{self.counter}]"
                if not self.same_counter_per_name:
                    self.counter += 1
            else:
                if element_name:
                    equation = f"{element_name}.{name}"
                else:
                    equation = f"{name}"
            partial_connections.append(
                BasePartialConnection(
                    equation=equation,
                    position=element_position,
                    port=self,
                    sub_port=sub_port_number,
                    name=element_name,
                )
            )
        if self.same_counter_per_name:
            self.counter += 1
        return partial_connections

    def link(
        self,
        merged_number: int,
        element_name: str,
        element_position: BasePosition,
        container_type: ContainerTypes,
        connected_container_type: ContainerTypes,
    ) -> list["PartialConnection"]:
        base_equations = self.base_equation(
            merged_number, element_name, element_position
        )

        return [
            PartialConnection(
                **(
                    b.model_dump()
                    | {
                        "connected_container_type": connected_container_type,
                        "container_type": container_type,
                    }
                )
            )
            for b in base_equations
        ]


def connection_color(edge: Tuple["BaseElement", "BaseElement"]) -> ConnectionView:
    from trano.elements.bus import DataBus
    from trano.elements.envelope import BaseSimpleWall
    from trano.elements.system import Weather, System, Control

    if any(isinstance(e, BaseSimpleWall) for e in edge):
        return ConnectionView(color="{191,0,0}", thickness=0.1)
    if any(isinstance(e, DataBus) for e in edge):
        return ConnectionView(color=None, thickness=0.05)
    if any(isinstance(e, Weather) for e in edge):
        return ConnectionView(color=None, thickness=0.05)
    if any(isinstance(e, Control) for e in edge):
        return ConnectionView(color="{139, 0, 0}", thickness=0.1, pattern="Dash")
    if all(isinstance(e, System) for e in edge):
        return ConnectionView(color="{0, 0, 139}", thickness=0.75)
    return ConnectionView()


def connect(
    edge_first: "ElementPort", edge_second: "ElementPort"
) -> list["Connection"]:
    connections = []
    try:
        for first_port in edge_first.ports:
            for second_port in edge_second.ports:
                if first_port.is_available(edge_first.available_ports()) and second_port.is_available(edge_second.available_ports()):
                    if (
                        edge_second.has_target(first_port.targets)
                        and edge_first.has_target(second_port.targets)
                        and first_port.is_complementary(second_port)
                        and (
                            (first_port.is_outlet() and second_port.is_inlet())
                            if (
                                second_port.with_directed_flow()
                                and first_port.with_directed_flow()
                            )
                            else True
                        )
                    ):
                        from trano.elements import BaseElement, ThreeWayValve
                        if issubclass(edge_first.element_type, ThreeWayValve) or issubclass(edge_second.element_type, ThreeWayValve):
                            a = 12
                        merged_number = max(
                            edge_first.merged_number, edge_second.merged_number
                        )
                        left_right = list(
                            zip(
                                first_port.link(merged_number, edge_first.name, edge_first.position, edge_first.container_type, edge_second.container_type),  # type: ignore
                                second_port.link(merged_number, edge_second.name, edge_second.position, edge_second.container_type, edge_first.container_type),  # type: ignore
                                strict=True,
                            )
                        )
                        for left, right in left_right:
                            connection = Connection(
                                left=left,
                                right=right,
                                connection_view=connection_color(
                                    (edge_first, edge_second)
                                ),
                            )
                            connections.append(connection)
                        first_port.set_connected()
                        second_port.set_connected()
                        current_connection_numbers = len(connections)
                        if edge_second.connection_limit_reached(
                            current_connection_numbers, first_port.medium
                        ) and edge_first.connection_limit_reached(
                            current_connection_numbers, first_port.medium
                        ):
                            raise ConnectionLimitReached
    except ConnectionLimitReached:
        ...

    return connections


class BasePartialConnection(BaseElementPosition):
    name: Optional[str] = None
    equation: str
    port: Port
    sub_port: int


class ContainerConnection(BasePartialConnection):
    container_type: Optional[ContainerTypes] = None


class PartialConnection(ContainerConnection):
    connected_container_type: Optional[ContainerTypes] = None

    def to_base_partial_connection(self) -> BasePartialConnection:
        return BasePartialConnection.model_validate(
            self.model_dump(exclude={"connected_container_type", "container_type"})
        )


class Connection(BaseModel):
    right: PartialConnection
    left: PartialConnection
    connection_view: ConnectionView = Field(default=ConnectionView())

    def equation_view(self):
        return tuple(sorted([self.left.equation, self.right.equation]))

    @model_validator(mode="after")
    def _connection_validator(self) -> "Connection":
        if (
            sorted(
                [
                    part.split(".")[-1]
                    for part in [self.right.equation, self.left.equation]
                ]
            )
            in INCOMPATIBLE_PORTS
        ):
            raise IncompatiblePortsError(
                f"Incompatible ports {self.right.equation} and {self.left.equation}."
            )
        return self

    @property
    def path(self) -> List[List[float] | Tuple[float, float]]:
        if self.left.position.global_.location.x < self.right.position.global_.location.x:
            mid_path = (self.right.position.global_.location.x- self.left.position.global_.location.x) / 2
            return [
                self.left.position.global_.coordinate(),
                (self.left.position.global_.location.x + mid_path, self.left.position.global_.location.y),
                (self.right.position.global_.location.x - mid_path, self.right.position.global_.location.y),
                self.right.position.global_.coordinate(),
            ]

        else:
            mid_path = (self.left.position.global_.location.x - self.right.position.global_.location.x) / 2
            return [
                self.left.position.global_.coordinate(),
                (self.left.position.global_.location.x - mid_path, self.left.position.global_.location.y),
                (self.right.position.global_.location.x + mid_path, self.right.position.global_.location.y),
                self.right.position.global_.coordinate(),
            ]

    @property
    def container_path(self) -> List[List[float] | Tuple[float, float]]:
        square_size = 30  # Each square is 30x30
        buffer = 10  # Additional spacing to avoid overlap
        # TODO: change these 0 locator into x,y???
        if self.left.container_position[0] < self.right.container_position[0]:
            mid_x = (
                self.right.container_position[0] - self.left.container_position[0]
            ) / 2
            if self.left.container_position[1] < self.right.container_position[1]:
                mid_y = (
                    self.right.container_position[1] - self.left.container_position[1]
                ) / 2
                path = [
                    self.left.container_position,
                    (
                        self.left.container_position[0],
                        self.left.container_position[1] + mid_y,
                    ),
                    # Move outside left square
                    (
                        self.left.container_position[0] + mid_x,
                        self.left.container_position[1] + mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0] - mid_x,
                        self.right.container_position[1] - mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0],
                        self.right.container_position[1] - mid_y,
                    ),
                    # Move outside right square
                    self.right.container_position,
                ]
            else:
                mid_y = (
                    self.left.container_position[1] - self.right.container_position[1]
                ) / 2
                path = [
                    self.left.container_position,
                    (
                        self.left.container_position[0],
                        self.left.container_position[1] - mid_y,
                    ),
                    # Move outside left square
                    (
                        self.left.container_position[0] + mid_x,
                        self.left.container_position[1] - mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0] - mid_x,
                        self.right.container_position[1] + mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0],
                        self.right.container_position[1] + mid_y,
                    ),
                    # Move outside right square
                    self.right.container_position,
                ]

        else:
            mid_x = (
                self.left.container_position[0] - self.right.container_position[0]
            ) / 2
            if self.left.container_position[1] < self.right.container_position[1]:
                mid_y = (
                    self.right.container_position[1] - self.left.container_position[1]
                ) / 2
                path = [
                    self.left.container_position,
                    (
                        self.left.container_position[0],
                        self.left.container_position[1] + mid_y,
                    ),
                    # Move outside left square
                    (
                        self.left.container_position[0] - mid_x,
                        self.left.container_position[1] + mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0] + mid_x,
                        self.right.container_position[1] - mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0],
                        self.right.container_position[1] - mid_y,
                    ),
                    # Move outside right square
                    self.right.container_position,
                ]
            else:
                mid_y = (
                    self.left.container_position[1] - self.right.container_position[1]
                ) / 2
                path = [
                    self.left.container_position,
                    (
                        self.left.container_position[0],
                        self.left.container_position[1] - mid_y,
                    ),
                    # Move outside left square
                    (
                        self.left.container_position[0] - mid_x,
                        self.left.container_position[1] - mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0] + mid_x,
                        self.right.container_position[1] + mid_y,
                    ),  # Midway above both squares
                    (
                        self.right.container_position[0],
                        self.right.container_position[1] + mid_y,
                    ),
                    # Move outside right square
                    self.right.container_position,
                ]

        return path


def _has_inlet_or_outlet(ports: List[Port]) -> bool:
    return bool([port for port in ports if port.flow == Flow.inlet_or_outlet])


def _is_inlet_or_outlet(ports: List[Port]) -> bool:
    return bool([port for port in ports if port.flow in [Flow.inlet, Flow.outlet]])
