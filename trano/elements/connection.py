import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, List, Optional, Tuple, Type

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, Field, field_validator, model_validator, computed_field

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
    from trano.elements import BaseElement
    from trano.elements.base import ElementPort

INCOMPATIBLE_PORTS = [sorted(["dataBus", "y"])]

logger = logging.getLogger(__name__)


class Port(BaseModel):

    names: list[str]
    targets: List[Any] = Field(default_factory=list)
    expected_ports: List[str] = Field(default_factory=list)
    connected: bool = False
    flow: Flow
    medium: Medium
    multi_connection: bool = False
    multi_object: bool = False
    bus_connection: bool = False
    use_counter: bool = True
    ignore_direction: bool = False
    counter: int = Field(default=1)
    connection_counter: int = Field(default=0)

    def set_connected(self) -> None:
        self.connected = True
        self.connection_counter += 1

    def get_compatible_port(self, port: "Port") -> bool:  # noqa: PLR0911, C901, PLR0912
        if self.medium == Medium.fluid:
            if self.expected_ports:
                return bool(set(port.names).intersection(set(self.expected_ports)))
            else:
                if self.flow == Flow.inlet:
                    return (port.flow in [Flow.outlet]) or (
                        port.flow in [Flow.inlet_or_outlet]
                        and port.multi_connection
                        and port.use_counter
                    )
                if self.flow == Flow.outlet:
                    return port.flow in [Flow.inlet] or (
                        port.flow in [Flow.inlet_or_outlet]
                        and port.multi_connection
                        and port.use_counter
                    )
                if (
                    self.flow == Flow.inlet_or_outlet
                    and self.multi_connection
                    and self.use_counter
                ):
                    return port.flow in [Flow.inlet, Flow.outlet]
                if self.flow == Flow.inlet_or_outlet:
                    return port.flow in [Flow.inlet_or_outlet]

        if self.medium == Medium.data:
            if self.expected_ports:
                return bool(set(port.names).intersection(set(self.expected_ports)))
            elif self.flow == Flow.inlet:
                return port.flow in [Flow.outlet]
            elif self.flow == Flow.outlet:
                return port.flow in [Flow.inlet]
            elif self.flow == Flow.undirected:
                return port.flow in [Flow.undirected, Flow.interchangeable_port]

            else:
                return True
        return self.flow == port.flow

    def similar_flow(self, port: "Port") -> bool:
        return self.flow == port.flow

    def is_complementary(self, port: "Port") -> bool:
        return self.medium == port.medium and self.get_compatible_port(port)

    def is_inlet(self) -> bool:
        return self.flow in [Flow.inlet] and self.medium == Medium.fluid

    def is_outlet(self) -> bool:
        return self.flow in [Flow.outlet] and self.medium == Medium.fluid

    def is_extended_inlet(self) -> bool:
        return (
            self.flow in [Flow.inlet_or_outlet]
            and self.medium == Medium.fluid
            and self.multi_connection
            and self.use_counter
        )

    def is_extended_outlet(self) -> bool:
        return (
            self.flow in [Flow.inlet_or_outlet]
            and self.medium == Medium.fluid
            and self.multi_connection
            and self.use_counter
        )

    def with_directed_flow(self) -> bool:
        return self.flow in [Flow.inlet, Flow.outlet]

    def bidirectional_flow(self) -> bool:
        return self.flow in [Flow.inlet_or_outlet] and self.medium == Medium.fluid

    def without_targets(self) -> "Port":
        return Port.model_validate(self.model_dump(exclude={"targets"}))

    def disconnect(self) -> "Port":
        return Port.model_validate(self.model_dump(exclude={"connected"}))

    def set_ignore_direction(self) -> "Port":
        port = Port.model_validate(self.model_dump())
        port.ignore_direction = True
        return port

    def reset_counter(self) -> "Port":
        self.counter = 1
        return self

    def substract_counter(self) -> "Port":
        if self.counter != 1:
            return Port.model_validate(
                self.model_dump(exclude={"counter"}) | {"counter": self.counter - 1}
            )
        return self

    @field_validator("targets")
    @classmethod
    def validate_targets(cls, values: List[str]) -> List[Type["BaseElement"]]:
        from trano.elements.base import BaseElement

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
            if self.medium == Medium.fluid:
                return not any(
                    (p.medium == self.medium and self.similar_flow(p))
                    for p in available_ports
                )
            else:
                return True
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
                equation = (
                    f"{element_name}.{name}[{self.counter}]"
                    if element_name
                    else f"{name}[{self.counter}]"
                )
                self.counter += 1
            elif element_name:
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

        return partial_connections

    def link(  # noqa: PLR0913
        self,
        merged_number: int,
        element_name: str | None,
        element_position: BasePosition,
        container_type: ContainerTypes | None,
        connected_container_type: ContainerTypes | None,
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


def connection_color(edge: Tuple["ElementPort", "ElementPort"]) -> ConnectionView:
    from trano.elements.bus import DataBus
    from trano.elements.envelope import BaseSimpleWall
    from trano.elements.system import Weather, System
    from trano.elements.base import Control

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


def check_flow_direction(first_port: Port, second_port: Port) -> bool:
    if first_port.medium == Medium.fluid and second_port.medium == Medium.fluid:
        if first_port.ignore_direction and second_port.ignore_direction:
            return first_port.get_compatible_port(second_port)
        elif second_port.with_directed_flow() and first_port.with_directed_flow():
            return first_port.is_outlet() and second_port.is_inlet()
        else:
            return (
                second_port.bidirectional_flow()
                and first_port.bidirectional_flow()
                or (
                    (first_port.is_outlet() and second_port.is_extended_inlet())
                    or (first_port.is_extended_outlet() and second_port.is_inlet())
                )
            )
    else:
        return True


def connect(
    edge_first: "ElementPort", edge_second: "ElementPort"
) -> list["Connection"]:
    connections = []
    try:
        for first_port in edge_first.ports:
            for second_port in edge_second.ports:
                available = first_port.is_available(
                    edge_first.available_ports()
                ) and second_port.is_available(edge_second.available_ports())
                has_targets = edge_second.has_target(
                    first_port.targets
                ) and edge_first.has_target(second_port.targets)
                complementarity = first_port.is_complementary(second_port)
                flow_direction = check_flow_direction(first_port, second_port)
                if available and has_targets and complementarity and flow_direction:
                    merged_number = max(
                        edge_first.merged_number, edge_second.merged_number
                    )
                    left_right = list(
                        zip(
                            first_port.link(
                                merged_number,
                                edge_first.name,
                                edge_first.position,
                                edge_first.container_type,
                                edge_second.container_type,
                            ),
                            second_port.link(
                                merged_number,
                                edge_second.name,
                                edge_second.position,
                                edge_second.container_type,
                                edge_first.container_type,
                            ),
                            strict=True,
                        )
                    )
                    for left, right in left_right:
                        connection = Connection(
                            left=left,
                            right=right,
                            connection_view=connection_color((edge_first, edge_second)),
                        )
                        connections.append(connection)
                    first_port.set_connected()
                    second_port.set_connected()
                    current_connection_numbers = len(connections)
                    allowed_connections = edge_first.get_connection_per_target(
                        edge_second.element_type
                    )
                    if current_connection_numbers >= allowed_connections:
                        raise ConnectionLimitReached
    except ConnectionLimitReached:
        ...

    return connections


class BasePartialConnection(BaseElementPosition):
    name: Optional[str] = None
    equation: str
    port: Port
    sub_port: int


class BasePartialConnectionWithContainerType(BasePartialConnection):
    container_type: Optional[ContainerTypes] = None


class PartialConnection(BasePartialConnectionWithContainerType):
    connected_container_type: Optional[ContainerTypes] = None

    def reset_port_counter(self) -> "PartialConnection":
        self.port.reset_counter()
        return self

    def to_base_partial_connection(self) -> BasePartialConnection:
        return BasePartialConnection.model_validate(
            self.model_dump(exclude={"connected_container_type", "container_type"})
        )


class Connection(BaseModel):
    right: PartialConnection
    left: PartialConnection
    connection_view: ConnectionView = Field(default=ConnectionView())

    def in_the_same_container(self) -> bool:
        return bool(self.left.name is not None) and bool(self.right.name is not None)

    def reset_counters(self) -> "Connection":
        self.left.reset_port_counter()
        self.right.reset_port_counter()
        return self

    def equation_view(self) -> Tuple[str, ...]:
        return tuple(sorted([self.left.equation, self.right.equation]))

    @computed_field
    def equation(self) -> str:
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(
                str(Path(__file__).parents[1].joinpath("templates"))
            ),
            autoescape=True,
        )
        annotation_template = environment.from_string(
            """{% import 'macros.jinja2' as macros %}        
        connect({{ connection.left.equation }},{{ connection.right.equation }})
            {% if not connection.connection_view.disabled %}
        annotation (Line(
        points={{ macros.connect_path(connection.path) }},
            {% if connection.connection_view.color %}
        color={{ connection.connection_view.color }},
            {% endif %}
        thickness={{ connection.connection_view.thickness }},pattern =
        LinePattern.{{ connection.connection_view.pattern }},
        smooth=Smooth.None))
            {% endif %}
            ;"""
        )

        return annotation_template.render(
            connection=self,
        )

    @model_validator(mode="after")
    def _connection_validator(self) -> "Connection":
        if (
            self.right.position.container.is_empty()
            or self.left.position.container.is_empty()
        ):
            logger.debug(
                f"Connection position still empty for {self.right.name} and {self.left.name}."
            )
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
        if (
            self.left.position.global_.location.c_.x
            < self.right.position.global_.location.c_.x
        ):
            mid_path = (
                self.right.position.global_.location.c_.x
                - self.left.position.global_.location.c_.x
            ) / 2
            return [
                self.left.position.global_.coordinate(),
                (
                    self.left.position.global_.location.c_.x + mid_path,
                    self.left.position.global_.location.c_.y,
                ),
                (
                    self.right.position.global_.location.c_.x - mid_path,
                    self.right.position.global_.location.c_.y,
                ),
                self.right.position.global_.coordinate(),
            ]

        else:
            mid_path = (
                self.left.position.global_.location.c_.x
                - self.right.position.global_.location.c_.x
            ) / 2
            return [
                self.left.position.global_.coordinate(),
                (
                    self.left.position.global_.location.c_.x - mid_path,
                    self.left.position.global_.location.c_.y,
                ),
                (
                    self.right.position.global_.location.c_.x + mid_path,
                    self.right.position.global_.location.c_.y,
                ),
                self.right.position.global_.coordinate(),
            ]


class ContainerConnection(Connection):
    source: Tuple[str, str]

    def get_container_equation(
        self,
    ) -> Optional[BasePartialConnectionWithContainerType]:
        if self.right.name is None:
            return self.right
        if self.left.name is None:
            return self.left
        return None

    @property
    def path(self) -> List[List[float] | Tuple[float, float]]:
        if (
            self.left.position.container.location.c_.x
            < self.right.position.container.location.c_.x
        ):
            mid_path = (
                self.right.position.container.location.c_.x
                - self.left.position.container.location.c_.x
            ) / 2
            return [
                self.left.position.container.coordinate(),
                (
                    self.left.position.container.location.c_.x + mid_path,
                    self.left.position.container.location.c_.y,
                ),
                (
                    self.right.position.container.location.c_.x - mid_path,
                    self.right.position.container.location.c_.y,
                ),
                self.right.position.container.coordinate(),
            ]

        else:
            mid_path = (
                self.left.position.container.location.c_.x
                - self.right.position.container.location.c_.x
            ) / 2
            return [
                self.left.position.container.coordinate(),
                (
                    self.left.position.container.location.c_.x - mid_path,
                    self.left.position.container.location.c_.y,
                ),
                (
                    self.right.position.container.location.c_.x + mid_path,
                    self.right.position.container.location.c_.y,
                ),
                self.right.position.container.coordinate(),
            ]
