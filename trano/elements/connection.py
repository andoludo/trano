from typing import TYPE_CHECKING, Any, List, Optional, Tuple, Type

from pydantic import BaseModel, Field, field_validator, model_validator

from trano import elements

from trano.elements.types import (
    ConnectionView,
    Flow,
    PartialConnection,
    BasePartialConnection,
    Medium,
)
from trano.exceptions import IncompatiblePortsError

if TYPE_CHECKING:
    from trano.elements import BaseElement
    from trano.elements.containers import Containers

INCOMPATIBLE_PORTS = [sorted(["dataBus", "y"])]


class Connection(BaseModel):
    right: BasePartialConnection
    left: BasePartialConnection
    connection_view: ConnectionView = Field(default=ConnectionView())

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


class Port(BaseModel):
    names: list[str]
    targets: Optional[List[Any]] = None
    connected: bool = False
    flow: Flow
    medium: Medium
    multi_connection: bool = False
    multi_object: bool = False
    bus_connection: bool = False
    use_counter: bool = True
    same_counter_per_name: bool = False
    counter: int = Field(default=1)

    def get_compatible_flow(self) -> Flow:
        if self.medium == Medium.fluid:
            if self.flow == Flow.inlet:
                return Flow.outlet
            if self.flow == Flow.outlet:
                return Flow.inlet
        return self.flow

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

    def is_available(self) -> bool:
        return self.multi_connection or not self.connected

    def is_controllable(self) -> bool:
        from trano.elements.base import Control

        return self.targets is not None and any(
            target == Control for target in self.targets
        )

    def base_equation(
        self,
        merged_number: int = 1,
        node_name: Optional[str] = None,
        position: Optional[List[float]] = None,
        container_position: Optional[List[float]] = None,
    ) -> List[BasePartialConnection]:
        self.connected = True
        partial_connections = []
        position = position or [0, 0]
        container_position = container_position or [0, 0]
        for sub_port_number, name in enumerate(self.names):
            if self.multi_connection and self.bus_connection:
                first_counter = self.counter
                last_counter = self.counter + merged_number - 1
                counter = (
                    f"{first_counter}"
                    if first_counter == last_counter
                    else f"{first_counter}:{last_counter}"
                )
                if node_name:
                    equation = (
                        f"{node_name}[{counter}].{name}"
                        if self.multi_object
                        else f"{node_name}.{name}[{counter}]"
                    )
                else:
                    equation = f"{name}[{counter}]" if self.multi_object else f"{name}"
                self.counter = last_counter + 1
            elif self.multi_connection and self.use_counter:
                if node_name:
                    equation = f"{node_name}.{name}[{self.counter}]"
                else:
                    equation = f"{name}[{self.counter}]"
                if not self.same_counter_per_name:
                    self.counter += 1
            else:
                if node_name:
                    equation = f"{node_name}.{name}"
                else:
                    equation = f"{name}"
            partial_connections.append(
                BasePartialConnection(
                    equation=equation,
                    position=position,
                    container_position=container_position,
                    port=self,
                    sub_port=sub_port_number,
                )
            )
        if self.same_counter_per_name:
            self.counter += 1
        return partial_connections

    def link(
        self, node: "BaseElement", connected_node: "BaseElement"
    ) -> list[PartialConnection]:

        from trano.elements.envelope import MergedBaseWall

        merged_number = 1
        if isinstance(node, MergedBaseWall):
            merged_number = len(node.surfaces)

        if isinstance(connected_node, MergedBaseWall):
            merged_number = len(connected_node.surfaces)
        base_equations = self.base_equation(merged_number, node.name, node.position)

        return [
            PartialConnection(
                equation=b.equation,
                position=node.position or b.position,
                container_position=node.container_position
                or node.position
                or b.position,
                port=self,
                container_type=node.container_type,
                connected_container_type=connected_node.container_type,
                sub_port=b.sub_port,
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
    containers: "Containers", edge: Tuple["BaseElement", "BaseElement"]
) -> list[Connection]:
    connections = []
    edge_first = edge[0]
    edge_second = edge[1]
    edge_first_ports_to_skip: List[Port] = []
    edge_second_ports_to_skip: List[Port] = []

    for first_port in edge_first.ports:
        for second_port in edge_second.ports:
            if first_port.is_available() and second_port.is_available():
                if (
                    any(isinstance(edge_second, t) for t in first_port.targets)
                    and any(isinstance(edge_first, t) for t in second_port.targets)
                    and first_port.medium == second_port.medium
                    and first_port.get_compatible_flow() == second_port.flow
                ):
                    left_right = list(
                        zip(
                            first_port.link(edge_first, edge_second),  # type: ignore
                            second_port.link(edge_second, edge_first),  # type: ignore
                            strict=True,
                        )
                    )
                    # containers.add_connection(left_right)
                    for left, right in left_right:

                        connection = Connection(
                            left=left.to_base_partial_connection(),
                            right=right.to_base_partial_connection(),
                            connection_view=connection_color(edge),
                        )
                        if left.container_type == right.container_type:
                            containers.add_connection_(connection, left.container_type)
                        connections.append(connection)
    # while True:
    #     current_port = edge_first._get_target_compatible_port(
    #         edge_second, Flow.outlet, ports_to_skip=edge_first_ports_to_skip
    #     )
    #     other_port = edge_second._get_target_compatible_port(
    #         edge_first, Flow.inlet, ports_to_skip=edge_second_ports_to_skip
    #     )
    #
    #     if any(port is None for port in [current_port, other_port]):
    #         break
    #     left_right = list(
    #         zip(
    #             current_port.link(edge_first, edge_second),  # type: ignore
    #             other_port.link(edge_second, edge_first),  # type: ignore
    #             strict=True,
    #         )
    #     )
    #     containers.add_connection(left_right)
    #     for left, right in left_right:
    #
    #         connection = Connection(
    #             left=left.to_base_partial_connection(),
    #             right=right.to_base_partial_connection(),
    #             connection_view=connection_color(edge),
    #         )
    #         if left.container_type == right.container_type:
    #             containers.add_connection_(connection, left.container_type)
    #         connections.append(connection)
    #     edge_first_ports_to_skip.append(current_port)  # type: ignore
    #     edge_second_ports_to_skip.append(other_port)  # type: ignore
    return connections


def _has_inlet_or_outlet(ports: List[Port]) -> bool:
    return bool([port for port in ports if port.flow == Flow.inlet_or_outlet])


def _is_inlet_or_outlet(ports: List[Port]) -> bool:
    return bool([port for port in ports if port.flow in [Flow.inlet, Flow.outlet]])
