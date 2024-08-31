from typing import List, Tuple, Optional, Any, Type

from pydantic import BaseModel, Field, field_validator


from trano.elements.types import PartialConnection, ConnectionView, Flow


class Connection(BaseModel):
    right: PartialConnection
    left: PartialConnection
    connection_view: ConnectionView = Field(default=ConnectionView())

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
    targets: Optional[List[Any]] = None
    available: bool = True
    flow: Flow = Field(default=Flow.undirected)
    multi_connection: bool = False
    multi_object: bool = False
    bus_connection: bool = False
    use_counter: bool = True
    counter: int = Field(default=1)

    @field_validator("targets")
    @classmethod
    def validate_targets(  # noqa: PLR0915, PLR0912, C901
        cls, values: List[str]
    ) -> List[Type["BaseElement"]]:  # TODO: reduce complexity
        # TODO: this function to be refactored!!!
        from trano.elements import BaseElement
        from trano.elements import Control
        targets: List[Type[BaseElement]] = []
        for value in values:
            if isinstance(value, str):
                if value == "DataBus":
                    from trano.elements.bus import DataBus

                    targets.append(DataBus)
                elif value == "Control":

                    targets.append(Control)
                elif value == "Space":
                    from trano.elements.space import Space

                    targets.append(Space)
                elif value == "BaseBoundary":
                    from trano.elements.boundary import BaseBoundary
                    targets.append(BaseBoundary)
                elif value == "System":
                    from trano.elements.system import System

                    targets.append(System)
                elif value == "AhuControl":
                    from trano.elements.control import AhuControl

                    targets.append(AhuControl)
                elif value == "BaseWeather":
                    from trano.elements.system import BaseWeather

                    targets.append(BaseWeather)
                elif value == "AirHandlingUnit":
                    from trano.elements.system import AirHandlingUnit

                    targets.append(AirHandlingUnit)
                elif value == "Ventilation":
                    from trano.elements.system import Ventilation

                    targets.append(Ventilation)
                elif value == "BaseInternalElement":
                    from trano.elements.envelope import BaseInternalElement

                    targets.append(BaseInternalElement)
                elif value == "BaseOccupancy":
                    from trano.elements.system import BaseOccupancy

                    targets.append(BaseOccupancy)
                elif value == "Emission":
                    from trano.elements.system import Emission

                    targets.append(Emission)
                elif value == "VAVControl":
                    from trano.elements.control import VAVControl

                    targets.append(VAVControl)
                elif value == "BaseWall":
                    from trano.elements.envelope import BaseWall

                    targets.append(BaseWall)
                elif value == "ThreeWayValve":
                    from trano.elements.system import ThreeWayValve

                    targets.append(ThreeWayValve)
                elif value == "TemperatureSensor":
                    from trano.elements.system import TemperatureSensor

                    targets.append(TemperatureSensor)
                elif value == "Boundary":
                    from trano.elements.boundary import Boundary

                    targets.append(Boundary)
                else:
                    raise ValueError(f"Target {value} not found")
            else:
                targets.append(value)
        return targets

    def is_available(self) -> bool:
        return self.multi_connection or self.available

    def is_controllable(self) -> bool:
        from trano.elements.base import Control
        return self.targets is not None and any(
            target == Control for target in self.targets
        )

    def link(
        self, node: "BaseElement", connected_node: "BaseElement"
    ) -> list[PartialConnection]:

        from trano.elements.envelope import MergedBaseWall

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


def connection_color(edge: Tuple["BaseElement", "BaseElement"]) -> ConnectionView:
    from trano.elements.bus import DataBus
    from trano.elements.envelope import BaseSimpleWall
    from trano.elements.system import Weather

    if any(isinstance(e, BaseSimpleWall) for e in edge):
        return ConnectionView(color="{191,0,0}", thickness=0.1)
    if any(isinstance(e, DataBus) for e in edge):
        return ConnectionView(color=None, thickness=0.05)
    if any(isinstance(e, Weather) for e in edge):
        return ConnectionView(color=None, thickness=0.05)
    return ConnectionView()


def connect(edge: Tuple["BaseElement", "BaseElement"]) -> list[Connection]:
    connections = []
    edge_first = edge[0]
    edge_second = edge[1]
    edge_first_ports_to_skip: List[Port] = []
    edge_second_ports_to_skip: List[Port] = []
    while True:
        current_port = edge_first._get_target_compatible_port(
            edge_second, Flow.outlet, ports_to_skip=edge_first_ports_to_skip
        )
        other_port = edge_second._get_target_compatible_port(
            edge_first, Flow.inlet, ports_to_skip=edge_second_ports_to_skip
        )
        if any(port is None for port in [current_port, other_port]):
            break
        for left, right in zip(
            current_port.link(edge_first, edge_second),  # type: ignore
            other_port.link(edge_second, edge_first),  # type: ignore
        ):
            connections.append(
                Connection(
                    left=left, right=right, connection_view=connection_color(edge)
                )
            )
        edge_first_ports_to_skip.append(current_port)  # type: ignore
        edge_second_ports_to_skip.append(other_port)  # type: ignore
    return connections


def _has_inlet_or_outlet(target: "BaseElement") -> bool:
    return bool([port for port in target.ports if port.flow == Flow.inlet_or_outlet])


def _is_inlet_or_outlet(target: "BaseElement") -> bool:
    return bool(
        [port for port in target.ports if port.flow in [Flow.inlet, Flow.outlet]]
    )
