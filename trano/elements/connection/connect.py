"""connect(): pair compatible ports between two elements."""

from typing import TYPE_CHECKING

from trano.elements.connection.connection import Connection
from trano.elements.connection.port import Port
from trano.elements.types import ConnectionView, Medium
from trano.exceptions import ConnectionLimitReached

if TYPE_CHECKING:
    from trano.elements.base import ElementPort


def connection_color(edge: tuple["ElementPort", "ElementPort"]) -> ConnectionView:
    from trano.elements.base import Control
    from trano.elements.bus import DataBus
    from trano.elements.envelope import BaseSimpleWall
    from trano.elements.system import System, Weather

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


def connect(edge_first: "ElementPort", edge_second: "ElementPort") -> list["Connection"]:
    connections = []
    try:
        for first_port in edge_first.ports:
            for second_port in edge_second.ports:
                available = first_port.is_available(edge_first.available_ports()) and second_port.is_available(
                    edge_second.available_ports()
                )
                has_targets = edge_second.has_target(first_port.targets) and edge_first.has_target(second_port.targets)
                complementarity = first_port.is_complementary(second_port)
                flow_direction = check_flow_direction(first_port, second_port)
                if available and has_targets and complementarity and flow_direction:
                    merged_number = max(edge_first.merged_number, edge_second.merged_number)
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
                    allowed_connections = edge_first.get_connection_per_target(edge_second.element_type)
                    if current_connection_numbers >= allowed_connections:
                        raise ConnectionLimitReached
    except ConnectionLimitReached:
        ...

    return connections
