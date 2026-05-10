from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field, field_validator

from trano import elements
from trano.elements.common_base import BaseElementPosition, BasePosition
from trano.elements.types import (
    ContainerTypes,
    Flow,
    Medium,
)

if TYPE_CHECKING:
    from trano.elements import BaseElement


class Port(BaseModel):
    names: list[str]
    targets: list[Any] = Field(default_factory=list)
    expected_ports: list[str] = Field(default_factory=list)
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
    no_check: bool = False

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
                        port.flow in [Flow.inlet_or_outlet] and port.multi_connection and port.use_counter
                    )
                if self.flow == Flow.outlet:
                    return port.flow in [Flow.inlet] or (
                        port.flow in [Flow.inlet_or_outlet] and port.multi_connection and port.use_counter
                    )
                if self.flow == Flow.inlet_or_outlet and self.multi_connection and self.use_counter:
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
            return Port.model_validate(self.model_dump(exclude={"counter"}) | {"counter": self.counter - 1})
        return self

    @field_validator("targets")
    @classmethod
    def validate_targets(cls, values: list[str]) -> list[type["BaseElement"]]:
        from trano.elements.base import BaseElement

        targets: list[type[BaseElement]] = []
        for value in values:
            if isinstance(value, str):
                if hasattr(elements, value):
                    targets.append(getattr(elements, value))
                else:
                    raise ValueError(f"Target {value} not found")
            else:
                targets.append(value)
        return targets

    def is_available(self, available_ports: list["Port"]) -> bool:
        if self.multi_connection and self.connected:
            if self.medium == Medium.fluid:
                return not any((p.medium == self.medium and self.similar_flow(p)) for p in available_ports)
            else:
                return True
        else:
            return not self.connected

    def is_controllable(self) -> bool:
        from trano.elements.base import Control

        return self.targets is not None and any(target == Control for target in self.targets)

    def base_equation(
        self,
        merged_number: int,
        element_name: str | None = None,
        element_position: BasePosition | None = None,
    ) -> list["BasePartialConnection"]:
        element_position = element_position or BasePosition()
        partial_connections = []
        for sub_port_number, name in enumerate(self.names):
            if self.multi_connection and self.bus_connection:
                first_counter = self.counter
                last_counter = self.counter + merged_number - 1
                counter = f"{first_counter}" if first_counter == last_counter else f"{first_counter}:{last_counter}"
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
                equation = f"{element_name}.{name}[{self.counter}]" if element_name else f"{name}[{self.counter}]"
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
        base_equations = self.base_equation(merged_number, element_name, element_position)

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


class BasePartialConnection(BaseElementPosition):
    name: str | None = None
    equation: str
    port: Port
    sub_port: int


class BasePartialConnectionWithContainerType(BasePartialConnection):
    container_type: ContainerTypes | None = None


class PartialConnection(BasePartialConnectionWithContainerType):
    connected_container_type: ContainerTypes | None = None

    def reset_port_counter(self) -> "PartialConnection":
        self.port.reset_counter()
        return self

    def to_base_partial_connection(self) -> BasePartialConnection:
        return BasePartialConnection.model_validate(
            self.model_dump(exclude={"connected_container_type", "container_type"})
        )
