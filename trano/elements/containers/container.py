"""Single Container: holds elements and their internal connections."""

from types import SimpleNamespace

from jinja2 import Template
from pydantic import Field, field_validator, model_validator

from trano.elements import BaseElement
from trano.elements.base import ElementPort
from trano.elements.common_base import BasePosition, BaseProperties, MediumTemplate, Point
from trano.elements.connection import ContainerConnection, connect
from trano.elements.containers._env import ENVIRONMENT
from trano.elements.containers.models import (
    BaseContainer,
    ContainerLayout,
    PortGroup,
    PortGroupMedium,
)
from trano.elements.types import ContainerTypes
from trano.elements.utils import wrap_with_raw


class Container(BaseContainer):
    name: ContainerTypes
    port_groups: list[PortGroup]
    port_groups_per_medium: list[PortGroupMedium] = Field(default_factory=list)
    connections: list[ContainerConnection] = Field(default_factory=list)
    elements: list[BaseElement] = Field(default_factory=list)
    template: str
    left_boundary: ContainerTypes | None = None
    data: BaseProperties | None = None
    layout: ContainerLayout = Field(default_factory=lambda: ContainerLayout(global_origin=Point(x=0, y=0)))
    prescribed_connection_equation: str = ""

    @field_validator("template")
    @classmethod
    def _template_validator(cls, value: str) -> str:
        return wrap_with_raw(value)

    def has_data(self) -> bool:
        return self.data is not None

    def set_data(self, data: BaseProperties) -> None:
        if self.name == data.container_type:
            self.data = data

    def contain_elements(self) -> bool:
        return bool(self.elements)

    def get_equation_view(self) -> set[tuple[str, ...]]:
        return {c.equation_view() for c in self.connections}

    def main_equation(self) -> str:
        location = f"{self.layout.global_origin.c_.x},{self.layout.global_origin.c_.y}"
        size = (
            f"{self.layout.global_origin.c_.x+self.component_size.c_.x},"
            f"{self.layout.global_origin.c_.y+self.component_size.c_.y}"
        )
        return (
            f"Components.Containers.{self.name} {self.name}1 "
            f"annotation (Placement(transformation(extent={{{{{location}}},"
            f"{{{size}}}}})));"
        )

    @model_validator(mode="after")
    def _validate(self) -> "Container":
        if self.name in [
            container_name for port_group in self.port_groups for container_name in port_group.connected_container_names
        ]:
            raise ValueError(f"Container {self.name} cannot be connected to itself.")
        return self

    def get_port_group(self, connected_container_name: ContainerTypes | None = None) -> PortGroup | None:
        for port_group in self.port_groups:
            if connected_container_name in port_group.connected_container_names:
                return port_group
        return None

    def _initialize_template(self, medium: MediumTemplate) -> None:
        ports = {}
        for port_group in self.port_groups:
            for port in port_group.ports:
                for name in port.names:
                    ports[name] = str(port.counter - 1)
        try:
            template = ENVIRONMENT.from_string(self.template)
        except:
            raise
        self.template = template.render(medium=medium, ports=SimpleNamespace(**ports))

    def build(self, template: Template, medium: MediumTemplate) -> str:
        self._initialize_template(medium)
        self.add_grouped_by_medium_connection()
        return template.render(container=self)

    def add_grouped_by_medium_connection(self) -> None:
        for group_per_medium in self.port_groups_per_medium:
            for element in self.elements:
                for element_port in element.ports:
                    if element_port.medium == group_per_medium.medium:
                        position = BasePosition()
                        position.set(element.position.x_container, element.position.y_container)
                        e1 = ElementPort(
                            ports=[element_port.without_targets()],
                            container_type=self.name,
                            position=position,
                            name=element.name,
                        )
                        e2 = ElementPort(
                            ports=[group_per_medium.port],
                            container_type=self.name,
                            position=position,
                        )
                        connections = connect(e1, e2)
                        if connections:
                            self.connections += [
                                ContainerConnection.model_validate(
                                    c.model_dump() | {"source": tuple(sorted([c.right.equation, c.left.equation]))}
                                )
                                for c in connections
                            ]
