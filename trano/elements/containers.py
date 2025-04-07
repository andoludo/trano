import logging
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, get_args, Tuple, cast, Set

import yaml
from jinja2 import Environment, FileSystemLoader, Template
from pydantic import BaseModel, Field, model_validator, computed_field, field_validator

from trano.elements import Port, Connection, BaseElement
from trano.elements.base import ElementPort
from trano.elements.common_base import (
    BasePosition,
    Point,
    BaseProperties,
    MediumTemplate,
)
from trano.elements.connection import (
    BasePartialConnectionWithContainerType,
    PartialConnection,
    connect,
    ContainerConnection,
)
from trano.elements.types import (
    ContainerTypes,
    ConnectionView,
    Medium,
    SystemContainerTypes,
)
from trano.elements.utils import wrap_with_raw
from trano.exceptions import ContainerNotFoundError

logger = logging.getLogger(__name__)
ENVIRONMENT = Environment(
    trim_blocks=True,
    lstrip_blocks=True,
    loader=FileSystemLoader(str(Path(__file__).parents[1].joinpath("templates"))),
    autoescape=True,
)
ENVIRONMENT.filters["frozenset"] = frozenset
ENVIRONMENT.filters["enumerate"] = enumerate


class PortGroup(BaseModel):
    connected_container_names: List[ContainerTypes]
    ports: List[Port]


class PortGroupMedium(BaseModel):
    medium: Medium
    port: Port


class BaseContainer(BaseModel):
    component_size: Point = Field(default=Point(x=20, y=20))


class ContainerInput(BaseModel):
    nodes: List[BaseElement]
    connections: List[Connection]
    data: BaseProperties
    medium: MediumTemplate


class ContainerLayout(BaseModel):
    bottom_left: Point = Point(x=-100, y=-100)
    top_right: Point = Point(x=100, y=100)
    global_origin: Point

    @computed_field  # type: ignore[prop-decorator]
    @property
    def scale(self) -> int:
        scale_x = int(self.top_right.c_.x - self.bottom_left.c_.x)
        scale_y = int(self.top_right.c_.y - self.bottom_left.c_.y)
        if scale_x != scale_y:
            raise ValueError("Scale x and y must be equal")
        return scale_x


class Container(BaseContainer):
    name: ContainerTypes
    port_groups: List[PortGroup]
    port_groups_per_medium: List[PortGroupMedium] = Field(default_factory=list)
    connections: List[ContainerConnection] = Field(default_factory=list)
    elements: List[BaseElement] = Field(default_factory=list)
    template: str
    left_boundary: Optional[ContainerTypes] = None
    data: Optional[BaseProperties] = None
    layout: ContainerLayout = Field(
        default_factory=lambda: ContainerLayout(global_origin=Point(x=0, y=0))
    )
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

    def get_equation_view(self) -> Set[Tuple[str, ...]]:
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
            container_name
            for port_group in self.port_groups
            for container_name in port_group.connected_container_names
        ]:
            raise ValueError(f"Container {self.name} cannot be connected to itself.")
        return self

    def get_port_group(
        self, connected_container_name: Optional[ContainerTypes] = None
    ) -> Optional[PortGroup]:
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
                        position.set(
                            element.position.x_container, element.position.y_container
                        )
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
                                    c.model_dump()
                                    | {
                                        "source": tuple(
                                            sorted([c.right.equation, c.left.equation])
                                        )
                                    }
                                )
                                for c in connections
                            ]


class MainContainerConnection(BaseModel):
    left: PartialConnection
    right: PartialConnection
    annotation: Optional[str] = None

    @classmethod
    def from_list(
        cls, connections: List[BasePartialConnectionWithContainerType]
    ) -> "MainContainerConnection":
        return cls(left=connections[0], right=connections[1])

    def get_equation(self) -> str:
        return (
            f"connect({self.left.container_type}1.{self.left.equation}, "
            f"{self.right.container_type}1.{self.right.equation})"
        )


class Location(BaseModel):
    point_1: Point
    point_2: Point


class ConnectionList(BaseModel):
    connection_type: List[str]
    annotation: str


class BusConnection(BaseModel):
    connection_type: Tuple[str, str]
    location: str

    @model_validator(mode="after")
    def _validator(self) -> "BusConnection":
        self.connection_type = cast(
            Tuple[str, str], tuple(sorted(self.connection_type))
        )

        return self

    def equation(self) -> str:
        return (
            f"connect({self.connection_type[0]}1.dataBus, {self.connection_type[1]}1.dataBus) "
            f"annotation (Line(points={self.location}, color={{255,204,51}}, thickness=0.5));"
        )


class Containers(BaseModel):
    containers: List[Container]
    main: Optional[str] = None
    connections: List[MainContainerConnection] = Field(default=[])
    bus_connections: List[BusConnection] = [
        BusConnection(
            connection_type=("bus", "envelope"),
            location="{{-83.8,48.8},{-83.8,56},{-60,56},{-60,26},{-74,26},{-74,20}}",
        ),
        BusConnection(
            connection_type=("emission", "envelope"),
            location="{{-95.8,16.8},{-95.8,26},{-55.8,26},{-55.8,16.8}}",
        ),
        BusConnection(
            connection_type=("distribution", "envelope"),
            location="{{-95.8,16.8},{-95.8,26},{-15.8,26},{-15.8,16.8}}",
        ),
        BusConnection(
            connection_type=("production", "envelope"),
            location="{{-95.8,16.8},{-95.8,26},{24.2,26},{24.2,16.8}}",
        ),
        BusConnection(
            connection_type=("ventilation", "envelope"),
            location="{{-44.1,-32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}}",
        ),
    ]
    connection_list: List[ConnectionList] = [
        ConnectionList(
            connection_type=["envelope1.heatPortCon", "emission1.heatPortCon"],
            annotation="""annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));""",
        ),
        ConnectionList(
            connection_type=["emission1.heatPortRad", "envelope1.heatPortRad"],
            annotation="""annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));""",
        ),
        ConnectionList(
            connection_type=["distribution1.port_a", "production1.port_b1"],
            annotation="""annotation (Line(points={{15.8,15},{30,15},{30,15},{36,15}},
                                          color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["production1.port_a1", "distribution1.port_b"],
            annotation="""annotation (Line(points={{16,5},{32,5},{32,5.2},{36,5.2}},
                                          color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["distribution1.port_a1", "emission1.port_b"],
            annotation="""annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["emission1.port_a", "distribution1.port_b1"],
            annotation="""annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["envelope1.ports_b", "bus1.port_b"],
            annotation="""annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["envelope1.y", "bus1.u"],
            annotation="""annotation (Line(points={{-85.7,10.1},{-94,10.1},
          {-94,40},{-85.8,40}}, color={0,0,127}));""",
        ),
        ConnectionList(
            connection_type=["envelope1.heatPortCon1", "bus1.heatPortCon"],
            annotation="""annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["envelope1.ports_a", "ventilation1.ports_b"],
            annotation="""annotation (Line(points={{-84.1,
          3.4},{-88,3.4},{-88,-14},{-48,-14},{-48,-20.2},{-43.9,-20.2}}, color={
          0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["ventilation1.ports_a", "envelope1.ports_b"],
            annotation="""annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["ventilation1.ports_b", "envelope1.ports_b"],
            annotation="""annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));""",
        ),
    ]

    def add_data(self, data: BaseProperties) -> None:
        for container in self.containers:
            container.set_data(data)

    @classmethod
    def load_from_config(cls) -> "Containers":
        config_yaml = (
            Path(__file__).parents[1].joinpath("elements/config/containers.yaml")
        )
        data = yaml.safe_load(config_yaml.read_text())
        return cls.model_validate(data)

    def _set_connection_annotation(self) -> None:
        for connection in self.connections:
            for connection_list in self.connection_list:
                if all(
                    c in connection.get_equation()
                    for c in connection_list.connection_type
                ):
                    connection.annotation = connection_list.annotation

            if connection.annotation is None:
                raise ValueError(
                    f"Connection {connection.get_equation()} not found in connection list."
                )

    @model_validator(mode="after")
    def _validate(self) -> "Containers":
        container_names = [container.name for container in self.containers]
        if len(container_names) != len(set(container_names)):
            raise ValueError("Containers must have unique names.")
        return self

    def build(self, container_input: ContainerInput) -> List[str]:
        self.assign_nodes(container_input.nodes)
        self.connect(container_input.connections)
        self.build_main_connections()
        self.add_data(container_input.data)
        template = self._template()
        main_template = self._main_template()
        self._set_connection_annotation()
        self.main = main_template.render(container=self)

        return [
            c.build(template, container_input.medium) for c in self.in_use_containers()
        ]

    def build_main_connections(self) -> None:
        connections = [
            conn
            for c in self.containers
            for conn in c.connections
            if not conn.in_the_same_container()
        ]
        couple_connections = {
            c.source: [
                c_.get_container_equation()
                for c_ in connections
                if c_.source == c.source
            ]
            for c in connections
        }
        for equations in couple_connections.values():
            if len(equations) == 2:
                self.connections += [MainContainerConnection.from_list(equations)]  # type: ignore

    def _get_connection_view(
        self, connected_container_name: Optional[ContainerTypes] = None
    ) -> ConnectionView:
        connection_view = ConnectionView()
        if connected_container_name in get_args(SystemContainerTypes):
            connection_view = ConnectionView(
                color="{0, 0, 139}", thickness=0.1, pattern="Dash"
            )
        if connected_container_name in ["bus"]:
            connection_view = ConnectionView(color=None, thickness=0.2, disabled=True)
        return connection_view

    def connect(self, connections: List[Connection]) -> None:
        for connection in connections:
            edge_left = connection.left
            edge_right = connection.right
            if edge_left.container_type == edge_right.container_type:
                container = self.get_container(edge_left.container_type)
                if not container:
                    raise ContainerNotFoundError(
                        f"Container {edge_left.container_type} not found."
                    )
                container.connections += [
                    ContainerConnection.model_validate(
                        connection.model_dump()
                        | {
                            "source": tuple(
                                sorted([edge_right.equation, edge_left.equation])
                            )
                        }
                    )
                ]

                continue

            for edge_1, edge_2 in [(edge_left, edge_right), (edge_right, edge_left)]:
                container = self.get_container(edge_1.container_type)
                if container:
                    port_group = container.get_port_group(edge_2.container_type)
                    if port_group:
                        connection_view = self._get_connection_view(
                            edge_2.container_type
                        )
                        container_position = BasePosition()
                        container_position.set(0, 0)
                        element_1 = ElementPort(
                            name=edge_1.name,
                            ports=[
                                edge_1.port.without_targets()
                                .disconnect()
                                .set_ignore_direction()
                                .substract_counter()
                            ],
                            container_type=edge_1.container_type,
                            position=edge_1.position,
                        )
                        element_2 = ElementPort(
                            ports=port_group.ports,
                            container_type=edge_1.container_type,
                            position=container_position,
                        )
                        connections_ = connect(element_1, element_2)
                        if not connections_:
                            logger.debug(
                                f"Element {element_1.name} from container {element_1.container_type} "
                                f"cannot be connected with container {edge_2.container_type}"
                            )
                        container.connections += [
                            ContainerConnection.model_validate(
                                c.model_copy(
                                    update={"connection_view": connection_view}
                                ).model_dump()
                                | {
                                    "source": tuple(
                                        sorted([edge_1.equation, edge_2.equation])
                                    )
                                }
                            )
                            for c in connections_
                        ]

    def assign_nodes(self, nodes: List[BaseElement]) -> None:
        for container_type in get_args(ContainerTypes):
            container = self.get_container(container_type)
            node_types = [
                node for node in nodes if node.container_type == container_type
            ]

            if container:
                container.elements.extend(node_types)

    def in_use_containers(self) -> List[Container]:
        return [c for c in self.containers if c.contain_elements()]

    def bus_equations(self) -> List[str]:
        containers = {container.name for container in self.in_use_containers()}
        return [
            bus_connection.equation()
            for bus_connection in self.bus_connections
            if set(bus_connection.connection_type).issubset(containers)
        ]

    def get_container(
        self, container_type: Optional[ContainerTypes] = None
    ) -> Optional[Container]:
        for container in self.containers:
            if container.name == container_type:
                return container
        return None

    def _template(self) -> Template:
        template = ENVIRONMENT.get_template("containers.jinja2")
        return template

    def _main_template(self) -> Template:
        template_ = """
model building

{% for container_ in container.in_use_containers() %}
{{ container_.main_equation() | safe }}
{% endfor %}
{% raw %}
Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p
annotation (Placement(transformation(extent={{-126,-18},{-92,18}}),
iconTransformation(
extent={{-112,-12},{-88,12}})));
equation
connect(term_p, bus1.term_p) annotation (Line(points={{-109,0},{-88,0},
        {-88,-10},{60,-10},{60,64},{-50,64},{-50,40},{-65,40}}, color={
        0,120,120}));
{% endraw %}
{% for connection in container.connections %}
connect({{ connection.left.container_type }}1.{{ connection.left.equation }},
{{ connection.right.container_type }}1.{{ connection.right.equation }})
{{ connection.annotation }}
{% endfor %}

{% for bus_equation in container.bus_equations() %}
{{ bus_equation | safe }}
{% endfor %}
{% for container_ in container.in_use_containers() %}
{{ container_.prescribed_connection_equation | safe }}
{% endfor %}
{% raw %}
annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
        Rectangle(
          extent={{-74,18},{22,-40}},
            fillColor={255,255,255},
            fillPattern=FillPattern.Forward,
            pattern=LinePattern.None,
            lineColor={238,46,47}),
        Rectangle(
          extent={{-62,2},{-38,-16}},
          lineColor={238,46,47},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-14,2},{8,-16}},
          lineColor={238,46,47},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
        Polygon(
          points={{-78,18},{26,18},{10,46},{-66,46},{-78,18}},
            lineColor={238,46,47},
            lineThickness=0.5,
            fillColor={244,125,35},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-60,42},{-68,22},{4,22},{6,42},{-60,42}},
            lineThickness=0.5,
            fillColor={28,108,200},
            fillPattern=FillPattern.Forward,
            pattern=LinePattern.None),
          Rectangle(
            extent={{26,0},{40,-40}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                  Diagram(
coordinateSystem(preserveAspectRatio=false)));
{% endraw %}
end building;
"""
        template = ENVIRONMENT.from_string(template_)
        return template


def containers_factory() -> Containers:
    return Containers.load_from_config()
