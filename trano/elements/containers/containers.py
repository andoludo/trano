"""Containers: collection orchestrating cross-container connections."""

import logging
from typing import get_args

from jinja2 import Template
from pydantic import BaseModel, Field, model_validator

from trano.elements import BaseElement, Connection
from trano.elements.base import ElementPort
from trano.elements.common_base import BaseProperties, BasePosition
from trano.elements.connection import ContainerConnection, connect
from trano.elements.containers._env import ENVIRONMENT
from trano.elements.containers.container import Container
from trano.elements.containers.models import (
    BusConnection,
    ConnectionList,
    ContainerInput,
    MainContainerConnection,
)
from trano.elements.types import ConnectionView, ContainerTypes, SystemContainerTypes
from trano.exceptions import ContainerNotFoundError

logger = logging.getLogger(__name__)


class Containers(BaseModel):
    containers: list[Container]
    main: str | None = None
    connections: list[MainContainerConnection] = Field(default=[])
    bus_connections: list[BusConnection] = [
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
    connection_list: list[ConnectionList] = [
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
        from pathlib import Path

        import yaml

        config_yaml = Path(__file__).parents[2].joinpath("elements/config/containers.yaml")
        data = yaml.safe_load(config_yaml.read_text())
        return cls.model_validate(data)

    def _set_connection_annotation(self) -> None:
        for connection in self.connections:
            for connection_list in self.connection_list:
                if all(c in connection.get_equation() for c in connection_list.connection_type):
                    connection.annotation = connection_list.annotation

            if connection.annotation is None:
                raise ValueError(f"Connection {connection.get_equation()} not found in connection list.")

    @model_validator(mode="after")
    def _validate(self) -> "Containers":
        container_names = [container.name for container in self.containers]
        if len(container_names) != len(set(container_names)):
            raise ValueError("Containers must have unique names.")
        return self

    def build(self, container_input: ContainerInput) -> list[str]:
        self.assign_nodes(container_input.nodes)
        self.connect(container_input.connections)
        self.build_main_connections()
        self.add_data(container_input.data)
        template = self._template()
        main_template = self._main_template()
        self._set_connection_annotation()
        self.main = main_template.render(container=self)

        return [c.build(template, container_input.medium) for c in self.in_use_containers()]

    def build_main_connections(self) -> None:
        connections = [conn for c in self.containers for conn in c.connections if not conn.in_the_same_container()]
        couple_connections = {
            c.source: [c_.get_container_equation() for c_ in connections if c_.source == c.source] for c in connections
        }
        for equations in couple_connections.values():
            if len(equations) == 2:
                self.connections += [MainContainerConnection.from_list(equations)]  # type: ignore

    def _get_connection_view(self, connected_container_name: ContainerTypes | None = None) -> ConnectionView:
        connection_view = ConnectionView()
        if connected_container_name in get_args(SystemContainerTypes):
            connection_view = ConnectionView(color="{0, 0, 139}", thickness=0.1, pattern="Dash")
        if connected_container_name in ["bus"]:
            connection_view = ConnectionView(color=None, thickness=0.2, disabled=True)
        return connection_view

    def connect(self, connections: list[Connection]) -> None:
        for connection in connections:
            edge_left = connection.left
            edge_right = connection.right
            if edge_left.container_type == edge_right.container_type:
                container = self.get_container(edge_left.container_type)
                if not container:
                    raise ContainerNotFoundError(f"Container {edge_left.container_type} not found.")
                container.connections += [
                    ContainerConnection.model_validate(
                        connection.model_dump() | {"source": tuple(sorted([edge_right.equation, edge_left.equation]))}
                    )
                ]

                continue

            for edge_1, edge_2 in [(edge_left, edge_right), (edge_right, edge_left)]:
                container = self.get_container(edge_1.container_type)
                if container:
                    port_group = container.get_port_group(edge_2.container_type)
                    if port_group:
                        connection_view = self._get_connection_view(edge_2.container_type)
                        container_position = BasePosition()
                        container_position.set(0, 0)
                        element_1 = ElementPort(
                            name=edge_1.name,
                            ports=[
                                edge_1.port.without_targets().disconnect().set_ignore_direction().substract_counter()
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
                                c.model_copy(update={"connection_view": connection_view}).model_dump()
                                | {"source": tuple(sorted([edge_1.equation, edge_2.equation]))}
                            )
                            for c in connections_
                        ]

    def assign_nodes(self, nodes: list[BaseElement]) -> None:
        for container_type in get_args(ContainerTypes):
            container = self.get_container(container_type)
            node_types = [node for node in nodes if node.container_type == container_type]

            if container:
                container.elements.extend(node_types)

    def in_use_containers(self) -> list[Container]:
        return [c for c in self.containers if c.contain_elements()]

    def bus_equations(self) -> list[str]:
        containers = {container.name for container in self.in_use_containers()}
        return [
            bus_connection.equation()
            for bus_connection in self.bus_connections
            if set(bus_connection.connection_type).issubset(containers)
        ]

    def get_container(self, container_type: ContainerTypes | None = None) -> Container | None:
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
