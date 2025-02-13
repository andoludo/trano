import json
import logging
from pathlib import Path
from random import randint
from typing import List, Optional, get_args

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, Field, model_validator

from trano.elements import Port, Connection, Control
from trano.elements.base import ElementPort
from trano.elements.common_base import BasePosition
from trano.elements.connection import (
    BasePartialConnectionWithContainerType,
    PartialConnection,
    connect,
    ContainerConnection,
)
from trano.elements.types import (
    ContainerTypes,
    Flow,
    ConnectionView,
    Medium,
)

logger = logging.getLogger(__name__)


class PortGroup(BaseModel):
    connected_container_name: ContainerTypes
    ports: List[Port]


class PortGroupMedium(BaseModel):
    medium: Medium
    port: Port


class Container(BaseModel):
    name: ContainerTypes
    port_groups: List[PortGroup]
    port_groups_per_medium: List[PortGroupMedium] = Field(default=[])
    connections: List[ContainerConnection] = Field(default=[])
    elements: list = Field(default=[])
    element_models: list = Field(default=[])
    template: str
    size: List[List[float]] = [[-100, -100], [100, 100]]
    left_boundary: Optional[ContainerTypes] = None

    def get_equation_view(self):
        return {c.equation_view() for c in self.connections}

    @model_validator(mode="after")
    def _validate(self) -> "Container":
        if self.name in [
            port_group.connected_container_name for port_group in self.port_groups
        ]:
            raise ValueError(f"Container {self.name} cannot be connected to itself.")
        return self

    def get_port_group(
        self, connected_container_name: ContainerTypes
    ) -> Optional[PortGroup]:
        for port_group in self.port_groups:
            if port_group.connected_container_name == connected_container_name:
                return port_group
        return None

    def _initialize_template(self):
        for port_group in self.port_groups:
            for port in port_group.ports:
                for name in port.names:
                    self.template = self.template.replace(
                        f"#{name}#", str(port.counter - 1)
                    )

    def build(self, template):
        self._initialize_template()
        self.add_grouped_by_medium_connection()
        return template.render(container=self)

    def add_grouped_by_medium_connection(self):
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
                                    (
                                        c.model_dump()
                                        | {
                                            "source": tuple(
                                                sorted(
                                                    [c.right.equation, c.left.equation]
                                                )
                                            )
                                        }
                                    )
                                )
                                for c in connections
                            ]

    def assign_position(self, graph):
        left_elements = []
        seen = set()
        postitions = []
        if self.left_boundary is None:
            for element in self.elements:
                element.position.set_container(randint(-90, 90), randint(-90, 90))

        for element in self.elements:
            if self.left_boundary in {
                e.container_type
                for e in list(graph.successors(element))
                + list(graph.predecessors(element))
                if not isinstance(e, Control)
            }:
                left_elements.append(element)
        seen = seen | {hash(l) for l in left_elements}
        postitions.append(tuple({hash(l) for l in left_elements}))
        while True:
            new_e = [
                e_
                for e in left_elements
                for e_ in list(graph.successors(e)) + list(graph.predecessors(e))
                if e_.container_type == self.name and not isinstance(e_, Control)
            ]
            left_elements = [e_ for e_ in new_e if hash(e_) not in seen]
            if not left_elements:
                break
            seen = seen | {hash(l) for l in left_elements}
            postitions.append(tuple({hash(l) for l in left_elements}))
        xstep = 200 / (len(postitions))
        for e in postitions:
            if not e:
                continue
            ystep = 200 / (len(e))
            for eid in e:
                element = [el for el in self.elements if hash(el) == eid][0]
                element.position.set_container(
                    -90 + xstep * (postitions.index(e)) + randint(0, 40),
                    -90 + ystep * (e.index(eid)) + randint(0, 40),
                )
                if element.control:
                    element.control.position.set_container(
                        element.position.x_container + 20,
                        element.position.y_container + 20,
                    )


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
        return f"connect({self.left.container_type}1.{self.left.equation}, {self.right.container_type}1.{self.right.equation})"


class ConnectionList(BaseModel):
    connection_type: List[str]
    annotation: str


class Containers(BaseModel):
    containers: List[Container]
    main: Optional[str] = None
    connections: List[MainContainerConnection] = Field(default=[])
    connection_list: List[ConnectionList] = [
        ConnectionList(
            connection_type=["envelope1.heatPortCon", "emission1.heatPortCon"],
            annotation="""annotation (Line(
          points={{-43.8,15.2},{-52,15.2},{-52,20},{-60,20},{-60,15},{-64,15}},
          color={191,0,0}));""",
        ),
        ConnectionList(
            connection_type=["emission1.heatPortRad", "envelope1.heatPortRad"],
            annotation="""annotation (Line(
          points={{-44,5},{-60,5},{-60,4.8},{-64,4.8}}, color={191,0,0}));""",
        ),
        ConnectionList(
            connection_type=["distribution1.port_a", "production1.port_b1"],
            annotation="""annotation (Line(points={{16,
          5},{30,5},{30,14.8},{36,14.8}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["production1.port_a1", "distribution1.port_b"],
            annotation="""annotation (Line(points={{36,
          5.2},{32,5.2},{32,15},{16,15}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["distribution1.port_a1", "emission1.port_b"],
            annotation="""annotation (Line(points={{-4,
          5.2},{-18,5.2},{-18,12},{-20,12},{-20,15},{-24,15}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["emission1.port_a", "distribution1.port_b1"],
            annotation="""annotation (Line(points={{-24,
          5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["envelope1.ports_b", "bus1.port_b"],
            annotation="""annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));""",
        ),
        ConnectionList(
            connection_type=["envelope1.heatPortCon1", "bus1.heatPortCon"],
            annotation="""annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));""",
        ),
    ]
    @classmethod
    def load_from_config(cls):
        config = Path(__file__).parents[1].joinpath("elements/config/containers.json")
        return cls.model_validate(json.loads(config.read_text()))

    def _set_connection_annotation(self):
        for connection in self.connections:
            for connection_list in self.connection_list:
                if all(
                    [
                        c in connection.get_equation()
                        for c in connection_list.connection_type
                    ]
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

    def build(self):
        template = self._template()
        main_template = self._main_template()
        self._set_connection_annotation()
        self.main = main_template.render(container=self)

        return [c.build(template) for c in self.containers]

    def build_main_connections(self):
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
        for _, equations in couple_connections.items():
            if len(equations) == 2:
                self.connections += [MainContainerConnection.from_list(equations)]


    def _get_connection_view(
        self, connected_container_name: ContainerTypes
    ) -> ConnectionView:
        connection_view = ConnectionView()
        if connected_container_name in [
            "emission",
            "production",
            "envelope",
            "distribution",
        ]:
            connection_view = ConnectionView(
                color="{0, 0, 139}", thickness=0.1, pattern="Dash"
            )
        if connected_container_name in ["bus"]:
            connection_view = ConnectionView(color=None, thickness=0.2, disabled=True)
        return connection_view

    def connect(self, connections: List[Connection]):
        for connection in connections:
            edge_left = connection.left
            edge_right = connection.right
            if edge_left.container_type == edge_right.container_type:
                container = self.get_container(edge_left.container_type)
                container.connections += [
                    ContainerConnection.model_validate(
                        (
                            connection.model_dump()
                            | {
                                "source": tuple(
                                    sorted([edge_right.equation, edge_left.equation])
                                )
                            }
                        )
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
                                f"cannot be connected with container {element_2.container_type}"
                            )
                        container.connections += [
                            ContainerConnection.model_validate(
                                (
                                    c.model_copy(
                                        update={"connection_view": connection_view}
                                    ).model_dump()
                                    | {
                                        "source": tuple(
                                            sorted([edge_1.equation, edge_2.equation])
                                        )
                                    }
                                )
                            )
                            for c in connections_
                        ]
    def assign_nodes(self, nodes, graph) -> None:
        for container_type in get_args(ContainerTypes):
            container = self.get_container(container_type)
            node_types = [
                node for node in nodes if node.container_type == container_type
            ]

            if container:
                container.elements.extend(node_types)
                container.assign_position(graph)

    def assign_models(self, element_models) -> None:
        for container_type in get_args(ContainerTypes):
            container = self.get_container(container_type)
            if container:
                element_models_ = [
                    c.container
                    for e in container.elements
                    for c in element_models
                    if hash(e) == c.id
                ]

                container.element_models.extend(element_models_)

    def get_container(self, container_type: ContainerTypes) -> Optional[Container]:
        for container in self.containers:
            if container.name == container_type:
                return container
        return None

    def _template(self):
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(
                str(Path(__file__).parents[1].joinpath("templates"))
            ),
            autoescape=True,
        )
        environment.filters["frozenset"] = frozenset
        environment.filters["enumerate"] = enumerate
        template = environment.get_template("containers.jinja2")
        return template

    def _main_template(self):
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(
                str(Path(__file__).parents[1].joinpath("templates"))
            ),
            autoescape=True,
        )
        environment.filters["frozenset"] = frozenset
        environment.filters["enumerate"] = enumerate
        template = environment.get_template("main_container.jinja2")
        return template


def containers_factory() -> Containers:
    return Containers.load_from_config()


