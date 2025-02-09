from pathlib import Path
from random import randint
from typing import List, Optional, get_args, Callable, Any, Tuple

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from trano.elements import Port, Connection, Control
from trano.elements.base import find_port, ElementPort
from trano.elements.connection import ContainerConnection, PartialConnection, connect
from trano.elements.types import (
    ContainerTypes,
    Flow,
    ConnectionView,
    Medium,
)
from jinja2 import Environment, FileSystemLoader


class PortGroup(BaseModel):
    connected_container_name: ContainerTypes
    ports: List[Port]


import random
from networkx import DiGraph, shortest_path


class Container(BaseModel):
    name: ContainerTypes
    port_groups: List[PortGroup]
    connections: List[Connection] = Field(default=[])
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
                    self.template = self.template.replace(f"#{name}#", str(port.counter -1))

    def build(self, template):
        self._initialize_template()
        return template.render(container=self)

    def assign_position(self, graph):
        left_elements = []
        seen = set()
        postitions = []
        if self.left_boundary is None:
            for element in self.elements:
                element.container_position = [randint(-90, 90), randint(-90, 90)]

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
                element.container_position = [
                    -90 + xstep * (postitions.index(e)) + randint(0, 40),
                    -90 + ystep * (e.index(eid)) + randint(0, 40),
                ]
                if element.control:
                    element.control.container_position = [
                        element.container_position[0] + 20,
                        element.container_position[1] + 20,
                    ]


class MainContainerConnection(BaseModel):
    left: ContainerConnection
    right: ContainerConnection
    annotation: Optional[str] = None

    @classmethod
    def from_list(
        cls, connections: List[ContainerConnection]
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
    ]

    def _set_connection_annotation(self):
        for connection in self.connections:
            for connection_list in self.connection_list:
                if all([c in connection.get_equation() for c in connection_list.connection_type]):
                    connection.annotation = connection_list.annotation

            if connection.annotation is None:
                raise ValueError(f"Connection {connection.get_equation()} not found in connection list.")

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

    def add_connection_(self, connection: Connection, container_type) -> "Containers":
        container = self.get_container(container_type)
        if container:
            container.connections.append(connection)

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
            for edge_1, edge_2 in [(edge_left, edge_right), (edge_right, edge_left)]:
                container = self.get_container(edge_1.container_type)
                if container:
                    port_group = container.get_port_group(
                        edge_2.container_type
                    )
                    if port_group:
                        connection_view = self._get_connection_view(
                            edge_2.container_type
                        )
                        element_1 = ElementPort(name=edge_1.name,ports = [edge_1.port.without_targets().disconnect()], container_type=edge_1.container_type)
                        element_2 = ElementPort(ports = port_group.ports, container_type=edge_2.container_type)
                        connections = connect(element_1, element_2)
                        container.connections += connections
                        # container.connections.append(
                        #     Connection(
                        #         left=container_connection,
                        #         right=partial_connection.to_base_partial_connection(),
                        #         connection_view=connection_view,
                        #     )
                        # )
    # def add_connection(
    #     self, connections: List[Tuple[PartialConnection, PartialConnection]]
    # ) -> List[List[ContainerConnection]]:
    #
    #     if len(connections) == 1:
    #         container_connections_final = []
    #         for p in connections[0]:
    #             container_conection = self.add_single_connection(p)
    #             container_connections_final.append(container_conection)
    #         if all(container_connections_final):
    #             self.connections += [
    #                 MainContainerConnection.from_list(container_connections_final)
    #             ]
    #     else:
    #         container_connections_final = []
    #         # TODO: this is a lot of complication for nothing.
    #         # better avoid using multiple names in ports, will facilitate things
    #         port_same_categories = [list(group) for group in list(zip(*connections))]
    #         for p in port_same_categories:
    #             container = self._get_container(p[0].container_type)
    #             if not container:
    #                 continue
    #             port_group = container.get_port_group(p[0].connected_container_type)
    #             if port_group:
    #                 connection_view = self._get_connection_view(
    #                     p[0].connected_container_type
    #                 )
    #                 port_ = p[0].port
    #                 container_port = find_port(
    #                     port_.get_opposite_flow(), port_group.ports, [], [port_]
    #                 )
    #                 container_connections = container_port.base_equation()
    #                 current_group = []
    #                 for l, r in [
    #                     (p_, c)
    #                     for p_ in p
    #                     for c in container_connections
    #                     if p_.sub_port == c.sub_port
    #                 ]:
    #                     container.connections.append(
    #                         Connection(
    #                             left=l.to_base_partial_connection(),
    #                             right=r,
    #                             connection_view=connection_view,
    #                         )
    #                     )
    #                     cc = ContainerConnection.model_validate(
    #                         (r.model_dump() | {"container_type": container.name})
    #                     )
    #                     current_group.append(cc)
    #                 container_connections_final.append(current_group)
    #         container_connections_final = [
    #             list(group) for group in list(zip(*container_connections_final))
    #         ]
    #         if all(container_connections_final):
    #             self.connections += [
    #                 MainContainerConnection.from_list(cf)
    #                 for cf in container_connections_final
    #             ]

    def add_single_connection(
        self, partial_connection: PartialConnection
    ) -> ContainerConnection:
        container = self.get_container(partial_connection.container_type)
        if container:
            port_group = container.get_port_group(
                partial_connection.connected_container_type
            )
            if port_group:
                connection_view = self._get_connection_view(
                    partial_connection.connected_container_type
                )
                port_ = partial_connection.port
                container_port = find_port(
                    port_.get_opposite_flow(), port_group.ports, [], [port_]
                )
                container_connections = container_port.base_equation()
                right = partial_connection.to_base_partial_connection()
                container_connection = next(
                    c for c in container_connections if c.sub_port == right.sub_port
                )
                container.connections.append(
                    Connection(
                        left=container_connection,
                        right=partial_connection.to_base_partial_connection(),
                        connection_view=connection_view,
                    )
                )

                return ContainerConnection.model_validate(
                    (
                        container_connection.model_dump()
                        | {"container_type": container.name}
                    )
                )

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
    distribution_container = Container(
        left_boundary="emission",
        name="distribution",
        template="""
                  parameter Real mRad_flow_nominal = 123;


package MediumW = IDEAS.Media.Water "Medium model";
        replaceable package Medium=Modelica.Media.Interfaces.PartialMedium;
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{90,-60},{110,-40}}),
        iconTransformation(extent={{90,-60},{110,-40}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{90,40},{110,60}}),
        iconTransformation(extent={{90,40},{110,60}})));
  Modelica.Fluid.Interfaces.FluidPort_a[#port_a1#] port_a1(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{-110,-58},{-90,-38}}),
        iconTransformation(extent={{-110,-58},{-90,-38}})));
  Modelica.Fluid.Interfaces.FluidPort_b[#port_b1#] port_b1(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{-110,40},{-90,60}}),
        iconTransformation(extent={{-110,38},{-90,58}})));
          Trano.Controls.BaseClasses.DataBus dataBus annotation (Placement(
        transformation(extent={{-118,68},{-78,108}}), iconTransformation(extent
          ={{-228,58},{-208,78}})));
       """,
        port_groups=[
            PortGroup(
                connected_container_name="emission",
                ports=[
                    Port(names=["port_a1"], flow=Flow.outlet, multi_connection=True, medium=Medium.fluid),
                    Port(names=["port_b1"], flow=Flow.inlet, multi_connection=True, medium=Medium.fluid),
                ],
            ),
            PortGroup(
                connected_container_name="production",
                ports=[
                    Port(names=["port_a"], flow=Flow.outlet, medium=Medium.fluid),
                    Port(names=["port_b"], flow=Flow.inlet, medium=Medium.fluid),
                ],
            ),
            PortGroup(
                connected_container_name="bus",
                ports=[Port(names=["dataBus"], bus_connection=True, use_counter=False, medium=Medium.data, flow= Flow.undirected)],
            ),
        ],
    )
    emission_container = Container(
        left_boundary="envelope",
        name="emission",
        template="""
package MediumW = IDEAS.Media.Water "Medium model";
  Modelica.Fluid.Interfaces.FluidPort_a[#port_a#] port_a (redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{90,-60},{110,-40}}),
        iconTransformation(extent={{90,-60},{110,-40}})));
  Modelica.Fluid.Interfaces.FluidPort_b[#port_b#] port_b(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{90,40},{110,60}}),
        iconTransformation(extent={{90,40},{110,60}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortCon#] heatPortCon
    "Nodes for convective heat gains"
    annotation (Placement(transformation(extent={{-108,42},{-88,62}}),
        iconTransformation(extent={{-108,42},{-88,62}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortRad#] heatPortRad
    "Nodes for radiative heat gains"
    annotation (Placement(transformation(extent={{-110,-60},{-90,-40}}),
        iconTransformation(extent={{-110,-60},{-90,-40}})));
                  Trano.Controls.BaseClasses.DataBus dataBus annotation (Placement(
        transformation(extent={{-118,68},{-78,108}}), iconTransformation(extent
          ={{-228,58},{-208,78}})));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={255,128,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
         """,
        port_groups=[
            PortGroup(
                connected_container_name="distribution",
                ports=[
                    Port(names=["port_a"], flow=Flow.outlet, multi_connection=True, medium=Medium.fluid),
                    Port(names=["port_b"], flow=Flow.inlet, multi_connection=True, medium=Medium.fluid),
                ],
            ),
            PortGroup(
                connected_container_name="envelope",
                ports=[
                    Port(
                        names=[ "heatPortRad"],
                        multi_connection=True,
                        same_counter_per_name=True, medium=Medium.heat, flow= Flow.radiative
                    ),
                    Port(
                        names=["heatPortCon"],
                        multi_connection=True,
                        same_counter_per_name=True, medium=Medium.heat, flow= Flow.convective
                    )
                ],
            ),
            PortGroup(
                connected_container_name="bus",
                ports=[Port(names=["dataBus"], bus_connection=True, use_counter=False, medium=Medium.data, flow= Flow.undirected)],
            ),
        ],
    )
    envelope_container = Container(
        name="envelope",
        template="""
replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortCon#] heatPortCon
    "Nodes for convective heat gains"
    annotation (Placement(transformation(extent={{90,40},{110,60}}),
        iconTransformation(extent={{90,40},{110,60}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[#heatPortRad#] heatPortRad
    "Nodes for radiative heat gains"
    annotation (Placement(transformation(extent={{90,-62},{110,-42}}),
        iconTransformation(extent={{90,-62},{110,-42}})));
              three_zones_hydronic_containers.Trano.Controls.BaseClasses.DataBus
                                                 dataBus annotation (Placement(
    transformation(extent={{-120,52},{-80,92}}),  iconTransformation(extent
      ={{-228,58},{-208,78}})));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={255,128,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
             """,
        port_groups=[
            PortGroup(
                connected_container_name="emission",
                ports=[
                    Port(
                        names=[ "heatPortRad"],
                        multi_connection=True,
                        same_counter_per_name=True, medium=Medium.heat, flow= Flow.radiative
                    ),
                    Port(
                        names=["heatPortCon"],
                        multi_connection=True,
                        same_counter_per_name=True, medium=Medium.heat, flow= Flow.convective
                    ),
                    # TODO: this is always considered as one port?
                ],
            ),
            PortGroup(
                connected_container_name="bus",
                ports=[Port(names=["dataBus"], bus_connection=True, use_counter=False, medium=Medium.data, flow= Flow.undirected)],
            ),
        ],
    )
    production_container = Container(
        left_boundary="distribution",
        name="production",
        template="""
          parameter Real mRad_flow_nominal = 123;


package MediumW = IDEAS.Media.Water "Medium model";
        Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{-110,-58},{-90,-38}}),
        iconTransformation(extent={{-110,-58},{-90,-38}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{-110,40},{-90,60}}),
        iconTransformation(extent={{-110,38},{-90,58}})));
                      three_zones_hydronic_containers.Trano.Controls.BaseClasses.DataBus
                                                 dataBus annotation (Placement(
    transformation(extent={{-120,52},{-80,92}}),  iconTransformation(extent
      ={{-228,58},{-208,78}})));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={28,108,200},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
                 """,
        port_groups=[
            PortGroup(
                connected_container_name="distribution",
                ports=[
                    Port(names=["port_a1"], flow=Flow.outlet, medium=Medium.fluid),
                    Port(names=["port_b1"], flow=Flow.inlet, medium=Medium.fluid),
                ],
            ),
            PortGroup(
                connected_container_name="bus",
                ports=[Port(names=["dataBus"], bus_connection=True, use_counter=False, medium=Medium.data, flow= Flow.undirected)],
            ),
        ],
    )
    return Containers(
        containers=[
            distribution_container,
            emission_container,
            envelope_container,
            production_container,
        ]
    )
