import itertools
from pathlib import Path
from typing import Optional

import networkx as nx
from jinja2 import Environment, FileSystemLoader
from networkx import DiGraph, Graph

from neosim.construction import Constructions
from neosim.model import (
    Connection,
    Emission,
    InternalElement,
    Occupancy,
    Space,
    System,
    Tilt,
    Weather,
    connect,
)


class Network:
    def __init__(self, name: str) -> None:
        self.graph = DiGraph()
        self.edge_attributes = []
        self.name = name

    def add_space(self, space: "Space") -> None:
        self.graph.add_node(space)
        for boundary in space.external_boundaries:
            self.graph.add_node(boundary)
            self.graph.add_edge(
                space,
                boundary,
            )
        self._build_space_emission(space)  # TODO: perhaps move to space
        self._build_control(space)  # TODO: perhaps move to space
        self._build_occupancy(space)
        space.assign_position()

    def _build_space_emission(self, space: "Space"):
        emission = space.find_emission()
        if emission:
            self.graph.add_node(emission)
            self.graph.add_edge(
                space,
                emission,
            )
            for system1, system2 in zip(space.emissions[:-1], space.emissions[1:]):
                if not self.graph.has_node(system1):
                    self.graph.add_node(system1)
                if not self.graph.has_node(system2):
                    self.graph.add_node(system2)
                self.graph.add_edge(
                    system1,
                    system2,
                )

    def _build_control(self, space: "Space"):
        if space.control:
            self.graph.add_node(space.control)
            self.graph.add_edge(
                space.control,
                space,
            )
            self.graph.add_edge(space.control, space.get_controllable_emission())

    def _build_occupancy(self, space: "Space"):
        if space.occupancy:
            self.graph.add_node(space.occupancy)
            self.connect_system(space, space.occupancy)

    def connect_spaces(
        self,
        space_1: "Space",
        space_2: "Space",
        internal_element: Optional[
            "InternalElement"
        ] = None,  # TODO: this should not be optional
    ) -> None:
        internal_element = internal_element or InternalElement(
            name=f"internal_{space_1.name}_{space_2.name}",
            surface=10,
            azimuth=10,
            construction=Constructions.internal_wall,
            tilt=Tilt.wall,
        )
        internal_element.position = [space_1.position[0] +(space_2.position[0] -space_1.position[0])/2, space_1.position[1]]
        self.graph.add_node(internal_element)
        self.graph.add_edge(
            space_1,
            internal_element,
        )
        self.graph.add_edge(
            space_2,
            internal_element,
        )

    def connect_system(self, space: "Space", system: "System") -> None:
        self.graph.add_edge(
            space,
            system,
        )

    def connect_systems(self, system_1, system_2):
        if system_1 not in self.graph.nodes:
            self.graph.add_node(system_1)
        if system_2 not in self.graph.nodes:
            self.graph.add_node(system_2)
        self.graph.add_edge(system_1, system_2)

    def connect_edges(self, edge: tuple) -> list[Connection]:
        return connect(edge)

    def merge_spaces(self, space_1: "Space", space_2: "Space") -> None:
        internal_elements = nx.shortest_path(self.graph, space_1, space_2)[1:-1]
        merged_space = space_1 + space_2
        merged_space.internal_elements = internal_elements
        self.graph = nx.contracted_nodes(self.graph, merged_space, space_2)

    def generate_layout(self) -> dict:
        # nodes = [n for n in self.graph.nodes if isinstance(n, Space)]
        # for i, n in enumerate(nodes):
        #     n.assign_position([200*i, 50])

        return nx.spring_layout(self.graph, k=10, dim=2, scale=200)

    def generate_graphs(self) -> None:
        layout = self.generate_layout()
        for node in self.graph.nodes:
            node.get_position(layout)
            if isinstance(node, Space):
                node.get_neighhors(self.graph)
        for edge in self.graph.edges:
            self.edge_attributes += self.connect_edges(edge)

    def model(self) -> str:
        self.generate_graphs()
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
        )
        environment.filters["frozenset"] = frozenset

        template = environment.get_template("buildings.jinja2")
        return template.render(network=self)

    def add_boiler_plate_spaces(self, spaces: list[Space]) -> None:
        for space in spaces:
            self.add_space(space)
        for combination in itertools.combinations(spaces, 2):
            self.connect_spaces(*combination)
        weather = Weather(name="weather")
        self.graph.add_node(weather)
        for i, space in enumerate(spaces):
            self.connect_system(space, weather)
