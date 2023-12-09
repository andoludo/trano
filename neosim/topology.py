import random
import string
from collections import Counter
from pathlib import Path
from typing import Optional

import networkx as nx
from jinja2 import Environment, FileSystemLoader
from networkx import Graph
from pydantic import BaseModel

from neosim.construction import Constructions
from neosim.model import (
    BaseWall,
    InternalElement,
    Occupancy,
    Space,
    System,
    Tilt,
    Weather,
)


class Edge(BaseModel):
    space: str
    other: str
    edge: str
    space_counter: int
    other_counter: int
    path: list

    @classmethod
    def from_edge(
        cls, edge: Space | BaseWall | System, counter: Counter, layout: dict
    ) -> "Edge":
        space = [e for e in edge if type(e).__name__ == "Space"][0]
        other = [e for e in edge if type(e).__name__ != "Space"][0]
        space.get_position(layout)
        other.get_position(layout)
        mid_path = (space.position[0] + other.position[0]) / 2
        path = [
            space.position,
            (space.position[0] + mid_path, space.position[1]),
            (other.position[0] - mid_path, other.position[1]),
            other.position,
        ]
        return cls(
            space=space.name,
            other=other.name,
            edge=f"{type(space).__name__}_{type(other).__name__}",
            space_counter=counter.get(_get_node_id(space, edge)),
            other_counter=counter.get(_get_node_id(other, edge)),
            path=path,
        )


counter = Counter()


def _get_node_id(
    node: Space | BaseWall | System, edge: Space | BaseWall | System
) -> str:
    suffix = _trailing_id(edge)
    return f"{node.__hash__()}_{suffix}"


def _trailing_id(edge: Space | BaseWall | System) -> str:
    return "_".join(sorted([type(edge_).__name__ for edge_ in edge]))


def random_name() -> str:
    return "".join(random.choice(string.ascii_lowercase) for i in range(16))


class Network:
    def __init__(self, name: str) -> None:
        self.graph = Graph()
        self.edge_attributes = None
        self.name = name

    def add_space(self, space: "Space") -> None:
        self.graph.add_node(space)
        for boundary in space.external_boundaries:
            space_id = (
                f"{space.__hash__()}_{type(space).__name__}_{type(boundary).__name__}"
            )
            boundary_id = f"{boundary.__hash__()}_{type(space).__name__}_{type(boundary).__name__}"
            counter.update(
                [
                    space_id,
                    boundary_id,
                ]
            )
            self.graph.add_node(boundary)
            self.graph.add_edge(
                space,
                boundary,
            )

    def connect_spaces(
        self,
        space_1: "Space",
        space_2: "Space",
        internal_element: Optional[
            "InternalElement"
        ] = None,  # TODO: this should not be optional
    ) -> None:
        internal_element = internal_element or InternalElement(
            name=str(random_name()),
            surface=10,
            azimuth=10,
            construction=Constructions.internal_wall,
            tilt=Tilt.wall,
        )
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

    def merge_spaces(self, space_1: "Space", space_2: "Space") -> None:
        internal_elements = nx.shortest_path(self.graph, space_1, space_2)[1:-1]
        merged_space = space_1 + space_2
        merged_space.internal_elements = internal_elements
        self.graph = nx.contracted_nodes(self.graph, merged_space, space_2)

    def generate_layout(self) -> dict:
        return nx.spring_layout(self.graph, dim=2, scale=200)

    def generate_graphs(self) -> None:
        layout = self.generate_layout()
        self.assign_edge_attributes(layout)
        for node in self.graph.nodes:
            node.get_position(layout)
            if isinstance(node, Space):
                node.get_neighhors(self.graph)

    def assign_edge_attributes(self, layout: dict) -> None:
        edge_attributes = []
        for edge in self.graph.edges:
            counter.update([_get_node_id(node, edge) for node in edge])
            edge_attributes.append(Edge.from_edge(edge, counter, layout))
        self.edge_attributes = edge_attributes

    def model(self) -> str:
        self.generate_graphs()
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parent.joinpath("templates"))),
        )

        template = environment.get_template("buildings.jinja2")
        return template.render(network=self)

    def add_boiler_plate_spaces(self, spaces: list[Space]) -> None:
        for space in spaces:
            self.add_space(space)
        weather = Weather(name="weather")
        self.graph.add_node(weather)
        for i, space in enumerate(spaces):
            occupancy = Occupancy(name=f"occupancy_{i}")
            self.graph.add_node(occupancy)
            self.connect_system(space, occupancy)
            self.connect_system(space, weather)
