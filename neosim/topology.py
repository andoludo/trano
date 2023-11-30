import networkx as nx
from networkx import Graph
from uuid import uuid4

from neosim.model import Space, InternalElement


class Network:
    def __init__(self):
        self.graph = Graph()
        self.layout = {}

    def add_space(self, space: "Space"):
        self.graph.add_node(space)
        for boundary in space.external_boundaries:
            self.graph.add_node(boundary)
            self.graph.add_edge(space, boundary)

    def connect_spaces(
        self,
        space_1: "Space",
        space_2: "Space",
        internal_element: "InternalElement" = None,
    ):
        internal_element = internal_element or InternalElement(
            name=str(uuid4()), surface=10, azimuth=10, layer_name="layer", tilt=90
        )
        self.graph.add_node(internal_element)
        self.graph.add_edge(space_1, internal_element)
        self.graph.add_edge(space_2, internal_element)

    def merge_spaces(self, space_1: "Space", space_2: "Space"):
        internal_elements = nx.shortest_path(self.graph, space_1, space_2)[1:-1]
        merged_space = space_1 + space_2
        merged_space.internal_elements = internal_elements
        self.graph = nx.contracted_nodes(self.graph, merged_space, space_2)

    def generate_layout(self):
        self.layout = nx.circular_layout(self.graph)

    def get_position(self, element):
        return self.layout.get(element)
