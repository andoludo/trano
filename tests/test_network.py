from uuid import uuid4

from pydantic import BaseModel

from networkx import Graph, draw
import matplotlib.pyplot as plt
import networkx as nx


class Network:
    def __init__(self):
        self.graph = Graph()

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
            name=str(uuid4()), surface=10, azimuth=10, layer_name="layer"
        )
        self.graph.add_node(internal_element)
        self.graph.add_edge(space_1, internal_element)
        self.graph.add_edge(space_2, internal_element)

    def merge_spaces(self, space_1: "Space", space_2: "Space"):
        internal_elements = nx.shortest_path(self.graph, space_1, space_2)[1:-1]
        merged_space = space_1 + space_2
        merged_space.internal_elements = internal_elements
        self.graph = nx.contracted_nodes(self.graph, merged_space, space_2)


class Space(BaseModel):
    name: str
    volume: float | int
    height: float | int
    elevation: float | int
    external_boundaries: list["Wall"]
    internal_elements: list["Wall"] = None

    def __hash__(self):
        return hash(f"{self.name}-{self.volume}")

    def __add__(self, other: "Space") -> "Space":
        self.name = (
            f"merge_{self.name.replace('merge','')}_{other.name.replace('merge','')}"
        )
        self.volume = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self


class Wall(BaseModel):
    name: str
    surface: float | int
    azimuth: float | int
    layer_name: str

    def __hash__(self):
        return hash(f"{self.name}-{self.surface}")


class InternalElement(BaseModel):
    name: str
    surface: float | int
    azimuth: float | int
    layer_name: str

    def __hash__(self):
        return hash(f"{self.name}-{self.surface}")


def test_network():
    space_1 = Space(
        name="space_1",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            Wall(name="w1_1", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w2_1", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w3_1", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w4_1", surface=10, azimuth=10, layer_name="layer"),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            Wall(name="w1_2", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w2_2", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w3_2", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w4_2", surface=10, azimuth=10, layer_name="layer"),
        ],
    )
    space_3 = Space(
        name="space_3",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            Wall(name="w1_3", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w2_3", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w3_3", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w4_3", surface=10, azimuth=10, layer_name="layer"),
        ],
    )
    space_4 = Space(
        name="space_4",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            Wall(name="w1_4", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w2_4", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w3_4", surface=10, azimuth=10, layer_name="layer"),
            Wall(name="w4_4", surface=10, azimuth=10, layer_name="layer"),
        ],
    )
    network = Network()
    network.add_space(space_1)
    network.add_space(space_2)
    network.add_space(space_3)
    network.add_space(space_4)
    network.connect_spaces(space_1, space_2)
    network.connect_spaces(space_1, space_3)
    network.connect_spaces(space_1, space_4)
    network.connect_spaces(space_2, space_4)
    network.merge_spaces(space_1, space_2)
    draw(network.graph, with_labels=True)
    plt.draw()
    plt.show()
