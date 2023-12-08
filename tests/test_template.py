from pathlib import Path

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel
import networkx as nx

from neosim.model import Space, Window


def test_space(building_1):
    building_1.generate_graphs()
    environment = Environment(
        trim_blocks=True,
        lstrip_blocks=True,
        loader=FileSystemLoader(
            str(Path(__file__).parents[1].joinpath("neosim", "templates"))
        )
    )

    template = environment.get_template("buildings.jinja2")
    content = template.render(network=building_1)
    a = 12
# TODO: NConeXtWin value needs to be provided always
# TODO: tilt in radiant (1.57)

def test_template(network):
    space = list(network.graph.nodes)[0]
    space.get_neighhors(network.graph)
    environment = Environment(
        loader=FileSystemLoader(
            str(Path(__file__).parents[1].joinpath("neosim", "templates"))
        )
    )

    template = environment.get_template("space.mo")
    content = template.render(space=space)
    assert content

class Connection(BaseModel):
    type: str
def test_connection(network):
    edges = nx.get_edge_attributes(network.graph, 'data')

    environment = Environment(
        loader=FileSystemLoader(
            str(Path(__file__).parent)
        )
    )

    template = environment.get_template("template.jinja2")
    content = template.render(edges=list(edges.values()))
    a = 12

def test_construction(network):
    nodes = list(network.graph.nodes)
    environment = Environment(
        loader=FileSystemLoader(
            str(Path(__file__).parent)
        )
    )
    nodes = [node for node in nodes if not isinstance(node, (Space, Window))]
    template = environment.get_template("construction.jinja2")
    content = template.render(nodes=nodes)
    a = 12

def test_window(network):
    nodes = list(network.graph.nodes)
    environment = Environment(
        loader=FileSystemLoader(
            str(Path(__file__).parent)
        )
    )
    nodes = [node for node in nodes if  isinstance(node,  Window)]
    template = environment.get_template("window.jinja2")
    content = template.render(nodes=nodes)
    a = 12