from pathlib import Path

from jinja2 import Environment, FileSystemLoader


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
