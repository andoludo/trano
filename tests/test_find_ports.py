from pathlib import Path

from tests.test_validity import get_path
from trano.data_models.conversion import convert_network
from trano.elements import Space, connect, Radiator
from trano.elements.components import _load_components
from trano.elements.containers import containers_factory
from trano.elements.envelope import MergedExternalWall
from trano.library.library import AvailableLibraries, Library
from tests.fixtures.two_spaces import two_spaces
import pytest


def test_component_to_json():
    components = _load_components()



@pytest.mark.parametrize("library_name", ["IDEAS"])
def test_connect(schema: Path, library_name: str) -> None:
    house = get_path("three_zones_hydronic_containers.yaml")
    network = convert_network(
        "three_zones_hydronic_containers",
        house,
        library=Library.from_configuration(library_name),
    )
    edge = network.get_edge(Space, Radiator)
    connections = connect(containers_factory(), edge)
    assert len(connections) == 2
    assert ["radiator" in c.right.equation for c in connections]
