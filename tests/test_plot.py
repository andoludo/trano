import docker
from buildingspy.io.outputfile import Reader

from neosim.models.elements.space import Space
from neosim.plot.plot import plot
from neosim.topology import Network
from tests.conftest import create_mos_file


def test_read(buildings_two_rooms_with_storage: Network) -> None:
    mat = Reader(
        "/home/aan/Documents/neosim/results/buildings_two_rooms_with_storage.building_res.mat",
        "openmodelica",
    )
    space = [
        s for s in buildings_two_rooms_with_storage.graph.nodes if isinstance(s, Space)
    ]
    plot(mat, space[0].figures[0])


def test_plot_buildings_simple_hydronic_two_zones_new(
    buildings_two_rooms_with_storage: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(
        buildings_two_rooms_with_storage, end_time=3600 * 24 * 7
    ) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
    assert results
