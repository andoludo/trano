from pathlib import Path

import docker
import pytest

from neosim.topology import Network
from tests.conftest import create_mos_file, is_success


@pytest.fixture(scope="session")
def client() -> docker.DockerClient:
    client = docker.DockerClient(base_url="unix://var/run/docker.sock")
    return client


@pytest.fixture(scope="session")
def container(client: docker.DockerClient) -> None:
    container = client.containers.run(
        "openmodelica/openmodelica:v1.22.4-ompython",
        command="tail -f /dev/null",
        volumes=[f"{str(Path(__file__).parents[1])}:/neosim"],
        detach=True,
    )
    container.exec_run(cmd="omc /neosim/neosim/library/install_package.mos")
    yield container
    container.stop()
    container.remove()


def test_simulate_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_free_float_single_zone) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_free_float_two_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_buildings_free_float_three_zones(
    buildings_free_float_three_zones: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_free_float_three_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_buildings_simple_hydronic(
    buildings_simple_hydronic: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_simple_hydronic) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_buildings_simple_hydronic_two_zones(
    buildings_simple_hydronic_two_zones: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_simple_hydronic_two_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_ideas_free_float_three_zones(
    ideas_free_float_three_zones: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(ideas_free_float_three_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_ideas_simple_hydronic_no_occupancy(
    ideas_simple_hydronic_no_occupancy: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(ideas_simple_hydronic_no_occupancy) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_test_space_1_different_construction_types_network(
    space_1_different_construction_types_network: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(space_1_different_construction_types_network) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_ideas_simple_hydronic_three_zones(
    ideas_simple_hydronic_three_zones: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(ideas_simple_hydronic_three_zones) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_ideas_many_spaces_simple_ventilation(
    ideas_many_spaces_simple_ventilation: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(ideas_many_spaces_simple_ventilation) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_many_spaces_simple_ventilation(
    many_spaces_simple_ventilation: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(many_spaces_simple_ventilation) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_buildings_free_float_single_zone_complex(
    buildings_free_float_single_zone_ahu_complex: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_free_float_single_zone_ahu_complex) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)
