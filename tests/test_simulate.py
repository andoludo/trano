import docker

from neosim.topology import Network
from tests.conftest import create_mos_file, is_success


def test_simulate_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_free_float_single_zone) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_buildings_simple_hydronic_two_zones_new(
    buildings_two_rooms_with_storage: Network,
    container: docker.models.containers.Container,
) -> None:
    with create_mos_file(buildings_two_rooms_with_storage) as mos_file_name:
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


def test_simulate_one_spaces_air_handling_unit(
    one_spaces_air_handling_unit: Network, container: docker.models.containers.Container
) -> None:

    with create_mos_file(one_spaces_air_handling_unit) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)


def test_simulate_two_spaces_air_handling_unit(
    two_spaces_air_handling_unit: Network, container: docker.models.containers.Container
) -> None:
    with create_mos_file(two_spaces_air_handling_unit) as mos_file_name:
        results = container.exec_run(cmd=f"omc /neosim/tests/{mos_file_name}")
        assert is_success(results)
