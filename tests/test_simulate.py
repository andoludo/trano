import tempfile
from pathlib import Path

from neosim.simulate.simulate import SimulationOptions, simulate
from neosim.topology import Network
from tests.conftest import is_success


def test_simulate_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            buildings_free_float_single_zone,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_buildings_simple_hydronic_two_zones_new(
    buildings_two_rooms_with_storage: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            buildings_two_rooms_with_storage,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            buildings_free_float_two_zones,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_buildings_free_float_three_zones(
    buildings_free_float_three_zones: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            buildings_free_float_three_zones,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_buildings_simple_hydronic(
    buildings_simple_hydronic: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            buildings_simple_hydronic,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_ideas_free_float_three_zones(
    ideas_free_float_three_zones: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            ideas_free_float_three_zones,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_ideas_simple_hydronic_no_occupancy(
    ideas_simple_hydronic_no_occupancy: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            ideas_simple_hydronic_no_occupancy,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_test_space_1_different_construction_types_network(
    space_1_different_construction_types_network: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            space_1_different_construction_types_network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_ideas_simple_hydronic_three_zones(
    ideas_simple_hydronic_three_zones: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            ideas_simple_hydronic_three_zones,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_one_spaces_air_handling_unit(
    one_spaces_air_handling_unit: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            one_spaces_air_handling_unit,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_two_spaces_air_handling_unit(
    two_spaces_air_handling_unit: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            two_spaces_air_handling_unit,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_house_model(house_model: Network) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path), house_model, options=SimulationOptions(end_time=3600)
        )
        assert is_success(results)


def test_simulate_building_multiple_internal_walls(
    building_multiple_internal_walls: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            building_multiple_internal_walls,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_building_multiple_internal_walls_ideas(
    building_multiple_internal_walls_ideas: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            building_multiple_internal_walls_ideas,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_space_1_ideal_heating_network(
    space_1_ideal_heating_network: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            space_1_ideal_heating_network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)
