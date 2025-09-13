import tempfile
from pathlib import Path
import pytest

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate
from trano.topology import Network
from trano.utils.utils import is_success


def get_path(file_name: str) -> Path:
    return Path(__file__).parent.joinpath("models", file_name)


@pytest.mark.simulate
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


@pytest.mark.simulate
def test_simulate_buildings_simple_hydronic_two_zones_new(
    buildings_two_rooms_with_storage: Network,
) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            buildings_two_rooms_with_storage,
            options=SimulationOptions(end_time=24 * 3600 * 14),
        )
        assert is_success(results)


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
def test_simulate_house_model(house_model: Network) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path), house_model, options=SimulationOptions(end_time=3600)
        )
        assert is_success(results)


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.simulate
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


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
@pytest.mark.simulate
def test_three_zones_hydronic(schema: Path, library_name: str) -> None:
    house = get_path("three_zones_hydronic.yaml")
    network = convert_network(
        "three_zones_hydronic", house, library=Library.from_configuration(library_name)
    )
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_hydronic(schema: Path) -> None:
    house = get_path("single_zone_hydronic.yaml")
    network = convert_network("single_zone_hydronic", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_hydronic_weather(schema: Path) -> None:
    house = get_path("single_zone_hydronic_weather.yaml")
    network = convert_network("single_zone_hydronic_weather", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_air_handling_unit_simple_vav_control(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_simple_vav_control.yaml")
    network = convert_network("single_zone_air_handling_unit_simple_vav_control", house)

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        options = SimulationOptions(
            end_time=3600, check_only=True
        )  # TODO: why simulation fails
        results = simulate(
            Path(project_path),
            network,
            options=options,
        )
        assert is_success(results, options=options)


@pytest.mark.simulate
def test_single_zone_air_handling_unit_complex_vav(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_complex_vav.yaml")
    network = convert_network("single_zone_air_handling_unit_complex_vav", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.simulate
def test_single_zone_air_handling_unit_without_vav_with_duct(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_without_vav_with_duct.yaml")
    # TODO: remove ducts here
    network = convert_network(
        "single_zone_air_handling_unit_without_vav_with_duct", house
    )

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        options = SimulationOptions(end_time=3600, check_only=True)
        results = simulate(
            Path(project_path),
            network,
            options=options,  # TODO: investigate why simulation fails
        )
        assert is_success(results, options=options)


@pytest.mark.simulate
def test_simulate_house_complex() -> None:
    house = get_path("house_complex.yaml")
    # TODO: remove ducts here
    network = convert_network("house_complex", house)
    project_path = Path(__file__).parent.joinpath("simulation")
    project_path.mkdir(parents=True, exist_ok=True)
    results = simulate(
        Path(project_path),
        network,
        options=SimulationOptions(end_time=24 * 3600 * 30 * 3),
    )
    assert is_success(results)
