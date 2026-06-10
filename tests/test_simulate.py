import tempfile
from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate
from trano.topology import Network
from trano.utils.utils import is_success

ONE_HOUR = 3600


def get_path(file_name: str) -> Path:
    return Path(__file__).parent.joinpath("models", file_name)


def simulate_in_temporary_directory(network: Network, options: SimulationOptions) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(Path(project_path), network, options=options)
        assert is_success(results, options=options)


# (network fixture name, simulation end time in seconds)
SIMULATED_FIXTURES: list[tuple[str, int]] = [
    ("buildings_free_float_single_zone", ONE_HOUR),
    ("buildings_two_rooms_with_storage", 24 * ONE_HOUR * 14),
    ("buildings_free_float_two_zones", ONE_HOUR),
    ("buildings_free_float_three_zones", ONE_HOUR),
    ("buildings_simple_hydronic", ONE_HOUR),
    ("ideas_free_float_three_zones", ONE_HOUR),
    ("ideas_simple_hydronic_no_occupancy", ONE_HOUR),
    ("space_1_different_construction_types_network", ONE_HOUR),
    ("ideas_simple_hydronic_three_zones", ONE_HOUR),
    ("one_spaces_air_handling_unit", ONE_HOUR),
    ("two_spaces_air_handling_unit", ONE_HOUR),
    ("house_model", ONE_HOUR),
    ("building_multiple_internal_walls", ONE_HOUR),
    ("building_multiple_internal_walls_ideas", ONE_HOUR),
    ("space_1_ideal_heating_network", ONE_HOUR),
]


@pytest.mark.simulate
@pytest.mark.parametrize(
    ("network_fixture", "end_time"), SIMULATED_FIXTURES, ids=[case[0] for case in SIMULATED_FIXTURES]
)
def test_simulate_fixture_network(request: pytest.FixtureRequest, network_fixture: str, end_time: int) -> None:
    network: Network = request.getfixturevalue(network_fixture)
    simulate_in_temporary_directory(network, SimulationOptions(end_time=end_time))


# (network name, yaml file, library name or None, check_only). check_only entries are
# models whose simulation fails for reasons still under investigation (see TODOs below).
YAML_SIMULATIONS: list[tuple[str, str, str | None, bool]] = [
    ("three_zones_hydronic", "three_zones_hydronic.yaml", "IDEAS", False),
    ("three_zones_hydronic", "three_zones_hydronic.yaml", "Buildings", False),
    ("single_zone_hydronic", "single_zone_hydronic.yaml", None, False),
    ("single_zone_hydronic_weather", "single_zone_hydronic_weather.yaml", None, False),
    # TODO: why does the simulation fail without check_only?
    (
        "single_zone_air_handling_unit_simple_vav_control",
        "single_zone_air_handling_unit_simple_vav_control.yaml",
        None,
        True,
    ),
    ("single_zone_air_handling_unit_complex_vav", "single_zone_air_handling_unit_complex_vav.yaml", None, False),
    # TODO: remove ducts here and investigate why the simulation fails without check_only.
    (
        "single_zone_air_handling_unit_without_vav_with_duct",
        "single_zone_air_handling_unit_without_vav_with_duct.yaml",
        None,
        True,
    ),
    ("house_infiltration_boiler", "house_infiltration_boiler.yaml", None, False),
    ("single_zone_hydronic_occupancy_from_data", "single_zone_hydronic_occupancy_from_data.yaml", None, True),
]


@pytest.mark.simulate
@pytest.mark.parametrize(
    ("model_name", "yaml_file", "library_name", "check_only"),
    YAML_SIMULATIONS,
    ids=[f"{case[0]}_{case[2]}" if case[2] else case[0] for case in YAML_SIMULATIONS],
)
def test_simulate_yaml_model(model_name: str, yaml_file: str, library_name: str | None, check_only: bool) -> None:
    library = Library.from_configuration(library_name) if library_name else None
    network = convert_network(model_name, get_path(yaml_file), library=library)
    simulate_in_temporary_directory(network, SimulationOptions(end_time=ONE_HOUR, check_only=check_only))


@pytest.mark.simulate
def test_simulate_house_complex() -> None:
    house = get_path("house_complex.yaml")
    network = convert_network("house_complex", house)
    project_path = Path(__file__).parent.joinpath("simulation")
    project_path.mkdir(parents=True, exist_ok=True)
    results = simulate(
        Path(project_path),
        network,
        options=SimulationOptions(end_time=24 * ONE_HOUR * 30 * 3),
    )
    assert is_success(results)
