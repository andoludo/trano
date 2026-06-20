from pathlib import Path

import pytest

from tests.fixtures.three_spaces import three_spaces
from tests.golden import assert_model_equals_golden
from trano.data_models.conversion import convert_network
from trano.elements import DataBus
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.topology import Network


def get_path(file_name: str, directory: str | None = None) -> Path:
    if directory:
        return Path(__file__).parent.joinpath("models", directory, file_name)
    return Path(__file__).parent.joinpath("models", file_name)


# Network fixtures whose generated model must match the golden file named after the network.
FIXTURE_CASES: list[str] = [
    "buildings_free_float_single_zone",
    "buildings_free_float_two_zones",
    "buildings_free_float_three_zones",
    "buildings_two_rooms_with_storage",
    "buildings_simple_hydronic",
    "buildings_simple_hydronic_three_zones",
    "ideas_free_float_single_zone",
    "ideas_free_float_three_zones",
    "ideas_simple_hydronic_three_zones",
    "ideas_simple_hydronic_no_occupancy",
    "space_1_ideal_heating_network",
    "space_1_different_construction_types_network",
    "one_spaces_air_handling_unit",
    "two_spaces_air_handling_unit",
    "space_with_door",
    "building_multiple_internal_walls",
    "building_multiple_internal_walls_ideas",
    "house_model",
]


@pytest.mark.parametrize("network_fixture", FIXTURE_CASES)
def test_fixture_network_matches_golden(request: pytest.FixtureRequest, network_fixture: str) -> None:
    network: Network = request.getfixturevalue(network_fixture)
    assert_model_equals_golden(network.model(), network.name)


# (network name, yaml file, library name or None, golden file name)
YAML_CASES: list[tuple[str, str, str | None, str]] = [
    ("three_zones_hydronic", "three_zones_hydronic.yaml", "IDEAS", "three_zones_hydronic_IDEAS_yaml"),
    ("three_zones_hydronic", "three_zones_hydronic.yaml", "Buildings", "three_zones_hydronic_Buildings_yaml"),
    ("single_zone_hydronic", "single_zone_hydronic.yaml", None, "single_zone_hydronic_yaml"),
    (
        "single_zone_hydronic_weather",
        "single_zone_hydronic_weather.yaml",
        None,
        "single_zone_hydronic_weather_yaml",
    ),
    (
        "single_zone_air_handling_unit_simple_vav_control",
        "single_zone_air_handling_unit_simple_vav_control.yaml",
        None,
        "single_zone_air_handling_unit_simple_vav_control_yaml",
    ),
    (
        "single_zone_air_handling_unit_complex_vav",
        "single_zone_air_handling_unit_complex_vav.yaml",
        None,
        "single_zone_air_handling_unit_complex_vav_yaml",
    ),
    ("two_zones", "two_zones.yaml", None, "two_zones_yaml"),
    ("two_zones_ideas", "two_zones_ideas.yaml", None, "two_zones_ideas_yaml"),
    # TODO: remove ducts here
    (
        "single_zone_air_handling_unit_without_vav_with_duct",
        "single_zone_air_handling_unit_without_vav_with_duct.yaml",
        None,
        "single_zone_air_handling_unit_without_vav_with_duct_yaml",
    ),
    (
        "three_zones_hydronic_container_IDEAS",
        "three_zones_hydronic_containers.yaml",
        "IDEAS",
        "three_zones_hydronic_container_IDEAS",
    ),
    (
        "three_zones_hydronic_container_Buildings",
        "three_zones_hydronic_containers.yaml",
        "Buildings",
        "three_zones_hydronic_container_Buildings",
    ),
    (
        "single_zone_air_handling_unit_complex_vav_containers_IDEAS",
        "single_zone_air_handling_unit_complex_vav_containers.yaml",
        "IDEAS",
        "single_zone_air_handling_unit_complex_vav_containers_IDEAS",
    ),
    (
        "single_zone_air_handling_unit_complex_vav_containers_Buildings",
        "single_zone_air_handling_unit_complex_vav_containers.yaml",
        "Buildings",
        "single_zone_air_handling_unit_complex_vav_containers_Buildings",
    ),
    (
        "three_zones_hydronic_reduced_orders_reduced_order",
        "three_zones_hydronic_reduced_orders.yaml",
        "reduced_order",
        "three_zones_hydronic_reduced_orders_reduced_order",
    ),
    (
        "three_zones_hydronic_reduced_orders_iso_13790",
        "three_zones_hydronic_reduced_orders.yaml",
        "iso_13790",
        "three_zones_hydronic_reduced_orders_iso_13790",
    ),
    (
        "three_zones_hydronic_reduced_orders_iso_13790_heat_pump",
        "three_zones_hydronic_reduced_orders_heat_pump.yaml",
        "iso_13790",
        "three_zones_hydronic_reduced_orders_iso_13790_heat_pump",
    ),
    (
        "three_zones_hydronic_reduced_orders_iso_13790_electric",
        "three_zones_hydronic_reduced_orders.yaml",
        "iso_13790",
        "three_zones_hydronic_reduced_orders_iso_13790_electric",
    ),
    (
        "three_zones_hydronic_reduced_orders_pv",
        "three_zones_hydronic_reduced_orders_pv.yaml",
        "iso_13790",
        "three_zones_hydronic_reduced_orders_pv",
    ),
    (
        "single_zone_hydronic_tilt_as_value",
        "single_zone_hydronic_tilt_as_value.yaml",
        "IDEAS",
        "single_zone_hydronic_tilt_as_value_yaml",
    ),
    (
        "house_multiple_construction_one_azimuth",
        "house_multiple_construction_one_azimuth.yaml",
        None,
        "house_multiple_construction_one_azimuth_yaml",
    ),
    ("house_complex", "house_complex.yaml", None, "house_complex"),
    ("house_infiltration_boiler", "house_infiltration_boiler.yaml", None, "house_infiltration_boiler"),
    (
        "house_infiltration_boiler_ideas",
        "house_infiltration_boiler.yaml",
        "IDEAS",
        "house_infiltration_boiler_ideas",
    ),
    (
        "house_infiltration_air_water_heat_pump",
        "house_infiltration_air_water_heat_pump.yaml",
        None,
        "house_infiltration_air_water_heat_pump",
    ),
    (
        "house_infiltration_air_water_heat_pump_ideas",
        "house_infiltration_air_water_heat_pump.yaml",
        "IDEAS",
        "house_infiltration_air_water_heat_pump_ideas",
    ),
    (
        "house_complex_dedicated_pumps",
        "house_complex_dedicated_pumps.yaml",
        None,
        "house_complex_dedicated_pumps",
    ),
    (
        "multizone_air_handling_unit_space_connected",
        "multizone_air_handling_unit_space_connected.yaml",
        None,
        "multizone_air_handling_unit_space_connected",
    ),
    (
        "multizone_air_handling_unit_space_connected_ideas",
        "multizone_air_handling_unit_space_connected.yaml",
        "IDEAS",
        "multizone_air_handling_unit_space_connected_ideas",
    ),
    ("house_complex_heat_meter", "house_complex_heat_meter.yaml", None, "house_complex_heat_meter"),
    (
        "single_zone_hydronic_occupancy_from_data",
        "single_zone_hydronic_occupancy_from_data.yaml",
        None,
        "single_zone_hydronic_occupancy_from_data",
    ),
    (
        "single_zone_hydronic_emission_from_data",
        "single_zone_hydronic_emission_from_data.yaml",
        None,
        "single_zone_hydronic_emission_from_data",
    ),
]


@pytest.mark.parametrize(
    ("model_name", "yaml_file", "library_name", "golden_name"),
    YAML_CASES,
    ids=[case[3] for case in YAML_CASES],
)
def test_yaml_network_matches_golden(
    model_name: str,
    yaml_file: str,
    library_name: str | None,
    golden_name: str,
) -> None:
    library = Library.from_configuration(library_name) if library_name else None
    network = convert_network(model_name, get_path(yaml_file), library=library)
    assert_model_equals_golden(network.model(), golden_name)


@pytest.mark.parametrize("library_name", [None, "IDEAS"])
def test_space_with_same_properties(space_with_same_properties: Space, library_name: str | None) -> None:
    network = Network(
        name="space_with_same_properties" + ("_ideas" if library_name else ""),
        library=Library.from_configuration(library_name) if library_name else None,
    )
    network.add_boiler_plate_spaces([space_with_same_properties])
    assert_model_equals_golden(network.model(), network.name)


@pytest.mark.parametrize(
    ("library_name", "network_name"),
    [("reduced_order", "reduced_order_three_zones"), ("iso_13790", "iso_13790_three_zones")],
)
def test_free_float_three_zones_low_order_libraries(library_name: str, network_name: str) -> None:
    network = Network(
        name=network_name,
        library=Library.from_configuration(library_name),
    )
    network.add_boiler_plate_spaces(three_spaces())
    assert_model_equals_golden(network.model(), network.name)


def test_template_buildings_free_float_single_zone_with_data(
    simple_space_1_with_occupancy: Space, buildings_library: Library
) -> None:
    network = Network(
        name="buildings_free_float_single_zone_with_data",
        library=buildings_library,
        external_data=Path(__file__).parent.joinpath("resources", "validation.csv"),
    )
    network.add_boiler_plate_spaces([simple_space_1_with_occupancy])
    assert_model_equals_golden(network.model(), network.name)


def test_bestest_case600ff(schema: Path) -> None:
    house = get_path("case600FF.yaml", "bestest")
    network = convert_network(
        "case600FF",
        house,
    )
    data_bus = DataBus(external_data=get_path("case600FF.csv", "bestest"))
    assert_model_equals_golden(network.model(data_bus=data_bus), network.name)


def test_yaml_realistic() -> None:
    network = convert_network("realistic_model", get_path("realistic.yaml"))
    assert_model_equals_golden(network.model(), network.name)
