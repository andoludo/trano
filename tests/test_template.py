from pathlib import Path
from typing import Optional

import pytest

from tests.conftest import _read, clean_model
from tests.fixtures.three_spaces import three_spaces
from trano.data_models.conversion import convert_network
from trano.elements import DataBus
from trano.elements.space import Space
from trano.elements.library.library import Library
from trano.topology import Network


def get_path(file_name: str, directory: Optional[str] = None) -> Path:
    if directory:
        return Path(__file__).parent.joinpath("models", directory, file_name)
    return Path(__file__).parent.joinpath("models", file_name)


def test_template_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
) -> None:
    model_ = buildings_free_float_single_zone.model(include_container=True)
    assert clean_model(model_, buildings_free_float_single_zone.name) == set(
        _read(buildings_free_float_single_zone.name)
    )


def test_template_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
) -> None:
    model_ = buildings_free_float_two_zones.model(include_container=True)
    assert clean_model(model_, buildings_free_float_two_zones.name) == set(
        _read(buildings_free_float_two_zones.name)
    )


def test_template_buildings_free_float_three_zones(
    buildings_free_float_three_zones: Network,
) -> None:
    model_ = buildings_free_float_three_zones.model()
    assert clean_model(model_, buildings_free_float_three_zones.name) == set(
        _read(buildings_free_float_three_zones.name)
    )


def test_buildings_two_rooms_with_storage(
    buildings_two_rooms_with_storage: Network,
) -> None:
    model_ = buildings_two_rooms_with_storage.model()
    assert clean_model(model_, buildings_two_rooms_with_storage.name) == set(
        _read(buildings_two_rooms_with_storage.name)
    )


def test_template_buildings_simple_hydronic(
    buildings_simple_hydronic: Network,
) -> None:
    model_ = buildings_simple_hydronic.model()
    assert clean_model(model_, buildings_simple_hydronic.name) == set(
        _read(buildings_simple_hydronic.name)
    )


def test_template_buildings_simple_hydronic_three_zones(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_three_zones.model()
    assert clean_model(model_, buildings_simple_hydronic_three_zones.name) == set(
        _read(buildings_simple_hydronic_three_zones.name)
    )


def test_template_ideas_free_float_single_zone(
    ideas_free_float_single_zone: Network,
) -> None:
    model_ = ideas_free_float_single_zone.model()
    assert clean_model(model_, ideas_free_float_single_zone.name) == set(
        _read(ideas_free_float_single_zone.name)
    )


def test_template_ideas_free_float_three_zones(
    ideas_free_float_three_zones: Network,
) -> None:
    model_ = ideas_free_float_three_zones.model()
    assert clean_model(model_, ideas_free_float_three_zones.name) == set(
        _read(ideas_free_float_three_zones.name)
    )


def test_ideas_simple_hydronic_three_zones(
    ideas_simple_hydronic_three_zones: Network,
) -> None:
    model_ = ideas_simple_hydronic_three_zones.model()
    assert clean_model(model_, ideas_simple_hydronic_three_zones.name) == set(
        _read(ideas_simple_hydronic_three_zones.name)
    )


def test_ideas_simple_hydronic_no_occupancy(
    ideas_simple_hydronic_no_occupancy: Network,
) -> None:
    model_ = ideas_simple_hydronic_no_occupancy.model()
    assert clean_model(model_, ideas_simple_hydronic_no_occupancy.name) == set(
        _read(ideas_simple_hydronic_no_occupancy.name)
    )


def test_space_1_ideal_heating(
    space_1_ideal_heating_network: Network,
) -> None:

    model_ = space_1_ideal_heating_network.model()
    assert clean_model(model_, space_1_ideal_heating_network.name) == set(
        _read(space_1_ideal_heating_network.name)
    )


def test_space_1_different_construction_types(
    space_1_different_construction_types_network: Network,
) -> None:

    model_ = space_1_different_construction_types_network.model()
    assert clean_model(
        model_, space_1_different_construction_types_network.name
    ) == set(_read(space_1_different_construction_types_network.name))


def test_one_spaces_air_handling_unit(one_spaces_air_handling_unit: Network) -> None:

    model_ = one_spaces_air_handling_unit.model()
    assert clean_model(model_, one_spaces_air_handling_unit.name) == set(
        _read(one_spaces_air_handling_unit.name)
    )


def test_two_spaces_air_handling_unit(two_spaces_air_handling_unit: Network) -> None:
    model_ = two_spaces_air_handling_unit.model()
    assert clean_model(model_, two_spaces_air_handling_unit.name) == set(
        _read(two_spaces_air_handling_unit.name)
    )


def test_space_with_same_properties(space_with_same_properties: Space) -> None:
    network = Network(
        name="space_with_same_properties",
    )
    network.add_boiler_plate_spaces([space_with_same_properties])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_space_with_same_properties_ideas(space_with_same_properties: Space) -> None:
    network = Network(
        name="space_with_same_properties_ideas",
        library=Library.from_configuration("IDEAS"),
    )
    network.add_boiler_plate_spaces([space_with_same_properties])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_space_with_door(space_with_door: Network) -> None:
    model_ = space_with_door.model()
    assert clean_model(model_, space_with_door.name) == set(_read(space_with_door.name))


def test_building_multiple_internal_walls(
    building_multiple_internal_walls: Network,
) -> None:
    model_ = building_multiple_internal_walls.model()
    assert clean_model(model_, building_multiple_internal_walls.name) == set(
        _read(building_multiple_internal_walls.name)
    )


def test_building_multiple_internal_walls_ideas(
    building_multiple_internal_walls_ideas: Network,
) -> None:
    model_ = building_multiple_internal_walls_ideas.model()
    assert clean_model(model_, building_multiple_internal_walls_ideas.name) == set(
        _read(building_multiple_internal_walls_ideas.name)
    )


def test_house_model(house_model: Network) -> None:
    model_ = house_model.model()
    assert clean_model(model_, house_model.name) == set(_read(house_model.name))


def test_template_buildings_free_float_single_zone_with_data(
    simple_space_1_with_occupancy: Space, buildings_library: Library
) -> None:
    network = Network(
        name="buildings_free_float_single_zone_with_data",
        library=buildings_library,
        external_data=Path(__file__).parent.joinpath("resources", "validation.csv"),
    )
    network.add_boiler_plate_spaces([simple_space_1_with_occupancy])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
def test_three_zones_hydronic_template(library_name: str) -> None:
    house = get_path("three_zones_hydronic.yaml")
    network = convert_network(
        "three_zones_hydronic", house, library=Library.from_configuration(library_name)
    )
    assert clean_model(network.model(), f"{network.name}_{library_name}_yaml") == set(
        _read(f"{network.name}_{library_name}_yaml")
    )


def test_single_zone_hydronic_template(schema: Path) -> None:
    house = get_path("single_zone_hydronic.yaml")
    network = convert_network("single_zone_hydronic", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_single_zone_hydronic_weather_template(schema: Path) -> None:
    house = get_path("single_zone_hydronic_weather.yaml")
    network = convert_network("single_zone_hydronic_weather", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_single_zone_air_handling_unit_simple_vav_control_template(
    schema: Path,
) -> None:
    house = get_path("single_zone_air_handling_unit_simple_vav_control.yaml")
    network = convert_network("single_zone_air_handling_unit_simple_vav_control", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_single_zone_air_handling_unit_complex_vav_template(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_complex_vav.yaml")
    network = convert_network("single_zone_air_handling_unit_complex_vav", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_two_zones_template(schema: Path) -> None:
    house = get_path("two_zones.yaml")
    network = convert_network("two_zones", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_two_zones_ideas_template(schema: Path) -> None:
    house = get_path("two_zones_ideas.yaml")
    network = convert_network("two_zones_ideas", house)
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_single_zone_air_handling_unit_without_vav_with_duct_template(
    schema: Path,
) -> None:
    house = get_path("single_zone_air_handling_unit_without_vav_with_duct.yaml")
    # TODO: remove ducts here
    network = convert_network(
        "single_zone_air_handling_unit_without_vav_with_duct", house
    )
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
def test_three_zones_hydronic_with_containers(schema: Path, library_name: str) -> None:
    house = get_path("three_zones_hydronic_containers.yaml")
    network = convert_network(
        f"three_zones_hydronic_container_{library_name}",
        house,
        library=Library.from_configuration(library_name),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
def test_single_zone_air_handling_unit_complex_vav_containers(
    schema: Path, library_name: str
) -> None:
    house = get_path("single_zone_air_handling_unit_complex_vav_containers.yaml")
    network = convert_network(
        f"single_zone_air_handling_unit_complex_vav_containers_{library_name}",
        house,
        library=Library.from_configuration(library_name),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_template_buildings_free_float_three_zones_reduced_order() -> None:
    network = Network(
        name="reduced_order_three_zones",
        library=Library.from_configuration("reduced_order"),
    )
    network.add_boiler_plate_spaces(three_spaces())
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_three_zones_hydronic_reduced_orders_reduced_order(schema: Path) -> None:
    house = get_path("three_zones_hydronic_reduced_orders.yaml")
    network = convert_network(
        "three_zones_hydronic_reduced_orders_reduced_order",
        house,
        library=Library.from_configuration("reduced_order"),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_template_buildings_free_float_three_zones_iso_13790() -> None:
    network = Network(
        name="iso_13790_three_zones",
        library=Library.from_configuration("iso_13790"),
    )
    network.add_boiler_plate_spaces(three_spaces())
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_three_zones_hydronic_reduced_orders_iso_13790(schema: Path) -> None:
    house = get_path("three_zones_hydronic_reduced_orders.yaml")
    network = convert_network(
        "three_zones_hydronic_reduced_orders_iso_13790",
        house,
        library=Library.from_configuration("iso_13790"),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_three_zones_hydronic_reduced_orders_iso_13790_heat_pump(schema: Path) -> None:
    house = get_path("three_zones_hydronic_reduced_orders_heat_pump.yaml")
    network = convert_network(
        "three_zones_hydronic_reduced_orders_iso_13790_heat_pump",
        house,
        library=Library.from_configuration("iso_13790"),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_three_zones_hydronic_reduced_orders_iso_13790_electric(schema: Path) -> None:
    house = get_path("three_zones_hydronic_reduced_orders.yaml")
    network = convert_network(
        "three_zones_hydronic_reduced_orders_iso_13790_electric",
        house,
        library=Library.from_configuration("iso_13790"),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_three_zones_hydronic_reduced_orders_pv(schema: Path) -> None:
    house = get_path("three_zones_hydronic_reduced_orders_pv.yaml")
    network = convert_network(
        "three_zones_hydronic_reduced_orders_pv",
        house,
        library=Library.from_configuration("iso_13790"),
    )
    model_ = network.model(include_container=True)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_single_zone_hydronic_tilt_as_value(schema: Path) -> None:
    house = get_path("single_zone_hydronic_tilt_as_value.yaml")
    network = convert_network(
        "single_zone_hydronic_tilt_as_value",
        house,
        library=Library.from_configuration("IDEAS"),
    )
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_single_zone_window_different_construction(schema: Path) -> None:
    house = get_path("house_multiple_construction_one_azimuth.yaml")
    network = convert_network(
        "house_multiple_construction_one_azimuth",
        house,
    )
    assert clean_model(network.model(), f"{network.name}_yaml") == set(
        _read(f"{network.name}_yaml")
    )


def test_bestest_case600ff(schema: Path) -> None:
    house = get_path("case600FF.yaml", "bestest")
    network = convert_network(
        "case600FF",
        house,
    )
    data_bus = DataBus(external_data=get_path("case600FF.csv", "bestest"))
    model_ = network.model(data_bus=data_bus)
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_complex(schema: Path) -> None:
    house = get_path("house_complex.yaml")

    network = convert_network(
        "house_complex",
        house,
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_infiltration_boiler(schema: Path) -> None:
    house = get_path("house_infiltration_boiler.yaml")

    network = convert_network(
        "house_infiltration_boiler",
        house,
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_infiltration_boiler_ideas(schema: Path) -> None:
    house = get_path("house_infiltration_boiler.yaml")

    network = convert_network(
        "house_infiltration_boiler_ideas",
        house,
        library=Library.from_configuration("IDEAS"),
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_infiltration_air_water_heat_pump(schema: Path) -> None:
    house = get_path("house_infiltration_air_water_heat_pump.yaml")

    network = convert_network(
        "house_infiltration_air_water_heat_pump",
        house,
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_infiltration_air_water_heat_pump_ideas(schema: Path) -> None:
    house = get_path("house_infiltration_air_water_heat_pump.yaml")

    network = convert_network(
        "house_infiltration_air_water_heat_pump_ideas",
        house,
        library=Library.from_configuration("IDEAS"),
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_complex_dedicated_pumps(schema: Path) -> None:
    house = get_path("house_complex_dedicated_pumps.yaml")

    network = convert_network("house_complex_dedicated_pumps", house)
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_multizone_air_handling_unit_space_connected(schema: Path) -> None:
    house = get_path("multizone_air_handling_unit_space_connected.yaml")

    network = convert_network("multizone_air_handling_unit_space_connected", house)
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_multizone_air_handling_unit_space_connected_ideas(schema: Path) -> None:
    house = get_path("multizone_air_handling_unit_space_connected.yaml")

    network = convert_network(
        "multizone_air_handling_unit_space_connected_ideas",
        house,
        library=Library.from_configuration("IDEAS"),
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_house_complex_heat_meter(schema: Path) -> None:
    house = get_path("house_complex_heat_meter.yaml")

    network = convert_network(
        "house_complex_heat_meter",
        house,
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))