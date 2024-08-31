from pathlib import Path

import pytest

from tests.conftest import _read, clean_model
from trano.elements.space import Space
from trano.library.library import Library
from trano.topology import Network


@pytest.mark.run(order=1)  # TODO: code smell!
def test_template_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
) -> None:
    model_ = buildings_free_float_single_zone.model()
    assert clean_model(model_, buildings_free_float_single_zone.name) == set(
        _read(buildings_free_float_single_zone.name)
    )


@pytest.mark.run(order=2)
def test_template_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
) -> None:
    model_ = buildings_free_float_two_zones.model()
    assert clean_model(model_, buildings_free_float_two_zones.name) == set(
        _read(buildings_free_float_two_zones.name)
    )


@pytest.mark.run(order=3)
def test_template_buildings_free_float_three_zones(
    buildings_free_float_three_zones: Network,
) -> None:
    model_ = buildings_free_float_three_zones.model()
    assert clean_model(model_, buildings_free_float_three_zones.name) == set(
        _read(buildings_free_float_three_zones.name)
    )


@pytest.mark.run(order=4)
def test_buildings_two_rooms_with_storage(
    buildings_two_rooms_with_storage: Network,
) -> None:
    model_ = buildings_two_rooms_with_storage.model()
    assert clean_model(model_, buildings_two_rooms_with_storage.name) == set(
        _read(buildings_two_rooms_with_storage.name)
    )


@pytest.mark.run(order=5)
def test_template_buildings_simple_hydronic(
    buildings_simple_hydronic: Network,
) -> None:
    model_ = buildings_simple_hydronic.model()
    assert clean_model(model_, buildings_simple_hydronic.name) == set(
        _read(buildings_simple_hydronic.name)
    )


@pytest.mark.run(order=6)
def test_template_buildings_simple_hydronic_three_zones(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_three_zones.model()
    assert clean_model(model_, buildings_simple_hydronic_three_zones.name) == set(
        _read(buildings_simple_hydronic_three_zones.name)
    )


@pytest.mark.run(order=7)
def test_template_ideas_free_float_single_zone(
    ideas_free_float_single_zone: Network,
) -> None:
    model_ = ideas_free_float_single_zone.model()
    assert clean_model(model_, ideas_free_float_single_zone.name) == set(
        _read(ideas_free_float_single_zone.name)
    )


@pytest.mark.run(order=8)
def test_template_ideas_free_float_three_zones(
    ideas_free_float_three_zones: Network,
) -> None:
    model_ = ideas_free_float_three_zones.model()
    assert clean_model(model_, ideas_free_float_three_zones.name) == set(
        _read(ideas_free_float_three_zones.name)
    )


@pytest.mark.run(order=9)
def test_ideas_simple_hydronic_three_zones(
    ideas_simple_hydronic_three_zones: Network,
) -> None:
    model_ = ideas_simple_hydronic_three_zones.model()
    assert clean_model(model_, ideas_simple_hydronic_three_zones.name) == set(
        _read(ideas_simple_hydronic_three_zones.name)
    )


@pytest.mark.run(order=10)
def test_ideas_simple_hydronic_no_occupancy(
    ideas_simple_hydronic_no_occupancy: Network,
) -> None:
    model_ = ideas_simple_hydronic_no_occupancy.model()
    assert clean_model(model_, ideas_simple_hydronic_no_occupancy.name) == set(
        _read(ideas_simple_hydronic_no_occupancy.name)
    )


@pytest.mark.run(order=11)
def test_space_1_ideal_heating(
    space_1_ideal_heating_network: Network,
) -> None:

    model_ = space_1_ideal_heating_network.model()
    assert clean_model(model_, space_1_ideal_heating_network.name) == set(
        _read(space_1_ideal_heating_network.name)
    )


@pytest.mark.run(order=12)
def test_space_1_different_construction_types(
    space_1_different_construction_types_network: Network,
) -> None:

    model_ = space_1_different_construction_types_network.model()
    assert clean_model(
        model_, space_1_different_construction_types_network.name
    ) == set(_read(space_1_different_construction_types_network.name))


@pytest.mark.run(order=13)
def test_one_spaces_air_handling_unit(one_spaces_air_handling_unit: Network) -> None:

    model_ = one_spaces_air_handling_unit.model()
    assert clean_model(model_, one_spaces_air_handling_unit.name) == set(
        _read(one_spaces_air_handling_unit.name)
    )


@pytest.mark.run(order=14)
def test_two_spaces_air_handling_unit(two_spaces_air_handling_unit: Network) -> None:
    model_ = two_spaces_air_handling_unit.model()
    assert clean_model(model_, two_spaces_air_handling_unit.name) == set(
        _read(two_spaces_air_handling_unit.name)
    )


@pytest.mark.run(order=15)
def test_space_with_same_properties(space_with_same_properties: Space) -> None:
    network = Network(
        name="space_with_same_properties",
    )
    network.add_boiler_plate_spaces([space_with_same_properties])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


@pytest.mark.run(order=16)
def test_space_with_same_properties_ideas(space_with_same_properties: Space) -> None:
    network = Network(
        name="space_with_same_properties_ideas",
        library=Library.from_configuration("IDEAS"),
    )
    network.add_boiler_plate_spaces([space_with_same_properties])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


@pytest.mark.run(order=17)
def test_space_with_door(space_with_door: Network) -> None:
    model_ = space_with_door.model()
    assert clean_model(model_, space_with_door.name) == set(_read(space_with_door.name))


@pytest.mark.run(order=18)
def test_building_multiple_internal_walls(
    building_multiple_internal_walls: Network,
) -> None:
    model_ = building_multiple_internal_walls.model()
    assert clean_model(model_, building_multiple_internal_walls.name) == set(
        _read(building_multiple_internal_walls.name)
    )


@pytest.mark.run(order=19)
def test_building_multiple_internal_walls_ideas(
    building_multiple_internal_walls_ideas: Network,
) -> None:
    model_ = building_multiple_internal_walls_ideas.model()
    assert clean_model(model_, building_multiple_internal_walls_ideas.name) == set(
        _read(building_multiple_internal_walls_ideas.name)
    )


@pytest.mark.run(order=20)
def test_house_model(house_model: Network) -> None:
    model_ = house_model.model()
    assert clean_model(model_, house_model.name) == set(_read(house_model.name))


@pytest.mark.run(order=21)
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
