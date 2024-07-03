import re
from pathlib import Path
from typing import Set

from trano.library.library import Ideas
from trano.models.elements.space import Space
from trano.topology import Network

OVERWRITE_MODELS = False


def remove_annotation(model: str) -> str:
    for documentation in re.findall(r"Documentation(.*?)</html>", model, re.DOTALL):
        model = model.replace(documentation, "").replace("Documentation", "")

    model = model.replace(" ", "").replace("\n", "")
    for annotation in re.findall(r"annotation(.*?);", model):
        model = model.replace(annotation, "").replace("annotation", "")

    return model


def remove_common_package(model: str) -> str:
    for annotation in re.findall(r"package Common(.*?)end Common;", model, re.DOTALL):
        model = (
            model.replace(annotation, "")
            .replace("package Common", "")
            .replace("end Common;", "")
        )
    return model


def clean_model(model: str, model_name: str) -> set:
    if OVERWRITE_MODELS:
        path_file = Path(__file__).parent.joinpath("data", f"{model_name}.mo")
        with path_file.open("w") as f:
            f.write(model)
    model = remove_common_package(model)
    model_ = remove_annotation(model)
    return {
        line
        for line in set(
            model_.replace("record", ";").replace(f"model{model_name}", "").split(";")
        )
        if "ReaderTMY3weather" not in line
    }


def _read(file_name: str) -> Set:
    return {
        line
        for line in set(
            remove_annotation(
                remove_common_package(
                    Path(__file__)
                    .parent.joinpath("data", f"{file_name}.mo")
                    .read_text()
                )
            )
            .replace("record", ";")
            .replace(f"model{file_name}", "")
            .split(";")
        )
        if "ReaderTMY3weather" not in line
    }


def test_template_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
) -> None:
    model_ = buildings_free_float_single_zone.model()
    assert clean_model(model_, buildings_free_float_single_zone.name) == set(
        _read(buildings_free_float_single_zone.name)
    )


def test_template_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
) -> None:
    model_ = buildings_free_float_two_zones.model()
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
    network = Network(name="space_with_same_properties_ideas", library=Ideas())
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
