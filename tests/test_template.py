import re
from pathlib import Path
from typing import Set

from neosim.library.buildings.buildings import BuildingsLibrary
from neosim.library.ideas.ideas import IdeasLibrary
from neosim.models.elements.space import Space
from neosim.models.elements.system import AirHandlingUnit
from neosim.topology import Network

OVERWRITE_MODELS = False


def remove_annotation(model: str) -> str:
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
    return set(
        model_.replace("record", ";").replace(f"model{model_name}", "").split(";")
    )


def _read(file_name: str) -> Set:
    return set(
        remove_annotation(
            Path(__file__).parent.joinpath("data", f"{file_name}.mo").read_text()
        )
        .replace("record", ";")
        .replace(f"model{file_name}", "")
        .split(";")
    )


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


def test_template_buildings_simple_hydronic(
    buildings_simple_hydronic: Network,
) -> None:
    model_ = buildings_simple_hydronic.model()
    assert clean_model(model_, buildings_simple_hydronic.name) == set(
        _read(buildings_simple_hydronic.name)
    )


def test_template_buildings_simple_hydronic_two_zones(
    buildings_simple_hydronic_two_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_two_zones.model()
    assert clean_model(model_, buildings_simple_hydronic_two_zones.name) == set(
        _read(buildings_simple_hydronic_two_zones.name)
    )


def test_template_buildings_simple_hydronic_three_zones(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_three_zones.model()
    assert clean_model(model_, buildings_simple_hydronic_three_zones.name) == set(
        _read(buildings_simple_hydronic_three_zones.name)
    )


def test_template_buildings_simple_hydronic_three_zones_draw(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    buildings_simple_hydronic_three_zones.plot()


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
    ideas_simple_hydronic_three_zones.plot()
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
    space_1_ideal_heating: Space,
) -> None:
    network = Network(name="space_1_ideal_heating")
    network.add_boiler_plate_spaces([space_1_ideal_heating])
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_space_1_different_construction_types(
    space_1_different_construction_types_network: Network,
) -> None:

    model_ = space_1_different_construction_types_network.model()
    assert clean_model(
        model_, space_1_different_construction_types_network.name
    ) == set(_read(space_1_different_construction_types_network.name))


def test_space_1_simple_ventilation(
    space_1_simple_ventilation: Space,
) -> None:

    network = Network(
        name="space_1_simple_ventilation",
        library=BuildingsLibrary(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces([space_1_simple_ventilation])
    ahu = AirHandlingUnit(name="ahu")
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_many_spaces_simple_ventilation(
    many_spaces_simple_ventilation: Network,
) -> None:

    model_ = many_spaces_simple_ventilation.model()
    assert clean_model(model_, many_spaces_simple_ventilation.name) == set(
        _read(many_spaces_simple_ventilation.name)
    )


def test_ideas_space_1_simple_ventilation(
    space_1_simple_ventilation: Space,
) -> None:

    network = Network(
        name="ideas_space_1_simple_ventilation",
        library=IdeasLibrary(
            constants="""
replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);  inner IDEAS.BoundaryConditions.SimInfoManager
sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));"""
        ),
    )
    network.add_boiler_plate_spaces([space_1_simple_ventilation])
    ahu = AirHandlingUnit(name="ahu")
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )

    model_ = network.model()
    assert clean_model(model_, network.name) == set(_read(network.name))


def test_ideas_many_spaces_simple_ventilation(
    ideas_many_spaces_simple_ventilation: Network,
) -> None:

    model_ = ideas_many_spaces_simple_ventilation.model()
    assert clean_model(model_, ideas_many_spaces_simple_ventilation.name) == set(
        _read(ideas_many_spaces_simple_ventilation.name)
    )


def test_template_buildings_simple_hydronic_three_zones_with_data_bus(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_three_zones.model()
    assert clean_model(model_, buildings_simple_hydronic_three_zones.name) == set(
        _read(buildings_simple_hydronic_three_zones.name)
    )


def test_buildings_free_float_single_zone_ahu_complex(
    buildings_free_float_single_zone_ahu_complex: Network,
) -> None:
    model_ = buildings_free_float_single_zone_ahu_complex.model()
    assert clean_model(
        model_, buildings_free_float_single_zone_ahu_complex.name
    ) == set(_read(buildings_free_float_single_zone_ahu_complex.name))
