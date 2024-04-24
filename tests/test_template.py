import re
from pathlib import Path
from typing import Set

from neosim.topology import Network


def remove_annotation(model: str) -> str:
    model = model.replace(" ", "").replace("\n", "")
    for annotation in re.findall(r"annotation(.*?);", model):
        model = model.replace(annotation, "").replace("annotation", "")
    return model


def _read(file_name: str) -> Set:
    return set(
        remove_annotation(
            Path(__file__).parent.joinpath("data", f"{file_name}.mo").read_text()
        )
        .replace("record", ";")
        .split(";")
    )


def test_template_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
) -> None:
    model_ = buildings_free_float_single_zone.model()
    model_ = remove_annotation(model_)
    assert set(model_.split(";")) == _read(buildings_free_float_single_zone.name)


def test_template_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
) -> None:
    model_ = buildings_free_float_two_zones.model()
    model_ = remove_annotation(model_)
    assert set(model_.split(";")) == _read(buildings_free_float_two_zones.name)


def test_template_buildings_free_float_three_zones(
    buildings_free_float_three_zones: Network,
) -> None:
    model_ = buildings_free_float_three_zones.model()
    model_ = remove_annotation(model_)
    assert set(model_.split(";")) == _read(buildings_free_float_three_zones.name)


def test_template_buildings_simple_hydronic(
    buildings_simple_hydronic: Network,
) -> None:
    model_ = buildings_simple_hydronic.model()
    model_ = remove_annotation(model_)
    assert set(model_.split(";")) == _read(buildings_simple_hydronic.name)


def test_template_buildings_simple_hydronic_two_zones(
    buildings_simple_hydronic_two_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_two_zones.model()
    model_ = remove_annotation(model_)
    assert set(model_.split(";")) == _read(buildings_simple_hydronic_two_zones.name)


# @pytest.mark.skip("To be checked")
def test_template_buildings_simple_hydronic_three_zones(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    model_ = buildings_simple_hydronic_three_zones.model()
    model_ = remove_annotation(model_)
    assert set(model_.split(";")) == set(
        _read(buildings_simple_hydronic_three_zones.name)
    )


def test_template_buildings_simple_hydronic_three_zones_draw(
    buildings_simple_hydronic_three_zones: Network,
) -> None:
    buildings_simple_hydronic_three_zones.plot()


def test_template_ideas_free_float_single_zone(
    ideas_free_float_single_zone: Network,
) -> None:
    model_ = ideas_free_float_single_zone.model("ideas.jinja2")
    model_ = remove_annotation(model_)
    assert set(model_.replace("record", ";").split(";")) == _read(
        ideas_free_float_single_zone.name
    )


def test_template_ideas_free_float_three_zones(
    ideas_free_float_three_zones: Network,
) -> None:
    model_ = ideas_free_float_three_zones.model("ideas.jinja2")
    model_ = remove_annotation(model_)
    assert set(model_.replace("record", ";").split(";")) == _read(
        ideas_free_float_three_zones.name
    )
