import re
from pathlib import Path

from neosim.topology import Network


def remove_annotation(model: str) -> str:
    model = model.replace(" ", "").replace("\n", "")
    for annotation in re.findall(r"annotation(.*?);", model):
        model = model.replace(annotation, "").replace("annotation", "")
    return model


def _read(file_name: str) -> str:
    return (
        Path(__file__)
        .parent.joinpath("data", file_name)
        .read_text()
        .replace(" ", "")
        .replace("\n", "")
    )


def test_template_buildings_free_float_single_zone(
    buildings_free_float_single_zone: Network,
) -> None:
    model_ = buildings_free_float_single_zone.model()
    model_ = remove_annotation(model_)
    assert model_ == _read(buildings_free_float_single_zone.name)


def test_template_buildings_free_float_two_zones(
    buildings_free_float_two_zones: Network,
) -> None:
    model_ = buildings_free_float_two_zones.model()
    model_ = remove_annotation(model_)
    assert model_ == _read(buildings_free_float_two_zones.name)


def test_template_buildings_free_float_three_zones(
    buildings_free_float_three_zones: Network,
) -> None:
    model_ = buildings_free_float_three_zones.model()
    model_ = remove_annotation(model_)
    assert model_ == _read(buildings_free_float_three_zones.name)
