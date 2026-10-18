"""Validation errors read as one line per error: the YAML path, what is expected and the field description."""

from pathlib import Path
from typing import Any

import pytest
import yaml

from trano.data_models.conversion import convert_network
from trano.exceptions import InvalidModelError

FIRST_MODEL = Path(__file__).parent / "tutorials" / "first_model.yaml"


def invalid_model_errors(tmp_path: Path, model: dict[str, Any]) -> InvalidModelError:
    path = tmp_path / "model.yaml"
    path.write_text(yaml.safe_dump(model))
    with pytest.raises(InvalidModelError) as raised:
        convert_network("model", path)
    return raised.value


def first_model() -> dict[str, Any]:
    return dict(yaml.safe_load(FIRST_MODEL.read_text()))


def test_a_bound_names_the_field_and_its_description(tmp_path: Path) -> None:
    model = first_model()
    model["spaces"][0]["parameters"]["floor_area"] = -5

    error = invalid_model_errors(tmp_path, model)

    assert error.errors == ("spaces[0].parameters.floor_area: must be >= 0 (got -5) (Floor area [m2])",)
    assert str(error) == "Invalid model:\n- " + error.errors[0]


def test_every_error_gets_its_line(tmp_path: Path) -> None:
    model = first_model()
    space = model["spaces"][0]
    space["parameters"]["ach"] = "often"
    space["parameters"]["volume"] = 250
    space["variant"] = "cube"
    del space["external_boundaries"]

    errors = {line.split(":", 1)[0]: line for line in invalid_model_errors(tmp_path, model).errors}

    assert errors["spaces[0].external_boundaries"] == (
        "spaces[0].external_boundaries: is required "
        "(Envelope elements of the space in contact with the outdoor air or the ground)"
    )
    assert errors["spaces[0].parameters.ach"] == (
        "spaces[0].parameters.ach: must be a number (got 'often') (Infiltration [1/h])"
    )
    unknown = errors["spaces[0].parameters.volume"]
    assert unknown.startswith("spaces[0].parameters.volume: unknown field; the fields of spaces[0].parameters are ")
    assert "floor_area" in unknown
    assert errors["spaces[0].variant"] == (
        "spaces[0].variant: must be one of default, infiltration, rectangular_zone (got 'cube') (Zone models)"
    )
    assert len(errors) == 4


def test_temperatures_in_celsius_are_explained(tmp_path: Path) -> None:
    model = first_model()
    model["spaces"][0]["parameters"]["temperature_initial"] = 20

    (line,) = invalid_model_errors(tmp_path, model).errors

    assert line.startswith("spaces[0].parameters.temperature_initial: 20 is not a temperature in kelvin")
    assert "(20 degC is 293.15 K)" in line


def test_semantic_errors_are_lines_too(tmp_path: Path) -> None:
    model = first_model()
    model["spaces"].append(dict(model["spaces"][0]))

    error = invalid_model_errors(tmp_path, model)

    assert error.errors
    assert all("\n" not in line for line in error.errors)
    assert str(error).splitlines()[1:] == [f"- {line}" for line in error.errors]
