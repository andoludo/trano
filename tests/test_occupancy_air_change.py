"""The outdoor-air change of a CO2 occupancy: `outdoor_air_change` renders as the block's `ACH`; its older name `ach`
still counts when it is the only one given, and is never rendered itself."""

from __future__ import annotations

import re
import warnings
from pathlib import Path

import pytest
import yaml

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library

FIXTURE = Path(__file__).parent / "models" / "house_infiltration_air_water_heat_pump.yaml"


def _model(parameters: dict, tmp_path: Path) -> str:
    doc = yaml.safe_load(FIXTURE.read_text())
    space = doc["spaces"][0]
    space["occupancy"] = {
        "variant": "co2",
        "parameters": {**parameters, "data": [{"component": "co2", "variable": "living_co2"}]},
    }
    path = tmp_path / "co2.yaml"
    path.write_text(yaml.safe_dump(doc, sort_keys=False))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=Library.from_configuration("Buildings")).model()


def _ach(text: str) -> str:
    match = re.search(r"Components\.BaseClasses\.Occupancy\w+\s+occupancy_\d+\(([^)]*)\)", text, re.DOTALL)
    assert match, "no CO2 occupancy instantiation"
    arguments = re.sub(r"\s+", "", match.group(1))
    assert "ach=" not in arguments.replace("ACH=", "")
    found = re.search(r"ACH=([0-9.]+)", arguments)
    assert found, arguments
    return found.group(1)


@pytest.mark.parametrize(
    ("parameters", "expected"),
    [
        ({"outdoor_air_change": 0.7}, "0.7"),
        ({"ach": 0.3}, "0.3"),  # the older name, alone
        ({"outdoor_air_change": 0.7, "ach": 0.3}, "0.7"),  # the explicit name wins
        ({}, "0.9"),
    ],
)
def test_the_air_change_of_a_co2_occupancy(parameters: dict, expected: str, tmp_path: Path) -> None:
    assert float(_ach(_model(parameters, tmp_path))) == pytest.approx(float(expected))
