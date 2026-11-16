"""The smallest opening of a proportional emission control (`minimum_opening`, the yMin of its controller): a
thermostatic valve with a minimum position, or a radiator kept on, stays that far open whatever the room temperature.
Without the key nothing changes; FastSmoothBuildings (0.9.0) takes it too."""

from __future__ import annotations

import re
import warnings
from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library

MODEL = Path(__file__).parent / "models" / "house_heat_pump_minimum_opening.yaml"
WITHOUT = Path(__file__).parent / "models" / "house_infiltration_air_water_heat_pump.yaml"


def _model(path: Path, library: str) -> str:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=Library.from_configuration(library)).model()


def _instance(text: str, name: str) -> str:
    match = re.search(rf"\s{name}\((.*?)\)\s*(\"[^\"]*\")?\s*annotation", text, re.DOTALL)
    assert match, f"no instance {name}"
    return re.sub(r"\s+", "", match.group(1))


@pytest.mark.parametrize("library", ["Buildings", "IDEAS"])
def test_the_minimum_opening_is_the_lower_limit_of_the_controller(library: str) -> None:
    text = _model(MODEL, library)
    assert "yMin=0.18" in _instance(text, "emission_control_002")
    assert "yMin" not in _instance(text, "emission_control_001")  # the other rooms keep the default, 0
    assert "LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.P, k=k, yMin=yMin)" in text


def test_without_the_key_the_controls_take_no_minimum(tmp_path: Path) -> None:
    text = _model(WITHOUT, "Buildings")
    assert all("yMin" not in _instance(text, f"emission_control_00{i}") for i in (1, 2, 3, 4, 5, 6))


def test_fast_smooth_buildings_takes_it_too() -> None:
    """FastSmoothBuildings (0.9.0) limits the output of its proportional controls to [yMin, 1]."""
    text = _model(MODEL, "FastSmoothBuildings")
    assert "yMin=0.18" in _instance(text, "emission_control_002")
    assert "yMin" not in _instance(text, "emission_control_001")
