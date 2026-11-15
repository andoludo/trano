"""A bypass: a `valve` under `systems` without a control is fixed open, from the supply to the return, so that the
water keeps flowing when the radiator valves close (a differential-pressure bypass, or a radiator without its
thermostatic valve). The valves of the spaces still need their emission control, and the pumps still follow the
controlled valves only."""

from __future__ import annotations

import re
import warnings
from pathlib import Path

import pytest
import yaml

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.validation.semantic import check

MODEL = Path(__file__).parent / "models" / "house_heat_pump_bypass.yaml"


def _model(path: Path, library: str) -> str:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=Library.from_configuration(library)).model()


def _instance(text: str, name: str) -> str:
    match = re.search(rf"\s{name}\((.*?)\)\s*(\"[^\"]*\")?\s*annotation", text, re.DOTALL)
    assert match, f"no instance {name}"
    return re.sub(r"\s+", "", match.group(1))


def test_a_valve_under_systems_without_a_control_is_valid() -> None:
    data = yaml.safe_load(MODEL.read_text())
    errors = [issue for issue in check(data) if issue.level == "error"]
    assert errors == []


def test_a_valve_of_a_space_still_needs_its_control() -> None:
    data = yaml.safe_load(MODEL.read_text())
    valve = next(e["valve"] for space in data["spaces"] for e in space.get("emissions", []) if "valve" in e)
    valve.pop("control")
    messages = [issue.message for issue in check(data) if issue.component == valve["id"]]
    assert any("needs a control" in message for message in messages)


@pytest.mark.parametrize("library", ["FastSmoothBuildings", "Buildings", "IDEAS"])
def test_the_bypass_is_fixed_open_between_the_supply_and_the_return(library: str) -> None:
    text = _model(MODEL, library)
    assert "y=1" in _instance(text, "valve_bypass")
    assert "valve_bypass.y" not in text  # no control drives it
    port = re.search(r"connect\(valve_bypass\.port_a(?:\[1\])?,port_a\[(\d+)\]\)", text)
    assert port, "the bypass is not connected"
    assert re.search(rf"connect\(temperature_sensor_001\.port_b(\[\d+\])?,port_b1\[{port.group(1)}\]\)", text)
    assert re.search(rf"connect\(split_valve_002\.port_1(\[\d+\])?,port_a1\[{port.group(1)}\]\)", text)


@pytest.mark.parametrize("library", ["FastSmoothBuildings", "Buildings"])
def test_the_pumps_follow_the_controlled_valves_only(library: str, tmp_path: Path) -> None:
    with_bypass = _model(MODEL, library)
    data = yaml.safe_load(MODEL.read_text())
    data["systems"] = [entry for entry in data["systems"] if "valve" not in entry]
    path = tmp_path / MODEL.name
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    without = _model(path, library)
    collector = re.compile(r"model CollectorControl\w+.*?end CollectorControl\w+;", re.DOTALL)
    assert collector.findall(with_bypass) == collector.findall(without)
