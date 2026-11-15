"""The two optional plant keys of the FastSmoothBuildings templates: `boiler_control: parameters: {always_on: true}`
makes the generator's demand 1 (a heat pump holding a combi tank whatever the room valves ask), `pump: parameters:
{constant_speed: true}` makes the pump run at its nominal speed (y = 1). Off, nothing changes."""

from __future__ import annotations

import re
import warnings
from pathlib import Path

import yaml

from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library

FIXTURE = Path(__file__).parent / "models" / "house_infiltration_air_water_heat_pump.yaml"


def _model(doc: dict, tmp_path: Path) -> str:
    path = tmp_path / "plant.yaml"
    path.write_text(yaml.safe_dump(doc, sort_keys=False))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return convert_network(path.stem, path, library=Library.from_configuration("FastSmoothBuildings")).model()


def _systems(doc: dict, kind: str) -> list[dict]:
    return [item[kind] for item in doc["systems"] if kind in item]


def _heat_pump_demand(text: str) -> str:
    match = re.search(r"HeatPumpControl \w+\(.*?demand=([^,]+),", text, re.DOTALL)
    assert match, "no HeatPumpControl instantiation"
    return match.group(1).strip()


def _pump_inputs(text: str) -> list[str]:
    return [m.strip() for m in re.findall(r"Hydronics\.Pump \w+\(.*?y=([^)]+)\)", text, re.DOTALL)]


def test_without_the_keys_the_demand_and_the_pumps_follow_their_controls(tmp_path: Path) -> None:
    doc = yaml.safe_load(FIXTURE.read_text())
    text = _model(doc, tmp_path)
    assert _heat_pump_demand(text).endswith(".y")
    assert all(y.endswith(".y") for y in _pump_inputs(text))


def test_always_on_makes_the_demand_one_and_constant_speed_the_pump_input_one(tmp_path: Path) -> None:
    doc = yaml.safe_load(FIXTURE.read_text())
    boiler = _systems(doc, "boiler")[0]
    boiler["control"]["boiler_control"].setdefault("parameters", {})["always_on"] = True
    pumps = _systems(doc, "pump")
    pumps[0].setdefault("parameters", {})["constant_speed"] = True
    text = _model(doc, tmp_path)
    assert _heat_pump_demand(text) == "1"
    inputs = _pump_inputs(text)
    assert inputs[0] == "1"
    assert all(y.endswith(".y") for y in inputs[1:])  # the other pumps keep their control
