"""The residential reference cases: one YAML per case generates the IDEAS and the Buildings model.

The two cases (examples/residential_before_renovation.yaml, residential_after_renovation.yaml) describe
the same semi-detached house before and after a deep renovation; see
docs/explanation/residential_reference_cases.md.
"""

import tempfile
from pathlib import Path

import numpy as np
import pytest
import yaml
from buildingspy.io.outputfile import Reader

from tests.golden import assert_model_equals_golden
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate
from trano.topology import Network
from trano.utils.utils import is_success

EXAMPLES = Path(__file__).parents[1].joinpath("examples")
CASES = ["before", "after"]
LIBRARIES = ["Buildings", "IDEAS"]
DAY = 24 * 3600


def example(case: str) -> Path:
    return EXAMPLES.joinpath(f"residential_{case}_renovation.yaml")


def network(case: str, library: str) -> Network:
    return convert_network(f"residential_{case}_renovation", example(case), library=Library.from_configuration(library))


def description(case: str) -> dict:
    return yaml.safe_load(example(case).read_text())


@pytest.mark.parametrize("case", CASES)
def test_the_example_validates_and_holds_the_house(schema: Path, case: str) -> None:
    house = network(case, "Buildings")
    spaces = description(case)["spaces"]

    assert [space["id"] for space in spaces] == [
        "LIVING:001",
        "KITCHEN:001",
        "BEDROOM_1:001",
        "BEDROOM_2:001",
        "BEDROOM_3:001",
        "BATHROOM:001",
    ]
    assert sum(space["parameters"]["floor_area"] for space in spaces) == pytest.approx(110.6)
    assert any(space["external_boundaries"].get("adiabatic_walls") for space in spaces)
    assert house.model()


def test_the_cases_share_the_house_its_use_and_its_emitters() -> None:
    """Only the envelope, the airtightness and the systems change with the renovation."""
    before, after = description("before"), description("after")

    def shared(space: dict) -> dict:
        boundaries = space["external_boundaries"]
        return {
            "parameters": {key: value for key, value in space["parameters"].items() if key != "n50"},
            "occupancy": space["occupancy"],
            "walls": [(wall["surface"], wall["azimuth"], wall["tilt"]) for wall in boundaries["external_walls"]],
            "windows": [(window["surface"], window["azimuth"]) for window in boundaries.get("windows") or []],
            "adiabatic_walls": boundaries.get("adiabatic_walls"),
            "floors": [(floor["surface"], floor["perimeter"]) for floor in boundaries.get("floor_on_grounds") or []],
            "emissions": space["emissions"],
        }

    assert [shared(space) for space in before["spaces"]] == [shared(space) for space in after["spaces"]]
    assert before["internal_walls"] == after["internal_walls"]
    assert before["weather"] == after["weather"]
    assert [space["parameters"]["n50"] for space in before["spaces"]] == [12.0] * 6
    assert [space["parameters"]["n50"] for space in after["spaces"]] == [3.0] * 6


def test_the_systems_of_the_cases() -> None:
    def systems(case: str) -> dict:
        return {key: value for system in description(case)["systems"] for key, value in system.items()}

    before, after = systems("before"), systems("after")
    assert before["boiler"]["variant"] == "without_storage"
    assert "air_handling_unit" not in before and "solar" not in description("before")
    assert after["boiler"]["variant"] == "air_water_heat_pump"
    assert after["air_handling_unit"]["variant"] == "system_d"
    assert description("after")["solar"][0]["photovoltaic"]["parameters"]["area"] == 21.0


@pytest.mark.parametrize("library", LIBRARIES)
@pytest.mark.parametrize("case", CASES)
def test_the_example_matches_its_golden_model(schema: Path, case: str, library: str) -> None:
    assert_model_equals_golden(network(case, library).model(), f"residential_{case}_renovation_{library}")


def _simulate(case: str, library: str, project: Path, options: SimulationOptions) -> Reader:
    results = simulate(project, network(case, library), options=options, container_name=f"trano_{case}_{library}")
    assert is_success(results, options=options), results.output.decode()[-3000:]
    return Reader(str(next(project.joinpath("results").glob("*.mat"))), "dymola")


@pytest.mark.simulate
@pytest.mark.parametrize("library", LIBRARIES)
@pytest.mark.parametrize("case", CASES)
def test_the_example_compiles(case: str, library: str) -> None:
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project:
        options = SimulationOptions(check_only=True)
        assert is_success(simulate(Path(project), network(case, library), options=options), options=options)


UPSTAIRS = ("bedroom_1", "bedroom_2", "bedroom_3", "bathroom")


def _radiator_heat(result: Reader, start: float, rooms: tuple[str, ...] | None = None) -> float:
    """Heat the radiators gave the rooms (all, or those given) after `start` [kWh]."""
    total = 0.0
    for name in result.varNames(r"emission1\.radiator_\w+_001\.Q_flow$"):
        if rooms is not None and not any(f".radiator_{room}_001." in name for room in rooms):
            continue
        t, q = result.values(name)
        keep = t >= start
        total += float(np.trapezoid(-q[keep], t[keep])) / 3.6e6
    return total


@pytest.mark.simulate
@pytest.mark.parametrize("case", CASES)
def test_ideas_and_buildings_agree_on_the_heating_demand(case: str) -> None:
    """Four January days after two days of warm-up.

    The rooms upstairs agree within 15 %. The whole house within 25 % only: the ground under the IDEAS
    slab starts at 10.8 degC and takes weeks to settle (the year agrees better, see
    docs/validation/residential_reference_cases.md).
    """
    options = SimulationOptions(end_time=6 * DAY, number_of_intervals=6 * 96)
    house, upstairs = {}, {}
    for library in LIBRARIES:
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project:
            result = _simulate(case, library, Path(project), options)
            house[library] = _radiator_heat(result, start=2 * DAY)
            upstairs[library] = _radiator_heat(result, start=2 * DAY, rooms=UPSTAIRS)

    assert upstairs["Buildings"] > 0
    assert upstairs["IDEAS"] == pytest.approx(upstairs["Buildings"], rel=0.15)
    assert house["IDEAS"] == pytest.approx(house["Buildings"], rel=0.25)
