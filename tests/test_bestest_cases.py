"""The ASHRAE 140 cases are described faithfully and the YAML files are the generated ones."""

import re
from pathlib import Path

import pytest

from tests.golden import remove_trano_package
from trano.data_models.conversion import convert_network
from trano.elements.library.library import Library
from trano.elements.space import Space
from validation.bestest.cases import (
    CASES,
    CASES_DIR,
    FLOOR_AREA,
    INFILTRATION,
    UnsupportedCaseError,
    building_description,
    case_file,
    render_case,
)


def supported_cases() -> list[str]:
    supported = []
    for case in CASES.values():
        try:
            building_description(case)
        except UnsupportedCaseError:
            continue
        supported.append(case.id)
    return supported


def test_the_27_cases_of_section_5_2_are_defined() -> None:
    assert len(CASES) == 27
    assert {case.id for case in CASES.values() if case.free_float} == {
        "600FF", "650FF", "680FF", "900FF", "950FF", "980FF"
    }  # fmt: skip
    assert CASES["960"].features == {"hvac", "sunspace"}
    assert CASES["650"].features == {"hvac", "setpoint_schedule", "night_ventilation"}
    assert CASES["630"].features == {"hvac", "shading"}


@pytest.mark.parametrize("case_id", supported_cases())
def test_committed_yaml_is_the_generated_one(case_id: str) -> None:
    path = case_file(case_id)

    assert path.exists(), f"run `python -m validation.bestest generate` to create {path.name}"
    assert path.read_text() == render_case(CASES[case_id])


def test_no_stale_yaml_is_committed() -> None:
    committed = {path.stem.removeprefix("case_") for path in CASES_DIR.glob("case_*.yaml")}

    assert committed == set(supported_cases())


def zone_declaration(path: Path) -> str:
    network = convert_network(path.stem, path, library=Library.from_configuration("Buildings"))
    model = remove_trano_package(network.model())
    match = re.search(r"MixedAirInf\s+zone_001\((.*?)energyDynamics", model, re.DOTALL)
    assert match
    return re.sub(r"\s+", " ", match.group(1))


def test_base_building_reaches_the_buildings_zone() -> None:
    declaration = zone_declaration(case_file("600FF"))

    assert f"AFlo={FLOOR_AREA}" in declaration and "hRoo=2.7" in declaration
    assert f"ACH={INFILTRATION}" in declaration
    assert "linearizeRadiation=false" in declaration
    # Gross south wall of 21.6 m2 hosting the 12 m2 window, opaque walls east, west, north and roof.
    assert re.search(r"datConExtWin\(.*?A=\{ 21\.6 \}.*?wWin=\{ 6\.0 \}, hWin=\{ 2\.0 \}", declaration)
    assert re.search(r"datConExt\(.*?A=\{ 21\.6, 16\.2, 16\.2, 48\.0 \}", declaration)


def test_free_floating_cases_have_no_hvac() -> None:
    network = convert_network("case_900FF", case_file("900FF"), library=Library.from_configuration("Buildings"))
    zone = next(node for node in network.graph.nodes if isinstance(node, Space))

    assert zone.emissions == []
    assert zone.occupancy is not None


def test_heavy_and_insulated_cases_swap_the_constructions() -> None:
    light, heavy, insulated = (building_description(CASES[case_id]) for case_id in ("600FF", "900FF", "680FF"))
    walls = {
        name: {
            boundary["construction"] for boundary in description["spaces"][0]["external_boundaries"]["external_walls"]
        }
        for name, description in (("light", light), ("heavy", heavy), ("insulated", insulated))
    }

    assert walls["light"] == {"LIGHT_WALL:001", "ROOF:001"}
    assert walls["heavy"] == {"HEAVY_WALL:001", "ROOF:001"}
    assert walls["insulated"] == {"LIGHT_WALL_INSULATED:001", "ROOF_INSULATED:001"}
    assert heavy["spaces"][0]["external_boundaries"]["floor_on_grounds"][0]["construction"] == "HEAVY_FLOOR:001"


def test_internal_gain_is_200_w_split_60_40() -> None:
    occupancy = building_description(CASES["600FF"])["spaces"][0]["occupancy"]["parameters"]

    assert occupancy["gain"] == "[120/48; 80/48; 0]"
    assert occupancy["occupancy"] == "{1, 86400}"  # an entry at 0 would switch the schedule off
