"""The ASHRAE 140 cases are described faithfully and the YAML files are the generated ones."""

import math
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


def hvac_parameters(case_id: str) -> dict[str, str | float]:
    return building_description(CASES[case_id])["spaces"][0]["emissions"][0]["ideal_heating_cooling"]["parameters"]  # type: ignore[no-any-return]


def test_the_base_case_heats_below_20_and_cools_above_27() -> None:
    parameters = hvac_parameters("600")

    assert parameters["heating_setpoint_schedule"] == "[0, 293.15]"
    assert parameters["cooling_setpoint_schedule"] == "[0, 300.15]"
    assert parameters["maximum_heating_power"] == parameters["maximum_cooling_power"] == 1e6


def test_the_setback_cases_ramp_the_heating_set_point_between_7_and_8() -> None:
    assert (
        hvac_parameters("640")["heating_setpoint_schedule"]
        == hvac_parameters("940")["heating_setpoint_schedule"]
        == "[0, 283.15; 25200, 283.15; 28800, 293.15; 82800, 293.15; 82800, 283.15; 86400, 283.15]"
    )


def test_the_single_set_point_cases_keep_a_dead_band_of_0_2_k() -> None:
    for case_id in ("685", "695", "985", "995"):
        parameters = hvac_parameters(case_id)
        assert parameters["heating_setpoint_schedule"] == "[0, 293.05]"
        assert parameters["cooling_setpoint_schedule"] == "[0, 293.25]"


def test_the_night_ventilation_cases_only_cool_between_7_and_18() -> None:
    hvac = CASES["650"].hvac
    assert hvac is not None

    assert hvac.emission["ideal_heating_cooling"]["parameters"]["maximum_heating_power"] == 0
    cooling_7_to_18 = "[0, 373.15; 25200, 373.15; 25200, 300.15; 64800, 300.15; 64800, 373.15; 86400, 373.15]"
    assert hvac.cooling_schedule == cooling_7_to_18


def test_the_materials_resolve_the_layers_with_18_states() -> None:
    description = building_description(CASES["900"])

    assert {material["number_of_states"] for material in description["material"]} == {18}
    assert "nStaRef=18" in zone_declaration(case_file("900")) or "nStaRef=18" in remove_trano_package(
        convert_network("case_900", case_file("900"), library=Library.from_configuration("Buildings")).model()
    )


def test_east_west_cases_split_the_12_m2_of_glazing_over_both_side_walls() -> None:
    windows = building_description(CASES["620"])["spaces"][0]["external_boundaries"]["windows"]

    assert [(window["surface"], window["width"], window["azimuth"]) for window in windows] == [
        (6.0, 3.0, pytest.approx(-math.pi / 2)),
        (6.0, 3.0, pytest.approx(math.pi / 2)),
    ]


def test_the_sun_space_case_has_a_light_zone_behind_a_heavy_sun_space() -> None:
    zone, sunspace = building_description(CASES["960"])["spaces"]

    assert zone["external_boundaries"]["windows"] == []
    assert {wall["construction"] for wall in zone["external_boundaries"]["external_walls"]} == {
        "LIGHT_WALL:001",
        "ROOF:001",
    }
    assert zone["external_boundaries"]["floor_on_grounds"][0]["construction"] == "LIGHT_FLOOR:001"
    assert {wall["construction"] for wall in sunspace["external_boundaries"]["external_walls"]} == {
        "HEAVY_WALL:001",
        "ROOF:001",
    }
    assert sunspace["external_boundaries"]["floor_on_grounds"][0]["construction"] == "HEAVY_FLOOR:001"
    assert sunspace["external_boundaries"]["windows"][0]["surface"] == 12.0
    assert "emissions" not in sunspace and sunspace["occupancy"] == {"variant": "none"}


def test_the_sun_space_has_no_occupancy_in_the_models() -> None:
    for library, expected_sunspace_occupancy in (("IDEAS", None), ("Buildings", "no_occupancy_sunspace_001")):
        network = convert_network("case_960", case_file("960"), library=Library.from_configuration(library))
        spaces = {node.name: node for node in network.graph.nodes if isinstance(node, Space)}
        sunspace = spaces["sunspace_001"].occupancy
        # A Buildings zone must have its gain input connected: it gets an occupancy with zero gains.
        assert (sunspace.name if sunspace else None) == expected_sunspace_occupancy, library
        assert sunspace is None or sunspace.parameters.gain == "[0; 0; 0]"  # type: ignore[union-attr]
        zone = spaces["zone_001"].occupancy
        assert zone is not None and zone.space_name == "zone_001" and zone.name == "occupancy_1"


def test_the_night_ventilation_cases_bring_in_outdoor_air_from_18_to_7() -> None:
    schedule = "[0, 0.391389; 25200, 0.391389; 25200, 0; 64800, 0; 64800, 0.391389; 86400, 0.391389]"
    for case_id in ("650", "950", "650FF", "950FF"):
        parameters = building_description(CASES[case_id])["spaces"][0]["parameters"]
        assert parameters["ventilation_schedule"] == schedule
    assert "ventilation_schedule" not in building_description(CASES["600"])["spaces"][0]["parameters"]
    assert "ventilationSchedule=[0, 0.391389; 25200" in zone_declaration(case_file("650"))


def test_the_shading_cases_mirror_the_overhang_and_fins_of_the_standard() -> None:
    south = building_description(CASES["610"])["spaces"][0]["external_boundaries"]["windows"][0]
    east, west = building_description(CASES["630"])["spaces"][0]["external_boundaries"]["windows"]

    assert south["overhang"] == {"depth": 1.0, "gap": 0.5, "width_left": 0.5, "width_right": 0.5}
    assert "side_fins" not in south
    for window in (east, west):
        assert window["overhang"] == {"depth": 1.0, "gap": 0.5, "width_left": 0.0, "width_right": 0.0}
        assert window["side_fins"] == {"depth": 1.0, "gap": 0.0, "height": 0.5}
    assert "ove(wL={ 0.5 }, wR={ 0.5 }, dep={ 1.0 }, gap={ 0.5 })" in zone_declaration(case_file("910"))
    assert "sidFin(h={ 0.5, 0.5 }, dep={ 1.0, 1.0 }, gap={ 0.0, 0.0 })" in zone_declaration(case_file("930"))


def test_every_case_of_section_5_2_is_supported() -> None:
    assert supported_cases() == list(CASES)
