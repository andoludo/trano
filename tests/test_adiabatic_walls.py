"""Adiabatic walls (party walls): the whole construction in the zone, no heat through its outer face."""

import re
from pathlib import Path

import pytest

from tests.constructions.constructions import Constructions
from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.golden import remove_trano_package
from trano.data_models.conversion import convert_network
from trano.elements import AdiabaticWall, ExternalWall
from trano.elements.aggregated_envelope import AggregatedEnvelope
from trano.elements.library.library import Library
from trano.elements.space import SpaceVariant
from trano.elements.types import Tilt
from trano.exceptions import InvalidBuildingStructureError, InvalidModelError

HOUSE = Path(__file__).parent.joinpath("models", "house_infiltration_boiler.yaml")
LIVING_ROOM_FLOOR = """      floor_on_grounds:
        - surface: 27.11
          construction: FLOOR:001
"""
PARTY_WALL = """      adiabatic_walls:
        - surface: 21.6
          azimuth: -1.57
          tilt: wall
          construction: INNERWALL:001
"""


def house_with_party_wall(tmp_path: Path, party_wall: str = PARTY_WALL) -> Path:
    model = HOUSE.read_text()
    assert LIVING_ROOM_FLOOR in model
    house = tmp_path / "party_wall.yaml"
    house.write_text(model.replace(LIVING_ROOM_FLOOR, LIVING_ROOM_FLOOR + party_wall))
    return house


def party_wall_model(tmp_path: Path, library: str) -> str:
    network = convert_network(
        "party_wall", house_with_party_wall(tmp_path), library=Library.from_configuration(library)
    )
    return re.sub(r"\s+", " ", remove_trano_package(network.model()))


def test_buildings_holds_the_party_wall_after_the_floor_with_an_open_outer_port(schema: Path, tmp_path: Path) -> None:
    model = party_wall_model(tmp_path, "Buildings")

    assert re.search(
        r"datConBou\( layers=\{ floor_001, innerwall_001 \}, A=\{ 27\.11, 21\.6 \}, "
        r"til=\{Buildings\.Types\.Tilt\.Floor,Buildings\.Types\.Tilt\.Wall\}, azi=\{ 0\.0, -1\.57 \}",
        model,
    )
    assert "connect(livingroom_001.surf_conBou[1],flooronground_" in model
    assert "livingroom_001.surf_conBou[2]" not in model


def test_ideas_renders_an_adiabatic_boundary_wall(schema: Path, tmp_path: Path) -> None:
    model = party_wall_model(tmp_path, "IDEAS")

    assert re.search(
        r"IDEAS\.Buildings\.Components\.BoundaryWall adiabaticwall_\d+\( redeclare parameter "
        r"party_wall\.Data\.Constructions\. innerwall_001 constructionType, redeclare package Medium = Medium, "
        r"use_T_in=false, use_Q_in=false, inc=IDEAS\.Types\.Tilt\.Wall, azi=-1\.57, A=21\.6\)",
        model,
    )
    assert re.search(r"connect\(livingroom_001\.propsBus\[\d+\],adiabaticwall_\d+\.propsBus_a\)", model)


@pytest.mark.parametrize("library", ["reduced_order", "iso_13790"])
def test_lumped_zones_render_no_component_for_the_party_wall(schema: Path, tmp_path: Path, library: str) -> None:
    assert "adiabaticwall" not in party_wall_model(tmp_path, library)


def test_the_party_wall_is_internal_thermal_mass_of_the_lumped_zones() -> None:
    wall = ExternalWall(name="w", surface=10, azimuth=0, tilt=Tilt.wall, construction=Constructions.external_wall)
    party = AdiabaticWall(name="p", surface=20, construction=Constructions.external_wall)

    with_party = AggregatedEnvelope.from_boundaries([wall, party])
    without_party = AggregatedEnvelope.from_boundaries([wall])

    assert with_party.internal_capacity == pytest.approx(3 * without_party.internal_capacity, rel=1e-6)
    assert with_party.exterior_walls == without_party.exterior_walls


def test_an_adiabatic_wall_is_a_vertical_wall_by_default() -> None:
    party = AdiabaticWall(name="p", surface=20, construction=Constructions.external_wall)

    assert party.tilt == Tilt.wall and party.azimuth == 0


def test_an_adiabatic_wall_azimuth_in_degrees_is_rejected() -> None:
    with pytest.raises(InvalidBuildingStructureError, match="radians"):
        AdiabaticWall(name="p", surface=20, azimuth=90, construction=Constructions.external_wall)


@pytest.mark.parametrize(
    "party_wall",
    [
        PARTY_WALL.replace("azimuth: -1.57", "azimuth: 90"),  # degrees
        PARTY_WALL.replace("surface: 21.6", "surface: large"),  # not a number
        PARTY_WALL + "          perimeter: 12\n",  # not an attribute of an adiabatic wall
    ],
    ids=["degrees", "surface_not_a_number", "unknown_attribute"],
)
def test_invalid_adiabatic_walls_are_rejected(schema: Path, tmp_path: Path, party_wall: str) -> None:
    # the schema rejects out-of-range values, the LinkML converter malformed ones (as in test_validity)
    with pytest.raises((InvalidModelError, ValueError, TypeError)):
        convert_network("party_wall", house_with_party_wall(tmp_path, party_wall))


def test_the_ideas_zone_template_does_not_take_adiabatic_walls() -> None:
    space = simple_space_1_fixture()
    space.external_boundaries.append(AdiabaticWall(name="party", surface=20, construction=Constructions.external_wall))

    with pytest.raises(InvalidBuildingStructureError, match="adiabatic walls"):
        space.variant = SpaceVariant.rectangular_zone
