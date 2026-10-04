import math

import pytest

from tests.constructions.constructions import Constructions, Glasses
from tests.fixtures.simple_space_1 import simple_space_1_fixture
from trano.elements import ExternalDoor, ExternalWall, FloorOnGround, Window
from trano.elements.construction import Construction, Glass
from trano.elements.library.library import Library
from trano.elements.space import Space, SpaceVariant
from trano.elements.envelope import MergedExternalWall
from trano.elements.jinja import compile_template
from trano.elements.types import TILT_RADIANS, Azimuth, Tilt, wind_pressure_table
from trano.elements.zone_template import RectangularZone, same_angle, to_radians
from trano.exceptions import UnknownComponentVariantError
from trano.topology import Network


def _wall(
    name: str, azimuth: float, surface: float = 10, tilt: Tilt = Tilt.wall, construction: Construction | None = None
) -> ExternalWall:
    return ExternalWall(
        name=name,
        surface=surface,
        azimuth=azimuth,
        tilt=tilt,
        construction=construction or Constructions.external_wall,
    )


def _window(
    name: str, azimuth: float, surface: float = 2, height: float | None = None, glazing: Glass | None = None
) -> Window:
    return Window(
        name=name,
        surface=surface,
        azimuth=azimuth,
        tilt=Tilt.wall,
        height=height,
        construction=glazing or Glasses.double_glazing,
    )


@pytest.mark.parametrize(
    ("azimuth", "expected"),
    [(0, 0), (1.57, 1.57), (-1.57, 2 * math.pi - 1.57), (180.0, math.pi), (270, 1.5 * math.pi), (-90.0, 1.5 * math.pi)],
)
def test_to_radians_accepts_degrees_and_radians(azimuth: float, expected: float) -> None:
    assert to_radians(azimuth) == pytest.approx(expected)


def test_same_angle_wraps_around() -> None:
    assert same_angle(0.0, 2 * math.pi)
    assert same_angle(-1.57, 3 * math.pi / 2)
    assert not same_angle(0.0, math.pi / 2)


def test_rectangular_zone_maps_the_four_orientations() -> None:
    zone = RectangularZone.from_boundaries(simple_space_1_fixture().external_boundaries, height=2.0, floor_area=20.0)
    assert zone.azimuth == 0
    assert [face.boundary_type for face in zone.faces] == ["OuterWall"] * 4 + ["SlabOnGround", "None"]
    # Face A is south, then west, north, east. Only the east wall carries a window.
    assert [face.window is not None for face in zone.faces] == [False, False, False, True, False, False]
    east = zone.face("D")
    assert east.window is not None
    assert east.window.area == 1
    assert east.window.height == 1
    assert east.length == pytest.approx((10 + 1) / 2.0)
    assert zone.length == 5.0
    assert zone.width == 4.0
    assert zone.ceiling_area == 20.0
    assert zone.external_surfaces == []
    assert zone.constructions() == {Constructions.external_wall, Glasses.double_glazing}


def test_rectangular_zone_keeps_unfitting_surfaces_apart() -> None:
    roof = _wall("roof", Azimuth.south, tilt=Tilt.pitched_roof_45)
    door = ExternalDoor(
        name="door", surface=2, azimuth=Azimuth.south, tilt=Tilt.wall, construction=Constructions.internal_wall
    )
    second_glazing = _window("win_single", Azimuth.south, glazing=Glasses.simple_glazing)
    skew = _wall("skew", 0.7)
    boundaries = [
        _wall("south", Azimuth.south, surface=12),
        _wall("south_bis", Azimuth.south, surface=3),
        _wall("ceiling", Azimuth.south, tilt=Tilt.ceiling),
        _window("win_a", Azimuth.south, surface=2, height=1),
        _window("win_b", Azimuth.south, surface=2, height=2),
        FloorOnGround(name="floor", surface=20, construction=Constructions.external_wall),
        roof,
        door,
        second_glazing,
        skew,
    ]
    zone = RectangularZone.from_boundaries(boundaries, height=3.0, floor_area=20.0)
    south = zone.face("A")
    assert south.construction == Constructions.external_wall
    assert south.area == 15  # both south walls share the construction and are lumped
    assert south.window is not None
    assert south.window.area == 4
    assert south.window.height == 1.5  # area-weighted
    assert zone.face("Cei").boundary_type == "OuterWall"
    assert zone.ceiling_area == 10
    assert [face.boundary_type for face in zone.faces[1:4]] == ["None"] * 3
    assert [element.name for element in zone.external_surfaces] == ["door", "roof", "skew", "win_single"]


def test_rectangular_zone_without_vertical_walls() -> None:
    zone = RectangularZone.from_boundaries(
        [FloorOnGround(name="floor", surface=9, construction=Constructions.external_wall)], height=3.0, floor_area=9.0
    )
    assert zone.azimuth == 0
    assert zone.length == 3.0
    assert zone.width == 3.0
    assert [face.boundary_type for face in zone.faces] == ["None"] * 4 + ["SlabOnGround", "None"]


def test_face_a_prefers_the_azimuth_matching_most_surfaces() -> None:
    boundaries = [_wall("w1", 0.3), _wall("w2", 0.3 + math.pi / 2), _wall("w3", 0.3 + math.pi), _wall("odd", 1.0)]
    zone = RectangularZone.from_boundaries(boundaries, height=3.0, floor_area=20.0)
    assert zone.azimuth == pytest.approx(0.3)
    assert [element.name for element in zone.external_surfaces] == ["odd"]


def test_space_exposes_zone_template_only_for_the_variant() -> None:
    space = simple_space_1_fixture()
    assert not space.uses_zone_template
    assert space.template_constructions() == set()
    space.variant = SpaceVariant.rectangular_zone
    assert space.uses_zone_template
    assert space.merged_external_boundaries == []
    assert space.template_constructions() == {Constructions.external_wall, Glasses.double_glazing}


def test_zone_template_variant_is_only_available_for_ideas() -> None:
    space = simple_space_1_fixture()
    space.variant = SpaceVariant.rectangular_zone
    network = Network(name="buildings_rectangular", library=Library.from_configuration("Buildings"))
    with pytest.raises(UnknownComponentVariantError):
        network.add_boiler_plate_spaces([space])


def test_zone_template_model_declares_every_construction_once() -> None:
    space: Space = simple_space_1_fixture()
    space.variant = SpaceVariant.rectangular_zone
    network = Network(name="ideas_rectangular", library=Library.from_configuration("IDEAS"))
    network.add_boiler_plate_spaces([space])
    model = network.model()
    assert "IDEAS.Buildings.Components.RectangularZoneTemplate space_1(" in model
    assert "IDEAS.Buildings.Components.OuterWall" not in model
    assert "IDEAS.Buildings.Components.Window" not in model
    # The data package is rendered once at package level and once in the envelope container.
    assert model.count("record external_wall") == model.count("package Data ")
    assert model.count("record  double_glazing") == model.count("package Data ")


@pytest.mark.parametrize(
    ("tilt", "table"),
    [
        (Tilt.wall, "Cp_Wall"),
        (Tilt.ceiling, "Cp_Roof_0_10"),
        (Tilt.floor, "Cp_Floor"),
        (Tilt.pitched_roof_20, "Cp_Roof_11_30"),
        (Tilt.pitched_roof_30, "Cp_Roof_30_45"),
        (Tilt.pitched_roof_45, "Cp_Roof_30_45"),
    ],
)
def test_wind_pressure_table_follows_ideas_selection(tilt: Tilt, table: str) -> None:
    assert wind_pressure_table(tilt) == table


def test_tilt_macro_and_python_mapping_agree() -> None:
    template = compile_template("{% import 'macros.jinja2' as macros %}{{ macros.convert_tilt(tilt, 'IDEAS') }}")
    for tilt in Tilt:
        rendered = template.render(tilt=tilt)
        expected = TILT_RADIANS[tilt.value]
        if rendered.startswith("IDEAS.Types.Tilt."):
            assert expected == {"Wall": math.pi / 2, "Ceiling": 0.0, "Floor": math.pi}[rendered.split(".")[-1]]
        else:
            assert float(rendered) == expected


def test_merged_walls_split_by_wind_pressure_table() -> None:
    walls = [_wall("south", Azimuth.south), _wall("roof", Azimuth.south, tilt=Tilt.pitched_roof_45)]
    merged = MergedExternalWall.from_base_elements(walls)
    assert [(m.name, m.wind_pressure_table) for m in merged] == [
        ("merged_roof", "Cp_Roof_30_45"),
        ("merged_south", "Cp_Wall"),
    ]
