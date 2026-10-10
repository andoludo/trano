"""A wall's ``surface`` is the gross area of the facade, its windows included.

Buildings takes the gross area and cuts the window out itself; IDEAS, AixLib, ISO 13790 and the
MPC model take the opaque wall on its own. The same YAML must give every library the same opaque
and glazed areas.
"""

import re

import pytest

from tests.constructions.constructions import Constructions, Glasses
from tests.golden import remove_trano_package
from trano.elements import ExternalWall, FloorOnGround, Window
from trano.elements.envelope import assign_windows_to_walls
from trano.elements.library.library import Library
from trano.elements.space import Space, SpaceVariant
from trano.elements.types import Azimuth, Tilt
from trano.exceptions import InvalidBuildingStructureError
from trano.mpc.estimation import EstimationSettings, zone_envelope
from trano.topology import Network

WALL = 10.0  # [m2] gross south wall
WINDOW = 4.0  # [m2] window in it


def wall(name: str, surface: float, azimuth: float = Azimuth.south) -> ExternalWall:
    return ExternalWall(
        name=name, surface=surface, azimuth=azimuth, tilt=Tilt.wall, construction=Constructions.external_wall
    )


def window(name: str, surface: float, azimuth: float = Azimuth.south) -> Window:
    return Window(name=name, surface=surface, azimuth=azimuth, tilt=Tilt.wall, construction=Glasses.double_glazing)


def room() -> Space:
    return Space(
        name="room",
        external_boundaries=[
            wall("south_wall", WALL),
            window("south_window", WINDOW),
            FloorOnGround(name="floor", surface=20, construction=Constructions.external_wall),
        ],
    )


def generated(library: str, space: Space) -> str:
    network = Network(name="areas", library=Library.from_configuration(library))
    network.add_boiler_plate_spaces([space])
    return re.sub(r"\s+", " ", remove_trano_package(network.model()))


def array(model: str, name: str) -> list[float]:
    match = re.search(rf"(?<![\w.]){name}=\{{ ?([^}}]*?) ?\}}", model)
    assert match, f"{name} not rendered"
    return [float(value) for value in match.group(1).split(",")]


def test_windows_are_cut_out_of_the_walls_facing_the_same_way() -> None:
    boundaries = [wall("big", 30), wall("small", 10), wall("north", 10, Azimuth.north), window("window", 8)]

    assign_windows_to_walls(boundaries)  # type: ignore[arg-type]

    # Spread over the host walls in proportion to their size; other orientations untouched.
    assert [boundary.opaque_surface for boundary in boundaries] == [24.0, 8.0, 10.0, 8.0]
    assert boundaries[0].surface == 30  # the gross area stays what the YAML says


def test_assignment_starts_from_scratch_each_time() -> None:
    boundaries = [wall("wall", WALL), window("window", WINDOW)]
    assign_windows_to_walls(boundaries)  # type: ignore[arg-type]
    assign_windows_to_walls(boundaries)  # type: ignore[arg-type]

    assert boundaries[0].opaque_surface == WALL - WINDOW


def test_a_window_cannot_exceed_its_wall() -> None:
    with pytest.raises(InvalidBuildingStructureError, match="gross area"):
        Space(name="room", external_boundaries=[wall("wall", 3), window("window", 5)])


def test_buildings_gets_the_gross_area_and_cuts_the_window_out_itself() -> None:
    model = generated("Buildings", room())
    windowed_wall = re.search(r"datConExtWin\((.*?)azi=\{[^}]*\}\)", model)

    assert windowed_wall
    assert array(windowed_wall.group(1), "A") == [WALL]  # "opaque construction and window combined"
    assert array(windowed_wall.group(1), "wWin")[0] * array(windowed_wall.group(1), "hWin")[0] == pytest.approx(WINDOW)


def test_ideas_gets_the_opaque_wall_next_to_the_window() -> None:
    model = generated("IDEAS", room())

    assert re.search(rf"OuterWall\[1\] \w+\(.*?A=\{{ {WALL - WINDOW} \}}", model)
    assert re.search(rf"Window\[1\] \w+\(.*?A=\{{ {WINDOW} \}}", model)


def test_ideas_zone_template_gets_the_gross_face_and_cuts_the_window_out_itself() -> None:
    space = room()
    space.variant = SpaceVariant.rectangular_zone
    face = space.rectangular_zone.face("A")

    assert face.area == WALL
    assert face.length == pytest.approx(WALL / space.rectangular_zone.height)
    assert face.window and face.window.area == WINDOW


@pytest.mark.parametrize(
    ("library", "opaque", "glazed"), [("reduced_order", "AExt", "AWin"), ("iso_13790", "AWal", "AWin")]
)
def test_lumped_zones_get_the_opaque_wall_areas(library: str, opaque: str, glazed: str) -> None:
    model = generated(library, room())

    assert array(model, opaque) == [WALL - WINDOW]
    assert array(model, glazed) == [WINDOW]


def test_mpc_model_counts_the_opaque_wall_only() -> None:
    space = room()
    settings = EstimationSettings()
    envelope = zone_envelope(space, settings)
    u_value = 1 / (
        Constructions.external_wall.total_thermal_resistance
        + settings.internal_surface_resistance
        + settings.external_surface_resistance
    )

    assert envelope.opaque_conductance == pytest.approx((WALL - WINDOW) * u_value)
