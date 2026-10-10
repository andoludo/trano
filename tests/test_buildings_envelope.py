"""Envelope of a Buildings MixedAir zone: each wall area counted once, floors on ground on a ground temperature."""

import re

import pytest

from tests.constructions.constructions import Constructions, GasMaterials, Glasses, GlassMaterials
from tests.fixtures.three_spaces import three_spaces
from tests.golden import remove_trano_package
from trano.elements import ExternalWall, Window, param_from_config
from trano.elements.construction import GasLayer, Glass, GlassLayer
from trano.elements.envelope import WallParameters, WindowedWallParameters
from trano.elements.library.library import Library
from trano.elements.types import Azimuth, Tilt
from trano.exceptions import InvalidBuildingStructureError
from trano.topology import Network

TRIPLE_GLAZING = Glass(
    name="triple_glazing",
    u_value_frame=1.4,
    layers=[
        GlassLayer(thickness=0.004, material=GlassMaterials.id_102),
        GasLayer(thickness=0.012, material=GasMaterials.argon),
        GlassLayer(thickness=0.004, material=GlassMaterials.id_102),
        GasLayer(thickness=0.012, material=GasMaterials.argon),
        GlassLayer(thickness=0.004, material=GlassMaterials.id_102),
    ],
)


def wall(name: str, surface: float, azimuth: float) -> ExternalWall:
    return ExternalWall(
        name=name, surface=surface, azimuth=azimuth, tilt=Tilt.wall, construction=Constructions.external_wall
    )


def window(name: str, surface: float, azimuth: float, glazing: Glass = Glasses.double_glazing) -> Window:
    return Window(name=name, surface=surface, azimuth=azimuth, tilt=Tilt.wall, construction=glazing, height=1.0)


def test_windows_of_one_orientation_share_the_walls_of_that_orientation() -> None:
    boundaries = [
        wall("south_1", 10, Azimuth.south),
        wall("south_2", 12, Azimuth.south),
        wall("north", 15, Azimuth.north),
        window("south_window_1", 2, Azimuth.south),
        window("south_window_2", 3, Azimuth.south),
    ]

    windowed = WindowedWallParameters.from_neighbors(boundaries)  # type: ignore[arg-type]
    opaque = WallParameters.from_neighbors(
        "space",
        boundaries,
        ExternalWall,
        filter=windowed.included_external_walls,  # type: ignore[arg-type]
    )

    # One datConExtWin entry: the gross area of both south walls, with both windows.
    assert windowed.number == 1
    assert windowed.surfaces == [22]
    assert windowed.window_width[0] * windowed.window_height[0] == pytest.approx(5)
    assert sorted(windowed.included_external_walls) == ["south_1", "south_2"]
    # The south walls are no longer opaque constructions of their own.
    assert opaque.surfaces == [15]
    assert sum(windowed.surfaces) + sum(opaque.surfaces) == 37


def test_glazings_of_one_orientation_split_its_gross_wall_area() -> None:
    boundaries = [
        wall("west", 20, Azimuth.west),
        window("double", 4, Azimuth.west),
        window("triple", 1, Azimuth.west, glazing=TRIPLE_GLAZING),
    ]

    windowed = WindowedWallParameters.from_neighbors(boundaries)  # type: ignore[arg-type]

    assert windowed.window_layers == ["double_glazing", "triple_glazing"]
    assert windowed.surfaces == pytest.approx([16, 4])
    assert [width * height for width, height in zip(windowed.window_width, windowed.window_height, strict=True)] == (
        pytest.approx([4, 1])
    )


def test_windows_larger_than_their_walls_are_rejected() -> None:
    with pytest.raises(InvalidBuildingStructureError, match="larger than the walls"):
        WindowedWallParameters.from_neighbors([wall("south", 4, Azimuth.south), window("big", 5, Azimuth.south)])  # type: ignore[list-item]


def test_window_needs_a_wall_with_its_orientation() -> None:
    with pytest.raises(InvalidBuildingStructureError, match="No wall found"):
        WindowedWallParameters.from_neighbors([wall("south", 10, Azimuth.south), window("east", 2, Azimuth.east)])  # type: ignore[list-item]


def test_window_surface_must_match_its_dimensions() -> None:
    with pytest.raises(InvalidBuildingStructureError, match="does not match its width"):
        Window(
            name="window",
            surface=2.0,
            width=2.0,
            height=1.2,
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=Glasses.double_glazing,
        )


def test_floor_on_ground_is_held_at_the_ground_temperature() -> None:
    network = Network(name="buildings_ground", library=Library.from_configuration("Buildings"))
    network.add_boiler_plate_spaces(three_spaces())
    model = remove_trano_package(network.model())

    ground = re.search(r"Buildings\.HeatTransfer\.Sources\.FixedTemperature\s+floor_1\s*\(\s*T\s*=\s*([\d.]+)\)", model)
    assert ground and float(ground.group(1)) == pytest.approx(283.15)
    assert re.search(r"connect\(space_1\.surf_conBou\[1\],\s*floor_1\.port\)", model)
    # A prescribed surface temperature cannot also be an initialized state of the floor.
    assert re.search(r"datConBou\([^)]*each stateAtSurface_a=false\)", model)


def test_layers_carry_their_discretization() -> None:
    network = Network(name="buildings_states", library=Library.from_configuration("Buildings"))
    network.add_boiler_plate_spaces(three_spaces())
    model = remove_trano_package(network.model())

    # Buildings' default of 3 states per 0.2 m reference layer, written for every solid layer.
    solids = re.findall(r"Solids\.Generic\((.*?)\)", model, re.DOTALL)
    assert solids and all("nStaRef=3)" in re.sub(r"\s+", "", solid + ")") for solid in solids)


def test_scheduled_ventilation_adds_outdoor_air_to_the_infiltration_zone() -> None:
    parameters = param_from_config("Space")
    assert parameters is not None
    network = Network(name="buildings_ventilation", library=Library.from_configuration("Buildings"))
    space = three_spaces()[0]
    space.variant = "infiltration"
    space.parameters = parameters(floor_area=48, average_room_height=2.7, ach=0.5, ventilation_schedule="[0, 0.4]")
    network.add_boiler_plate_spaces([space])
    model = network.model()

    assert re.search(r"MixedAirInf\s+space_1\([^;]*ventilationSchedule=\[0, 0\.4\]", model)
    assert "+ venSch.y[1])" in model and "table=ventilationSchedule" in model
