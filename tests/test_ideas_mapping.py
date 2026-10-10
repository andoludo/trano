"""The IDEAS zone, its envelope and the simulation settings follow the YAML, not IDEAS' own BESTEST example."""

import re

import pytest

from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.fixtures.three_spaces import three_spaces
from tests.golden import remove_trano_package
from trano.elements import FloorOnGround, Weather, Window, param_from_config
from trano.elements.envelope import Overhang, SideFins
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.types import Azimuth, Tilt
from trano.exceptions import InvalidBuildingStructureError
from trano.topology import Network

SpaceParameters = param_from_config("Space")
WeatherParameters = param_from_config("Weather")
assert SpaceParameters is not None and WeatherParameters is not None


def ideas_model(spaces: list[Space], weather: Weather | None = None) -> str:
    network = Network(name="ideas_mapping", library=Library.from_configuration("IDEAS"))
    network.add_boiler_plate_spaces(spaces, weather=weather)
    return re.sub(r"\s+", " ", remove_trano_package(network.model()))


def zone(model: str, name: str = "space_1") -> str:
    start = model.index(f"IDEAS.Buildings.Components.Zone {name}(")
    return model[start : model.index("annotation", start)]


def test_the_default_zone_keeps_the_library_infiltration_and_takes_its_mass_factor_and_start() -> None:
    declaration = zone(ideas_model([simple_space_1_fixture()]))

    assert "n50=" not in declaration
    assert "mSenFac=1.0" in declaration and "T_start=294.15" in declaration


def test_the_infiltration_zone_converts_the_air_change_rate_to_n50() -> None:
    space = simple_space_1_fixture()
    space.variant = "infiltration"
    space.parameters = SpaceParameters(floor_area=48, average_room_height=2.7, ach=0.414)

    model = ideas_model([space])

    assert "n50=0.414*space_1.n50toAch" in zone(model)
    # A fixed infiltration flow needs IDEAS' fixed n50 air exchange, not the pressure driven one.
    assert "interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.None" in model


def test_zones_without_infiltration_keep_the_pressure_driven_air_exchange() -> None:
    assert "InterZonalAirFlow.OnePort" in ideas_model([simple_space_1_fixture()])


def test_the_simulation_manager_is_the_inner_sim_with_the_weather_file() -> None:
    weather = Weather(parameters=WeatherParameters(path='"/simulation/site.mos"'))
    model = ideas_model([simple_space_1_fixture()], weather)

    assert re.search(r'inner IDEAS\.BoundaryConditions\.SimInfoManager sim\(.*?filNam="/simulation/site.mos"\)', model)
    assert "pAtmSou" not in model
    assert "connect(sim.weaDatBus,dataBus)" in model.replace(" ", "") and "weather.weaDatBus" not in model
    assert "linIntRad=true, linExtRad=true" in model


def test_the_radiation_is_not_linearized_when_a_zone_asks_for_the_emissive_power() -> None:
    space = simple_space_1_fixture()
    space.parameters = SpaceParameters(floor_area=48, average_room_height=2.7, linearize_emissive_power="false")

    assert "linIntRad=false, linExtRad=false" in ideas_model([space])


def test_a_floor_over_outdoor_air_follows_the_outdoor_temperature() -> None:
    space = simple_space_1_fixture()
    floor = next(boundary for boundary in space.external_boundaries if isinstance(boundary, FloorOnGround))
    floor.variant = "outdoor_air"
    model = ideas_model([space])

    assert re.search(r"Trano\.ThermalZones\.BoundaryWallOutdoorAir \w+\(.*?inc=IDEAS\.Types\.Tilt\.Floor", model)
    assert "SlabOnGround" not in model


def test_windows_carry_their_frame_fraction_and_frame_u_value() -> None:
    model = ideas_model(three_spaces())

    assert re.search(
        r"frac=\{ 0\.1 \}, redeclare parameter IDEAS\.Buildings\.Data\.Frames\.Wood fraType\(each U_value=", model
    )


def shaded_window(name: str, azimuth: Azimuth, surface: float = 4, **shading: object) -> Window:
    from tests.constructions.constructions import Glasses

    return Window(
        name=name,
        surface=surface,
        width=surface / 2,
        height=2,
        azimuth=azimuth,
        tilt=Tilt.wall,
        construction=Glasses.double_glazing,
        **shading,  # type: ignore[arg-type]
    )


def with_windows(windows: list[Window]) -> Space:
    """A copy of the first fixture space with other windows (the merged envelope is built at construction)."""
    space = three_spaces()[0]
    walls = [wall for wall in space.external_boundaries if not isinstance(wall, Window)]
    return Space(name=space.name, parameters=space.parameters, external_boundaries=walls + windows)


def test_overhang_and_fins_become_one_shading_box_for_the_window_array() -> None:
    overhang, fins = Overhang(depth=1.0, gap=0.5), SideFins(depth=1.0, height=0.5)
    windows = [
        shaded_window(n, a, overhang=overhang, side_fins=fins) for n, a in (("e", Azimuth.east), ("w", Azimuth.west))
    ]
    model = ideas_model([with_windows(windows)])

    assert re.search(
        r"redeclare IDEAS\.Buildings\.Components\.Shading\.Box shaType\( each hWin=2\.0, each wWin=2\.0, "
        r"each wLeft=0\.0, each wRight=0\.0, each ovDep=1\.0, each ovGap=0\.5, each hFin=0\.5, "
        r"each finDep=1\.0, each finGap=0\.0\)",
        model,
    )


def test_an_overhang_alone_is_an_overhang_shading() -> None:
    window = shaded_window("s", Azimuth.south, overhang=Overhang(depth=1.0, width_left=0.5))
    model = ideas_model([with_windows([window])])

    assert re.search(
        r"Shading\.Overhang shaType\( each hWin=2\.0, each wWin=2\.0, each wLeft=0\.5, each wRight=0\.0, "
        r"each dep=1\.0, each gap=0\.0\)",
        model,
    )


def test_shaded_windows_of_one_glazing_must_share_their_size() -> None:
    windows = [
        shaded_window("big", Azimuth.east, surface=6, overhang=Overhang(depth=1.0)),
        shaded_window("small", Azimuth.west, surface=4, overhang=Overhang(depth=1.0)),
    ]

    with pytest.raises(InvalidBuildingStructureError, match="differ in size"):  # the envelope is merged on construction
        with_windows(windows)
