"""Envelope of a space as the AixLib reduced-order (VDI 6007) and ISO 13790 zones take it.

Expected values are derived by hand from the layer data: EN ISO 6946 surface resistances
(0.13 wall, 0.10 roof, 0.17 floor inside, 0.04 outside) and lumped elements made of two
equal halves of the conduction resistance around the capacitance, in parallel.
"""

import math
import re

import pytest

from tests.constructions.constructions import Glasses, Materials
from tests.fixtures.three_spaces import three_spaces
from tests.golden import remove_trano_package
from tests.test_yaml_values import NUMBER
from trano.elements import ExternalWall, FloorOnGround, Window
from trano.elements.aggregated_envelope import AggregatedEnvelope
from trano.elements.envelope import assign_windows_to_walls
from trano.elements.construction import Construction, Layer
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.types import Azimuth, Tilt
from trano.topology import Network

CONCRETE = Construction(name="concrete", layers=[Layer(material=Materials.concrete, thickness=0.2)])
INSULATED = Construction(
    name="insulated",
    layers=[
        Layer(material=Materials.concrete, thickness=0.2),
        Layer(material=Materials.insulation_board, thickness=0.1),
    ],
)
CONCRETE_RESISTANCE = 0.2 / 1.4  # [m2.K/W]
CONCRETE_CAPACITANCE = 0.2 * 2240 * 840  # [J/(m2.K)]
INSULATED_RESISTANCE = CONCRETE_RESISTANCE + 0.1 / 0.03
INSULATED_CAPACITANCE = CONCRETE_CAPACITANCE + 0.1 * 40 * 1200


def wall(
    name: str,
    surface: float,
    azimuth: float = Azimuth.south,
    tilt: Tilt = Tilt.wall,
    construction: Construction = CONCRETE,
) -> ExternalWall:
    return ExternalWall(name=name, surface=surface, azimuth=azimuth, tilt=tilt, construction=construction)


def window(name: str, surface: float, azimuth: float = Azimuth.south) -> Window:
    return Window(name=name, surface=surface, azimuth=azimuth, tilt=Tilt.wall, construction=Glasses.double_glazing)


def envelope_of(*boundaries: ExternalWall | Window | FloorOnGround) -> AggregatedEnvelope:
    """The space cuts the windows out of their walls before the envelope is aggregated."""
    assign_windows_to_walls(list(boundaries))
    return AggregatedEnvelope.from_boundaries(list(boundaries))


def test_exterior_walls_are_lumped_in_kelvin_per_watt() -> None:
    walls = AggregatedEnvelope.from_boundaries(
        [wall("south", 40), wall("west", 60, Azimuth.west, construction=INSULATED)]
    ).exterior_walls

    half_conductance = 40 / (CONCRETE_RESISTANCE / 2) + 60 / (INSULATED_RESISTANCE / 2)
    conductance = 40 / (0.13 + CONCRETE_RESISTANCE + 0.04) + 60 / (0.13 + INSULATED_RESISTANCE + 0.04)
    assert walls.area == 100
    assert walls.resistance == pytest.approx(1 / half_conductance, rel=1e-5)
    assert walls.resistance_remaining == walls.resistance
    assert walls.capacitance == pytest.approx(40 * CONCRETE_CAPACITANCE + 60 * INSULATED_CAPACITANCE, rel=1e-5)
    assert walls.u_value == pytest.approx(conductance / 100, rel=1e-5)


def test_surfaces_facing_the_same_way_form_one_orientation() -> None:
    envelope = envelope_of(
        wall("west_rounded", 10, Azimuth.west),
        wall("west_exact", 15, math.pi / 2),
        wall("south", 20),
        wall("east", 5, Azimuth.east),
        window("south_window", 4),
        window("east_window", 2, Azimuth.east),
    )

    # One entry per orientation, sorted by azimuth: east, south, west; the windows cut out of their walls.
    assert envelope.azimuths == [-1.57, 0.0, 1.57]
    assert envelope.opaque_areas == [3.0, 16.0, 25.0]
    assert envelope.window_areas == [2.0, 4.0, 0.0]
    assert envelope.tilts == [pytest.approx(math.pi / 2, rel=1e-5)] * 3


def test_roofs_are_only_part_of_the_roof() -> None:
    envelope = AggregatedEnvelope.from_boundaries(
        [
            wall("south", 20),
            wall("flat_roof", 50, tilt=Tilt.ceiling),
            wall("pitched_roof", 30, Azimuth.north, tilt=Tilt.pitched_roof_35, construction=INSULATED),
        ]
    )

    assert envelope.opaque_areas == [20.0]
    assert envelope.exterior_walls.area == 20
    assert envelope.roof.area == 80
    assert envelope.roof.u_value == pytest.approx(
        (50 / (0.10 + CONCRETE_RESISTANCE + 0.04) + 30 / (0.10 + INSULATED_RESISTANCE + 0.04)) / 80, rel=1e-5
    )
    assert envelope.roof_tilts == [0.0, pytest.approx(math.radians(35), rel=1e-5)]
    assert envelope.roof_azimuths == [0.0, 3.14]
    assert sum(envelope.roof_weighting_factors) == pytest.approx(1)


def test_weighting_factors_are_the_conductance_shares_of_the_orientations() -> None:
    envelope = envelope_of(
        wall("south", 30),
        wall("north", 10, Azimuth.north),
        wall("west", 10, Azimuth.west, construction=INSULATED),
        window("south_window", 3),
        window("north_window", 1, Azimuth.north),
    )
    south, west, north = (
        area / (0.13 + resistance + 0.04)
        for area, resistance in [
            (30 - 3, CONCRETE_RESISTANCE),  # opaque part: the window is cut out
            (10, INSULATED_RESISTANCE),
            (10 - 1, CONCRETE_RESISTANCE),
        ]
    )

    # Sorted by azimuth: south, west, north. VDI 6007 needs each set of factors to sum to 1.
    assert envelope.wall_weighting_factors == pytest.approx(
        [south / (south + west + north), west / (south + west + north), north / (south + west + north)], rel=1e-5
    )
    assert envelope.window_weighting_factors == [0.75, 0.0, 0.25]


def test_windows_are_lumped_from_their_glazing() -> None:
    glazing = Glasses.double_glazing.properties
    windows = envelope_of(
        wall("south", 30),
        wall("west", 10, Azimuth.west),
        window("south_window", 3),
        window("west_window", 2, Azimuth.west),
    ).windows

    assert windows.area == 5
    assert windows.resistance == pytest.approx(glazing.internal_resistance / 5, rel=1e-5)
    assert windows.u_value == pytest.approx(glazing.u_value, rel=1e-5)
    assert windows.g_value == pytest.approx(glazing.g_value, rel=1e-5)


def test_floor_on_ground_has_no_exterior_surface_resistance() -> None:
    envelope = AggregatedEnvelope.from_boundaries([FloorOnGround(name="floor", surface=50, construction=CONCRETE)])

    assert envelope.floor.area == 50
    assert envelope.floor.u_value == pytest.approx(1 / (0.17 + CONCRETE_RESISTANCE), rel=1e-5)
    assert envelope.floor.resistance == pytest.approx(CONCRETE_RESISTANCE / 2 / 50, rel=1e-5)
    assert envelope.ground_temperature == 283.15


def test_ground_temperature_is_area_weighted() -> None:
    envelope = AggregatedEnvelope.from_boundaries(
        [
            FloorOnGround(name="cold", surface=30, construction=CONCRETE, ground_temperature=280.15),
            FloorOnGround(name="warm", surface=10, construction=CONCRETE, ground_temperature=284.15),
        ]
    )

    assert envelope.ground_temperature == pytest.approx(281.15)


@pytest.mark.parametrize(("conditioned_area", "share"), [(50, 1.0), (100, 0.5), (0, 1.0)])
def test_iso_floor_u_value_keeps_the_ground_conductance(conditioned_area: float, share: float) -> None:
    envelope = AggregatedEnvelope.from_boundaries([FloorOnGround(name="floor", surface=50, construction=CONCRETE)])

    # ISO 13790 multiplies UFlo by the conditioned floor area, not by the floor-on-ground area.
    assert envelope.floor_u_value(conditioned_area) == pytest.approx(envelope.floor.u_value * share, rel=1e-5)


def test_space_without_exterior_boundaries_keeps_valid_parameters() -> None:
    envelope = AggregatedEnvelope.from_boundaries([])

    assert envelope.opaque_areas == [0.0]
    assert envelope.window_areas == [0.0]
    assert envelope.wall_weighting_factors == [1.0]
    assert envelope.window_weighting_factors == [1.0]
    assert envelope.roof_weighting_factors == [1.0]
    assert min(envelope.exterior_walls.resistance, envelope.roof.resistance, envelope.floor.resistance) > 0


# --------------------------------------------------------------------------- #
# Rendered zones
# --------------------------------------------------------------------------- #


def zone_model(library: str) -> tuple[Space, str]:
    spaces = three_spaces()
    network = Network(name=f"{library}_envelope", library=Library.from_configuration(library))
    network.add_boiler_plate_spaces(spaces)
    model = remove_trano_package(network.model())
    start = model.index(" space_1(")
    return spaces[0], model[start : model.index("annotation", start)]


def parameter(declaration: str, name: str) -> list[float]:
    """Numbers of a scalar or array parameter of the zone declaration."""
    match = re.search(rf"(?<![\w.]){name}\s*=\s*(\{{[^}}]*\}}|[^,\n]+)", declaration)
    assert match, f"{name} not rendered"
    return [float(number) for number in re.findall(NUMBER, match.group(1))]


def test_reduced_order_zone_takes_the_aggregated_envelope() -> None:
    space, declaration = zone_model("reduced_order")
    envelope = space.aggregated_envelope

    assert parameter(declaration, "nOrientations") == [4]
    # Orientations sorted by azimuth: east, south, west, north; windows east and south.
    assert parameter(declaration, "aziExtWalls") == [-1.57, 0, 1.57, 3.14]
    # Walls of 10 m2 everywhere; the 5 m2 windows are cut out of the east and south walls.
    assert parameter(declaration, "AExt") == [5, 5, 10, 10]
    assert parameter(declaration, "AWin") == [5, 5, 0, 0]
    assert parameter(declaration, "ATransparent") == [5, 5, 0, 0]
    assert parameter(declaration, "RExt") == [envelope.exterior_walls.resistance]
    assert parameter(declaration, "RExt")[0] == pytest.approx(
        space.external_boundaries[0].construction.total_thermal_resistance / 2 / 30, rel=1e-5
    )
    assert parameter(declaration, "CExt") == [envelope.exterior_walls.capacitance]
    assert parameter(declaration, "RWin") == [envelope.windows.resistance]
    assert parameter(declaration, "UWin") == [envelope.windows.u_value]
    assert parameter(declaration, "gWin") == [envelope.windows.g_value]
    assert parameter(declaration, "AFloor") == [10]
    assert parameter(declaration, "TSoil") == [283.15]
    assert sum(parameter(declaration, "wfWall")) == pytest.approx(1, abs=1e-5)
    assert sum(parameter(declaration, "wfWin")) == pytest.approx(1, abs=1e-5)
    assert parameter(declaration, "wfGro") == [0]


def test_iso_13790_zone_takes_the_aggregated_envelope() -> None:
    space, declaration = zone_model("iso_13790")
    envelope = space.aggregated_envelope
    floor_area = float(space.parameters.floor_area)  # type: ignore[union-attr]

    assert parameter(declaration, "nOrientations") == [4]
    assert parameter(declaration, "surAzi") == [-1.57, 0, 1.57, 3.14]
    assert parameter(declaration, "AWal") == [5, 5, 10, 10]
    assert parameter(declaration, "AWin") == [5, 5, 0, 0]
    assert parameter(declaration, "UWal") == [envelope.exterior_walls.u_value]
    assert parameter(declaration, "UWin") == [envelope.windows.u_value]
    assert parameter(declaration, "gFac") == [envelope.windows.g_value]
    assert parameter(declaration, "UFlo")[0] == pytest.approx(envelope.floor.u_value * 10 / floor_area, rel=1e-5)


def test_the_mass_class_follows_the_capacity_reached_from_the_room() -> None:
    from trano.elements.construction import Construction, Layer, Material

    concrete = Material(name="concrete", thermal_conductivity=0.51, specific_heat_capacity=1000, density=1400)
    board = Material(name="board", thermal_conductivity=0.16, specific_heat_capacity=840, density=950)
    foam = Material(name="foam", thermal_conductivity=0.04, specific_heat_capacity=1400, density=10)
    light = Construction(
        name="light", layers=[Layer(material=foam, thickness=0.066), Layer(material=board, thickness=0.012)]
    )
    heavy = Construction(
        name="heavy", layers=[Layer(material=foam, thickness=0.06), Layer(material=concrete, thickness=0.2)]
    )

    assert light.internal_heat_capacity == pytest.approx(0.012 * 950 * 840 + 0.066 * 10 * 1400)
    assert heavy.internal_heat_capacity == pytest.approx(0.1 * 1400 * 1000)  # only the first 0.1 m from the room


def test_the_bestest_zones_are_light_and_heavy_for_iso_13790() -> None:
    from trano.data_models.conversion import convert_network
    from trano.elements.space import Space
    from validation.bestest.cases import case_file

    classes = {}
    for case_id in ("600", "900"):
        network = convert_network(
            f"case_{case_id}", case_file(case_id), library=Library.from_configuration("iso_13790")
        )
        zone = next(node for node in network.graph.nodes if isinstance(node, Space))
        classes[case_id] = zone.aggregated_envelope.mass_class(zone.parameters.floor_area)  # type: ignore[union-attr]
    assert classes == {"600": "Light", "900": "Heavy"}
