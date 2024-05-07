import tempfile
from contextlib import contextmanager
from pathlib import Path

import docker
import jinja2
import pytest

from neosim.construction import Constructions
from neosim.glass import Glasses
from neosim.library.buildings.buildings import BuildingsLibrary
from neosim.library.ideas.ideas import IdeasLibrary
from neosim.models.constants import Azimuth, Flow, Tilt
from neosim.models.elements.base import Port
from neosim.models.elements.boundary import Boundary
from neosim.models.elements.control import (
    AhuControl,
    Control,
    SpaceControl,
    SpaceSubstanceVentilationControl,
)
from neosim.models.elements.space import Space
from neosim.models.elements.system import (
    VAV,
    AirHandlingUnit,
    Boiler,
    Duct,
    Emission,
    EmissionVariant,
    Occupancy,
    Pump,
    SplitValve,
    System,
    ThreeWayValve,
    Valve,
    Weather,
)
from neosim.models.elements.wall import ExternalWall, FloorOnGround, Window
from neosim.topology import Network


@contextmanager
def create_mos_file(network: Network, check_only: bool = False) -> str:
    model = network.model()
    with tempfile.NamedTemporaryFile(
        mode="w", dir=Path(__file__).parent, suffix=".mo"
    ) as temp_model_file, tempfile.NamedTemporaryFile(
        mode="w", dir=Path(__file__).parent, suffix=".mos"
    ) as temp_mos_file:
        Path(temp_model_file.name).write_text(model)
        environment = jinja2.Environment()
        if check_only:
            template = environment.from_string(
                """
    getVersion();
    loadFile("/neosim/tests/{{model_file}}");
    checkModel({{model_name}}.building);
    """
            )
        else:
            template = environment.from_string(
                """
    getVersion();
    loadFile("/neosim/tests/{{model_file}}");
    checkModel({{model_name}}.building);
    simulate({{model_name}}.building,startTime = 0, stopTime = 3600);
    """
            )
        mos_file = template.render(
            model_file=Path(temp_model_file.name).name, model_name=network.name
        )
        Path(temp_mos_file.name).write_text(mos_file)
        yield Path(temp_mos_file.name).name


def is_success(results: docker.models.containers.ExecResult) -> bool:
    return "The simulation finished successfully" in results.output.decode()


@pytest.fixture
def simple_space_1() -> Space:
    return Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
    )


@pytest.fixture
def simple_space_1_with_occupancy() -> Space:
    return Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
    )


@pytest.fixture
def buildings_free_float_single_zone(simple_space_1_with_occupancy: Space) -> Network:
    network = Network(
        name="buildings_free_float_single_zone",
        library=BuildingsLibrary(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces([simple_space_1_with_occupancy])
    return network


@pytest.fixture
def ideas_free_float_single_zone(simple_space_1: Space) -> Network:
    network = Network(
        name="ideas_free_float_single_zone",
        library=IdeasLibrary(
            constants="""
replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);  inner IDEAS.BoundaryConditions.SimInfoManager
sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));"""
        ),
    )
    network.add_boiler_plate_spaces([simple_space_1])
    return network


@pytest.fixture
def buildings_free_float_two_zones() -> Network:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=100,
        floor_area=50,
        height=2,
        elevation=10,
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            FloorOnGround(
                name="floor_1", surface=10, construction=Constructions.external_wall
            ),
        ],
    )

    network = Network(
        name="buildings_free_float_two_zones",
        library=BuildingsLibrary(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces([space_1, space_2])
    return network


@pytest.fixture
def buildings_free_float_three_zones_spaces() -> list:
    space_1 = Space(
        name="space_1",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            Window(
                name="win1_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            Window(
                name="win2_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            FloorOnGround(
                name="floor_1", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            Window(
                name="win1_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            Window(
                name="win2_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    space_3 = Space(
        name="space_3",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        occupancy=Occupancy(name="occupancy_2"),
        external_boundaries=[
            ExternalWall(
                name="w1_3",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_3",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_3",
                surface=10,
                azimuth=Azimuth.east,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            Window(
                name="w4_3",
                surface=10,
                azimuth=Azimuth.east,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
        ],
    )

    return [space_1, space_2, space_3]


@pytest.fixture
def ideas_free_float_three_zones_spaces() -> list:
    space_1 = Space(
        name="space_1",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            Window(
                name="win1_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            Window(
                name="win2_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            FloorOnGround(
                name="floor_1", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            Window(
                name="win1_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            Window(
                name="win2_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    space_3 = Space(
        name="space_3",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_3",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_3",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_3",
                surface=10,
                azimuth=Azimuth.east,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            Window(
                name="w4_3",
                surface=10,
                azimuth=Azimuth.east,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
        ],
    )

    return [space_1, space_2, space_3]


@pytest.fixture
def buildings_free_float_three_zones(
    buildings_free_float_three_zones_spaces: list,
) -> Network:
    network = Network(
        name="buildings_free_float_three_zones",
        library=BuildingsLibrary(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces(buildings_free_float_three_zones_spaces)
    return network


@pytest.fixture
def ideas_free_float_three_zones(
    ideas_free_float_three_zones_spaces: list,
) -> Network:
    network = Network(
        name="ideas_free_float_three_zones",
        library=IdeasLibrary(
            constants="""
replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);  inner IDEAS.BoundaryConditions.SimInfoManager
sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));"""
        ),
    )
    network.add_boiler_plate_spaces(ideas_free_float_three_zones_spaces)
    return network


@pytest.fixture
def space_1() -> Space:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        emissions=[Valve(name="valve"), Emission(name="emission")],
        control=SpaceControl(name="space_control"),
    )
    return space_1


@pytest.fixture
def space_2() -> Space:
    space_2 = Space(
        name="space_2",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_2",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_2",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        emissions=[Valve(name="valve_2"), Emission(name="emission_2")],
        control=SpaceControl(name="space_control_2"),
    )
    return space_2


@pytest.fixture
def space_3() -> Space:
    space_3 = Space(
        name="space_3",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_2"),
        external_boundaries=[
            ExternalWall(
                name="w1_3",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_3",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_3",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_3",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_4", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_3",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        emissions=[Valve(name="valve_3"), Emission(name="emission_3")],
        control=SpaceControl(name="space_control_3"),
    )
    return space_3


@pytest.fixture
def buildings_simple_hydronic(space_1: Space) -> Network:
    network = Network(name="buildings_simple_hydronic")
    network.add_boiler_plate_spaces([space_1])

    pump = Pump(name="pump", control=Control(name="pump_control"))
    boiler = Boiler(name="boiler")
    split_valve = SplitValve(name="split_valve")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=Control(name="three_way_valve_control")
    )
    network.connect_systems(three_way_valve, space_1.first_emission())
    network.connect_systems(space_1.last_emission(), split_valve)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)

    return network


@pytest.fixture
def buildings_simple_hydronic_two_zones(space_1: Space, space_2: Space) -> Network:
    network = Network(name="buildings_simple_hydronic_two_zones")
    network.add_boiler_plate_spaces([space_1, space_2])

    pump = Pump(name="pump", control=Control(name="pump_control"))
    boiler = Boiler(name="boiler")
    split_valve = SplitValve(name="split_valve")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=Control(name="three_way_valve_control")
    )
    network.connect_systems(three_way_valve, space_1.first_emission())
    network.connect_systems(three_way_valve, space_2.first_emission())
    network.connect_systems(space_1.last_emission(), split_valve)
    network.connect_systems(space_2.last_emission(), split_valve)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)

    return network


@pytest.fixture
def buildings_simple_hydronic_three_zones(
    space_1: Space, space_2: Space, space_3: Space
) -> Network:
    network = Network(name="buildings_simple_hydronic_three_zones")
    network.add_boiler_plate_spaces([space_1, space_2, space_3])

    pump = Pump(name="pump", control=Control(name="pump_control"))
    boiler = Boiler(name="boiler")
    split_valve = SplitValve(name="split_valve")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=Control(name="three_way_valve_control")
    )
    split_valve_2 = SplitValve(name="split_valve_2")
    three_way_valve_2 = ThreeWayValve(
        name="three_way_valve_2", control=Control(name="three_way_valve_control_2")
    )
    network.connect_systems(three_way_valve, space_1.first_emission())
    network.connect_systems(three_way_valve, space_2.first_emission())
    network.connect_systems(three_way_valve_2, space_3.first_emission())
    network.connect_systems(space_1.last_emission(), split_valve)
    network.connect_systems(space_2.last_emission(), split_valve)
    network.connect_systems(space_3.last_emission(), split_valve_2)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(pump, three_way_valve_2)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(three_way_valve_2, split_valve_2)
    network.connect_systems(split_valve, boiler)
    network.connect_systems(split_valve_2, boiler)

    # # check if controllable # noqa : E800
    # if pump.get_controllable_ports(): # noqa : E800
    #     pump_control = Control(name="pump_control") # noqa : E800
    #     network.graph.add_edge(pump, pump_control) # noqa : E800
    #
    # if three_way_valve.get_controllable_ports(): # noqa : E800
    #     three_way_valve_control = Control(name="three_way_valve_control") # noqa : E800
    #     network.graph.add_edge(three_way_valve, three_way_valve_control) # noqa : E800
    # undirected_graph = network.graph.to_undirected() # noqa : E800
    # space_controls = [node for node in undirected_graph.nodes if isinstance(node, SpaceControl)] # noqa : E800
    # paths = shortest_path(undirected_graph, pump_control, space_controls[0]) # noqa : E800
    return network


@pytest.fixture
def ideas_simple_hydronic_three_zones(
    space_1: Space, space_2: Space, space_3: Space
) -> Network:
    network = Network(name="ideas_simple_hydronic_three_zones", library=IdeasLibrary())
    network.add_boiler_plate_spaces([space_1, space_2, space_3])

    pump = Pump(name="pump", control=Control(name="pump_control"))
    boiler = Boiler(name="boiler")
    split_valve = SplitValve(name="split_valve")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=Control(name="three_way_valve_control")
    )
    split_valve_2 = SplitValve(name="split_valve_2")
    three_way_valve_2 = ThreeWayValve(
        name="three_way_valve_2", control=Control(name="three_way_valve_control_2")
    )
    network.connect_systems(three_way_valve, space_1.first_emission())
    network.connect_systems(three_way_valve, space_2.first_emission())
    network.connect_systems(three_way_valve_2, space_3.first_emission())
    network.connect_systems(space_1.last_emission(), split_valve)
    network.connect_systems(space_2.last_emission(), split_valve)
    network.connect_systems(space_3.last_emission(), split_valve_2)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(pump, three_way_valve_2)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(three_way_valve_2, split_valve_2)
    network.connect_systems(split_valve, boiler)
    network.connect_systems(split_valve_2, boiler)

    # # check if controllable # noqa : E800
    # if pump.get_controllable_ports(): # noqa : E800
    #     pump_control = Control(name="pump_control") # noqa : E800
    #     network.graph.add_edge(pump, pump_control) # noqa : E800
    #
    # if three_way_valve.get_controllable_ports(): # noqa : E800
    #     three_way_valve_control = Control(name="three_way_valve_control") # noqa : E800
    #     network.graph.add_edge(three_way_valve, three_way_valve_control) # noqa : E800
    # undirected_graph = network.graph.to_undirected() # noqa : E800
    # space_controls = [node for node in undirected_graph.nodes if isinstance(node, SpaceControl)] # noqa : E800
    # paths = shortest_path(undirected_graph, pump_control, space_controls[0]) # noqa : E800
    return network


@pytest.fixture
def space_1_no_occupancy() -> Space:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        emissions=[Valve(name="valve"), Emission(name="emission")],
        control=SpaceControl(name="space_control"),
    )
    return space_1


@pytest.fixture
def ideas_simple_hydronic_no_occupancy(space_1_no_occupancy: Space) -> Network:
    network = Network(name="ideas_simple_hydronic_no_occupancy", library=IdeasLibrary())
    network.add_boiler_plate_spaces([space_1_no_occupancy])

    pump = Pump(name="pump", control=Control(name="pump_control"))
    boiler = Boiler(name="boiler")
    split_valve = SplitValve(name="split_valve")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=Control(name="three_way_valve_control")
    )
    network.connect_systems(three_way_valve, space_1_no_occupancy.first_emission())
    network.connect_systems(space_1_no_occupancy.last_emission(), split_valve)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)

    return network


@pytest.fixture
def space_1_ideal_heating() -> Space:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        emissions=[Emission(name="emission", variant=EmissionVariant.ideal)],
        control=SpaceControl(name="space_control"),
    )
    return space_1


@pytest.fixture
def space_1_different_construction_types() -> Space:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.internal_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.test_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            Window(
                name="win1_2",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.simple_glazing,
            ),
        ],
        emissions=[Emission(name="emission", variant=EmissionVariant.ideal)],
        control=SpaceControl(name="space_control"),
    )
    return space_1


@pytest.fixture
def space_1_simple_ventilation() -> Space:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        ventilation_inlets=[Duct(name="pressure_drop_duct_in"), VAV(name="vav_in")],
        ventilation_outlets=[VAV(name="vav_out"), Duct(name="pressure_drop_duct_out")],
        ventilation_control=SpaceSubstanceVentilationControl(
            name="ventilation_control"
        ),
    )

    return space_1


@pytest.fixture
def space_2_simple_ventilation() -> Space:
    space_2 = Space(
        name="space_2",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_2",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
        ventilation_inlets=[Duct(name="pressure_drop_duct_in_1"), VAV(name="vav_in_2")],
        ventilation_outlets=[
            VAV(name="vav_out_2"),
            Duct(name="pressure_drop_duct_out_2"),
        ],
        ventilation_control=SpaceSubstanceVentilationControl(
            name="ventilation_control_2"
        ),
    )

    return space_2


@pytest.fixture
def ideas_many_spaces_simple_ventilation(
    space_1_simple_ventilation: Space, space_2_simple_ventilation: Space
) -> Network:
    network = Network(
        name="ideas_many_spaces_simple_ventilation",
        library=IdeasLibrary(
            constants="""
    replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Medium in the component"
    annotation (choicesAllMatching = true);  inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                                  "Data reader"
        annotation (Placement(transformation(extent={{-96,76},{-76,96}})));"""
        ),
    )
    network.add_boiler_plate_spaces(
        [space_1_simple_ventilation, space_2_simple_ventilation]
    )
    ahu = AirHandlingUnit(name="ahu")
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )

    network.connect_systems(
        ahu, space_2_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_2_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    return network


@pytest.fixture
def many_spaces_simple_ventilation(
    space_1_simple_ventilation: Space, space_2_simple_ventilation: Space
) -> Network:
    network = Network(
        name="many_spaces_simple_ventilation",
        library=BuildingsLibrary(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
    package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces(
        [space_1_simple_ventilation, space_2_simple_ventilation]
    )
    ahu = AirHandlingUnit(name="ahu")
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )

    network.connect_systems(
        ahu, space_2_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_2_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    return network


@pytest.fixture
def space_1_different_construction_types_network(
    space_1_different_construction_types: Space,
) -> Network:
    network = Network(
        name="space_1_different_construction_types", library=IdeasLibrary()
    )
    network.add_boiler_plate_spaces([space_1_different_construction_types])
    return network


@pytest.fixture
def buildings_free_float_single_zone_ahu_complex(
    space_1_simple_ventilation: Space,
) -> Network:
    network = Network(
        name="single_space_ahu",
        library=BuildingsLibrary(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
        package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces([space_1_simple_ventilation])
    boundary = Boundary(name="boundary")
    ahu = AirHandlingUnit(
        name="ahu",
        template="""  {{package_name}}.Common.Fluid.Ventilation.AhuWithEconomizer {{element.name}}(redeclare
        package
              MediumA =                                                                    Medium,
      VRoo={100,100},
      AFlo={20,20},
      mCooVAV_flow_nominal={0.01,0.01})
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));""",
        ports=[
            Port(
                targets=[System],
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[System],
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[Boundary],
                names=["ports"],
            ),
            Port(
                targets=[AhuControl],
                names=["dataBus"],
            ),
        ],
        control=AhuControl(
            name="ahu_control",
            template="""  {{package_name}}.Common.Controls.ventilation.AHU_G36 {{element.name}}
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));""",
            ports=[
                Port(
                    targets=[AirHandlingUnit],
                    names=["dataBus"],
                ),
            ],
        ),
    )
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    network.connect_elements(boundary, ahu)
    weather = [n for n in network.graph.nodes if isinstance(n, Weather)][0]
    network.connect_elements(boundary, weather)
    return network
