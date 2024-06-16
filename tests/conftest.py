import tempfile
from contextlib import contextmanager
from pathlib import Path

import docker
import jinja2
import pytest

from neosim.construction import Constructions
from neosim.glass import Glasses
from neosim.library.library import Buildings, Ideas, Library
from neosim.models.constants import Azimuth, Flow, Tilt
from neosim.models.elements.ahu import AirHandlingUnit
from neosim.models.elements.base import Port
from neosim.models.elements.boiler import Boiler
from neosim.models.elements.boundary import Boundary
from neosim.models.elements.controls.ahu import AhuControl
from neosim.models.elements.controls.boiler import BoilerControl
from neosim.models.elements.controls.collector import CollectorControl
from neosim.models.elements.controls.emission import EmissionControl
from neosim.models.elements.controls.three_way_valve import ThreeWayValveControl
from neosim.models.elements.controls.vav import VAVControl
from neosim.models.elements.damper import VAV, DamperVariant
from neosim.models.elements.duct import Duct
from neosim.models.elements.envelope.external_wall import ExternalDoor, ExternalWall
from neosim.models.elements.envelope.floor_on_ground import FloorOnGround
from neosim.models.elements.envelope.internal_element import InternalElement
from neosim.models.elements.envelope.window import Window
from neosim.models.elements.occupancy import Occupancy
from neosim.models.elements.pump import Pump, PumpParameters
from neosim.models.elements.radiator import Radiator
from neosim.models.elements.space import Space, SpaceParameter
from neosim.models.elements.split_valve import SplitValve, SplitValveParameters
from neosim.models.elements.system import System
from neosim.models.elements.temperature_sensor import TemperatureSensor
from neosim.models.elements.three_way_valve import (
    ThreeWayValve,
    ThreeWayValveParameters,
)
from neosim.models.elements.valve import Valve
from neosim.models.elements.weather import Weather
from neosim.topology import Network


@pytest.fixture(scope="session")
def client() -> docker.DockerClient:
    client = docker.DockerClient(base_url="unix://var/run/docker.sock")
    return client


@pytest.fixture(scope="session")
def container(client: docker.DockerClient) -> None:
    container = client.containers.run(
        "openmodelica/openmodelica:v1.22.4-ompython",
        command="tail -f /dev/null",
        volumes=[
            f"{str(Path(__file__).parents[1])}:/neosim",
            f"{str(Path(__file__).parents[1])}/results:/results",
        ],
        detach=True,
    )
    container.exec_run(cmd="omc /neosim/neosim/library/install_package.mos")
    yield container
    container.exec_run(
        cmd='find / -name "*_res.mat" -exec cp {} /results \;'  # noqa: W605
    )  # noqa: W605
    container.stop()
    container.remove()


@contextmanager
def create_mos_file(
    network: Network, check_only: bool = False, end_time: int = 3600
) -> str:
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
                f"""
    getVersion();
    loadFile("/neosim/tests/{{{{model_file}}}}");
    checkModel({{{{model_name}}}}.building);
    simulate({{{{model_name}}}}.building,startTime = 0, stopTime = {end_time});
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
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        library=Buildings(
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
        library=Ideas(
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
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        library=Buildings(
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
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        library=Buildings(
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
        library=Ideas(
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
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        emissions=[
            Valve(name="valve", control=EmissionControl(name="emission_valve_control")),
            Radiator(name="emission"),
        ],
    )
    return space_1


@pytest.fixture
def space_2() -> Space:
    space_2 = Space(
        name="space_2",
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
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
        emissions=[
            Valve(
                name="valve_2", control=EmissionControl(name="emission_valve_control_2")
            ),
            Radiator(name="emission_2"),
        ],
    )
    return space_2


@pytest.fixture
def space_3() -> Space:
    space_3 = Space(
        name="space_3",
        occupancy=Occupancy(name="occupancy_2"),
        external_boundaries=[
            ExternalWall(
                name="w1_3",
                surface=10,
                azimuth=Azimuth.west,
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
        emissions=[
            Valve(
                name="valve_3", control=EmissionControl(name="emission_valve_control_3")
            ),
            Radiator(name="emission_3"),
        ],
    )
    return space_3


@pytest.fixture
def buildings_two_rooms_with_storage(space_1: Space, space_2: Space) -> Network:
    Q_flow_nominal = 2200  # noqa: N806
    scaFacRad = 1.5  # noqa: N806
    TSup_nominal = 273.15 + 50 + 5  # noqa: N806
    TRet_nominal = 273.15 + 40 + 5  # noqa: N806
    dTRad_nominal = TSup_nominal - TRet_nominal  # noqa: N806
    mRad_flow_nominal = scaFacRad * Q_flow_nominal / dTRad_nominal / 4200  # noqa: N806
    dpPip_nominal = 10000  # noqa: N806
    dpVal_nominal = 6000  # noqa: N806
    dpRoo_nominal = 6000  # noqa: N806
    dpThrWayVal_nominal = 6000  # noqa: N806
    dp_nominal = (
        dpPip_nominal + dpVal_nominal + dpRoo_nominal + dpThrWayVal_nominal
    )  # noqa: N806

    network = Network(name="buildings_two_rooms_with_storage")
    network.add_boiler_plate_spaces([space_1, space_2])

    pump = Pump(
        name="pump",
        control=CollectorControl(name="pump_control"),
        parameters=PumpParameters(
            dp_nominal=dp_nominal, m_flow_nominal=mRad_flow_nominal
        ),
    )
    boiler = Boiler(name="boiler", control=BoilerControl(name="boiler_control"))
    split_valve = SplitValve(
        name="split_valve",
        parameters=SplitValveParameters(m_flow_nominal=mRad_flow_nominal),
    )
    three_way_valve_control = ThreeWayValveControl(name="three_way_valve_control")
    three_way_valve = ThreeWayValve(
        name="three_way_valve",
        control=three_way_valve_control,
        parameters=ThreeWayValveParameters(
            m_flow_nominal=mRad_flow_nominal, dp_valve_nominal=dpThrWayVal_nominal
        ),
    )
    temperature_sensor = TemperatureSensor(name="temperature_sensor")
    network.connect_systems(temperature_sensor, space_1.first_emission())
    network.connect_systems(temperature_sensor, space_2.first_emission())
    network.connect_systems(space_1.last_emission(), split_valve)
    network.connect_systems(space_2.last_emission(), split_valve)
    network.connect_systems(pump, temperature_sensor)
    network.connect_systems(three_way_valve, pump)
    network.connect_systems(boiler, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)
    network.connect_systems(three_way_valve_control, temperature_sensor)

    return network


@pytest.fixture
def buildings_simple_hydronic(space_1: Space) -> Network:
    network = Network(name="buildings_simple_hydronic")
    network.add_boiler_plate_spaces([space_1])

    pump = Pump(name="pump", control=CollectorControl(name="pump_control"))
    boiler = Boiler(name="boiler", control=BoilerControl(name="boiler_control"))
    split_valve = SplitValve(name="split_valve")
    t = ThreeWayValveControl(name="three_way_valve_control")
    three_way_valve = ThreeWayValve(name="three_way_valve", control=t)
    temperature_sensor = TemperatureSensor(name="temperature_sensor")
    network.connect_systems(temperature_sensor, space_1.first_emission())
    network.connect_systems(three_way_valve, temperature_sensor)
    network.connect_systems(space_1.last_emission(), split_valve)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)
    network.connect_systems(t, temperature_sensor)

    return network


@pytest.fixture
def buildings_simple_hydronic_three_zones(
    space_1: Space, space_2: Space, space_3: Space
) -> Network:
    network = Network(name="buildings_simple_hydronic_three_zones")
    network.add_boiler_plate_spaces([space_1, space_2, space_3])

    pump = Pump(name="pump", control=CollectorControl(name="pump_control"))
    boiler = Boiler(name="boiler", control=BoilerControl(name="boiler_control"))
    split_valve = SplitValve(name="split_valve")
    three_way_valve_control = ThreeWayValveControl(name="three_way_valve_control")
    three_way_valve_control_2 = ThreeWayValveControl(name="three_way_valve_control_2")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=three_way_valve_control
    )
    split_valve_2 = SplitValve(name="split_valve_2")
    three_way_valve_2 = ThreeWayValve(
        name="three_way_valve_2", control=three_way_valve_control_2
    )
    temperature_sensor = TemperatureSensor(name="temperature_sensor")
    temperature_sensor_2 = TemperatureSensor(name="temperature_sensor_2")
    network.connect_systems(temperature_sensor, space_1.first_emission())
    network.connect_systems(temperature_sensor, space_2.first_emission())
    network.connect_systems(three_way_valve, temperature_sensor)
    network.connect_systems(temperature_sensor_2, space_3.first_emission())
    network.connect_systems(three_way_valve_2, temperature_sensor_2)
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
    network.connect_systems(temperature_sensor, three_way_valve_control)
    network.connect_systems(temperature_sensor_2, three_way_valve_control_2)

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
    network = Network(name="ideas_simple_hydronic_three_zones", library=Ideas())
    network.add_boiler_plate_spaces([space_1, space_2, space_3])

    pump = Pump(name="pump", control=CollectorControl(name="pump_control"))
    boiler = Boiler(name="boiler", control=BoilerControl(name="BoilerControl"))
    split_valve = SplitValve(name="split_valve")
    three_way_valve_control_2 = ThreeWayValveControl(name="three_way_valve_control_2")
    three_way_valve_control = ThreeWayValveControl(name="three_way_valve_control")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=three_way_valve_control
    )
    split_valve_2 = SplitValve(name="split_valve_2")
    three_way_valve_2 = ThreeWayValve(
        name="three_way_valve_2", control=three_way_valve_control_2
    )
    temperature_sensor = TemperatureSensor(name="temperature_sensor")
    temperature_sensor_2 = TemperatureSensor(name="temperature_sensor_2")
    network.connect_systems(temperature_sensor, space_1.first_emission())
    network.connect_systems(temperature_sensor, space_2.first_emission())
    network.connect_systems(three_way_valve, temperature_sensor)
    network.connect_systems(temperature_sensor_2, space_3.first_emission())
    network.connect_systems(three_way_valve_2, temperature_sensor_2)
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
    network.connect_systems(temperature_sensor, three_way_valve_control)
    network.connect_systems(temperature_sensor_2, three_way_valve_control_2)

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
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        emissions=[
            Valve(name="valve", control=EmissionControl(name="emission_control")),
            Radiator(name="emission"),
        ],
    )
    return space_1


@pytest.fixture
def ideas_simple_hydronic_no_occupancy(space_1_no_occupancy: Space) -> Network:
    network = Network(name="ideas_simple_hydronic_no_occupancy", library=Ideas())
    network.add_boiler_plate_spaces([space_1_no_occupancy])

    pump = Pump(name="pump", control=CollectorControl(name="pump_control"))
    boiler = Boiler(name="boiler", control=BoilerControl(name="boiler_control"))
    split_valve = SplitValve(name="split_valve")
    three_way_valve_control = ThreeWayValveControl(name="three_way_valve_control")
    three_way_valve = ThreeWayValve(
        name="three_way_valve", control=three_way_valve_control
    )
    temperature_sensor = TemperatureSensor(name="temperature_sensor")
    network.connect_systems(temperature_sensor, space_1_no_occupancy.first_emission())
    network.connect_systems(three_way_valve, temperature_sensor)
    network.connect_systems(space_1_no_occupancy.last_emission(), split_valve)
    network.connect_systems(boiler, pump)
    network.connect_systems(pump, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)
    network.connect_systems(temperature_sensor, three_way_valve_control)

    return network


@pytest.fixture
def space_1_ideal_heating() -> Space:
    space_1 = Space(
        name="space_1",
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        emissions=[
            Radiator(
                name="emission",
                variant="ideal",
                control=EmissionControl(name="emission_control"),
            )
        ],
    )
    return space_1


@pytest.fixture
def space_1_different_construction_types() -> Space:
    space_1 = Space(
        name="space_1",
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        emissions=[
            Radiator(
                name="emission",
                variant="ideal",
                control=EmissionControl(name="emission_control"),
            )
        ],
    )
    return space_1


@pytest.fixture
def space_1_simple_ventilation() -> Space:
    space_1 = Space(
        name="space_1",
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
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
        ventilation_inlets=[
            Duct(name="pressure_drop_duct_in"),
            VAV(
                name="vav_in",
                control=VAVControl(name="vav_in_control"),
                variant="complex",
            ),
        ],
        ventilation_outlets=[Duct(name="pressure_drop_duct_out")],
    )

    return space_1


@pytest.fixture
def space_2_simple_ventilation() -> Space:
    space_2 = Space(
        name="space_2",
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
        ventilation_inlets=[
            Duct(name="pressure_drop_duct_in_2"),
            VAV(
                name="vav_in_2",
                control=VAVControl(name="vav_in_control_2"),
                variant="complex",
            ),
        ],
        ventilation_outlets=[Duct(name="pressure_drop_duct_out_2")],
    )

    return space_2


@pytest.fixture
def ideas_many_spaces_simple_ventilation(
    space_1_simple_ventilation: Space, space_2_simple_ventilation: Space
) -> Network:
    network = Network(
        name="ideas_many_spaces_simple_ventilation",
        library=Ideas(
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
        library=Buildings(
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
    network = Network(name="space_1_different_construction_types", library=Ideas())
    network.add_boiler_plate_spaces([space_1_different_construction_types])
    return network


@pytest.fixture
def buildings_free_float_single_zone_ahu_complex(
    space_1_simple_ventilation: Space,
) -> Network:
    network = Network(
        name="buildings_free_float_single_zone_ahu_complex",
        library=Buildings(
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


@pytest.fixture
def space_1_ideal_heating_network(space_1_ideal_heating: Space) -> Network:
    network = Network(name="space_1_ideal_heating")
    network.add_boiler_plate_spaces([space_1_ideal_heating])
    return network


@pytest.fixture
def space_1_simple_ventilation_vav_control() -> Space:
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
        ventilation_inlets=[
            Duct(name="pressure_drop_duct_in"),
            VAV(
                name="vav_in",
                control=VAVControl(name="vav_in_control"),
                variant=DamperVariant.complex,
            ),
        ],
        ventilation_outlets=[],
    )

    return space_1


@pytest.fixture
def vav_ventilation_control(space_1_simple_ventilation_vav_control: Space) -> Network:
    network = Network(
        name="vav_ventilation_control",
        library=Buildings(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
    package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    boundary = Boundary(name="boundary")
    network.add_boiler_plate_spaces([space_1_simple_ventilation_vav_control])
    ahu = AirHandlingUnit(name="ahu", control=AhuControl(name="ahu_control"))
    network.connect_systems(
        ahu, space_1_simple_ventilation_vav_control.get_last_ventilation_inlet()
    )
    network.connect_systems(space_1_simple_ventilation_vav_control, ahu)
    network.connect_elements(boundary, ahu)
    weather = [n for n in network.graph.nodes if isinstance(n, Weather)][0]
    network.connect_elements(boundary, weather)
    return network


@pytest.fixture
def one_spaces_air_handling_unit(space_1_simple_ventilation: Space) -> Network:

    network = Network(
        name="one_spaces_air_handling_unit",
        library=Buildings(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces([space_1_simple_ventilation])
    ahu = AirHandlingUnit(name="ahu", control=AhuControl(name="ahu_control"))
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    boundary = Boundary(name="boundary")
    network.connect_elements(boundary, ahu)
    weather = [n for n in network.graph.nodes if isinstance(n, Weather)][0]
    network.connect_elements(boundary, weather)
    return network


@pytest.fixture
def two_spaces_air_handling_unit(
    space_1_simple_ventilation: Space, space_2_simple_ventilation: Space
) -> Network:

    network = Network(
        name="two_spaces_air_handling_unit",
        library=Buildings(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces(
        [space_1_simple_ventilation, space_2_simple_ventilation]
    )
    ahu = AirHandlingUnit(name="ahu", control=AhuControl(name="ahu_control"))
    network.connect_systems(
        ahu, space_1_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        ahu, space_2_simple_ventilation.get_last_ventilation_inlet()
    )
    network.connect_systems(
        space_1_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    network.connect_systems(
        space_2_simple_ventilation.get_last_ventilation_outlet(), ahu
    )
    boundary = Boundary(name="boundary")
    network.connect_elements(boundary, ahu)
    weather = [n for n in network.graph.nodes if isinstance(n, Weather)][0]
    network.connect_elements(boundary, weather)
    return network


@pytest.fixture
def space_with_same_properties() -> Space:
    space = Space(
        name="bed",
        parameters=SpaceParameter(floor_area=11.3, average_room_height=3.75),
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="bw",
                surface=13,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="bw2",
                surface=9.29,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            Window(
                name="window",
                surface=1.3,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1.3,
                construction=Glasses.double_glazing,
            ),
        ],
    )

    return space


@pytest.fixture
def space_with_door() -> Network:
    space = Space(
        name="door",
        parameters=SpaceParameter(floor_area=11.3, average_room_height=3.75),
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalDoor(
                name="door",
                surface=13,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.door,
            ),
            ExternalWall(
                name="wall",
                surface=9.29,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            Window(
                name="window",
                surface=1.3,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1.3,
                construction=Glasses.double_glazing,
            ),
        ],
    )
    network = Network(
        name="space_with_door",
    )
    network.add_boiler_plate_spaces([space])
    return network


def building_with_multiple_internal_walls(
    network_name: str, library: Library = None
) -> Network:
    space_1 = Space(
        name="space_1",
        parameters=SpaceParameter(floor_area=11.3, average_room_height=3.75),
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="bw",
                surface=13,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        parameters=SpaceParameter(floor_area=11.3, average_room_height=3.75),
        occupancy=Occupancy(name="occupancy_1"),
        external_boundaries=[
            ExternalWall(
                name="bw_1",
                surface=13,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
        ],
    )
    network = Network(name=network_name, library=library)
    network.add_boiler_plate_spaces([space_1, space_2], create_internal=False)
    internal_1 = InternalElement(
        name=f"internal_{space_1.name}_{space_2.name}_1",
        surface=10,
        azimuth=45,
        construction=Constructions.internal_wall,
        tilt=Tilt.wall,
    )
    door = InternalElement(
        name=f"internal_{space_1.name}_{space_2.name}_2",
        surface=10,
        azimuth=10,
        construction=Constructions.door,
        tilt=Tilt.wall,
    )
    network.connect_spaces(space_1, space_2, internal_1)
    network.connect_spaces(space_1, space_2, door)
    return network


@pytest.fixture
def building_multiple_internal_walls() -> Network:
    return building_with_multiple_internal_walls("multiple_internal_walls_buildings")


@pytest.fixture
def building_multiple_internal_walls_ideas() -> Network:
    return building_with_multiple_internal_walls(
        "multiple_internal_walls_ideas", library=Ideas()
    )
