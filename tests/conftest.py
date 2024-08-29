import re
from pathlib import Path
from typing import Set

import docker
import pytest

from tests.fixtures.house import house_model_fixture
from tests.fixtures.simple_space_1 import simple_space_1_fixture
from tests.fixtures.simple_space_1_with_occupancy import (
    simple_space_1_with_occupancy_fixture,
)
from tests.fixtures.spaces_with_different_construction_types import (
    space_with_door_fixture,
    space_with_same_properties_fixture,
)
from tests.fixtures.spaces_with_emissions import (
    space_1_fixture,
    space_1_ideal_heating_fixture,
    space_1_no_occupancy_fixture,
    space_2_fixture,
    space_3_fixture,
)
from tests.fixtures.spaces_with_ventilation import (
    space_1_simple_ventilation_fixture,
    space_1_simple_ventilation_vav_control_fixture,
    space_2_simple_ventilation_fixture,
)
from tests.fixtures.three_spaces import three_spaces
from tests.fixtures.two_spaces import two_spaces
from trano.construction import Constructions
from trano.glass import Glasses
from trano.library.library import Buildings, Ideas, Library
from trano.models.constants import Azimuth, Flow, Tilt
from trano.models.elements.base import Port, param_from_config
from trano.models.elements.boundary import Boundary
from trano.models.elements.control import (
    AhuControl,
    BoilerControl,
    CollectorControl,
    EmissionControl,
    ThreeWayValveControl,
)
from trano.models.elements.envelope import (
    ExternalWall,
    FloorOnGround,
    InternalElement,
    Window,
)
from trano.models.elements.space import Space
from trano.models.elements.system import (
    AirHandlingUnit,
    Boiler,
    Occupancy,
    Pump,
    Radiator,
    SplitValve,
    System,
    TemperatureSensor,
    ThreeWayValve,
    Weather,
)
from trano.topology import Network

OVERWRITE_MODELS = False

BoilerParameters = param_from_config("Boiler")
OccupancyParameters = param_from_config("Occupancy")
PumpParameters = param_from_config("Pump")
RadiatorParameter = param_from_config("Radiator")
SpaceParameter = param_from_config("Space")
SplitValveParameters = param_from_config("SplitValve")
ThreeWayValveParameters = param_from_config("ThreeWayValve")


def is_success(results: docker.models.containers.ExecResult) -> bool:
    return "The simulation finished successfully" in results.output.decode()


@pytest.fixture
def result_data_path() -> Path:
    return Path(__file__).parent.joinpath("resources", "data.mat")


@pytest.fixture
def simple_space_1() -> Space:
    return simple_space_1_fixture()


@pytest.fixture
def simple_space_1_with_occupancy() -> Space:
    return simple_space_1_with_occupancy_fixture()


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
annotation (choicesAllMatching = true);"""
        ),
    )
    network.add_boiler_plate_spaces([simple_space_1])
    return network


@pytest.fixture
def buildings_free_float_two_zones() -> Network:

    network = Network(
        name="buildings_free_float_two_zones",
        library=Buildings(
            constants="""package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";"""
        ),
    )
    network.add_boiler_plate_spaces(two_spaces())
    return network


@pytest.fixture
def buildings_free_float_three_zones_spaces() -> list:
    return three_spaces()


@pytest.fixture
def ideas_free_float_three_zones_spaces() -> list:

    return three_spaces(occupancy=False)


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
annotation (choicesAllMatching = true);"""
        ),
    )
    network.add_boiler_plate_spaces(ideas_free_float_three_zones_spaces)
    return network


@pytest.fixture
def space_1() -> Space:
    return space_1_fixture()


@pytest.fixture
def space_2() -> Space:
    return space_2_fixture()


@pytest.fixture
def space_3() -> Space:
    return space_3_fixture()


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
        parameters=SplitValveParameters(m_flow_nominal=str(mRad_flow_nominal)),
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

    return space_1_no_occupancy_fixture()


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
    return space_1_ideal_heating_fixture()


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
    return space_1_simple_ventilation_fixture()


@pytest.fixture
def space_2_simple_ventilation() -> Space:

    return space_2_simple_ventilation_fixture()


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
    annotation (choicesAllMatching = true);"""
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
    return space_1_simple_ventilation_vav_control_fixture()


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
    return space_with_same_properties_fixture()


@pytest.fixture
def space_with_door() -> Network:
    space = space_with_door_fixture()
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


@pytest.fixture
def house_model() -> Network:
    return house_model_fixture()


def remove_annotation(model: str) -> str:
    for documentation in re.findall(r"Documentation(.*?)</html>", model, re.DOTALL):
        model = model.replace(documentation, "").replace("Documentation", "")

    model = model.replace(" ", "").replace("\n", "")
    for annotation in re.findall(r"annotation(.*?);", model):
        model = model.replace(annotation, "").replace("annotation", "")

    return model


def remove_common_package(model: str) -> str:
    for annotation in re.findall(r"package Common(.*?)end Common;", model, re.DOTALL):
        model = (
            model.replace(annotation, "")
            .replace("package Common", "")
            .replace("end Common;", "")
        )
    return model


def clean_model(model: str, model_name: str) -> set:
    if OVERWRITE_MODELS:
        path_file = Path(__file__).parent.joinpath("data", f"{model_name}.mo")
        with path_file.open("w") as f:
            f.write(model)
    model = remove_common_package(model)
    model_ = remove_annotation(model)
    return {
        line
        for line in set(
            model_.replace("record", ";").replace(f"model{model_name}", "").split(";")
        )
        if "ReaderTMY3weather" not in line
    }


def _read(file_name: str) -> Set:
    return {
        line
        for line in set(
            remove_annotation(
                remove_common_package(
                    Path(__file__)
                    .parent.joinpath("data", f"{file_name}.mo")
                    .read_text()
                )
            )
            .replace("record", ";")
            .replace(f"model{file_name}", "")
            .split(";")
        )
        if "ReaderTMY3weather" not in line
    }
