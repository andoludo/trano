import pytest

from tests.fixtures.house import house_model_fixture
from tests.fixtures.libraries import ideas_library_fixture
from tests.fixtures.network_builders import (
    building_with_multiple_internal_walls,
    hydronic_network,
    ventilated_network,
)
from tests.fixtures.spaces_with_different_construction_types import space_with_door_fixture
from tests.fixtures.three_spaces import three_spaces
from tests.fixtures.two_spaces import two_spaces
from trano.elements import Port, param_from_config
from trano.elements.boundary import Boundary
from trano.elements.control import AhuControl, BoilerControl, CollectorControl, ThreeWayValveControl
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.system import (
    AirHandlingUnit,
    Boiler,
    Pump,
    SplitValve,
    System,
    TemperatureSensor,
    ThreeWayValve,
    Weather,
)
from trano.elements.types import Flow
from trano.topology import Network

PumpParameters = param_from_config("Pump")
ThreeWayValveParameters = param_from_config("ThreeWayValve")


@pytest.fixture
def buildings_free_float_single_zone(simple_space_1_with_occupancy: Space, buildings_library: Library) -> Network:
    network = Network(
        name="buildings_free_float_single_zone",
        library=buildings_library,
    )
    network.add_boiler_plate_spaces([simple_space_1_with_occupancy])
    return network


@pytest.fixture
def ideas_free_float_single_zone(simple_space_1: Space) -> Network:
    network = Network(name="ideas_free_float_single_zone", library=ideas_library_fixture())
    network.add_boiler_plate_spaces([simple_space_1])
    return network


@pytest.fixture
def buildings_free_float_two_zones(buildings_library: Library) -> Network:
    network = Network(
        name="buildings_free_float_two_zones",
        library=buildings_library,
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
    buildings_free_float_three_zones_spaces: list, buildings_library: Library
) -> Network:
    network = Network(
        name="buildings_free_float_three_zones",
        library=buildings_library,
    )
    network.add_boiler_plate_spaces(buildings_free_float_three_zones_spaces)
    return network


@pytest.fixture
def ideas_free_float_three_zones(
    ideas_free_float_three_zones_spaces: list,
) -> Network:
    network = Network(
        name="ideas_free_float_three_zones",
        library=ideas_library_fixture(),
    )
    network.add_boiler_plate_spaces(ideas_free_float_three_zones_spaces)
    return network


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
    dp_nominal = dpPip_nominal + dpVal_nominal + dpRoo_nominal + dpThrWayVal_nominal

    network = Network(name="buildings_two_rooms_with_storage")
    network.add_boiler_plate_spaces([space_1, space_2])

    pump = Pump(
        name="pump",
        control=CollectorControl(name="pump_control"),
        parameters=PumpParameters(dp_nominal=dp_nominal, m_flow_nominal=mRad_flow_nominal),
    )
    boiler = Boiler(name="boiler", control=BoilerControl(name="boiler_control"))
    split_valve = SplitValve(name="split_valve")
    three_way_valve_control = ThreeWayValveControl(name="three_way_valve_control")
    three_way_valve = ThreeWayValve(
        name="three_way_valve",
        control=three_way_valve_control,
        parameters=ThreeWayValveParameters(m_flow_nominal=mRad_flow_nominal, dp_valve_nominal=dpThrWayVal_nominal),
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
    return hydronic_network("buildings_simple_hydronic", [[space_1]], control_to_sensor=True)


@pytest.fixture
def buildings_simple_hydronic_three_zones(space_1: Space, space_2: Space, space_3: Space) -> Network:
    return hydronic_network("buildings_simple_hydronic_three_zones", [[space_1, space_2], [space_3]])


@pytest.fixture
def ideas_simple_hydronic_three_zones(space_1: Space, space_2: Space, space_3: Space) -> Network:
    return hydronic_network(
        "ideas_simple_hydronic_three_zones",
        [[space_1, space_2], [space_3]],
        library=ideas_library_fixture(co2_medium=False),
        boiler_control_name="BoilerControl",
    )


@pytest.fixture
def ideas_simple_hydronic_no_occupancy(space_1_no_occupancy: Space) -> Network:
    return hydronic_network(
        "ideas_simple_hydronic_no_occupancy",
        [[space_1_no_occupancy]],
        library=ideas_library_fixture(co2_medium=False),
    )


@pytest.fixture
def ideas_many_spaces_simple_ventilation(
    space_1_simple_ventilation: Space, space_2_simple_ventilation: Space
) -> Network:
    return ventilated_network(
        "ideas_many_spaces_simple_ventilation",
        [space_1_simple_ventilation, space_2_simple_ventilation],
        library=ideas_library_fixture(),
    )


@pytest.fixture
def many_spaces_simple_ventilation(
    space_1_simple_ventilation: Space,
    space_2_simple_ventilation: Space,
    buildings_library: Library,
) -> Network:
    return ventilated_network(
        "many_spaces_simple_ventilation",
        [space_1_simple_ventilation, space_2_simple_ventilation],
        library=buildings_library,
    )


@pytest.fixture
def one_spaces_air_handling_unit(space_1_simple_ventilation: Space, buildings_library: Library) -> Network:
    return ventilated_network(
        "one_spaces_air_handling_unit",
        [space_1_simple_ventilation],
        library=buildings_library,
        ahu_control=True,
        with_boundary=True,
    )


@pytest.fixture
def two_spaces_air_handling_unit(
    space_1_simple_ventilation: Space,
    space_2_simple_ventilation: Space,
    buildings_library: Library,
) -> Network:
    return ventilated_network(
        "two_spaces_air_handling_unit",
        [space_1_simple_ventilation, space_2_simple_ventilation],
        library=buildings_library,
        ahu_control=True,
        with_boundary=True,
        inlets_first=True,
    )


@pytest.fixture
def space_1_different_construction_types_network(
    space_1_different_construction_types: Space,
) -> Network:
    network = Network(
        name="space_1_different_construction_types",
        library=ideas_library_fixture(co2_medium=False),
    )
    network.add_boiler_plate_spaces([space_1_different_construction_types])
    return network


@pytest.fixture
def buildings_free_float_single_zone_ahu_complex(
    space_1_simple_ventilation: Space, buildings_library: Library
) -> Network:
    network = Network(
        name="buildings_free_float_single_zone_ahu_complex",
        library=buildings_library,
    )
    network.add_boiler_plate_spaces([space_1_simple_ventilation])
    boundary = Boundary(name="boundary")
    ahu = AirHandlingUnit(
        name="ahu",
        template="""  {{package_name}}.Trano.Fluid.Ventilation.AhuWithEconomizer {{element.name}}(redeclare
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
            template="""  {{package_name}}.Trano.Controls.ventilation.AHU_G36 {{element.name}}
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
    network.connect_systems(ahu, space_1_simple_ventilation.get_last_ventilation_inlet())
    network.connect_systems(space_1_simple_ventilation.get_last_ventilation_outlet(), ahu)
    network.connect_elements(boundary, ahu)
    weather = next(n for n in network.graph.nodes if isinstance(n, Weather))
    network.connect_elements(boundary, weather)
    return network


@pytest.fixture
def space_1_ideal_heating_network(space_1_ideal_heating: Space) -> Network:
    network = Network(name="space_1_ideal_heating")
    network.add_boiler_plate_spaces([space_1_ideal_heating])
    return network


@pytest.fixture
def vav_ventilation_control(space_1_simple_ventilation_vav_control: Space, buildings_library: Library) -> Network:
    network = Network(
        name="vav_ventilation_control",
        library=buildings_library,
    )
    boundary = Boundary(name="boundary")
    network.add_boiler_plate_spaces([space_1_simple_ventilation_vav_control])
    ahu = AirHandlingUnit(name="ahu", control=AhuControl(name="ahu_control"))
    network.connect_systems(ahu, space_1_simple_ventilation_vav_control.get_last_ventilation_inlet())
    network.connect_systems(space_1_simple_ventilation_vav_control, ahu)
    network.connect_elements(boundary, ahu)
    weather = next(n for n in network.graph.nodes if isinstance(n, Weather))
    network.connect_elements(boundary, weather)
    return network


@pytest.fixture
def space_with_door() -> Network:
    network = Network(
        name="space_with_door",
    )
    network.add_boiler_plate_spaces([space_with_door_fixture()])
    return network


@pytest.fixture
def building_multiple_internal_walls() -> Network:
    return building_with_multiple_internal_walls("multiple_internal_walls_buildings")


@pytest.fixture
def building_multiple_internal_walls_ideas() -> Network:
    return building_with_multiple_internal_walls(
        "multiple_internal_walls_ideas", library=ideas_library_fixture(co2_medium=False)
    )


@pytest.fixture
def house_model() -> Network:
    return house_model_fixture()
