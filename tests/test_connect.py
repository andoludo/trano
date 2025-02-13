from pathlib import Path

import pytest

from tests.test_validity import get_path
from trano.data_models.conversion import convert_network
from trano.elements import (
    Space,
    connect,
    Radiator,
    InternalElement,
    EmissionControl,
    SplitValve,
    Port, DataBus, ThreeWayValve, TemperatureSensor, CollectorControl,
)
from trano.elements.base import ElementPort, Control
from trano.elements.common_base import BasePosition
from trano.elements.containers import containers_factory
from trano.elements.envelope import MergedExternalWall, MergedWindows, FloorOnGround
from trano.elements.system import Occupancy, Weather, Valve, Duct, Pump, VAV, Boiler
from trano.elements.types import Flow, Medium
from trano.library.library import Library
from trano.topology import Network


@pytest.fixture(scope="module")
def house_ideas() -> Network:
    house = get_path("three_zones_hydronic_containers.yaml")
    return convert_network(
        "three_zones_hydronic_containers",
        house,
        library=Library.from_configuration("IDEAS"),
    )
@pytest.fixture(scope="module")
def house_buildings() -> Network:
    house = get_path("three_zones_hydronic_containers.yaml")
    return convert_network(
        "three_zones_hydronic_containers",
        house,
        library=Library.from_configuration("Buildings"),
    )

def test_connect_space_radiator(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, Radiator)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 2
    assert {c.equation_view() for c in connections} == {
        ("radiator_003.heatPortCon", "space_001.gainCon"),
        ("radiator_003.heatPortRad", "space_001.gainRad"),
    }


def test_connect_space_internal_wall(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, InternalElement)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("internal_space_001_space_002_cavitywall.propsBus_a", "space_001.propsBus[1]")
    }

def test_connect_space_internal_wall_buildings(house_buildings: Network) -> None:

    edge = house_buildings.get_edge(Space, InternalElement)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('internal_space_001_space_002_cavitywall.port_a', 'space_001.surf_surBou[1]')}


def test_connect_space_merged_external_wall(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, MergedExternalWall)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        (
            "merged_externalwall_17_externalwall_18_externalwall_19[1:3].propsBus_a",
            "space_001.propsBus[1:3]",
        )
    }


def test_connect_space_merged_windows(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, MergedWindows)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("merged_window_7_window_8[1:2].propsBus_a", "space_001.propsBus[1:2]")
    }


def test_connect_space_floor_on_ground(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, FloorOnGround)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1_ = ElementPort.from_element_without_ports(edge[0])
    e2_ = ElementPort.from_element_without_ports(edge[1])
    for edge in [(e1, e2), (e2_, e1_)]:
        connections = connect(*edge)
        assert len(connections) == 1
        assert {c.equation_view() for c in connections} == {
            ("flooronground_8.propsBus_a", "space_001.propsBus[1]")
        }


def test_connect_space_occupancy(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, Occupancy)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("occupancy_1.y", "space_001.yOcc")
    }

def test_connect_space_occupancy_buildings(house_buildings: Network) -> None:
    edge = house_buildings.get_edge(Space, Occupancy)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e2, e1)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('occupancy_1.y', 'space_001.qGai_flow')}

def test_connect_space_occupancy_inverted(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, Occupancy)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e2, e1)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("occupancy_1.y", "space_001.yOcc")
    }


def test_connect_radiator_valve(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Radiator, Valve)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('radiator_003.port_b', 'valve_003.port_a')}


def test_connect_valve_emission_control(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Valve, EmissionControl)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {("control_1.y", "valve_003.y")}


def test_connect_valve_split_valve(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Valve, SplitValve)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('split_valve_001.port_1', 'valve_003.port_b')}


def test_connect_multi_connection_split_valve(house_ideas: Network) -> None:
    split_valve = ElementPort(
        name="split_valve_001",
        element_type=SplitValve,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.inlet_or_outlet,
                medium=Medium.fluid,
                names=["port_3"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_1 = ElementPort(
        name="duct_001",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_2 = ElementPort(
        name="duct_002",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_3 = ElementPort(
        name="duct_003",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet_or_outlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    connections_full = []

    connections = connect(duct_1, split_valve)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(split_valve, duct_2)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(duct_3, split_valve)
    connections_full += [c.equation_view() for c in connections]
    assert len(connections_full) == 3
    assert sorted(connections_full) == [('duct_001.port_2', 'split_valve_001.port_1'),
 ('duct_002.port_1', 'split_valve_001.port_2'),
 ('duct_003.port_1', 'split_valve_001.port_3')]

@pytest.fixture
def system_fixtures():
    split_valve = ElementPort(
        name="split_valve_001",
        element_type=SplitValve,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.inlet_or_outlet,
                medium=Medium.fluid,
                names=["port_3"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_1 = ElementPort(
        name="duct_001",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_2 = ElementPort(
        name="duct_002",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_3 = ElementPort(
        name="duct_003",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.inlet_or_outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    duct_4 = ElementPort(
        name="duct_004",
        element_type=Duct,
        container_type="distribution",
        ports=[
            Port(
                flow=Flow.inlet,
                medium=Medium.fluid,
                names=["port_1"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                flow=Flow.outlet,
                medium=Medium.fluid,
                names=["port_2"],
                multi_connection=True,
                use_counter=False,
            ),
        ],
    )
    return split_valve, duct_1, duct_2, duct_3, duct_4

def test_connect_four_connections_split_valve(house_ideas: Network, system_fixtures) -> None:

    connections_full = []
    split_valve, duct_1, duct_2, duct_3, duct_4 = system_fixtures
    connections = connect(duct_1, split_valve)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(split_valve, duct_2)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(duct_3, split_valve)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(duct_4, split_valve)
    connections_full += [c.equation_view() for c in connections]
    assert len(connections_full) == 4
    assert sorted(connections_full) == [('duct_001.port_2', 'split_valve_001.port_1'),
 ('duct_002.port_1', 'split_valve_001.port_2'),
 ('duct_003.port_2', 'split_valve_001.port_3'),
 ('duct_004.port_2', 'split_valve_001.port_1')]
def test_connect_four_connections_split_valve_mixed(house_ideas: Network, system_fixtures) -> None:

    connections_full = []
    split_valve, duct_1, duct_2, duct_3, duct_4 = system_fixtures
    connections = connect(duct_1, split_valve)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(duct_4, split_valve)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(split_valve, duct_2)
    connections_full += [c.equation_view() for c in connections]
    connections = connect(duct_3, split_valve)
    connections_full += [c.equation_view() for c in connections]

    assert len(connections_full) == 4
    assert sorted(connections_full) == [('duct_001.port_2', 'split_valve_001.port_1'),
 ('duct_002.port_1', 'split_valve_001.port_2'),
 ('duct_003.port_2', 'split_valve_001.port_3'),
 ('duct_004.port_2', 'split_valve_001.port_1')]
def test_container_connect_space_radiator(house_ideas: Network) -> None:
    containers = containers_factory()
    edge = house_ideas.get_edge(Space, Radiator)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])

    connections = connect(e1, e2)
    containers.connect(connections)

    assert containers.get_container("envelope").get_equation_view() == {
        ("heatPortCon[1]", "space_001.gainCon"),
        ("heatPortRad[1]", "space_001.gainRad"),
    }
    assert containers.get_container("emission").get_equation_view() == {
        ("heatPortCon[1]", "radiator_003.heatPortCon"),
        ("heatPortRad[1]", "radiator_003.heatPortRad"),
    }


def test_connect_databus(house_ideas: Network) -> None:
    house_ideas._build_data_bus()
    edge = house_ideas.get_edge(Space, DataBus)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 2
    assert {c.equation_view() for c in connections} == {('data_bus.port[1]', 'space_001.gainCon'),
 ('data_bus.port_a[1]', 'space_001.ports[1]')}


def test_connect_databus_buildings(house_buildings: Network) -> None:
    house_buildings._build_data_bus()
    edge = house_buildings.get_edge(Space, DataBus)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 2
    assert {c.equation_view() for c in connections} == {('data_bus.port[1]', 'space_001.heaPorAir'), ('data_bus.port_a[1]', 'space_001.ports[1]')}

def test_connect_three_way_valve_split_valve(house_ideas: Network) -> None:
    edge_1 = house_ideas.get_edge(ThreeWayValve, SplitValve)
    edge_2 = house_ideas.get_edge(ThreeWayValve, Pump)
    edge_3 = house_ideas.get_edge(ThreeWayValve, TemperatureSensor)
    three_way_valve = ElementPort.from_element_without_ports(edge_1[0])
    split_valve = ElementPort.from_element_without_ports(edge_1[1])
    pump = ElementPort.from_element_without_ports(edge_2[0])
    temperature_sensor = ElementPort.from_element_without_ports(edge_3[1])
    split_valve.ports[0].connected = True
    split_valve.ports[1].connected = True
    connections = connect(three_way_valve, temperature_sensor)
    connections += connect(pump, three_way_valve)
    connections += connect(three_way_valve, split_valve)

    assert len(connections) == 3
    assert {c.equation_view() for c in connections} == {('pump_001.port_b', 'three_way_valve_001.port_1'), ('split_valve_001.port_3', 'three_way_valve_001.port_3'), ('temperature_sensor_001.port_a', 'three_way_valve_001.port_2')}


def test_ideas_free_float_single_zone(ideas_free_float_single_zone: Network) -> None:
    edge = ideas_free_float_single_zone.get_edge(Space, MergedWindows)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1.ports[0].connected = True
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('merged_win1_1[1].propsBus_a', 'space_1.propsBus[1]')}

def test_one_spaces_air_handling_unit(one_spaces_air_handling_unit: Network) -> None:
    edge = one_spaces_air_handling_unit.get_edge(Space, VAV)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('space_1.ports[1]', 'vav_in.port_bAir')}

def test_container_connect_emission_distribution(house_ideas: Network) -> None:
    containers = containers_factory()
    house_ideas.assign_container_type()
    edge = house_ideas.get_edge(TemperatureSensor, Radiator)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1.position.set(1, 1)
    e2.position.set(1, 1)
    connections = connect(e1, e2)
    containers.connect(connections)

    assert containers.get_container("distribution").get_equation_view() == {('port_b1[1]', 'temperature_sensor_001.port_b')}
    assert containers.get_container("emission").get_equation_view() == {('port_a[1]', 'radiator_001.port_a')}

def test_container_connect_emission_distribution_split_valve(house_ideas: Network) -> None:
    containers = containers_factory()
    house_ideas.assign_container_type()
    edge = house_ideas.get_edge(SplitValve, Valve)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1.position.set(1, 1)
    e2.position.set(1, 1)
    connections = connect(e1, e2)
    containers.connect(connections)

    assert containers.get_container("distribution").get_equation_view() == {('port_a1[1]', 'split_valve_001.port_1')}
    assert containers.get_container("emission").get_equation_view() == {('port_b[1]', 'valve_003.port_b')}

def test_container_connect_emission_production(house_ideas: Network) -> None:
    containers = containers_factory()
    house_ideas.assign_container_type()
    edge = house_ideas.get_edge(SplitValve, Boiler)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1.position.set(1, 1)
    e2.position.set(1, 1)
    connections = connect(e2, e1)
    containers.connect(connections)

    assert containers.get_container("distribution").get_equation_view() == {('port_a', 'split_valve_001.port_1')}
    assert containers.get_container("production").get_equation_view() == {('boiler_001.port_b', 'port_b1')}

def test_connection_container():
    e1 = ElementPort.model_validate({
    'container_type': 'emission',
    'name': 'radiator_001',
    'ports': [
        {
            'flow': 'inlet',
            'medium': 'fluid',
            'names': ['port_a'],
            'ignore_direction': True
        }
    ],
    'position': {
        'container': {
            'annotation': '''annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position.container.coordinate()) }},
    extent = {% raw %}{{10, -10}, {-10, 10}}
    {% endraw %})));''',
            'location': {'x': 1.0, 'y': 1.0}
        },
        'global_': {'location': {'x': 1.0, 'y': 1.0}}
    }
}
)
    e2 = ElementPort.model_validate({
    'container_type': 'distribution',
    'ports': [
        {
            'flow': 'outlet',
            'medium': 'fluid',
            'multi_connection': True,
            'names': ['port_a'],
            'ignore_direction': True
        },
        {
            'flow': 'inlet',
            'medium': 'fluid',
            'multi_connection': True,
            'names': ['port_b'],
            'ignore_direction': True
        }
    ],
    'position': {}
}
)
    connections = connect(e1, e2)
    assert {c.equation_view() for c in connections} == {('port_a[1]', 'radiator_001.port_a')}

def test_container_connect_control_databus(house_ideas: Network) -> None:
    containers = containers_factory()
    house_ideas._build_data_bus()
    house_ideas.assign_container_type()
    edge = house_ideas.get_edge(CollectorControl, DataBus)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1.position.set(1, 1)
    e2.position.set(1, 1)
    connections = connect(e1, e2)
    containers.connect(connections)

    assert containers.get_container("distribution").get_equation_view() == {('control_5.dataBus', 'dataBus')}

def test_container_envelope_databus(house_ideas: Network) -> None:
    containers = containers_factory()
    house_ideas._build_data_bus()
    house_ideas.assign_container_type()
    edge = house_ideas.get_edge(Space, DataBus)
    e1 = ElementPort.from_element_without_ports(edge[0])
    e2 = ElementPort.from_element_without_ports(edge[1])
    e1.position.set(1, 1)
    e2.position.set(1, 1)
    connections = connect(e1, e2)
    containers.connect(connections)

    assert containers.get_container("bus").get_equation_view() == {('data_bus.port[1]', 'heatPortCon[1]'), ('data_bus.port_a[1]', 'port_b[1]')}
    assert containers.get_container("envelope").get_equation_view() == {('heatPortCon1[1]', 'space_001.gainCon'), ('ports_b[1]', 'space_001.ports[1]')}


def test_container_per_medium_connection(house_ideas: Network) -> None:
    containers = containers_factory()
    house_ideas._build_data_bus()
    house_ideas.assign_container_type()
    containers.assign_nodes(house_ideas.graph.nodes, house_ideas.graph)
    bus_container = containers.get_container("bus")
    bus_container.add_grouped_by_medium_connection()
    assert bus_container.connections
