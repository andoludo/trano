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
    Port, DataBus, ThreeWayValve, TemperatureSensor,
)
from trano.elements.base import ElementPort
from trano.elements.containers import containers_factory
from trano.elements.envelope import MergedExternalWall, MergedWindows, FloorOnGround
from trano.elements.system import Occupancy, Weather, Valve, Duct, Pump
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


def test_connect_space_radiator(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, Radiator)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 2
    assert {c.equation_view() for c in connections} == {
        ("radiator_003.heatPortCon", "space_001.gainCon"),
        ("radiator_003.heatPortRad", "space_001.gainRad"),
    }


def test_connect_space_internal_wall(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, InternalElement)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("internal_space_001_space_002_cavitywall.propsBus_a", "space_001.propsBus[1]")
    }


def test_connect_space_merged_external_wall(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, MergedExternalWall)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
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
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("merged_window_7_window_8[1:2].propsBus_a", "space_001.propsBus[1:2]")
    }


def test_connect_space_floor_on_ground(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, FloorOnGround)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("flooronground_8.propsBus_a", "space_001.propsBus[1]")
    }


def test_connect_space_occupancy(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Space, Occupancy)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {
        ("occupancy_1.y", "space_001.yOcc")
    }


def test_connect_radiator_valve(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Radiator, Valve)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('radiator_003.port_b', 'valve_003.port_a')}


def test_connect_valve_emission_control(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Valve, EmissionControl)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {("control_1.y", "valve_003.y")}


def test_connect_valve_split_valve(house_ideas: Network) -> None:

    edge = house_ideas.get_edge(Valve, SplitValve)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
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
 ('duct_003.port_2', 'split_valve_001.port_3')]


def test_connect_four_connections_split_valve(house_ideas: Network) -> None:
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
                flow=Flow.outlet,
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
    connections_full = []

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

def test_container_connect_space_radiator(house_ideas: Network) -> None:
    containers = containers_factory()
    edge = house_ideas.get_edge(Space, Radiator)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])

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
    data_bus = house_ideas._build_data_bus()
    edge = house_ideas.get_edge(Space, DataBus)
    e1 = ElementPort.from_element(edge[0])
    e2 = ElementPort.from_element(edge[1])
    connections = connect(e1, e2)
    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('data_bus.port_a[1]', 'space_001.ports[1]')}


def test_connect_three_way_valve_split_valve(house_ideas: Network) -> None:
    edge_1 = house_ideas.get_edge(ThreeWayValve, SplitValve)
    edge_2 = house_ideas.get_edge(ThreeWayValve, Pump)
    edge_3 = house_ideas.get_edge(ThreeWayValve, TemperatureSensor)
    three_way_valve = ElementPort.from_element(edge_1[0])
    split_valve = ElementPort.from_element(edge_1[1])
    pump = ElementPort.from_element(edge_2[0])
    temperature_sensor = ElementPort.from_element(edge_3[1])
    split_valve.ports[0].connected = True
    split_valve.ports[1].connected = True
    connections = connect(three_way_valve, temperature_sensor)
    connections += connect(pump, three_way_valve)
    connections += connect(three_way_valve, split_valve)

    assert len(connections) == 1
    assert {c.equation_view() for c in connections} == {('data_bus.port_a[1]', 'space_001.ports[1]')}