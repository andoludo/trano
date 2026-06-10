"""Builders for the network shapes shared by several fixtures.

The connection order in these builders replicates the original hand-written
fixtures: the generated Modelica models must keep matching the golden files in
tests/data, and the order of `connect_systems` calls affects port numbering.
"""

from dataclasses import dataclass

from tests.constructions.constructions import Constructions
from trano.elements import ExternalWall, InternalElement, param_from_config
from trano.elements.boundary import Boundary
from trano.elements.control import (
    AhuControl,
    BoilerControl,
    CollectorControl,
    ThreeWayValveControl,
)
from trano.elements.library.library import Library
from trano.elements.space import Space
from trano.elements.system import (
    AirHandlingUnit,
    Boiler,
    Occupancy,
    Pump,
    SplitValve,
    TemperatureSensor,
    ThreeWayValve,
    Weather,
)
from trano.elements.types import Azimuth, Tilt
from trano.topology import Network

SpaceParameter = param_from_config("Space")


@dataclass
class _HydronicLoop:
    spaces: list[Space]
    control: ThreeWayValveControl
    valve: ThreeWayValve
    split_valve: SplitValve
    sensor: TemperatureSensor


def _suffixed(base: str, index: int) -> str:
    return base if index == 0 else f"{base}_{index + 1}"


def hydronic_network(
    name: str,
    zone_groups: list[list[Space]],
    library: Library | None = None,
    boiler_control_name: str = "boiler_control",
    control_to_sensor: bool = False,
) -> Network:
    """Network with a boiler and pump feeding one three-way-valve loop per zone group."""
    network = Network(name=name, library=library)
    network.add_boiler_plate_spaces([space for group in zone_groups for space in group])

    pump = Pump(name="pump", control=CollectorControl(name="pump_control"))
    boiler = Boiler(name="boiler", control=BoilerControl(name=boiler_control_name))
    loops = []
    for index, group in enumerate(zone_groups):
        control = ThreeWayValveControl(name=_suffixed("three_way_valve_control", index))
        loop = _HydronicLoop(
            spaces=group,
            control=control,
            valve=ThreeWayValve(name=_suffixed("three_way_valve", index), control=control),
            split_valve=SplitValve(name=_suffixed("split_valve", index)),
            sensor=TemperatureSensor(name=_suffixed("temperature_sensor", index)),
        )
        loops.append(loop)
        for space in loop.spaces:
            network.connect_systems(loop.sensor, space.first_emission())
        network.connect_systems(loop.valve, loop.sensor)
    for loop in loops:
        for space in loop.spaces:
            network.connect_systems(space.last_emission(), loop.split_valve)
    network.connect_systems(boiler, pump)
    for loop in loops:
        network.connect_systems(pump, loop.valve)
    for loop in loops:
        network.connect_systems(loop.valve, loop.split_valve)
    for loop in loops:
        network.connect_systems(loop.split_valve, boiler)
    for loop in loops:
        # The direction of this connection affects the generated model (see golden files).
        if control_to_sensor:
            network.connect_systems(loop.control, loop.sensor)
        else:
            network.connect_systems(loop.sensor, loop.control)
    return network


def ventilated_network(  # noqa: PLR0913
    name: str,
    spaces: list[Space],
    library: Library | None = None,
    *,
    ahu_control: bool = False,
    with_boundary: bool = False,
    inlets_first: bool = False,
) -> Network:
    """Network with an air handling unit serving every space."""
    network = Network(name=name, library=library)
    network.add_boiler_plate_spaces(spaces)
    if ahu_control:
        ahu = AirHandlingUnit(name="ahu", control=AhuControl(name="ahu_control"))
    else:
        ahu = AirHandlingUnit(name="ahu")
    if inlets_first:
        for space in spaces:
            network.connect_systems(ahu, space.get_last_ventilation_inlet())
        for space in spaces:
            network.connect_systems(space.get_last_ventilation_outlet(), ahu)
    else:
        for space in spaces:
            network.connect_systems(ahu, space.get_last_ventilation_inlet())
            network.connect_systems(space.get_last_ventilation_outlet(), ahu)
    if with_boundary:
        boundary = Boundary(name="boundary")
        network.connect_elements(boundary, ahu)
        weather = next(node for node in network.graph.nodes if isinstance(node, Weather))
        network.connect_elements(boundary, weather)
    return network


def building_with_multiple_internal_walls(network_name: str, library: Library | None = None) -> Network:
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
