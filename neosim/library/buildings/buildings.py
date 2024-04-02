from typing import TYPE_CHECKING, Dict, List

if TYPE_CHECKING:
    from neosim.model import Port


def buildings_ports() -> Dict[str, List["Port"]]:
    from neosim.model import (
        Boiler,
        Control,
        Emission,
        Flow,
        InternalElement,
        Occupancy,
        Port,
        Pump,
        Space,
        SpaceControl,
        SplitValve,
        System,
        ThreeWayValve,
        Valve,
        Weather,
    )

    return {
        Space.__name__: [
            Port(target=InternalElement, names=["surf_surBou"], multi_connection=True),
            Port(target=Occupancy, names=["qGai_flow"]),
            Port(target=Weather, names=["weaBus"]),
            Port(target=Emission, names=["heaPorAir", "heaPorRad"]),
            Port(target=SpaceControl, names=["heaPorAir"]),
        ],
        Emission.__name__: [
            Port(target=Space, names=["heatPortCon", "heatPortRad"]),
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ],
        Valve.__name__: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(target=Control, names=["y"]),
        ],
        Boiler.__name__: [
            Port(
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
        ],
        Pump.__name__: [
            Port(
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(target=Control, names=["y"]),
        ],
        SplitValve.__name__: [
            Port(
                names=["port_1"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(names=["port_2"], flow=Flow.outlet),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
        ],
        ThreeWayValve.__name__: [
            Port(names=["port_1"], flow=Flow.inlet),
            Port(
                names=["port_2"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
            Port(target=Control, names=["y"]),
        ],
        Occupancy.__name__: [
            Port(target=Space, names=["y"]),
        ],
        Weather.__name__: [
            Port(
                target=Space, names=["weaBus"], multi_connection=True, use_counter=False
            ),
        ],
        InternalElement.__name__: [
            Port(target=Space, names=["port_a"]),
            Port(target=Space, names=["port_b"]),
        ],
        SpaceControl.__name__: [
            Port(target=Space, names=["port"]),
            Port(target=System, names=["y"]),
        ],
        Control.__name__: [
            Port(target=System, names=["y"]),
        ],
    }
