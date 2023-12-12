def buildings_ports() -> dict:
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
        SplitValve,
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
            Port(target=Control, names=["y"]),
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
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ],
        Pump.__name__: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(target=Control, names=["y"]),
        ],
        SplitValve.__name__: [
            Port(names=["port_1"], flow=Flow.inlet),
            Port(names=["port_2"], flow=Flow.outlet),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
        ],
        ThreeWayValve.__name__: [
            Port(names=["port_1"], flow=Flow.inlet),
            Port(names=["port_2"], flow=Flow.outlet),
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
    }
