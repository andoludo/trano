from typing import TYPE_CHECKING, Callable, Dict, List

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.material import Material

if TYPE_CHECKING:
    from neosim.model import Port


def ideas_ports() -> Dict[str, Callable[[], List["Port"]]]:
    from neosim.model import (
        BaseWall,
        Boiler,
        Control,
        Emission,
        FloorOnGround,
        Flow,
        InternalElement,
        MergedExternalWall,
        MergedWindows,
        Occupancy,
        Port,
        Pump,
        Space,
        SpaceControl,
        SplitValve,
        System,
        ThreeWayValve,
        Valve,
    )

    return {
        Space.__name__: lambda: [
            Port(
                target=BaseWall,
                names=["propsBus"],
                multi_connection=True,
                bus_connection=True,
            ),
            Port(target=Occupancy, names=["yOcc"]),
            Port(target=Emission, names=["gainCon", "gainRad"]),
            Port(target=SpaceControl, names=["gainCon"]),
        ],
        MergedExternalWall.__name__: lambda: [
            Port(
                target=Space,
                names=["propsBus_a"],
                multi_connection=True,
                multi_object=True,
                bus_connection=True,
            ),
        ],
        MergedWindows.__name__: lambda: [
            Port(
                target=Space,
                names=["propsBus_a"],
                multi_connection=True,
                multi_object=True,
                bus_connection=True,
            ),
        ],
        FloorOnGround.__name__: lambda: [
            Port(target=Space, names=["propsBus_a"], multi_connection=False),
        ],
        InternalElement.__name__: lambda: [
            Port(target=Space, names=["propsBus_a"], multi_connection=False),
            Port(target=Space, names=["propsBus_b"], multi_connection=False),
        ],
        Emission.__name__: lambda: [
            Port(target=Space, names=["heatPortCon", "heatPortRad"]),
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ],
        Valve.__name__: lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(target=Control, names=["y"]),
        ],
        Boiler.__name__: lambda: [
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
        Pump.__name__: lambda: [
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
            Port(target=Control, names=["m_flow_in"]),
        ],
        SplitValve.__name__: lambda: [
            Port(
                names=["port_1"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(names=["port_2"], flow=Flow.outlet),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
        ],
        ThreeWayValve.__name__: lambda: [
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
        Occupancy.__name__: lambda: [
            Port(target=Space, names=["y"]),
        ],
        SpaceControl.__name__: lambda: [
            Port(target=Space, names=["port"]),
            Port(target=System, names=["y"]),
        ],
        Control.__name__: lambda: [
            Port(target=System, names=["y"]),
        ],
    }


class IdeasData(BaseModel):
    constructions: List[Construction]
    materials: List[Material]
    glazing: List[Glass]


def extract_data(nodes: NodeView) -> IdeasData:
    from neosim.model import BaseSimpleWall, MergedBaseWall

    merged_constructions = {
        construction
        for node in [node_ for node_ in nodes if isinstance(node_, MergedBaseWall)]
        for construction in node.constructions
    }

    constructions = {
        node.construction
        for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
    }
    merged_constructions.update(constructions)
    wall_constructions = [
        c for c in merged_constructions if isinstance(c, Construction)
    ]
    glazing = [c for c in merged_constructions if isinstance(c, Glass)]
    materials = {
        layer.material
        for construction in merged_constructions
        for layer in construction.layers
    }
    return IdeasData(
        materials=list(materials), constructions=wall_constructions, glazing=glazing
    )
