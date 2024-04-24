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
        FloorOnGround,
        InternalElement,
        MergedExternalWall,
        MergedWindows,
        Port,
        Space,
    )

    return {
        Space.__name__: lambda: [
            Port(
                target=BaseWall,
                names=["propsBus"],
                multi_connection=True,
                bus_connection=True,
            ),
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
