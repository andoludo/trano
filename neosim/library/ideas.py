from typing import Callable, List

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, Field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.library.base import BaseSpace, DefaultLibrary, LibraryData
from neosim.material import Material
from neosim.models.constants import Flow
from neosim.models.elements.base import Port
from neosim.models.elements.control import Control, SpaceControl
from neosim.models.elements.space import Space
from neosim.models.elements.system import Emission, Occupancy
from neosim.models.elements.wall import BaseWall


class IdeasSpace(BaseSpace):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=BaseWall,
                names=["propsBus"],
                multi_connection=True,
                bus_connection=True,
            ),
            Port(target=Occupancy, names=["yOcc"]),
            Port(target=Emission, names=["gainCon", "gainRad"]),
            Port(target=SpaceControl, names=["gainCon"]),
        ]
    )


class IdeasMergedExternalWall(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=Space,
                names=["propsBus_a"],
                multi_connection=True,
                multi_object=True,
                bus_connection=True,
            ),
        ]
    )


class IdeasMergedWindows(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=Space,
                names=["propsBus_a"],
                multi_connection=True,
                multi_object=True,
                bus_connection=True,
            ),
        ]
    )


class IdeasFloorOnGround(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["propsBus_a"], multi_connection=False),
        ]
    )


class IdeasInternalElement(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["propsBus_a"], multi_connection=False),
            Port(target=Space, names=["propsBus_b"], multi_connection=False),
        ]
    )


class IdeasPump(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
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
        ]
    )


class IdeasData(BaseModel):
    constructions: List[Construction]
    materials: List[Material]
    glazing: List[Glass]


class IdeasLibrary(DefaultLibrary):
    template: str = "ideas.jinja2"
    merged_external_boundaries: bool = True
    space: LibraryData = Field(default=IdeasSpace())
    externalwall: LibraryData = Field(default=IdeasMergedExternalWall())
    flooronground: LibraryData = Field(default=IdeasFloorOnGround())
    mergedexternalwall: LibraryData = Field(default=IdeasMergedExternalWall())
    mergedwindows: LibraryData = Field(default=IdeasMergedWindows())
    internalelement: LibraryData = Field(default=IdeasInternalElement())
    pump: LibraryData = Field(default=IdeasPump())

    def extract_data(self, nodes: NodeView) -> IdeasData:  # noqa: PLR6301
        from neosim.models.elements.merged_wall import MergedBaseWall
        from neosim.models.elements.wall import BaseSimpleWall

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
