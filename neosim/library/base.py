from typing import Any, Callable, List

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, Field

from neosim.models.constants import Flow
from neosim.models.elements.base import BaseElement, Port
from neosim.models.elements.control import Control, SpaceControl
from neosim.models.elements.space import Space
from neosim.models.elements.system import Emission, Occupancy, System, Weather
from neosim.models.elements.wall import InternalElement


class LibraryData(BaseModel):
    ports_factory: Callable[[], List[Port]]


class BaseSpace(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=InternalElement, names=["surf_surBou"], multi_connection=True),
            Port(target=Occupancy, names=["qGai_flow"]),
            Port(target=Weather, names=["weaBus"]),
            Port(target=Emission, names=["heaPorAir", "heaPorRad"]),
            Port(target=SpaceControl, names=["heaPorAir"]),
        ]
    )


class BaseEmission(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["heatPortCon", "heatPortRad"]),
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class BaseValve(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(target=Control, names=["y"]),
        ]
    )


class BaseBoiler(LibraryData):
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
        ]
    )


class BasePump(LibraryData):
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
            Port(target=Control, names=["y"]),
        ]
    )


class BaseSplitValve(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                names=["port_1"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(names=["port_2"], flow=Flow.outlet),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
        ]
    )


class BaseThreeWayValve(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_1"], flow=Flow.inlet),
            Port(
                names=["port_2"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
            Port(target=Control, names=["y"]),
        ]
    )


class BaseOccupancy(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["y"]),
        ]
    )


class BaseWeather(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=Space, names=["weaBus"], multi_connection=True, use_counter=False
            ),
        ]
    )


class BaseInternalElement(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["port_a"]),
            Port(target=Space, names=["port_b"]),
        ]
    )


class BaseSpaceControl(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["port"]),
            Port(target=System, names=["y"]),
        ]
    )


class BaseControl(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=System, names=["y"]),
        ]
    )


class DefaultLibrary(BaseModel):
    template: str
    merged_external_boundaries: bool = False
    control: LibraryData = Field(default=BaseControl())
    spacecontrol: LibraryData = Field(default=BaseSpaceControl())
    internalelement: LibraryData = Field(default=BaseInternalElement())
    weather: LibraryData = Field(default=BaseWeather())
    occupancy: LibraryData = Field(default=BaseOccupancy())
    threewayvalve: LibraryData = Field(default=BaseThreeWayValve())
    splitvalve: LibraryData = Field(default=BaseSplitValve())
    pump: LibraryData = Field(default=BasePump())
    boiler: LibraryData = Field(default=BaseBoiler())
    valve: LibraryData = Field(default=BaseValve())
    emission: LibraryData = Field(default=BaseEmission())
    space: LibraryData = Field(default=BaseSpace())
    externalwall: LibraryData = Field(default=LibraryData(ports_factory=list))
    flooronground: LibraryData = Field(default=LibraryData(ports_factory=list))
    window: LibraryData = Field(default=LibraryData(ports_factory=list))
    mergedexternalwall: LibraryData = Field(default=LibraryData(ports_factory=list))
    mergedwindows: LibraryData = Field(default=LibraryData(ports_factory=list))

    def assign_ports(self, element: BaseElement) -> Any:  # noqa : ANN401
        return getattr(self, type(element).__name__.lower()).ports_factory()

    def extract_data(self, nodes: NodeView) -> Any:  # noqa : ANN401
        ...
