from typing import Any, Callable, List

from pydantic import BaseModel, Field

from neosim.models.constants import Flow
from neosim.models.elements.base import BaseElement, Port
from neosim.models.elements.control import Control, SpaceControl
from neosim.models.elements.space import Space
from neosim.models.elements.system import Emission, Occupancy, System, Weather
from neosim.models.elements.wall import InternalElement


class LibraryData(BaseModel):
    ports_factory: Callable[[], List[Port]]


class BuildingsSpace(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=InternalElement, names=["surf_surBou"], multi_connection=True),
            Port(target=Occupancy, names=["qGai_flow"]),
            Port(target=Weather, names=["weaBus"]),
            Port(target=Emission, names=["heaPorAir", "heaPorRad"]),
            Port(target=SpaceControl, names=["heaPorAir"]),
        ]
    )


class BuildingsEmission(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["heatPortCon", "heatPortRad"]),
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class BuildingsValve(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(target=Control, names=["y"]),
        ]
    )


class BuildingsBoiler(LibraryData):
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


class BuildingsPump(LibraryData):
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


class BuildingsSplitValve(LibraryData):
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


class BuildingsThreeWayValve(LibraryData):
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


class BuildingsOccupancy(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["y"]),
        ]
    )


class BuildingsWeather(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=Space, names=["weaBus"], multi_connection=True, use_counter=False
            ),
        ]
    )


class BuildingsInternalElement(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["port_a"]),
            Port(target=Space, names=["port_b"]),
        ]
    )


class BuildingsSpaceControl(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["port"]),
            Port(target=System, names=["y"]),
        ]
    )


class BuildingsControl(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=System, names=["y"]),
        ]
    )


class DefaultLibrary(BaseModel):
    control: LibraryData = Field(default=BuildingsControl())
    spacecontrol: LibraryData = Field(default=BuildingsSpaceControl())
    internalelement: LibraryData = Field(default=BuildingsInternalElement())
    weather: LibraryData = Field(default=BuildingsWeather())
    occupancy: LibraryData = Field(default=BuildingsOccupancy())
    threewayvalve: LibraryData = Field(default=BuildingsThreeWayValve())
    splitvalve: LibraryData = Field(default=BuildingsSplitValve())
    pump: LibraryData = Field(default=BuildingsPump())
    boiler: LibraryData = Field(default=BuildingsBoiler())
    valve: LibraryData = Field(default=BuildingsValve())
    emission: LibraryData = Field(default=BuildingsEmission())
    space: LibraryData = Field(default=BuildingsSpace())
    externalwall: LibraryData = Field(default=LibraryData(ports_factory=list))
    flooronground: LibraryData = Field(default=LibraryData(ports_factory=list))
    window: LibraryData = Field(default=LibraryData(ports_factory=list))

    def assign_ports(self, element: BaseElement) -> Any:  # noqa : ANN401
        return getattr(self, type(element).__name__.lower()).ports_factory()


class BuildingsLibrary(DefaultLibrary):
    ...
