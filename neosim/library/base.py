from typing import Any, Callable, Dict, List

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, Field

from neosim.models.constants import Flow
from neosim.models.elements.base import BaseElement, Port
from neosim.models.elements.control import Control, SpaceControl
from neosim.models.elements.space import Space
from neosim.models.elements.system import (
    Emission,
    IdealHeatingEmission,
    Occupancy,
    System,
    Weather,
)
from neosim.models.elements.wall import InternalElement


class LibraryData(BaseModel):
    template: str = ""
    ports_factory: Callable[[], List[Port]]


class BaseSpace(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=InternalElement, names=["surf_surBou"], multi_connection=True),
            Port(target=Occupancy, names=["qGai_flow"]),
            Port(target=Weather, names=["weaBus"]),
            Port(target=Emission, names=["heaPorAir", "heaPorRad"]),
            Port(target=IdealHeatingEmission, names=["heaPorAir", "heaPorRad"]),
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


class BaseIdealHeatingEmission(LibraryData):
    template: str = """
    Neosim.HeatTransfer.IdealHeatingSystem.IdealHeatEmission {{ element.name }}
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["heatPortCon", "heatPortRad"]),
            Port(target=Control, names=["y"]),
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
    template: str = """
    Neosim.Fluid.Boilers.Simple {{ element.name }}(
    redeclare package Medium = MediumW) "Boiler"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
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
    template: str = """
    Neosim.Occupancy.SimpleOccupancy {{ element.name }} annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
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
    template: str = """
    Neosim.Controls.SpaceControls.PID {{ element.name }}(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["port"]),
            Port(target=System, names=["y"]),
        ]
    )


class BaseControl(LibraryData):
    template: str = """
    Modelica.Blocks.Sources.Constant {{ element.name }}(k= 1)
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=System, names=["y"]),
        ]
    )


class DefaultLibrary(BaseModel):
    constants: str
    merged_external_boundaries: bool = False
    functions: Dict[str, Callable[[Any], Any]] = Field(default={})
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
    idealheatingemission: LibraryData = Field(default=BaseIdealHeatingEmission())
    space: LibraryData = Field(default=BaseSpace())
    externalwall: LibraryData = Field(default=LibraryData(ports_factory=list))
    flooronground: LibraryData = Field(default=LibraryData(ports_factory=list))
    window: LibraryData = Field(default=LibraryData(ports_factory=list))
    mergedexternalwall: LibraryData = Field(default=LibraryData(ports_factory=list))
    mergedwindows: LibraryData = Field(default=LibraryData(ports_factory=list))

    def assign_ports(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.ports:
            return element.ports
        return getattr(self, type(element).__name__.lower()).ports_factory()

    def assign_template(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.template:
            return element.template
        return getattr(self, type(element).__name__.lower()).template

    def assign_properties(self, element: BaseElement) -> BaseElement:
        element.ports = self.assign_ports(element)
        element.template = self.assign_template(element)
        return element

    def extract_data(self, package_name: str, nodes: NodeView) -> Any:  # noqa : ANN401
        ...
