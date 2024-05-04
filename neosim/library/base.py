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
    Ventilation,
    Weather,
)
from neosim.models.elements.wall import InternalElement


class LibraryData(BaseModel):
    template: str = ""
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]]


class BaseSpace(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[InternalElement], names=["surf_surBou"], multi_connection=True
            ),
            Port(targets=[Occupancy], names=["qGai_flow"]),
            Port(targets=[Weather], names=["weaBus"]),
            Port(targets=[Emission], names=["heaPorAir", "heaPorRad"]),
            Port(targets=[IdealHeatingEmission], names=["heaPorAir", "heaPorRad"]),
            Port(targets=[SpaceControl], names=["heaPorAir"]),
            Port(
                targets=[Ventilation, Control],
                names=["ports"],
                multi_connection=True,
                flow=Flow.inlet_or_outlet,
            ),
        ]
    )


class BaseEmission(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["heatPortCon", "heatPortRad"]),
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
            Port(targets=[Space], names=["heatPortCon", "heatPortRad"]),
            Port(targets=[Control], names=["y"]),
        ]
    )


class BaseValve(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(targets=[Control], names=["y"]),
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
            Port(targets=[Control], names=["y"]),
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
            Port(targets=[Control], names=["y"]),
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
            Port(targets=[Space], names=["y"]),
        ]
    )


class BaseWeather(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space],
                names=["weaBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseInternalElement(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["port_a"]),
            Port(targets=[Space], names=["port_b"]),
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
            Port(targets=[Space], names=["port"]),
            Port(targets=[System], names=["y"]),
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
            Port(targets=[System], names=["y"]),
        ]
    )


class BaseDamper(LibraryData):
    template: str = """  Buildings.Fluid.Actuators.Dampers.Exponential {{ element.name }}(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(targets=[Control], names=["y"]),
        ]
    )


class BaseAirHandlingUnit(LibraryData):
    template: str = """Neosim.Fluid.Ventilation.SimpleHVACBuildings {{ element.name }}
    (redeclare package Medium = Medium)
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


class BaseDuct(LibraryData):
    template: str = """  Buildings.Fluid.FixedResistances.PressureDrop {{ element.name }}(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class BaseVentilationControl(LibraryData):
    template: str = """    Neosim.Controls.SpaceControls.PIDSubstance
    {{ element.name }}(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""

    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["port_a"], flow=Flow.inlet),
            Port(
                targets=[System], names=["y"], multi_connection=True, use_counter=False
            ),
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
    damper: LibraryData = Field(default=BaseDamper())
    airhandlingunit: LibraryData = Field(default=BaseAirHandlingUnit())
    duct: LibraryData = Field(default=BaseDuct())
    spacesubstanceventilationcontrol: LibraryData = Field(
        default=BaseVentilationControl()
    )

    def _get_field_value(self, element: BaseElement) -> Any:  # noqa: ANN401
        element_names = [
            type(element).__name__.lower(),
            type(element).__base__.__name__.lower(),  # type: ignore
            type(element).__base__.__base__.__name__.lower(),  # type: ignore
        ]
        for element_name in element_names:
            if hasattr(self, element_name):
                return getattr(self, element_name)
        raise ValueError(f"Element {element_names[0]} not found in library")

    def assign_ports(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.ports:
            return element.ports
        return self._get_field_value(element).ports_factory()

    def assign_template(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.template:
            return element.template
        return self._get_field_value(element).template

    def assign_properties(self, element: BaseElement) -> BaseElement:
        element.ports = self.assign_ports(element)
        element.template = self.assign_template(element)
        return element

    def extract_data(self, package_name: str, nodes: NodeView) -> Any:  # noqa : ANN401
        ...
