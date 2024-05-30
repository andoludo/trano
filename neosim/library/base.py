import abc
from typing import Any, Callable, Dict, List, Optional

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, ConfigDict, Field

from neosim.library.dynamic_components import (
    dynamic_ahu_controller_template,
    dynamic_ahu_template,
    dynamic_boiler_control_template,
    dynamic_boiler_template,
    dynamic_collector_control_template,
    dynamic_data_server_template,
    dynamic_emission_control_template,
    dynamic_pump_template,
    dynamic_three_way_valve_control_template,
    dynamic_vav_box_template,
    dynamic_vav_control_template,
)
from neosim.models.constants import Flow
from neosim.models.elements.base import (
    BaseElement,
    BaseVariant,
    DynamicComponentTemplate,
    Port,
)
from neosim.models.elements.boundary import Boundary
from neosim.models.elements.control import AhuControl, Control, DataBus, SpaceControl
from neosim.models.elements.space import Space
from neosim.models.elements.system import (
    AirHandlingUnit,
    DamperVariant,
    Emission,
    EmissionVariant,
    Occupancy,
    System,
    TemperatureSensor,
    ThreeWayValve,
    Ventilation,
    Weather,
)
from neosim.models.elements.wall import InternalElement


class LibraryData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    template: str = ""
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    component_template: Optional[DynamicComponentTemplate] = None
    ports_factory: Callable[[], List[Port]]
    variant: str = BaseVariant.default


class BaseSpace(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[InternalElement], names=["surf_surBou"], multi_connection=True
            ),
            Port(targets=[Occupancy], names=["qGai_flow"]),
            Port(targets=[Weather], names=["weaBus"]),
            Port(targets=[Emission], names=["heaPorAir", "heaPorRad"]),
            Port(targets=[DataBus], names=["heaPorAir"]),
            Port(targets=[SpaceControl], names=["heaPorAir"]),
            Port(
                targets=[Ventilation, Control, DataBus],
                names=["ports"],
                multi_connection=True,
                flow=Flow.inlet_or_outlet,
            ),
        ]
    )


class BaseEmission(LibraryData):
    variant: str = EmissionVariant.radiator
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["heatPortCon", "heatPortRad"]),
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class BaseIdealHeatingEmission(LibraryData):
    variant: str = EmissionVariant.ideal
    template: str = """
    {{package_name}}.Common.HeatTransfer.IdealHeatingSystem.IdealHeatEmission
    {{ element.name }}"""
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
    {{package_name}}.Common.Fluid.Boilers.BoilerWithStorage{{ element.name | capitalize}} {{ element.name }}(
    redeclare package MediumW = MediumW) "Boiler" """
    component_template: Optional[DynamicComponentTemplate] = dynamic_boiler_template
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
            Port(
                targets=[Control, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BasePump(LibraryData):
    template: str = """  {{ package_name }}.Common.Fluid.Ventilation.Pump{{ element.name | capitalize }}
     {{ element.name }}(
    redeclare package Medium = MediumW
    )"""
    component_template: DynamicComponentTemplate = dynamic_pump_template
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
            Port(
                targets=[Control, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
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
                # multi_connection=True,
                # use_counter=False,
            ),
            Port(names=["port_3"], flow=Flow.inlet_or_outlet),
            Port(targets=[Control], names=["y"]),
        ]
    )


class BaseOccupancy(LibraryData):
    template: str = """
    {{package_name}}.Common.Occupancy.SimpleOccupancy {{ element.name }}"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["y"]),
        ]
    )


class BaseWeather(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space, Boundary],
                names=["weaBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseInternalElement(LibraryData):
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["port_a"], flow=Flow.interchangeable_port),
            Port(targets=[Space], names=["port_b"], flow=Flow.interchangeable_port),
        ]
    )


class BaseSpaceControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.VAVControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_vav_control_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[System, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseAhuControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.AhuControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_ahu_controller_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[System, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseEmissionControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.EmissionControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_emission_control_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[System],
                names=["y"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseThreeWayValveControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.ThreeWayValveControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_three_way_valve_control_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[ThreeWayValve],
                names=["y"],
            ),
            Port(
                targets=[TemperatureSensor],
                names=["u"],
            ),
        ]
    )


class BaseCollectorControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.CollectorControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_collector_control_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[DataBus, System],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseBoilerControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: str = dynamic_boiler_control_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[DataBus, System],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseControl(LibraryData):
    template: str = """
    Modelica.Blocks.Sources.Constant {{ element.name }}(k= 1)
    """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[System], names=["y"]),
        ]
    )


class BaseDamper(LibraryData):
    template: str = """  {{ library_name }}.Fluid.Actuators.Dampers.PressureIndependent
    {{ element.name }}(
    redeclare package Medium = Medium,
    m_flow_nominal=100*1.2/3600,
    dpDamper_nominal=50,
    allowFlowReversal=false,
    dpFixed_nominal=50) "VAV box for room" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
            Port(targets=[Control], names=["y"]),
        ]
    )


class BaseTemperatureSensor(LibraryData):
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
            Port(targets=[Control], names=["T"]),
        ]
    )


class BaseDamperDetailed(LibraryData):
    variant: str = DamperVariant.complex
    template: str = """  {{ package_name }}.Common.Fluid.Ventilation.VAVBox{{ element.name | capitalize }}
     {{ element.name }}(
    redeclare package MediumA = Medium,
    mCooAir_flow_nominal=100*1.2/3600,
    mHeaAir_flow_nominal=100*1.2/3600,
    VRoo=100,
    allowFlowReversal=false,
    THeaWatInl_nominal=90,
    THeaWatOut_nominal=60,
    THeaAirInl_nominal=30,
    THeaAirDis_nominal=25
    )"""
    component_template: DynamicComponentTemplate = dynamic_vav_box_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_aAir"], flow=Flow.inlet),
            Port(names=["port_bAir"], flow=Flow.outlet),
            Port(
                targets=[Control, DataBus],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseAirHandlingUnit(LibraryData):
    template: str = """{{package_name}}.Common.Fluid.Ventilation.Ahu{{ element.name | capitalize}}
    {{ element.name }}
    (redeclare package MediumA = Medium,
    {% raw %}
    VRoo={100,100},
    AFlo={20,20},
    mCooVAV_flow_nominal={0.01,0.01}{% endraw %})"""
    component_template: DynamicComponentTemplate = dynamic_ahu_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[System, Space],
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[System, Space],
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[Boundary],
                names=["ports"],
            ),
            Port(
                targets=[AhuControl],
                names=["dataBus"],
            ),
        ]
    )


class BaseDuct(LibraryData):
    template: str = """  {{ library_name }}.Fluid.FixedResistances.PressureDrop
    {{ element.name }}(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(names=["port_a"], flow=Flow.inlet),
            Port(names=["port_b"], flow=Flow.outlet),
        ]
    )


class BaseVentilationControl(LibraryData):
    template: str = """    {{package_name}}.Common.Controls.SpaceControls.PIDSubstance
    {{ element.name }}(redeclare package
      Medium = Medium, yMax = 0.9, yMin = 0.1, setPoint = 400)"""

    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["port_a"], flow=Flow.inlet),
            Port(
                targets=[System], names=["y"], multi_connection=True, use_counter=False
            ),
        ]
    )


class BaseDataBus(LibraryData):
    template: str = """    {{package_name}}.Common.Controls.ventilation.DataServer
    {{ element.name }} (redeclare package
      Medium = Medium)"""
    component_template: DynamicComponentTemplate = dynamic_data_server_template
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space], names=["port"], multi_connection=True, use_counter=True
            ),
            Port(
                targets=[Space],
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=True,
            ),
            Port(
                targets=[System, Control],
                names=["dataBus"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class BaseBoundary(LibraryData):
    template: str = """  Buildings.Fluid.Sources.Outside {{ element.name }}
    (nPorts=2,redeclare package Medium = Medium)"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Weather], names=["weaBus"]),
            Port(
                targets=[AirHandlingUnit],
                names=["ports"],
                multi_connection=True,
                use_counter=False,
            ),
        ]
    )


class MaterialProperties(BaseModel):
    data: str
    is_package: bool


class DefaultLibrary(BaseModel):
    library_name: str
    constants: str
    merged_external_boundaries: bool = False
    functions: Dict[str, Callable[[Any], Any]] = Field(default={})
    control: List[LibraryData] = Field(default=[BaseControl()])
    spacecontrol: List[LibraryData] = Field(default=[BaseSpaceControl()])
    internalelement: List[LibraryData] = Field(default=[BaseInternalElement()])
    weather: List[LibraryData] = Field(default=[BaseWeather()])
    occupancy: List[LibraryData] = Field(default=[BaseOccupancy()])
    threewayvalve: List[LibraryData] = Field(default=[BaseThreeWayValve()])
    splitvalve: List[LibraryData] = Field(default=[BaseSplitValve()])
    pump: List[LibraryData] = Field(default=[BasePump()])
    boiler: List[LibraryData] = Field(default=[BaseBoiler()])
    valve: List[LibraryData] = Field(default=[BaseValve()])
    emission: List[LibraryData] = Field(
        default=[BaseEmission(), BaseIdealHeatingEmission()]
    )
    space: List[LibraryData] = Field(default=[BaseSpace()])
    externalwall: List[LibraryData] = Field(default=[LibraryData(ports_factory=list)])
    flooronground: List[LibraryData] = Field(default=[LibraryData(ports_factory=list)])
    window: List[LibraryData] = Field(default=[LibraryData(ports_factory=list)])
    mergedexternalwall: List[LibraryData] = Field(
        default=[LibraryData(ports_factory=list)]
    )
    mergedwindows: List[LibraryData] = Field(default=[LibraryData(ports_factory=list)])
    damper: List[LibraryData] = Field(default=[BaseDamper(), BaseDamperDetailed()])
    airhandlingunit: List[LibraryData] = Field(default=[BaseAirHandlingUnit()])
    duct: List[LibraryData] = Field(default=[BaseDuct()])
    spacesubstanceventilationcontrol: List[LibraryData] = Field(
        default=[BaseVentilationControl()]
    )
    databus: List[LibraryData] = Field(default=[BaseDataBus()])
    boundary: List[LibraryData] = Field(default=[BaseBoundary()])
    ahucontrol: List[LibraryData] = Field(default=[BaseAhuControl()])
    emissioncontrol: List[LibraryData] = Field(default=[BaseEmissionControl()])
    collectorcontrol: List[LibraryData] = Field(default=[BaseCollectorControl()])
    temperaturesensor: List[LibraryData] = Field(default=[BaseTemperatureSensor()])
    threewayvalvecontrol: List[LibraryData] = Field(
        default=[BaseThreeWayValveControl()]
    )
    boilercontrol: List[LibraryData] = Field(default=[BaseBoilerControl()])

    def _get_field_value(self, element: BaseElement) -> Any:  # noqa: ANN401
        element_names = [
            type(element).__name__.lower(),
            type(element).__base__.__name__.lower(),  # type: ignore
            type(element).__base__.__base__.__name__.lower(),  # type: ignore
        ]
        for element_name in element_names:
            if hasattr(self, element_name):
                libraries_data = [
                    ld
                    for ld in getattr(self, element_name)
                    if ld.variant == element.variant
                ]
                if libraries_data:
                    return libraries_data[0]
        raise ValueError(
            f"Element '{element_names[0]}' with variant '{element.variant}' not "
            f"found in library '{self.library_name}'."
        )

    def assign_ports(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.ports:
            return element.ports
        return self._get_field_value(element).ports_factory()

    def assign_annotation_template(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.annotation_template:
            return element.annotation_template
        return self._get_field_value(element).annotation_template

    def assign_component_template(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.component_template:
            return element.component_template
        return self._get_field_value(element).component_template

    def assign_template(self, element: BaseElement) -> Any:  # noqa : ANN401
        if element.template:
            return element.template
        return self._get_field_value(element).template

    def assign_properties(self, element: BaseElement) -> BaseElement:
        element.ports = self.assign_ports(element)
        element.template = self.assign_template(element)
        element.annotation_template = self.assign_annotation_template(element)
        element.component_template = self.assign_component_template(element)
        return element

    @abc.abstractmethod
    def extract_data(
        self, package_name: str, nodes: NodeView
    ) -> MaterialProperties:  # : ANN401
        ...
