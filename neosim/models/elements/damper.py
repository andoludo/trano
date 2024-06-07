from typing import Callable, List

from pydantic import Field

from neosim.controller.parser import ControllerBus, RealInput, RealOutput
from neosim.models.constants import Flow
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseVariant,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from neosim.models.elements.bus import DataBus
from neosim.models.elements.controls.base import Control
from neosim.models.elements.system import Ventilation

dynamic_vav_box_template = DynamicComponentTemplate(
    template="""
model VAVBox{{ element.name | capitalize}}
extends {{ package_name }}.Common.Fluid.Ventilation.PartialVAVBox;
{{bus_template}}
equation
{{bus_ports | safe}}
 end VAVBox{{ element.name | capitalize}};
 """,
    category="ventilation",
    bus=ControllerBus(
        real_inputs=[
            RealInput(name="yDam", target="element.name", component="vav", port="y")
        ],
        real_outputs=[
            RealOutput(
                name="y_actual",
                target="element.name",
                component="vav",
                port="y_actual",
            ),
            RealOutput(
                name="VDis_flow",
                target="element.control.name",
                component="senVolFlo",
                port="V_flow",
            ),
            RealOutput(
                name="TDis", target="element.control.name", component="senTem", port="T"
            ),
        ],
    ),
)


class DamperVariant(BaseVariant):
    complex: str = "complex"


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


class Damper(Ventilation):
    ...


class VAV(Damper):
    variant: str = DamperVariant.default
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseDamper, BaseDamperDetailed],
        buildings=[BaseDamper, BaseDamperDetailed],
    )
