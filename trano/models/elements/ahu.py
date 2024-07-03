from typing import Callable, List

from pydantic import Field

from trano.controller.parser import ControllerBus, IntegerOutput, RealInput, RealOutput
from trano.models.constants import Flow
from trano.models.elements.base import (
    AvailableLibraries,
    BaseBoundary,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from trano.models.elements.controls.ahu import AhuControl
from trano.models.elements.space import Space
from trano.models.elements.system import System, Ventilation

dynamic_ahu_template = DynamicComponentTemplate(
    template="""
    model Ahu{{ element.name | capitalize}}
    extends {{ package_name }}.Common.Fluid.Ventilation.PartialAhu;
    {{bus_template}}
    equation
    {{bus_ports | safe}}
     end Ahu{{ element.name | capitalize}};
     """,
    category="ventilation",
    bus=ControllerBus(
        integer_outputs=[
            IntegerOutput(
                name="u1SupFan",
                target="element.name",
                component="fanSup",
                port="y_actual",
            )
        ],
        real_inputs=[
            RealInput(
                name="ySupFan", target="element.name", component="fanSup1", port="y"
            ),
            RealInput(
                name="ySupFan", target="element.name", component="fanSup", port="y"
            ),
            RealInput(
                name="yRetDam", target="element.name", component="damRet", port="y"
            ),
            RealInput(
                name="yOutDam", target="element.name", component="damOut", port="y"
            ),
            RealInput(
                name="yOutDam", target="element.name", component="damExh", port="y"
            ),
        ],
        real_outputs=[
            RealOutput(
                name="TOut", target="element.control.name", component="TOut", port="T"
            ),
            RealOutput(
                name="VAirOut_flow",
                target="element.control.name",
                component="VOut1",
                port="V_flow",
            ),
            RealOutput(
                name="TAirSup",
                target="element.control.name",
                component="TSup",
                port="T",
            ),
            RealOutput(
                name="TAirMix",
                target="element.control.name",
                component="TMix",
                port="T",
            ),
            RealOutput(
                name="dpDuc",
                target="element.control.name",
                component="dpDisSupFan",
                port="p_rel",
            ),
        ],
    ),
)


class BaseAirHandlingUnit(LibraryData):
    template: str = """{{package_name}}.Common.Fluid.
    Ventilation.Ahu{{ element.name | capitalize}}
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
                targets=[BaseBoundary],
                names=["ports"],
            ),
            Port(
                targets=[AhuControl],
                names=["dataBus"],
            ),
        ]
    )


class AirHandlingUnit(Ventilation):
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseAirHandlingUnit],
        buildings=[BaseAirHandlingUnit],
    )
