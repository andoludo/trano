from typing import Callable, List

from pydantic import Field

from trano.controller.parser import ControllerBus, RealInput, RealOutput
from trano.models.elements.base import (
    BaseParameter,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from trano.models.elements.bus import DataBus
from trano.models.elements.controls.base import Control
from trano.models.elements.system import System


class BoilerParameters(BaseParameter):
    tsup_nominal: float = Field(
        80 + 273.15,
        alias="TSup_nominal",
        title="Check for temperature at the bottom of the tank",
    )
    threshold_outdoor_air_cutoff: float = Field(
        15 + 273.15,
        alias="threshold_outdoor_air_cutoff",
        title="Output true if outdoor air is below heating cut-off limit",
    )
    threshold_to_switch_off_boiler: float = Field(
        15 + 273.15,
        alias="threshold_to_switch_off_boiler",
        title="Threshold to switch boiler off",
    )


dynamic_boiler_control_template = DynamicComponentTemplate(
    template="""
    model BoilerControl{{ element.name | capitalize}}
    extends {{ package_name }}.Common.Controls.ventilation.PartialBoilerControl;
    {{bus_template}}
    equation
    {{bus_ports | safe}}
     end BoilerControl{{ element.name | capitalize}};
     """,
    category="control",
    bus=ControllerBus(
        real_outputs=[
            RealOutput(
                name="yBoiCon",
                target="element.controllable_element.name",
                component="booToReaBoi",
                port="y",
            ),
            RealOutput(
                name="yPumBoi",
                target="element.controllable_element.name",
                component="booToReaPum",
                port="y",
            ),
        ],
        real_inputs=[
            RealInput(
                name="TStoTop",
                target="element.controllable_element.name",
                component="sub1",
                port="u1",
            ),
            RealInput(
                name="TStoBot",
                target="element.controllable_element.name",
                component="greThr",
                port="u",
            ),
            RealInput(
                name="TAirOut",
                target="element.controllable_element.name",
                component="lesThrTOut",
                port="u",
            ),
        ],
    ),
)


class BaseBoilerControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})"""
    component_template: DynamicComponentTemplate = dynamic_boiler_control_template
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


class SimpleBoilerControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})"""
    component_template: DynamicComponentTemplate = dynamic_boiler_control_template
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


class BoilerControl(Control):
    ...
