from typing import Callable, List

from pydantic import Field

from neosim.controller.parser import ControllerBus, RealInput, RealOutput
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseParameter,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from neosim.models.elements.bus import DataBus
from neosim.models.elements.controls.base import Control
from neosim.models.elements.system import System


class ControlLoopsParameters(BaseParameter):
    k_coo_con: float = Field(
        0.1,
        alias="kCooCon",
        title="Gain of controller for cooling control loop",
    )
    ti_coo_con: float = Field(
        900,
        alias="TiCooCon",
        title="Time constant of integrator block for cooling control loop",
    )
    k_hea_con: float = Field(
        0.1,
        alias="kHeaCon",
        title="Gain of controller for heating control loop",
    )
    ti_hea_con: float = Field(
        900,
        alias="TiHeaCon",
        title="Time constant of integrator block for heating control loop",
    )
    tim_che: float = Field(
        30,
        alias="timChe",
        title="Threshold time to check the zone temperature status",
    )
    dt_hys: float = Field(
        0.25,
        alias="dTHys",
        title="Delta between the temperature hysteresis high and low limit",
    )
    loo_hys: float = Field(
        0.01,
        alias="looHys",
        title="Threshold value to check if the controller output is near zero",
    )


dynamic_emission_control_template = DynamicComponentTemplate(
    template="""model EmissionControl{{ element.name | capitalize}}
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops emissionControl({{ macros.render_parameters(parameters) | safe}})
{% raw %}annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); {% endraw %}
  Modelica.Blocks.Interfaces.RealOutput y
    {% raw %}annotation (Placement(transformation(extent={{100,-8},{120,12}})));{% endraw %}
{{bus_template}}
equation
  connect(emissionControl.yHea, y) {% raw %}annotation (Line(points={{34.4,-21.2},{96,-21.2},
          {96,2},{110,2}}, color={0,0,127}));{% endraw %}
{{bus_ports | safe}}
end EmissionControl{{ element.name  | capitalize}};""",
    category="control",
    bus=ControllerBus(
        real_outputs=[
            RealOutput(
                name="yCoo",
                target="element.controllable_element.name",
                component="emissionControl",
                port="yCoo",
            ),
            RealOutput(
                name="yHea",
                target="element.controllable_element.name",
                component="emissionControl",
                port="yHea",
            ),
        ],
        real_inputs=[
            RealInput(
                default=273.15 + 25,
                name="TCooSet",
                target="element.space_name",
                component="emissionControl",
                port="TCooSet",
            ),
            RealInput(
                default=273.15 + 21,
                name="THeaSet",
                target="element.space_name",
                component="emissionControl",
                port="THeaSet",
            ),
            RealInput(
                name="TZon",
                target="element.space_name",
                component="emissionControl",
                port="TZon",
            ),
        ],
    ),
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


class EmissionControl(Control):
    parameters: ControlLoopsParameters = Field(default=ControlLoopsParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseEmissionControl],
        buildings=[BaseEmissionControl],
    )
