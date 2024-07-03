from typing import Callable, List

from pydantic import Field

from trano.controller.parser import ControllerBus, RealInput, RealOutput
from trano.models.elements.base import (
    AvailableLibraries,
    Axis,
    BaseParameter,
    DynamicComponentTemplate,
    Figure,
    LibraryData,
    Line,
    Port,
)
from trano.models.elements.bus import DataBus
from trano.models.elements.controls.base import Control
from trano.models.elements.system import System


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
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops
emissionControl({{ macros.render_parameters(parameters) | safe}})
{% raw %}annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); {% endraw %}
Modelica.Blocks.Interfaces.RealOutput y
{% raw %}annotation (Placement(transformation(extent={{100,-8},{120,12}})));{% endraw %}
{% raw %}

  Buildings.Controls.SetPoints.OccupancySchedule
                                   occSch2(firstEntryOccupied=true, occupancy=
        schedule)

annotation (Placement(transformation(extent={{-116,-36},{-96,-16}})));
  Buildings.Controls.OBC.CDL.Reals.Switch switch2
annotation (Placement(transformation(extent={{-70,-26},{-50,-6}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant setpoint(k=THeaSet)
    "Heat gain if occupied in room 2"
    annotation (Placement(transformation(extent={{-116,14},{-96,34}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant setback(k=THeaSetBack)
    "Heat gain if occupied in room 2"
    annotation (Placement(transformation(extent={{-112,-82},{-92,-62}})));
{% endraw %}
{{bus_template}}
equation
connect(emissionControl.yHea, y) {% raw %}annotation (Line(points={{34.4,-21.2},{96,-21.2},
{96,2},{110,2}}, color={0,0,127}));{% endraw %}
{% raw %}
  connect(
emissionControl.yHea, y) annotation (Line(points={{60.4,-21.2},{96,-21.2},{96,2},
          {110,2}}, color={0,0,127}));
  connect(occSch2.
              occupied, switch2.u2) annotation (Line(
  points={{-95,-32},{-78,-32},{-78,-16},{-72,-16}},
  color={255,0,255},
  smooth=Smooth.None));
  connect(setpoint.y, switch2.u1) annotation (Line(points={{-94,24},{-90,24},{-90,
          -8},{-72,-8}}, color={0,0,127}));
  connect(setback.y, switch2.u3)
    annotation (Line(points={{-90,-72},{-72,-72},{-72,-24}}, color={0,0,127}));
      connect(switch2.y, emissionControl.THeaSet) annotation (Line(points={{-52,-22},
          {-52,-21.2},{-42.4,-21.2}}, color={0,0,127}));
    {% endraw %}
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
    component_template: DynamicComponentTemplate = dynamic_emission_control_template
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
    figures: List[Figure] = Field(
        default=[
            Figure(
                right_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.name }}.emissionControl.conHea.u_s",
                            label="Zone controller setpoint [K]",
                        ),
                        Line(
                            template="{{ element.name }}.emissionControl.conHea.u_m",
                            label="Zone controller measured [K]",
                        ),
                    ],
                    label="Zone controller input [K]",
                ),
                left_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.name }}.y", label="Control signal [-]"
                        ),
                    ],
                    label="Control signal [-]",
                ),
            )
        ]
    )


class EmissionControl(Control):
    parameters: ControlLoopsParameters = Field(default=ControlLoopsParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseEmissionControl],
        buildings=[BaseEmissionControl],
    )
