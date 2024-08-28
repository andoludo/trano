from typing import Callable, List

from pydantic import Field

from trano.controller.parser import BooleanInput, ControllerBus, RealInput
from trano.models.elements.base import DynamicComponentTemplate, LibraryData, Port
from trano.models.elements.bus import DataBus
from trano.models.elements.controls.base import Control
from trano.models.elements.temperature_sensor import TemperatureSensor
from trano.models.elements.three_way_valve import ThreeWayValve

dynamic_three_way_valve_control_template = DynamicComponentTemplate(
    template="""model ThreeWayValveControl{{ element.name | capitalize}}
  Buildings.Controls.OBC.CDL.Reals.PIDWithReset
                                      conVal(
    {{ macros.render_parameters(parameters) | safe}}) "Controller for pump"
    {% raw %}annotation (Placement(transformation(extent={{-12,-10},{8,10}})));{% endraw %}
  Modelica.Blocks.Interfaces.RealOutput y
    {% raw %}annotation (Placement(transformation(extent={{100,-10},{120,10}})));{% endraw %}
  Modelica.Blocks.Interfaces.RealInput u
    {% raw %}annotation (Placement(transformation(extent={{-138,-20},{-98,20}})));{% endraw %}
        {{bus_template}}
equation
{{bus_ports | safe}}
  connect(conVal.y, y)
    {% raw %}annotation (Line(points={{10,0},{110,0}}, color={0,0,127}));{% endraw %}
  connect(u, conVal.u_m) {% raw %}annotation (Line(points={{-118,0},{-22,0},{-22,-20},{0,
          -20},{0,-16},{-2,-16},{-2,-12}}, color={0,0,127}));{% endraw %}
  {% raw %}annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));{% endraw %}
end ThreeWayValveControl{{ element.name | capitalize}};""",
    category="control",
    bus=ControllerBus(
        real_inputs=[
            RealInput(
                default=273.15 + 90,
                name="TColSet",
                target="element.name",
                component="conVal",
                port="u_s",
            ),
        ],
        boolean_inputs=[
            BooleanInput(
                default="true",
                name="trigger",
                target="element.name",
                component="conVal",
                port="trigger",
            ),
        ],
    ),
)


class BaseThreeWayValveControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.
    ThreeWayValveControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: DynamicComponentTemplate = (
        dynamic_three_way_valve_control_template
    )
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


class ThreeWayValveControl(Control):
    ...
