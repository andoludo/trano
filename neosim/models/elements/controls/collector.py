from typing import Callable, List, Optional

from pydantic import Field

from neosim.controller.parser import ControllerBus, RealInput, RealOutput
from neosim.models.elements.base import (
    AvailableLibraries,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)
from neosim.models.elements.bus import DataBus
from neosim.models.elements.controls.base import Control, PIDParameters
from neosim.models.elements.system import System

dynamic_collector_control_template = DynamicComponentTemplate(
    template="""model CollectorControl{{ element.name | capitalize}}
Buildings.Controls.OBC.CDL.Reals.PIDWithReset
conPum({{ macros.render_parameters(parameters) | safe}}) "Controller for pump"
{% raw %}annotation (Placement(transformation(extent={{54,-10},{74,10}})));{% endraw %}
Buildings.Controls.OBC.CDL.Reals.MultiMax
mulMax(nin={{ element.valves | length }})
"Maximum radiator valve position"
{% raw %}annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));{% endraw %}
Buildings.Controls.OBC.CDL.Reals.Hysteresis
hysPum(uLow=0.01, uHigh=0.5)
"Hysteresis for pump"
{% raw %}annotation (Placement(transformation(extent={{-26,-10},{-6,10}})));{% endraw %}
Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
"Conversion from boolean to real signal"
{% raw %}annotation (Placement(transformation(extent={{14,-10},{34,10}})));{% endraw %}
{{bus_template}}
equation
connect(mulMax.y,hysPum. u) {% raw %}annotation (Line(
points={{-54,0},{-28,0}},
color={0,0,127},
smooth=Smooth.None));{% endraw %}
connect(hysPum.y,conPum. trigger) {% raw %}annotation (Line(points={{-4,0},{4,0},{4,-18},
{58,-18},{58,-12}},     color={255,0,255}));{% endraw %}
connect(hysPum.y,booToRea. u)
{% raw %}annotation (Line(points={{-4,0},{12,0}},   color={255,0,255}));{% endraw %}
connect(booToRea.y,conPum. u_s)
{% raw %}annotation (Line(points={{36,0},{52,0}},     color={0,0,127}));{% endraw %}
{% raw %}annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
coordinateSystem(preserveAspectRatio=false)));{% endraw %}
{{bus_ports | safe}}
end CollectorControl{{ element.name | capitalize}};""",
    category="control",
    bus=ControllerBus(
        real_outputs=[
            RealOutput(
                name="y",
                target="element.controllable_element.name",
                component="conPum",
                port="y",
            ),
        ],
        real_inputs=[
            RealInput(
                default=25,
                name="y_gain",
                target="element.controllable_element.name",
                component="conPum",
                port="u_m",
            ),
            RealInput(
                default=25,
                name="yHea",
                target="[vav.name for vav in element.valves]",
                component="mulMax",
                multi=True,
                port="u",
            ),
        ],
    ),
)


class BaseCollectorControl(LibraryData):
    template: str = """
    {{package_name}}.Common.Controls.ventilation.CollectorControl{{ element.name | capitalize}}
    {{ element.name }}"""
    component_template: DynamicComponentTemplate = dynamic_collector_control_template
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


class CollectorControl(Control):
    valves: Optional[List[str]] = None
    parameters: PIDParameters = Field(default=PIDParameters())
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[BaseCollectorControl],
        buildings=[BaseCollectorControl],
    )
