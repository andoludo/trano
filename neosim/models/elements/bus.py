from typing import Callable, List, Optional

from pydantic import Field

from neosim.controller.parser import BaseInput, ControllerBus, RealOutput
from neosim.models.constants import Flow
from neosim.models.elements.base import (
    AvailableLibraries,
    BaseElement,
    DynamicComponentTemplate,
    LibraryData,
    Port,
)

dynamic_data_server_template = DynamicComponentTemplate(
    template="""
model DataServer
replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[{{ element.spaces | length}}] TRoo {% raw %}annotation (
                Placement(transformation(origin={-544,-226},    extent = {{480, 216}, {500, 236}})));{% endraw %}
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[{{ element.spaces | length}}] port {% raw %}annotation (
                Placement(transformation(extent={{-112,-10},{-92,10}}),      iconTransformation(extent = {{-110, -10}, {-90, 10}})));{% endraw %}

Buildings.Fluid.Sensors.PPM[{{ element.spaces | length}}] TRoo1(redeclare
package Medium = Medium){% raw %}annotation (
          Placement(transformation(origin={-542,-268},    extent = {{480, 216}, {500, 236}})));{% endraw %}
Modelica.Fluid.Interfaces.FluidPort_a[{{ element.spaces | length}}] port_a(redeclare package Medium
= Medium){% raw %}annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
  iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));  {% endraw %}
{{bus_template}}
{% for input in element.non_connected_ports %}
{{ input.input_model | safe}}
{% endfor %}
equation
{% for index, space in element.spaces|enumerate %}
connect(port[{{index + 1}}],TRoo[{{index + 1}}]. port);
connect(port_a[{{index + 1}}], TRoo1[{{index + 1}}].port);
{% endfor %}
{{bus_ports | safe}}

{% for input in element.non_connected_ports %}
connect(dataBus.{{ input.name }}{{ input.target }}, {{ input.name }}{{input.evaluated_element_name | capitalize}}.y);
{% endfor %}
end DataServer;
      """,
    category="control",
    bus=ControllerBus(
        real_outputs=[
            RealOutput(
                name="TZon",
                target="element.spaces",
                component="TRoo",
                port="T",
            ),
            RealOutput(
                name="ppmCO2",
                target="element.spaces",
                component="TRoo1",
                port="ppm",
            ),
        ],
    ),
)


def data_bus_factory():
    from neosim.models.elements.controls.base import Control
    from neosim.models.elements.space import Space
    from neosim.models.elements.system import System

    class BaseDataBus(LibraryData):
        template: str = """    {{package_name}}.Common.Controls.ventilation.DataServer
        {{ element.name }} (redeclare package
          Medium = Medium)"""
        component_template: DynamicComponentTemplate = dynamic_data_server_template
        ports_factory: Callable[[], List[Port]] = Field(
            default=lambda: [
                Port(
                    targets=[Space],
                    names=["port"],
                    multi_connection=True,
                    use_counter=True,
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

    return BaseDataBus()


class DataBus(BaseElement):
    name: str
    position: Optional[List[float]] = None
    spaces: List[str]
    non_connected_ports: List[BaseInput] = Field(default=[])
    libraries_data: AvailableLibraries = AvailableLibraries(
        ideas=[data_bus_factory],
        buildings=[data_bus_factory],
    )
