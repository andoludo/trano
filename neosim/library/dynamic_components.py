from pathlib import Path

from neosim.controller.parser import ControllerBus, IntegerOutput, RealInput, RealOutput
from neosim.models.elements.base import DynamicComponentTemplate

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

dynamic_ahu_controller_template = DynamicComponentTemplate(
    template="""model AhuControl{{ element.name | capitalize}}
  parameter Real VUncDesOutAir_flow=0.03;
  parameter Real VDesTotOutAir_flow=0.03;
      Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.Controller
        mulAHUCon(
        eneStd=Buildings.Controls.OBC.ASHRAE.G36.Types.EnergyStandard.ASHRAE90_1,
        venStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,
        ashCliZon=Buildings.Controls.OBC.ASHRAE.G36.Types.ASHRAEClimateZone.Zone_1A,
        have_frePro=false,
        minOADes=Buildings.Controls.OBC.ASHRAE.G36.Types.OutdoorAirSection.DedicatedDampersAirflow,
        buiPreCon=Buildings.Controls.OBC.ASHRAE.G36.Types.PressureControl.BarometricRelief,
        ecoHigLimCon=Buildings.Controls.OBC.ASHRAE.G36.Types.ControlEconomizer.FixedDryBulb,
        cooCoi=Buildings.Controls.OBC.ASHRAE.G36.Types.CoolingCoil.None,
        heaCoi=Buildings.Controls.OBC.ASHRAE.G36.Types.HeatingCoil.None,
        have_perZonRehBox=false, VUncDesOutAir_flow = VUncDesOutAir_flow, VDesTotOutAir_flow = VDesTotOutAir_flow)
        {% raw %}annotation (Placement(transformation(extent={{-12,-14},{28,74}})));{% endraw %}
{{bus_template}}
equation
{{bus_ports | safe}}
end AhuControl{{ element.name  | capitalize}};""",
    category="control",
    bus=ControllerBus.from_configuration(
        Path("/home/aan/Documents/neosim/neosim/controller/ahu_controller.json")
    ),
)

dynamic_vav_control_template = DynamicComponentTemplate(
    template="""model VAVControl{{ element.name | capitalize}}
Buildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller rehBoxCon(
have_winSen=true,
have_occSen=true,
have_CO2Sen=true,
have_hotWatCoi=true,
VOccMin_flow=0.003,
VAreMin_flow=0.003,
VAreBreZon_flow=0.003,
VPopBreZon_flow=0.003,
VMin_flow=0.003,
VCooMax_flow=0.003,
VHeaMin_flow=0.003,
VHeaMax_flow=0.003)
{% raw %}annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); {% endraw %}
{{bus_template}}
equation
{{bus_ports | safe}}
end VAVControl{{ element.name  | capitalize}};""",
    category="control",
    bus=ControllerBus.from_configuration(
        Path("/home/aan/Documents/neosim/neosim/controller/vav_controller.json")
    ),
)


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
equation
{% for index, space in element.spaces|enumerate %}
connect(port[{{index + 1}}],TRoo[{{index + 1}}]. port);
connect(port_a[{{index + 1}}], TRoo1[{{index + 1}}].port);
{% endfor %}
{{bus_ports | safe}}
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
