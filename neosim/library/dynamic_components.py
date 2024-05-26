from pathlib import Path

from neosim.controller.parser import (
    ControllerBus,
    IntegerOutput,
    RealInput,
    RealOutput,
    BooleanInput,
)
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

  Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.SetPoints.OutdoorAirFlow.ASHRAE62_1.SumZone
    sumZon(nZon={{element.vavs | length }}, nGro=1,     final zonGroMat=[1],
    final zonGroMatTra=[1])
    {% raw %}annotation (Placement(transformation(extent={{-72,32},{-52,52}})));{% endraw %}
      Buildings.Controls.OBC.CDL.Integers.MultiSum preRetReq(final nin={{element.vavs | length }})
    {% raw %}annotation (Placement(transformation(extent={{-72,80},{-60,92}})));{% endraw %}
  Buildings.Controls.OBC.CDL.Integers.MultiSum temResReq(final nin={{element.vavs | length }})
    {% raw %}annotation (Placement(transformation(extent={{-72,56},{-60,68}})));{% endraw %}
equation
{{bus_ports | safe}}
{% raw %}
  connect(sumZon.VSumAdjPopBreZon_flow, mulAHUCon.VSumAdjPopBreZon_flow)
    annotation (Line(points={{-50,50},{-22,50},{-22,55},{-14,55}}, color={0,0,127}));
  connect(sumZon.VSumAdjAreBreZon_flow, mulAHUCon.VSumAdjAreBreZon_flow)
    annotation (Line(points={{-50,46},{-20,46},{-20,53},{-14,53}}, color={0,0,127}));
  connect(sumZon.VSumZonPri_flow, mulAHUCon.VSumZonPri_flow) annotation (Line(
        points={{-50,38},{-38,38},{-38,44},{-14,44},{-14,50}}, color={0,0,127}));
  connect(sumZon.uOutAirFra_max, mulAHUCon.uOutAirFra_max) annotation (Line(
        points={{-50,34},{-34,34},{-34,40},{-20,40},{-20,42},{-14,42},{-14,47}},
        color={0,0,127}));
      connect(temResReq.y, mulAHUCon.uZonTemResReq) annotation (Line(points={{-58.8,
          62},{-58.8,63},{-14,63}}, color={255,127,0}));
  connect(preRetReq.y, mulAHUCon.uZonPreResReq) annotation (Line(points={{-58.8,
          86},{-22,86},{-22,71},{-14,71}}, color={255,127,0}));
        {% endraw %}
end AhuControl{{ element.name  | capitalize}};""",
    category="control",
    bus=ControllerBus.from_configuration(
        Path("/home/aan/Documents/neosim/neosim/controller/ahu_controller.json")
    ),
)

dynamic_vav_control_template = DynamicComponentTemplate(
    template="""model VAVControl{{ element.name | capitalize}}
Buildings.Controls.OBC.ASHRAE.G36.TerminalUnits.Reheat.Controller rehBoxCon(
venStd=Buildings.Controls.OBC.ASHRAE.G36.Types.VentilationStandard.ASHRAE62_1,
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


dynamic_pump_template = DynamicComponentTemplate(
    template="""
model Pump{{ element.name | capitalize}}
extends {{ package_name }}.Common.Fluid.Ventilation.PartialPump;
{{bus_template}}
equation
{{bus_ports | safe}}
 end Pump{{ element.name | capitalize}};
 """,
    category="ventilation",
    bus=ControllerBus(
        real_inputs=[
            RealInput(name="y", target="element.name", component="pumRad", port="y")
        ],
        real_outputs=[
            RealOutput(
                name="y_gain",
                target="element.name",
                component="gain",
                port="y",
            ),
            RealOutput(
                name="T",
                target="element.control.name",
                component="temSup",
                port="T",
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
{% for input in element.non_connected_ports %}
{{ input.default_template | safe}}
{% endfor %}
equation
{% for index, space in element.spaces|enumerate %}
connect(port[{{index + 1}}],TRoo[{{index + 1}}]. port);
connect(port_a[{{index + 1}}], TRoo1[{{index + 1}}].port);
{% endfor %}
{{bus_ports | safe}}

{% for input in element.non_connected_ports %}
connect(dataBus.{{ input.name }}{{ input.target }}, {{ input.name }}.y);
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

dynamic_emission_control_template = DynamicComponentTemplate(
    template="""model EmissionControl{{ element.name | capitalize}}
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops emissionControl()
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
                default=25,
                name="TCooSet",
                target="element.space_name",
                component="emissionControl",
                port="TCooSet",
            ),
            RealInput(
                default=21,
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


dynamic_collector_control_template = DynamicComponentTemplate(
    template="""model CollectorControl{{ element.name | capitalize}}
  Buildings.Controls.OBC.CDL.Reals.PIDWithReset
                                      conPum(
    yMax=1,
    Td=60,
    yMin=0.05,
    k=0.5,
    Ti=15) "Controller for pump"
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


dynamic_three_way_valve_control_template = DynamicComponentTemplate(
    template="""model ThreeWayValveControl{{ element.name | capitalize}}
  Buildings.Controls.OBC.CDL.Reals.PIDWithReset
                                      conVal(
    yMax=1,
    yMin=0,
    xi_start=1,
    Td=60,
    k=0.1,
    Ti=120) "Controller for pump"
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
                default=90,
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


dynamic_boiler_template = DynamicComponentTemplate(
    template="""
    model BoilerWithStorage{{ element.name | capitalize}}
    extends {{ package_name }}.Common.Fluid.Boilers.PartialBoilerWithStorage;
    {{bus_template}}
    equation
    {{bus_ports | safe}}
     end BoilerWithStorage{{ element.name | capitalize}};
     """,
    category="boiler",
    bus=ControllerBus(
        real_inputs=[
            RealInput(
                name="yBoiCon", target="element.name", component="boi", port="y"
            ),
            RealInput(
                name="yPumBoi", target="element.name", component="pumBoi", port="y"
            )
        ],
        real_outputs=[
            RealOutput(
                name="TStoTop", target="element.name", component="tanTemTop", port="T"
            ),
            RealOutput(
                name="TStoBot",
                target="element.name",
                component="tanTemBot",
                port="T",
            )
        ],
    ),
)
