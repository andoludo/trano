package case600FF

package Trano
  package Occupancy

    model SimpleOccupancy

      parameter Real occupancy[:]=3600*{7,19}
    "Occupancy table, each entry switching occupancy on or off";

      parameter Real gain[:, :]=[35; 70; 30]
    "Gain to convert from occupancy (per person) to radiant, convective and latent heat in [W/m2] ";

          parameter Real k=1/6/4
    "Heat gain if occupied";

  parameter Boolean firstEntryOccupied = true
    "Set to true if first entry in occupancy denotes a changed from unoccupied to occupied";

  Buildings.Controls.SetPoints.OccupancySchedule
                                   occSch2(firstEntryOccupied=firstEntryOccupied,
  occupancy=occupancy)
"Occupancy schedule"
annotation (Placement(transformation(extent={{-66,-22},{-46,-2}})));
      Buildings.Controls.OBC.CDL.Reals.Switch switch2
    annotation (Placement(transformation(extent={{-20,-12},{0,8}})));
      Modelica.Blocks.Math.MatrixGain gai2(K=gain)
    "Gain to convert from occupancy (per person) to radiant, convective and latent heat in [W/m2] "
    annotation (Placement(transformation(extent={{18,-12},{38,8}})));
      extends Modelica.Blocks.Interfaces.MO(
                                         final nout=3);
      Buildings.Controls.OBC.CDL.Reals.Sources.Constant occ2(k=k)
    "Heat gain if occupied in room 2"
    annotation (Placement(transformation(extent={{-66,28},{-46,48}})));
      Buildings.Controls.OBC.CDL.Reals.Sources.Constant zero(k=0)
    "Heat gain if occupied in room 2"
    annotation (Placement(transformation(extent={{-62,-68},{-42,-48}})));
    equation
      connect(
          occSch2.occupied,switch2. u2) annotation (Line(
      points={{-45,-18},{-28,-18},{-28,-2},{-22,-2}},
      color={255,0,255},
      smooth=Smooth.None));
      connect(
          switch2.y,gai2. u[1]) annotation (Line(
      points={{2,-2},{16,-2}},
      color={0,0,127},
      smooth=Smooth.None));
      connect(
          occ2.y,switch2. u1) annotation (Line(points={{-44,38},{-40,38},{-40,6},
          {-22,6}},        color={0,0,127}));
      connect(
          zero.y, switch2.u3)
    annotation (Line(points={{-40,-58},{-22,-58},{-22,-10}}, color={0,0,127}));
      connect(
          gai2.y, y) annotation (Line(points={{39,-2},{96,-2},{96,0},{110,0}},
        color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(extent={{10,70},{-26,34}}, lineColor={28,108,200}),
        Line(points={{-8,34},{-8,-26}}, color={28,108,200}),
        Line(points={{-8,-26},{-48,-68}}, color={28,108,200}),
        Line(points={{-8,-26},{34,-70},{32,-70}}, color={28,108,200}),
        Line(points={{-8,20},{-48,-8}}, color={28,108,200}),
        Line(points={{-8,20},{44,-8}}, color={28,108,200})}),    Diagram(
        coordinateSystem(preserveAspectRatio=false)));
    end SimpleOccupancy;

    model ISO13790

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,30},{120,50}})));
      Modelica.Blocks.Interfaces.RealOutput y1
        annotation (Placement(transformation(extent={{100,-48},{120,-28}})));
      Modelica.Blocks.Sources.Constant const(k=10)
        annotation (Placement(transformation(extent={{-44,30},{-24,50}})));
      Modelica.Blocks.Sources.Constant const1(k=1)
        annotation (Placement(transformation(extent={{-42,-50},{-22,-30}})));
    equation
      connect(const.y, y)
        annotation (Line(points={{-23,40},{110,40}}, color={0,0,127}));
      connect(const1.y, y1) annotation (Line(points={{-21,-40},{96,-40},{96,-38},
              {110,-38}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(extent={{10,70},{-26,34}}, lineColor={28,108,200}),
        Line(points={{-8,34},{-8,-26}}, color={28,108,200}),
        Line(points={{-8,-26},{-48,-68}}, color={28,108,200}),
        Line(points={{-8,-26},{34,-70},{32,-70}}, color={28,108,200}),
        Line(points={{-8,20},{-48,-8}}, color={28,108,200}),
        Line(points={{-8,20},{44,-8}}, color={28,108,200})}),    Diagram(
        coordinateSystem(preserveAspectRatio=false)));
    end ISO13790;
  end Occupancy;

  package Controls
  package BaseClasses
    expandable connector DataBus
      extends Modelica.Icons.SignalBus;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Rectangle(
              extent={{-20,2},{22,-2}},
              lineColor={255,204,51},
              lineThickness=0.5)}),
        Documentation(info="<html>
<p>
This connector defines the <code>expandable connector</code> ControlBus that
is used to connect control signals.
Note, this connector is empty. When using it, the actual content is
constructed by the signals connected to this bus.
</p>
</html>"));
    end DataBus;
  annotation (
        Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
                fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Ellipse(lineColor = {128, 128, 128}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-30, -30}, {30, 30}})}));
  end BaseClasses;

    package ventilation

      partial model PartialBoilerControl
        parameter Modelica.Units.SI.Temperature TSup_nominal=80 + 273.15
          "Check for temperature at the bottom of the tank";
        parameter Modelica.Units.SI.Temperature threshold_outdoor_air_cutoff=15 +
            273.15 "Output true if outdoor air is below heating cut-off limit";
        parameter Modelica.Units.SI.Temperature threshold_to_switch_off_boiler=15
             + 273.15 "Threshold to switch boiler off";
        Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr(t=
        TSup_nominal + 5) "Check for temperature at the bottom of the tank"
          annotation (
      Placement(transformation(extent={{-114,-142},{-94,-122}})));
        Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToReaPum
          "Signal converter for pump" annotation (Placement(transformation(
          extent={{-94,-32},{-114,-12}})));
        Buildings.Controls.OBC.CDL.Reals.Greater lesThr
          "Check for temperature at the top of the tank" annotation (
      Placement(transformation(extent={{-114,-80},{-94,-60}})));
        Modelica.Blocks.MathBoolean.Or pumOnSig(nu=3)
          "Signal for pump being on"
          annotation (Placement(transformation(extent={{146,-2},{166,18}})));
        Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToReaBoi
          "Signal converter for boiler"
          annotation (Placement(transformation(extent={{-94,-2},{-114,18}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant dTThr(k=1)
          "Threshold to switch boiler off" annotation (Placement(
        transformation(extent={{-204,-112},{-184,-92}})));
        Buildings.Controls.OBC.CDL.Reals.Subtract sub1 annotation (Placement(
        transformation(extent={{-164,-88},{-144,-68}})));
        Modelica.Blocks.Logical.LessThreshold lesThrTOut(threshold=
              threshold_outdoor_air_cutoff)
          "Output true if outdoor air is below heating cut-off limit"
          annotation (Placement(transformation(extent={{-114,38},{-94,58}})));
        Buildings.Controls.OBC.CDL.Logical.And and1
          "Logical test to enable pump and subsequently the boiler"
          annotation (Placement(transformation(extent={{-74,38},{-54,58}})));
        Modelica.StateGraph.InitialStep off(nIn=1, nOut=1)
                      "Pump and furnace off"
          annotation (Placement(transformation(extent={{-74,78},{-54,98}})));
        Modelica.StateGraph.TransitionWithSignal T1 "Transition to pump on"
          annotation (Placement(transformation(extent={{-44,78},{-24,98}})));
        Modelica.StateGraph.StepWithSignal pumOn(nIn=1, nOut=1)
                           "Pump on"
          annotation (Placement(transformation(extent={{-14,78},{6,98}})));
        Modelica.StateGraph.Transition T3(enableTimer=true, waitTime=10)
          "Transition to boiler on"
          annotation (Placement(transformation(extent={{16,78},{36,98}})));
        Modelica.StateGraph.StepWithSignal boiOn(nIn=1, nOut=1)
                           "Boiler on"
          annotation (Placement(transformation(extent={{46,78},{66,98}})));
        Modelica.StateGraph.TransitionWithSignal T2
          "Transition that switches boiler off"
          annotation (Placement(transformation(extent={{76,78},{96,98}})));
        Modelica.StateGraph.StepWithSignal pumOn2(nIn=1, nOut=1)
                            "Pump on"
          annotation (Placement(transformation(extent={{106,78},{126,98}})));
        Modelica.StateGraph.Transition T4(enableTimer=true, waitTime=10)
          "Transition to boiler on"
          annotation (Placement(transformation(extent={{136,78},{156,98}})));
        inner Modelica.StateGraph.StateGraphRoot stateGraphRoot
          "Root of the state graph" annotation (Placement(transformation(
          extent={{-134,98},{-114,118}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant dTThr1(k=
              threshold_to_switch_off_boiler) "Threshold to switch boiler off"
          annotation (Placement(
        transformation(extent={{-208,-22},{-188,-2}})));
      equation
        connect(
          booToReaPum.u, pumOnSig.y)
                     annotation (Line(
      points={{-92,-22},{176,-22},{176,8},{167.5,8}},
      color={255,0,255},
      smooth=Smooth.None));
        connect(
          sub1.y, lesThr.u2)
             annotation (Line(
      points={{-142,-78},{-116,-78}},
      color={0,0,127},
      smooth=Smooth.None));
        connect(
          dTThr.y, sub1.u2)
            annotation (Line(
      points={{-182,-102},{-174,-102},{-174,-84},{-166,-84}},
      color={0,0,127},
      smooth=Smooth.None));
        connect(
          lesThr.y, and1.u2)
             annotation (Line(
      points={{-92,-70},{-84,-70},{-84,40},{-76,40}},
      color={255,0,255},
      smooth=Smooth.None));
        connect(
          lesThrTOut.y, and1.u1)
                 annotation (Line(
      points={{-93,48},{-76,48}},
      color={255,0,255},
      smooth=Smooth.None));
        connect(
          and1.y, T1.condition)
                annotation (Line(points={{-52,48},{-44,48},{-44,42},{-36,
          42},{-36,58},{-34,58},{-34,76}}, color={255,0,255}));
        connect(
          greThr.y, T2.condition)
                  annotation (Line(points={{-92,-132},{86,-132},{86,76}},
        color={255,0,255}));
        connect(
          boiOn.active, booToReaBoi.u)
                       annotation (Line(points={{56,77},{56,8},{-92,8}},
        color={255,0,255}));
        connect(
          pumOn2.active, pumOnSig.u[1])
                        annotation (Line(points={{116,77},{116,12.6667},{146,12.6667}},
           color={255,0,255}));
        connect(
          boiOn.active, pumOnSig.u[2])
                       annotation (Line(points={{56,77},{56,8},{146,8}},
        color={255,0,255}));
        connect(
          pumOn.active, pumOnSig.u[3])
                       annotation (Line(points={{-4,77},{-4,3.33333},{146,3.33333}},
                         color={255,0,255}));
        connect(
          off.outPort[1], T1.inPort)
          annotation (Line(points={{-53.5,88},{-38,88}}, color={0,0,0}));
        connect(
          T1.outPort, pumOn.inPort[1])
          annotation (Line(points={{-32.5,88},{-15,88}}, color={0,0,0}));
        connect(
          pumOn.outPort[1], T3.inPort)
                       annotation (Line(points={{6.5,88},{22,88}},
                     color={0,0,0}));
        connect(
          T3.outPort, boiOn.inPort[1])
          annotation (Line(points={{27.5,88},{45,88}}, color={0,0,0}));
        connect(
          boiOn.outPort[1], T2.inPort)
          annotation (Line(points={{66.5,88},{82,88}}, color={0,0,0}));
        connect(
          T2.outPort, pumOn2.inPort[1])
          annotation (Line(points={{87.5,88},{105,88}}, color={0,0,0}));
        connect(
          pumOn2.outPort[1], T4.inPort)
          annotation (Line(points={{126.5,88},{142,88}}, color={0,0,0}));
        connect(
          T4.outPort, off.inPort[1])
                     annotation (Line(points={{147.5,88},{166,88},{166,118},
          {-94,118},{-94,88},{-75,88}},
                                color={0,0,0}));
        connect(dTThr1.y, lesThr.u1)
                       annotation (Line(points={{-186,-12},{-124,-12},{
          -124,-70},{-116,-70}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={
            {-260,-180},{260,160}})), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-260,-180},{260,160}})));
      end PartialBoilerControl;

    end ventilation;
  annotation (
      Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(origin = {0, 35.1488}, fillColor = {255, 255, 255}, extent = {{-30, -20.1488}, {30, 20.1488}}), Rectangle(origin = {0, -34.8512}, fillColor = {255, 255, 255}, extent = {{-30, -20.1488}, {30, 20.1488}}), Line(origin = {-51.25, 0}, points = {{21.25, -35}, {-13.75, -35}, {-13.75, 35}, {6.25, 35}}), Polygon(origin = {-40, 35}, pattern = LinePattern.None,
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{10, 0}, {-5, 5}, {-5, -5}, {10, 0}}), Line(origin = {51.25, 0}, points = {{-21.25, 35}, {13.75, 35}, {13.75, -35}, {-6.25, -35}}), Polygon(origin = {40, -35}, pattern = LinePattern.None,
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-10, 0}, {5, 5}, {5, -5}, {-10, 0}})}));
  end Controls;

  package Fluid
  package Boilers
partial model PartialBoilerWithoutStorage
  replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
      "Medium model" annotation (choicesAllMatching=true);
  extends Buildings.Fluid.Interfaces.PartialTwoPort(
                                            redeclare package Medium = MediumW);
parameter Boolean useStorageTank = false "Use storage tank"annotation(Dialog(tab="Storage tank", group="Properties"));
      parameter Modelica.Units.SI.Temperature TSet=323.15;
   parameter Modelica.Units.SI.Temperature TSouSet=278.15;
   parameter Integer modusValue=1;

  parameter Buildings.Fluid.HeatPumps.Data.EquationFitReversible.EnergyPlus
                                                          per
   "Reverse heat pump performance data"
   annotation (Placement(transformation(extent={{-90,76},{-70,96}})));
  parameter Real a[:]={0.9} "Coefficients for efficiency curve";
  parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant
    "Curve used to compute the efficiency";
  parameter Modelica.Units.SI.Temperature T_nominal=353.15
    "Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)"
    annotation (Dialog(enable=(effCur == Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear)));
  parameter Modelica.Units.SI.Temperature TempSet=353.15;

  parameter Buildings.Fluid.Data.Fuels.Generic fue "Fuel type"
    annotation (choicesAllMatching=true);

  parameter Modelica.Units.SI.Power Q_flow_nominal "Nominal heating power";
  parameter Boolean linearizeFlowResistance=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(enable=computeFlowResistance,
       tab="Flow resistance"));
  parameter Modelica.Units.SI.PressureDifference dp_nominal(min=0, displayUnit=
"Pa") "Pressure difference" annotation (Dialog(group="Nominal condition"));
parameter Modelica.Units.SI.Pressure dp[:](each displayUnit="Pa")=(3000 + 2000)*{2,1} "Pressure";
parameter Real V_flow[:] = 0.001/1000*{0.5,1};
  parameter Real deltaM=0.1
    "Fraction of nominal flow rate where flow transitions to laminar";
parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_rate_boiler;
parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_radiator_loop;
  parameter Boolean show_T=false;

  parameter Modelica.Units.SI.Volume VTan "Tank volume";
  parameter Modelica.Units.SI.Length hTan "Height of tank (without insulation)";
  parameter Modelica.Units.SI.Length dIns "Thickness of insulation";
  parameter Modelica.Units.SI.ThermalConductivity kIns=0.04
    "Specific heat conductivity of insulation";
  parameter Integer nSeg(min=2) = 2 "Number of volume segments";

  Buildings.Fluid.Boilers.BoilerPolynomial boi(
    a=a,
    effCur=effCur,
    redeclare package Medium = MediumW,
    Q_flow_nominal=Q_flow_nominal,
    m_flow_nominal=nominal_mass_flow_rate_boiler,
    fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue(),
    dp_nominal=dp_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_start=293.15) "Boiler"
    annotation (Placement(transformation(extent={{-24,-10},{-4,10}})));
  Buildings.HeatTransfer.Sources.FixedTemperature TAmb(T=288.15)
    "Ambient temperature in boiler room"
    annotation (Placement(transformation(extent={{20,60},{40,80}})));
  Buildings.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
MediumW) "Fixed boundary condition, needed to provide a pressure in the system"
    annotation (Placement(transformation(extent={{-74,68},{-54,88}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TSupply(redeclare package
      Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
    "Radiator" annotation (Placement(transformation(
        origin={26,-1},
        extent={{-10,-10},{10,10}},
        rotation=0)));
      IDEAS.Controls.Continuous.LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.P)
                                              annotation (Placement(
            transformation(extent={{-20,60},{0,80}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=TempSet)
    annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
Modelica.Blocks.Sources.RealExpression realExpression1(y=boi.VFue_flow)
annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
Modelica.Blocks.Continuous.Integrator GasUsage annotation (
Placement(transformation(extent={{-32,-50},{-12,-30}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TReturn(redeclare package
      Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
    "Radiator" annotation (Placement(transformation(
        origin={-76,1},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Buildings.Fluid.Sensors.VolumeFlowRate senVolFlo(redeclare package Medium =
        MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
    annotation (Placement(transformation(extent={{50,-12},{70,8}})));
      Modelica.Blocks.Routing.RealPassThrough pumpDemandSignal
    annotation (Placement(transformation(extent={{-86,-120},{-60,-94}})));
equation
  connect(
  TAmb.port, boi.heatPort)
           annotation (Line(
      points={{40,70},{44,70},{44,28},{-14,28},{-14,7.2}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(
  bou.ports[1], boi.port_a)
            annotation (Line(
      points={{-54,78},{-50,78},{-50,0},{-24,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(boi.port_b, TSupply.port_a) annotation (Line(points={{-4,0},{14,0},{14,
          -1},{16,-1}}, color={0,127,255}));
      connect(boi.T, conPID.u_m) annotation (Line(points={{-3,8},{-2,8},{-2,42},
          {-10,42},{-10,58}},                  color={0,0,127}));
      connect(conPID.y, boi.y) annotation (Line(points={{1,70},{6,70},{6,24},{-26,
          24},{-26,8}},                             color={0,0,127}));
  connect(realExpression.y, conPID.u_s) annotation (Line(points={{-69,50},{-32,50},
          {-32,70},{-22,70}}, color={0,0,127}));
  connect(realExpression1.y, GasUsage.u)
    annotation (Line(points={{-59,-40},{-34,-40}}, color={0,0,127}));
  connect(port_a, TReturn.port_a) annotation (Line(points={{-100,0},{-93,0},{-93,
          1},{-86,1}}, color={0,127,255}));
  connect(TReturn.port_b, boi.port_a) annotation (Line(points={{-66,1},{-45,1},{
          -45,0},{-24,0}}, color={0,127,255}));
  connect(TSupply.port_b, senVolFlo.port_a) annotation (Line(points={{36,-1},{43,
          -1},{43,-2},{50,-2}}, color={0,127,255}));
  connect(senVolFlo.port_b, port_b) annotation (Line(points={{70,-2},{86,-1},{86,
          0},{100,0}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(extent={{-100,-120},{100,100}}), graphics={
Rectangle(fillPattern=FillPattern.Solid, extent={{-80,80},{80,-80}}),
Rectangle(
  fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          extent={{-68,70},{70,-70}}),
        Polygon(
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          points={{-68,18},{-68,18},{-54,32},{-28,16},{0,30},{26,16},{46,32},{70,
              18},{70,18},{70,-70},{70,-70},{-68,-70},{-68,-70},{-68,18}},
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(extent={{-100,-120},
            {100,100}})));
end PartialBoilerWithoutStorage;


    partial model PartialBoilerWithStorage
      replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);
      extends Buildings.Fluid.Interfaces.PartialTwoPort(
                                                redeclare package Medium = MediumW);
parameter Boolean useStorageTank = false "Use storage tank"annotation(Dialog(tab="Storage tank", group="Properties"));
      parameter Modelica.Units.SI.Temperature TSet=323.15;
   parameter Modelica.Units.SI.Temperature TSouSet=278.15;
   parameter Integer modusValue=1;

  parameter Buildings.Fluid.HeatPumps.Data.EquationFitReversible.EnergyPlus
                                                          per
   "Reverse heat pump performance data"
   annotation (Placement(transformation(extent={{-90,76},{-70,96}})));
      parameter Real a[:]={0.9} "Coefficients for efficiency curve";
      parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant
        "Curve used to compute the efficiency";
      parameter Modelica.Units.SI.Temperature T_nominal=353.15
        "Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)"
        annotation (Dialog(enable=(effCur == Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear)));

      parameter Buildings.Fluid.Data.Fuels.Generic fue "Fuel type"
        annotation (choicesAllMatching=true);

      parameter Modelica.Units.SI.Power Q_flow_nominal "Nominal heating power";
      parameter Boolean linearizeFlowResistance=false
        "= true, use linear relation between m_flow and dp for any flow rate"
        annotation (Dialog(enable=computeFlowResistance,
           tab="Flow resistance"));
      parameter Modelica.Units.SI.PressureDifference dp_nominal(min=0, displayUnit=
    "Pa") "Pressure difference" annotation (Dialog(group="Nominal condition"));
    parameter Modelica.Units.SI.Pressure dp[:]=(3000 + 2000)*{2,1} "Pressure";
    parameter Real V_flow[:] = 0.001/1000*{0.5,1};
      parameter Real deltaM=0.1
        "Fraction of nominal flow rate where flow transitions to laminar";
    parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_rate_boiler;
    parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_radiator_loop;
      parameter Boolean show_T=false;

      parameter Modelica.Units.SI.Volume VTan "Tank volume";
      parameter Modelica.Units.SI.Length hTan "Height of tank (without insulation)";
      parameter Modelica.Units.SI.Length dIns "Thickness of insulation";
      parameter Modelica.Units.SI.ThermalConductivity kIns=0.04
        "Specific heat conductivity of insulation";
      parameter Integer nSeg(min=2) = 2 "Number of volume segments";

      Buildings.Fluid.Movers.SpeedControlled_y pumBoi(
        redeclare package Medium = MediumW,
    per(pressure(V_flow=V_flow, dp=dp)),
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
        "Pump for boiler circuit" annotation (Placement(transformation(extent={{-10,
        -10},{10,10}}, origin={-8,10})));

      Buildings.Fluid.Boilers.BoilerPolynomial boi(
        a=a,
        effCur=effCur,
        redeclare package Medium = MediumW,
        Q_flow_nominal=Q_flow_nominal,
        m_flow_nominal=nominal_mass_flow_rate_boiler,
        fue=fue,
        dp_nominal=dp_nominal,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        T_start=293.15) "Boiler"
        annotation (Placement(transformation(extent={{-74,0},{-54,20}})));
      Buildings.HeatTransfer.Sources.FixedTemperature TAmb(T=288.15)
        "Ambient temperature in boiler room"
        annotation (Placement(transformation(extent={{-14,74},{6,94}})));
      Buildings.Fluid.Storage.StratifiedEnhanced tan1(
        m_flow_nominal=nominal_mass_flow_radiator_loop,
        dIns=dIns,
        redeclare package Medium = MediumW,
        hTan=hTan,
        nSeg=nSeg,
        show_T=true,
        VTan=VTan,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Storage tank"
        annotation (Placement(transformation(extent={{12,-72},{52,-32}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tanTemBot
        "Tank temperature"
        annotation (Placement(transformation(extent={{68,-66},{88,-46}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tanTemTop
        "Tank temperature"
        annotation (Placement(transformation(extent={{68,-34},{88,-14}})));
      Buildings.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package
            Medium =
    MediumW) "Fixed boundary condition, needed to provide a pressure in the system"
        annotation (Placement(transformation(extent={{-74,68},{-54,88}})));
      Buildings.Fluid.FixedResistances.Junction splVal3(
        dp_nominal={0,0,0},
        m_flow_nominal=nominal_mass_flow_rate_boiler*{-1,-1,1},
        redeclare package Medium = MediumW,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Flow splitter"
        annotation (Placement(transformation(
    extent={{10,-10},{-10,10}},
    rotation=180,
    origin={30,-98})));
      Buildings.Fluid.FixedResistances.Junction splVal4(
        dp_nominal={0,0,0},
        m_flow_nominal=nominal_mass_flow_rate_boiler*{1,1,-1},
        redeclare package Medium = MediumW,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Flow splitter"
        annotation (Placement(transformation(
    extent={{10,10},{-10,-10}},
    rotation=180,
    origin={32,10})));
            Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor1(
          redeclare package Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
                                          "Radiator"  annotation (
        Placement(transformation(origin={-36,11},
        extent = {{-10, -10}, {10, 10}},
            rotation=0)));
            Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor2(
          redeclare package Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
                                          "Radiator"  annotation (
        Placement(transformation(origin={66,-85},
        extent = {{-10, -10}, {10, 10}},
            rotation=0)));
            Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor3(
          redeclare package Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
                                          "Radiator"  annotation (
        Placement(transformation(origin={32,-16.5},
        extent={{-8.5,-8},{8.5,8}},
            rotation=-90)));
            Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor4(
          redeclare package Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
                                          "Radiator"  annotation (
        Placement(transformation(origin={-34,-97},
        extent = {{-10, -10}, {10, 10}},
            rotation=0)));
      Buildings.Fluid.Sensors.MassFlowRate senMasFlo1(redeclare package Medium =
            MediumW)
        annotation (Placement(transformation(extent={{10,-10},{-10,10}},
            rotation=90,
            origin={46,30})));
      Buildings.Fluid.Sensors.MassFlowRate senMasFlo2(redeclare package Medium =
            MediumW)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-78,-46})));
      Buildings.Fluid.Sensors.MassFlowRate senMasFlo3(redeclare package Medium =
            MediumW)
        annotation (Placement(transformation(extent={{-6,-7},{6,7}},
            rotation=0,
            origin={13,10})));
      Buildings.Fluid.Sensors.MassFlowRate senMasFlo4(redeclare package Medium =
            MediumW)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=0,
            origin={94,-86})));
      Modelica.Blocks.Math.Gain gain(k=Q_flow_nominal)
        annotation (Placement(transformation(extent={{-70,-22},{-58,-10}})));
      Modelica.Blocks.Continuous.Integrator integrator
        annotation (Placement(transformation(extent={{-46,-26},{-30,-10}})));
      Modelica.Blocks.Math.Gain gain1(k=2.77778e-7)
        annotation (Placement(transformation(extent={{-46,-52},{-26,-32}})));
      Modelica.Blocks.Math.Gain gain2(k=0.9*(1/11))
        annotation (Placement(transformation(extent={{-26,-80},{-6,-60}})));
      Modelica.Blocks.Routing.RealPassThrough Boiy
        annotation (Placement(transformation(extent={{-122,36},{-104,54}})));
    equation
      connect(
      TAmb.port, boi.heatPort)
               annotation (Line(
          points={{6,84},{20,84},{20,30},{-64,30},{-64,17.2}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(
      tan1.heaPorVol[1], tanTemTop.port)
                         annotation (Line(
          points={{32,-52.6},{32,-52},{64,-52},{64,-24},{68,-24}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(
      tanTemBot.port, tan1.heaPorVol[tan1.nSeg])
                                 annotation (Line(
          points={{68,-56},{56,-56},{56,-52},{32,-52}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(
      tan1.heaPorTop, TAmb.port)
                 annotation (Line(
          points={{36,-37.2},{62,-37.2},{62,84},{6,84}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(
      TAmb.port, tan1.heaPorSid)
                 annotation (Line(
          points={{6,84},{62,84},{62,-50},{54,-50},{54,-52},{43.2,-52}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(
      TAmb.port, tan1.heaPorBot)
                 annotation (Line(
          points={{6,84},{62,84},{62,-50},{54,-50},{54,-52},{8,-52},{8,-80},{36,-80},
              {36,-66.8}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(
      bou.ports[1], boi.port_a)
                annotation (Line(
          points={{-54,78},{-48,78},{-48,32},{-80,32},{-80,10},{-74,10}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(
      tan1.port_b, splVal3.port_3) annotation (Line(points={{32,-72},{32,-84},{30,-84},
              {30,-88}}, color={0,127,255}));
      connect(boi.port_b, temperature_sensor1.port_a) annotation (Line(points={{-54,
              10},{-50,10},{-50,11},{-46,11}}, color={0,127,255}));
      connect(temperature_sensor1.port_b, pumBoi.port_a) annotation (Line(points={{-26,11},
              {-23,11},{-23,10},{-18,10}},     color={0,127,255}));
      connect(splVal3.port_2, temperature_sensor2.port_a) annotation (Line(points={{40,-98},
              {52,-98},{52,-85},{56,-85}},         color={0,127,255}));
      connect(splVal4.port_3, temperature_sensor3.port_a)
        annotation (Line(points={{32,0},{32,-8}}, color={0,127,255}));
      connect(temperature_sensor3.port_b, tan1.port_a)
        annotation (Line(points={{32,-25},{32,-32}}, color={0,127,255}));
      connect(splVal3.port_1, temperature_sensor4.port_b) annotation (Line(points={{20,-98},
              {18,-97},{-24,-97}},                 color={0,127,255}));
      connect(senMasFlo1.port_b, splVal4.port_2) annotation (Line(points={{46,20},{46,
              16},{42,16},{42,10}}, color={0,127,255}));
      connect(boi.port_a, senMasFlo2.port_b)
        annotation (Line(points={{-74,10},{-78,10},{-78,-36}}, color={0,127,255}));
      connect(temperature_sensor4.port_a, senMasFlo2.port_a) annotation (Line(
            points={{-44,-97},{-44,-98},{-78,-98},{-78,-56}}, color={0,127,255}));
      connect(splVal4.port_1, senMasFlo3.port_b)
        annotation (Line(points={{22,10},{19,10}}, color={0,127,255}));
      connect(senMasFlo3.port_a, pumBoi.port_b)
        annotation (Line(points={{7,10},{2,10}}, color={0,127,255}));
      connect(temperature_sensor2.port_b, senMasFlo4.port_a) annotation (Line(
            points={{76,-85},{80,-85},{80,-86},{84,-86}}, color={0,127,255}));
      connect(senMasFlo1.port_a, port_b) annotation (Line(points={{46,40},{46,44},{86,
              44},{86,0},{100,0}}, color={0,127,255}));
      connect(port_a, senMasFlo4.port_b) annotation (Line(points={{-100,0},{-84,0},{
              -84,-32},{-96,-32},{-96,-114},{110,-114},{110,-86},{104,-86}}, color={
              0,127,255}));
      connect(gain.y, integrator.u) annotation (Line(points={{-57.4,-16},{-54,-16},{
              -54,-18},{-47.6,-18}}, color={0,0,127}));
      connect(integrator.y, gain1.u) annotation (Line(points={{-29.2,-18},{-30,-18},
              {-30,-4},{-74,-4},{-74,-12},{-76,-12},{-76,-32},{-48,-32},{-48,-42}},
            color={0,0,127}));
      connect(gain1.y, gain2.u) annotation (Line(points={{-25,-42},{-34,-42},{-34,-60},
              {-60,-60},{-60,-70},{-28,-70}}, color={0,0,127}));
      connect(Boiy.y, boi.y) annotation (Line(points={{-103.1,45},{-84,45},{-84,18},
              {-76,18}}, color={0,0,127}));
      connect(Boiy.y, gain.u) annotation (Line(points={{-103.1,45},{-84,45},{-84,2},
              {-82,2},{-82,-16},{-71.2,-16}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(extent={{-100,-120},{100,100}}), graphics={
    Rectangle(fillPattern=FillPattern.Solid, extent={{-80,80},{80,-80}}),
    Rectangle(
      fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              extent={{-68,70},{70,-70}}),
            Polygon(
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              points={{-68,18},{-68,18},{-54,32},{-28,16},{0,30},{26,16},{46,32},{70,
                  18},{70,18},{70,-70},{70,-70},{-68,-70},{-68,-70},{-68,18}},
              smooth=Smooth.Bezier)}), Diagram(coordinateSystem(extent={{-100,-120},
                {100,100}})));
    end PartialBoilerWithStorage;

    partial model PartialWaterWaterHeatPump
        replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
      "Medium model" annotation (choicesAllMatching=true);
  extends Buildings.Fluid.Interfaces.PartialTwoPort(
                                            redeclare package Medium = MediumW);



parameter Boolean useStorageTank = false "Use storage tank"annotation(Dialog(tab="Storage tank", group="Properties"));
      parameter Modelica.Units.SI.Temperature TSet=323.15;
   parameter Modelica.Units.SI.Temperature TSouSet=278.15;
   parameter Integer modusValue=1;

  parameter Buildings.Fluid.HeatPumps.Data.EquationFitReversible.EnergyPlus
                                                          per
   "Reverse heat pump performance data"
   annotation (Placement(transformation(extent={{-90,76},{-70,96}})));

  parameter Real a[:]={0.9} "Coefficients for efficiency curve";
  parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant
    "Curve used to compute the efficiency";
  parameter Modelica.Units.SI.Temperature T_nominal=353.15
    "Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)"
    annotation (Dialog(enable=(effCur == Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear)));

  parameter Buildings.Fluid.Data.Fuels.Generic fue "Fuel type"
    annotation (choicesAllMatching=true);

  parameter Modelica.Units.SI.Power Q_flow_nominal "Nominal heating power";
  parameter Boolean linearizeFlowResistance=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(enable=computeFlowResistance,
       tab="Flow resistance"));
  parameter Modelica.Units.SI.PressureDifference dp_nominal(min=0, displayUnit=
"Pa") "Pressure difference" annotation (Dialog(group="Nominal condition"));
parameter Modelica.Units.SI.Pressure dp[:]=(3000 + 2000)*{2,1} "Pressure";
parameter Real V_flow[:] = 0.001/1000*{0.5,1};
  parameter Real deltaM=0.1
    "Fraction of nominal flow rate where flow transitions to laminar";
parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_rate_boiler;
parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_radiator_loop;
  parameter Boolean show_T=false;

  parameter Modelica.Units.SI.Volume VTan "Tank volume";
  parameter Modelica.Units.SI.Length hTan "Height of tank (without insulation)";
  parameter Modelica.Units.SI.Length dIns "Thickness of insulation";
  parameter Modelica.Units.SI.ThermalConductivity kIns=0.04
    "Specific heat conductivity of insulation";
  parameter Integer nSeg(min=2) = 2 "Number of volume segments";



  parameter Modelica.Units.SI.MassFlowRate mSou_flow_nominal=per.hea.mSou_flow
    "Source heat exchanger nominal mass flow rate";
  parameter Modelica.Units.SI.MassFlowRate mLoa_flow_nominal=per.hea.mLoa_flow
    "Load heat exchanger nominal mass flow rate";
  Buildings.Fluid.HeatPumps.EquationFitReversible heaPum(
    redeclare package Medium1 = MediumW,
    redeclare package Medium2 = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T1_start=281.4,
    per=per)
   "Water to Water heat pump"
   annotation (Placement(transformation(extent={{6,-8},{26,12}})));
  Buildings.Fluid.Sources.MassFlowSource_T
                           souPum(
    redeclare package Medium = MediumW,
    m_flow=mSou_flow_nominal,
    nPorts=1,
    use_T_in=true)
   "Source side water pump"
   annotation (Placement(transformation(
      extent={{-10,-10},{10,10}},
      rotation=180,
      origin={52,-62})));
  Modelica.Fluid.Sources.FixedBoundary loaVol(redeclare package Medium =
        MediumW, nPorts=1)
   "Volume for the load side"
   annotation (Placement(transformation(extent={{-30,-46},{-50,-26}})));
  Modelica.Fluid.Sources.FixedBoundary souVol(redeclare package Medium =
        MediumW, nPorts=1)
   "Volume for source side"
   annotation (Placement(transformation(extent={{-50,-74},{-30,-54}})));
  Modelica.Blocks.Sources.IntegerExpression modus(y=modusValue)
    annotation (Placement(transformation(extent={{-8,58},{12,78}})));
  Modelica.Blocks.Sources.RealExpression SupSetTemp(y=TSet)
    annotation (Placement(transformation(extent={{-8,76},{12,96}})));
  Modelica.Blocks.Sources.RealExpression souSetTemp(y=TSouSet)
    annotation (Placement(transformation(extent={{-60,-106},{-40,-86}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort retTemp(redeclare package Medium =
        MediumW, m_flow_nominal=0.3)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort supTemp(redeclare package Medium =
        MediumW, m_flow_nominal=0.3)
    annotation (Placement(transformation(extent={{52,-2},{72,18}})));
  Buildings.Fluid.Storage.StratifiedEnhanced storage(
    redeclare package Medium = MediumW,
    m_flow_nominal=nominal_mass_flow_radiator_loop,
    VTan=VTan,
    hTan=hTan,
    dIns=dIns,
    nSeg=nSeg) if useStorageTank
    annotation (Placement(transformation(extent={{-90,20},{-50,60}})));
          Modelica.Blocks.Routing.RealPassThrough pumpDemandSignal
    annotation (Placement(transformation(extent={{-86,-120},{-60,-94}})));
equation
  connect(souPum.ports[1],heaPum. port_a2)
    annotation (Line(points={{42,-62},{32,-62},{32,-4},{26,-4}},
                                             color={0,127,255}));
  connect(heaPum.port_b2,souVol. ports[1])
    annotation (Line(points={{6,-4},{-24,-4},{-24,-64},{-30,-64}},
                                                               color={0,127,255}));
  connect(modus.y, heaPum.uMod) annotation (Line(points={{13,68},{14,68},{14,46},
          {-2,46},{-2,2},{5,2}},
                     color={255,127,0}));
  connect(SupSetTemp.y, heaPum.TSet) annotation (Line(points={{13,86},{18,86},{18,
          16},{0,16},{0,11},{4.6,11}},
                         color={0,0,127}));
  connect(souSetTemp.y, souPum.T_in) annotation (Line(points={{-39,-96},{74,-96},
          {74,-66},{64,-66}}, color={0,0,127}));

  connect(retTemp.port_b, heaPum.port_a1)
    annotation (Line(points={{-40,0},{0,0},{0,8},{6,8}}, color={0,127,255}));
  connect(heaPum.port_b1,supTemp. port_a) annotation (Line(points={{26,8},{52,8}},
                            color={0,127,255}));
  connect(supTemp.port_b, port_b)
    annotation (Line(points={{72,8},{86,8},{86,0},{100,0}},
                                                        color={0,127,255}));
  connect(loaVol.ports[1], retTemp.port_b) annotation (Line(points={{-50,-36},{-54,
          -36},{-54,-18},{-36,-18},{-36,0},{-40,0}}, color={0,127,255}));


          if useStorageTank then
  connect(port_a, storage.port_a) annotation (Line(points={{-100,0},{-86,0},{-86,
          14},{-94,14},{-94,66},{-70,66},{-70,60}}, color={0,127,255}));
  connect(storage.port_b, retTemp.port_a)
    annotation (Line(points={{-70,20},{-70,0},{-60,0}}, color={0,127,255}));

          else
  connect(port_a, retTemp.port_a)
    annotation (Line(points={{-100,0},{-60,0}}, color={0,127,255}));
 end if;
  annotation (Icon(coordinateSystem(extent={{-100,-120},{100,100}}), graphics={
Rectangle(fillPattern=FillPattern.Solid, extent={{-80,80},{80,-80}}),
Rectangle(
  fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          extent={{-68,70},{70,-70}}),
        Polygon(
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          points={{-68,18},{-68,18},{-54,32},{-28,16},{0,30},{26,16},{46,32},{70,
              18},{70,18},{70,-70},{70,-70},{-68,-70},{-68,-70},{-68,18}},
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(extent={{-100,-120},
            {100,100}}), graphics={
                              Line(points={{-18,-52}},color={28,108,200})}));
    end PartialWaterWaterHeatPump;
    partial model PartialAirWaterHeatPump
replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
"Medium model" annotation (choicesAllMatching=true);
  extends
      Buildings.Fluid.Interfaces.PartialTwoPort(
                                    redeclare package Medium = MediumW);

  parameter Boolean useStorageTank=false "Use storage tank"
annotation (Dialog(tab="Storage tank", group="Properties"));
  parameter Real a[:]={0.9} "Coefficients for efficiency curve";
  parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant
    "Curve used to compute the efficiency";
  parameter Modelica.Units.SI.Temperature T_nominal=353.15
    "Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)"
    annotation (Dialog(enable=(effCur == Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear)))
    ;

  parameter Buildings.Fluid.Data.Fuels.Generic fue "Fuel type"
    annotation (choicesAllMatching=true);

  parameter Modelica.Units.SI.Power Q_flow_nominal "Nominal heating power";
  parameter Boolean linearizeFlowResistance=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(enable=computeFlowResistance, tab="Flow resistance"));
  parameter Modelica.Units.SI.PressureDifference dp_nominal(min=0, displayUnit="Pa")
"Pressure difference" annotation (Dialog(group="Nominal condition"));
  parameter Modelica.Units.SI.Pressure dp[:]=(3000 + 2000)*{2,1} "Pressure";
  parameter Real V_flow[:]=0.001/1000*{0.5,1};
  parameter Real deltaM=0.1
    "Fraction of nominal flow rate where flow transitions to laminar";
  parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_rate_boiler;
  parameter Modelica.Units.SI.MassFlowRate nominal_mass_flow_radiator_loop;
  parameter Boolean show_T=false;

  parameter Modelica.Units.SI.Volume VTan "Tank volume";
  parameter Modelica.Units.SI.Length hTan "Height of tank (without insulation)";
  parameter Modelica.Units.SI.Length dIns "Thickness of insulation";
  parameter Modelica.Units.SI.ThermalConductivity kIns=0.04
    "Specific heat conductivity of insulation";
  parameter Integer nSeg(min=2) = 2 "Number of volume segments";

  parameter Modelica.Units.SI.Temperature TSet=323.15;
  parameter Modelica.Units.SI.Temperature TSouSet=278.15;
  parameter Integer modusValue=1;

  parameter Buildings.Fluid.HeatPumps.Data.EquationFitReversible.EnergyPlus
                                                  per
"Reverse heat pump performance data"
annotation (Placement(transformation(extent={{-90,76},{-70,96}})));
  parameter Modelica.Units.SI.MassFlowRate mSou_flow_nominal=per.hea.mSou_flow
    "Source heat exchanger nominal mass flow rate";
  parameter Modelica.Units.SI.MassFlowRate mLoa_flow_nominal=per.hea.mLoa_flow
    "Load heat exchanger nominal mass flow rate";
  Buildings.Fluid.Sources.MassFlowSource_T
                   souPum(
redeclare package Medium = Buildings.Media.Air,
m_flow=1.7,
    use_T_in=true,
nPorts=1) "Source side water pump" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={52,-62})));
  Modelica.Fluid.Sources.FixedBoundary loaVol(redeclare package Medium =
MediumW, nPorts=1) "Volume for the load side"
annotation (Placement(transformation(extent={{-30,-46},{-50,-26}})));
  Modelica.Fluid.Sources.FixedBoundary souVol(redeclare package Medium =
    Buildings.Media.Air, nPorts=1) "Volume for source side"
annotation (Placement(transformation(extent={{-50,-74},{-30,-54}})));
  Modelica.Blocks.Sources.BooleanExpression modus(y=true)
    annotation (Placement(transformation(extent={{-8,58},{12,78}})));
  Modelica.Blocks.Sources.RealExpression SupSetTemp(y=TSet)
    annotation (Placement(transformation(extent={{-8,76},{12,96}})));
  Modelica.Blocks.Sources.RealExpression souSetTemp(y=TSouSet)
    annotation (Placement(transformation(extent={{-60,-106},{-40,-86}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort retTemp(redeclare package Medium
      =
MediumW, m_flow_nominal=0.3)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort supTemp(redeclare package Medium
      =
MediumW, m_flow_nominal=0.3)
    annotation (Placement(transformation(extent={{52,-2},{72,18}})));
  Buildings.Fluid.Storage.StratifiedEnhanced storage(
    redeclare package Medium = MediumW,
    m_flow_nominal=nominal_mass_flow_radiator_loop,
    VTan=VTan,
    hTan=hTan,
    dIns=dIns,
    nSeg=nSeg) if useStorageTank
    annotation (Placement(transformation(extent={{-90,20},{-50,60}})));
  Buildings.Fluid.HeatPumps.ModularReversible.AirToWaterTableData2D
airToWaterTableData2D(
    redeclare package MediumCon = MediumW,
use_intSafCtr=false,
QHea_flow_nominal=Q_flow_nominal,
TConHea_nominal=TSet,
    TEvaHea_nominal=293.15,
    TConCoo_nominal=303.15,
    TEvaCoo_nominal=283.15,
redeclare
  Buildings.Fluid.HeatPumps.ModularReversible.Data.TableData2D.EN14511.Vitocal251A08
  datTabHea,
redeclare
  Buildings.Fluid.Chillers.ModularReversible.Data.TableData2D.EN14511.Vitocal251A08
  datTabCoo)
annotation (Placement(transformation(extent={{-14,-12},{24,26}})));
  IDEAS.Controls.Continuous.LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.P,
  k=5) annotation (Placement(
transformation(extent={{50,50},{70,70}})));
          Modelica.Blocks.Routing.RealPassThrough pumpDemandSignal
    annotation (Placement(transformation(extent={{-86,-120},{-60,-94}})));
equation
  connect(
  souSetTemp.y, souPum.T_in) annotation (Line(points={{-39,-96},{74,-96},
  {74,-66},{64,-66}}, color={0,0,127}));

  connect(
  supTemp.port_b, port_b) annotation (Line(points={{72,8},{86,8},{86,0},{100,0}},
                                                color={0,127,255}));
  connect(
  loaVol.ports[1], retTemp.port_b) annotation (Line(points={{-50,-36},{-54,
  -36},{-54,-18},{-36,-18},{-36,0},{-40,0}}, color={0,127,255}));

  if useStorageTank then
connect(
  port_a, storage.port_a) annotation (Line(points={{-100,0},{-86,0},{-86,
  14},{-94,14},{-94,66},{-70,66},{-70,60}}, color={0,127,255}));
connect(
  storage.port_b, retTemp.port_a)
      annotation (Line(points={{-70,20},{-70,0},{-60,0}}, color={0,127,255}));

  else
connect(
  port_a, retTemp.port_a)
      annotation (Line(points={{-100,0},{-60,0}}, color={0,127,255}));
  end if;
  connect(
      retTemp.port_b, airToWaterTableData2D.port_a1) annotation (Line(
    points={{-40,0},{-20,0},{-20,18.4},{-14,18.4}}, color={0,127,255}));
  connect(
      airToWaterTableData2D.port_b1, supTemp.port_a) annotation (Line(
    points={{24,18.4},{46,18.4},{46,8},{52,8}}, color={0,127,255}));
  connect(
      souPum.ports[1], airToWaterTableData2D.port_a2) annotation (Line(
    points={{42,-62},{30,-62},{30,-4.4},{24,-4.4}}, color={0,127,255}));
  connect(
      airToWaterTableData2D.port_b2, souVol.ports[1]) annotation (Line(
    points={{-14,-4.4},{-24,-4.4},{-24,-64},{-30,-64}}, color={0,127,255}));
  connect(
      modus.y, airToWaterTableData2D.hea) annotation (Line(points={{13,68},{
      10,68},{10,44},{-28,44},{-28,3.01},{-16.09,3.01}}, color={255,0,255}));
  connect(
      supTemp.T, conPID.u_m)
annotation (Line(points={{62,19},{62,48},{60,48}}, color={0,0,127}));
  connect(
      conPID.y, airToWaterTableData2D.ySet) annotation (Line(points={{71,60},
      {76,60},{76,38},{-18,38},{-18,10.61},{-16.09,10.61}}, color={0,0,127}));
  connect(
      SupSetTemp.y, conPID.u_s) annotation (Line(points={{13,86},{40,86},{40,
      60},{48,60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(extent={{-100,-120},{100,100}}), graphics={
    Rectangle(fillPattern=FillPattern.Solid, extent={{-80,80},{80,-80}}),
    Rectangle(
      fillColor={255,255,255},
  fillPattern=FillPattern.Solid,
  extent={{-68,70},{70,-70}}),
Polygon(
  lineColor={0,0,255},
  fillColor={0,0,255},
  fillPattern=FillPattern.Solid,
  points={{-68,18},{-68,18},{-54,32},{-28,16},{0,30},{26,16},{46,32},{70,
      18},{70,18},{70,-70},{70,-70},{-68,-70},{-68,-70},{-68,18}},
  smooth=Smooth.Bezier)}), Diagram(coordinateSystem(extent={{-100,-120},
    {100,100}}), graphics={
                      Line(points={{-18,-52}},color={28,108,200})}));
end PartialAirWaterHeatPump;

  end Boilers;

    package Ventilation

      model SimpleHVACBuildings

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
            "Medium model" annotation (choicesAllMatching=true);
        Buildings.Fluid.Movers.FlowControlled_dp
                                 fanSup(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          inputType=Buildings.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=2*100*1.2/3600) "Supply fan"
          annotation (Placement(transformation(extent={{4,6},{24,26}})));
        Buildings.Fluid.Movers.FlowControlled_dp
                                 fanRet(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          inputType=Buildings.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=2*100*1.2/3600) "Return fan"
          annotation (Placement(transformation(extent={{24,-34},{4,-14}})));
        Buildings.Fluid.HeatExchangers.ConstantEffectiveness
                                             hex(
          redeclare package Medium1 = Medium,
          redeclare package Medium2 = Medium,
          m1_flow_nominal=2*100*1.2/3600,
          m2_flow_nominal=2*100*1.2/3600,
          dp1_nominal=100,
          dp2_nominal=100)
          "Heat exchanger with constant heat recovery effectivity"
          annotation (Placement(transformation(extent={{-26,-14},{-6,6}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare final package
            Medium = Medium)
          "Fluid connector b (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{118,1},{86,31}}),
        iconTransformation(extent={{110,31},{90,49}})));

        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package
            Medium = Medium)
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{84,-40},{118,-8}}),
        iconTransformation(extent={{90,-49},{110,-31}})));
        Buildings.Fluid.Sources.Boundary_pT bou(
          T=295.15,
          nPorts=2,
          redeclare package Medium =                                                        Medium)
          annotation (Placement(transformation(extent={{-78,-14},{-58,6}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TSup(
          redeclare package Medium = Medium,
          m_flow_nominal=2*100*1.2/3600,
          allowFlowReversal=false)
          annotation (Placement(transformation(extent={{48,6},{68,26}})));
        Controls.BaseClasses.DataBus dataBus annotation (Placement(
              transformation(
          extent={{-120,22},{-80,62}}), iconTransformation(extent={{-102,36},{-82,
                  56}})));
      equation
        connect(
          hex.port_b1, fanSup.port_a)
          annotation (Line(points={{-6,2},{-6,16},{4,16}}, color={0,127,255}));
        connect(
          hex.port_a2, fanRet.port_b) annotation (Line(points={{-6,-10},{-6,-24},
                {4,-24}}, color={0,127,255}));
        connect(
          fanRet.port_a, port_a)
          annotation (Line(points={{24,-24},{101,-24}}, color={0,127,255}));
        connect(
          bou.ports[1], hex.port_b2) annotation (Line(points={{-58,-2},{-32,-2},
          {-32,-10},{-26,-10}}, color={0,127,255}));
        connect(
          bou.ports[2], hex.port_a1) annotation (Line(points={{-58,-6},{-32,-6},
          {-32,2},{-26,2}}, color={0,127,255}));
        connect(
          fanSup.port_b, TSup.port_a)
          annotation (Line(points={{24,16},{48,16}}, color={0,127,255}));
        connect(
          TSup.port_b, port_b)
          annotation (Line(points={{68,16},{102,16}}, color={0,127,255}));
        connect(
          TSup.T, dataBus.TSupAhu) annotation (Line(points={{58,27},{58,42},{
          -100,42}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -60},
            {100,60}}), graphics={Rectangle(
          extent={{-102,80},{102,-82}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward),
              Ellipse(
                extent={{-58,66},{72,-66}},
                lineColor={28,108,200},
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-46,54},{60,-54}},
                lineColor={28,108,200},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{35,35},{-1,13},{1,-15},{-35,-35},{35,35}},
                lineColor={28,108,200},
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward,
                origin={7,-1},
                rotation=-90),
              Polygon(
                points={{42,34},{6,12},{8,-16},{-28,-36},{42,34}},
                lineColor={28,108,200},
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward)}),
                                              Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-60},{100,60}})));
      end SimpleHVACBuildings;

        partial model PartialAhu
          replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);

        constant Integer numZon(min=2) = 2 "Total number of served VAV boxes";

        parameter Modelica.Units.SI.Volume VRoo[numZon] "Room volume per zone";
        parameter Modelica.Units.SI.Area AFlo[numZon] "Floor area per zone";

        final parameter Modelica.Units.SI.Area ATot=sum(AFlo)
          "Total floor area for all zone";

        constant Real conv=1.2/3600
          "Conversion factor for nominal mass flow rate";

        parameter Modelica.Units.SI.HeatFlowRate QHeaAHU_flow_nominal(min=0)=
          mHeaAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir
          *                                                                 (
          THeaAirSup_nominal - THeaAirMix_nominal)
          "Nominal heating heat flow rate of air handler unit coil";

        parameter Modelica.Units.SI.HeatFlowRate QCooAHU_flow_nominal(max=0) = 1.3
          *mCooAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir
          *                                                                 (
          TCooAirSup_nominal - TCooAirMix_nominal)
          "Nominal total cooling heat flow rate of air handler unit coil (negative number)";

        parameter Modelica.Units.SI.MassFlowRate mCooVAV_flow_nominal[numZon]
          "Design mass flow rate per zone for cooling"
          annotation (Dialog(group="Nominal mass flow rate"));

        parameter Modelica.Units.SI.MassFlowRate mHeaVAV_flow_nominal[numZon]=0.3
            *mCooVAV_flow_nominal "Design mass flow rate per zone for heating"
          annotation (Dialog(group="Nominal mass flow rate"));

        parameter Modelica.Units.SI.MassFlowRate mAir_flow_nominal=0.01
          "Nominal mass flow rate for fan"
          annotation (Dialog(group="Nominal mass flow rate"));
        parameter Modelica.Units.SI.MassFlowRate mCooAir_flow_nominal=0.7*sum(
            mCooVAV_flow_nominal) "Nominal mass flow rate for fan"
          annotation (Dialog(group="Nominal mass flow rate"));
        parameter Modelica.Units.SI.MassFlowRate mHeaAir_flow_nominal=0.7*sum(
            mHeaVAV_flow_nominal) "Nominal mass flow rate for fan"
          annotation (Dialog(group="Nominal mass flow rate"));

        parameter Modelica.Units.SI.MassFlowRate mHeaWat_flow_nominal=
            QHeaAHU_flow_nominal/Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
            /10 "Nominal water mass flow rate for heating coil in AHU"
          annotation (Dialog(group="Nominal mass flow rate"));
        parameter Modelica.Units.SI.MassFlowRate mCooWat_flow_nominal=
            QCooAHU_flow_nominal/Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
            /(-6) "Nominal water mass flow rate for cooling coil"
          annotation (Dialog(group="Nominal mass flow rate"));

        parameter Real ratOAFlo_A(final unit="m3/(s.m2)") = 0.3e-3
          "Outdoor airflow rate required per unit area";
        parameter Real ratOAFlo_P=2.5e-3
          "Outdoor airflow rate required per person";
        parameter Real ratP_A=5e-2 "Occupant density";
        parameter Real effZ(final unit="1") = 0.8
          "Zone air distribution effectiveness (limiting value)";
        parameter Real divP(final unit="1") = 0.7 "Occupant diversity ratio";

        parameter Modelica.Units.SI.VolumeFlowRate VZonOA_flow_nominal[numZon]=
                                                                     (ratOAFlo_P
            *ratP_A + ratOAFlo_A)*AFlo/effZ
          "Zone outdoor air flow rate of each VAV box";

        parameter Modelica.Units.SI.VolumeFlowRate Vou_flow_nominal=
                                                          (divP*ratOAFlo_P*ratP_A +
            ratOAFlo_A)
                      *sum(AFlo) "System uncorrected outdoor air flow rate";
        parameter Real effVen(final unit="1") = if divP < 0.6 then 0.88*divP + 0.22
           else 0.75 "System ventilation efficiency";
        parameter Modelica.Units.SI.VolumeFlowRate Vot_flow_nominal=
            Vou_flow_nominal/effVen "System design outdoor air flow rate";

        parameter Modelica.Units.SI.Temperature THeaOn=293.15
          "Heating setpoint during on"
          annotation (Dialog(group="Room temperature setpoints"));
        parameter Modelica.Units.SI.Temperature THeaOff=285.15
          "Heating setpoint during off"
          annotation (Dialog(group="Room temperature setpoints"));
        parameter Modelica.Units.SI.Temperature TCooOn=297.15
          "Cooling setpoint during on"
          annotation (Dialog(group="Room temperature setpoints"));
        parameter Modelica.Units.SI.Temperature TCooOff=303.15
          "Cooling setpoint during off"
          annotation (Dialog(group="Room temperature setpoints"));
        parameter Modelica.Units.SI.PressureDifference dpBuiStaSet(min=0) = 12
          "Building static pressure";
        parameter Real yFanMin=0.1 "Minimum fan speed";

        parameter Modelica.Units.SI.Temperature TCooAirMix_nominal(displayUnit="degC")=
             303.15
          "Mixed air temperature during cooling nominal conditions (used to size cooling coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature TCooAirSup_nominal(displayUnit="degC")=
             285.15
          "Supply air temperature during cooling nominal conditions (used to size cooling coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.MassFraction wCooAirMix_nominal=0.017
          "Humidity ratio of mixed air at a nominal conditions used to size cooling coil (in kg/kg dry total)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature TCooWatInl_nominal(displayUnit="degC")=
             279.15 "Cooling coil nominal inlet water temperature" annotation (
            Dialog(group="Air handler unit nominal temperatures and humidity"));

        parameter Modelica.Units.SI.Temperature THeaAirMix_nominal(displayUnit="degC")=
             277.15
          "Mixed air temperature during heating nominal conditions (used to size heating coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature THeaAirSup_nominal(displayUnit="degC")=
             285.15
          "Supply air temperature during heating nominal conditions (used to size heating coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature THeaWatInl_nominal(displayUnit="degC")
          "Reheat coil nominal inlet water temperature" annotation (Dialog(
              group="Air handler unit nominal temperatures and humidity"));

        parameter Boolean allowFlowReversal=false
          "= false to simplify equations, assuming, but not enforcing, no flow reversal"
          annotation (Evaluate=true);

        Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y fanSup(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          dp_nominal=780 + 10 + dpBuiStaSet,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
                                                           "Supply air fan"
          annotation (Placement(transformation(extent={{246,-82},{266,-62}})));
        Buildings.Fluid.Sensors.VolumeFlowRate senSupFlo(redeclare package
            Medium =
             MediumA, m_flow_nominal=mAir_flow_nominal)
          "Sensor for supply fan flow rate"
          annotation (Placement(transformation(extent={{346,-82},{366,-62}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TSup(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          allowFlowReversal=allowFlowReversal)
          annotation (Placement(transformation(extent={{276,-82},{296,-62}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TMix(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          allowFlowReversal=allowFlowReversal,
          transferHeat=true) "Mixed air temperature sensor"
          annotation (Placement(transformation(extent={{82,-82},{102,-62}})));
        Buildings.Fluid.Sensors.VolumeFlowRate VOut1(redeclare package Medium
            = MediumA, m_flow_nominal=mAir_flow_nominal)
          "Outside air volume flow rate"
          annotation (Placement(transformation(extent={{-68,-80},{-48,-60}})));
        Buildings.Fluid.Actuators.Dampers.Exponential
                                  damRet(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          from_dp=false,
          dpDamper_nominal=5,
          dpFixed_nominal=5)
                   "Return air damper" annotation (Placement(transformation(
              origin={52,-6},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Buildings.Fluid.Actuators.Dampers.Exponential
                                  damOut(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          from_dp=false,
          dpDamper_nominal=5,
          dpFixed_nominal=5)
                   "Outdoor air damper"
          annotation (Placement(transformation(extent={{2,-80},{22,-60}})));
        Buildings.Fluid.FixedResistances.PressureDrop
                                  dpSupDuc(
          m_flow_nominal=mAir_flow_nominal,
          redeclare package Medium = MediumA,
          allowFlowReversal=allowFlowReversal,
          dp_nominal=200 + 200 + 100 + 40)
                                 "Pressure drop for supply duct"
          annotation (Placement(transformation(extent={{196,-82},{216,-62}})));
        Buildings.Fluid.FixedResistances.Junction
                              splRetOut(
          redeclare package Medium = MediumA,
          tau=15,
          m_flow_nominal=mAir_flow_nominal*{1,1,1},
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          dp_nominal(each displayUnit="Pa") = {0,0,0},
          portFlowDirection_1=if allowFlowReversal then Modelica.Fluid.Types.PortFlowDirection.Bidirectional
               else Modelica.Fluid.Types.PortFlowDirection.Entering,
          portFlowDirection_2=if allowFlowReversal then Modelica.Fluid.Types.PortFlowDirection.Bidirectional
               else Modelica.Fluid.Types.PortFlowDirection.Leaving,
          portFlowDirection_3=if allowFlowReversal then Modelica.Fluid.Types.PortFlowDirection.Bidirectional
               else Modelica.Fluid.Types.PortFlowDirection.Entering,
          linearized=true) "Flow splitter" annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=0,
              origin={52,-70})));
        Buildings.Fluid.Actuators.Dampers.Exponential damExh(
          from_dp=false,
          dpFixed_nominal=5,
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          dpDamper_nominal=5)
                    "Exhaust air damper"
          annotation (Placement(transformation(extent={{-16,-14},{-36,6}})));
        Buildings.Fluid.FixedResistances.PressureDrop dpRetDuc(
          m_flow_nominal=mAir_flow_nominal,
          redeclare package Medium = MediumA,
          allowFlowReversal=allowFlowReversal,
          dp_nominal=40)
               "Pressure drop for return duct"
          annotation (Placement(transformation(extent={{368,0},{348,20}})));
        Buildings.Fluid.Sensors.VolumeFlowRate senRetFlo(redeclare package
            Medium =
             MediumA, m_flow_nominal=mAir_flow_nominal)
          "Sensor for return fan flow rate"
          annotation (Placement(transformation(extent={{234,0},{214,20}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TRet(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          allowFlowReversal=allowFlowReversal)
                                     "Return air temperature sensor"
          annotation (Placement(transformation(extent={{138,0},{118,20}})));
        Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y fanSup1(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          dp_nominal=780 + 10 + dpBuiStaSet,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
                                                           "Supply air fan"
          annotation (Placement(transformation(extent={{278,0},{258,20}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare final package
            Medium = MediumA)
          "Fluid connector b (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{556,-5},{524,25}}),
              iconTransformation(extent={{562,16},{520,61}})));
        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package
            Medium = MediumA)
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{522,-90},{556,-58}}),
              iconTransformation(extent={{522,-96},{564,-53}})));
        Modelica.Fluid.Interfaces.FluidPorts_b ports[2](redeclare each package
            Medium =
           MediumA, each m_flow(max=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Leaving then 0
                 else +Modelica.Constants.inf, min=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Entering
                 then 0 else -Modelica.Constants.inf)) "Fluid ports"
          annotation (Placement(transformation(extent={{-110,54},{-76,-100}}),
              iconTransformation(extent={{-110,54},{-76,-100}})));
        Buildings.Fluid.Sensors.RelativePressure dpDisSupFan(redeclare package
            Medium = MediumA) "Supply fan static discharge pressure"
          annotation (Placement(transformation(
              extent={{-18,22},{18,-22}},
              rotation=90,
              origin={404,-28})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TOut(
          redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal,
          allowFlowReversal=allowFlowReversal,
          transferHeat=true) "Mixed air temperature sensor"
          annotation (Placement(transformation(extent={{-32,-80},{-12,-60}})));

      protected
        parameter Modelica.Fluid.Types.PortFlowDirection flowDirection=Modelica.Fluid.Types.PortFlowDirection.Bidirectional
          "Allowed flow direction"
          annotation (Evaluate=true, Dialog(tab="Advanced"));
        equation
        connect(TSup.port_a, fanSup.port_b)
                                  annotation (Line(
            points={{276,-72},{266,-72}},
            color={0,127,255},
            smooth=Smooth.None,
            thickness=0.5));
        connect(TSup.port_b, senSupFlo.port_a)
          annotation (Line(points={{296,-72},{346,-72}}, color={0,127,255}));
        connect(dpSupDuc.port_b, fanSup.port_a)
          annotation (Line(points={{216,-72},{246,-72}}, color={0,127,255}));
        connect(damOut.port_b, splRetOut.port_1)
          annotation (Line(points={{22,-70},{42,-70}}, color={0,127,255}));
        connect(splRetOut.port_2, TMix.port_a) annotation (Line(points={{62,-70},
                {72,-70},{72,-72},{82,-72}}, color={0,127,255}));
        connect(damRet.port_b, splRetOut.port_3)
                                       annotation (Line(points={{52,-16},{52,-60}},
              color={0,127,255}));
        connect(
        dpSupDuc.port_a, TMix.port_b)
          annotation (Line(points={{196,-72},{102,-72}}, color={0,127,255}));
        connect(senRetFlo.port_b, TRet.port_a)
                                     annotation (Line(points={{214,10},{138,10}},
              color={0,127,255}));
        connect(
        TRet.port_b, damRet.port_a) annotation (Line(points={{118,10},{52,10},{52,
                4}}, color={0,127,255}));
        connect(
        TRet.port_b, damExh.port_a) annotation (Line(points={{118,10},{-6,10},{-6,
                -4},{-16,-4}}, color={0,127,255}));
        connect(
        senRetFlo.port_a, fanSup1.port_b)
          annotation (Line(points={{234,10},{258,10}}, color={0,127,255}));
        connect(
        fanSup1.port_a, dpRetDuc.port_b)
          annotation (Line(points={{278,10},{348,10}}, color={0,127,255}));
        connect(senSupFlo.port_b, port_a)
                                annotation (Line(points={{366,-72},{516,-72},{516,-74},
                {539,-74}}, color={0,127,255}));
        connect(dpRetDuc.port_a, port_b)
          annotation (Line(points={{368,10},{540,10}}, color={0,127,255}));
        connect(damExh.port_b, ports[1])
                               annotation (Line(points={{-36,-4},{-84,-4},{-84,15.5},
                {-93,15.5}},
                     color={0,127,255}));
        connect(VOut1.port_a, ports[2])
                              annotation (Line(points={{-68,-70},{-84,-70},{-84,
                -61.5},{-93,-61.5}},
                            color={0,127,255}));
        connect(dpDisSupFan.port_a, port_a)
                                  annotation (Line(points={{404,-46},{404,-72},{516,-72},
                {516,-74},{539,-74}}, color={0,127,255}));
        connect(VOut1.port_a, dpDisSupFan.port_b)
                                        annotation (Line(points={{-68,-70},{-84,-70},{
                -84,-26},{376,-26},{376,0},{404,0},{404,-10}}, color={0,127,255}));
        connect(VOut1.port_b, TOut.port_a)
          annotation (Line(points={{-48,-70},{-32,-70}}, color={0,127,255}));
        connect(TOut.port_b, damOut.port_a)
          annotation (Line(points={{-12,-70},{2,-70}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -180},
        {540,100}}), graphics={Rectangle(
                extent={{-100,128},{540,-194}},
                lineColor={215,215,215},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{180,42},{310,-90}},
                  lineColor={28,108,200},
                  fillColor={244,125,35},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{192,32},{298,-76}},
                  lineColor={28,108,200},
                  fillColor={215,215,215},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{280,10},{244,-12},{246,-40},{210,-60},{280,10}},
                  lineColor={28,108,200},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{35,35},{-1,13},{1,-15},{-35,-35},{35,35}},
                  lineColor={28,108,200},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Forward,
                  origin={245,-25},
                  rotation=-90)}),   Diagram(coordinateSystem(
        preserveAspectRatio=false, extent={{-100,-180},{540,100}})));
        end PartialAhu;

      partial model PartialPump
         replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
             "Medium model for air" annotation (choicesAllMatching=true);

       parameter Modelica.Units.SI.MassFlowRate m_flow_nominal
         "Nominal mass flow rate of radiator loop";
       parameter Modelica.Units.SI.PressureDifference dp_nominal(displayUnit="Pa")
         "Pressure difference of loop";

       Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y pumRad(
         redeclare package Medium = Medium,
         m_flow_nominal=m_flow_nominal,
         dp_nominal=dp_nominal,
         energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
         "Pump that serves the radiators" annotation (Placement(transformation(
             extent={{-10,-10},{10,10}},
             rotation=0,
             origin={-4,0})));
       Buildings.Fluid.Sensors.RelativePressure dpSen(redeclare package Medium =
             Medium)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=180,
             origin={-6,32})));
       Buildings.Fluid.Sensors.TemperatureTwoPort temSup(redeclare package
            Medium =
             Medium, m_flow_nominal=m_flow_nominal)
         annotation (Placement(transformation(extent={{-10,-10},{10,10}},
             rotation=0,
             origin={42,0})));
       Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter
                                                  gain(k=1/dp_nominal)
         "Gain used to normalize pressure measurement signal"
         annotation (Placement(transformation(extent={{10,-10},{-10,10}},
             rotation=-90,
             origin={-6,76})));
             extends Buildings.Fluid.Interfaces.PartialTwoPort;
      equation
       connect(pumRad.port_b,dpSen. port_a)
                                          annotation (Line(
           points={{6,0},{14,0},{14,32},{4,32}},
           color={0,127,255},
           smooth=Smooth.None));
       connect(dpSen.port_b,pumRad. port_a)
                                          annotation (Line(
           points={{-16,32},{-22,32},{-22,0},{-14,0}},
           color={0,127,255},
           smooth=Smooth.None));
       connect(pumRad.port_b,temSup. port_a) annotation (Line(
           points={{6,0},{32,0}},
           color={0,127,255},
           smooth=Smooth.None));
       connect(gain.u,dpSen. p_rel) annotation (Line(
           points={{-6,64},{-6,41}},
           color={0,0,127},
           smooth=Smooth.None));
       connect(port_a, pumRad.port_a)
         annotation (Line(points={{-100,0},{-14,0}}, color={0,127,255}));
       connect(temSup.port_b, port_b)
         annotation (Line(points={{52,0},{100,0}}, color={0,127,255}));
       annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
             Rectangle(
               extent={{-100,16},{100,-16}},
               lineColor={0,0,0},
               fillColor={0,127,255},
               fillPattern=FillPattern.HorizontalCylinder),
             Ellipse(
               extent={{-58,58},{58,-58}},
               lineColor={0,0,0},
               fillPattern=FillPattern.Sphere,
               fillColor={0,100,199}),
             Polygon(
               points={{0,50},{0,-50},{54,0},{0,50}},
               lineColor={0,0,0},
               pattern=LinePattern.None,
               fillPattern=FillPattern.HorizontalCylinder,
               fillColor={255,255,255}),
             Ellipse(
               extent={{4,16},{36,-16}},
               lineColor={0,0,0},
               fillPattern=FillPattern.Sphere,
               visible=energyDynamics <> Modelica.Fluid.Types.Dynamics.SteadyState,
               fillColor={0,100,199})}), Diagram(coordinateSystem(
               preserveAspectRatio=false)));
      end PartialPump;

    partial model PartialVAVBox
        "Supply box of a VAV system with a hot water reheat coil"
        extends Modelica.Blocks.Icons.Block;
      replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
            "Medium model for air" annotation (choicesAllMatching=true);

        parameter Boolean allowFlowReversal=true
          "= false to simplify equations, assuming, but not enforcing, no flow reversal";
        parameter Modelica.Units.SI.MassFlowRate mCooAir_flow_nominal
          "Nominal air mass flow rate from cooling sizing calculations";
        parameter Modelica.Units.SI.MassFlowRate mHeaAir_flow_nominal
          "Nominal air mass flow rate from heating sizing calculations";
        final parameter Modelica.Units.SI.MassFlowRate mHeaWat_flow_nominal=
            QHea_flow_nominal/(cpWatLiq*(THeaWatInl_nominal -
            THeaWatOut_nominal))
          "Nominal mass flow rate of hot water to reheat coil";
        parameter Modelica.Units.SI.Volume VRoo "Room volume";
        parameter Modelica.Units.SI.Temperature THeaWatInl_nominal(start=55 + 273.15,
            displayUnit="degC") "Reheat coil nominal inlet water temperature";
        parameter Modelica.Units.SI.Temperature THeaWatOut_nominal(start=
    THeaWatInl_nominal - 10, displayUnit="degC")
          "Reheat coil nominal outlet water temperature";
        parameter Modelica.Units.SI.Temperature THeaAirInl_nominal(start=12 + 273.15,
            displayUnit="degC")
          "Inlet air nominal temperature into VAV box during heating";
        parameter Modelica.Units.SI.Temperature THeaAirDis_nominal(start=28 + 273.15,
            displayUnit="degC")
          "Discharge air temperature from VAV box during heating";
        parameter Modelica.Units.SI.HeatFlowRate QHea_flow_nominal=
            mHeaAir_flow_nominal*cpAir*(THeaAirDis_nominal - THeaAirInl_nominal)
          "Nominal heating heat flow rate";
        Modelica.Fluid.Interfaces.FluidPort_a port_aAir(redeclare package
            Medium = MediumA)
          "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
          annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
    iconTransformation(extent={{-10,-110},{10,-90}})));
        Modelica.Fluid.Interfaces.FluidPort_a port_bAir(redeclare package
            Medium = MediumA)
          "Fluid connector b (positive design flow direction is from port_a1 to port_b1)"
          annotation (Placement(transformation(extent={{-10,90},{10,110}}),
    iconTransformation(extent={{-10,90},{10,110}})));
        Buildings.Fluid.Actuators.Dampers.Exponential vav(
          redeclare package Medium = MediumA,
          m_flow_nominal=mCooAir_flow_nominal,
          dpDamper_nominal=20,
          allowFlowReversal=allowFlowReversal,
          dpFixed_nominal=130) "VAV box for room" annotation (Placement(
    transformation(
    extent={{-10,-10},{10,10}},
    rotation=90,
    origin={0,10})));

        Buildings.Fluid.Sensors.TemperatureTwoPort senTem(
          redeclare package Medium = MediumA,
          initType=Modelica.Blocks.Types.Init.InitialState,
          m_flow_nominal=mCooAir_flow_nominal,
          allowFlowReversal=allowFlowReversal) "Supply air temperature sensor"
          annotation (Placement(transformation(
    extent={{-10,10},{10,-10}},
    rotation=90,
    origin={0,40})));
        Buildings.Fluid.Sensors.VolumeFlowRate senVolFlo(
          redeclare package Medium = MediumA,
          initType=Modelica.Blocks.Types.Init.InitialState,
          m_flow_nominal=mCooAir_flow_nominal,
          allowFlowReversal=allowFlowReversal)
          "Supply air volumetric flow rate sensor" annotation (Placement(
    transformation(
    extent={{-10,10},{10,-10}},
    rotation=90,
    origin={0,80})));
      protected
        constant Modelica.Units.SI.SpecificHeatCapacity cpAir=Buildings.Utilities.Psychrometrics.Constants.cpAir
          "Air specific heat capacity";
        constant Modelica.Units.SI.SpecificHeatCapacity cpWatLiq=Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
          "Water specific heat capacity";
    equation
        connect(
      vav.port_b, senTem.port_a) annotation (Line(points={{6.66134e-16,20},{
      0,20},{0,30},{-4.44089e-16,30}}, color={0,127,255}));
        connect(
      senTem.port_b, senVolFlo.port_a) annotation (Line(points={{0,50},{0,70},{-6.66134e-16,
                70}},                    color={0,127,255}));
        connect(
      senVolFlo.port_b, port_bAir) annotation (Line(points={{4.44089e-16,90},{0,
                90},{0,100}},                    color={0,127,255}));
        connect(
      vav.port_a, port_aAir) annotation (Line(points={{-5.55112e-16,0},{0,-100}},
              color={0,127,255}));
        annotation (Icon(graphics={
    Rectangle(
      extent={{-108.07,-16.1286},{93.93,-20.1286}},
      lineColor={0,0,0},
      fillPattern=FillPattern.HorizontalCylinder,
      fillColor={0,127,255},
      origin={-18.1286,6.07},
      rotation=90),
    Rectangle(
      extent={{100.8,-22},{128.8,-44}},
      lineColor={0,0,0},
      fillPattern=FillPattern.HorizontalCylinder,
      fillColor={192,192,192},
      origin={-32,-76.8},
      rotation=90),
    Rectangle(
      extent={{102.2,-11.6667},{130.2,-25.6667}},
      lineColor={0,0,0},
      fillPattern=FillPattern.HorizontalCylinder,
      fillColor={0,127,255},
      origin={-17.6667,-78.2},
      rotation=90),
    Polygon(
      points={{-12,32},{16,48},{16,46},{-12,30},{-12,32}},
      pattern=LinePattern.None,
      smooth=Smooth.None,
      fillColor={0,0,0},
      fillPattern=FillPattern.Solid,
      lineColor={0,0,0}),
    Line(points={{-100,80},{-38,80},{-38,38},{-10,38}}, color={255,255,0},
      thickness=1)}));
    end PartialVAVBox;

    end Ventilation;
  end Fluid;

  package HeatTransfer
  package IdealHeatingSystem
  model IdealHeatEmission
    parameter Real frad=0.3 "radiative fraction";
      parameter Real power=2000 "heating power";
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortCon
      "Heat port for convective heat transfer with room air temperature"
      annotation (Placement(transformation(extent={{-30,62},{-10,82}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortRad
      "Heat port for radiative heat transfer with room radiation temperature"
      annotation (Placement(transformation(extent={{10,62},{30,82}})));
    Modelica.Blocks.Interfaces.RealInput y
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Math.Gain HeatingPower(k=power)
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    Modelica.Blocks.Math.Gain convectiveGain(k=1 - frad)
      annotation (Placement(transformation(extent={{2,-60},{22,-40}})));
    Modelica.Blocks.Math.Gain radiativeGain(k=frad)
      annotation (Placement(transformation(extent={{2,-90},{22,-70}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1, uMin=0)
          annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
      protected
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preSumCon(final
        alpha=0)
      "Heat input into radiator from convective heat transfer"
      annotation (Placement(transformation(extent={{52,-60},{72,-40}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preSumRad(final
        alpha=0)
      "Heat input into radiator from radiative heat transfer"
      annotation (Placement(transformation(extent={{52,-90},{72,-70}})));
  equation

    connect(preSumCon.port,heatPortCon)        annotation (Line(
        points={{72,-50},{80,-50},{80,40},{-20,40},{-20,72}},
        color={191,0,0}));
    connect(preSumRad.port,heatPortRad)         annotation (Line(
        points={{72,-80},{86,-80},{86,50},{20,50},{20,72}},
        color={191,0,0}));
    connect(preSumCon.Q_flow, convectiveGain.y)
      annotation (Line(points={{52,-50},{23,-50}}, color={0,0,127}));
    connect(radiativeGain.y, preSumRad.Q_flow)
      annotation (Line(points={{23,-80},{52,-80}}, color={0,0,127}));
        connect(HeatingPower.y, convectiveGain.u) annotation (Line(points={{-19,
                0},{-6,0},{-6,-50},{0,-50}}, color={0,0,127}));
        connect(HeatingPower.y, radiativeGain.u) annotation (Line(points={{-19,
                0},{-6,0},{-6,-80},{0,-80}}, color={0,0,127}));
        connect(y, limiter.u)
          annotation (Line(points={{-120,0},{-88,0}}, color={0,0,127}));
        connect(limiter.y, HeatingPower.u)
          annotation (Line(points={{-65,0},{-42,0}}, color={0,0,127}));
    annotation (Icon(graphics={
          Ellipse(
            extent={{-20,20},{20,-22}},
            fillColor={127,0,0},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(
            extent={{-20,20},{20,-22}},
            fillColor={127,0,0},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Rectangle(
            extent={{-80,58},{80,-62}},
            lineColor={0,0,0},
            fillColor={127,0,0},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-66,28},{66,28}}),
          Line(
            points={{-66,0},{66,0}}),
          Line(
            points={{-66,-32},{66,-32}}),
          Line(
            points={{-66,58},{-66,-62}}),
          Line(
            points={{66,58},{66,-62}})}));
  end IdealHeatEmission;
  end IdealHeatingSystem;
  end HeatTransfer;

  package BaseClasses
    package Containers
      partial model envelope

        annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={238,46,47},
            fillColor={235,235,235},
            fillPattern=FillPattern.Solid),
              Polygon(
                points={{-56,14},{48,14},{32,42},{-44,42},{-56,14}},
                fillColor={207,138,69},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-52,14},{44,-44}},
                fillColor={90,90,0},
                fillPattern=FillPattern.Forward,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-40,-2},{-16,-20}},
                lineColor={238,46,47},
                fillColor={28,108,200},
                fillPattern=FillPattern.HorizontalCylinder),
              Rectangle(
                extent={{8,-2},{30,-20}},
                lineColor={238,46,47},
                fillColor={28,108,200},
                fillPattern=FillPattern.HorizontalCylinder)}));

      end envelope;

      partial model emission

      annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-100,100},{100,-100}},
            fillColor={215,215,215},
            fillPattern=FillPattern.Forward,
                pattern=LinePattern.None,
                lineColor={215,215,215}),
              Rectangle(
                extent={{-76,60},{78,56}},
                pattern=LinePattern.None,
                fillColor={28,108,200},
                fillPattern=FillPattern.Forward),
              Rectangle(
                extent={{-74,-58},{74,-62}},
                pattern=LinePattern.None,
                fillColor={127,0,0},
                fillPattern=FillPattern.Forward),
              Rectangle(
                extent={{-54,70},{-36,-74}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward),
              Rectangle(
                extent={{-26,70},{-8,-74}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward),
              Rectangle(
                extent={{0,70},{18,-74}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward),
              Rectangle(
                extent={{26,70},{44,-74}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward),
              Line(
                points={{-76,54},{68,52},{70,54},{-190,72},{-178,18},{-238,48},{-198,10},
                    {-182,44}},
                color={0,0,0},
                pattern=LinePattern.None),
              Line(
                points={{-52,88},{16,74}},
                color={0,0,0},
                pattern=LinePattern.None),
              Line(
                points={{-168,66},{-138,28},{-66,78},{0,84},{-6,68},{-4,76},{14,88}},
                color={0,0,0},
                pattern=LinePattern.None),
              Ellipse(
                extent={{48,-50},{66,-68}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward)}));

      end emission;

      model distribution
      annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-100,100},{100,-100}},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Rectangle(
                extent={{-6,22},{50,-22}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={28,108,200},
                fillPattern=FillPattern.HorizontalCylinder),
              Rectangle(
                extent={{-36,70},{10,-76}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={28,108,200},
                fillPattern=FillPattern.VerticalCylinder),
              Rectangle(
                extent={{-42,84},{16,68}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={117,117,117},
                fillPattern=FillPattern.VerticalCylinder),
              Rectangle(
                extent={{-42,-70},{16,-86}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={117,117,117},
                fillPattern=FillPattern.VerticalCylinder),
              Rectangle(
                extent={{46,28},{66,-28}},
                lineColor={0,0,0},
                pattern=LinePattern.None,
                fillColor={117,117,117},
                fillPattern=FillPattern.HorizontalCylinder)}));
      end distribution;

      model production
        annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-100,100},{100,-100}},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-44,-36},{38,-66}},
                lineColor={0,0,0},
                fillColor={238,46,47},
                fillPattern=FillPattern.Sphere),
              Rectangle(
                extent={{-44,44},{38,-52}},
                lineColor={0,0,0},
                fillColor={238,46,47},
                fillPattern=FillPattern.VerticalCylinder),
              Ellipse(
                extent={{-44,60},{38,30}},
                lineColor={0,0,0},
                fillColor={238,46,47},
                fillPattern=FillPattern.Sphere)}));
      end production;

      model bus
        annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-102,100},{100,-100}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Forward),
              Line(points={{-4,56},{-10,60},{-52,0}}, color={215,215,215}),
              Polygon(
                points={{-4,66},{-64,-38},{62,-38},{-4,66}},
                lineColor={255,255,0},
                fillColor={255,255,0},
                fillPattern=FillPattern.Forward),
              Polygon(
                points={{8,34},{-10,12},{4,-4},{-18,-32},{8,34}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward)}));
      end bus;

      model solar
        annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-102,100},{100,-100}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Forward),
              Line(points={{-4,56},{-10,60},{-52,0}}, color={215,215,215}),
              Polygon(
                points={{-76,-48},{-28,67},{82,67},{33,-48},{-76,-48}},
                smooth=Smooth.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward,
                pattern=LinePattern.Dot,
                lineColor={28,108,200}),
              Polygon(
                points={{-65,-41},{-53,-15},{-30,-15},{-41,-41},{-65,-41}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{-49,-5},{-37,21},{-14,21},{-25,-5},{-49,-5}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{-34,29},{-22,55},{1,55},{-10,29},{-34,29}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{-32,-41},{-20,-15},{3,-15},{-8,-41},{-32,-41}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{-16,-5},{-4,21},{19,21},{8,-5},{-16,-5}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{-1,31},{11,57},{34,57},{23,31},{-1,31}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{1,-41},{13,-15},{36,-15},{25,-41},{1,-41}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{17,-5},{29,21},{52,21},{41,-5},{17,-5}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Polygon(
                points={{32,31},{44,57},{67,57},{56,31},{32,31}},
                smooth=Smooth.None,
                fillColor={6,13,150},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None)}));
      end solar;

      model ventilation
        annotation (
      Icon(
        coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-102,100},{100,-100}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Forward),
              Line(points={{-4,56},{-10,60},{-52,0}}, color={215,215,215}),
              Ellipse(
                extent={{-62,66},{68,-66}},
                lineColor={28,108,200},
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,54},{56,-54}},
                lineColor={28,108,200},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{38,34},{2,12},{4,-16},{-32,-36},{38,34}},
                lineColor={28,108,200},
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward),
              Polygon(
                points={{35,35},{-1,13},{1,-15},{-35,-35},{35,35}},
                lineColor={28,108,200},
                fillColor={0,0,0},
                fillPattern=FillPattern.Forward,
                origin={3,-1},
                rotation=-90)}));
      end ventilation;
    end Containers;
  end BaseClasses;
    package ThermalZones
  package BaseClasses
    partial model RoomHeatMassBalanceInf "Base model for a room"
      extends Buildings.ThermalZones.Detailed.BaseClasses.ConstructionRecords;

      replaceable package Medium =
        Modelica.Media.Interfaces.PartialMedium "Medium in the component"
          annotation (choicesAllMatching = true);

      constant Boolean homotopyInitialization = true "= true, use homotopy method"
        annotation(HideResult=true);

      parameter Integer nPorts=0 "Number of ports" annotation (Evaluate=true,
          Dialog(
          connectorSizing=true,
          tab="General",
          group="Ports"));
      Modelica.Fluid.Vessels.BaseClasses.VesselFluidPorts_b ports[nPorts](
          redeclare each package Medium = Medium) "Fluid inlets and outlets"
        annotation (Placement(transformation(
            extent={{-40,-10},{40,10}},
            origin={-260,-60},
            rotation=90), iconTransformation(
            extent={{-40,-10},{40,10}},
            rotation=90,
            origin={-150,-100})));
      final parameter Modelica.Units.SI.Volume V=AFlo*hRoo "Volume";
      parameter Modelica.Units.SI.Area AFlo "Floor area";
      parameter Modelica.Units.SI.Length hRoo "Average room height";
      parameter Real ACH(unit="1/h") = 0.5
      "Air change rate (1/h)"annotation(Dialog(group="Air Infiltration"));

      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorAir
        "Heat port to air volume" annotation (Placement(transformation(extent={{-270,30},
                {-250,50}}),   iconTransformation(extent={{-20,-10},{0,10}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPorRad
        "Heat port for radiative heat gain and radiative temperature" annotation (
          Placement(transformation(extent={{-270,-10},{-250,10}}),
                    iconTransformation(extent={{-20,-48},{0,-28}})));
      ////////////////////////////////////////////////////////////////////////
      // Constructions
      Buildings.ThermalZones.Detailed.Constructions.Construction conExt[NConExt](
        final A=datConExt.A,
        final til=datConExt.til,
        final layers=datConExt.layers,
        final steadyStateInitial=datConExt.steadyStateInitial,
        final T_a_start=datConExt.T_a_start,
        final T_b_start=datConExt.T_b_start,
        final stateAtSurface_a=datConExt.stateAtSurface_a,
        final stateAtSurface_b=datConExt.stateAtSurface_b) if haveConExt
        "Heat conduction through exterior construction that have no window"
        annotation (Placement(transformation(extent={{288,100},{242,146}})));
      Buildings.ThermalZones.Detailed.Constructions.ConstructionWithWindow conExtWin[
        NConExtWin](
        final A=datConExtWin.A,
        final til=datConExtWin.til,
        final layers=datConExtWin.layers,
        final steadyStateInitial=datConExtWin.steadyStateInitial,
        final T_a_start=datConExtWin.T_a_start,
        final T_b_start=datConExtWin.T_b_start,
        final AWin=datConExtWin.AWin,
        final fFra=datConExtWin.fFra,
        final glaSys=datConExtWin.glaSys,
        each final homotopyInitialization=homotopyInitialization,
        each final linearizeRadiation=linearizeRadiation,
        each final steadyStateWindow=steadyStateWindow,
        final stateAtSurface_a=datConExtWin.stateAtSurface_a,
        final stateAtSurface_b=datConExtWin.stateAtSurface_b) if haveConExtWin
        "Heat conduction through exterior construction that have a window"
        annotation (Placement(transformation(extent={{280,44},{250,74}})));

      Buildings.ThermalZones.Detailed.Constructions.Construction conPar[NConPar](
        A=datConPar.A,
        til=datConPar.til,
        final layers=datConPar.layers,
        steadyStateInitial=datConPar.steadyStateInitial,
        T_a_start=datConPar.T_a_start,
        T_b_start=datConPar.T_b_start,
        final stateAtSurface_a=datConPar.stateAtSurface_a,
        final stateAtSurface_b=datConPar.stateAtSurface_b) if haveConPar
        "Heat conduction through partitions that have both sides inside the thermal zone"
        annotation (Placement(transformation(extent={{282,-122},{244,-84}})));

      Buildings.ThermalZones.Detailed.Constructions.Construction conBou[NConBou](
        A=datConBou.A,
        til=datConBou.til,
        final layers=datConBou.layers,
        steadyStateInitial=datConBou.steadyStateInitial,
        T_a_start=datConBou.T_a_start,
        T_b_start=datConBou.T_b_start,
        final stateAtSurface_a=datConBou.stateAtSurface_a,
        final stateAtSurface_b=datConBou.stateAtSurface_b) if haveConBou
        "Heat conduction through opaque constructions that have the boundary conditions of the other side exposed"
        annotation (Placement(transformation(extent={{282,-156},{242,-116}})));
      parameter Boolean linearizeRadiation=true
        "Set to true to linearize emissive power";

      parameter Boolean steadyStateWindow = false
        "Set to false to add thermal capacity at window, which generally leads to faster simulation"
        annotation (Dialog(tab="Dynamics", group="Glazing system"));
      ////////////////////////////////////////////////////////////////////////
      // Convection
      parameter Buildings.HeatTransfer.Types.InteriorConvection intConMod=Buildings.HeatTransfer.Types.InteriorConvection.Temperature
        "Convective heat transfer model for room-facing surfaces of opaque constructions"
        annotation (Dialog(group="Convective heat transfer"));
      parameter Modelica.Units.SI.CoefficientOfHeatTransfer hIntFixed=3.0
        "Constant convection coefficient for room-facing surfaces of opaque constructions"
        annotation (Dialog(group="Convective heat transfer", enable=(intConMod ==
              Buildings.HeatTransfer.Types.InteriorConvection.Fixed)));
      parameter Buildings.HeatTransfer.Types.ExteriorConvection extConMod=Buildings.HeatTransfer.Types.ExteriorConvection.TemperatureWind
        "Convective heat transfer model for exterior facing surfaces of opaque constructions"
        annotation (Dialog(group="Convective heat transfer"));
      parameter Modelica.Units.SI.CoefficientOfHeatTransfer hExtFixed=10.0
        "Constant convection coefficient for exterior facing surfaces of opaque constructions"
        annotation (Dialog(group="Convective heat transfer", enable=(extConMod ==
              Buildings.HeatTransfer.Types.ExteriorConvection.Fixed)));
      parameter Modelica.Units.SI.MassFlowRate m_flow_nominal(min=0) = V*1.2/3600
        "Nominal mass flow rate" annotation (Dialog(group="Nominal condition"));
      parameter Boolean sampleModel = false
        "Set to true to time-sample the model, which can give shorter simulation time if there is already time sampling in the system model"
        annotation (Evaluate=true, Dialog(tab="Experimental (may be changed in future releases)"));
      ////////////////////////////////////////////////////////////////////////
      // Control signals
      Modelica.Blocks.Interfaces.RealInput uWin[nConExtWin](
        each min=0, each max=1, each unit="1") if haveControllableWindow
        "Control signal for window state (used for electrochromic windows, removed otherwise)"
         annotation (Placement(
            transformation(extent={{-20,-20},{20,20}},   origin={-280,140}),
            iconTransformation(
            extent={{-16,-16},{16,16}},
            origin={-216,130})));

      ////////////////////////////////////////////////////////////////////////
      // Models for boundary conditions
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a surf_conBou[nConBou]
     if haveConBou "Heat port at surface b of construction conBou" annotation (
          Placement(transformation(extent={{-270,-190},{-250,-170}}),
            iconTransformation(extent={{50,-170},{70,-150}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a surf_surBou[nSurBou]
     if haveSurBou "Heat port of surface that is connected to the room air"
        annotation (Placement(transformation(extent={{-270,-150},{-250,-130}}),
            iconTransformation(extent={{-48,-150},{-28,-130}})));
      Modelica.Blocks.Interfaces.RealInput qGai_flow[3](each unit="W/m2")
        "Radiant, convective and latent heat input into room (positive if heat gain)"
        annotation (Placement(transformation(extent={{-300,60},{-260,100}}),
            iconTransformation(extent={{-232,64},{-200,96}})));
      // Reassign the tilt since a construction that is declared as a ceiling of the
      // room model has an exterior-facing surface that is a floor
      Buildings.ThermalZones.Detailed.BaseClasses.ExteriorBoundaryConditions bouConExt(
        final nCon=nConExt,
        linearizeRadiation=linearizeRadiation,
        final conMod=extConMod,
        final conPar=datConExt,
        final hFixed=hExtFixed) if haveConExt
        "Exterior boundary conditions for constructions without a window"
        annotation (Placement(transformation(extent={{352,114},{382,144}})));
      // Reassign the tilt since a construction that is declared as a ceiling of the
      // room model has an exterior-facing surface that is a floor
      Buildings.ThermalZones.Detailed.BaseClasses.ExteriorBoundaryConditionsWithWindow
        bouConExtWin(
        final nCon=nConExtWin,
        final conPar=datConExtWin,
        linearizeRadiation=linearizeRadiation,
        final conMod=extConMod,
        final hFixed=hExtFixed) if haveConExtWin
        "Exterior boundary conditions for constructions with a window"
        annotation (Placement(transformation(extent={{352,44},{382,74}})));

      Buildings.HeatTransfer.Windows.BaseClasses.WindowRadiation conExtWinRad[
        NConExtWin](
        final AWin=(1 .- datConExtWin.fFra) .* datConExtWin.AWin,
        final N={size(datConExtWin[i].glaSys.glass, 1) for i in 1:NConExtWin},
        final tauGlaSol=datConExtWin.glaSys.glass.tauSol,
        final rhoGlaSol_a=datConExtWin.glaSys.glass.rhoSol_a,
        final rhoGlaSol_b=datConExtWin.glaSys.glass.rhoSol_b,
        final xGla=datConExtWin.glaSys.glass.x,
        final tauShaSol_a=datConExtWin.glaSys.shade.tauSol_a,
        final tauShaSol_b=datConExtWin.glaSys.shade.tauSol_b,
        final rhoShaSol_a=datConExtWin.glaSys.shade.rhoSol_a,
        final rhoShaSol_b=datConExtWin.glaSys.shade.rhoSol_b,
        final haveExteriorShade=datConExtWin.glaSys.haveExteriorShade,
        final haveInteriorShade=datConExtWin.glaSys.haveInteriorShade)
        if haveConExtWin "Model for solar radiation through shades and window"
        annotation (Placement(transformation(extent={{320,-24},{300,-4}})));

      Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data"
        annotation (Placement(transformation(extent={{170,150},{190,170}}),
            iconTransformation(extent={{166,166},{192,192}})));

      replaceable
        Buildings.ThermalZones.Detailed.BaseClasses.PartialAirHeatMassBalance air
        constrainedby
        Buildings.ThermalZones.Detailed.BaseClasses.PartialAirHeatMassBalance(
        redeclare final package Medium = Medium,
        nPorts=nPorts + 2,
        final nConExt=nConExt,
        final nConExtWin=nConExtWin,
        final nConPar=nConPar,
        final nConBou=nConBou,
        final nSurBou=nSurBou,
        final datConExt=datConExt,
        final datConExtWin=datConExtWin,
        final datConPar=datConPar,
        final datConBou=datConBou,
        final surBou=surBou,
        final haveShade=haveShade,
        final V=V) "Convective heat and mass balance of air"
        annotation (Placement(transformation(extent={{40,-142},{64,-118}})));

      Buildings.ThermalZones.Detailed.BaseClasses.SolarRadiationExchange solRadExc(
        final nConExt=nConExt,
        final nConExtWin=nConExtWin,
        final nConPar=nConPar,
        final nConBou=nConBou,
        final nSurBou=nSurBou,
        final datConExt = datConExt,
        final datConExtWin = datConExtWin,
        final datConPar = datConPar,
        final datConBou = datConBou,
        final surBou = surBou,
        final is_floorConExt=is_floorConExt,
        final is_floorConExtWin=is_floorConExtWin,
        final is_floorConPar_a=is_floorConPar_a,
        final is_floorConPar_b=is_floorConPar_b,
        final is_floorConBou=is_floorConBou,
        final is_floorSurBou=is_floorSurBou,
        final tauGla={datConExtWin[i].glaSys.glass[size(datConExtWin[i].glaSys.glass, 1)].tauSol[1] for i in 1:NConExtWin})
        if haveConExtWin "Solar radiative heat exchange"
        annotation (Placement(transformation(extent={{-100,40},{-80,60}})));

      Buildings.ThermalZones.Detailed.BaseClasses.InfraredRadiationGainDistribution irRadGai(
        final nConExt=nConExt,
        final nConExtWin=nConExtWin,
        final nConPar=nConPar,
        final nConBou=nConBou,
        final nSurBou=nSurBou,
        final datConExt = datConExt,
        final datConExtWin = datConExtWin,
        final datConPar = datConPar,
        final datConBou = datConBou,
        final surBou = surBou,
        final haveShade=haveShade)
        "Distribution for infrared radiative heat gains (e.g., due to equipment and people)"
        annotation (Placement(transformation(extent={{-100,-40},{-80,-20}})));

      Buildings.ThermalZones.Detailed.BaseClasses.InfraredRadiationExchange irRadExc(
        final nConExt=nConExt,
        final nConExtWin=nConExtWin,
        final nConPar=nConPar,
        final nConBou=nConBou,
        final nSurBou=nSurBou,
        final datConExt = datConExt,
        final datConExtWin = datConExtWin,
        final datConPar = datConPar,
        final datConBou = datConBou,
        final surBou = surBou,
        final linearizeRadiation = linearizeRadiation,
        final homotopyInitialization = homotopyInitialization,
        final sampleModel = sampleModel)
        "Infrared radiative heat exchange"
        annotation (Placement(transformation(extent={{-100,0},{-80,20}})));

      Buildings.ThermalZones.Detailed.BaseClasses.RadiationTemperature radTem(
        final nConExt=nConExt,
        final nConExtWin=nConExtWin,
        final nConPar=nConPar,
        final nConBou=nConBou,
        final nSurBou=nSurBou,
        final datConExt=datConExt,
        final datConExtWin=datConExtWin,
        final datConPar=datConPar,
        final datConBou=datConBou,
        final surBou=surBou,
        final haveShade=haveShade) "Radiative temperature of the room"
        annotation (Placement(transformation(extent={{-100,-80},{-80,-60}})));

      Buildings.HeatTransfer.Windows.BaseClasses.ShadeRadiation shaRad[NConExtWin](
        final A=(1 .- datConExtWin.fFra) .* datConExtWin.AWin,
        final thisSideHasShade=haveInteriorShade,
        final absIR_air=datConExtWin.glaSys.shade.absIR_a,
        final absIR_glass={(datConExtWin[i].glaSys.glass[size(datConExtWin[i].glaSys.glass,
            1)].absIR_b) for i in 1:NConExtWin},
        final tauIR_air=tauIRSha_air,
        final tauIR_glass=tauIRSha_glass,
        each final linearize=linearizeRadiation,
        each final homotopyInitialization=homotopyInitialization) if haveShade
        "Radiation model for room-side window shade"
        annotation (Placement(transformation(extent={{-60,90},{-40,110}})));

      Buildings.Fluid.Sources.Boundary_pT souInf(
        redeclare package Medium = Medium,
        use_T_in=true,
        nPorts=1) "Source model for air infiltration"
        annotation (Placement(transformation(extent={{4,-170},{18,-156}})));
      Buildings.Fluid.Sources.MassFlowSource_T sinInf(
        redeclare package Medium = Medium,
        use_m_flow_in=true,
        nPorts=1) "Sink model for air infiltration"
        annotation (Placement(transformation(extent={{2,-194},{20,-176}})));
      Modelica.Blocks.Sources.RealExpression airInfiltration(y=ACH*V*1.2/3600)
        annotation (Placement(transformation(extent={{-60,-188},{-40,-168}})));
    protected
      final parameter Modelica.Units.SI.TransmissionCoefficient tauIRSha_air[
        NConExtWin]=datConExtWin.glaSys.shade.tauIR_a
        "Infrared transmissivity of shade for radiation coming from the exterior or the room"
        annotation (Dialog(group="Shading"));
      final parameter Modelica.Units.SI.TransmissionCoefficient tauIRSha_glass[
        NConExtWin]=datConExtWin.glaSys.shade.tauIR_b
        "Infrared transmissivity of shade for radiation coming from the glass"
        annotation (Dialog(group="Shading"));

      // If at least one glass layer in the room has mutiple states, then
      // set haveControllableWindow=true. In this case, the input connector for
      // the control signal will be enabled. Otherwise, it is removed.
      final parameter Boolean haveControllableWindow=
      Modelica.Math.BooleanVectors.anyTrue(
        {datConExtWin[i].glaSys.haveControllableWindow for i in 1:NConExtWin})
        "Flag, true if the windows allow multiple states, such as for electrochromic windows"
        annotation(Evaluate=true);

      final parameter Boolean haveExteriorShade[NConExtWin]=
        {datConExtWin[i].glaSys.haveExteriorShade for i in 1:NConExtWin}
        "Set to true if window has exterior shade (at surface a)"
        annotation (Dialog(group="Shading"));
      final parameter Boolean haveInteriorShade[NConExtWin]=
        {datConExtWin[i].glaSys.haveInteriorShade for i in 1:NConExtWin}
        "Set to true if window has interior shade (at surface b)"
        annotation (Dialog(group="Shading"));

      final parameter Boolean haveShade=
        Modelica.Math.BooleanVectors.anyTrue(haveExteriorShade[:]) or
        Modelica.Math.BooleanVectors.anyTrue(haveInteriorShade[:])
        "Set to true if the windows have a shade";

      final parameter Boolean is_floorConExt[NConExt]=
        datConExt.is_floor "Flag to indicate if floor for exterior constructions";
      final parameter Boolean is_floorConExtWin[NConExtWin]=
        datConExtWin.is_floor "Flag to indicate if floor for constructions";
      final parameter Boolean is_floorConPar_a[NConPar]=
        datConPar.is_floor "Flag to indicate if floor for constructions";
      final parameter Boolean is_floorConPar_b[NConPar]=
        datConPar.is_ceiling "Flag to indicate if floor for constructions";
      final parameter Boolean is_floorConBou[NConBou]=
        datConBou.is_floor
        "Flag to indicate if floor for constructions with exterior boundary conditions exposed to outside of room model";
      parameter Boolean is_floorSurBou[NSurBou]=
        surBou.is_floor
        "Flag to indicate if floor for constructions that are modeled outside of this room";

      Buildings.HeatTransfer.Windows.BaseClasses.ShadingSignal shaSig[NConExtWin](
          each final haveShade=haveShade) if haveConExtWin "Shading signal"
        annotation (Placement(transformation(extent={{-220,150},{-200,170}})));

      Buildings.ThermalZones.Detailed.BaseClasses.HeatGain heaGai(final AFlo=AFlo)
        "Model to convert internal heat gains"
        annotation (Placement(transformation(extent={{-220,70},{-200,90}})));

      Buildings.ThermalZones.Detailed.BaseClasses.RadiationAdapter radiationAdapter
        annotation (Placement(transformation(extent={{-180,120},{-160,140}})));
      Modelica.Blocks.Math.Add add
        annotation (Placement(transformation(extent={{-140,110},{-120,130}})));

      Modelica.Blocks.Math.Add sumJToWin[NConExtWin](
        each final k1=1,
        each final k2=1)
        if haveConExtWin
        "Sum of radiosity flows from room surfaces toward the window"
        annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));

      Buildings.HeatTransfer.Radiosity.RadiositySplitter radShaOut[NConExtWin]
        if haveConExtWin
        "Splitter for radiosity that strikes shading device or unshaded part of window"
        annotation (Placement(transformation(extent={{-100,120},{-80,140}})));

      Modelica.Blocks.Math.Sum sumJFroWin[NConExtWin](each nin=if haveShade then 2
             else 1)
        if haveConExtWin "Sum of radiosity fom window to room surfaces"
        annotation (Placement(transformation(extent={{-20,4},{-40,24}})));

      Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TSha[NConExtWin]
        if haveShade "Temperature of shading device"
        annotation (Placement(transformation(extent={{-20,-78},{-40,-58}})));

    initial equation
      assert(homotopyInitialization, "In " + getInstanceName() +
        ": The constant homotopyInitialization has been modified from its default value. This constant will be removed in future releases.",
        level = AssertionLevel.warning);

    equation
      connect(conBou.opa_a, surf_conBou) annotation (Line(
          points={{282,-122.667},{282,-122},{288,-122},{288,-216},{-240,-216},{-240,
              -180},{-260,-180}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(bouConExtWin.opa_a, conExtWin.opa_a) annotation (Line(
          points={{352,69},{280,69}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExtWin.JInUns_a, bouConExtWin.JOutUns) annotation (Line(
          points={{280.5,60},{304,60},{304,58},{351.5,58}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(bouConExtWin.JInUns, conExtWin.JOutUns_a) annotation (Line(
          points={{351.5,60},{316,60},{316,58},{280.5,58}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(conExtWin.glaUns_a, bouConExtWin.glaUns) annotation (Line(
          points={{280,55},{352,55}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(bouConExtWin.glaSha, conExtWin.glaSha_a) annotation (Line(
          points={{352,53},{280,53}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExtWin.JInSha_a, bouConExtWin.JOutSha) annotation (Line(
          points={{280.5,51},{286,51},{286,52},{292,52},{292,49},{351.5,49}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(bouConExtWin.JInSha, conExtWin.JOutSha_a) annotation (Line(
          points={{351.5,51},{290,51},{290,49},{280.5,49}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(conExtWin.fra_a, bouConExtWin.fra) annotation (Line(
          points={{280,46},{352,46}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExt.opa_a, bouConExt.opa_a) annotation (Line(
          points={{288,138.333},{334,138.333},{334,139},{352,139}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(weaBus, bouConExtWin.weaBus) annotation (Line(
          points={{180,160},{400,160},{400,60.05},{378.15,60.05}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(weaBus, bouConExt.weaBus) annotation (Line(
          points={{180,160},{400,160},{400,130},{378.15,130},{378.15,130.05}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(bouConExtWin.QAbsSolSha_flow, conExtWinRad.QAbsExtSha_flow)
        annotation (Line(
          points={{351,62},{312,62},{312,46},{290,46},{290,-5},{299,-5}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(bouConExtWin.inc, conExtWinRad.incAng) annotation (Line(
          points={{382.5,68},{390,68},{390,-15},{321.5,-15}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(bouConExtWin.HDir, conExtWinRad.HDir) annotation (Line(
          points={{382.5,65},{388,65},{388,-10},{321.5,-10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(bouConExtWin.HDif, conExtWinRad.HDif) annotation (Line(
          points={{382.5,62},{392,62},{392,-6},{321.5,-6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(conExtWin.QAbsSha_flow, conExtWinRad.QAbsGlaSha_flow) annotation (
          Line(
          points={{261,43},{261,38},{260,38},{260,-12},{280,-12},{280,-13},{299,-13}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(conExtWinRad.QAbsGlaUns_flow, conExtWin.QAbsUns_flow) annotation (
          Line(
          points={{299,-9},{284,-9},{284,-10},{268,-10},{268,36},{269,36},{269,43}},
          color={0,0,127},
          smooth=Smooth.None));
     // Connect statements from the model BaseClasses.MixedAir
      connect(conExt.opa_b, irRadExc.conExt) annotation (Line(
          points={{241.847,138.333},{160,138.333},{160,60},{-60,60},{-60,20},{-80,
              20},{-80,19.1667}},
          color={190,0,0},
          smooth=Smooth.None));
      connect(conExtWin.fra_b, irRadExc.conExtWinFra) annotation (Line(
          points={{249.9,46},{160,46},{160,60},{-60,60},{-60,10},{-79.9167,10}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conPar.opa_a, irRadExc.conPar_a) annotation (Line(
          points={{282,-90.3333},{288,-90.3333},{288,-106},{160,-106},{160,60},{
              -60,60},{-60,8},{-80,8},{-80,7.5},{-79.9167,7.5}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conPar.opa_b, irRadExc.conPar_b) annotation (Line(
          points={{243.873,-90.3333},{160,-90.3333},{160,60},{-60,60},{-60,5.83333},
              {-79.9167,5.83333}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(conBou.opa_b, irRadExc.conBou) annotation (Line(
          points={{241.867,-122.667},{160,-122.667},{160,60},{-60,60},{-60,3.33333},
              {-79.9167,3.33333}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(surf_surBou, irRadExc.conSurBou) annotation (Line(
          points={{-260,-140},{-232,-140},{-232,-210},{160,-210},{160,60},{-60,60},
              {-60,0.833333},{-79.9583,0.833333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(irRadGai.conExt, conExt.opa_b) annotation (Line(
          points={{-80,-20.8333},{-80,-20},{-60,-20},{-60,60},{160,60},{160,138.333},
              {241.847,138.333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(irRadGai.conExtWinFra, conExtWin.fra_b) annotation (Line(
          points={{-79.9167,-30},{-60,-30},{-60,60},{160,60},{160,46},{249.9,46}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(irRadGai.conPar_a, conPar.opa_a) annotation (Line(
          points={{-79.9167,-32.5},{-60,-32.5},{-60,60},{160,60},{160,-106},{288,
              -106},{288,-90.3333},{282,-90.3333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(irRadGai.conPar_b, conPar.opa_b) annotation (Line(
          points={{-79.9167,-34.1667},{-60,-34.1667},{-60,60},{160,60},{160,-90.3333},
              {243.873,-90.3333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(irRadGai.conBou, conBou.opa_b) annotation (Line(
          points={{-79.9167,-36.6667},{-60,-36.6667},{-60,60},{160,60},{160,-122.667},
              {241.867,-122.667}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(irRadGai.conSurBou, surf_surBou) annotation (Line(
          points={{-79.9583,-39.1667},{-60,-39.1667},{-60,60},{160,60},{160,-210},
              {-232,-210},{-232,-140},{-260,-140}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(conExtWin.opa_b, irRadExc.conExtWin) annotation (Line(
          points={{249.9,69},{160,69},{160,60},{-60,60},{-60,16},{-80,16},{-80,17.5}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExtWin.opa_b, irRadGai.conExtWin) annotation (Line(
          points={{249.9,69},{160,69},{160,60},{-60,60},{-60,-22},{-70,-22},{-70,-22.5},
              {-80,-22.5}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(conExt.opa_b, solRadExc.conExt) annotation (Line(
          points={{241.847,138.333},{160,138.333},{160,60},{-80,60},{-80,59.1667}},
          color={190,0,0},
          smooth=Smooth.None));
      connect(conExtWin.fra_b, solRadExc.conExtWinFra) annotation (Line(
          points={{249.9,46},{160,46},{160,60},{-60,60},{-60,50},{-79.9167,50}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conPar.opa_a, solRadExc.conPar_a) annotation (Line(
          points={{282,-90.3333},{288,-90.3333},{288,-106},{160,-106},{160,60},{
              -60,60},{-60,48},{-79.9167,48},{-79.9167,47.5}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conPar.opa_b, solRadExc.conPar_b) annotation (Line(
          points={{243.873,-90.3333},{160,-90.3333},{160,60},{-60,60},{-60,46},{
              -70,46},{-70,45.8333},{-79.9167,45.8333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conBou.opa_b, solRadExc.conBou) annotation (Line(
          points={{241.867,-122.667},{160,-122.667},{160,60},{-60,60},{-60,43.3333},
              {-79.9167,43.3333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(surf_surBou, solRadExc.conSurBou) annotation (Line(
          points={{-260,-140},{-232,-140},{-232,-210},{160,-210},{160,60},{-60,60},
              {-60,40},{-70,40},{-70,40.8333},{-79.9583,40.8333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExtWin.opa_b, solRadExc.conExtWin) annotation (Line(
          points={{249.9,69},{160,69},{160,60},{-60,60},{-60,57.5},{-80,57.5}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(solRadExc.JInDifConExtWin, conExtWinRad.QTraDif_flow) annotation (
          Line(
          points={{-79.5833,53.3333},{20,53.3333},{20,-20},{299,-20}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(solRadExc.HOutConExtWin,conExtWinRad.HRoo)  annotation (Line(
          points={{-79.5833,55},{10,55},{10,-34},{328,-34},{328,-21.6},{321.5,-21.6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(conExt.opa_b, radTem.conExt) annotation (Line(
          points={{241.847,138.333},{160,138.333},{160,60},{-60,60},{-60,-60.8333},
              {-80,-60.8333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExtWin.opa_b, radTem.conExtWin) annotation (Line(
          points={{249.9,69},{160,69},{160,60},{-60,60},{-60,-62.5},{-80,-62.5}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conExtWin.fra_b, radTem.conExtWinFra) annotation (Line(
          points={{249.9,46},{160,46},{160,60},{-60,60},{-60,-70},{-79.9167,-70}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conPar.opa_a, radTem.conPar_a) annotation (Line(
          points={{282,-90.3333},{288,-90.3333},{288,-106},{160,-106},{160,60},{
              -60,60},{-60,-72.5},{-79.9167,-72.5}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(conPar.opa_b, radTem.conPar_b) annotation (Line(
          points={{243.873,-90.3333},{160,-90.3333},{160,60},{-60,60},{-60,-74.1667},
              {-79.9167,-74.1667}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(conBou.opa_b, radTem.conBou) annotation (Line(
          points={{241.867,-122.667},{160,-122.667},{160,60},{-60,60},{-60,-76.6667},
              {-79.9167,-76.6667}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(surf_surBou, radTem.conSurBou) annotation (Line(
          points={{-260,-140},{-232,-140},{-232,-210},{160,-210},{160,60},{-60,60},
              {-60,-79.1667},{-79.9583,-79.1667}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(radTem.glaUns, conExtWin.glaUns_b) annotation (Line(
          points={{-80,-65},{-60,-65},{-60,60},{160,60},{160,55},{250,55}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(radTem.glaSha, conExtWin.glaSha_b) annotation (Line(
          points={{-80,-66.6667},{-60,-66.6667},{-60,60},{160,60},{160,53},{250,
              53}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(radTem.TRad, radiationAdapter.TRad) annotation (Line(
          points={{-100.417,-77.6667},{-144,-77.6667},{-144,-78},{-186,-78},{-186,
              130},{-182,130}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(radiationAdapter.rad, heaPorRad)
                                         annotation (Line(
          points={{-170.2,120},{-170,120},{-170,114},{-226,114},{-226,4.44089e-16},
              {-260,4.44089e-16}},
          color={191,0,0},
          smooth=Smooth.None));

      connect(radiationAdapter.QRad_flow, add.u1) annotation (Line(
          points={{-159,130},{-150,130},{-150,126},{-142,126}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, irRadGai.Q_flow) annotation (Line(
          points={{-119,120},{-116,120},{-116,-30},{-100.833,-30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(irRadExc.JOutConExtWin, sumJToWin.u1)
                                               annotation (Line(
          points={{-79.5833,15},{-50,15},{-50,-14},{-42,-14}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(irRadGai.JOutConExtWin, sumJToWin.u2)
                                               annotation (Line(
          points={{-79.5833,-25},{-46,-25},{-46,-26},{-42,-26}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(shaSig.y, radShaOut.u) annotation (Line(
          points={{-199,160},{-110,160},{-110,124},{-102,124}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(shaSig.y, shaRad.u) annotation (Line(
          points={{-199,160},{-64,160},{-64,108},{-61,108}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(sumJToWin.y, radShaOut.JIn)
                                     annotation (Line(
          points={{-19,-20},{0,-20},{0,148},{-106,148},{-106,136},{-101,136}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(radShaOut.JOut_1, shaRad.JIn_air) annotation (Line(
          points={{-79,136},{-70,136},{-70,96},{-61,96}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(radShaOut.JOut_2, conExtWin.JInUns_b) annotation (Line(
          points={{-79,124},{-20,124},{-20,58},{249.5,58}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(shaRad.JOut_glass, conExtWin.JInSha_b) annotation (Line(
          points={{-39,96},{20,96},{20,72},{220,72},{220,49},{249.5,49}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(conExtWin.JOutSha_b, shaRad.JIn_glass) annotation (Line(
          points={{249.5,51},{222,51},{222,70},{16,70},{16,92},{-39,92}},
          color={0,127,0},
          smooth=Smooth.None));

      connect(irRadExc.JInConExtWin, sumJFroWin.y) annotation (Line(
          points={{-79.5833,13.3333},{-46,13.3333},{-46,14},{-41,14}},
          color={0,127,0},
          smooth=Smooth.None));

      connect(shaRad.QSolAbs_flow, conExtWinRad.QAbsIntSha_flow) annotation (Line(
          points={{-50,89},{-50,86},{148,86},{148,-17},{299,-17}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sumJFroWin.u[1], conExtWin.JOutUns_b) annotation (Line(
          points={{-18,14},{164,14},{164,60},{249.5,60}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sumJFroWin.u[2], shaRad.JOut_air) annotation (Line(
          points={{-18,14},{-10,14},{-10,40},{-40,40},{-40,64},{-66,64},{-66,92},{
              -61,92}},
          color={0,127,0},
          smooth=Smooth.None));
      connect(radTem.sha, TSha.port) annotation (Line(
          points={{-80,-68.4167},{-64,-68.4167},{-64,-68},{-40,-68}},
          color={191,0,0},
          smooth=Smooth.None));
          connect(souInf.ports[1], air.ports[1]);
    connect(sinInf.ports[1], air.ports[2]);
      for i in 1:nPorts loop
        connect(ports[i],air. ports[i+2])
                                      annotation (Line(
          points={{-260,-60},{-218,-60},{-218,-206},{52,-206},{52,-141.9}},
          color={0,127,255},
          smooth=Smooth.None));
      end for;

      connect(air.conExt, conExt.opa_b) annotation (Line(
          points={{64,-119},{160,-119},{160,138.333},{241.847,138.333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.conExtWin, conExtWin.opa_b) annotation (Line(
          points={{64,-121},{160,-121},{160,69},{249.9,69}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.glaUns, conExtWin.glaUns_b) annotation (Line(
          points={{64,-124},{160,-124},{160,55},{250,55}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.glaSha, conExtWin.glaSha_b) annotation (Line(
          points={{64,-126},{160,-126},{160,53},{250,53}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.conExtWinFra, conExtWin.fra_b) annotation (Line(
          points={{64.1,-130},{160,-130},{160,46},{249.9,46}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.conPar_a, conPar.opa_a) annotation (Line(
          points={{64.1,-133},{160,-133},{160,-106},{288,-106},{288,-90.3333},{282,
              -90.3333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.conPar_b, conPar.opa_b) annotation (Line(
          points={{64.1,-135},{160,-135},{160,-90},{243.873,-90},{243.873,-90.3333}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.conBou, conBou.opa_b) annotation (Line(
          points={{64.1,-138},{160,-138},{160,-122.667},{241.867,-122.667}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.conSurBou, surf_surBou) annotation (Line(
          points={{64.05,-141},{160,-141},{160,-210},{-232,-210},{-232,-140},{-260,-140}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(shaRad.QRadAbs_flow,air. QRadAbs_flow) annotation (Line(
          points={{-55,89},{-55,72},{4,72},{4,-125},{39.5,-125}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(air.TSha, shaRad.TSha) annotation (Line(
          points={{39.5,-127},{2,-127},{2,70},{-45,70},{-45,89}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(air.heaPorAir, heaPorAir) annotation (Line(
          points={{40,-130},{-10,-130},{-10,-88},{-200,-88},{-200,40},{-260,40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(air.TSha, TSha.T) annotation (Line(
          points={{39.5,-127},{2,-127},{2,-68},{-18,-68}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(uWin, conExtWinRad.uSta) annotation (Line(points={{-280,140},{-240,
              140},{-240,180},{420,180},{420,-40},{305.2,-40},{305.2,-25.6}}, color=
             {0,0,127}));
      connect(qGai_flow,heaGai. qGai_flow) annotation (Line(
          points={{-280,80},{-222,80}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(air.QCon_flow,heaGai. QCon_flow) annotation (Line(
          points={{39,-135},{-14,-135},{-14,-92},{-190,-92},{-190,80},{-198,80}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(air.QLat_flow,heaGai. QLat_flow) annotation (Line(
          points={{39,-138},{-18,-138},{-18,-96},{-194,-96},{-194,74},{-198,74}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(heaGai.QRad_flow, add.u2) annotation (Line(
          points={{-198,86},{-152,86},{-152,114},{-142,114}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(conExtWinRad.QTraDir_flow, solRadExc.JInDirConExtWin) annotation (
          Line(points={{299,-23},{18,-23},{18,51.6667},{-79.5833,51.6667}}, color={
              0,0,127}));
      connect(weaBus.TDryBul, souInf.T_in) annotation (Line(
          points={{180.05,160.05},{100,160.05},{100,-146},{2.6,-146},{2.6,-160.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-3,6},{-3,6}},
          horizontalAlignment=TextAlignment.Right));

      connect(airInfiltration.y, sinInf.m_flow_in) annotation (Line(points={{-39,-178},
              {-8,-178},{-8,-177.8},{0.2,-177.8}}, color={0,0,127}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-260,-220},{460,
                200}})),
            Icon(coordinateSystem(preserveAspectRatio=false,extent={{-200,-200},{200,
                200}}), graphics={
            Text(
              extent={{-104,210},{84,242}},
              textColor={0,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="%name"),
            Text(
              extent={{-220,100},{-144,68}},
              textColor={0,0,127},
              textString="q"),
            Text(
              extent={{-14,-160},{44,-186}},
              textColor={0,0,0},
              fillColor={61,61,61},
              fillPattern=FillPattern.Solid,
              textString="boundary"),
            Rectangle(
              extent={{-160,-160},{160,160}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-140,140},{140,-140}},
              pattern=LinePattern.None,
              lineColor={117,148,176},
              fillColor={170,213,255},
              fillPattern=FillPattern.Sphere),
            Rectangle(
              extent={{140,70},{160,-70}},
              lineColor={95,95,95},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{146,70},{154,-70}},
              lineColor={95,95,95},
              fillColor={170,213,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-60,12},{-22,-10}},
              textColor={0,0,0},
              fillColor={61,61,61},
              fillPattern=FillPattern.Solid,
              textString="air"),
            Text(
              extent={{-72,-22},{-22,-50}},
              textColor={0,0,0},
              fillColor={61,61,61},
              fillPattern=FillPattern.Solid,
              textString="radiation"),
            Text(
              extent={{-104,-124},{-54,-152}},
              textColor={0,0,0},
              fillColor={61,61,61},
              fillPattern=FillPattern.Solid,
              textString="surface"),
            Text(
              extent={{-198,144},{-122,112}},
              textColor={0,0,127},
              textString="uWin"),
            Rectangle(
              extent={{-140,140},{140,-140}},
              lineColor={117,148,176},
              fillPattern=FillPattern.Solid,
              fillColor=DynamicSelect({170,213,255},
                min(1, max(0, (1-(heaPorAir.T-295.15)/10)))*{28,108,200}+
                min(1, max(0, (heaPorAir.T-295.15)/10))*{255,0,0})),
            Text(
              extent={{134,-84},{14,-134}},
              textColor={255,255,255},
              textString=DynamicSelect("", String(heaPorAir.T-273.15, format=".1f")))}),
        preferredView="info",
        defaultComponentName="roo");
    end RoomHeatMassBalanceInf;
  end BaseClasses;

  model MixedAirInf
    "Model of a room in which the air is completely mixed"
    extends Trano.ThermalZones.BaseClasses.RoomHeatMassBalanceInf(
    redeclare Buildings.ThermalZones.Detailed.BaseClasses.MixedAirHeatMassBalance air(
      final energyDynamics=energyDynamics,
      final massDynamics = energyDynamics,
      final p_start=p_start,
      final T_start=T_start,
      final X_start=X_start,
      final C_start=C_start,
      final C_nominal=C_nominal,
      final mSenFac=mSenFac,
      final m_flow_nominal=m_flow_nominal,
      final homotopyInitialization=homotopyInitialization,
      final conMod=intConMod,
      final hFixed=hIntFixed,
      final use_C_flow = use_C_flow),
      datConExt(
        each T_a_start = T_start,
        each T_b_start = T_start),
      datConExtWin(
        each T_a_start = T_start,
        each T_b_start = T_start),
      datConBou(
        each T_a_start = T_start,
        each T_b_start = T_start),
      datConPar(
        each T_a_start = T_start,
        each T_b_start = T_start));

    ////////////////////////////////////////////////////////////////////////////
    // Media declaration. This is identical to
    // Buildings.Fluid.Interfaces.LumpedVolumeDeclarations, except
    // that the comments have been changed to avoid a confusion about
    // what energyDynamics refers to.
    replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium in the component"
        annotation (choicesAllMatching = true);

    // Ports
    parameter Boolean use_C_flow=false
      "Set to true to enable input connector for trace substance that is connected to room air"
      annotation (Dialog(group="Ports"));

    // Assumptions
    parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
      "Type of energy balance for zone air: dynamic (3 initialization options) or steady state"
      annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Zone air"));

    parameter Real mSenFac(min=1)=1
      "Factor for scaling the sensible thermal mass of the zone air volume"
      annotation(Dialog(tab="Dynamics", group="Zone air"));

    // Initialization
    parameter Medium.AbsolutePressure p_start = Medium.p_default
      "Start value of zone air pressure"
      annotation(Dialog(tab = "Initialization"));
    parameter Medium.Temperature T_start=Medium.T_default
      "Start value of zone air temperature"
      annotation(Dialog(tab = "Initialization"));
    parameter Medium.MassFraction X_start[Medium.nX](
         quantity=Medium.substanceNames) = Medium.X_default
      "Start value of zone air mass fractions m_i/m"
      annotation (Dialog(tab="Initialization", enable=Medium.nXi > 0));
    parameter Medium.ExtraProperty C_start[Medium.nC](
         quantity=Medium.extraPropertiesNames)=fill(0, Medium.nC)
      "Start value of zone air trace substances"
      annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));
    parameter Medium.ExtraProperty C_nominal[Medium.nC](
         quantity=Medium.extraPropertiesNames) = fill(1E-2, Medium.nC)
      "Nominal value of zone air trace substances. (Set to typical order of magnitude.)"
     annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));

    ////////////////////////////////////////////////////////////////////////////
    // Input connectors
    Modelica.Blocks.Interfaces.RealInput uSha[nConExtWin](each min=0, each max=1)
      if haveShade
      "Control signal for the shading device (removed if no shade is present)"
      annotation (Placement(transformation(extent={{-300,160},{-260,200}}),
          iconTransformation(extent={{-232,164},{-200,196}})));

    Modelica.Blocks.Interfaces.RealInput C_flow[Medium.nC] if use_C_flow
      "Trace substance mass flow rate added to the room air. Enable if use_C_flow = true"
      annotation (Placement(transformation(extent={{-300,-130},{-260,-90}}),
          iconTransformation(extent={{-232,12},{-200,44}})));

  equation
    connect(uSha, conExtWin.uSha) annotation (Line(
        points={{-280,180},{308,180},{308,62},{281,62}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(uSha, bouConExtWin.uSha) annotation (Line(
        points={{-280,180},{308,180},{308,64},{351,64}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(uSha, conExtWinRad.uSha) annotation (Line(
        points={{-280,180},{422,180},{422,-40},{310.2,-40},{310.2,-25.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(irRadGai.uSha,uSha)
      annotation (Line(
        points={{-100.833,-22.5},{-110,-22.5},{-110,180},{-280,180}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(uSha, radTem.uSha) annotation (Line(
        points={{-280,180},{-110,180},{-110,-62},{-100.833,-62},{-100.833,-62.5}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(uSha, shaSig.u) annotation (Line(
        points={{-280,180},{-248,180},{-248,160},{-222,160}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(air.uSha,uSha)  annotation (Line(
        points={{39.6,-120},{8,-120},{8,180},{-280,180}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(C_flow, air.C_flow) annotation (Line(points={{-280,-110},{-200,-110},{
            -200,-114},{-200,-114},{-200,-202},{-18,-202},{-18,-141},{39,-141}},
          color={0,0,127}));
    annotation (
     Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-200},
              {200,200}}), graphics={
          Text(
            extent={{-198,198},{-122,166}},
            textColor={0,0,127},
            textString="uSha"),
          Text(
            extent={{-190,44},{-128,14}},
            textColor={0,0,127},
            textString="C_flow",
            visible=use_C_flow)}));
  end MixedAirInf;
end ThermalZones;
  annotation (uses(Buildings(version = "11.0.0"), Modelica(version = "4.0.0"),
      IDEAS(version="3.0.0")),
  Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
            fillPattern =                                                                            FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
end Trano;



package Components
  package Containers
    model envelope

            parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic win_01(
    final glass={
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003048,
        k=1.0,
        tauSol={ 0.834 },
        rhoSol_a={ 0.075 },
        rhoSol_b={ 0.075 },
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84)
        ,
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003048,
        k=1.0,
        tauSol={ 0.834 },
        rhoSol_a={ 0.075 },
        rhoSol_b={ 0.075 },
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84)
        
    },
    final gas={
            Buildings.HeatTransfer.Data.Gases.Air(x=0.012)
            
    },
    UFra=1.4)
    annotation (
    defaultComponentPrefixes="parameter",
    defaultComponentName="datGlaSys");
    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        exterior_wall_001(
    final nLay=3,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.009,
        k=0.14,
        c=900.0,
        d=530.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.066,
        k=0.04,
        c=840.0,
        d=12.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.012,
        k=0.16,
        c=840.0,
        d=950.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        floor_001(
    final nLay=2,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=1.003,
        k=0.04,
        c=0.0,
        d=0.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.025,
        k=0.14,
        c=1200.0,
        d=650.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        roof_001(
    final nLay=3,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.019,
        k=0.14,
        c=900.0,
        d=530.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.1118,
        k=0.04,
        c=840.0,
        d=12.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.01,
        k=0.16,
        c=840.0,
        d=950.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));

// Define Medium Package
extends Trano.BaseClasses.Containers.envelope ;
replaceable package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"})
  constrainedby Modelica.Media.Interfaces.PartialMedium

  annotation (choicesAllMatching = true);

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[0] heatPortCon
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[1] heatPortCon1
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{-4,98},{6,108}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[0] heatPortRad

  annotation (
      Placement(transformation(extent= {{90,-62},{110,-42}} )),
      iconTransformation(extent= {{90,-62},{110,-42}} )
  );
  Modelica.Blocks.Interfaces.RealOutput y[0] annotation (Placement(transformation(
      extent= {{-100,-16},{-134,18}} ), iconTransformation(extent= {{-100,-16},{-134,18}} )));
// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
  annotation (Placement(transformation(extent= {{-20,80},{20,120}} )),iconTransformation(extent=  {{-228,58},{-208,78}} ));

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPorts_b[1] ports_b(
  redeclare package Medium = Medium)
  annotation (Placement(transformation(extent= {{-106,34},{-96,78}} ), iconTransformation(
      extent= {{-106,34},{-96,78}} )), iconTransformation(extent=  {{-106,30},{-92,86}} ));

Modelica.Fluid.Interfaces.FluidPorts_a[0] ports_a(
  redeclare package Medium = Medium)
  annotation ( Placement(transformation(extent= {{-104,-80},{-94,-34}} ), iconTransformation(
      extent= {{-104,-80},{-94,-34}} )),iconTransformation(extent=  {{-108,-92},{-94,-40}} ));
    Buildings.ThermalZones.Detailed.MixedAir space_001(
        redeclare package Medium = Medium,
            hRoo=2.7,
    AFlo=48.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=4,
                    datConExt(
                    layers={ exterior_wall_001, exterior_wall_001, exterior_wall_001, roof_001 },
    A={ 21.6, 16.2, 16.2, 48.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling},
                    azi={ 180.0, 270.0, 90.0, 0.0 }),
                    nSurBou=0,                    nConBou=1,
                    datConBou(
                    layers={ floor_001 },
    A={ 48.0 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 0.0 }),
                    nConExtWin=1,
                    datConExtWin(
                    layers={ exterior_wall_001 },
    A={ 21.6 },
    til={Buildings.Types.Tilt.Wall},
                    glaSys={ win_01 },
                    wWin={ 6.0 },
                    hWin={ 2.0 },
                    azi={ 0.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 1.9979594109687469, -2.8373435996150107 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
    
    
    
        case600FF.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[80/48; 120/48; 0],
    k=1/1/1,
    occupancy=3600*{0, 24}
) annotation (
    Placement(transformation(origin = { -13.002040589031253, -2.8373435996150107 },
    extent = {{ 3, -3}, {-3, 3}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather(    filNam=Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_CO_Denver.Intl.AP.725650_TMY3.mos")
)
     annotation (
    Placement(transformation(origin = { 100.0, -35.38915623997433 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(space_001.qGai_flow,occupancy_1.y)
        annotation (Line(
        points={{ 1.9979594109687469, -2.8373435996150107 }    ,{ -5.502040589031253, -2.8373435996150107 }    ,{ -5.502040589031253, -2.8373435996150107 }    ,{ -13.002040589031253, -2.8373435996150107 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_001.weaBus,weather.weaBus)
        annotation (Line(
        points={{ 1.9979594109687469, -2.8373435996150107 }    ,{ 50.99897970548437, -2.8373435996150107 }    ,{ 50.99897970548437, -35.38915623997433 }    ,{ 100.0, -35.38915623997433 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(occupancy_1.dataBus,dataBus)
            ;        
        connect(space_001.heaPorAir,heatPortCon1[1])
            ;        
        connect(space_001.ports[1],ports_b[1])
            ;        
        connect(weather.weaBus,dataBus)
        annotation (Line(
        points={{ 100.0, -35.38915623997433 }    ,{ 100.0, -35.38915623997433 }    ,{ 100.0, -35.38915623997433 }    ,{ 100.0, -35.38915623997433 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;end envelope;
    model bus


extends Trano.BaseClasses.Containers.bus;
// Define Medium Package
package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"});

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_b[1] port_b(
  redeclare package Medium = Medium)
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[1] heatPortCon

  annotation (
      Placement(transformation(extent= {{-108,42},{-88,62}} )),
      iconTransformation(extent= {{-108,42},{-88,62}} )
  );
// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
  annotation (
      Placement(transformation(extent= {{-118,68},{-78,108}} )), 
      iconTransformation(extent= {{-228,58},{-208,78}} )
  );
  Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p annotation (
  Placement(transformation(extent= {{66,-24},{114,24}} ), iconTransformation(
      extent= {{66,-24},{114,24}} )));
        case600FF.Components.BaseClasses.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 0.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(data_bus.port[1],heatPortCon[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[1],port_b[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.dataBus,dataBus)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(data_bus.term_p,term_p)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;end bus;

model building

Components.Containers.envelope envelope1 annotation (Placement(transformation(extent={{-84.0,0.0},{-64.0,20.0}})));
Components.Containers.bus bus1 annotation (Placement(transformation(extent={{-84.0,30.0},{-64.0,50.0}})));

Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p
annotation (Placement(transformation(extent={{-126,-18},{-92,18}}),
iconTransformation(
extent={{-112,-12},{-88,12}})));
equation
connect(term_p, bus1.term_p) annotation (Line(points={{-109,0},{-88,0},
        {-88,-10},{60,-10},{60,64},{-50,64},{-50,40},{-65,40}}, color={
        0,120,120}));
connect(envelope1.heatPortCon1[1],
bus1.heatPortCon[1])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[1],
bus1.port_b[1])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));

connect(bus1.dataBus, envelope1.dataBus) annotation (Line(points={{-83.8,48.8},{-83.8,56},{-60,56},{-60,26},{-74,26},{-74,20}}, color={255,204,51}, thickness=0.5));



annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
        Rectangle(
          extent={{-74,18},{22,-40}},
            fillColor={255,255,255},
            fillPattern=FillPattern.Forward,
            pattern=LinePattern.None,
            lineColor={238,46,47}),
        Rectangle(
          extent={{-62,2},{-38,-16}},
          lineColor={238,46,47},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-14,2},{8,-16}},
          lineColor={238,46,47},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
        Polygon(
          points={{-78,18},{26,18},{10,46},{-66,46},{-78,18}},
            lineColor={238,46,47},
            lineThickness=0.5,
            fillColor={244,125,35},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-60,42},{-68,22},{4,22},{6,42},{-60,42}},
            lineThickness=0.5,
            fillColor={28,108,200},
            fillPattern=FillPattern.Forward,
            pattern=LinePattern.None),
          Rectangle(
            extent={{26,0},{40,-40}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                  Diagram(
coordinateSystem(preserveAspectRatio=false)));
end building;
  end Containers;

  package BaseClasses
        model OccupancyOccupancy_1
extends case600FF.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_001, occSch2.occupied);
 end OccupancyOccupancy_1;
 
        model DataServer
replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[1]
TRoo annotation (
Placement(transformation(origin={-544,-226},
extent = {{480, 216}, {500, 236}})));Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[1]
port annotation (
Placement(transformation(extent={{-112,-10},{-92,10}}),
iconTransformation(extent = {{-110, -10}, {-90, 10}})));Buildings.Fluid.Sensors.PPM[1] TRoo1(redeclare
package Medium = Medium)annotation (
Placement(transformation(origin={-542,-268},
extent = {{480, 216}, {500, 236}})));Modelica.Fluid.Interfaces.FluidPort_a[1]
port_a(redeclare package Medium
= Medium)annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));  Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
    tableOnFile=false,
    table=[0.0,293.15;3600.0,285.73596;3600.0,285.73596;7200.0,281.54343;10800.0,278.19498;14400.0,275.43002;18000.0,273.27002;21600.0,271.58206;25200.0,270.40155;28800.0,269.92743;32400.0,271.53067;36000.0,278.5456;39600.0,286.87137;43200.0,294.80823;46800.0,299.70322;50400.0,304.21143;54000.0,305.46237;57600.0,305.1635;61200.0,301.38977;64800.0,296.79562;68400.0,292.56265;72000.0,288.89627;75600.0,285.75018;79200.0,283.06958;82800.0,280.76053;86400.0,278.8642;90000.0,277.4146;93600.0,276.05743;97200.0,274.90744;100800.0,273.77957;104400.0,272.90625;108000.0,271.98444;111600.0,271.51764;115200.0,271.9857;118800.0,273.9943;122400.0,280.1072;126000.0,290.1367;129600.0,296.22882;133200.0,301.6421;136800.0,303.491;140400.0,306.2017;144000.0,305.816;147600.0,302.8868;151200.0,299.1967;154800.0,295.56863;158400.0,292.24817;162000.0,289.23538;165600.0,286.70007;169200.0,284.84912;172800.0,283.02966;176400.0,281.50693;180000.0,280.21005;183600.0,279.08975;187200.0,278.046;190800.0,277.13824;194400.0,276.40005;198000.0,275.90192;201600.0,275.8993;205200.0,278.02463;208800.0,282.59933;212400.0,290.97238;216000.0,300.76462;219600.0,307.9591;223200.0,312.45132;226800.0,313.08624;230400.0,309.44702;234000.0,303.90845;237600.0,298.55646;241200.0,294.0508;244800.0,290.14618;248400.0,286.53723;252000.0,283.27032;255600.0,280.29495;259200.0,277.56985;262800.0,275.27332;266400.0,272.96823;270000.0,270.77158;273600.0,269.08566;277200.0,267.7469;280800.0,266.57816;284400.0,265.79807;288000.0,266.48462;291600.0,268.66788;295200.0,271.6971;298800.0,274.11313;302400.0,276.23257;306000.0,278.27722;309600.0,280.1381;313200.0,282.18634;316800.0,283.42706;320400.0,283.6251;324000.0,283.06018;327600.0,282.52786;331200.0,282.1704;334800.0,281.80588;338400.0,281.36996;342000.0,280.91092;345600.0,281.0857;349200.0,280.8853;352800.0,281.12213;356400.0,281.75925;360000.0,282.0094;363600.0,282.04984;367200.0,282.0359;370800.0,281.6016;374400.0,282.9773;378000.0,289.05618;381600.0,297.66763;385200.0,306.5099;388800.0,315.49548;392400.0,322.64282;396000.0,327.39233;399600.0,327.6462;403200.0,325.19873;406800.0,319.3165;410400.0,313.03986;414000.0,307.5665;417600.0,302.87955;421200.0,298.73642;424800.0,295.0535;428400.0,292.02167;432000.0,289.2355;435600.0,286.99225;439200.0,285.27335;442800.0,283.78708;446400.0,282.37015;450000.0,281.05878;453600.0,279.8051;457200.0,278.81677;460800.0,278.68372;464400.0,282.49048;468000.0,288.6723;471600.0,297.87296;475200.0,306.6249;478800.0,313.55328;482400.0,318.3792;486000.0,320.57318;489600.0,319.20166;493200.0,313.51782;496800.0,307.05246;500400.0,301.50894;504000.0,296.6553;507600.0,292.4551;511200.0,288.61847;514800.0,285.39413;518400.0,282.4947;522000.0,280.00885;525600.0,278.04507;529200.0,276.05234;532800.0,274.11917;536400.0,272.28162;540000.0,270.6053;543600.0,269.34506;547200.0,269.38843;550800.0,270.46542;554400.0,271.80545;558000.0,273.2336;561600.0,274.9095;565200.0,276.45193;568800.0,277.53928;572400.0,278.2633;576000.0,278.55435;579600.0,278.58466;583200.0,278.71814;586800.0,278.7509;590400.0,278.6885;594000.0,278.3295;597600.0,277.5598;601200.0,276.90277;604800.0,276.62808;608400.0,276.58603;612000.0,276.42612;615600.0,276.01605;619200.0,275.5309;622800.0,274.91965;626400.0,274.25217;630000.0,273.56726;633600.0,274.06497;637200.0,279.83496;640800.0,288.49243;644400.0,296.4454;648000.0,304.6756;651600.0,312.75198;655200.0,319.88644;658800.0,323.7284;662400.0,323.77997;666000.0,318.60803;669600.0,311.7458;673200.0,305.7815;676800.0,300.64767;680400.0,296.4065;684000.0,292.73572;687600.0,289.72058;691200.0,287.02347;694800.0,284.59085;698400.0,282.525;702000.0,280.7705;705600.0,279.27234;709200.0,277.92834;712800.0,276.78818;716400.0,275.62836;720000.0,276.13004;723600.0,281.41367;727200.0,289.53735;730800.0,298.66574;734400.0,307.10226;738000.0,314.06308;741600.0,319.20728;745200.0,321.6883;748800.0,320.63266;752400.0,315.38956;756000.0,309.07983;759600.0,303.61005;763200.0,299.11252;766800.0,295.07974;770400.0,291.60938;774000.0,288.56622;777600.0,285.70834;781200.0,283.29465;784800.0,281.18466;788400.0,279.3576;792000.0,277.72388;795600.0,276.26147;799200.0,274.92923;802800.0,273.91818;806400.0,275.0019;810000.0,280.91968;813600.0,289.5751;817200.0,299.2379;820800.0,308.6172;824400.0,316.91013;828000.0,323.21512;831600.0,327.05005;835200.0,326.75482;838800.0,321.51978;842400.0,314.16238;846000.0,307.69388;849600.0,302.15326;853200.0,297.60974;856800.0,293.55325;860400.0,289.98114;864000.0,286.7285;867600.0,284.03558;871200.0,281.63336;874800.0,279.55164;878400.0,277.86166;882000.0,276.5579;885600.0,275.51105;889200.0,274.75967;892800.0,275.06442;896400.0,278.38425;900000.0,285.74817;903600.0,293.3195;907200.0,302.53458;910800.0,310.16513;914400.0,315.8372;918000.0,314.70862;921600.0,311.691;925200.0,307.83618;928800.0,303.81815;932400.0,300.28445;936000.0,297.10822;939600.0,294.23203;943200.0,292.1059;946800.0,290.1078;950400.0,288.3053;954000.0,286.6999;957600.0,284.94156;961200.0,283.23666;964800.0,281.4541;968400.0,279.7168;972000.0,278.08755;975600.0,276.71884;979200.0,275.70123;982800.0,275.65277;986400.0,276.38345;990000.0,277.73785;993600.0,279.3097;997200.0,280.59906;1000800.0,281.66046;1004400.0,282.38797;1008000.0,282.74374;1011600.0,282.18057;1015200.0,281.18582;1018800.0,280.39136;1022400.0,279.79202;1026000.0,278.79892;1029600.0,277.67172;1033200.0,276.87564;1036800.0,276.0937;1040400.0,275.78705;1044000.0,275.94168;1047600.0,276.17212;1051200.0,276.15793;1054800.0,276.23004;1058400.0,276.0972;1062000.0,276.07288;1065600.0,278.00363;1069200.0,284.86417;1072800.0,294.15173;1076400.0,304.29437;1080000.0,313.7667;1083600.0,321.29544;1087200.0,327.38495;1090800.0,330.37512;1094400.0,329.90588;1098000.0,324.54572;1101600.0,317.62158;1105200.0,311.70285;1108800.0,307.02673;1112400.0,302.8069;1116000.0,298.94852;1119600.0,295.42233;1123200.0,292.46674;1126800.0,290.50446;1130400.0,289.0736;1134000.0,287.93665;1137600.0,286.91354;1141200.0,285.97037;1144800.0,285.12494;1148400.0,284.43927;1152000.0,284.45322;1155600.0,287.3185;1159200.0,292.24814;1162800.0,299.60083;1166400.0,309.27557;1170000.0,312.59848;1173600.0,313.71582;1177200.0,313.68036;1180800.0,310.4088;1184400.0,306.58835;1188000.0,303.00375;1191600.0,299.73746;1195200.0,296.7601;1198800.0,293.8435;1202400.0,291.10104;1206000.0,288.72406;1209600.0,286.43878;1213200.0,284.5283;1216800.0,282.76346;1220400.0,281.16562;1224000.0,279.7139;1227600.0,278.5602;1231200.0,277.59183;1234800.0,276.74652;1238400.0,277.7362;1242000.0,283.82986;1245600.0,292.8687;1249200.0,303.00598;1252800.0,313.1654;1256400.0,321.77972;1260000.0,328.29025;1263600.0,332.21527;1267200.0,331.891;1270800.0,326.89206;1274400.0,319.64438;1278000.0,313.21353;1281600.0,307.7246;1285200.0,302.89755;1288800.0,298.75104;1292400.0,294.95624;1296000.0,291.68628;1299600.0,289.39496;1303200.0,287.57773;1306800.0,286.447;1310400.0,284.8281;1314000.0,283.0506;1317600.0,281.43494;1321200.0,280.21378;1324800.0,279.58798;1328400.0,280.6631;1332000.0,283.2225;1335600.0,287.05173;1339200.0,291.81622;1342800.0,297.3973;1346400.0,302.37683;1350000.0,305.44418;1353600.0,305.67624;1357200.0,302.35703;1360800.0,298.0204;1364400.0,294.37503;1368000.0,291.43756;1371600.0,288.7816;1375200.0,286.45013;1378800.0,284.3544;1382400.0,282.5552;1386000.0,280.94812;1389600.0,279.6656;1393200.0,278.53076;1396800.0,277.97562;1400400.0,277.37674;1404000.0,276.80423;1407600.0,276.30304;1411200.0,275.4029;1414800.0,275.35825;1418400.0,278.67276;1422000.0,283.74884;1425600.0,290.58795;1429200.0,297.2169;1432800.0,302.89633;1436400.0,306.72922;1440000.0,304.35187;1443600.0,299.64978;1447200.0,294.7976;1450800.0,290.3283;1454400.0,286.50754;1458000.0,283.24002;1461600.0,280.38217;1465200.0,277.85095;1468800.0,275.65247;1472400.0,273.7571;1476000.0,272.10504;1479600.0,270.62933;1483200.0,269.29172;1486800.0,268.2541;1490400.0,267.31918;1494000.0,266.66644;1497600.0,268.39185;1501200.0,274.8971;1504800.0,284.15085;1508400.0,294.91867;1512000.0,305.5859;1515600.0,314.95206;1519200.0,322.659;1522800.0,327.4425;1526400.0,328.84952;1530000.0,325.38416;1533600.0,318.66016;1537200.0,313.12927;1540800.0,308.571;1544400.0,304.78326;1548000.0,301.5591;1551600.0,298.81284;1555200.0,296.274;1558800.0,294.2292;1562400.0,292.6075;1566000.0,291.04263;1569600.0,289.4208;1573200.0,287.60608;1576800.0,286.30322;1580400.0,285.79672;1584000.0,287.06583;1587600.0,292.8207;1591200.0,301.06372;1594800.0,310.4924;1598400.0,319.61316;1602000.0,326.4704;1605600.0,331.15518;1609200.0,333.6188;1612800.0,332.47635;1616400.0,327.564;1620000.0,320.63474;1623600.0,314.6759;1627200.0,308.71173;1630800.0,302.92703;1634400.0,297.97144;1638000.0,293.65695;1641600.0,289.80783;1645200.0,286.6097;1648800.0,283.87494;1652400.0,281.63116;1656000.0,279.73825;1659600.0,278.1464;1663200.0,276.70828;1666800.0,275.68726;1670400.0,276.53116;1674000.0,281.70746;1677600.0,289.83115;1681200.0,299.3726;1684800.0,308.34244;1688400.0,316.13205;1692000.0,322.08157;1695600.0,325.52725;1699200.0,325.55933;1702800.0,321.32584;1706400.0,314.56693;1710000.0,308.40237;1713600.0,303.08902;1717200.0,298.4036;1720800.0,294.41138;1724400.0,290.87808;1728000.0,287.8638;1731600.0,285.0431;1735200.0,282.51663;1738800.0,280.3217;1742400.0,278.48904;1746000.0,276.8562;1749600.0,275.3796;1753200.0,274.27292;1756800.0,275.53693;1760400.0,281.6731;1764000.0,290.63303;1767600.0,300.92633;1771200.0,311.10675;1774800.0,319.46756;1778400.0,324.50897;1782000.0,328.27783;1785600.0,328.0472;1789200.0,322.285;1792800.0,315.95557;1796400.0,309.97424;1800000.0,304.87827;1803600.0,300.4446;1807200.0,296.63397;1810800.0,293.3773;1814400.0,290.60907;1818000.0,288.30542;1821600.0,286.3108;1825200.0,284.53403;1828800.0,282.89865;1832400.0,281.51633;1836000.0,280.2255;1839600.0,279.21335;1843200.0,280.6336;1846800.0,286.7613;1850400.0,295.61014;1854000.0,305.6443;1857600.0,315.63776;1861200.0,324.19852;1864800.0,330.55588;1868400.0,334.45377;1872000.0,334.77228;1875600.0,330.10583;1879200.0,322.24033;1882800.0,315.2023;1886400.0,309.17596;1890000.0,304.1898;1893600.0,299.74344;1897200.0,295.63638;1900800.0,292.19876;1904400.0,289.4836;1908000.0,286.77524;1911600.0,284.66714;1915200.0,282.9852;1918800.0,281.59576;1922400.0,280.15836;1926000.0,279.22928;1929600.0,280.11746;1933200.0,285.36356;1936800.0,290.49402;1940400.0,296.4023;1944000.0,301.01474;1947600.0,308.15402;1951200.0,316.91446;1954800.0,320.2564;1958400.0,318.2428;1962000.0,313.91455;1965600.0,309.05057;1969200.0,304.62888;1972800.0,300.63046;1976400.0,297.12744;1980000.0,294.05457;1983600.0,291.49698;1987200.0,289.18658;1990800.0,287.2178;1994400.0,285.58682;1998000.0,284.27887;2001600.0,283.27927;2005200.0,282.5545;2008800.0,281.9809;2012400.0,281.57837;2016000.0,281.43976;2019600.0,282.62943;2023200.0,285.49744;2026800.0,293.42136;2030400.0,298.77002;2034000.0,304.27274;2037600.0,311.72327;2041200.0,315.4395;2044800.0,315.1733;2048400.0,312.69366;2052000.0,308.07297;2055600.0,303.6657;2059200.0,299.94067;2062800.0,296.5981;2066400.0,293.46848;2070000.0,290.66263;2073600.0,288.21024;2077200.0,286.16898;2080800.0,284.16693;2084400.0,282.44843;2088000.0,280.93375;2091600.0,279.93616;2095200.0,278.51794;2098800.0,277.30356;2102400.0,278.15778;2106000.0,283.83783;2109600.0,291.9522;2113200.0,295.9867;2116800.0,300.94003;2120400.0,307.3403;2124000.0,309.49356;2127600.0,313.03934;2131200.0,312.41278;2134800.0,308.29343;2138400.0,303.84195;2142000.0,299.76993;2145600.0,296.56738;2149200.0,294.06973;2152800.0,291.81732;2156400.0,289.77396;2160000.0,287.8503;2163600.0,286.0419;2167200.0,284.28156;2170800.0,282.5579;2174400.0,280.73685;2178000.0,278.9821;2181600.0,277.39532;2185200.0,275.9519;2188800.0,274.9253;2192400.0,275.3408;2196000.0,278.9086;2199600.0,285.01514;2203200.0,293.3188;2206800.0,301.79776;2210400.0,308.09183;2214000.0,311.7217;2217600.0,312.46423;2221200.0,309.07587;2224800.0,302.8095;2228400.0,297.65817;2232000.0,293.4627;2235600.0,289.98816;2239200.0,287.06778;2242800.0,284.6553;2246400.0,282.6363;2250000.0,280.86807;2253600.0,279.34784;2257200.0,278.2669;2260800.0,277.4208;2264400.0,276.87216;2268000.0,276.4662;2271600.0,276.32538;2275200.0,276.83606;2278800.0,279.43707;2282400.0,284.96722;2286000.0,290.0222;2289600.0,297.41006;2293200.0,299.06232;2296800.0,302.20612;2300400.0,303.64322;2304000.0,302.69635;2307600.0,299.87;2311200.0,296.0615;2314800.0,292.65256;2318400.0,289.68417;2322000.0,287.09338;2325600.0,284.7836;2329200.0,282.39465;2332800.0,280.2469;2336400.0,278.42056;2340000.0,276.96298;2343600.0,275.66824;2347200.0,274.57605;2350800.0,273.71002;2354400.0,272.77863;2358000.0,271.55212;2361600.0,271.72064;2365200.0,275.43802;2368800.0,280.75943;2372400.0,288.99234;2376000.0,296.27344;2379600.0,303.55765;2383200.0,307.39975;2386800.0,305.5339;2390400.0,301.14484;2394000.0,297.01663;2397600.0,293.0241;2401200.0,289.66565;2404800.0,286.73306;2408400.0,284.2074;2412000.0,281.93076;2415600.0,279.5822;2419200.0,277.63306;2422800.0,275.69913;2426400.0,273.77414;2430000.0,271.6374;2433600.0,270.02676;2437200.0,268.74905;2440800.0,267.7076;2444400.0,266.5765;2448000.0,267.2917;2451600.0,272.83374;2455200.0,281.1327;2458800.0,290.52063;2462400.0,299.6854;2466000.0,307.66302;2469600.0,313.70682;2473200.0,316.96854;2476800.0,316.87802;2480400.0,312.7775;2484000.0,305.9017;2487600.0,299.60446;2491200.0,294.3113;2494800.0,289.72073;2498400.0,285.80072;2502000.0,282.50052;2505600.0,279.68015;2509200.0,277.23175;2512800.0,275.2997;2516400.0,273.5611;2520000.0,272.0587;2523600.0,270.63733;2527200.0,269.4604;2530800.0,268.51373;2534400.0,268.89822;2538000.0,273.97253;2541600.0,281.83032;2545200.0,290.92078;2548800.0,299.96405;2552400.0,307.71677;2556000.0,313.63217;2559600.0,317.78708;2563200.0,318.13187;2566800.0,314.1402;2570400.0,307.42148;2574000.0,301.61688;2577600.0,296.34036;2581200.0,292.20187;2584800.0,288.686;2588400.0,285.79626;2592000.0,283.21106;2595600.0,280.9927;2599200.0,279.07205;2602800.0,277.41925;2606400.0,275.98267;2610000.0,274.53925;2613600.0,273.04257;2617200.0,271.977;2620800.0,272.11142;2624400.0,273.87195;2628000.0,277.77798;2631600.0,284.579;2635200.0,292.28485;2638800.0,298.7396;2642400.0,300.65582;2646000.0,301.16977;2649600.0,303.29938;2653200.0,301.14523;2656800.0,297.05682;2660400.0,293.01114;2664000.0,289.1239;2667600.0,286.1134;2671200.0,283.51953;2674800.0,281.28278;2678400.0,279.2273;2682000.0,277.59436;2685600.0,275.9618;2689200.0,274.57593;2692800.0,273.31375;2696400.0,272.29184;2700000.0,271.3561;2703600.0,270.6834;2707200.0,272.33603;2710800.0,278.39142;2714400.0,286.94647;2718000.0,296.3919;2721600.0,305.493;2725200.0,313.77045;2728800.0,319.34982;2732400.0,321.8365;2736000.0,321.18927;2739600.0,316.99954;2743200.0,309.76434;2746800.0,303.15482;2750400.0,297.50555;2754000.0,292.72403;2757600.0,288.4399;2761200.0,284.7396;2764800.0,281.7033;2768400.0,279.07452;2772000.0,276.86856;2775600.0,274.731;2779200.0,272.8561;2782800.0,271.05728;2786400.0,269.50107;2790000.0,268.244;2793600.0,269.52252;2797200.0,275.4731;2800800.0,284.28064;2804400.0,294.23022;2808000.0,304.32816;2811600.0,313.6676;2815200.0,320.45392;2818800.0,323.70795;2822400.0,323.37677;2826000.0,318.99332;2829600.0,311.5074;2833200.0,304.52576;2836800.0,298.7758;2840400.0,293.72995;2844000.0,289.43973;2847600.0,285.7452;2851200.0,282.51764;2854800.0,279.7287;2858400.0,277.46115;2862000.0,275.54022;2865600.0,273.7694;2869200.0,272.19745;2872800.0,270.6818;2876400.0,269.40286;2880000.0,270.02658;2883600.0,275.33585;2887200.0,283.04697;2890800.0,291.4688;2894400.0,298.4166;2898000.0,306.44165;2901600.0,310.4333;2905200.0,313.80103;2908800.0,312.75732;2912400.0,308.4901;2916000.0,303.252;2919600.0,298.3462;2923200.0,294.19446;2926800.0,290.5508;2930400.0,287.3218;2934000.0,284.3851;2937600.0,281.74;2941200.0,279.33362;2944800.0,277.18304;2948400.0,275.13312;2952000.0,273.32104;2955600.0,271.52728;2959200.0,269.999;2962800.0,268.59198;2966400.0,269.77695;2970000.0,273.9741;2973600.0,282.3508;2977200.0,291.96857;2980800.0,300.68707;2984400.0,307.3737;2988000.0,313.7565;2991600.0,317.1765;2995200.0,314.87592;2998800.0,309.6254;3002400.0,304.67255;3006000.0,299.90442;3009600.0,295.5801;3013200.0,291.77292;3016800.0,288.57498;3020400.0,285.57996;3024000.0,282.89014;3027600.0,280.36435;3031200.0,278.17712;3034800.0,276.50696;3038400.0,275.1014;3042000.0,273.71893;3045600.0,272.40485;3049200.0,271.45627;3052800.0,272.85965;3056400.0,278.2268;3060000.0,285.9778;3063600.0,296.04233;3067200.0,306.10583;3070800.0,315.06305;3074400.0,321.16278;3078000.0,324.33856;3081600.0,324.40695;3085200.0,320.81134;3088800.0,314.15598;3092400.0,308.23715;3096000.0,303.16345;3099600.0,298.59457;3103200.0,294.69196;3106800.0,291.43732;3110400.0,288.5337;3114000.0,285.90042;3117600.0,283.4357;3121200.0,281.3626;3124800.0,279.55728;3128400.0,278.0767;3132000.0,276.91174;3135600.0,275.98932;3139200.0,276.26633;3142800.0,278.42245;3146400.0,279.66672;3150000.0,285.96964;3153600.0,295.55447;3157200.0,301.3099;3160800.0,308.5787;3164400.0,312.2342;3168000.0,312.87643;3171600.0,309.32538;3175200.0,304.36124;3178800.0,300.00464;3182400.0,296.2385;3186000.0,292.98828;3189600.0,289.98242;3193200.0,287.26025;3196800.0,284.8875;3200400.0,282.77454;3204000.0,280.75504;3207600.0,278.81995;3211200.0,277.20422;3214800.0,275.9067;3218400.0,274.75998;3222000.0,273.75272;3225600.0,272.98465;3229200.0,272.67096;3232800.0,272.66672;3236400.0,272.75974;3240000.0,273.5255;3243600.0,279.1827;3247200.0,284.0344;3250800.0,283.99374;3254400.0,285.1699;3258000.0,283.83218;3261600.0,281.88947;3265200.0,280.09402;3268800.0,278.53558;3272400.0,277.29428;3276000.0,276.30444;3279600.0,276.59686;3283200.0,277.31833;3286800.0,277.03506;3290400.0,276.5232;3294000.0,275.60587;3297600.0,275.0886;3301200.0,275.04523;3304800.0,274.04486;3308400.0,272.1869;3312000.0,270.75497;3315600.0,269.89066;3319200.0,269.62073;3322800.0,269.89047;3326400.0,272.07367;3330000.0,274.54745;3333600.0,274.66357;3337200.0,276.73306;3340800.0,276.90244;3344400.0,275.76157;3348000.0,273.63724;3351600.0,271.5306;3355200.0,269.87265;3358800.0,268.48117;3362400.0,267.1699;3366000.0,265.89435;3369600.0,264.6645;3373200.0,263.6584;3376800.0,262.8332;3380400.0,262.0734;3384000.0,261.48642;3387600.0,260.87457;3391200.0,260.34863;3394800.0,260.22766;3398400.0,261.8157;3402000.0,268.16626;3405600.0,277.62418;3409200.0,287.70142;3412800.0,297.11777;3416400.0,305.444;3420000.0,312.20676;3423600.0,315.74567;3427200.0,316.46683;3430800.0,312.9657;3434400.0,306.79407;3438000.0,301.44406;3441600.0,297.22534;3445200.0,293.82587;3448800.0,291.51514;3452400.0,289.5639;3456000.0,288.18997;3459600.0,286.8825;3463200.0,285.49084;3466800.0,284.29544;3470400.0,283.32803;3474000.0,282.29587;3477600.0,281.30103;3481200.0,280.30792;3484800.0,280.16803;3488400.0,282.64963;3492000.0,286.49213;3495600.0,290.6696;3499200.0,295.3699;3502800.0,298.0699;3506400.0,302.44846;3510000.0,302.12222;3513600.0,299.93784;3517200.0,297.23148;3520800.0,294.10632;3524400.0,291.13766;3528000.0,288.76407;3531600.0,286.5485;3535200.0,284.53116;3538800.0,282.68582;3542400.0,281.164;3546000.0,279.64145;3549600.0,278.12518;3553200.0,276.73267;3556800.0,275.49448;3560400.0,274.59946;3564000.0,273.67038;3567600.0,272.7863;3571200.0,274.6681;3574800.0,280.5211;3578400.0,289.01416;3582000.0,298.7951;3585600.0,307.39313;3589200.0,314.8718;3592800.0,318.118;3596400.0,315.16058;3600000.0,309.31854;3603600.0,303.87802;3607200.0,298.82498;3610800.0,294.45413;3614400.0,290.70383;3618000.0,287.48273;3621600.0,284.74442;3625200.0,282.43417;3628800.0,280.40497;3632400.0,278.6717;3636000.0,277.2669;3639600.0,275.91116;3643200.0,274.47003;3646800.0,272.9968;3650400.0,271.67172;3654000.0,270.49365;3657600.0,271.27808;3661200.0,275.5316;3664800.0,283.2139;3668400.0,292.69672;3672000.0,301.6914;3675600.0,309.2075;3679200.0,315.09723;3682800.0,318.5206;3686400.0,318.82635;3690000.0,315.70584;3693600.0,309.36835;3697200.0,303.34753;3700800.0,298.0444;3704400.0,293.38217;3708000.0,289.12646;3711600.0,285.50058;3715200.0,282.44373;3718800.0,280.10275;3722400.0,278.11987;3726000.0,276.3938;3729600.0,274.97787;3733200.0,273.74487;3736800.0,272.64905;3740400.0,271.9209;3744000.0,274.29688;3747600.0,280.3735;3751200.0,288.86517;3754800.0,298.62823;3758400.0,308.25085;3762000.0,315.89874;3765600.0,321.78543;3769200.0,325.28796;3772800.0,324.49548;3776400.0,320.00433;3780000.0,314.47705;3783600.0,309.04877;3787200.0,304.3404;3790800.0,300.2482;3794400.0,296.711;3798000.0,293.62683;3801600.0,291.167;3805200.0,289.16043;3808800.0,287.33203;3812400.0,285.86835;3816000.0,284.5935;3819600.0,283.15207;3823200.0,281.8767;3826800.0,280.82056;3830400.0,280.5271;3834000.0,281.14313;3837600.0,283.64066;3841200.0,287.9188;3844800.0,293.57416;3848400.0,297.77124;3852000.0,302.28726;3855600.0,306.693;3859200.0,308.49512;3862800.0,306.54395;3866400.0,302.3737;3870000.0,298.3182;3873600.0,294.843;3877200.0,291.78256;3880800.0,289.14426;3884400.0,286.76144;3888000.0,284.7162;3891600.0,282.81003;3895200.0,281.10834;3898800.0,279.34088;3902400.0,277.7207;3906000.0,276.23242;3909600.0,275.0991;3913200.0,274.23785;3916800.0,275.92148;3920400.0,281.471;3924000.0,289.74115;3927600.0,299.36215;3931200.0,308.85468;3934800.0,316.79626;3938400.0,323.15137;3942000.0,327.50696;3945600.0,328.16483;3949200.0,324.86603;3952800.0,318.46613;3956400.0,312.34592;3960000.0,307.03464;3963600.0,302.32257;3967200.0,298.2623;3970800.0,294.81985;3974400.0,292.0199;3978000.0,289.41473;3981600.0,287.40964;3985200.0,286.28912;3988800.0,285.3396;3992400.0,284.3709;3996000.0,283.62112;3999600.0,282.9449;4003200.0,283.05533;4006800.0,284.1098;4010400.0,285.68094;4014000.0,290.8204;4017600.0,299.9132;4021200.0,305.91556;4024800.0,309.93207;4028400.0,314.17908;4032000.0,313.43265;4035600.0,310.71194;4039200.0,306.52908;4042800.0,302.45425;4046400.0,298.7467;4050000.0,295.67648;4053600.0,293.0935;4057200.0,290.84155;4060800.0,288.71704;4064400.0,287.09232;4068000.0,285.4251;4071600.0,283.59464;4075200.0,282.09842;4078800.0,280.96082;4082400.0,280.1455;4086000.0,279.35535;4089600.0,279.54578;4093200.0,282.11273;4096800.0,283.87268;4100400.0,285.53174;4104000.0,287.69806;4107600.0,292.2928;4111200.0,295.86566;4114800.0,299.59894;4118400.0,302.93256;4122000.0,302.56198;4125600.0,300.35486;4129200.0,297.20065;4132800.0,293.86267;4136400.0,292.0896;4140000.0,290.75177;4143600.0,289.67892;4147200.0,288.89932;4150800.0,288.09702;4154400.0,287.286;4158000.0,286.37482;4161600.0,285.84427;4165200.0,285.45575;4168800.0,284.60748;4172400.0,283.5279;4176000.0,285.50034;4179600.0,291.62512;4183200.0,298.5747;4186800.0,304.4664;4190400.0,312.14468;4194000.0,315.22702;4197600.0,316.97183;4201200.0,320.4781;4204800.0,320.43552;4208400.0,316.28598;4212000.0,311.45023;4215600.0,306.78476;4219200.0,302.3974;4222800.0,298.64813;4226400.0,295.4496;4230000.0,292.7175;4233600.0,290.37976;4237200.0,288.41788;4240800.0,286.7291;4244400.0,285.23193;4248000.0,283.85095;4251600.0,282.4366;4255200.0,281.04385;4258800.0,279.69125;4262400.0,279.6512;4266000.0,281.88483;4269600.0,286.8144;4273200.0,294.58582;4276800.0,302.19666;4280400.0,302.121;4284000.0,299.8217;4287600.0,298.10025;4291200.0,299.93716;4294800.0,299.3389;4298400.0,296.16806;4302000.0,292.8756;4305600.0,290.0091;4309200.0,287.49008;4312800.0,285.24747;4316400.0,283.33365;4320000.0,281.84918;4323600.0,280.64133;4327200.0,279.5289;4330800.0,278.63193;4334400.0,277.725;4338000.0,276.85617;4341600.0,275.83054;4345200.0,275.07822;4348800.0,277.301;4352400.0,282.635;4356000.0,290.9042;4359600.0,300.2798;4363200.0,309.30475;4366800.0,316.7197;4370400.0,321.74133;4374000.0,323.99115;4377600.0,320.57147;4381200.0,316.02875;4384800.0,310.3005;4388400.0,305.02542;4392000.0,300.63107;4395600.0,296.87573;4399200.0,293.63797;4402800.0,290.7954;4406400.0,288.3196;4410000.0,286.14642;4413600.0,284.1505;4417200.0,282.3242;4420800.0,280.6498;4424400.0,278.98993;4428000.0,277.3692;4431600.0,276.2007;4435200.0,277.3261;4438800.0,277.08466;4442400.0,277.06146;4446000.0,277.6534;4449600.0,278.74487;4453200.0,279.85495;4456800.0,280.653;4460400.0,280.96942;4464000.0,280.83435;4467600.0,280.24817;4471200.0,279.16928;4474800.0,278.0788;4478400.0,277.0835;4482000.0,276.38116;4485600.0,275.68298;4489200.0,274.8744;4492800.0,274.09235;4496400.0,273.31088;4500000.0,272.2747;4503600.0,271.4066;4507200.0,270.811;4510800.0,270.34937;4514400.0,269.89847;4518000.0,269.5461;4521600.0,269.438;4525200.0,269.82956;4528800.0,270.59024;4532400.0,271.70062;4536000.0,272.7769;4539600.0,273.97095;4543200.0,275.00305;4546800.0,275.5925;4550400.0,275.61536;4554000.0,275.3262;4557600.0,274.64545;4561200.0,273.9587;4564800.0,273.27826;4568400.0,272.62457;4572000.0,272.042;4575600.0,271.54224;4579200.0,270.95932;4582800.0,270.26212;4586400.0,269.45648;4590000.0,268.74576;4593600.0,267.99704;4597200.0,267.1491;4600800.0,266.27313;4604400.0,265.60864;4608000.0,267.9521;4611600.0,274.00507;4615200.0,282.1998;4618800.0,291.4194;4622400.0,300.4311;4626000.0,308.1952;4629600.0,313.9978;4633200.0,317.15054;4636800.0,316.57535;4640400.0,312.18884;4644000.0,307.00644;4647600.0,302.07922;4651200.0,297.75574;4654800.0,294.28796;4658400.0,291.29565;4662000.0,288.73145;4665600.0,287.0301;4669200.0,285.41888;4672800.0,283.81763;4676400.0,282.81372;4680000.0,281.8839;4683600.0,281.27902;4687200.0,280.57928;4690800.0,280.2425;4694400.0,282.54837;4698000.0,287.35767;4701600.0,293.22757;4705200.0,299.57727;4708800.0,308.21292;4712400.0,315.61267;4716000.0,319.93204;4719600.0,320.44916;4723200.0,320.3938;4726800.0,316.9829;4730400.0,312.45697;4734000.0,308.01932;4737600.0,302.6215;4741200.0,297.03183;4744800.0,292.17294;4748400.0,288.082;4752000.0,284.67624;4755600.0,281.70892;4759200.0,279.09286;4762800.0,276.86182;4766400.0,274.8924;4770000.0,273.12836;4773600.0,271.6047;4777200.0,270.56677;4780800.0,271.22968;4784400.0,274.57162;4788000.0,279.2121;4791600.0,282.95624;4795200.0,287.9913;4798800.0,297.12524;4802400.0,304.1885;4806000.0,306.6649;4809600.0,308.44037;4813200.0,306.8378;4816800.0,302.0481;4820400.0,297.09952;4824000.0,292.9242;4827600.0,289.35254;4831200.0,286.16214;4834800.0,283.31046;4838400.0,280.89838;4842000.0,278.81985;4845600.0,277.11038;4849200.0,275.55148;4852800.0,274.14468;4856400.0,272.9178;4860000.0,271.90933;4863600.0,271.32077;4867200.0,272.67215;4870800.0,277.6935;4874400.0,286.09375;4878000.0,294.46576;4881600.0,304.06033;4885200.0,312.5072;4888800.0,316.8947;4892400.0,317.85767;4896000.0,317.95367;4899600.0,315.69873;4903200.0,311.0199;4906800.0,305.78323;4910400.0,301.2862;4914000.0,297.44;4917600.0,294.058;4921200.0,291.27466;4924800.0,288.76343;4928400.0,286.74084;4932000.0,284.9939;4935600.0,283.4147;4939200.0,282.0803;4942800.0,280.9053;4946400.0,279.96838;4950000.0,279.42184;4953600.0,279.9288;4957200.0,281.39523;4960800.0,283.6143;4964400.0,288.2279;4968000.0,296.76843;4971600.0,304.1172;4975200.0,310.5304;4978800.0,311.9955;4982400.0,310.65826;4986000.0,308.39822;4989600.0,305.10526;4993200.0,301.87088;4996800.0,298.94376;5000400.0,296.27133;5004000.0,293.93356;5007600.0,291.9051;5011200.0,289.99216;5014800.0,288.17648;5018400.0,286.7138;5022000.0,285.49152;5025600.0,284.212;5029200.0,282.80203;5032800.0,281.62283;5036400.0,280.81235;5040000.0,280.6801;5043600.0,282.903;5047200.0,286.60013;5050800.0,289.19522;5054400.0,293.54022;5058000.0,297.5523;5061600.0,302.82245;5065200.0,300.88736;5068800.0,297.48068;5072400.0,294.33377;5076000.0,291.55347;5079600.0,289.15112;5083200.0,287.1083;5086800.0,285.36597;5090400.0,284.0055;5094000.0,282.99084;5097600.0,282.25284;5101200.0,281.45523;5104800.0,281.40866;5108400.0,281.34772;5112000.0,281.41724;5115600.0,281.24503;5119200.0,280.72537;5122800.0,280.361;5126400.0,280.8491;5130000.0,283.27368;5133600.0,289.71262;5137200.0,296.2928;5140800.0,305.6052;5144400.0,312.2224;5148000.0,314.7591;5151600.0,312.80167;5155200.0,310.00787;5158800.0,306.892;5162400.0,303.80618;5166000.0,300.01498;5169600.0,295.99457;5173200.0,292.46863;5176800.0,289.3719;5180400.0,286.61786;5184000.0,284.25555;5187600.0,282.21646;5191200.0,280.385;5194800.0,278.79047;5198400.0,277.46848;5202000.0,276.32553;5205600.0,275.3411;5209200.0,274.5374;5212800.0,274.63052;5216400.0,274.85794;5220000.0,276.60144;5223600.0,280.5973;5227200.0,287.4161;5230800.0,293.96912;5234400.0,298.98642;5238000.0,301.22037;5241600.0,300.35803;5245200.0,298.205;5248800.0,295.52643;5252400.0,292.44955;5256000.0,289.8122;5259600.0,287.514;5263200.0,285.6958;5266800.0,284.2445;5270400.0,283.13965;5274000.0,282.5731;5277600.0,281.53052;5281200.0,280.38824;5284800.0,279.3532;5288400.0,279.12854;5292000.0,279.26657;5295600.0,278.87424;5299200.0,281.33652;5302800.0,286.96985;5306400.0,294.59766;5310000.0,303.62686;5313600.0,312.31094;5317200.0,319.16055;5320800.0,321.23486;5324400.0,322.14282;5328000.0,322.4397;5331600.0,318.52115;5335200.0,314.03162;5338800.0,309.83566;5342400.0,306.32104;5346000.0,303.11575;5349600.0,300.32974;5353200.0,297.97314;5356800.0,295.94724;5360400.0,294.25342;5364000.0,292.67313;5367600.0,291.37988;5371200.0,290.18018;5374800.0,288.96503;5378400.0,287.6451;5382000.0,287.28128;5385600.0,289.5281;5389200.0,293.53317;5392800.0,300.05927;5396400.0,303.31955;5400000.0,309.10788;5403600.0,314.90082;5407200.0,316.94678;5410800.0,317.38318;5414400.0,316.284;5418000.0,314.3629;5421600.0,311.14163;5425200.0,307.93497;5428800.0,305.0692;5432400.0,302.62762;5436000.0,300.51053;5439600.0,298.66327;5443200.0,297.08627;5446800.0,295.68842;5450400.0,294.4193;5454000.0,293.23044;5457600.0,292.11496;5461200.0,291.10287;5464800.0,290.3253;5468400.0,289.86655;5472000.0,289.5509;5475600.0,289.42886;5479200.0,289.74588;5482800.0,290.41202;5486400.0,291.2506;5490000.0,294.10803;5493600.0,293.54367;5497200.0,291.8595;5500800.0,290.40863;5504400.0,288.99008;5508000.0,287.36722;5511600.0,285.8099;5515200.0,284.4479;5518800.0,283.25015;5522400.0,282.24484;5526000.0,281.31915;5529600.0,280.54297;5533200.0,279.8835;5536800.0,279.305;5540400.0,278.7596;5544000.0,278.153;5547600.0,277.4396;5551200.0,276.63214;5554800.0,276.05298;5558400.0,276.19608;5562000.0,277.33746;5565600.0,279.46103;5569200.0,286.0079;5572800.0,294.70792;5576400.0,303.05496;5580000.0,309.35733;5583600.0,312.4771;5587200.0,313.23355;5590800.0,310.59125;5594400.0,305.6992;5598000.0,300.94534;5601600.0,296.9467;5605200.0,293.4305;5608800.0,290.42365;5612400.0,287.8854;5616000.0,285.6066;5619600.0,283.69235;5623200.0,281.98923;5626800.0,280.30792;5630400.0,278.95914;5634000.0,277.7125;5637600.0,276.52686;5641200.0,275.7817;5644800.0,276.4804;5648400.0,281.0379;5652000.0,288.9397;5655600.0,297.4508;5659200.0,305.27045;5662800.0,312.54742;5666400.0,317.92627;5670000.0,320.59354;5673600.0,320.33542;5677200.0,317.06497;5680800.0,311.5311;5684400.0,305.81467;5688000.0,301.03012;5691600.0,296.99203;5695200.0,293.5987;5698800.0,290.50024;5702400.0,287.82272;5706000.0,285.60718;5709600.0,283.9648;5713200.0,282.5358;5716800.0,281.1761;5720400.0,279.95297;5724000.0,279.0364;5727600.0,278.71466;5731200.0,280.92517;5734800.0,286.08493;5738400.0,293.45248;5742000.0,301.5993;5745600.0,307.8652;5749200.0,312.35016;5752800.0,316.95718;5756400.0,318.7687;5760000.0,318.46326;5763600.0,314.81573;5767200.0,309.53134;5770800.0,304.3564;5774400.0,299.95212;5778000.0,296.21457;5781600.0,292.8626;5785200.0,289.74203;5788800.0,286.7002;5792400.0,284.24484;5796000.0,282.17465;5799600.0,280.32217;5803200.0,278.67297;5806800.0,277.3013;5810400.0,276.24155;5814000.0,275.75995;5817600.0,277.89456;5821200.0,283.39352;5824800.0,291.11658;5828400.0,300.10873;5832000.0,309.0291;5835600.0,316.8296;5839200.0,322.49323;5842800.0,325.33936;5846400.0,325.3815;5850000.0,322.2102;5853600.0,316.82654;5857200.0,311.4533;5860800.0,306.85104;5864400.0,302.78683;5868000.0,299.23267;5871600.0,296.33426;5875200.0,293.9464;5878800.0,292.0167;5882400.0,290.4917;5886000.0,289.25247;5889600.0,288.15646;5893200.0,287.09628;5896800.0,286.25378;5900400.0,285.87537;5904000.0,287.0514;5907600.0,291.5196;5911200.0,298.28113;5914800.0,306.3121;5918400.0,314.52875;5922000.0,320.68668;5925600.0,324.7204;5929200.0,326.74014;5932800.0,326.53195;5936400.0,323.277;5940000.0,318.2685;5943600.0,313.32877;5947200.0,308.8649;5950800.0,305.07614;5954400.0,301.84427;5958000.0,299.13684;5961600.0,296.73172;5965200.0,294.49976;5968800.0,292.63373;5972400.0,291.03683;5976000.0,289.43518;5979600.0,288.2223;5983200.0,287.02164;5986800.0,286.12912;5990400.0,288.19397;5994000.0,293.3904;5997600.0,300.0941;6001200.0,307.8295;6004800.0,314.44354;6008400.0,317.83453;6012000.0,322.7534;6015600.0,321.89005;6019200.0,319.68842;6022800.0,316.06427;6026400.0,311.73245;6030000.0,307.23618;6033600.0,302.82257;6037200.0,298.87625;6040800.0,295.79434;6044400.0,293.33813;6048000.0,291.04865;6051600.0,288.85345;6055200.0,287.11276;6058800.0,285.35266;6062400.0,283.66168;6066000.0,282.10693;6069600.0,280.6443;6073200.0,279.70572;6076800.0,281.31967;6080400.0,286.1276;6084000.0,293.361;6087600.0,301.5841;6091200.0,309.41898;6094800.0,315.99072;6098400.0,320.56915;6102000.0,322.139;6105600.0,320.41034;6109200.0,316.0229;6112800.0,310.60538;6116400.0,305.2776;6120000.0,300.7266;6123600.0,296.86707;6127200.0,293.49258;6130800.0,290.54846;6134400.0,287.96082;6138000.0,285.6176;6141600.0,283.51782;6145200.0,281.67722;6148800.0,280.01266;6152400.0,278.5966;6156000.0,277.35068;6159600.0,276.81732;6163200.0,279.39572;6166800.0,285.0792;6170400.0,292.6549;6174000.0,301.1213;6177600.0,308.94635;6181200.0,315.25223;6184800.0,319.79697;6188400.0,321.29404;6192000.0,320.7889;6195600.0,317.7603;6199200.0,312.65646;6202800.0,307.34055;6206400.0,302.6166;6210000.0,298.46588;6213600.0,295.02045;6217200.0,292.05948;6220800.0,289.47638;6224400.0,287.3747;6228000.0,285.64346;6231600.0,283.9955;6235200.0,282.27463;6238800.0,280.6993;6242400.0,279.19604;6246000.0,277.98605;6249600.0,277.6544;6253200.0,278.19907;6256800.0,281.33142;6260400.0,285.08044;6264000.0,290.2232;6267600.0,294.35254;6271200.0,299.14786;6274800.0,300.6232;6278400.0,300.38184;6282000.0,298.4161;6285600.0,295.49567;6289200.0,292.54626;6292800.0,289.9684;6296400.0,287.69827;6300000.0,285.65222;6303600.0,283.90543;6307200.0,282.37564;6310800.0,281.15976;6314400.0,280.12515;6318000.0,279.14648;6321600.0,278.21875;6325200.0,277.34738;6328800.0,276.5848;6332400.0,276.11053;6336000.0,276.27597;6339600.0,276.91116;6343200.0,278.02478;6346800.0,279.76913;6350400.0,284.05417;6354000.0,286.3313;6357600.0,290.0438;6361200.0,293.86307;6364800.0,295.03354;6368400.0,294.20163;6372000.0,292.34195;6375600.0,289.89847;6379200.0,287.5826;6382800.0,285.64075;6386400.0,283.75708;6390000.0,282.005;6393600.0,280.35532;6397200.0,278.95416;6400800.0,277.83896;6404400.0,276.90283;6408000.0,276.21878;6411600.0,275.64352;6415200.0,275.10196;6418800.0,274.66635;6422400.0,274.61502;6426000.0,275.00388;6429600.0,275.7784;6433200.0,277.36288;6436800.0,279.0225;6440400.0,280.3561;6444000.0,281.46094;6447600.0,282.12683;6451200.0,282.36304;6454800.0,282.70624;6458400.0,282.27505;6462000.0,281.17862;6465600.0,279.96884;6469200.0,278.86713;6472800.0,277.78674;6476400.0,276.6812;6480000.0,275.79922;6483600.0,275.06454;6487200.0,274.36102;6490800.0,273.5662;6494400.0,272.72748;6498000.0,272.0844;6501600.0,271.40707;6505200.0,271.44818;6508800.0,274.4667;6512400.0,280.74606;6516000.0,289.0038;6519600.0,297.02582;6523200.0,305.7266;6526800.0,313.5012;6530400.0,319.326;6534000.0,321.95175;6537600.0,321.01923;6541200.0,317.4788;6544800.0,313.02133;6548400.0,308.21893;6552000.0,303.93893;6555600.0,300.19052;6559200.0,296.71185;6562800.0,293.4967;6566400.0,290.66943;6570000.0,287.98895;6573600.0,285.6418;6577200.0,283.53217;6580800.0,281.8486;6584400.0,280.37927;6588000.0,279.08664;6591600.0,278.70923;6595200.0,281.4139;6598800.0,287.76303;6602400.0,295.25394;6606000.0,302.52853;6609600.0,311.09692;6613200.0,318.40808;6616800.0,323.63763;6620400.0,326.25723;6624000.0,325.58417;6627600.0,322.31155;6631200.0,317.48065;6634800.0,312.10776;6638400.0,307.1566;6642000.0,302.87582;6645600.0,299.2435;6649200.0,296.01114;6652800.0,293.27545;6656400.0,291.0482;6660000.0,289.2737;6663600.0,287.7273;6667200.0,286.3013;6670800.0,284.86975;6674400.0,283.42868;6678000.0,282.50586;6681600.0,284.0894;6685200.0,288.7583;6688800.0,295.65872;6692400.0,303.1693;6696000.0,310.40286;6699600.0,316.628;6703200.0,321.0988;6706800.0,322.75626;6710400.0,321.68866;6714000.0,317.93604;6717600.0,313.30276;6721200.0,308.61896;6724800.0,304.59167;6728400.0,301.27585;6732000.0,298.54544;6735600.0,296.27887;6739200.0,294.15186;6742800.0,292.47543;6746400.0,291.19443;6750000.0,289.99533;6753600.0,288.9352;6757200.0,288.00165;6760800.0,287.16803;6764400.0,286.64346;6768000.0,287.5884;6771600.0,291.60608;6775200.0,297.4019;6778800.0,303.8842;6782400.0,307.77698;6786000.0,314.6732;6789600.0,319.593;6793200.0,322.15543;6796800.0,321.93875;6800400.0,318.9682;6804000.0,314.32013;6807600.0,309.37317;6811200.0,304.96872;6814800.0,301.0586;6818400.0,297.7295;6822000.0,294.59625;6825600.0,291.77792;6829200.0,289.15182;6832800.0,286.7216;6836400.0,284.57843;6840000.0,282.76862;6843600.0,281.39462;6847200.0,280.24628;6850800.0,279.74646;6854400.0,281.90002;6858000.0,286.92044;6861600.0,292.52643;6865200.0,299.78592;6868800.0,306.43753;6872400.0,311.55594;6876000.0,309.63235;6879600.0,307.27106;6883200.0,304.98697;6886800.0,302.5355;6890400.0,300.3013;6894000.0,297.71457;6897600.0,295.21152;6901200.0,292.97202;6904800.0,291.0462;6908400.0,289.10498;6912000.0,287.18582;6915600.0,285.63574;6919200.0,284.38837;6922800.0,283.35843;6926400.0,282.2244;6930000.0,281.13892;6933600.0,280.4236;6937200.0,280.13516;6940800.0,281.93768;6944400.0,284.0109;6948000.0,289.15887;6951600.0,295.93826;6955200.0,301.2674;6958800.0,305.01044;6962400.0,306.778;6966000.0,307.7052;6969600.0,307.51367;6973200.0,305.28775;6976800.0,302.25394;6980400.0,299.02655;6984000.0,296.15244;6987600.0,293.7506;6991200.0,291.64453;6994800.0,289.774;6998400.0,288.19907;7002000.0,286.85242;7005600.0,285.6366;7009200.0,284.4934;7012800.0,283.3013;7016400.0,282.0566;7020000.0,280.9139;7023600.0,280.415;7027200.0,281.79956;7030800.0,285.81543;7034400.0,291.8555;7038000.0,300.07648;7041600.0,307.65442;7045200.0,313.76596;7048800.0,317.9417;7052400.0,319.10056;7056000.0,318.6296;7059600.0,316.1882;7063200.0,311.9495;7066800.0,307.83673;7070400.0,304.22632;7074000.0,300.87482;7077600.0,297.81906;7081200.0,294.9258;7084800.0,291.99213;7088400.0,289.49277;7092000.0,287.4127;7095600.0,285.44687;7099200.0,283.9811;7102800.0,282.6931;7106400.0,281.47888;7110000.0,281.10257;7113600.0,283.571;7117200.0,288.9648;7120800.0,296.23013;7124400.0,304.64847;7128000.0,313.02246;7131600.0,320.1539;7135200.0,324.96194;7138800.0,327.392;7142400.0,326.98972;7146000.0,323.36584;7149600.0,318.8581;7153200.0,314.34644;7156800.0,310.04636;7160400.0,306.35443;7164000.0,302.9404;7167600.0,299.86746;7171200.0,297.18378;7174800.0,294.69324;7178400.0,292.50473;7182000.0,290.576;7185600.0,288.81708;7189200.0,287.3599;7192800.0,285.9279;7196400.0,285.2022;7200000.0,286.5965;7203600.0,291.14212;7207200.0,297.58316;7210800.0,305.1383;7214400.0,312.15973;7218000.0,316.0942;7221600.0,319.07773;7225200.0,319.4557;7228800.0,316.90414;7232400.0,314.48538;7236000.0,311.34247;7239600.0,307.95654;7243200.0,304.62732;7246800.0,301.6776;7250400.0,298.97165;7254000.0,296.66278;7257600.0,294.16248;7261200.0,292.18552;7264800.0,290.92618;7268400.0,289.88898;7272000.0,288.87717;7275600.0,287.88974;7279200.0,286.94675;7282800.0,286.6897;7286400.0,288.65295;7290000.0,292.7337;7293600.0,298.7539;7297200.0,305.6641;7300800.0,311.91885;7304400.0,313.39566;7308000.0,315.65402;7311600.0,316.7033;7315200.0,313.61652;7318800.0,310.0409;7322400.0,306.4025;7326000.0,302.99127;7329600.0,299.97144;7333200.0,297.35236;7336800.0,294.58505;7340400.0,291.62695;7344000.0,289.15146;7347600.0,287.0845;7351200.0,285.2667;7354800.0,283.7053;7358400.0,282.48303;7362000.0,281.37817;7365600.0,280.26953;7369200.0,279.5765;7372800.0,280.1884;7376400.0,283.14905;7380000.0,288.466;7383600.0,296.06662;7387200.0,303.4552;7390800.0,309.522;7394400.0,314.09766;7398000.0,316.45944;7401600.0,315.77277;7405200.0,312.75232;7408800.0,308.7373;7412400.0,304.4498;7416000.0,300.5504;7419600.0,297.04446;7423200.0,293.8755;7426800.0,291.3112;7430400.0,289.04266;7434000.0,286.82266;7437600.0,284.7381;7441200.0,282.89914;7444800.0,281.3413;7448400.0,280.01788;7452000.0,278.78702;7455600.0,278.4901;7459200.0,280.49683;7462800.0,285.3273;7466400.0,292.23593;7470000.0,299.89783;7473600.0,307.21765;7477200.0,313.27014;7480800.0,317.7764;7484400.0,319.9109;7488000.0,319.46356;7491600.0,316.4591;7495200.0,312.1133;7498800.0,307.33636;7502400.0,302.9863;7506000.0,299.48376;7509600.0,296.2847;7513200.0,293.36047;7516800.0,290.55466;7520400.0,288.1616;7524000.0,286.06412;7527600.0,284.35434;7531200.0,282.92874;7534800.0,281.80072;7538400.0,281.02203;7542000.0,280.72708;7545600.0,281.15076;7549200.0,282.49472;7552800.0,286.16672;7556400.0,287.2215;7560000.0,287.66678;7563600.0,287.2271;7567200.0,286.55124;7570800.0,286.52185;7574400.0,286.46387;7578000.0,286.25998;7581600.0,285.2085;7585200.0,283.50623;7588800.0,282.00674;7592400.0,280.6476;7596000.0,279.35745;7599600.0,278.23865;7603200.0,277.30682;7606800.0,276.45972;7610400.0,275.79657;7614000.0,275.2053;7617600.0,274.55594;7621200.0,273.8103;7624800.0,273.15894;7628400.0,273.45056;7632000.0,275.7714;7635600.0,280.41168;7639200.0,287.12314;7642800.0,295.24957;7646400.0,303.39938;7650000.0,310.34546;7653600.0,315.6233;7657200.0,318.40805;7660800.0,318.08835;7664400.0,315.35034;7668000.0,311.43167;7671600.0,307.07047;7675200.0,302.81573;7678800.0,299.02472;7682400.0,295.66376;7686000.0,292.75708;7689600.0,290.07547;7693200.0,287.69028;7696800.0,285.6487;7700400.0,283.979;7704000.0,282.6251;7707600.0,281.3874;7711200.0,280.3027;7714800.0,280.2967;7718400.0,282.58426;7722000.0,287.3256;7725600.0,294.18378;7729200.0,302.1571;7732800.0,309.8189;7736400.0,316.44406;7740000.0,321.45633;7743600.0,324.1384;7747200.0,323.9107;7750800.0,321.08984;7754400.0,317.03766;7758000.0,312.5206;7761600.0,308.30722;7765200.0,304.52515;7768800.0,301.18057;7772400.0,298.16498;7776000.0,295.43018;7779600.0,292.71597;7783200.0,290.58475;7786800.0,288.3527;7790400.0,286.3073;7794000.0,284.51447;7797600.0,283.0411;7801200.0,282.2746;7804800.0,281.74396;7808400.0,281.575;7812000.0,281.6392;7815600.0,282.13425;7819200.0,282.67706;7822800.0,283.07104;7826400.0,284.61224;7830000.0,284.59674;7833600.0,284.1595;7837200.0,283.44156;7840800.0,282.3896;7844400.0,281.16064;7848000.0,280.0458;7851600.0,279.00525;7855200.0,277.959;7858800.0,277.0029;7862400.0,276.03516;7866000.0,275.25818;7869600.0,274.57306;7873200.0,273.9305;7876800.0,273.29395;7880400.0,272.7778;7884000.0,272.18295;7887600.0,271.97226;7891200.0,272.19156;7894800.0,272.6894;7898400.0,273.3892;7902000.0,274.2053;7905600.0,275.3023;7909200.0,276.16412;7912800.0,276.59906;7916400.0,276.63074;7920000.0,276.3564;7923600.0,275.91992;7927200.0,275.13422;7930800.0,274.26877;7934400.0,273.42627;7938000.0,272.68372;7941600.0,271.94412;7945200.0,271.3565;7948800.0,270.8476;7952400.0,270.50125;7956000.0,270.17804;7959600.0,269.50894;7963200.0,269.07892;7966800.0,268.90305;7970400.0,268.7066;7974000.0,268.83447;7977600.0,269.30798;7981200.0,270.15973;7984800.0,271.31046;7988400.0,273.8339;7992000.0,277.76022;7995600.0,283.02005;7999200.0,288.18784;8002800.0,291.56473;8006400.0,292.9097;8010000.0,292.15805;8013600.0,290.72696;8017200.0,288.36658;8020800.0,285.94568;8024400.0,283.7381;8028000.0,281.74432;8031600.0,279.96564;8035200.0,278.49054;8038800.0,277.41592;8042400.0,276.54034;8046000.0,275.7425;8049600.0,275.2021;8053200.0,274.74615;8056800.0,273.9422;8060400.0,274.42093;8064000.0,276.6734;8067600.0,280.93448;8071200.0,287.28436;8074800.0,294.8875;8078400.0,301.88638;8082000.0,307.27408;8085600.0,311.41907;8089200.0,312.28894;8092800.0,310.16467;8096400.0,306.63193;8100000.0,302.7326;8103600.0,298.79175;8107200.0,295.31403;8110800.0,292.38287;8114400.0,289.81824;8118000.0,287.82617;8121600.0,286.04745;8125200.0,284.49692;8128800.0,283.02805;8132400.0,281.79108;8136000.0,280.748;8139600.0,279.8643;8143200.0,279.10873;8146800.0,279.146;8150400.0,280.25018;8154000.0,282.69385;8157600.0,288.91833;8161200.0,296.21127;8164800.0,303.37045;8168400.0,309.72382;8172000.0,314.25854;8175600.0,316.3894;8179200.0,316.30127;8182800.0,314.06445;8186400.0,310.21014;8190000.0,305.52484;8193600.0,301.28796;8197200.0,297.38565;8200800.0,293.95572;8204400.0,291.06082;8208000.0,288.4982;8211600.0,286.67297;8215200.0,285.10428;8218800.0,283.7511;8222400.0,282.43344;8226000.0,281.22946;8229600.0,280.12912;8233200.0,280.31992;8236800.0,282.50516;8240400.0,286.97678;8244000.0,293.12173;8247600.0,300.27353;8251200.0,307.31342;8254800.0,313.45947;8258400.0,318.09607;8262000.0,320.14807;8265600.0,319.80923;8269200.0,317.4678;8272800.0,314.23657;8276400.0,310.04932;8280000.0,306.25854;8283600.0,302.98364;8287200.0,300.02008;8290800.0,297.32483;8294400.0,294.98364;8298000.0,292.84637;8301600.0,290.8481;8305200.0,289.337;8308800.0,287.95862;8312400.0,286.4338;8316000.0,284.89218;8319600.0,285.04184;8323200.0,287.56482;8326800.0,291.81778;8330400.0,297.5282;8334000.0,304.313;8337600.0,311.1251;8341200.0,316.81815;8344800.0,320.91385;8348400.0,322.86868;8352000.0,322.19592;8355600.0,319.41342;8359200.0,315.67853;8362800.0,311.34164;8366400.0,307.52115;8370000.0,304.17368;8373600.0,301.13773;8377200.0,298.3393;8380800.0,295.94537;8384400.0,294.43805;8388000.0,292.6833;8391600.0,291.11365;8395200.0,289.7021;8398800.0,288.4875;8402400.0,287.44846;8406000.0,287.2987;8409600.0,287.4575;8413200.0,290.4425;8416800.0,296.13995;8420400.0,302.97806;8424000.0,309.63556;8427600.0,315.0472;8431200.0,318.8542;8434800.0,320.35623;8438400.0,319.573;8442000.0,316.8487;8445600.0,313.29114;8449200.0,308.79465;8452800.0,304.26907;8456400.0,300.04474;8460000.0,296.25748;8463600.0,293.1391;8467200.0,290.44366;8470800.0,288.1505;8474400.0,285.80328;8478000.0,284.0242;8481600.0,282.53384;8485200.0,281.4674;8488800.0,280.50528;8492400.0,281.2385;8496000.0,283.72055;8499600.0,288.03537;8503200.0,293.9861;8506800.0,300.7244;8510400.0,307.54532;8514000.0,312.20496;8517600.0,312.46335;8521200.0,314.06168;8524800.0,313.65808;8528400.0,311.2798;8532000.0,307.36707;8535600.0,303.26904;8539200.0,299.65;8542800.0,296.15244;8546400.0,293.53513;8550000.0,291.2572;8553600.0,289.20938;8557200.0,287.46436;8560800.0,285.9535;8564400.0,284.61307;8568000.0,283.48804;8571600.0,282.5222;8575200.0,281.64145;8578800.0,282.08698;8582400.0,284.08072;8586000.0,287.12463;8589600.0,292.20486;8593200.0,298.29074;8596800.0,304.70142;8600400.0,309.83508;8604000.0,313.48553;8607600.0,314.91537;8611200.0,312.56113;8614800.0,309.97742;8618400.0,306.49536;8622000.0,302.2739;8625600.0,298.23926;8629200.0,294.73798;8632800.0,291.31006;8636400.0,288.15714;8640000.0,285.56873;8643600.0,283.23163;8647200.0,281.1618;8650800.0,279.01822;8654400.0,277.08917;8658000.0,275.45447;8661600.0,274.0956;8665200.0,274.96075;8668800.0,277.71954;8672400.0,282.15607;8676000.0,288.3748;8679600.0,295.39734;8683200.0,302.55164;8686800.0,308.66223;8690400.0,313.41257;8694000.0,314.53217;8697600.0,314.60648;8701200.0,312.38715;8704800.0,309.23105;8708400.0,305.1642;8712000.0,301.59543;8715600.0,298.41672;8719200.0,295.83643;8722800.0,292.93808;8726400.0,290.1185;8730000.0,287.66977;8733600.0,285.94296;8737200.0,284.62302;8740800.0,283.21014;8744400.0,282.00287;8748000.0,280.9313;8751600.0,281.64465;8755200.0,284.06314;8758800.0,288.3474;8762400.0,294.48962;8766000.0,301.94742;8769600.0,309.39944;8773200.0,315.25778;8776800.0,319.57025;8780400.0,321.71753;8784000.0,320.1857;8787600.0,317.0974;8791200.0,313.99393;8794800.0,310.02066;8798400.0,306.36072;8802000.0,303.0567;8805600.0,299.96945;8809200.0,297.02667;8812800.0,294.47345;8816400.0,292.13995;8820000.0,290.26813;8823600.0,288.40613;8827200.0,286.6744;8830800.0,285.34763;8834400.0,284.26974;8838000.0,284.93497;8841600.0,287.2556;8845200.0,291.2401;8848800.0,296.68002;8852400.0,298.9098;8856000.0,302.99762;8859600.0,308.36035;8863200.0,310.19427;8866800.0,308.59137;8870400.0,306.2102;8874000.0,304.7046;8877600.0,303.42535;8881200.0,300.8096;8884800.0,297.8291;8888400.0,295.61075;8892000.0,293.70523;8895600.0,292.01456;8899200.0,290.51346;8902800.0,289.2369;8906400.0,288.1709;8910000.0,287.09518;8913600.0,286.14236;8917200.0,285.23587;8920800.0,284.18008;8924400.0,283.91202;8928000.0,283.6895;8931600.0,283.1953;8935200.0,282.79874;8938800.0,283.10742;8942400.0,284.51257;8946000.0,287.90744;8949600.0,288.53622;8953200.0,289.9599;8956800.0,289.66678;8960400.0,288.92764;8964000.0,287.4614;8967600.0,285.8277;8971200.0,284.4083;8974800.0,283.1195;8978400.0,281.77347;8982000.0,280.5593;8985600.0,279.51434;8989200.0,278.42093;8992800.0,277.43744;8996400.0,276.4711;9000000.0,275.6089;9003600.0,274.6682;9007200.0,273.98807;9010800.0,273.75195;9014400.0,274.06586;9018000.0,275.82977;9021600.0,278.33255;9025200.0,282.3431;9028800.0,285.96234;9032400.0,289.17166;9036000.0,291.79764;9039600.0,291.5913;9043200.0,292.64212;9046800.0,290.93286;9050400.0,288.87994;9054000.0,286.64188;9057600.0,284.62753;9061200.0,282.87302;9064800.0,281.14172;9068400.0,279.6509;9072000.0,278.36514;9075600.0,277.23975;9079200.0,276.3159;9082800.0,275.35168;9086400.0,274.534;9090000.0,273.7512;9093600.0,273.04852;9097200.0,274.0012;9100800.0,276.4544;9104400.0,279.04538;9108000.0,282.88535;9111600.0,284.35028;9115200.0,284.36557;9118800.0,284.58862;9122400.0,283.99048;9126000.0,283.1398;9129600.0,282.2736;9133200.0,281.6827;9136800.0,280.778;9140400.0,279.65186;9144000.0,278.67755;9147600.0,277.74603;9151200.0,276.8129;9154800.0,275.88126;9158400.0,274.72684;9162000.0,273.60342;9165600.0,272.56793;9169200.0,271.81644;9172800.0,271.17032;9176400.0,270.5778;9180000.0,270.11496;9183600.0,271.12482;9187200.0,273.4239;9190800.0,277.07803;9194400.0,282.47372;9198000.0,289.11157;9201600.0,295.90027;9205200.0,301.7655;9208800.0,306.3688;9212400.0,308.72626;9216000.0,308.92642;9219600.0,307.4481;9223200.0,305.20007;9226800.0,301.82462;9230400.0,298.4467;9234000.0,295.4588;9237600.0,292.94528;9241200.0,290.93698;9244800.0,289.25528;9248400.0,287.71167;9252000.0,286.46945;9255600.0,285.3558;9259200.0,284.29797;9262800.0,283.3499;9266400.0,282.767;9270000.0,284.26428;9273600.0,287.19766;9277200.0,291.36615;9280800.0,296.81564;9284400.0,303.26834;9288000.0,309.79132;9291600.0,314.36908;9295200.0,316.55136;9298800.0,317.91684;9302400.0,316.4243;9306000.0,313.82962;9309600.0,311.26645;9313200.0,308.15634;9316800.0,304.8968;9320400.0,301.93967;9324000.0,299.25232;9327600.0,296.75974;9331200.0,294.64563;9334800.0,292.73386;9338400.0,291.02472;9342000.0,289.56006;9345600.0,288.24405;9349200.0,287.18738;9352800.0,286.60248;9356400.0,287.42694;9360000.0,289.8382;9363600.0,294.07855;9367200.0,299.68063;9370800.0,305.29736;9374400.0,308.34927;9378000.0,313.30264;9381600.0,317.90176;9385200.0,319.50363;9388800.0,319.2267;9392400.0,317.36523;9396000.0,314.70532;9399600.0,311.20728;9403200.0,307.97144;9406800.0,304.93192;9410400.0,302.0778;9414000.0,299.5189;9417600.0,297.1427;9421200.0,295.29596;9424800.0,293.4262;9428400.0,291.60825;9432000.0,290.2519;9435600.0,289.19583;9439200.0,288.2622;9442800.0,288.5157;9446400.0,290.12912;9450000.0,293.66006;9453600.0,299.43805;9457200.0,305.89117;9460800.0,308.54733;9464400.0,310.12387;9468000.0,311.94025;9471600.0,310.48904;9475200.0,308.94806;9478800.0,307.26068;9482400.0,305.27496;9486000.0,303.06604;9489600.0,300.85043;9493200.0,298.71472;9496800.0,297.1702;9500400.0,295.6183;9504000.0,294.05972;9507600.0,292.49445;9511200.0,291.0789;9514800.0,289.73123;9518400.0,288.2309;9522000.0,286.97714;9525600.0,285.7937;9529200.0,286.36633;9532800.0,288.2358;9536400.0,291.7571;9540000.0,296.71655;9543600.0,302.6214;9547200.0,307.2777;9550800.0,310.88232;9554400.0,313.01804;9558000.0,311.12973;9561600.0,308.46252;9565200.0,306.36273;9568800.0,303.6631;9572400.0,300.34302;9576000.0,297.44632;9579600.0,294.97415;9583200.0,292.78857;9586800.0,290.84372;9590400.0,289.09778;9594000.0,287.57843;9597600.0,286.12717;9601200.0,284.78506;9604800.0,283.33554;9608400.0,282.1315;9612000.0,280.95862;9615600.0,280.2757;9619200.0,279.9697;9622800.0,280.0002;9626400.0,280.32037;9630000.0,281.01706;9633600.0,281.55862;9637200.0,282.40408;9640800.0,283.25494;9644400.0,283.96115;9648000.0,283.79352;9651600.0,283.3291;9655200.0,282.83005;9658800.0,282.1421;9662400.0,281.3828;9666000.0,280.6441;9669600.0,279.93277;9673200.0,279.17606;9676800.0,278.45206;9680400.0,277.87463;9684000.0,277.32983;9687600.0,276.92517;9691200.0,276.511;9694800.0,276.12183;9698400.0,275.7163;9702000.0,275.90247;9705600.0,276.1157;9709200.0,276.4703;9712800.0,277.03632;9716400.0,277.80087;9720000.0,278.87878;9723600.0,280.11908;9727200.0,281.77838;9730800.0,283.06863;9734400.0,283.4065;9738000.0,282.89197;9741600.0,282.04156;9745200.0,281.36514;9748800.0,280.52478;9752400.0,279.74692;9756000.0,279.09427;9759600.0,278.48132;9763200.0,277.95566;9766800.0,277.37604;9770400.0,276.72726;9774000.0,276.10413;9777600.0,275.57443;9781200.0,275.12427;9784800.0,274.77908;9788400.0,275.9226;9792000.0,278.75574;9795600.0,282.4067;9799200.0,285.53986;9802800.0,288.315;9806400.0,292.98578;9810000.0,295.21136;9813600.0,296.59473;9817200.0,297.1669;9820800.0,296.27997;9824400.0,294.6149;9828000.0,292.77118;9831600.0,290.9442;9835200.0,289.1856;9838800.0,287.6255;9842400.0,286.27667;9846000.0,285.1568;9849600.0,284.10114;9853200.0,283.21902;9856800.0,282.28186;9860400.0,281.41483;9864000.0,280.59555;9867600.0,280.0211;9871200.0,279.45386;9874800.0,279.70532;9878400.0,279.87885;9882000.0,280.10608;9885600.0,282.3745;9889200.0,287.21542;9892800.0,292.1631;9896400.0,295.06985;9900000.0,296.65668;9903600.0,298.2964;9907200.0,297.14444;9910800.0,296.11365;9914400.0,294.81573;9918000.0,292.87665;9921600.0,290.7684;9925200.0,288.8897;9928800.0,286.93616;9932400.0,285.19846;9936000.0,283.61435;9939600.0,282.2908;9943200.0,281.07852;9946800.0,280.09595;9950400.0,279.19232;9954000.0,278.27664;9957600.0,277.61533;9961200.0,278.98737;9964800.0,281.76917;9968400.0,285.48578;9972000.0,287.38898;9975600.0,291.22897;9979200.0,297.65387;9982800.0,302.576;9986400.0,306.27;9990000.0,308.4212;9993600.0,308.2051;9997200.0,306.03528;10000800.0,303.22385;10004400.0,300.55936;10008000.0,297.71005;10011600.0,295.04803;10015200.0,292.64584;10018800.0,290.7414;10022400.0,289.19473;10026000.0,287.55524;10029600.0,286.15613;10033200.0,285.31976;10036800.0,284.59186;10040400.0,284.05283;10044000.0,283.9408;10047600.0,285.08694;10051200.0,286.61374;10054800.0,289.21185;10058400.0,294.4201;10062000.0,301.3418;10065600.0,307.31342;10069200.0,312.88354;10072800.0,316.76044;10076400.0,316.9018;10080000.0,315.05368;10083600.0,312.3184;10087200.0,309.63467;10090800.0,306.25302;10094400.0,303.1198;10098000.0,300.34903;10101600.0,297.86154;10105200.0,295.5922;10108800.0,293.63333;10112400.0,291.854;10116000.0,290.33286;10119600.0,288.8814;10123200.0,287.68365;10126800.0,286.3555;10130400.0,285.26843;10134000.0,285.52106;10137600.0,287.05533;10141200.0,288.9888;10144800.0,292.50378;10148400.0,294.5146;10152000.0,296.6934;10155600.0,298.83627;10159200.0,299.54102;10162800.0,301.58206;10166400.0,302.23077;10170000.0,300.54126;10173600.0,298.35474;10177200.0,296.19943;10180800.0,294.3086;10184400.0,292.46317;10188000.0,291.03394;10191600.0,289.86313;10195200.0,288.86664;10198800.0,288.0087;10202400.0,287.27383;10206000.0,286.34195;10209600.0,285.4985;10213200.0,284.85678;10216800.0,284.1989;10220400.0,284.59158;10224000.0,286.16193;10227600.0,287.6435;10231200.0,289.1228;10234800.0,290.05762;10238400.0,290.6458;10242000.0,292.1465;10245600.0,293.13864;10249200.0,292.63403;10252800.0,292.73553;10256400.0,292.24533;10260000.0,291.45038;10263600.0,290.07544;10267200.0,288.78003;10270800.0,287.69223;10274400.0,286.77295;10278000.0,285.98785;10281600.0,285.3862;10285200.0,284.82388;10288800.0,284.3142;10292400.0,283.90863;10296000.0,283.5288;10299600.0,283.23853;10303200.0,282.93307;10306800.0,283.0379;10310400.0,283.24722;10314000.0,283.70892;10317600.0,284.33896;10321200.0,285.14102;10324800.0,285.99792;10328400.0,286.5304;10332000.0,286.82486;10335600.0,286.57355;10339200.0,286.21854;10342800.0,285.7854;10346400.0,285.25122;10350000.0,284.78033;10353600.0,284.1308;10357200.0,283.35187;10360800.0,282.58035;10364400.0,281.82812;10368000.0,281.10886;10371600.0,280.39282;10375200.0,279.4756;10378800.0,278.62967;10382400.0,277.81378;10386000.0,277.22476;10389600.0,277.31207;10393200.0,278.7275;10396800.0,281.14362;10400400.0,284.7052;10404000.0,289.601;10407600.0,295.56174;10411200.0,301.57834;10414800.0,306.80035;10418400.0,310.31558;10422000.0,311.5867;10425600.0,311.6166;10429200.0,310.6367;10432800.0,308.897;10436400.0,306.41794;10440000.0,303.6563;10443600.0,300.9602;10447200.0,298.42395;10450800.0,296.0807;10454400.0,293.84204;10458000.0,291.86642;10461600.0,290.0436;10465200.0,288.42303;10468800.0,286.7794;10472400.0,285.27707;10476000.0,284.34482;10479600.0,284.59552;10483200.0,285.77765;10486800.0,288.4092;10490400.0,292.27414;10494000.0,297.19617;10497600.0,302.41876;10501200.0,306.64938;10504800.0,308.82388;10508400.0,309.40543;10512000.0,308.54987;10515600.0,306.71097;10519200.0,304.0957;10522800.0,301.01785;10526400.0,297.93665;10530000.0,295.04224;10533600.0,292.62045;10537200.0,290.5267;10540800.0,288.66122;10544400.0,287.00916;10548000.0,285.4176;10551600.0,283.9086;10555200.0,282.4436;10558800.0,281.18622;10562400.0,280.50644;10566000.0,281.1206;10569600.0,282.7693;10573200.0,285.80716;10576800.0,290.2997;10580400.0,295.6564;10584000.0,298.68826;10587600.0,300.24753;10591200.0,301.8812;10594800.0,303.44354;10598400.0,302.9016;10602000.0,301.7881;10605600.0,300.40097;10609200.0,298.5146;10612800.0,296.3954;10616400.0,294.2784;10620000.0,292.22443;10623600.0,290.3367;10627200.0,288.74387;10630800.0,287.32053;10634400.0,286.1783;10638000.0,285.13837;10641600.0,284.23132;10645200.0,283.45407;10648800.0,282.91516;10652400.0,282.83182;10656000.0,283.16693;10659600.0,283.91562;10663200.0,285.12527;10666800.0,286.8374;10670400.0,288.3471;10674000.0,289.1703;10677600.0,289.5495;10681200.0,289.4362;10684800.0,289.1318;10688400.0,288.40753;10692000.0,287.38092;10695600.0,286.10822;10699200.0,284.8502;10702800.0,283.7566;10706400.0,282.7458;10710000.0,281.71402;10713600.0,280.8173;10717200.0,280.04517;10720800.0,279.4565;10724400.0,279.0548;10728000.0,278.6772;10731600.0,278.42355;10735200.0,278.407;10738800.0,278.88174;10742400.0,281.02618;10746000.0,284.38474;10749600.0,288.71213;10753200.0,293.83035;10756800.0,298.86813;10760400.0,303.61206;10764000.0,306.99127;10767600.0,308.3634;10771200.0,308.1395;10774800.0,306.79483;10778400.0,305.06726;10782000.0,302.6467;10785600.0,299.68713;10789200.0,297.11404;10792800.0,294.77213;10796400.0,292.75363;10800000.0,290.97354;10803600.0,289.39636;10807200.0,287.9983;10810800.0,286.71768;10814400.0,285.57498;10818000.0,284.48822;10821600.0,283.81775;10825200.0,284.73956;10828800.0,286.83264;10832400.0,290.38727;10836000.0,295.56378;10839600.0,300.4434;10843200.0,305.58096;10846800.0,309.08228;10850400.0,311.3451;10854000.0,312.1773;10857600.0,310.69324;10861200.0,308.59958;10864800.0,306.31125;10868400.0,303.9723;10872000.0,301.77826;10875600.0,299.9187;10879200.0,298.0215;10882800.0,296.22214;10886400.0,294.5818;10890000.0,292.98016;10893600.0,291.39474;10897200.0,289.99728;10900800.0,288.6839;10904400.0,287.4425;10908000.0,286.84503;10911600.0,287.0425;10915200.0,287.97873;10918800.0,290.5804;10922400.0,294.28058;10926000.0,299.094;10929600.0,304.35995;10933200.0,307.90082;10936800.0,310.47858;10940400.0,312.129;10944000.0,312.01535;10947600.0,310.91055;10951200.0,309.27908;10954800.0,306.90323;10958400.0,304.02573;10962000.0,301.34848;10965600.0,298.94717;10969200.0,296.6964;10972800.0,294.6208;10976400.0,292.68335;10980000.0,290.96896;10983600.0,289.35394;10987200.0,287.94482;10990800.0,286.6836;10994400.0,286.41394;10998000.0,287.7075;11001600.0,290.02905;11005200.0,293.22366;11008800.0,297.8125;11012400.0,303.19095;11016000.0,308.60855;11019600.0,312.929;11023200.0,315.84686;11026800.0,317.54877;11030400.0,317.38068;11034000.0,316.33408;11037600.0,314.66425;11041200.0,312.26712;11044800.0,309.5529;11048400.0,307.06128;11052000.0,304.87894;11055600.0,302.92566;11059200.0,301.03302;11062800.0,299.23584;11066400.0,297.73148;11070000.0,296.3783;11073600.0,295.07547;11077200.0,293.78995;11080800.0,293.38727;11084400.0,294.06195;11088000.0,295.26413;11091600.0,297.30643;11095200.0,300.2141;11098800.0,304.2144;11102400.0,308.56912;11106000.0,311.3889;11109600.0,311.7193;11113200.0,312.8407;11116800.0,313.1739;11120400.0,312.8226;11124000.0,311.69012;11127600.0,309.9495;11131200.0,307.9384;11134800.0,305.89813;11138400.0,303.99197;11142000.0,302.28006;11145600.0,300.667;11149200.0,298.95062;11152800.0,297.49146;11156400.0,296.12973;11160000.0,294.80075;11163600.0,293.64127;11167200.0,293.194;11170800.0,293.93427;11174400.0,296.0071;11178000.0,299.28314;11181600.0,303.3858;11185200.0,308.35416;11188800.0,313.07278;11192400.0,316.92227;11196000.0,319.5109;11199600.0,320.4412;11203200.0,319.79144;11206800.0,318.4746;11210400.0,316.70972;11214000.0,314.29318;11217600.0,311.51178;11221200.0,308.891;11224800.0,306.51328;11228400.0,304.52875;11232000.0,302.79755;11235600.0,301.24933;11239200.0,299.79468;11242800.0,298.43948;11246400.0,297.14072;11250000.0,295.9077;11253600.0,295.2896;11257200.0,295.87814;11260800.0,297.4374;11264400.0,300.2667;11268000.0,303.82138;11271600.0,308.55283;11275200.0,313.0596;11278800.0,316.73868;11282400.0,319.3635;11286000.0,320.35587;11289600.0,319.88464;11293200.0,318.67358;11296800.0,316.94498;11300400.0,314.53568;11304000.0,311.8283;11307600.0,309.1932;11311200.0,306.60312;11314800.0,304.33282;11318400.0,302.1201;11322000.0,300.45117;11325600.0,299.0156;11329200.0,296.87924;11332800.0,294.77774;11336400.0,292.9181;11340000.0,292.25912;11343600.0,292.7259;11347200.0,293.7219;11350800.0,296.04572;11354400.0,299.9852;11358000.0,304.35968;11361600.0,308.61072;11365200.0,312.26593;11368800.0,314.3885;11372400.0,314.9675;11376000.0,314.0886;11379600.0,312.7916;11383200.0,311.07343;11386800.0,308.74957;11390400.0,305.58347;11394000.0,302.42245;11397600.0,299.55515;11401200.0,297.0674;11404800.0,294.82544;11408400.0,292.54395;11412000.0,290.39447;11415600.0,288.67682;11419200.0,287.20685;11422800.0,285.6996;11426400.0,285.59604;11430000.0,286.94833;11433600.0,289.28806;11437200.0,292.43155;11440800.0,296.75745;11444400.0,301.44116;11448000.0,306.65213;11451600.0,311.04916;11455200.0,314.38098;11458800.0,316.13165;11462400.0,315.72733;11466000.0,313.9871;11469600.0,311.92877;11473200.0,309.7093;11476800.0,307.28094;11480400.0,305.04666;11484000.0,302.93094;11487600.0,301.02365;11491200.0,299.3621;11494800.0,297.64038;11498400.0,295.85034;11502000.0,294.08127;11505600.0,292.24896;11509200.0,290.43237;11512800.0,290.18234;11516400.0,291.68015;11520000.0,293.83472;11523600.0,296.62286;11527200.0,300.4826;11530800.0,305.4026;11534400.0,310.3399;11538000.0,313.57684;11541600.0,315.59384;11545200.0,316.84006;11548800.0,316.77942;11552400.0,315.95074;11556000.0,314.65497;11559600.0,312.4075;11563200.0,309.67606;11566800.0,307.07718;11570400.0,304.6195;11574000.0,302.31348;11577600.0,300.63388;11581200.0,299.01227;11584800.0,297.57806;11588400.0,296.05054;11592000.0,294.50598;11595600.0,293.12158;11599200.0,292.70358;11602800.0,293.67786;11606400.0,295.5063;11610000.0,296.21304;11613600.0,296.86353;11617200.0,297.69415;11620800.0,298.46262;11624400.0,298.50873;11628000.0,297.91412;11631600.0,297.38696;11635200.0,297.86826;11638800.0,298.2626;11642400.0,297.92154;11646000.0,296.61578;11649600.0,294.78326;11653200.0,293.2248;11656800.0,292.00256;11660400.0,290.92654;11664000.0,289.96426;11667600.0,289.0078;11671200.0,288.18997;11674800.0,287.35013;11678400.0,286.68002;11682000.0,286.15088;11685600.0,285.7769;11689200.0,285.7246;11692800.0,285.87878;11696400.0,286.59964;11700000.0,287.8026;11703600.0,290.62186;11707200.0,292.72272;11710800.0,297.3895;11714400.0,299.33478;11718000.0,300.10883;11721600.0,300.8231;11725200.0,300.82397;11728800.0,299.46704;11732400.0,297.72592;11736000.0,295.8874;11739600.0,294.32712;11743200.0,292.7829;11746800.0,291.46854;11750400.0,290.2844;11754000.0,289.23193;11757600.0,288.4123;11761200.0,287.58652;11764800.0,286.65112;11768400.0,285.9173;11772000.0,286.015;11775600.0,287.2266;11779200.0,289.39868;11782800.0,292.15833;11786400.0,295.23358;11790000.0,300.02246;11793600.0,304.54376;11797200.0,307.93774;11800800.0,309.92014;11804400.0,311.48056;11808000.0,311.67215;11811600.0,310.4632;11815200.0,308.14975;11818800.0,305.726;11822400.0,303.46478;11826000.0,301.36523;11829600.0,299.40308;11833200.0,297.57346;11836800.0,296.08093;11840400.0,294.81873;11844000.0,293.78995;11847600.0,292.82562;11851200.0,291.9903;11854800.0,291.2507;11858400.0,290.8935;11862000.0,291.78842;11865600.0,293.74854;11869200.0,296.52356;11872800.0,299.7832;11876400.0,303.98172;11880000.0,308.50116;11883600.0,312.4203;11887200.0,314.92526;11890800.0,315.77512;11894400.0,315.52835;11898000.0,314.9275;11901600.0,313.86252;11905200.0,312.24994;11908800.0,309.11734;11912400.0,305.94128;11916000.0,303.14786;11919600.0,300.81836;11923200.0,298.86353;11926800.0,296.95438;11930400.0,295.69803;11934000.0,295.0867;11937600.0,294.55276;11941200.0,294.05206;11944800.0,294.08752;11948400.0,295.07538;11952000.0,296.82385;11955600.0,299.56924;11959200.0,303.5287;11962800.0,307.66943;11966400.0,311.70776;11970000.0,315.10168;11973600.0,317.89127;11977200.0,318.27448;11980800.0,316.55716;11984400.0,314.1913;11988000.0,312.21893;11991600.0,309.73587;11995200.0,307.19864;11998800.0,305.02194;12002400.0,303.1172;12006000.0,301.48715;12009600.0,299.96945;12013200.0,298.66605;12016800.0,297.58255;12020400.0,296.67523;12024000.0,295.83957;12027600.0,295.23672;12031200.0,295.09027;12034800.0,296.23276;12038400.0,297.85965;12042000.0,299.94037;12045600.0,302.9114;12049200.0,306.18448;12052800.0,307.43835;12056400.0,307.91196;12060000.0,308.43643;12063600.0,309.42767;12067200.0,309.0893;12070800.0,308.10983;12074400.0,306.32455;12078000.0,304.6866;12081600.0,302.62552;12085200.0,300.74622;12088800.0,299.08582;12092400.0,297.70282;12096000.0,296.38782;12099600.0,295.24533;12103200.0,294.23828;12106800.0,293.35477;12110400.0,292.703;12114000.0,292.07495;12117600.0,291.789;12121200.0,292.77002;12124800.0,294.2516;12128400.0,297.41275;12132000.0,301.14474;12135600.0,305.8003;12139200.0,309.7482;12142800.0,313.36243;12146400.0,316.685;12150000.0,316.56967;12153600.0,314.95764;12157200.0,312.9007;12160800.0,310.64606;12164400.0,308.2904;12168000.0,305.8005;12171600.0,303.55945;12175200.0,301.58652;12178800.0,299.80438;12182400.0,298.43088;12186000.0,297.1515;12189600.0,295.8933;12193200.0,294.5381;12196800.0,293.13892;12200400.0,292.11707;12204000.0,291.88132;12207600.0,292.97412;12211200.0,294.9855;12214800.0,297.76724;12218400.0,300.4648;12222000.0,303.6194;12225600.0,307.44473;12229200.0,311.1071;12232800.0,313.61984;12236400.0,312.85724;12240000.0,312.09296;12243600.0,310.58255;12247200.0,307.75443;12250800.0,304.80118;12254400.0,302.25577;12258000.0,300.0338;12261600.0,298.2268;12265200.0,296.61496;12268800.0,295.104;12272400.0,293.85822;12276000.0,292.68225;12279600.0,291.52872;12283200.0,290.65213;12286800.0,289.96454;12290400.0,289.86414;12294000.0,289.99826;12297600.0,290.8259;12301200.0,293.06784;12304800.0,296.72873;12308400.0,300.97247;12312000.0,301.90735;12315600.0,302.24976;12319200.0,303.30072;12322800.0,302.6119;12326400.0,301.5199;12330000.0,301.5832;12333600.0,300.60416;12337200.0,298.9244;12340800.0,297.31018;12344400.0,295.92545;12348000.0,294.71747;12351600.0,293.52808;12355200.0,292.38193;12358800.0,291.41925;12362400.0,290.58386;12366000.0,289.74503;12369600.0,289.06848;12373200.0,288.43594;12376800.0,288.02237;12380400.0,288.79083;12384000.0,290.7407;12387600.0,293.52646;12391200.0,294.7795;12394800.0,297.47647;12398400.0,299.8148;12402000.0,303.43533;12405600.0,305.13492;12409200.0,307.51797;12412800.0,307.88376;12416400.0,307.09393;12420000.0,306.0678;12423600.0,304.20248;12427200.0,301.84802;12430800.0,299.81186;12434400.0,297.95493;12438000.0,296.14536;12441600.0,294.5136;12445200.0,292.95526;12448800.0,291.502;12452400.0,290.10568;12456000.0,288.68912;12459600.0,287.56183;12463200.0,287.70844;12466800.0,288.92175;12470400.0,290.93323;12474000.0,293.77313;12477600.0,297.6184;12481200.0,302.17664;12484800.0,306.42023;12488400.0,309.82022;12492000.0,312.34515;12495600.0,313.2923;12499200.0,313.501;12502800.0,313.17908;12506400.0,312.1321;12510000.0,310.15482;12513600.0,307.48767;12517200.0,304.8904;12520800.0,302.6041;12524400.0,300.44745;12528000.0,298.51096;12531600.0,296.57178;12535200.0,294.8825;12538800.0,293.3296;12542400.0,291.82907;12546000.0,290.60806;12549600.0,290.54053;12553200.0,291.72028;12556800.0,293.84906;12560400.0,296.51285;12564000.0,300.46387;12567600.0,304.0908;12571200.0,306.5631;12574800.0,309.56372;12578400.0,311.6577;12582000.0,312.48224;12585600.0,312.5222;12589200.0,311.83054;12592800.0,310.26697;12596400.0,308.17026;12600000.0,305.5104;12603600.0,302.9263;12607200.0,300.59424;12610800.0,298.27542;12614400.0,296.31717;12618000.0,294.72278;12621600.0,293.55817;12625200.0,292.49725;12628800.0,291.58902;12632400.0,290.9025;12636000.0,290.57297;12639600.0,291.23666;12643200.0,293.0736;12646800.0,294.40814;12650400.0,297.20184;12654000.0,299.95682;12657600.0,304.2678;12661200.0,307.82162;12664800.0,310.8224;12668400.0,311.6016;12672000.0,311.7953;12675600.0,311.71332;12679200.0,310.71573;12682800.0,308.63678;12686400.0,305.904;12690000.0,303.25534;12693600.0,300.82965;12697200.0,298.8828;12700800.0,297.06763;12704400.0,295.366;12708000.0,294.2688;12711600.0,293.5468;12715200.0,292.90524;12718800.0,292.23828;12722400.0,291.78864;12726000.0,291.84958;12729600.0,293.31558;12733200.0,296.07156;12736800.0,299.4603;12740400.0,303.29053;12744000.0,307.37015;12747600.0,310.54266;12751200.0,312.59167;12754800.0,314.06583;12758400.0,313.9331;12762000.0,312.75912;12765600.0,311.1805;12769200.0,309.20312;12772800.0,306.67322;12776400.0,304.02908;12780000.0,301.3376;12783600.0,299.03992;12787200.0,297.223;12790800.0,295.43607;12794400.0,293.72003;12798000.0,292.11874;12801600.0,290.69095;12805200.0,289.47913;12808800.0,289.26312;12812400.0,290.34464;12816000.0,292.2964;12819600.0,295.1515;12823200.0,298.6867;12826800.0,302.8271;12830400.0,306.9571;12834000.0,310.82922;12837600.0,313.33136;12841200.0,314.1716;12844800.0,314.46036;12848400.0,313.82376;12852000.0,312.38058;12855600.0,309.6905;12859200.0,306.31448;12862800.0,303.3493;12866400.0,300.81845;12870000.0,298.5133;12873600.0,296.4829;12877200.0,294.59515;12880800.0,292.91412;12884400.0,291.38724;12888000.0,290.05362;12891600.0,288.94412;12895200.0,288.8442;12898800.0,290.27975;12902400.0,292.7539;12906000.0,295.7321;12909600.0,299.59207;12913200.0,303.21484;12916800.0,307.60645;12920400.0,311.65564;12924000.0,314.36612;12927600.0,315.8039;12931200.0,315.53534;12934800.0,314.18915;12938400.0,312.6597;12942000.0,310.39737;12945600.0,307.55682;12949200.0,304.9679;12952800.0,302.4984;12956400.0,300.4938;12960000.0,298.7815;12963600.0,297.40857;12967200.0,296.01508;12970800.0,294.91382;12974400.0,293.9812;12978000.0,293.1068;12981600.0,292.955;12985200.0,293.6988;12988800.0,295.02972;12992400.0,297.33908;12996000.0,299.94598;12999600.0,303.3278;13003200.0,306.71796;13006800.0,310.44653;13010400.0,312.05984;13014000.0,312.08694;13017600.0,311.66388;13021200.0,310.08145;13024800.0,307.68063;13028400.0,305.32727;13032000.0,302.85208;13035600.0,300.799;13039200.0,299.0208;13042800.0,297.4584;13046400.0,296.05093;13050000.0,294.80655;13053600.0,293.8024;13057200.0,293.0519;13060800.0,292.38538;13064400.0,291.5803;13068000.0,291.79968;13071600.0,293.16843;13075200.0,295.04605;13078800.0,297.6127;13082400.0,301.63965;13086000.0,305.55927;13089600.0,309.47726;13093200.0,312.95953;13096800.0,315.5203;13100400.0,316.91177;13104000.0,317.10776;13107600.0,316.81583;13111200.0,315.76578;13114800.0,313.65817;13118400.0,310.8239;13122000.0,308.09576;13125600.0,305.78363;13129200.0,303.5153;13132800.0,301.6456;13136400.0,300.01453;13140000.0,298.60513;13143600.0,297.42325;13147200.0,296.32172;13150800.0,295.20853;13154400.0,294.63193;13158000.0,295.50067;13161600.0,297.27213;13165200.0,299.60736;13168800.0,302.83795;13172400.0,305.86166;13176000.0,309.98056;13179600.0,313.54;13183200.0,314.24472;13186800.0,312.36417;13190400.0,311.335;13194000.0,309.8916;13197600.0,308.80106;13201200.0,307.10193;13204800.0,304.7772;13208400.0,302.81256;13212000.0,301.04108;13215600.0,299.46558;13219200.0,298.2331;13222800.0,296.9537;13226400.0,295.63156;13230000.0,294.4611;13233600.0,293.33865;13237200.0,292.3952;13240800.0,292.53708;13244400.0,292.60263;13248000.0,294.37683;13251600.0,297.2112;13255200.0,300.8036;13258800.0,304.88058;13262400.0,307.59207;13266000.0,311.36407;13269600.0,313.65866;13273200.0,315.137;13276800.0,314.6641;13280400.0,313.25018;13284000.0,311.67804;13287600.0,309.5954;13291200.0,307.3801;13294800.0,305.3142;13298400.0,303.3548;13302000.0,301.55243;13305600.0,299.78632;13309200.0,298.05713;13312800.0,296.58786;13316400.0,295.23373;13320000.0,293.94577;13323600.0,292.85083;13327200.0,292.58945;13330800.0,293.27298;13334400.0,295.29828;13338000.0,297.99426;13341600.0,301.75806;13345200.0,305.62674;13348800.0,308.36447;13352400.0,311.96255;13356000.0,315.18997;13359600.0,317.15646;13363200.0,317.92706;13366800.0,317.7787;13370400.0,316.38065;13374000.0,314.4975;13377600.0,311.9359;13381200.0,309.46323;13384800.0,307.46808;13388400.0,305.27628;13392000.0,302.8746;13395600.0,300.8366;13399200.0,299.05258;13402800.0,297.44672;13406400.0,295.8369;13410000.0,294.32947;13413600.0,294.2189;13417200.0,295.4974;13420800.0,297.6174;13424400.0,300.46695;13428000.0,304.07944;13431600.0,308.12454;13435200.0,311.955;13438800.0,315.01822;13442400.0,317.23926;13446000.0,318.22012;13449600.0,318.22046;13453200.0,317.83603;13456800.0,316.78885;13460400.0,314.92984;13464000.0,312.47476;13467600.0,310.11093;13471200.0,308.05914;13474800.0,306.04166;13478400.0,304.26724;13482000.0,302.5224;13485600.0,301.09128;13489200.0,300.00552;13492800.0,299.1835;13496400.0,298.69064;13500000.0,298.58508;13503600.0,299.47842;13507200.0,301.25766;13510800.0,303.60962;13514400.0,306.80365;13518000.0,310.58813;13521600.0,314.20895;13525200.0,317.37634;13528800.0,319.82474;13532400.0,320.66498;13536000.0,320.56555;13539600.0,319.74918;13543200.0,318.5366;13546800.0,316.3529;13550400.0,313.487;13554000.0,310.61948;13557600.0,307.87735;13561200.0,305.54196;13564800.0,303.5128;13568400.0,301.5046;13572000.0,299.79086;13575600.0,298.31;13579200.0,296.80786;13582800.0,295.7394;13586400.0,295.75626;13590000.0,296.88766;13593600.0,299.11603;13597200.0,302.03568;13600800.0,305.5949;13604400.0,309.44232;13608000.0,313.08548;13611600.0,316.5952;13615200.0,318.56;13618800.0,319.3521;13622400.0,319.2569;13626000.0,319.18243;13629600.0,317.90277;13633200.0,315.36493;13636800.0,312.77164;13640400.0,310.17725;13644000.0,307.7609;13647600.0,305.44272;13651200.0,302.94977;13654800.0,300.58936;13658400.0,298.70975;13662000.0,296.8851;13665600.0,295.25537;13669200.0,293.6625;13672800.0,293.5705;13676400.0,294.79712;13680000.0,296.69516;13683600.0,298.9053;13687200.0,301.9155;13690800.0,305.507;13694400.0,309.0519;13698000.0,312.34875;13701600.0,314.77002;13705200.0,315.4914;13708800.0,315.13458;13712400.0,314.66953;13716000.0,314.1858;13719600.0,312.74124;13723200.0,310.20407;13726800.0,307.42737;13730400.0,304.7294;13734000.0,302.5483;13737600.0,300.40247;13741200.0,298.2714;13744800.0,296.22375;13748400.0,294.17664;13752000.0,292.26202;13755600.0,290.5892;13759200.0,290.36957;13762800.0,291.73352;13766400.0,293.6269;13770000.0,295.86627;13773600.0,298.90012;13777200.0,302.578;13780800.0,306.39896;13784400.0,309.80637;13788000.0,312.50363;13791600.0,313.73248;13795200.0,313.13904;13798800.0,312.0183;13802400.0,311.04364;13806000.0,309.24313;13809600.0,306.63382;13813200.0,304.0849;13816800.0,301.77753;13820400.0,299.75134;13824000.0,297.86594;13827600.0,296.1078;13831200.0,294.48553;13834800.0,292.89166;13838400.0,291.4022;13842000.0,290.20978;13845600.0,290.47833;13849200.0,292.10516;13852800.0,294.03513;13856400.0,296.4031;13860000.0,299.8026;13863600.0,303.39758;13867200.0,307.3992;13870800.0,310.7605;13874400.0,313.21808;13878000.0,314.65558;13881600.0,314.8684;13885200.0,314.48575;13888800.0,313.6764;13892400.0,312.16473;13896000.0,309.8543;13899600.0,307.2946;13903200.0,304.93356;13906800.0,302.64813;13910400.0,300.58508;13914000.0,298.97742;13917600.0,297.48322;13921200.0,295.9679;13924800.0,294.526;13928400.0,293.15872;13932000.0,293.05557;13935600.0,294.13223;13939200.0,295.89734;13942800.0,298.22626;13946400.0,301.30817;13950000.0,305.08255;13953600.0,309.19766;13957200.0,312.62933;13960800.0,315.49557;13964400.0,317.07474;13968000.0,317.17746;13971600.0,317.1231;13975200.0,315.8441;13978800.0,314.18918;13982400.0,312.00876;13986000.0,309.66537;13989600.0,307.5783;13993200.0,305.6736;13996800.0,303.79865;14000400.0,302.10074;14004000.0,300.45844;14007600.0,298.96918;14011200.0,297.55978;14014800.0,296.5774;14018400.0,296.88837;14022000.0,298.26028;14025600.0,300.52216;14029200.0,303.17947;14032800.0,306.53348;14036400.0,310.4073;14040000.0,314.18674;14043600.0,316.04355;14047200.0,317.51205;14050800.0,319.6169;14054400.0,319.31952;14058000.0,318.0962;14061600.0,316.42508;14065200.0,314.38052;14068800.0,312.23856;14072400.0,310.00034;14076000.0,307.514;14079600.0,305.10556;14083200.0,303.14984;14086800.0,301.22824;14090400.0,299.5137;14094000.0,298.007;14097600.0,296.57693;14101200.0,295.37292;14104800.0,295.55685;14108400.0,297.02957;14112000.0,299.55093;14115600.0,302.53986;14119200.0,306.35422;14122800.0,310.58652;14126400.0,314.80716;14130000.0,318.8015;14133600.0,321.26443;14137200.0,322.3163;14140800.0,321.58337;14144400.0,320.83615;14148000.0,319.79092;14151600.0,317.79974;14155200.0,315.04718;14158800.0,312.39148;14162400.0,310.1985;14166000.0,308.32007;14169600.0,306.4591;14173200.0,304.99417;14176800.0,303.66556;14180400.0,302.26852;14184000.0,301.32413;14187600.0,300.6603;14191200.0,300.61145;14194800.0,301.9004;14198400.0,304.1538;14202000.0,306.88358;14205600.0,309.24567;14209200.0,312.13025;14212800.0,316.31384;14216400.0,319.75095;14220000.0,322.37405;14223600.0,323.48077;14227200.0,323.65344;14230800.0,323.11386;14234400.0,321.89975;14238000.0,320.11514;14241600.0,317.35587;14245200.0,314.37927;14248800.0,311.39682;14252400.0,308.49313;14256000.0,305.74442;14259600.0,303.1575;14263200.0,300.95944;14266800.0,298.64743;14270400.0,296.57748;14274000.0,294.92957;14277600.0,294.87955;14281200.0,295.9972;14284800.0,297.75705;14288400.0,299.49023;14292000.0,301.59647;14295600.0,304.86118;14299200.0,308.695;14302800.0,311.91284;14306400.0,314.68298;14310000.0,316.18094;14313600.0,316.73547;14317200.0,316.42474;14320800.0,315.5589;14324400.0,314.2183;14328000.0,311.91476;14331600.0,309.3753;14335200.0,306.77966;14338800.0,304.65524;14342400.0,302.71008;14346000.0,300.85403;14349600.0,299.2637;14353200.0,298.06882;14356800.0,297.058;14360400.0,296.07465;14364000.0,295.45007;14367600.0,295.25626;14371200.0,295.40427;14374800.0,296.2304;14378400.0,299.15176;14382000.0,302.4015;14385600.0,305.8859;14389200.0,309.32727;14392800.0,312.09805;14396400.0,313.82678;14400000.0,314.23236;14403600.0,314.30554;14407200.0,314.04422;14410800.0,312.54987;14414400.0,310.1195;14418000.0,307.92868;14421600.0,305.95834;14425200.0,304.12833;14428800.0,302.31052;14432400.0,300.57773;14436000.0,299.0427;14439600.0,297.57544;14443200.0,296.11365;14446800.0,295.0879;14450400.0,294.5533;14454000.0,294.7451;14457600.0,296.235;14461200.0,298.55493;14464800.0,301.8934;14468400.0,305.5241;14472000.0,309.29462;14475600.0,312.08017;14479200.0,314.51822;14482800.0,315.92395;14486400.0,316.32333;14490000.0,316.17984;14493600.0,315.11722;14497200.0,313.11258;14500800.0,310.7323;14504400.0,308.49417;14508000.0,306.56113;14511600.0,304.94904;14515200.0,303.30975;14518800.0,301.83844;14522400.0,300.3904;14526000.0,298.84723;14529600.0,297.52606;14533200.0,296.4771;14536800.0,295.91318;14540400.0,295.82742;14544000.0,296.4406;14547600.0,298.6609;14551200.0,302.13046;14554800.0,305.9656;14558400.0,309.908;14562000.0,313.20547;14565600.0,312.66864;14569200.0,312.21967;14572800.0,311.60574;14576400.0,310.44803;14580000.0,309.11017;14583600.0,307.54413;14587200.0,305.78027;14590800.0,304.22656;14594400.0,302.61633;14598000.0,301.237;14601600.0,300.08722;14605200.0,298.94183;14608800.0,298.05313;14612400.0,297.19763;14616000.0,296.32196;14619600.0,295.60355;14623200.0,295.81857;14626800.0,297.16714;14630400.0,299.23563;14634000.0,301.74615;14637600.0,304.87527;14641200.0,308.94128;14644800.0,312.53806;14648400.0,315.55627;14652000.0,317.81915;14655600.0,318.9046;14659200.0,318.9074;14662800.0,317.96872;14666400.0,315.71658;14670000.0,312.89816;14673600.0,310.24045;14677200.0,307.77924;14680800.0,305.48062;14684400.0,303.4701;14688000.0,301.8578;14691600.0,300.28265;14695200.0,298.82584;14698800.0,297.48682;14702400.0,296.1854;14706000.0,295.06018;14709600.0,295.0786;14713200.0,296.36462;14716800.0,298.79492;14720400.0,301.64117;14724000.0,304.65128;14727600.0,308.5782;14731200.0,312.3977;14734800.0,314.83798;14738400.0,316.98386;14742000.0,318.09384;14745600.0,316.11765;14749200.0,313.17166;14752800.0,310.54138;14756400.0,308.1587;14760000.0,305.83374;14763600.0,303.8328;14767200.0,302.1559;14770800.0,300.73047;14774400.0,299.40396;14778000.0,298.1711;14781600.0,297.07733;14785200.0,296.24265;14788800.0,295.29807;14792400.0,294.48654;14796000.0,294.47836;14799600.0,295.06113;14803200.0,296.70282;14806800.0,299.05548;14810400.0,302.06464;14814000.0,305.8574;14817600.0,309.64664;14821200.0,313.0021;14824800.0,314.5518;14828400.0,314.9337;14832000.0,314.63623;14835600.0,313.82898;14839200.0,312.41626;14842800.0,309.93298;14846400.0,307.31863;14850000.0,305.0506;14853600.0,303.04907;14857200.0,301.28375;14860800.0,299.7757;14864400.0,298.63495;14868000.0,297.649;14871600.0,296.79056;14875200.0,295.99258;14878800.0,295.28876;14882400.0,295.31003;14886000.0,296.48596;14889600.0,298.52563;14893200.0,300.9812;14896800.0,302.3711;14900400.0,302.8463;14904000.0,303.62222;14907600.0,304.5425;14911200.0,306.33463;14914800.0,308.7663;14918400.0,310.0728;14922000.0,310.19345;14925600.0,308.87018;14929200.0,307.02188;14932800.0,304.7351;14936400.0,302.58325;14940000.0,300.7062;14943600.0,299.25546;14947200.0,298.09702;14950800.0,296.9414;14954400.0,295.65543;14958000.0,294.35617;14961600.0,293.1274;14965200.0,292.0364;14968800.0,292.24075;14972400.0,293.70114;14976000.0,295.65823;14979600.0,298.07755;14983200.0,300.9578;14986800.0,304.52362;14990400.0,308.4952;14994000.0,312.00513;14997600.0,314.7596;15001200.0,316.07196;15004800.0,316.79703;15008400.0,317.03778;15012000.0,315.90894;15015600.0,314.2554;15019200.0,311.83316;15022800.0,309.46832;15026400.0,307.0723;15030000.0,304.6282;15033600.0,302.5869;15037200.0,300.7554;15040800.0,299.00488;15044400.0,297.25607;15048000.0,295.7224;15051600.0,294.61053;15055200.0,294.70972;15058800.0,296.0022;15062400.0,298.08286;15066000.0,300.75674;15069600.0,304.2341;15073200.0,308.37054;15076800.0,312.3382;15080400.0,315.5224;15084000.0,317.91632;15087600.0,319.51083;15091200.0,319.76028;15094800.0,319.4079;15098400.0,318.42963;15102000.0,316.9598;15105600.0,314.56973;15109200.0,311.99417;15112800.0,309.7758;15116400.0,307.7433;15120000.0,305.56967;15123600.0,303.7175;15127200.0,302.1565;15130800.0,300.72095;15134400.0,299.37177;15138000.0,298.06723;15141600.0,298.15;15145200.0,299.48242;15148800.0,301.92014;15152400.0,304.85223;15156000.0,307.63443;15159600.0,310.98828;15163200.0,314.64563;15166800.0,317.57373;15170400.0,319.61923;15174000.0,320.9226;15177600.0,321.28275;15181200.0,320.87076;15184800.0,319.84348;15188400.0,318.13052;15192000.0,315.5196;15195600.0,312.68066;15199200.0,309.95813;15202800.0,307.65887;15206400.0,305.21188;15210000.0,303.07724;15213600.0,301.15958;15217200.0,299.45715;15220800.0,297.80737;15224400.0,296.23898;15228000.0,296.1956;15231600.0,297.4229;15235200.0,299.59323;15238800.0,302.3403;15242400.0,305.85498;15246000.0,310.11035;15249600.0,314.47046;15253200.0,318.32004;15256800.0,321.27087;15260400.0,322.9944;15264000.0,323.78397;15267600.0,323.94284;15271200.0,323.29843;15274800.0,321.8643;15278400.0,319.3583;15282000.0,316.55215;15285600.0,313.76025;15289200.0,311.3526;15292800.0,309.28265;15296400.0,307.23657;15300000.0,305.05014;15303600.0,303.13177;15307200.0,301.34644;15310800.0,299.5413;15314400.0,299.04837;15318000.0,299.9361;15321600.0,301.6836;15325200.0,303.78003;15328800.0,306.33908;15332400.0,309.6936;15336000.0,313.27307;15339600.0,316.5495;15343200.0,319.37595;15346800.0,320.91412;15350400.0,321.21536;15354000.0,321.1855;15357600.0,320.476;15361200.0,318.69232;15364800.0,316.1697;15368400.0,313.63934;15372000.0,311.1967;15375600.0,308.91687;15379200.0,306.52457;15382800.0,304.46008;15386400.0,302.70633;15390000.0,300.93344;15393600.0,299.1421;15397200.0,297.3376;15400800.0,296.90247;15404400.0,297.85428;15408000.0,299.3814;15411600.0,301.32767;15415200.0,304.3647;15418800.0,308.26813;15422400.0,312.14938;15426000.0,315.75784;15429600.0,318.2979;15433200.0,320.07526;15436800.0,320.022;15440400.0,319.4872;15444000.0,318.23346;15447600.0,316.57968;15451200.0,314.2287;15454800.0,311.75253;15458400.0,309.35855;15462000.0,306.98834;15465600.0,304.92017;15469200.0,303.21304;15472800.0,301.7908;15476400.0,300.28604;15480000.0,298.75363;15483600.0,297.49152;15487200.0,297.57632;15490800.0,298.94885;15494400.0,301.0568;15498000.0,303.44882;15501600.0,305.0766;15505200.0,308.88205;15508800.0,313.35706;15512400.0,317.0986;15516000.0,319.5016;15519600.0,321.34473;15523200.0,320.8882;15526800.0,318.9086;15530400.0,317.37082;15534000.0,314.97327;15537600.0,312.34378;15541200.0,309.86124;15544800.0,307.74475;15548400.0,306.09235;15552000.0,304.30457;15555600.0,302.4201;15559200.0,300.8411;15562800.0,299.34692;15566400.0,298.03036;15570000.0,297.1649;15573600.0,297.398;15577200.0,298.90134;15580800.0,301.24472;15584400.0,304.04712;15588000.0,307.4319;15591600.0,310.3779;15595200.0,314.18765;15598800.0,317.46478;15602400.0,320.00888;15606000.0,321.26807;15609600.0,321.85867;15613200.0,321.83292;15616800.0,320.72052;15620400.0,318.66937;15624000.0,316.1207;15627600.0,313.82318;15631200.0,311.68307;15634800.0,309.5684;15638400.0,307.52734;15642000.0,305.5489;15645600.0,303.56366;15649200.0,301.64246;15652800.0,299.7615;15656400.0,297.9051;15660000.0,297.27475;15663600.0,297.92633;15667200.0,299.46704;15670800.0,301.36386;15674400.0,304.07263;15678000.0,307.47922;15681600.0,310.88873;15685200.0,313.8338;15688800.0,316.1494;15692400.0,317.30862;15696000.0,317.71948;15699600.0,317.5768;15703200.0,316.46127;15706800.0,314.40796;15710400.0,312.03683;15714000.0,309.5554;15717600.0,306.95013;15721200.0,304.78848;15724800.0,303.1656;15728400.0,301.3725;15732000.0,299.5761;15735600.0,297.82367;15739200.0,296.1827;15742800.0,294.87366;15746400.0,294.86578;15750000.0,295.9489;15753600.0,297.72565;15757200.0,300.07037;15760800.0,302.6412;15764400.0,305.7573;15768000.0,309.1963;15771600.0,312.5078;15775200.0,314.90112;15778800.0,316.2904;15782400.0,316.61465;15786000.0,316.3305;15789600.0,315.51;15793200.0,313.9283;15796800.0,311.62006;15800400.0,308.8636;15804000.0,306.17844;15807600.0,303.90027;15811200.0,301.88693;15814800.0,300.15366;15818400.0,298.67044;15822000.0,297.3449;15825600.0,296.15448;15829200.0,295.02432;15832800.0,294.9217;15836400.0,295.86835;15840000.0,297.5467;15843600.0,299.78482;15847200.0,302.59253;15850800.0,305.712;15854400.0,309.24374;15858000.0,312.342;15861600.0,314.63766;15865200.0,315.81348;15868800.0,316.3233;15872400.0,315.89398;15876000.0,314.43497;15879600.0,312.63516;15883200.0,310.35443;15886800.0,307.91312;15890400.0,305.50433;15894000.0,303.43655;15897600.0,301.48105;15901200.0,299.6707;15904800.0,297.95026;15908400.0,296.3754;15912000.0,295.1359;15915600.0,293.97372;15919200.0,294.0115;15922800.0,295.31967;15926400.0,297.45013;15930000.0,300.03094;15933600.0,303.55276;15937200.0,307.8582;15940800.0,311.55475;15944400.0,314.99835;15948000.0,317.84482;15951600.0,319.29474;15955200.0,319.68845;15958800.0,319.54865;15962400.0,318.81985;15966000.0,316.98676;15969600.0,314.38657;15973200.0,311.74832;15976800.0,309.34708;15980400.0,307.18814;15984000.0,305.1462;15987600.0,303.07422;15991200.0,301.41086;15994800.0,300.09088;15998400.0,298.88498;16002000.0,297.80566;16005600.0,297.84244;16009200.0,299.34186;16012800.0,301.54404;16016400.0,304.26047;16020000.0,307.9216;16023600.0,311.81613;16027200.0,316.06702;16030800.0,319.57898;16034400.0,322.08047;16038000.0,323.00995;16041600.0,323.156;16045200.0,322.05286;16048800.0,321.11758;16052400.0,319.67203;16056000.0,316.82083;16059600.0,313.7056;16063200.0,311.27737;16066800.0,309.1013;16070400.0,306.78873;16074000.0,304.62393;16077600.0,302.6344;16081200.0,300.7684;16084800.0,299.10098;16088400.0,297.67413;16092000.0,297.43744;16095600.0,298.53986;16099200.0,300.63184;16102800.0,302.13843;16106400.0,305.65292;16110000.0,310.5047;16113600.0,314.69223;16117200.0,316.5923;16120800.0,317.54187;16124400.0,318.95517;16128000.0,319.44705;16131600.0,319.1021;16135200.0,318.0882;16138800.0,316.11127;16142400.0,313.7726;16146000.0,311.36606;16149600.0,309.16037;16153200.0,307.2606;16156800.0,305.34866;16160400.0,303.49625;16164000.0,301.8705;16167600.0,300.37024;16171200.0,299.28592;16174800.0,298.46402;16178400.0,298.0995;16182000.0,298.33688;16185600.0,299.63593;16189200.0,301.50534;16192800.0,304.44177;16196400.0,307.42157;16200000.0,310.70416;16203600.0,313.71463;16207200.0,315.14505;16210800.0,316.50848;16214400.0,316.60358;16218000.0,315.20996;16221600.0,314.00974;16225200.0,312.10284;16228800.0,309.72638;16232400.0,307.576;16236000.0,305.6164;16239600.0,303.91052;16243200.0,302.39056;16246800.0,301.13196;16250400.0,299.95825;16254000.0,298.89383;16257600.0,297.858;16261200.0,296.85712;16264800.0,296.4004;16268400.0,296.34772;16272000.0,297.65683;16275600.0,299.45084;16279200.0,302.2848;16282800.0,306.42307;16286400.0,310.29462;16290000.0,313.21335;16293600.0,315.3777;16297200.0,315.11688;16300800.0,314.47934;16304400.0,313.41257;16308000.0,312.34824;16311600.0,310.60712;16315200.0,308.31262;16318800.0,306.2547;16322400.0,304.26627;16326000.0,302.4097;16329600.0,300.65002;16333200.0,299.05054;16336800.0,297.75797;16340400.0,296.48944;16344000.0,295.36914;16347600.0,294.4949;16351200.0,294.7118;16354800.0,296.26062;16358400.0,298.22055;16362000.0,300.68777;16365600.0,303.80777;16369200.0,306.5739;16372800.0,310.1331;16376400.0,314.09818;16380000.0,316.71332;16383600.0,317.6995;16387200.0,317.82867;16390800.0,317.31705;16394400.0,314.95462;16398000.0,311.73242;16401600.0,308.63113;16405200.0,305.93225;16408800.0,303.63174;16412400.0,301.5301;16416000.0,299.6606;16419600.0,298.03564;16423200.0,296.5768;16426800.0,295.34613;16430400.0,294.31778;16434000.0,293.43716;16437600.0,293.43616;16441200.0,294.71533;16444800.0,296.6334;16448400.0,298.94424;16452000.0,302.0292;16455600.0,306.42157;16459200.0,310.5216;16462800.0,314.0567;16466400.0,316.4991;16470000.0,318.2128;16473600.0,317.1966;16477200.0,315.8147;16480800.0,314.45975;16484400.0,312.49405;16488000.0,310.22537;16491600.0,307.96136;16495200.0,305.9558;16498800.0,304.22504;16502400.0,302.68124;16506000.0,301.17395;16509600.0,299.87686;16513200.0,298.53632;16516800.0,297.3535;16520400.0,296.3572;16524000.0,295.92886;16527600.0,296.10547;16531200.0,297.70486;16534800.0,299.93048;16538400.0,303.1207;16542000.0,306.8442;16545600.0,310.60464;16549200.0,313.49777;16552800.0,314.57428;16556400.0,314.8795;16560000.0,315.3666;16563600.0,314.8205;16567200.0,313.67264;16570800.0,311.7463;16574400.0,309.30936;16578000.0,306.9975;16581600.0,304.83875;16585200.0,302.91626;16588800.0,301.3355;16592400.0,299.8585;16596000.0,298.48923;16599600.0,297.14462;16603200.0,295.89154;16606800.0,294.8054;16610400.0,294.68085;16614000.0,295.83096;16617600.0,297.80624;16621200.0,300.26584;16624800.0,303.79333;16628400.0,308.3316;16632000.0,312.54364;16635600.0,314.2528;16639200.0,316.21515;16642800.0,315.22128;16646400.0,313.01843;16650000.0,311.60263;16653600.0,309.23386;16657200.0,306.8947;16660800.0,304.52603;16664400.0,302.36688;16668000.0,300.51282;16671600.0,298.8731;16675200.0,297.41873;16678800.0,296.0224;16682400.0,294.71918;16686000.0,293.62338;16689600.0,292.52014;16693200.0,291.43375;16696800.0,291.41998;16700400.0,292.78632;16704000.0,294.83508;16707600.0,297.29745;16711200.0,300.73178;16714800.0,304.76126;16718400.0,308.81616;16722000.0,312.44135;16725600.0,315.6733;16729200.0,317.0152;16732800.0,316.97043;16736400.0,316.4364;16740000.0,315.29672;16743600.0,313.709;16747200.0,311.30402;16750800.0,308.82413;16754400.0,306.54138;16758000.0,304.5352;16761600.0,302.70584;16765200.0,301.06277;16768800.0,299.59012;16772400.0,298.15045;16776000.0,296.68872;16779600.0,295.32764;16783200.0,295.28992;16786800.0,296.5175;16790400.0,298.3619;16794000.0,300.58884;16797600.0,303.7056;16801200.0,307.7253;16804800.0,311.86758;16808400.0,315.3302;16812000.0,317.8845;16815600.0,319.19302;16819200.0,319.2632;16822800.0,318.6931;16826400.0,317.5908;16830000.0,315.90668;16833600.0,313.45554;16837200.0,310.89282;16840800.0,308.66446;16844400.0,306.66684;16848000.0,304.72;16851600.0,302.8856;16855200.0,301.00214;16858800.0,299.23285;16862400.0,297.5007;16866000.0,295.8889;16869600.0,295.6945;16873200.0,297.01077;16876800.0,299.1238;16880400.0,302.00412;16884000.0,305.8271;16887600.0,310.20847;16891200.0,314.33847;16894800.0,318.115;16898400.0,320.79395;16902000.0,322.41162;16905600.0,322.70978;16909200.0,322.35986;16912800.0,320.7664;16916400.0,318.26584;16920000.0,315.34723;16923600.0,312.43982;16927200.0,309.5218;16930800.0,306.89737;16934400.0,304.6314;16938000.0,302.58807;16941600.0,300.60385;16945200.0,298.81314;16948800.0,297.1457;16952400.0,295.81992;16956000.0,295.64078;16959600.0,296.80313;16963200.0,299.1051;16966800.0,302.12463;16970400.0,306.08395;16974000.0,310.25714;16977600.0,314.3885;16981200.0,318.30682;16984800.0,321.10693;16988400.0,322.6702;16992000.0,323.1318;16995600.0,322.532;16999200.0,320.94183;17002800.0,318.48145;17006400.0,315.49228;17010000.0,312.75394;17013600.0,310.22977;17017200.0,307.91394;17020800.0,305.86652;17024400.0,304.12653;17028000.0,302.52347;17031600.0,301.02246;17035200.0,299.58884;17038800.0,298.2728;17042400.0,298.03143;17046000.0,299.1255;17049600.0,301.1541;17053200.0,303.74075;17056800.0,307.31567;17060400.0,311.9333;17064000.0,316.4049;17067600.0,320.14578;17071200.0,323.1775;17074800.0,324.5095;17078400.0,324.6305;17082000.0,323.53076;17085600.0,321.63364;17089200.0,318.6008;17092800.0,315.33786;17096400.0,312.51627;17100000.0,310.29874;17103600.0,308.4395;17107200.0,306.67896;17110800.0,304.8977;17114400.0,303.26663;17118000.0,301.77878;17121600.0,300.44174;17125200.0,299.1586;17128800.0,298.83704;17132400.0,299.8546;17136000.0,301.4051;17139600.0,303.90668;17143200.0,307.3534;17146800.0,311.58264;17150400.0,315.12662;17154000.0,317.7951;17157600.0,320.80756;17161200.0,320.8706;17164800.0,319.75293;17168400.0,318.35034;17172000.0,315.87128;17175600.0,313.107;17179200.0,310.55307;17182800.0,308.3398;17186400.0,306.19608;17190000.0,304.1294;17193600.0,302.13528;17197200.0,300.4347;17200800.0,298.8994;17204400.0,297.59814;17208000.0,296.5028;17211600.0,295.5666;17215200.0,295.38763;17218800.0,296.58502;17222400.0,298.5449;17226000.0,300.90448;17229600.0,304.04245;17233200.0,307.92712;17236800.0,312.16983;17240400.0,315.7233;17244000.0,318.5863;17247600.0,318.63495;17251200.0,318.30713;17254800.0,317.1944;17258400.0,315.19974;17262000.0,312.23367;17265600.0,309.43604;17269200.0,306.85614;17272800.0,304.6881;17276400.0,303.00183;17280000.0,301.49496;17283600.0,300.20087;17287200.0,299.1347;17290800.0,297.89377;17294400.0,296.6143;17298000.0,295.28653;17301600.0,295.09943;17305200.0,296.28;17308800.0,298.42615;17312400.0,301.0476;17316000.0,304.19897;17319600.0,308.5922;17323200.0,312.5725;17326800.0,315.99072;17330400.0,319.0217;17334000.0,319.60202;17337600.0,317.37573;17341200.0,315.26743;17344800.0,312.68527;17348400.0,310.10776;17352000.0,307.67862;17355600.0,305.4771;17359200.0,303.56055;17362800.0,301.79813;17366400.0,300.33572;17370000.0,298.88074;17373600.0,297.56003;17377200.0,296.4362;17380800.0,295.59164;17384400.0,294.96982;17388000.0,294.8696;17391600.0,295.62997;17395200.0,297.38416;17398800.0,300.00967;17402400.0,303.86688;17406000.0,308.28586;17409600.0,312.15186;17413200.0,314.80194;17416800.0,315.98535;17420400.0,316.78192;17424000.0,317.1425;17427600.0,316.59616;17431200.0,314.55164;17434800.0,312.14102;17438400.0,309.74005;17442000.0,307.6882;17445600.0,305.7691;17449200.0,304.03354;17452800.0,302.60196;17456400.0,301.35226;17460000.0,300.1514;17463600.0,298.98047;17467200.0,297.89606;17470800.0,296.99686;17474400.0,296.55374;17478000.0,297.0593;17481600.0,298.4708;17485200.0,300.36774;17488800.0,302.05447;17492400.0,304.7867;17496000.0,308.68576;17499600.0,311.6848;17503200.0,311.1117;17506800.0,309.53482;17510400.0,308.0389;17514000.0,306.3911;17517600.0,304.49277;17521200.0,302.44684;17524800.0,300.53204;17528400.0,298.84485;17532000.0,297.32642;17535600.0,296.1058;17539200.0,295.0978;17542800.0,294.22684;17546400.0,293.4731;17550000.0,292.85107;17553600.0,292.23355;17557200.0,291.77786;17560800.0,291.41116;17564400.0,291.21274;17568000.0,291.46323;17571600.0,292.3888;17575200.0,293.38824;17578800.0,294.1049;17582400.0,295.4153;17586000.0,296.79276;17589600.0,298.1234;17593200.0,300.69476;17596800.0,300.80914;17600400.0,300.22775;17604000.0,299.31058;17607600.0,298.284;17611200.0,297.02243;17614800.0,295.84933;17618400.0,294.74036;17622000.0,293.846;17625600.0,293.14572;17629200.0,292.46298;17632800.0,291.8136;17636400.0,291.4079;17640000.0,291.08118;17643600.0,290.84998;17647200.0,290.85217;17650800.0,291.37637;17654400.0,293.34854;17658000.0,294.7909;17661600.0,296.13406;17665200.0,297.78207;17668800.0,299.23056;17672400.0,300.4164;17676000.0,301.22336;17679600.0,301.47974;17683200.0,301.64648;17686800.0,301.50964;17690400.0,301.17026;17694000.0,300.20346;17697600.0,298.8519;17701200.0,297.50916;17704800.0,296.45804;17708400.0,295.3771;17712000.0,294.30844;17715600.0,293.1603;17719200.0,292.1134;17722800.0,291.1382;17726400.0,290.31772;17730000.0,289.66125;17733600.0,289.48935;17737200.0,290.2607;17740800.0,292.6335;17744400.0,295.34235;17748000.0,299.01544;17751600.0,303.75858;17755200.0,308.51022;17758800.0,312.78012;17762400.0,313.82617;17766000.0,311.1088;17769600.0,309.78854;17773200.0,309.4298;17776800.0,308.6732;17780400.0,306.80157;17784000.0,304.5378;17787600.0,302.29727;17791200.0,300.09766;17794800.0,298.3247;17798400.0,296.78754;17802000.0,295.37128;17805600.0,294.03665;17809200.0,292.7698;17812800.0,291.68536;17816400.0,290.73355;17820000.0,290.5985;17823600.0,291.90106;17827200.0,293.93448;17830800.0,296.8142;17834400.0,300.7475;17838000.0,305.51743;17841600.0,309.3768;17845200.0,310.78174;17848800.0,312.4364;17852400.0,310.82318;17856000.0,309.57358;17859600.0,308.77054;17863200.0,307.92813;17866800.0,306.36832;17870400.0,304.17523;17874000.0,302.019;17877600.0,300.13318;17881200.0,298.43768;17884800.0,296.8258;17888400.0,295.31744;17892000.0,294.0295;17895600.0,292.79474;17899200.0,291.76932;17902800.0,290.86365;17906400.0,290.74414;17910000.0,292.1134;17913600.0,294.35516;17917200.0,297.41074;17920800.0,301.21756;17924400.0,305.5845;17928000.0,309.56757;17931600.0,313.81802;17935200.0,316.5294;17938800.0,317.88846;17942400.0,318.0841;17946000.0,317.34772;17949600.0,316.06076;17953200.0,314.24545;17956800.0,311.74948;17960400.0,309.17743;17964000.0,306.73923;17967600.0,304.4995;17971200.0,302.5763;17974800.0,300.75308;17978400.0,299.04608;17982000.0,297.48077;17985600.0,296.09256;17989200.0,294.84546;17992800.0,294.41843;17996400.0,295.4387;18000000.0,297.4258;18003600.0,300.31375;18007200.0,304.3386;18010800.0,309.234;18014400.0,314.16724;18018000.0,318.05368;18021600.0,320.97253;18025200.0,322.33972;18028800.0,322.14526;18032400.0,321.19034;18036000.0,319.73898;18039600.0,317.55087;18043200.0,314.64465;18046800.0,311.87442;18050400.0,309.69305;18054000.0,307.66336;18057600.0,305.66568;18061200.0,303.69852;18064800.0,301.85736;18068400.0,300.0505;18072000.0,298.50357;18075600.0,297.19864;18079200.0,296.48004;18082800.0,297.18655;18086400.0,299.0307;18090000.0,301.87546;18093600.0,305.95105;18097200.0,310.75018;18100800.0,315.6296;18104400.0,319.70654;18108000.0,322.8432;18111600.0,324.17975;18115200.0,324.16678;18118800.0,323.29565;18122400.0,321.67764;18126000.0,319.2347;18129600.0,315.90164;18133200.0,312.57993;18136800.0,310.0672;18140400.0,307.86237;18144000.0,305.72583;18147600.0,303.87042;18151200.0,302.19128;18154800.0,300.80646;18158400.0,299.43182;18162000.0,298.04514;18165600.0,297.5226;18169200.0,298.41165;18172800.0,300.30304;18176400.0,302.84778;18180000.0,306.97604;18183600.0,311.6561;18187200.0,316.78452;18190800.0,319.2912;18194400.0,321.1704;18198000.0,322.28882;18201600.0,320.8073;18205200.0,319.20142;18208800.0,317.40668;18212400.0,315.09592;18216000.0,312.57327;18219600.0,310.01505;18223200.0,307.5666;18226800.0,305.3972;18230400.0,303.42682;18234000.0,301.64572;18237600.0,300.07007;18241200.0,298.65814;18244800.0,297.2988;18248400.0,296.04688;18252000.0,295.51944;18255600.0,296.16205;18259200.0,297.7399;18262800.0,300.25848;18266400.0,303.9072;18270000.0,308.4136;18273600.0,313.32654;18277200.0,317.54425;18280800.0,320.77835;18284400.0,322.82877;18288000.0,322.69034;18291600.0,321.8104;18295200.0,320.06525;18298800.0,317.86224;18302400.0,315.14725;18306000.0,312.45795;18309600.0,310.04865;18313200.0,307.93652;18316800.0,306.02347;18320400.0,304.25433;18324000.0,302.69278;18327600.0,301.23135;18331200.0,299.85248;18334800.0,298.8156;18338400.0,297.90274;18342000.0,298.30106;18345600.0,298.85422;18349200.0,299.93686;18352800.0,303.2253;18356400.0,307.8402;18360000.0,312.639;18363600.0,316.84036;18367200.0,320.00845;18370800.0,319.96613;18374400.0,318.1542;18378000.0,316.5216;18381600.0,314.24564;18385200.0,311.65118;18388800.0,308.79163;18392400.0,306.19797;18396000.0,304.1958;18399600.0,302.44327;18403200.0,301.10593;18406800.0,299.95227;18410400.0,298.90292;18414000.0,297.7859;18417600.0,296.7744;18421200.0,295.97336;18424800.0,295.53885;18428400.0,296.49203;18432000.0,298.51456;18435600.0,301.17395;18439200.0,304.95404;18442800.0,309.4685;18446400.0,314.2613;18450000.0,318.4773;18453600.0,321.4808;18457200.0,318.78156;18460800.0,315.88284;18464400.0,313.48904;18468000.0,311.95938;18471600.0,310.1271;18475200.0,307.8474;18478800.0,305.5369;18482400.0,303.78406;18486000.0,302.29138;18489600.0,300.7464;18493200.0,299.19962;18496800.0,297.90427;18500400.0,296.88602;18504000.0,295.7373;18507600.0,294.91;18511200.0,294.32025;18514800.0,295.24957;18518400.0,297.2768;18522000.0,300.0661;18525600.0,304.09756;18529200.0,308.94794;18532800.0,314.1958;18536400.0,318.61588;18540000.0,321.86813;18543600.0,323.4558;18547200.0,323.554;18550800.0,322.7606;18554400.0,321.63184;18558000.0,319.39117;18561600.0,316.38782;18565200.0,313.7366;18568800.0,311.4897;18572400.0,309.48975;18576000.0,307.8133;18579600.0,305.98926;18583200.0,304.1919;18586800.0,302.39856;18590400.0,300.81604;18594000.0,299.7158;18597600.0,298.8626;18601200.0,299.4849;18604800.0,301.2098;18608400.0,303.89746;18612000.0,307.60452;18615600.0,312.05768;18619200.0,316.59344;18622800.0,320.68463;18626400.0,323.8161;18630000.0,324.8994;18633600.0,324.934;18637200.0,323.25446;18640800.0,320.64;18644400.0,318.13895;18648000.0,315.57028;18651600.0,313.2197;18655200.0,311.07864;18658800.0,309.1956;18662400.0,307.58054;18666000.0,306.1864;18669600.0,304.75955;18673200.0,303.41266;18676800.0,302.10666;18680400.0,300.97485;18684000.0,300.13876;18687600.0,300.76572;18691200.0,302.73654;18694800.0,305.78772;18698400.0,310.0295;18702000.0,314.59918;18705600.0,319.2727;18709200.0,323.13794;18712800.0,325.92456;18716400.0,327.20892;18720000.0,326.82285;18723600.0,325.55612;18727200.0,323.97696;18730800.0,321.086;18734400.0,317.83063;18738000.0,315.15314;18741600.0,312.76624;18745200.0,310.57132;18748800.0,308.52634;18752400.0,306.72427;18756000.0,305.19894;18759600.0,303.8908;18763200.0,302.62894;18766800.0,301.39325;18770400.0,300.5371;18774000.0,300.76956;18777600.0,302.27502;18781200.0,304.8148;18784800.0,308.67157;18788400.0,313.4135;18792000.0,318.26917;18795600.0,322.2111;18799200.0,324.8896;18802800.0,325.4353;18806400.0,322.28497;18810000.0,318.58594;18813600.0,315.17346;18817200.0,312.50528;18820800.0,310.02505;18824400.0,307.73224;18828000.0,305.49698;18831600.0,303.73904;18835200.0,302.0929;18838800.0,300.74918;18842400.0,299.6453;18846000.0,298.76065;18849600.0,297.67477;18853200.0,296.46008;18856800.0,296.0102;18860400.0,296.8461;18864000.0,298.65384;18867600.0,301.55646;18871200.0,305.8395;18874800.0,311.19016;18878400.0,316.33365;18882000.0,320.66034;18885600.0,323.98752;18889200.0,324.09357;18892800.0,324.0152;18896400.0,322.62903;18900000.0,320.89578;18903600.0,318.43692;18907200.0,315.32434;18910800.0,312.23425;18914400.0,309.63187;18918000.0,307.39542;18921600.0,305.56122;18925200.0,303.86957;18928800.0,302.22156;18932400.0,300.73395;18936000.0,299.63263;18939600.0,298.39062;18943200.0,297.486;18946800.0,298.1617;18950400.0,299.9896;18954000.0,302.79608;18957600.0,306.86844;18961200.0,312.10168;18964800.0,317.58868;18968400.0,321.9707;18972000.0,325.2859;18975600.0,326.5617;18979200.0,326.07587;18982800.0,324.3196;18986400.0,321.8853;18990000.0,318.5377;18993600.0,315.12042;18997200.0,312.16068;19000800.0,309.6492;19004400.0,307.22043;19008000.0,304.9451;19011600.0,303.10876;19015200.0,301.63766;19018800.0,300.31354;19022400.0,299.1765;19026000.0,298.1859;19029600.0,297.29153;19033200.0,296.7549;19036800.0,296.38086;19040400.0,296.15836;19044000.0,296.13187;19047600.0,296.15106;19051200.0,296.47165;19054800.0,297.6843;19058400.0,298.4948;19062000.0,300.55072;19065600.0,301.58817;19069200.0,302.09225;19072800.0,301.53778;19076400.0,300.27325;19080000.0,298.9472;19083600.0,297.7008;19087200.0,296.51102;19090800.0,295.03592;19094400.0,293.8711;19098000.0,292.97757;19101600.0,292.1928;19105200.0,291.5747;19108800.0,291.03687;19112400.0,290.46906;19116000.0,289.9691;19119600.0,290.04672;19123200.0,290.55737;19126800.0,291.5325;19130400.0,295.6715;19134000.0,301.5791;19137600.0,307.3998;19141200.0,309.971;19144800.0,311.74063;19148400.0,314.46994;19152000.0,315.16315;19155600.0,312.83963;19159200.0,310.15836;19162800.0,307.6095;19166400.0,305.2396;19170000.0,302.97458;19173600.0,300.9657;19177200.0,299.24802;19180800.0,297.78348;19184400.0,296.4004;19188000.0,295.13318;19191600.0,294.04587;19195200.0,292.91254;19198800.0,291.98703;19202400.0,291.4399;19206000.0,291.84213;19209600.0,293.8348;19213200.0,297.0531;19216800.0,301.2109;19220400.0,306.4751;19224000.0,311.9114;19227600.0,316.6959;19231200.0,320.31778;19234800.0,321.94562;19238400.0,321.95474;19242000.0,320.41727;19245600.0,318.45847;19249200.0,315.75485;19252800.0,312.7691;19256400.0,310.0717;19260000.0,307.63837;19263600.0,305.56638;19267200.0,303.48804;19270800.0,301.51083;19274400.0,300.09525;19278000.0,298.9085;19281600.0,297.5178;19285200.0,296.33826;19288800.0,295.3583;19292400.0,295.97095;19296000.0,298.0102;19299600.0,300.98776;19303200.0,305.17877;19306800.0,310.34003;19310400.0,315.46182;19314000.0,319.9058;19317600.0,323.08316;19321200.0,324.64703;19324800.0,324.55145;19328400.0,323.14905;19332000.0,321.19266;19335600.0,318.26154;19339200.0,315.02515;19342800.0,311.96136;19346400.0,309.14352;19350000.0,306.6825;19353600.0,304.5181;19357200.0,302.4967;19360800.0,300.72897;19364400.0,299.01205;19368000.0,297.6225;19371600.0,296.27936;19375200.0,295.23218;19378800.0,295.82224;19382400.0,297.28418;19386000.0,300.1153;19389600.0,304.1227;19393200.0,309.2086;19396800.0,314.37335;19400400.0,316.03888;19404000.0,317.7773;19407600.0,316.10242;19411200.0,315.84647;19414800.0,313.5263;19418400.0,311.13882;19422000.0,308.98926;19425600.0,306.71484;19429200.0,304.80508;19432800.0,303.12354;19436400.0,301.39322;19440000.0,299.97754;19443600.0,298.78583;19447200.0,297.7795;19450800.0,296.86563;19454400.0,296.2099;19458000.0,295.5839;19461600.0,294.96313;19465200.0,295.61707;19468800.0,297.10385;19472400.0,299.8119;19476000.0,301.9677;19479600.0,306.72766;19483200.0,308.93716;19486800.0,309.5971;19490400.0,311.0829;19494000.0,311.31458;19497600.0,310.26425;19501200.0,309.47116;19504800.0,308.4436;19508400.0,306.53357;19512000.0,304.25638;19515600.0,302.42038;19519200.0,300.882;19522800.0,299.47867;19526400.0,298.47623;19530000.0,297.67526;19533600.0,296.8259;19537200.0,296.0036;19540800.0,295.11087;19544400.0,294.33875;19548000.0,293.78537;19551600.0,294.2959;19555200.0,296.1981;19558800.0,299.53116;19562400.0,303.35303;19566000.0,307.71674;19569600.0,313.0831;19573200.0,315.93304;19576800.0,314.07016;19580400.0,312.06546;19584000.0,311.92514;19587600.0,310.42703;19591200.0,308.66388;19594800.0,306.7423;19598400.0,304.56512;19602000.0,302.58768;19605600.0,300.79352;19609200.0,299.2009;19612800.0,297.8529;19616400.0,296.7841;19620000.0,295.76285;19623600.0,294.69098;19627200.0,293.52313;19630800.0,292.4287;19634400.0,291.6108;19638000.0,292.33936;19641600.0,294.4088;19645200.0,297.99298;19648800.0,301.03085;19652400.0,303.94348;19656000.0,309.89587;19659600.0,315.33234;19663200.0,319.28708;19666800.0,321.00244;19670400.0,319.95822;19674000.0,317.8755;19677600.0,315.5509;19681200.0,312.63336;19684800.0,309.25662;19688400.0,306.36646;19692000.0,303.9546;19695600.0,301.58206;19699200.0,299.58282;19702800.0,297.90488;19706400.0,296.50366;19710000.0,295.32358;19713600.0,294.09857;19717200.0,292.75793;19720800.0,291.60538;19724400.0,292.17792;19728000.0,294.1764;19731600.0,297.5948;19735200.0,302.14136;19738800.0,307.64038;19742400.0,313.3996;19746000.0,318.2785;19749600.0,321.83194;19753200.0,323.3876;19756800.0,322.99393;19760400.0,321.3714;19764000.0,319.2127;19767600.0,316.18396;19771200.0,313.17874;19774800.0,310.12207;19778400.0,307.30255;19782000.0,304.96204;19785600.0,302.94894;19789200.0,301.14877;19792800.0,299.5469;19796400.0,298.20035;19800000.0,296.85205;19803600.0,295.571;19807200.0,294.53772;19810800.0,295.1157;19814400.0,296.9693;19818000.0,300.12622;19821600.0,304.7968;19825200.0,310.43008;19828800.0,316.19852;19832400.0,321.06378;19836000.0,324.68622;19839600.0,326.5182;19843200.0,326.37045;19846800.0,324.68298;19850400.0,322.32214;19854000.0,319.08304;19857600.0,315.42242;19861200.0,312.37473;19864800.0,309.9537;19868400.0,307.678;19872000.0,305.6857;19875600.0,303.86377;19879200.0,302.2665;19882800.0,300.80936;19886400.0,299.65652;19890000.0,298.64017;19893600.0,297.9368;19897200.0,298.60666;19900800.0,300.73608;19904400.0,304.2978;19908000.0,309.16504;19911600.0,315.0568;19915200.0,321.40442;19918800.0,325.79028;19922400.0,328.39465;19926000.0,326.90015;19929600.0,324.87375;19933200.0,322.4087;19936800.0,319.7215;19940400.0,316.63672;19944000.0,313.46072;19947600.0,310.7671;19951200.0,308.45337;19954800.0,306.36246;19958400.0,304.5988;19962000.0,302.79926;19965600.0,301.44736;19969200.0,300.37027;19972800.0,299.4644;19976400.0,298.60458;19980000.0,298.03143;19983600.0,298.9197;19987200.0,300.9315;19990800.0,304.28925;19994400.0,308.62512;19998000.0,310.9424;20001600.0,315.87146;20005200.0,320.74084;20008800.0,320.04553;20012400.0,317.97247;20016000.0,316.1666;20019600.0,315.41605;20023200.0,313.50507;20026800.0,311.16794;20030400.0,308.87814;20034000.0,306.85223;20037600.0,305.19647;20041200.0,303.3704;20044800.0,301.54153;20048400.0,300.3951;20052000.0,299.16586;20055600.0,298.2275;20059200.0,297.3853;20062800.0,296.6733;20066400.0,296.1077;20070000.0,296.88257;20073600.0,299.06247;20077200.0,302.55365;20080800.0,307.2286;20084400.0,312.08466;20088000.0,317.67737;20091600.0,322.38327;20095200.0,325.91998;20098800.0,327.68558;20102400.0,327.3459;20106000.0,325.41302;20109600.0,322.0539;20113200.0,318.66888;20116800.0,315.54483;20120400.0,312.88248;20124000.0,310.35516;20127600.0,308.0733;20131200.0,306.05695;20134800.0,304.2026;20138400.0,302.62814;20142000.0,300.95602;20145600.0,299.49606;20149200.0,298.3592;20152800.0,297.56528;20156400.0,297.9054;20160000.0,298.75223;20163600.0,300.3816;20167200.0,304.90363;20170800.0,310.32254;20174400.0,315.18414;20178000.0,319.14746;20181600.0,321.03647;20185200.0,320.35837;20188800.0,318.3663;20192400.0,315.2569;20196000.0,312.27518;20199600.0,309.64813;20203200.0,307.23935;20206800.0,305.10727;20210400.0,303.26263;20214000.0,301.90106;20217600.0,300.586;20221200.0,299.15176;20224800.0,297.76797;20228400.0,296.58942;20232000.0,295.5928;20235600.0,294.6202;20239200.0,293.79242;20242800.0,294.51556;20246400.0,296.63858;20250000.0,298.66797;20253600.0,299.46555;20257200.0,302.91336;20260800.0,308.99753;20264400.0,314.8449;20268000.0,319.4333;20271600.0,321.6299;20275200.0,321.71213;20278800.0,320.39752;20282400.0,318.5407;20286000.0,315.485;20289600.0,312.3833;20293200.0,309.70612;20296800.0,307.11133;20300400.0,304.95618;20304000.0,303.27014;20307600.0,301.6691;20311200.0,300.25336;20314800.0,298.8217;20318400.0,297.65448;20322000.0,296.55353;20325600.0,295.71777;20329200.0,296.26724;20332800.0,298.4938;20336400.0,302.18588;20340000.0,307.34286;20343600.0,313.41953;20347200.0,319.67163;20350800.0,325.0163;20354400.0,327.9884;20358000.0,329.35077;20361600.0,328.55182;20365200.0,325.7409;20368800.0,322.69016;20372400.0,319.31992;20376000.0,316.27216;20379600.0,313.88992;20383200.0,311.68414;20386800.0,309.53326;20390400.0,308.21777;20394000.0,306.55313;20397600.0,304.8732;20401200.0,303.08258;20404800.0,301.53845;20408400.0,300.11847;20412000.0,299.18826;20415600.0,299.0726;20419200.0,299.03265;20422800.0,299.04355;20426400.0,299.61102;20430000.0,301.00766;20433600.0,305.47467;20437200.0,311.16806;20440800.0,315.32025;20444400.0,316.2067;20448000.0,317.11066;20451600.0,316.3262;20455200.0,314.68713;20458800.0,311.93176;20462400.0,308.66397;20466000.0,305.82108;20469600.0,303.44345;20473200.0,301.16724;20476800.0,299.529;20480400.0,298.29114;20484000.0,297.16257;20487600.0,296.00308;20491200.0,295.14337;20494800.0,294.34167;20498400.0,293.6514;20502000.0,294.2625;20505600.0,296.39462;20509200.0,300.32855;20512800.0,305.7516;20516400.0,311.87732;20520000.0,318.15582;20523600.0,323.65698;20527200.0,327.5454;20530800.0,329.5293;20534400.0,329.39313;20538000.0,327.52625;20541600.0,324.65247;20545200.0,320.8617;20548800.0,317.15085;20552400.0,313.70703;20556000.0,310.9534;20559600.0,308.3698;20563200.0,306.07483;20566800.0,304.29077;20570400.0,302.7846;20574000.0,301.03143;20577600.0,299.52744;20581200.0,298.12622;20584800.0,297.00647;20588400.0,297.57193;20592000.0,299.5428;20595600.0,303.09036;20599200.0,308.40945;20602800.0,314.4947;20606400.0,320.65027;20610000.0,325.68115;20613600.0,325.79175;20617200.0,326.0496;20620800.0,325.50446;20624400.0,323.1327;20628000.0,320.74182;20631600.0,317.58185;20635200.0,314.56735;20638800.0,311.9416;20642400.0,309.57947;20646000.0,307.58167;20649600.0,305.75443;20653200.0,304.12323;20656800.0,302.6118;20660400.0,301.21967;20664000.0,300.02576;20667600.0,298.61066;20671200.0,297.63333;20674800.0,297.5518;20678400.0,299.20474;20682000.0,302.54593;20685600.0,307.40585;20689200.0,313.33994;20692800.0,319.15082;20696400.0,320.91537;20700000.0,323.73593;20703600.0,324.42618;20707200.0,324.12354;20710800.0,321.4402;20714400.0,319.01285;20718000.0,316.0575;20721600.0,312.6989;20725200.0,309.84927;20728800.0,307.21146;20732400.0,305.25272;20736000.0,303.55856;20739600.0,301.96533;20743200.0,300.46384;20746800.0,299.06705;20750400.0,297.89304;20754000.0,296.81256;20757600.0,295.99292;20761200.0,296.37732;20764800.0,298.59354;20768400.0,302.5262;20772000.0,308.07202;20775600.0,314.27396;20779200.0,320.26578;20782800.0,325.2119;20786400.0,329.0053;20790000.0,330.65894;20793600.0,329.11548;20797200.0,326.26575;20800800.0,323.0214;20804400.0,319.14685;20808000.0,315.50015;20811600.0,312.43832;20815200.0,309.64502;20818800.0,307.1982;20822400.0,305.0804;20826000.0,303.01962;20829600.0,301.0518;20833200.0,299.26672;20836800.0,297.5705;20840400.0,295.99142;20844000.0,294.63742;20847600.0,294.7761;20851200.0,296.5307;20854800.0,299.9877;20858400.0,304.88742;20862000.0,310.59998;20865600.0,316.09894;20869200.0,321.26715;20872800.0,324.925;20876400.0,324.6196;20880000.0,320.5465;20883600.0,317.37393;20887200.0,315.08966;20890800.0,311.8734;20894400.0,308.8887;20898000.0,306.56418;20901600.0,304.62305;20905200.0,302.93173;20908800.0,301.14212;20912400.0,299.18692;20916000.0,297.7034;20919600.0,296.33878;20923200.0,295.2176;20926800.0,294.19107;20930400.0,293.40335;20934000.0,293.64005;20937600.0,295.5253;20941200.0,298.18845;20944800.0,302.48315;20948400.0,307.40973;20952000.0,313.41373;20955600.0,314.31683;20959200.0,313.5101;20962800.0,314.75583;20966400.0,313.4809;20970000.0,311.5452;20973600.0,309.69138;20977200.0,307.3976;20980800.0,305.28036;20984400.0,303.21213;20988000.0,301.31207;20991600.0,299.60638;20995200.0,298.04434;20998800.0,296.52795;21002400.0,295.0276;21006000.0,293.66043;21009600.0,292.36484;21013200.0,290.96286;21016800.0,289.8081;21020400.0,290.5084;21024000.0,292.7833;21027600.0,297.02112;21031200.0,303.2425;21034800.0,310.30408;21038400.0,317.4343;21042000.0,323.07803;21045600.0,326.60562;21049200.0,328.4782;21052800.0,327.9649;21056400.0,325.75595;21060000.0,322.78052;21063600.0,318.3992;21067200.0,314.2508;21070800.0,310.65286;21074400.0,307.62704;21078000.0,305.13437;21081600.0,302.89468;21085200.0,300.91663;21088800.0,299.01422;21092400.0,297.34607;21096000.0,295.91672;21099600.0,294.58908;21103200.0,293.57025;21106800.0,294.14847;21110400.0,296.41742;21114000.0,300.93997;21117600.0,306.92285;21121200.0,313.52866;21124800.0,320.25272;21128400.0,325.99588;21132000.0,330.27817;21135600.0,332.41904;21139200.0,331.27673;21142800.0,328.39328;21146400.0,324.92557;21150000.0,320.65668;21153600.0,316.48004;21157200.0,312.62546;21160800.0,309.45947;21164400.0,306.8481;21168000.0,304.4644;21171600.0,302.24524;21175200.0,300.88483;21178800.0,299.16733;21182400.0,297.53076;21186000.0,296.35855;21189600.0,295.1993;21193200.0,295.73843;21196800.0,298.02924;21200400.0,302.3459;21204000.0,308.36493;21207600.0,315.1466;21211200.0,322.039;21214800.0,327.00186;21218400.0,331.2429;21222000.0,332.71863;21225600.0,331.46545;21229200.0,328.69214;21232800.0,325.01556;21236400.0,320.724;21240000.0,316.28955;21243600.0,312.40967;21247200.0,308.9514;21250800.0,306.22983;21254400.0,303.7763;21258000.0,302.4063;21261600.0,301.15866;21265200.0,299.68674;21268800.0,298.43402;21272400.0,297.36118;21276000.0,296.3652;21279600.0,296.85214;21283200.0,299.07855;21286800.0,303.50394;21290400.0,309.4158;21294000.0,316.1343;21297600.0,322.4875;21301200.0,327.58795;21304800.0,331.46237;21308400.0,333.37814;21312000.0,332.82343;21315600.0,330.2411;21319200.0,326.52136;21322800.0,322.09427;21326400.0,318.09198;21330000.0,314.8723;21333600.0,312.14172;21337200.0,309.96664;21340800.0,307.82288;21344400.0,305.4676;21348000.0,303.6153;21351600.0,301.86954;21355200.0,300.21304;21358800.0,298.68515;21362400.0,297.43958;21366000.0,297.45493;21369600.0,298.8506;21373200.0,303.3572;21376800.0,309.46298;21380400.0,316.37302;21384000.0,322.76627;21387600.0,327.45947;21391200.0,327.57254;21394800.0,328.30563;21398400.0,325.88422;21402000.0,324.21643;21405600.0,321.43436;21409200.0,317.85776;21412800.0,314.26947;21416400.0,310.91376;21420000.0,308.2552;21423600.0,305.80258;21427200.0,304.04263;21430800.0,301.96655;21434400.0,300.01907;21438000.0,298.56638;21441600.0,297.2508;21445200.0,296.29037;21448800.0,295.3426;21452400.0,295.4073;21456000.0,296.8519;21459600.0,299.53574;21463200.0,302.21002;21466800.0,304.59778;21470400.0,305.27783;21474000.0,312.12296;21477600.0,316.82117;21481200.0,316.42896;21484800.0,316.13422;21488400.0,315.47675;21492000.0,312.9352;21495600.0,310.12567;21499200.0,307.52692;21502800.0,305.2619;21506400.0,303.1276;21510000.0,300.90854;21513600.0,298.90347;21517200.0,297.36545;21520800.0,296.02505;21524400.0,294.97906;21528000.0,293.8425;21531600.0,292.9877;21535200.0,292.32965;21538800.0,292.9793;21542400.0,295.481;21546000.0,299.94907;21549600.0,306.26218;21553200.0,313.27396;21556800.0,320.1149;21560400.0,325.77374;21564000.0,330.00043;21567600.0,332.2653;21571200.0,332.15576;21574800.0,329.76178;21578400.0,326.17035;21582000.0,321.57794;21585600.0,317.58932;21589200.0,314.33014;21592800.0,311.24057;21596400.0,308.55762;21600000.0,306.2471;21603600.0,304.10107;21607200.0,302.13602;21610800.0,300.4556;21614400.0,298.56653;21618000.0,297.24863;21621600.0,296.04352;21625200.0,296.5701;21628800.0,299.2406;21632400.0,303.66617;21636000.0,309.36438;21639600.0,312.92572;21643200.0,320.33737;21646800.0,326.22598;21650400.0,327.97833;21654000.0,329.93637;21657600.0,327.00394;21661200.0,323.83545;21664800.0,320.57333;21668400.0,317.16812;21672000.0,314.14517;21675600.0,311.64948;21679200.0,309.5762;21682800.0,307.71213;21686400.0,306.1064;21690000.0,304.66934;21693600.0,303.37088;21697200.0,302.0604;21700800.0,300.17694;21704400.0,298.9603;21708000.0,297.91315;21711600.0,298.51675;21715200.0,301.08475;21718800.0,305.71698;21722400.0,311.0461;21726000.0,313.95532;21729600.0,320.71088;21733200.0,326.51166;21736800.0,329.37946;21740400.0,327.39706;21744000.0,323.99844;21747600.0,320.31082;21751200.0,316.98587;21754800.0,313.7016;21758400.0,310.65802;21762000.0,308.24762;21765600.0,306.2386;21769200.0,304.63748;21772800.0,303.3366;21776400.0,302.05133;21780000.0,301.0167;21783600.0,300.1961;21787200.0,299.42886;21790800.0,298.7609;21794400.0,298.1241;21798000.0,298.10266;21801600.0,298.93625;21805200.0,301.1556;21808800.0,306.24393;21812400.0,313.53476;21816000.0,320.52316;21819600.0,325.87204;21823200.0,330.09225;21826800.0,331.98947;21830400.0,331.58096;21834000.0,329.17532;21837600.0,325.5131;21841200.0,321.15622;21844800.0,316.7939;21848400.0,312.93216;21852000.0,310.06348;21855600.0,307.79095;21859200.0,305.63873;21862800.0,303.71075;21866400.0,302.12372;21870000.0,300.43686;21873600.0,298.80756;21877200.0,297.31296;21880800.0,295.99725;21884400.0,296.4561;21888000.0,298.9117;21891600.0,303.48883;21895200.0,309.32983;21898800.0,316.0436;21902400.0,322.6629;21906000.0,328.20013;21909600.0,332.5094;21913200.0,334.61212;21916800.0,333.7208;21920400.0,330.6521;21924000.0,326.44067;21927600.0,321.46207;21931200.0,317.04602;21934800.0,313.5672;21938400.0,310.69537;21942000.0,308.04977;21945600.0,305.69855;21949200.0,303.38916;21952800.0,301.38913;21956400.0,299.66562;21960000.0,298.4873;21963600.0,296.76617;21967200.0,294.70737;21970800.0,294.94257;21974400.0,297.3786;21978000.0,302.15454;21981600.0,308.3178;21985200.0,315.15634;21988800.0,322.42953;21992400.0,328.10782;21996000.0,332.32394;21999600.0,334.12122;22003200.0,333.01813;22006800.0,329.9634;22010400.0,325.21814;22014000.0,320.43118;22017600.0,316.31168;22021200.0,312.34348;22024800.0,308.55743;22028400.0,305.17624;22032000.0,302.29974;22035600.0,299.42065;22039200.0,296.57437;22042800.0,294.21356;22046400.0,292.50732;22050000.0,290.888;22053600.0,289.4946;22057200.0,289.56824;22060800.0,291.02124;22064400.0,292.53787;22068000.0,296.28793;22071600.0,303.0611;22075200.0,310.02618;22078800.0,316.07715;22082400.0,321.05734;22086000.0,323.28946;22089600.0,320.67502;22093200.0,316.54752;22096800.0,312.54608;22100400.0,308.9364;22104000.0,305.391;22107600.0,302.3322;22111200.0,299.7191;22114800.0,297.38617;22118400.0,295.33707;22122000.0,293.5272;22125600.0,292.06448;22129200.0,290.7447;22132800.0,289.57043;22136400.0,288.47375;22140000.0,287.61646;22143600.0,287.99405;22147200.0,290.6177;22150800.0,295.6378;22154400.0,301.7958;22158000.0,308.40408;22161600.0,314.78442;22165200.0,319.4668;22168800.0,316.72012;22172400.0,312.81436;22176000.0,309.5618;22179600.0,307.4373;22183200.0,304.94373;22186800.0,301.90387;22190400.0,298.9107;22194000.0,296.6392;22197600.0,294.6797;22201200.0,293.0599;22204800.0,291.68292;22208400.0,290.46957;22212000.0,289.234;22215600.0,288.08463;22219200.0,287.2176;22222800.0,286.5183;22226400.0,285.7429;22230000.0,286.5222;22233600.0,289.29974;22237200.0,294.33743;22240800.0,300.86398;22244400.0,308.1535;22248000.0,315.4707;22251600.0,321.36774;22255200.0,325.71237;22258800.0,327.94043;22262400.0,327.46918;22266000.0,324.70685;22269600.0,320.58765;22273200.0,315.89462;22276800.0,311.6112;22280400.0,308.30823;22284000.0,305.35535;22287600.0,302.5312;22291200.0,299.9321;22294800.0,297.99582;22298400.0,295.87134;22302000.0,294.4173;22305600.0,292.99277;22309200.0,291.74188;22312800.0,290.79517;22316400.0,291.25336;22320000.0,293.70752;22323600.0,296.34262;22327200.0,300.0078;22330800.0,304.21655;22334400.0,305.60596;22338000.0,307.15207;22341600.0,310.58456;22345200.0,315.25375;22348800.0,317.20834;22352400.0,316.37744;22356000.0,313.6794;22359600.0,311.08347;22363200.0,308.47498;22366800.0,306.19418;22370400.0,303.8978;22374000.0,301.47195;22377600.0,299.48312;22381200.0,297.628;22384800.0,295.8805;22388400.0,294.27997;22392000.0,293.01138;22395600.0,291.5499;22399200.0,289.978;22402800.0,290.71048;22406400.0,294.23245;22410000.0,299.89572;22413600.0,307.18494;22417200.0,315.06866;22420800.0,322.55075;22424400.0,328.53433;22428000.0,332.7927;22431600.0,334.8349;22435200.0,334.01984;22438800.0,330.83673;22442400.0,326.05432;22446000.0,320.6553;22449600.0,315.83957;22453200.0,312.22845;22456800.0,308.84265;22460400.0,305.65863;22464000.0,302.5099;22467600.0,299.8562;22471200.0,297.65503;22474800.0,295.6566;22478400.0,294.12115;22482000.0,292.5624;22485600.0,291.24026;22489200.0,291.6166;22492800.0,294.09653;22496400.0,298.61987;22500000.0,304.97925;22503600.0,312.3912;22507200.0,318.22177;22510800.0,323.06726;22514400.0,325.16214;22518000.0,327.68832;22521600.0,327.60214;22525200.0,324.9289;22528800.0,320.79242;22532400.0,316.282;22536000.0,312.3172;22539600.0,309.09818;22543200.0,306.24918;22546800.0,303.49756;22550400.0,301.09323;22554000.0,299.0001;22557600.0,297.07874;22561200.0,295.33743;22564800.0,293.86908;22568400.0,292.37283;22572000.0,291.13235;22575600.0,291.04376;22579200.0,291.29062;22582800.0,293.45584;22586400.0,300.2674;22590000.0,308.13037;22593600.0,315.75125;22597200.0,322.1421;22600800.0,327.17816;22604400.0,330.41394;22608000.0,330.5487;22611600.0,327.74152;22615200.0,323.4697;22618800.0,318.3702;22622400.0,313.68796;22626000.0,309.90253;22629600.0,306.89142;22633200.0,304.2344;22636800.0,302.16492;22640400.0,300.13446;22644000.0,298.06277;22647600.0,296.21545;22651200.0,294.7187;22654800.0,293.39767;22658400.0,292.2322;22662000.0,292.87045;22665600.0,295.81076;22669200.0,301.0179;22672800.0,307.9253;22676400.0,315.27008;22680000.0,322.4169;22683600.0,327.78314;22687200.0,332.3413;22690800.0,334.4213;22694400.0,333.41376;22698000.0,329.3902;22701600.0,324.66266;22705200.0,319.76093;22708800.0,315.49094;22712400.0,311.81793;22716000.0,308.40952;22719600.0,305.78375;22723200.0,303.68454;22726800.0,301.88837;22730400.0,300.44037;22734000.0,299.25732;22737600.0,298.28696;22741200.0,297.39783;22744800.0,296.85364;22748400.0,297.0026;22752000.0,298.31668;22755600.0,303.51044;22759200.0,310.01978;22762800.0,312.8457;22766400.0,313.04056;22770000.0,312.93585;22773600.0,311.59067;22777200.0,310.15292;22780800.0,310.57724;22784400.0,310.259;22788000.0,308.63037;22791600.0,306.44485;22795200.0,304.56155;22798800.0,302.86807;22802400.0,301.1335;22806000.0,299.4327;22809600.0,297.7505;22813200.0,296.3708;22816800.0,295.21487;22820400.0,294.33273;22824000.0,293.54913;22827600.0,292.62555;22831200.0,291.58765;22834800.0,291.2719;22838400.0,292.45065;22842000.0,296.9132;22845600.0,302.13785;22849200.0,307.13477;22852800.0,314.3476;22856400.0,319.6771;22860000.0,319.8688;22863600.0,317.6975;22867200.0,314.84415;22870800.0,311.5772;22874400.0,308.44107;22878000.0,305.567;22881600.0,302.99704;22885200.0,300.52197;22888800.0,298.26666;22892400.0,296.12204;22896000.0,294.4439;22899600.0,292.85455;22903200.0,291.67422;22906800.0,290.67746;22910400.0,289.58362;22914000.0,288.69196;22917600.0,287.88593;22921200.0,288.63406;22924800.0,291.85022;22928400.0,297.35446;22932000.0,304.18948;22935600.0,312.1254;22939200.0,319.46988;22942800.0,325.45743;22946400.0,325.55032;22950000.0,325.07648;22953600.0,323.37686;22957200.0,320.98526;22960800.0,317.95117;22964400.0,314.22827;22968000.0,311.11148;22971600.0,308.2291;22975200.0,305.5547;22978800.0,303.0768;22982400.0,300.98312;22986000.0,299.01175;22989600.0,296.93866;22993200.0,295.28317;22996800.0,294.15765;23000400.0,293.2115;23004000.0,292.04004;23007600.0,291.20288;23011200.0,290.67072;23014800.0,291.06018;23018400.0,295.8568;23022000.0,298.90747;23025600.0,301.2299;23029200.0,305.0667;23032800.0,311.37814;23036400.0,315.31665;23040000.0,315.40924;23043600.0,314.0548;23047200.0,310.85516;23050800.0,307.2375;23054400.0,303.97864;23058000.0,301.13257;23061600.0,298.84402;23065200.0,296.94513;23068800.0,295.40982;23072400.0,293.98465;23076000.0,292.70908;23079600.0,291.53214;23083200.0,290.37827;23086800.0,289.38483;23090400.0,288.54813;23094000.0,287.94055;23097600.0,287.73593;23101200.0,288.18207;23104800.0,291.50766;23108400.0,297.9601;23112000.0,306.40726;23115600.0,313.97418;23119200.0,319.61914;23122800.0,322.6071;23126400.0,323.08862;23130000.0,320.83618;23133600.0,316.70187;23137200.0,312.22058;23140800.0,308.4863;23144400.0,304.88733;23148000.0,301.68765;23151600.0,298.9215;23155200.0,296.39764;23158800.0,294.04898;23162400.0,292.2566;23166000.0,290.76984;23169600.0,289.1747;23173200.0,287.7359;23176800.0,286.5369;23180400.0,286.9605;23184000.0,290.21918;23187600.0,295.65765;23191200.0,302.23108;23194800.0,309.3927;23198400.0,317.01904;23202000.0,323.6713;23205600.0,328.57864;23209200.0,331.28094;23212800.0,330.77774;23216400.0,327.45032;23220000.0,322.60342;23223600.0,317.82773;23227200.0,313.53073;23230800.0,309.7349;23234400.0,306.60312;23238000.0,304.09973;23241600.0,302.03253;23245200.0,300.26105;23248800.0,298.6532;23252400.0,297.14075;23256000.0,296.01672;23259600.0,294.7604;23263200.0,293.95853;23266800.0,294.4674;23270400.0,296.07013;23274000.0,301.30222;23277600.0,305.06097;23281200.0,311.02682;23284800.0,319.8302;23288400.0,326.80994;23292000.0,328.9618;23295600.0,325.72113;23299200.0,324.89075;23302800.0,321.8069;23306400.0,317.14783;23310000.0,312.80493;23313600.0,309.1235;23317200.0,306.18793;23320800.0,303.61633;23324400.0,301.47784;23328000.0,299.6856;23331600.0,298.1849;23335200.0,296.85034;23338800.0,295.4994;23342400.0,293.74442;23346000.0,292.21176;23349600.0,290.83698;23353200.0,290.4874;23356800.0,289.77988;23360400.0,289.17532;23364000.0,289.00818;23367600.0,290.42758;23371200.0,293.42233;23374800.0,298.1828;23378400.0,299.008;23382000.0,299.63022;23385600.0,299.0058;23389200.0,297.6762;23392800.0,295.7582;23396400.0,293.52142;23400000.0,291.5462;23403600.0,289.93393;23407200.0,288.30426;23410800.0,286.72476;23414400.0,285.36517;23418000.0,284.65234;23421600.0,283.97845;23425200.0,283.3105;23428800.0,282.90927;23432400.0,282.54703;23436000.0,282.18243;23439600.0,281.99503;23443200.0,282.93848;23446800.0,287.4005;23450400.0,289.34552;23454000.0,294.11588;23457600.0,301.28363;23461200.0,304.7054;23464800.0,310.7;23468400.0,312.53925;23472000.0,312.18384;23475600.0,310.19357;23479200.0,307.46295;23482800.0,304.62814;23486400.0,301.99402;23490000.0,299.87906;23493600.0,297.9344;23497200.0,296.12613;23500800.0,294.53818;23504400.0,292.99557;23508000.0,291.65927;23511600.0,290.58377;23515200.0,289.85782;23518800.0,289.25647;23522400.0,288.72452;23526000.0,289.57233;23529600.0,293.10876;23533200.0,299.29333;23536800.0,305.9222;23540400.0,314.08368;23544000.0,322.12317;23547600.0,328.53802;23551200.0,332.9596;23554800.0,335.0898;23558400.0,334.42953;23562000.0,331.03738;23565600.0,325.97217;23569200.0,321.12033;23572800.0,316.9474;23576400.0,313.2689;23580000.0,310.005;23583600.0,307.17435;23587200.0,304.72855;23590800.0,302.60043;23594400.0,300.7441;23598000.0,299.0969;23601600.0,297.6094;23605200.0,296.19016;23608800.0,294.98544;23612400.0,294.75516;23616000.0,298.2678;23619600.0,304.61514;23623200.0,310.7977;23626800.0,318.1544;23630400.0,325.4433;23634000.0,331.49963;23637600.0,335.44565;23641200.0,336.72263;23644800.0,335.49042;23648400.0,331.4021;23652000.0,325.83624;23655600.0,320.7943;23659200.0,316.77844;23662800.0,313.3282;23666400.0,310.2593;23670000.0,307.04718;23673600.0,304.14615;23677200.0,301.05267;23680800.0,298.30164;23684400.0,295.7167;23688000.0,293.63248;23691600.0,291.9389;23695200.0,289.91217;23698800.0,289.1261;23702400.0,292.51596;23706000.0,298.63284;23709600.0,306.535;23713200.0,314.65042;23716800.0,322.13382;23720400.0,328.48233;23724000.0,332.64737;23727600.0,334.82172;23731200.0,334.531;23734800.0,331.07913;23738400.0,326.00037;23742000.0,320.58646;23745600.0,315.78833;23749200.0,311.69962;23752800.0,308.38745;23756400.0,305.44296;23760000.0,302.6014;23763600.0,300.01746;23767200.0,297.7052;23770800.0,295.97192;23774400.0,294.48544;23778000.0,293.09802;23781600.0,292.07413;23785200.0,291.34253;23788800.0,292.40524;23792400.0,298.48163;23796000.0,307.01074;23799600.0,315.22083;23803200.0,322.50082;23806800.0,325.84137;23810400.0,323.27942;23814000.0,322.29437;23817600.0,321.19257;23821200.0,318.11023;23824800.0,313.43625;23828400.0,308.9956;23832000.0,305.09;23835600.0,301.86105;23839200.0,299.28183;23842800.0,297.19568;23846400.0,295.49405;23850000.0,294.0872;23853600.0,292.7352;23857200.0,291.575;23860800.0,290.34793;23864400.0,289.07623;23868000.0,287.93195;23871600.0,287.17404;23875200.0,287.0981;23878800.0,287.80652;23882400.0,289.4442;23886000.0,295.8039;23889600.0,302.62112;23893200.0,304.1694;23896800.0,305.34946;23900400.0,306.41785;23904000.0,305.50974;23907600.0,303.26282;23911200.0,301.13348;23914800.0,299.34018;23918400.0,298.07852;23922000.0,296.64264;23925600.0,295.31644;23929200.0,293.9154;23932800.0,292.446;23936400.0,291.10568;23940000.0,289.86273;23943600.0,288.77902;23947200.0,287.6808;23950800.0,286.76218;23954400.0,285.92493;23958000.0,285.22998;23961600.0,284.71884;23965200.0,284.52747;23968800.0,284.51718;23972400.0,284.75394;23976000.0,284.92075;23979600.0,285.23923;23983200.0,285.86676;23986800.0,286.02725;23990400.0,285.58044;23994000.0,284.5077;23997600.0,283.07724;24001200.0,281.76126;24004800.0,280.62198;24008400.0,279.65842;24012000.0,278.83414;24015600.0,278.18158;24019200.0,277.5104;24022800.0,276.8848;24026400.0,276.32416;24030000.0,275.84164;24033600.0,275.4235;24037200.0,275.08435;24040800.0,274.7429;24044400.0,274.51648;24048000.0,275.07672;24051600.0,276.41156;24055200.0,278.00555;24058800.0,279.4943;24062400.0,280.55225;24066000.0,281.2184;24069600.0,281.5918;24073200.0,281.93594;24076800.0,281.83936;24080400.0,281.26804;24084000.0,280.50613;24087600.0,279.71985;24091200.0,279.0495;24094800.0,278.46844;24098400.0,278.04147;24102000.0,277.70868;24105600.0,277.41226;24109200.0,277.16016;24112800.0,276.92795;24116400.0,276.7782;24120000.0,276.62796;24123600.0,276.50656;24127200.0,276.3361;24130800.0,276.229;24134400.0,276.80646;24138000.0,278.0859;24141600.0,279.6882;24145200.0,281.0732;24148800.0,282.28357;24152400.0,283.4392;24156000.0,284.28488;24159600.0,284.83255;24163200.0,284.9778;24166800.0,284.4695;24170400.0,283.69806;24174000.0,282.9092;24177600.0,282.23932;24181200.0,281.44473;24184800.0,280.5852;24188400.0,279.89975;24192000.0,279.4588;24195600.0,279.13705;24199200.0,278.82745;24202800.0,278.55774;24206400.0,278.40393;24210000.0,278.20007;24213600.0,277.868;24217200.0,277.6911;24220800.0,279.9074;24224400.0,286.18152;24228000.0,293.83917;24231600.0,302.79932;24235200.0,311.0119;24238800.0,317.43057;24242400.0,321.60962;24246000.0,323.0423;24249600.0,321.69122;24253200.0,317.6552;24256800.0,311.6912;24260400.0,306.24176;24264000.0,301.47287;24267600.0,297.4264;24271200.0,294.26285;24274800.0,291.57416;24278400.0,289.16907;24282000.0,287.1273;24285600.0,285.45587;24289200.0,283.98077;24292800.0,282.42908;24296400.0,281.04483;24300000.0,279.8708;24303600.0,279.2768;24307200.0,282.8667;24310800.0,289.863;24314400.0,298.44983;24318000.0,306.6704;24321600.0,314.60208;24325200.0,321.24503;24328800.0,326.28528;24332400.0,328.69608;24336000.0,327.92316;24339600.0,324.38245;24343200.0,318.56592;24346800.0,313.2056;24350400.0,308.4661;24354000.0,304.08304;24357600.0,300.39536;24361200.0,297.27994;24364800.0,294.58273;24368400.0,292.26294;24372000.0,289.8471;24375600.0,287.93616;24379200.0,286.02734;24382800.0,284.90506;24386400.0,283.74167;24390000.0,283.20053;24393600.0,287.22372;24397200.0,294.2384;24400800.0,302.76862;24404400.0,312.01553;24408000.0,320.53226;24411600.0,327.731;24415200.0,332.5009;24418800.0,334.31888;24422400.0,333.00665;24426000.0,328.55066;24429600.0,322.42188;24433200.0,316.85425;24436800.0,312.1758;24440400.0,308.18652;24444000.0,304.42722;24447600.0,301.48752;24451200.0,299.2772;24454800.0,297.21777;24458400.0,295.55035;24462000.0,294.28415;24465600.0,292.54037;24469200.0,290.37064;24472800.0,288.67374;24476400.0,287.65738;24480000.0,290.78772;24483600.0,297.43726;24487200.0,305.92914;24490800.0,314.37482;24494400.0,322.04807;24498000.0,328.512;24501600.0,333.01242;24505200.0,335.1045;24508800.0,334.01883;24512400.0,330.03073;24516000.0,324.39026;24519600.0,319.49936;24523200.0,315.3964;24526800.0,311.8489;24530400.0,308.83246;24534000.0,305.86932;24537600.0,302.81168;24541200.0,299.91708;24544800.0,297.23117;24548400.0,295.1785;24552000.0,293.3184;24555600.0,291.80594;24559200.0,290.34293;24562800.0,289.57904;24566400.0,293.03394;24570000.0,298.58554;24573600.0,306.61343;24577200.0,315.081;24580800.0,323.1598;24584400.0,329.4734;24588000.0,333.79874;24591600.0,335.55835;24595200.0,334.05225;24598800.0,329.3525;24602400.0,322.91275;24606000.0,317.32666;24609600.0,312.57736;24613200.0,308.2856;24616800.0,304.5617;24620400.0,301.37326;24624000.0,298.05423;24627600.0,294.78214;24631200.0,291.99365;24634800.0,289.8931;24638400.0,288.052;24642000.0,286.60886;24645600.0,285.07513;24649200.0,283.99734;24652800.0,284.6807;24656400.0,288.8784;24660000.0,296.46747;24663600.0,305.3231;24667200.0,312.16974;24670800.0,316.2929;24674400.0,321.7368;24678000.0,324.9945;24681600.0,324.85486;24685200.0,321.23694;24688800.0,315.51025;24692400.0,310.19797;24696000.0,305.68076;24699600.0,301.76154;24703200.0,298.2414;24706800.0,295.38406;24710400.0,292.90613;24714000.0,290.9518;24717600.0,289.31412;24721200.0,287.6226;24724800.0,286.28802;24728400.0,284.80542;24732000.0,283.0617;24735600.0,282.05408;24739200.0,286.05795;24742800.0,292.2585;24746400.0,299.3152;24750000.0,307.2328;24753600.0,313.84985;24757200.0,319.28394;24760800.0,320.0087;24764400.0,317.41898;24768000.0,313.13876;24771600.0,309.16586;24775200.0,305.39206;24778800.0,301.84204;24782400.0,298.714;24786000.0,296.30652;24789600.0,294.21457;24793200.0,292.1962;24796800.0,290.42404;24800400.0,288.56122;24804000.0,286.78223;24807600.0,284.71323;24811200.0,282.94812;24814800.0,281.40384;24818400.0,280.29938;24822000.0,279.93732;24825600.0,283.68246;24829200.0,290.56082;24832800.0,298.42075;24836400.0,307.64813;24840000.0,316.09375;24843600.0,323.04776;24847200.0,328.0933;24850800.0,330.2596;24854400.0,328.81775;24858000.0,324.53406;24861600.0,318.35812;24865200.0,312.9702;24868800.0,308.3335;24872400.0,304.3745;24876000.0,300.83707;24879600.0,297.43268;24883200.0,294.66245;24886800.0,292.3908;24890400.0,290.1607;24894000.0,288.50067;24897600.0,286.96512;24901200.0,285.73328;24904800.0,284.41602;24908400.0,283.76514;24912000.0,287.07156;24915600.0,293.78067;24919200.0,302.60165;24922800.0,312.01318;24926400.0,319.85727;24930000.0,325.9675;24933600.0,329.778;24937200.0,331.3065;24940800.0,329.04523;24944400.0,324.61288;24948000.0,318.39673;24951600.0,312.5628;24955200.0,307.48553;24958800.0,303.1596;24962400.0,299.29733;24966000.0,296.0536;24969600.0,293.47333;24973200.0,291.10367;24976800.0,289.055;24980400.0,287.25272;24984000.0,285.63458;24987600.0,284.4005;24991200.0,283.32568;24994800.0,282.90814;24998400.0,287.20575;25002000.0,294.29996;25005600.0,302.7124;25009200.0,311.2903;25012800.0,319.5126;25016400.0,326.17184;25020000.0,330.59174;25023600.0,332.88226;25027200.0,331.9094;25030800.0,327.4344;25034400.0,321.12363;25038000.0,315.28073;25041600.0,310.12683;25045200.0,306.0085;25048800.0,301.93256;25052400.0,298.77414;25056000.0,296.16284;25059600.0,293.91455;25063200.0,292.01596;25066800.0,290.5033;25070400.0,289.16122;25074000.0,288.0114;25077600.0,286.91568;25081200.0,286.40082;25084800.0,290.02377;25088400.0,297.08447;25092000.0,304.81665;25095600.0,313.58313;25099200.0,322.13098;25102800.0,329.2843;25106400.0,334.57428;25110000.0,336.93127;25113600.0,336.13727;25117200.0,331.5349;25120800.0,324.8744;25124400.0,319.09732;25128000.0,313.76144;25131600.0,308.9142;25135200.0,305.2213;25138800.0,302.01453;25142400.0,299.33902;25146000.0,296.77307;25149600.0,294.57797;25153200.0,292.71838;25156800.0,291.36307;25160400.0,290.07065;25164000.0,288.85687;25167600.0,288.44382;25171200.0,290.10016;25174800.0,296.57413;25178400.0,304.6315;25182000.0,314.33084;25185600.0,322.74805;25189200.0,328.6165;25192800.0,332.65164;25196400.0,332.7182;25200000.0,327.94086;25203600.0,322.9345;25207200.0,317.71127;25210800.0,313.14362;25214400.0,309.1764;25218000.0,305.6317;25221600.0,302.7148;25225200.0,300.30435;25228800.0,297.84445;25232400.0,295.41772;25236000.0,293.34317;25239600.0,291.4309;25243200.0,289.68008;25246800.0,288.314;25250400.0,286.5995;25254000.0,286.16397;25257600.0,290.0671;25261200.0,296.73242;25264800.0,305.09363;25268400.0,313.557;25272000.0,320.95847;25275600.0,327.61417;25279200.0,332.0282;25282800.0,333.74408;25286400.0,332.08374;25290000.0,327.4372;25293600.0,321.0545;25297200.0,315.00494;25300800.0,309.64993;25304400.0,305.61328;25308000.0,301.96213;25311600.0,299.23224;25315200.0,297.07208;25318800.0,295.13574;25322400.0,293.58255;25326000.0,291.73093;25329600.0,289.56445;25333200.0,287.7364;25336800.0,286.31146;25340400.0,285.8805;25344000.0,288.99927;25347600.0,295.2771;25351200.0,303.60663;25354800.0,312.3611;25358400.0,321.28296;25362000.0,328.13696;25365600.0,332.7269;25369200.0,334.83255;25372800.0,332.45538;25376400.0,326.34305;25380000.0,320.50977;25383600.0,315.2525;25387200.0,310.85217;25390800.0,307.27258;25394400.0,304.10638;25398000.0,301.2559;25401600.0,298.67087;25405200.0,296.35263;25408800.0,294.2023;25412400.0,292.29422;25416000.0,290.4815;25419600.0,288.99695;25423200.0,287.54056;25426800.0,286.61145;25430400.0,286.64432;25434000.0,286.2544;25437600.0,285.89908;25441200.0,285.7683;25444800.0,285.65543;25448400.0,285.5445;25452000.0,285.3528;25455600.0,285.03873;25459200.0,284.4841;25462800.0,283.81317;25466400.0,283.1193;25470000.0,282.45813;25473600.0,281.86566;25477200.0,281.38126;25480800.0,280.9541;25484400.0,280.5919;25488000.0,280.27774;25491600.0,280.0541;25495200.0,279.8424;25498800.0,279.6837;25502400.0,279.5587;25506000.0,279.39984;25509600.0,279.28806;25513200.0,279.33206;25516800.0,279.75812;25520400.0,280.7509;25524000.0,283.7648;25527600.0,287.06305;25531200.0,290.08063;25534800.0,294.10776;25538400.0,299.37445;25542000.0,300.1176;25545600.0,298.859;25549200.0,296.7892;25552800.0,294.57086;25556400.0,292.6634;25560000.0,291.09854;25563600.0,289.89993;25567200.0,288.89;25570800.0,288.05893;25574400.0,287.27194;25578000.0,286.66418;25581600.0,286.17056;25585200.0,285.75497;25588800.0,285.3799;25592400.0,284.92065;25596000.0,284.1683;25599600.0,283.6415;25603200.0,283.70956;25606800.0,286.3948;25610400.0,287.8629;25614000.0,296.7938;25617600.0,307.18478;25621200.0,315.6877;25624800.0,321.89807;25628400.0,324.7436;25632000.0,323.1918;25635600.0,319.08203;25639200.0,313.84818;25642800.0,309.31442;25646400.0,305.38947;25650000.0,301.9336;25653600.0,298.92523;25657200.0,295.85498;25660800.0,293.17017;25664400.0,290.90213;25668000.0,288.8467;25671600.0,287.4756;25675200.0,286.21893;25678800.0,285.0947;25682400.0,284.25336;25686000.0,284.2211;25689600.0,287.5632;25693200.0,294.39584;25696800.0,303.531;25700400.0,312.53525;25704000.0,320.2442;25707600.0,326.8977;25711200.0,331.33386;25714800.0,332.11902;25718400.0,330.57126;25722000.0,324.3519;25725600.0,318.29462;25729200.0,312.87216;25732800.0,308.34268;25736400.0,304.55545;25740000.0,301.0193;25743600.0,297.71283;25747200.0,294.89487;25750800.0,292.08154;25754400.0,289.97952;25758000.0,288.42914;25761600.0,286.89655;25765200.0,285.5878;25768800.0,284.3916;25772400.0,284.11432;25776000.0,288.2435;25779600.0,295.33322;25783200.0,303.74094;25786800.0,312.71802;25790400.0,320.62534;25794000.0,326.82617;25797600.0,330.86386;25801200.0,332.93463;25804800.0,331.58615;25808400.0,326.076;25812000.0,319.34256;25815600.0,313.6604;25819200.0,308.68356;25822800.0,304.16757;25826400.0,300.62152;25830000.0,297.58493;25833600.0,294.77136;25837200.0,292.2293;25840800.0,290.29697;25844400.0,288.4574;25848000.0,286.6957;25851600.0,285.59613;25855200.0,284.5445;25858800.0,283.9968;25862400.0,286.70648;25866000.0,292.519;25869600.0,299.00632;25873200.0,307.15573;25876800.0,313.7356;25880400.0,320.9827;25884000.0,325.3195;25887600.0,322.2654;25891200.0,317.5392;25894800.0,313.63522;25898400.0,309.14734;25902000.0,305.2884;25905600.0,301.77142;25909200.0,298.87167;25912800.0,296.06937;25916400.0,293.6773;25920000.0,291.6782;25923600.0,290.21222;25927200.0,289.21365;25930800.0,288.37433;25934400.0,287.55573;25938000.0,286.79324;25941600.0,286.11227;25945200.0,285.756;25948800.0,285.91617;25952400.0,286.20245;25956000.0,287.77496;25959600.0,289.85593;25963200.0,290.85025;25966800.0,293.17264;25970400.0,293.75964;25974000.0,295.6792;25977600.0,298.0307;25981200.0,297.8362;25984800.0,295.83145;25988400.0,293.9285;25992000.0,292.39517;25995600.0,291.00836;25999200.0,289.726;26002800.0,288.5189;26006400.0,287.6055;26010000.0,286.91635;26013600.0,286.35513;26017200.0,285.86807;26020800.0,285.44528;26024400.0,285.09833;26028000.0,284.7748;26031600.0,284.37827;26035200.0,284.9255;26038800.0,285.76187;26042400.0,288.13474;26046000.0,296.9828;26049600.0,307.01276;26053200.0,315.4941;26056800.0,320.97604;26060400.0,324.17844;26064000.0,324.34573;26067600.0,320.345;26071200.0,314.97623;26074800.0,309.7336;26078400.0,305.9261;26082000.0,301.79196;26085600.0,298.362;26089200.0,295.5611;26092800.0,293.12277;26096400.0,290.9167;26100000.0,289.1766;26103600.0,287.57706;26107200.0,286.17715;26110800.0,284.66855;26114400.0,283.54358;26118000.0,282.77884;26121600.0,283.92426;26125200.0,290.7233;26128800.0,300.013;26132400.0,308.55286;26136000.0,316.63837;26139600.0,319.9096;26143200.0,324.91647;26146800.0,327.24118;26150400.0,325.62283;26154000.0,320.87503;26157600.0,315.1935;26161200.0,310.23315;26164800.0,306.10254;26168400.0,302.4118;26172000.0,299.01172;26175600.0,296.13892;26179200.0,293.69577;26182800.0,291.40503;26186400.0,289.36914;26190000.0,287.44666;26193600.0,285.8203;26197200.0,284.3511;26200800.0,283.14963;26204400.0,282.9238;26208000.0,285.48105;26211600.0,287.1322;26215200.0,287.66275;26218800.0,287.36163;26222400.0,287.07556;26226000.0,286.9436;26229600.0,286.8356;26233200.0,286.53275;26236800.0,286.1869;26240400.0,285.48798;26244000.0,284.7249;26247600.0,284.0227;26251200.0,283.25085;26254800.0,282.40533;26258400.0,281.43222;26262000.0,280.41843;26265600.0,279.28113;26269200.0,278.27103;26272800.0,276.91656;26276400.0,275.62622;26280000.0,274.24222;26283600.0,273.09845;26287200.0,272.0416;26290800.0,271.3702;26294400.0,272.06738;26298000.0,271.98047;26301600.0,272.08447;26305200.0,273.02765;26308800.0,273.81287;26312400.0,274.0179;26316000.0,274.09592;26319600.0,273.8417;26323200.0,273.44748;26326800.0,272.76614;26330400.0,271.93484;26334000.0,271.23508;26337600.0,270.6696;26341200.0,270.1855;26344800.0,269.73816;26348400.0,269.39493;26352000.0,269.08514;26355600.0,268.92966;26359200.0,268.78568;26362800.0,268.63065;26366400.0,268.54004;26370000.0,268.42392;26373600.0,268.27316;26377200.0,268.2684;26380800.0,268.46118;26384400.0,269.4561;26388000.0,270.74945;26391600.0,271.61935;26395200.0,272.29562;26398800.0,273.17038;26402400.0,274.92133;26406000.0,277.22412;26409600.0,278.89334;26413200.0,278.788;26416800.0,277.42038;26420400.0,276.48193;26424000.0,275.66733;26427600.0,275.00485;26431200.0,274.43732;26434800.0,273.76373;26438400.0,272.95874;26442000.0,272.52725;26445600.0,272.16818;26449200.0,271.73203;26452800.0,271.43927;26456400.0,271.2482;26460000.0,270.7175;26463600.0,270.049;26467200.0,269.92285;26470800.0,270.11453;26474400.0,270.81332;26478000.0,272.0196;26481600.0,273.60025;26485200.0,277.33188;26488800.0,280.42313;26492400.0,282.65488;26496000.0,285.0662;26499600.0,284.41965;26503200.0,282.12784;26506800.0,280.4078;26510400.0,278.69073;26514000.0,277.06912;26517600.0,275.5222;26521200.0,274.25354;26524800.0,273.1181;26528400.0,272.40604;26532000.0,271.85696;26535600.0,271.50662;26539200.0,271.4104;26542800.0,271.23404;26546400.0,270.65262;26550000.0,270.31348;26553600.0,270.7303;26557200.0,271.63828;26560800.0,272.41464;26564400.0,273.59323;26568000.0,275.0287;26571600.0,280.19894;26575200.0,287.3206;26578800.0,291.96115;26582400.0,292.54233;26586000.0,290.03668;26589600.0,287.7608;26593200.0,285.73822;26596800.0,283.7334;26600400.0,282.00516;26604000.0,280.54373;26607600.0,279.43735;26611200.0,278.31467;26614800.0,277.40237;26618400.0,276.6701;26622000.0,276.02267;26625600.0,275.51157;26629200.0,274.62778;26632800.0,273.84244;26636400.0,273.6877;26640000.0,276.56686;26643600.0,279.0471;26647200.0,282.06635;26650800.0,287.37558;26654400.0,295.06055;26658000.0,303.99002;26661600.0,311.415;26665200.0,316.0166;26668800.0,316.92975;26672400.0,313.1194;26676000.0,307.5658;26679600.0,302.49982;26683200.0,298.39392;26686800.0,295.0683;26690400.0,292.35266;26694000.0,290.02557;26697600.0,288.18164;26701200.0,286.59082;26704800.0,285.10883;26708400.0,283.92807;26712000.0,282.93704;26715600.0,282.21335;26719200.0,281.48456;26722800.0,280.8885;26726400.0,284.0299;26730000.0,291.45605;26733600.0,299.86304;26737200.0,309.23196;26740800.0,317.84366;26744400.0,324.91208;26748000.0,329.8274;26751600.0,331.77536;26755200.0,330.21027;26758800.0,324.43265;26762400.0,317.12115;26766000.0,310.94897;26769600.0,305.7068;26773200.0,301.53714;26776800.0,298.1611;26780400.0,295.1087;26784000.0,292.68253;26787600.0,290.60745;26791200.0,288.81134;26794800.0,287.37543;26798400.0,286.0825;26802000.0,285.05374;26805600.0,284.17874;26809200.0,283.7008;26812800.0,286.07513;26816400.0,291.757;26820000.0,297.06546;26823600.0,300.92148;26827200.0,302.1111;26830800.0,301.96225;26834400.0,303.92728;26838000.0,302.91803;26841600.0,301.00787;26845200.0,298.58365;26848800.0,296.5166;26852400.0,294.72894;26856000.0,293.14865;26859600.0,291.60562;26863200.0,290.65366;26866800.0,289.43396;26870400.0,288.07938;26874000.0,286.98587;26877600.0,286.00296;26881200.0,285.00348;26884800.0,283.8894;26888400.0,283.28333;26892000.0,282.69672;26895600.0,281.89966;26899200.0,284.52072;26902800.0,285.1441;26906400.0,285.97418;26910000.0,290.80005;26913600.0,297.6534;26917200.0,298.4354;26920800.0,301.92358;26924400.0,307.5533;26928000.0,309.70782;26931600.0,307.13724;26935200.0,303.62277;26938800.0,300.79352;26942400.0,297.82837;26946000.0,295.04538;26949600.0,293.0821;26953200.0,291.66428;26956800.0,290.11124;26960400.0,288.7882;26964000.0,287.67615;26967600.0,286.89713;26971200.0,286.24683;26974800.0,285.32822;26978400.0,284.38065;26982000.0,283.87085;26985600.0,286.51285;26989200.0,292.34674;26992800.0,296.11957;26996400.0,299.5305;27000000.0,302.38492;27003600.0,304.2219;27007200.0,304.954;27010800.0,305.17868;27014400.0,303.67874;27018000.0,300.49725;27021600.0,297.36136;27025200.0,294.53885;27028800.0,292.02332;27032400.0,289.86615;27036000.0,287.93808;27039600.0,286.4438;27043200.0,285.0673;27046800.0,283.63754;27050400.0,281.98996;27054000.0,281.03244;27057600.0,280.1688;27061200.0,279.24243;27064800.0,278.4567;27068400.0,277.948;27072000.0,279.31738;27075600.0,282.22772;27079200.0,285.96173;27082800.0,289.90817;27086400.0,293.6861;27090000.0,296.91235;27093600.0,299.08243;27097200.0,299.82373;27100800.0,298.95294;27104400.0,296.38644;27108000.0,293.7047;27111600.0,291.32663;27115200.0,289.17337;27118800.0,287.08722;27122400.0,285.38934;27126000.0,283.7521;27129600.0,282.2239;27133200.0,281.11227;27136800.0,280.06046;27140400.0,279.09274;27144000.0,278.11316;27147600.0,277.1331;27151200.0,276.3006;27154800.0,275.89865;27158400.0,278.07385;27162000.0,283.52487;27165600.0,290.622;27169200.0,299.50247;27172800.0,306.64175;27176400.0,310.07596;27180000.0,309.91602;27183600.0,308.1616;27187200.0,304.286;27190800.0,300.55368;27194400.0,296.8637;27198000.0,293.41064;27201600.0,290.3694;27205200.0,287.65137;27208800.0,285.40723;27212400.0,283.48828;27216000.0,281.38446;27219600.0,279.82114;27223200.0,278.5211;27226800.0,277.35852;27230400.0,276.2415;27234000.0,275.3306;27237600.0,274.3181;27241200.0,273.7492;27244800.0,276.4468;27248400.0,283.12332;27252000.0,290.58957;27255600.0,299.3388;27259200.0,307.96082;27262800.0,315.39078;27266400.0,320.9249;27270000.0,323.67117;27273600.0,323.305;27277200.0,318.0403;27280800.0,311.75342;27284400.0,306.41403;27288000.0,302.23642;27291600.0,298.28207;27295200.0,294.99716;27298800.0,292.44855;27302400.0,290.4732;27306000.0,288.96295;27309600.0,287.7992;27313200.0,286.79202;27316800.0,286.06567;27320400.0,285.53665;27324000.0,285.0934;27327600.0,284.36926;27331200.0,283.52838;27334800.0,283.32574;27338400.0,286.58212;27342000.0,288.56226;27345600.0,290.1329;27349200.0,290.88394;27352800.0,294.0396;27356400.0,293.42038;27360000.0,292.11176;27363600.0,290.8155;27367200.0,289.8255;27370800.0,288.66443;27374400.0,287.58084;27378000.0,286.66058;27381600.0,285.47467;27385200.0,284.35254;27388800.0,283.27393;27392400.0,282.37253;27396000.0,281.58447;27399600.0,280.95502;27403200.0,280.11505;27406800.0,279.395;27410400.0,278.74557;27414000.0,278.3683;27417600.0,278.8888;27421200.0,286.05756;27424800.0,293.2384;27428400.0,293.92923;27432000.0,302.71338;27435600.0,309.43582;27439200.0,311.8512;27442800.0,314.87476;27446400.0,313.59164;27450000.0,308.84555;27453600.0,303.7584;27457200.0,299.4145;27460800.0,295.6071;27464400.0,292.4328;27468000.0,289.7526;27471600.0,287.508;27475200.0,285.53558;27478800.0,283.91406;27482400.0,282.4722;27486000.0,281.26596;27489600.0,280.18594;27493200.0,279.35138;27496800.0,278.82883;27500400.0,278.47562;27504000.0,278.56665;27507600.0,282.20386;27511200.0,282.9693;27514800.0,285.85913;27518400.0,286.2723;27522000.0,287.28873;27525600.0,294.24805;27529200.0,299.41815;27532800.0,300.97992;27536400.0,297.63086;27540000.0,294.3073;27543600.0,291.2662;27547200.0,288.5766;27550800.0,286.30194;27554400.0,284.3375;27558000.0,282.22986;27561600.0,280.0349;27565200.0,278.6809;27568800.0,277.5733;27572400.0,276.61642;27576000.0,275.84332;27579600.0,275.298;27583200.0,274.72845;27586800.0,274.29318;27590400.0,275.2723;27594000.0,282.6291;27597600.0,290.89316;27601200.0,299.6752;27604800.0,309.24634;27608400.0,316.68887;27612000.0,321.29083;27615600.0,323.34787;27619200.0,322.03647;27622800.0,316.87537;27626400.0,311.07874;27630000.0,305.87143;27633600.0,301.56335;27637200.0,297.88647;27640800.0,294.94214;27644400.0,292.5075;27648000.0,290.41943;27651600.0,288.6974;27655200.0,287.4969;27658800.0,286.33374;27662400.0,285.3347;27666000.0,284.55298;27669600.0,284.19016;27673200.0,283.31064;27676800.0,282.7246;27680400.0,286.25092;27684000.0,292.34915;27687600.0,298.554;27691200.0,308.00433;27694800.0,315.67978;27698400.0,320.02292;27702000.0,321.89905;27705600.0,319.04715;27709200.0,313.6464;27712800.0,308.73557;27716400.0,304.60706;27720000.0,300.93198;27723600.0,297.70657;27727200.0,295.2949;27730800.0,293.2177;27734400.0,291.41626;27738000.0,289.72964;27741600.0,288.16077;27745200.0,286.4399;27748800.0,284.66895;27752400.0,282.76764;27756000.0,281.03775;27759600.0,279.73196;27763200.0,280.10898;27766800.0,286.7646;27770400.0,294.74237;27774000.0,302.51974;27777600.0,306.15436;27781200.0,313.79782;27784800.0,319.9384;27788400.0,322.69266;27792000.0,321.86398;27795600.0,315.76437;27799200.0,310.17767;27802800.0,305.5477;27806400.0,301.70236;27810000.0,298.3558;27813600.0,295.4995;27817200.0,292.89752;27820800.0,290.79504;27824400.0,289.1347;27828000.0,287.88177;27831600.0,286.8785;27835200.0,285.56055;27838800.0,284.55573;27842400.0,283.7263;27846000.0,282.8334;27849600.0,282.976;27853200.0,289.27753;27856800.0,296.59467;27860400.0,304.8944;27864000.0,313.4821;27867600.0,320.67267;27871200.0,325.67685;27874800.0,328.2741;27878400.0,327.01093;27882000.0,321.16364;27885600.0,314.88406;27889200.0,309.46698;27892800.0,304.70032;27896400.0,300.62854;27900000.0,297.33032;27903600.0,294.29712;27907200.0,291.74988;27910800.0,289.83832;27914400.0,288.3348;27918000.0,287.12823;27921600.0,285.99216;27925200.0,284.8905;27928800.0,283.8898;27932400.0,283.0161;27936000.0,286.87943;27939600.0,294.01022;27943200.0,302.38165;27946800.0,310.8186;27950400.0,318.95874;27954000.0,324.84854;27957600.0,329.23495;27961200.0,331.35178;27964800.0,329.66092;27968400.0,324.1212;27972000.0,317.79233;27975600.0,312.09894;27979200.0,306.55603;27982800.0,302.20068;27986400.0,298.62393;27990000.0,295.58774;27993600.0,292.51785;27997200.0,289.78033;28000800.0,287.51065;28004400.0,285.9987;28008000.0,284.7379;28011600.0,283.54175;28015200.0,282.4142;28018800.0,281.87332;28022400.0,285.84573;28026000.0,292.51837;28029600.0,300.98523;28033200.0,309.7001;28036800.0,317.53308;28040400.0,324.03027;28044000.0,328.49393;28047600.0,330.15045;28051200.0,328.54318;28054800.0,323.2498;28058400.0,316.91498;28062000.0,310.80646;28065600.0,305.6998;28069200.0,301.3566;28072800.0,297.27734;28076400.0,294.08582;28080000.0,291.2616;28083600.0,288.95508;28087200.0,287.50195;28090800.0,286.26474;28094400.0,285.14572;28098000.0,284.20114;28101600.0,283.49976;28105200.0,283.208;28108800.0,286.99283;28112400.0,293.40744;28116000.0,300.50702;28119600.0,310.3673;28123200.0,318.2508;28126800.0,325.89557;28130400.0,329.89664;28134000.0,326.77493;28137600.0,323.52576;28141200.0,318.3402;28144800.0,313.82675;28148400.0,309.1413;28152000.0,305.14395;28155600.0,301.39618;28159200.0,297.8824;28162800.0,295.03192;28166400.0,292.73892;28170000.0,290.63657;28173600.0,288.74496;28177200.0,286.874;28180800.0,285.04562;28184400.0,283.35986;28188000.0,281.94763;28191600.0,280.76517;28195200.0,280.17593;28198800.0,284.36224;28202400.0,288.3094;28206000.0,295.15497;28209600.0,303.357;28213200.0,310.12448;28216800.0,314.01843;28220400.0,316.083;28224000.0,315.6829;28227600.0,310.3074;28231200.0,305.2573;28234800.0,300.94666;28238400.0,296.9754;28242000.0,293.23013;28245600.0,289.91852;28249200.0,287.24075;28252800.0,285.00742;28256400.0,283.23532;28260000.0,281.6494;28263600.0,280.14066;28267200.0,278.59015;28270800.0,277.23938;28274400.0,275.9605;28278000.0,274.90982;28281600.0,274.06577;28285200.0,274.26154;28288800.0,276.7116;28292400.0,277.23505;28296000.0,276.97342;28299600.0,277.1642;28303200.0,277.21588;28306800.0,276.71298;28310400.0,276.2568;28314000.0,276.0065;28317600.0,275.14404;28321200.0,274.33118;28324800.0,273.62036;28328400.0,272.8883;28332000.0,272.3108;28335600.0,271.78885;28339200.0,271.2267;28342800.0,270.8473;28346400.0,270.4924;28350000.0,270.19467;28353600.0,269.88272;28357200.0,269.6076;28360800.0,269.29456;28364400.0,269.03833;28368000.0,268.78943;28371600.0,269.1483;28375200.0,270.01657;28378800.0,279.84116;28382400.0,291.24835;28386000.0,298.43445;28389600.0,296.04163;28393200.0,297.016;28396800.0,298.42123;28400400.0,295.02094;28404000.0,290.54843;28407600.0,286.97083;28411200.0,283.81915;28414800.0,281.66;28418400.0,279.44885;28422000.0,277.27744;28425600.0,275.12015;28429200.0,273.32806;28432800.0,271.3166;28436400.0,269.55823;28440000.0,268.16064;28443600.0,266.84195;28447200.0,265.3875;28450800.0,264.3192;28454400.0,264.07092;28458000.0,271.64282;28461600.0,281.31064;28465200.0,291.20108;28468800.0,300.51505;28472400.0,308.02042;28476000.0,310.49127;28479600.0,311.7258;28483200.0,310.4868;28486800.0,304.22818;28490400.0,298.5224;28494000.0,293.63623;28497600.0,289.35645;28501200.0,286.04016;28504800.0,283.39575;28508400.0,281.22696;28512000.0,279.02457;28515600.0,277.3832;28519200.0,276.14542;28522800.0,275.2141;28526400.0,274.43826;28530000.0,273.73468;28533600.0,273.11038;28537200.0,272.9103;28540800.0,273.39005;28544400.0,281.02698;28548000.0,290.01544;28551600.0,297.21768;28555200.0,306.90594;28558800.0,315.3409;28562400.0,321.1484;28566000.0,324.58005;28569600.0,324.00653;28573200.0,316.95532;28576800.0,310.11133;28580400.0,304.31808;28584000.0,299.26843;28587600.0,294.83456;28591200.0,291.21234;28594800.0,288.02628;28598400.0,285.11945;28602000.0,282.70587;28605600.0,280.80655;28609200.0,279.28192;28612800.0,277.88174;28616400.0,276.974;28620000.0,276.3192;28623600.0,275.8731;28627200.0,276.27908;28630800.0,284.12924;28634400.0,294.07513;28638000.0,304.60638;28641600.0,314.85117;28645200.0,323.48053;28648800.0,329.1631;28652400.0,331.7471;28656000.0,329.81525;28659600.0,322.20248;28663200.0,315.43723;28666800.0,309.59982;28670400.0,304.68478;28674000.0,300.81415;28677600.0,297.45905;28681200.0,294.6274;28684800.0,292.05447;28688400.0,289.81705;28692000.0,288.13583;28695600.0,286.75055;28699200.0,285.56332;28702800.0,284.61322;28706400.0,283.89612;28710000.0,283.2978;28713600.0,283.41937;28717200.0,290.92236;28720800.0,300.73407;28724400.0,310.8574;28728000.0,319.5697;28731600.0,326.36182;28735200.0,330.66254;28738800.0,331.9196;28742400.0,329.61057;28746000.0,321.738;28749600.0,314.52142;28753200.0,308.57184;28756800.0,303.4467;28760400.0,299.13046;28764000.0,295.15;28767600.0,291.70114;28771200.0,288.5863;28774800.0,285.91644;28778400.0,283.6091;28782000.0,281.79324;28785600.0,280.04462;28789200.0,278.43057;28792800.0,276.78745;28796400.0,275.43127;28800000.0,275.10593;28803600.0,282.81213;28807200.0,292.42303;28810800.0,302.25916;28814400.0,311.39017;28818000.0,318.9027;28821600.0,322.6876;28825200.0,319.4615;28828800.0,317.97137;28832400.0,312.31207;28836000.0,306.6049;28839600.0,301.68042;28843200.0,297.55612;28846800.0,293.9844;28850400.0,291.1021;28854000.0,288.84937;28857600.0,287.00253;28861200.0,285.6054;28864800.0,284.40536;28868400.0,283.47583;28872000.0,282.76965;28875600.0,281.84937;28879200.0,281.04675;28882800.0,280.79736;28886400.0,283.05145;28890000.0,289.63403;28893600.0,298.89667;28897200.0,308.93265;28900800.0,318.29126;28904400.0,326.34598;28908000.0,331.8903;28911600.0,334.18524;28915200.0,332.42075;28918800.0,326.12488;28922400.0,319.25995;28926000.0,313.12805;28929600.0,307.9175;28933200.0,303.3484;28936800.0,299.36636;28940400.0,295.98596;28944000.0,293.00552;28947600.0,290.3543;28951200.0,287.92673;28954800.0,285.63464;28958400.0,284.04016;28962000.0,282.63284;28965600.0,280.96414;28969200.0,280.07385;28972800.0,283.01328;28976400.0,290.1695;28980000.0,299.00128;28983600.0,308.49432;28987200.0,317.16425;28990800.0,324.9012;28994400.0,329.68973;28998000.0,331.4074;29001600.0,330.25897;29005200.0,324.22482;29008800.0,317.84702;29012400.0,312.13168;29016000.0,306.92645;29019600.0,302.36133;29023200.0,298.32568;29026800.0,294.7364;29030400.0,291.99368;29034000.0,289.88882;29037600.0,287.79794;29041200.0,286.02878;29044800.0,284.44995;29048400.0,283.18823;29052000.0,282.17114;29055600.0,281.12723;29059200.0,282.06152;29062800.0,287.44043;29066400.0,295.2506;29070000.0,302.1712;29073600.0,309.9693;29077200.0,315.64285;29080800.0,319.9719;29084400.0,320.77237;29088000.0,317.14102;29091600.0,312.01547;29095200.0,307.24448;29098800.0,303.2015;29102400.0,299.52682;29106000.0,296.1051;29109600.0,293.1922;29113200.0,290.5817;29116800.0,288.58694;29120400.0,286.85025;29124000.0,285.4294;29127600.0,284.15622;29131200.0,283.17804;29134800.0,282.27747;29138400.0,281.20108;29142000.0,280.45602;29145600.0,281.02542;29149200.0,287.96524;29152800.0,295.31805;29156400.0,304.90732;29160000.0,311.97275;29163600.0,319.69226;29167200.0,322.79718;29170800.0,321.46655;29174400.0,316.73157;29178000.0,311.38742;29181600.0,305.5719;29185200.0,300.55713;29188800.0,296.26;29192400.0,292.61862;29196000.0,289.5365;29199600.0,286.87216;29203200.0,284.65768;29206800.0,282.7908;29210400.0,281.1036;29214000.0,279.59103;29217600.0,278.2815;29221200.0,277.17908;29224800.0,276.20224;29228400.0,275.4244;29232000.0,274.82327;29235600.0,274.68066;29239200.0,274.89984;29242800.0,275.25787;29246400.0,276.08282;29250000.0,278.2103;29253600.0,285.74704;29257200.0,287.32706;29260800.0,286.81464;29264400.0,284.89935;29268000.0,283.03543;29271600.0,281.35626;29275200.0,280.03122;29278800.0,278.8113;29282400.0,277.7128;29286000.0,276.79947;29289600.0,275.80078;29293200.0,274.76477;29296800.0,274.04575;29300400.0,273.6056;29304000.0,273.32843;29307600.0,273.12903;29311200.0,272.99884;29314800.0,272.88132;29318400.0,272.92703;29322000.0,273.2684;29325600.0,273.80182;29329200.0,274.4836;29332800.0,275.3061;29336400.0,276.2614;29340000.0,276.92664;29343600.0,277.01614;29347200.0,276.85025;29350800.0,276.41583;29354400.0,275.9798;29358000.0,275.46246;29361600.0,274.86966;29365200.0,274.19495;29368800.0,273.54123;29372400.0,272.8948;29376000.0,272.22873;29379600.0,271.68365;29383200.0,271.13116;29386800.0,271.04358;29390400.0,271.14767;29394000.0,271.17825;29397600.0,271.59402;29401200.0,272.28815;29404800.0,273.39484;29408400.0,275.5382;29412000.0,277.87442;29415600.0,280.21057;29419200.0,287.82925;29422800.0,292.5275;29426400.0,299.36752;29430000.0,301.6917;29433600.0,300.5524;29437200.0,296.91382;29440800.0,293.4785;29444400.0,290.65848;29448000.0,288.08694;29451600.0,285.88702;29455200.0,283.9752;29458800.0,282.2767;29462400.0,280.83185;29466000.0,279.4348;29469600.0,278.1372;29473200.0,277.05826;29476800.0,276.1084;29480400.0,275.2452;29484000.0,274.4877;29487600.0,273.84018;29491200.0,273.37393;29494800.0,273.58218;29498400.0,274.02216;29502000.0,276.0821;29505600.0,281.58487;29509200.0,281.78503;29512800.0,283.4226;29516400.0,285.28452;29520000.0,285.37808;29523600.0,284.01773;29527200.0,281.9203;29530800.0,280.08444;29534400.0,278.41745;29538000.0,276.8313;29541600.0,275.39035;29545200.0,274.08948;29548800.0,272.82086;29552400.0,271.60547;29556000.0,270.66446;29559600.0,269.81564;29563200.0,269.04077;29566800.0,268.33347;29570400.0,267.72064;29574000.0,267.3096;29577600.0,268.89465;29581200.0,274.16275;29584800.0,282.41235;29588400.0,290.9107;29592000.0,299.99826;29595600.0,305.98853;29599200.0,311.44434;29602800.0,312.88675;29606400.0,311.48178;29610000.0,306.2456;29613600.0,300.99942;29617200.0,296.51584;29620800.0,292.69144;29624400.0,289.4693;29628000.0,286.39688;29631600.0,283.68207;29635200.0,281.1685;29638800.0,278.81824;29642400.0,276.68787;29646000.0,274.82285;29649600.0,273.29092;29653200.0,271.9755;29656800.0,270.68417;29660400.0,269.7678;29664000.0,272.03415;29667600.0,278.80392;29671200.0,287.77835;29674800.0,297.7016;29678400.0,307.404;29682000.0,315.3949;29685600.0,321.08884;29689200.0,323.3661;29692800.0,321.5448;29696400.0,315.1162;29700000.0,308.24023;29703600.0,302.4565;29707200.0,297.3902;29710800.0,293.13458;29714400.0,289.493;29718000.0,286.3824;29721600.0,283.75604;29725200.0,281.57147;29728800.0,279.53217;29732400.0,277.87903;29736000.0,276.5616;29739600.0,275.4255;29743200.0,274.53516;29746800.0,273.72845;29750400.0,273.29343;29754000.0,273.5337;29757600.0,274.23123;29761200.0,276.63885;29764800.0,282.9863;29768400.0,286.1224;29772000.0,293.6974;29775600.0,294.05493;29779200.0,294.53775;29782800.0,292.53354;29786400.0,290.23337;29790000.0,288.0393;29793600.0,286.0721;29797200.0,284.03748;29800800.0,282.04932;29804400.0,280.0621;29808000.0,278.2321;29811600.0,276.6365;29815200.0,275.1753;29818800.0,274.00375;29822400.0,272.81342;29826000.0,271.6866;29829600.0,270.802;29833200.0,270.23776;29836800.0,271.0387;29840400.0,276.4664;29844000.0,285.53973;29847600.0,295.32785;29851200.0,302.8245;29854800.0,309.56433;29858400.0,312.35095;29862000.0,314.68103;29865600.0,313.5703;29869200.0,308.33984;29872800.0,303.33438;29876400.0,298.90082;29880000.0,294.88852;29883600.0,291.44348;29887200.0,288.42892;29890800.0,285.81033;29894400.0,283.4524;29898000.0,281.36694;29901600.0,279.695;29905200.0,278.28265;29908800.0,277.04617;29912400.0,276.0135;29916000.0,274.87793;29919600.0,273.76373;29923200.0,274.97214;29926800.0,280.69095;29930400.0,289.3248;29934000.0,298.7534;29937600.0,307.42627;29941200.0,313.91202;29944800.0,318.29608;29948400.0,317.66995;29952000.0,314.19855;29955600.0,308.5049;29959200.0,303.2063;29962800.0,298.66736;29966400.0,294.77332;29970000.0,291.1258;29973600.0,288.22824;29977200.0,285.93655;29980800.0,283.70746;29984400.0,281.59213;29988000.0,279.8175;29991600.0,278.19257;29995200.0,276.69647;29998800.0,275.28207;30002400.0,274.01678;30006000.0,273.112;30009600.0,273.3806;30013200.0,277.64468;30016800.0,285.62408;30020400.0,294.78516;30024000.0,302.9904;30027600.0,310.83948;30031200.0,315.57047;30034800.0,316.8431;30038400.0,315.31195;30042000.0,309.79053;30045600.0,304.18005;30049200.0,299.4066;30052800.0,295.13925;30056400.0,291.4122;30060000.0,288.15686;30063600.0,285.3967;30067200.0,283.06793;30070800.0,281.26102;30074400.0,279.70654;30078000.0,278.11145;30081600.0,276.60648;30085200.0,275.17093;30088800.0,273.80106;30092400.0,272.76562;30096000.0,272.6254;30099600.0,275.49545;30103200.0,282.92313;30106800.0,290.5982;30110400.0,298.68674;30114000.0,306.33118;30117600.0,312.92593;30121200.0,315.5001;30124800.0,313.97385;30128400.0,308.677;30132000.0,303.66196;30135600.0,299.34442;30139200.0,295.57916;30142800.0,292.33237;30146400.0,289.46045;30150000.0,286.59735;30153600.0,284.04333;30157200.0,281.77463;30160800.0,279.7989;30164400.0,278.22305;30168000.0,276.96738;30171600.0,275.78848;30175200.0,274.49548;30178800.0,273.3148;30182400.0,274.17303;30186000.0,280.06906;30189600.0,288.24017;30193200.0,296.3949;30196800.0,304.5188;30200400.0,309.14407;30204000.0,312.3841;30207600.0,313.66925;30211200.0,310.48755;30214800.0,306.2367;30218400.0,302.4203;30222000.0,299.34216;30225600.0,296.82303;30229200.0,294.6211;30232800.0,292.60028;30236400.0,290.62106;30240000.0,288.61954;30243600.0,286.68515;30247200.0,284.61243;30250800.0,282.92264;30254400.0,281.53964;30258000.0,280.32333;30261600.0,279.11996;30265200.0,278.40918;30268800.0,278.6862;30272400.0,280.0406;30276000.0,284.534;30279600.0,289.30322;30283200.0,297.09268;30286800.0,305.41095;30290400.0,311.17548;30294000.0,313.0053;30297600.0,310.81638;30301200.0,306.61694;30304800.0,302.0078;30308400.0,297.90686;30312000.0,294.19742;30315600.0,290.93353;30319200.0,288.2512;30322800.0,285.82513;30326400.0,283.73407;30330000.0,281.97623;30333600.0,280.18707;30337200.0,278.89728;30340800.0,278.00272;30344400.0,277.21875;30348000.0,276.34683;30351600.0,275.5508;30355200.0,277.24277;30358800.0,283.01328;30362400.0,292.36118;30366000.0,302.542;30369600.0,312.37997;30373200.0,320.97092;30376800.0,327.39056;30380400.0,330.64944;30384000.0,329.56732;30387600.0,323.62274;30391200.0,316.79205;30394800.0,311.00415;30398400.0,305.76343;30402000.0,301.1941;30405600.0,297.62607;30409200.0,294.45227;30412800.0,291.6054;30416400.0,289.0587;30420000.0,286.94772;30423600.0,285.43967;30427200.0,283.4592;30430800.0,282.66263;30434400.0,282.20804;30438000.0,281.68842;30441600.0,281.82367;30445200.0,282.28333;30448800.0,285.11472;30452400.0,293.5801;30456000.0,301.51352;30459600.0,309.7365;30463200.0,314.89435;30466800.0,315.63986;30470400.0,314.54718;30474000.0,309.5319;30477600.0,304.31064;30481200.0,300.0258;30484800.0,296.27142;30488400.0,293.20996;30492000.0,290.68478;30495600.0,288.46036;30499200.0,286.36993;30502800.0,284.36237;30506400.0,282.33878;30510000.0,280.28815;30513600.0,278.52505;30517200.0,277.05518;30520800.0,275.9034;30524400.0,275.06525;30528000.0,275.88333;30531600.0,281.58667;30535200.0,290.49594;30538800.0,300.6223;30542400.0,310.59152;30546000.0,318.81638;30549600.0,323.72467;30553200.0,325.78262;30556800.0,324.11475;30560400.0,318.0419;30564000.0,311.60446;30567600.0,306.16324;30571200.0,301.41553;30574800.0,297.34958;30578400.0,293.72253;30582000.0,290.4056;30585600.0,287.87784;30589200.0,285.6402;30592800.0,283.56226;30596400.0,281.79184;30600000.0,280.21188;30603600.0,278.85446;30607200.0,277.683;30610800.0,276.62708;30614400.0,277.9146;30618000.0,284.10013;30621600.0,293.096;30625200.0,303.26276;30628800.0,313.11005;30632400.0,321.63123;30636000.0,327.61124;30639600.0,330.53363;30643200.0,329.5042;30646800.0,323.11636;30650400.0,316.30048;30654000.0,310.3621;30657600.0,305.2508;30661200.0,300.90427;30664800.0,297.10748;30668400.0,293.85867;30672000.0,291.2134;30675600.0,288.90695;30679200.0,286.8314;30682800.0,285.12106;30686400.0,283.74005;30690000.0,282.4114;30693600.0,280.9473;30697200.0,279.64355;30700800.0,280.0275;30704400.0,285.40262;30708000.0,294.74478;30711600.0,304.965;30715200.0,314.70108;30718800.0,323.07245;30722400.0,329.09558;30726000.0,331.4988;30729600.0,329.75522;30733200.0,323.84647;30736800.0,316.473;30740400.0,310.02234;30744000.0,304.38412;30747600.0,299.52264;30751200.0,295.37314;30754800.0,291.97644;30758400.0,289.07053;30762000.0,286.4869;30765600.0,284.02036;30769200.0,282.22958;30772800.0,280.6571;30776400.0,279.0105;30780000.0,277.43094;30783600.0,276.22266;30787200.0,275.42755;30790800.0,275.71695;30794400.0,276.9069;30798000.0,278.54004;30801600.0,280.3431;30805200.0,282.09747;30808800.0,283.52585;30812400.0,284.36218;30816000.0,284.2963;30819600.0,283.5096;30823200.0,282.37164;30826800.0,281.21;30830400.0,280.32077;30834000.0,279.50403;30837600.0,278.87177;30841200.0,278.39694;30844800.0,278.5947;30848400.0,279.11703;30852000.0,279.4127;30855600.0,279.26703;30859200.0,278.1813;30862800.0,277.16522;30866400.0,276.21738;30870000.0,275.48297;30873600.0,275.75406;30877200.0,279.21698;30880800.0,285.70746;30884400.0,289.70892;30888000.0,294.5727;30891600.0,298.20956;30895200.0,306.01175;30898800.0,311.18588;30902400.0,312.81943;30906000.0,309.77594;30909600.0,305.25964;30913200.0,301.2064;30916800.0,297.33917;30920400.0,293.9173;30924000.0,290.96353;30927600.0,288.5313;30931200.0,286.57675;30934800.0,284.74338;30938400.0,283.02307;30942000.0,281.48248;30945600.0,280.04578;30949200.0,278.8841;30952800.0,278.32922;30956400.0,278.05258;30960000.0,278.5432;30963600.0,280.4814;30967200.0,285.2895;30970800.0,292.48148;30974400.0,299.0353;30978000.0,307.6686;30981600.0,315.5717;30985200.0,316.30765;30988800.0,313.2037;30992400.0,309.43784;30996000.0,305.37064;30999600.0,301.84177;31003200.0,298.73453;31006800.0,295.9356;31010400.0,293.5434;31014000.0,291.07892;31017600.0,288.91016;31021200.0,286.8326;31024800.0,285.07227;31028400.0,283.6008;31032000.0,282.4189;31035600.0,281.4356;31039200.0,280.65598;31042800.0,279.97815;31046400.0,279.7027;31050000.0,283.2577;31053600.0,291.36472;31057200.0,301.27255;31060800.0,310.97446;31064400.0,319.13693;31068000.0,325.12866;31071600.0,328.06857;31075200.0,327.2974;31078800.0,321.84097;31082400.0,315.5009;31086000.0,309.87115;31089600.0,304.92786;31093200.0,300.51077;31096800.0,296.69556;31100400.0,293.31525;31104000.0,290.56512;31107600.0,288.2153;31111200.0,286.05954;31114800.0,284.25906;31118400.0,282.77103;31122000.0,281.36176;31125600.0,279.75818;31129200.0,278.1148;31132800.0,278.31223;31136400.0,283.3393;31140000.0,291.49228;31143600.0,301.02765;31147200.0,310.29224;31150800.0,318.27145;31154400.0,323.88174;31158000.0,326.24164;31161600.0,324.6233;31165200.0,318.7614;31168800.0,312.49817;31172400.0,306.97638;31176000.0,302.15427;31179600.0,297.95935;31183200.0,294.00342;31186800.0,290.51895;31190400.0,287.54343;31194000.0,284.9136;31197600.0,282.5877;31201200.0,280.65637;31204800.0,278.98938;31208400.0,277.24454;31212000.0,275.875;31215600.0,274.77432;31219200.0,276.11917;31222800.0,282.54645;31226400.0,291.01065;31230000.0,300.87338;31233600.0,309.7482;31237200.0,317.489;31240800.0,322.38583;31244400.0,324.4846;31248000.0,322.83633;31251600.0,317.30515;31255200.0,310.90903;31258800.0,305.22302;31262400.0,300.5156;31266000.0,296.4104;31269600.0,292.83704;31273200.0,289.74698;31276800.0,287.0066;31280400.0,284.5437;31284000.0,282.50775;31287600.0,280.6291;31291200.0,278.87845;31294800.0,277.36423;31298400.0,276.0128;31302000.0,275.01328;31305600.0,274.37387;31309200.0,274.91855;31312800.0,277.16913;31316400.0,281.54727;31320000.0,283.3403;31323600.0,283.91708;31327200.0,284.30792;31330800.0,287.5164;31334400.0,288.44507;31338000.0,287.02176;31341600.0,285.59137;31345200.0,284.52237;31348800.0,283.52374;31352400.0,282.59384;31356000.0,281.50052;31359600.0,280.4694;31363200.0,279.59933;31366800.0,278.85327;31370400.0,278.14392;31374000.0,277.45963;31377600.0,276.72644;31381200.0,275.85696;31384800.0,274.9787;31388400.0,274.25684;31392000.0,273.6744;31395600.0,273.33432;31399200.0,273.3169;31402800.0,273.57095;31406400.0,273.81125;31410000.0,274.00906;31413600.0,274.20828;31417200.0,274.13028;31420800.0,273.8855;31424400.0,273.4625;31428000.0,273.01382;31431600.0,272.57892;31435200.0,272.1718;31438800.0,271.76834;31442400.0,271.3385;31446000.0,270.89197;31449600.0,270.4426;31453200.0,269.94315;31456800.0,269.4685;31460400.0,268.89313;31464000.0,268.35617;31467600.0,267.8559;31471200.0,267.4354;31474800.0,267.08978;31478400.0,266.8991;31482000.0,268.34283;31485600.0,272.74277;31489200.0,282.0447;31492800.0,289.5467;31496400.0,291.75885;31500000.0,296.97284;31503600.0,295.74048;31507200.0,293.6715;31510800.0,289.70047;31514400.0,285.55814;31518000.0,281.82504;31521600.0,278.72598;31525200.0,275.89136;31528800.0,273.2734;31532400.0,270.78244;31534200.0,269.51593;31534200.0,269.51593;31536000.0,268.23813;31536000.0,268.23813])
annotation (Placement(transformation(extent={{-4,-34},{40,10}})));

Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p annotation (
Placement(transformation(extent={{74,-20},{110,20}}), iconTransformation(
    extent={{74,-20},{110,20}})));
Buildings.Electrical.AC.OnePhase.Loads.Resistive loa(linearized=true, mode=
  Buildings.Electrical.Types.Load.VariableZ_P_input)
annotation (Placement(transformation(extent={{-32,-72},{16,-26}})));
Modelica.Blocks.Math.Sum sum1(nin= 1)

annotation (Placement(transformation(extent={{-72,-36},{-52,-16}})));
  Modelica.Blocks.Sources.Constant const(k=0)
annotation (Placement(transformation(extent={{-84,32},{-64,52}})));
equation
connect(dataBus.roo_heaPorAir_T, combiTimeTable.y[1]);
connect(port[1],TRoo[1]. port);
connect(port_a[1], TRoo1[1].port);
connect(dataBus.TZonSpace_001, TRoo[1].T);
connect(dataBus.ppmCO2Space_001, TRoo1[1].ppm);


connect(term_p, loa.terminal) annotation (Line(points={{92,0},{-32,0},{-32,-51},
{ -28,-51 } }, color={0,120,120}));

        connect(const.y, sum1.u[1]) annotation (Line(points={{-63,42},{-58,42},{-58,14},
    {-76,14},{-76,-20},{-74,-20},{-74,-26}}, color={0,0,127}));
connect(sum1.y, loa.Pow) annotation (Line(points={{-51,-26},{-38,-26},{-38,-76},
    {24,-76},{24,-49},{16,-49}}, color={0,0,127}));
end DataServer;
  end BaseClasses;
end Components;

model building
  Components.Containers.building building
    annotation (Placement(transformation(extent={{12,-24},{60,24}})));
  Buildings.Electrical.AC.OnePhase.Sources.Grid gri
    annotation (Placement(transformation(extent={{-46,28},{-26,48}})));
equation
  connect(gri.terminal, building.term_p)
    annotation (Line(points={{-36,28},{-36,0},{12,0}}, color={0,120,120}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end building;
end case600FF;