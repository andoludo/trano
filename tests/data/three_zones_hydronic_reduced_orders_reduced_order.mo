package three_zones_hydronic_reduced_orders_reduced_order

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
    model distribution


extends Trano.BaseClasses.Containers.distribution;
parameter Real mRad_flow_nominal = 123;
package MediumW = AixLib.Media.Water;

replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = MediumW)
    annotation (
Placement(transformation(extent= {{88,40},{108,60}} ), iconTransformation(
      extent= {{88,40},{108,60}} )),
iconTransformation(extent=  {{90,-60},{110,-40}} ));

Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = MediumW)
annotation (
Placement(transformation(extent= {{90,-60},{110,-40}} ), iconTransformation(
      extent= {{90,-60},{110,-40}} )),
iconTransformation(extent=  {{90,40},{110,60}} ));

Modelica.Fluid.Interfaces.FluidPort_a[3] port_a1(
    redeclare package Medium = MediumW)
    annotation (
        Placement(transformation(extent= {{-110,-58},{-90,-38}} )),
        iconTransformation(extent= {{-110,-58},{-90,-38}} )
    );

Modelica.Fluid.Interfaces.FluidPort_b[3] port_b1(
    redeclare package Medium = MediumW)
    annotation (
        Placement(transformation(extent= {{-110,40},{-90,60}} )),
        iconTransformation(extent= {{-110,38},{-90,58}} )
    );

// Data Bus Connection
Trano.Controls.BaseClasses.DataBus dataBus 
    annotation (
        Placement(transformation(extent= {{-118,68},{-78,108}} )), 
        iconTransformation(extent= {{-228,58},{-208,78}} )
    );
      three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.PumpPump_001
     pump_001(
         dp_nominal=30000.0,
    m_flow_nominal=0.15
,
    redeclare package Medium = MediumW

    ) annotation (
    Placement(transformation(origin = { -0.06883013810072214, -11.022242378017367 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.CollectorControlControl_5
    control_5 annotation (
    Placement(transformation(origin = { -0.15864612759224883, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             three_way_valve_001(
    redeclare package Medium = MediumW,
          dpFixed_nominal={2000,0},
    dpValve_nominal=6000.0,
    fraK=0.7,
    deltaM=0.02,
    m_flow_nominal=0.3,
    delta0=0.01,
    R=50.0,
    linearized={true, true},
    l={0.01,0.01}
,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve"  annotation (
    Placement(transformation(origin = { -59.54946917886594, 12.14363752550966 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.
    ThreeWayValveControlControl_6
    control_6 annotation (
    Placement(transformation(origin = { -100.0, 8.68050545237162 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             three_way_valve_002(
    redeclare package Medium = MediumW,
          dpFixed_nominal={2000,0},
    dpValve_nominal=6000.0,
    fraK=0.7,
    deltaM=0.02,
    m_flow_nominal=0.3,
    delta0=0.01,
    R=50.0,
    linearized={true, true},
    l={0.01,0.01}
,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve"  annotation (
    Placement(transformation(origin = { 59.46708028081312, 12.019954237183313 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.
    ThreeWayValveControlControl_7
    control_7 annotation (
    Placement(transformation(origin = { 91.22566871890444, -44.189977530869285 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.FixedResistances.Junction split_valve_001 (
        dp_nominal={5000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.1*{1,-1,-1},
    linearized=true
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { -76.6369611796294, 99.81447506751047 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor_001(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { -91.3097848475244, -44.045680361155206 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.FixedResistances.Junction split_valve_002 (
        dp_nominal={5000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.1*{1,-1,-1},
    linearized=true
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { 76.52693659250224, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor_002(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { 100.0, 8.643400465873711 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(split_valve_001.port_1,port_a1[1])
        annotation (Line(
        points={{ -76.6369611796294, 99.81447506751047 }    ,{ -38.3184805898147, 99.81447506751047 }    ,{ -38.3184805898147, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve_001.port_1,port_a1[2])
        annotation (Line(
        points={{ -76.6369611796294, 99.81447506751047 }    ,{ -38.3184805898147, 99.81447506751047 }    ,{ -38.3184805898147, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve_002.port_1,port_a1[3])
        annotation (Line(
        points={{ 76.52693659250224, 100.0 }    ,{ 38.26346829625112, 100.0 }    ,{ 38.26346829625112, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(pump_001.dataBus,control_5.dataBus)
        annotation (Line(
        points={{ -0.06883013810072214, -11.022242378017367 }    ,{ -0.11373813284648548, -11.022242378017367 }    ,{ -0.11373813284648548, -100.0 }    ,{ -0.15864612759224883, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pump_001.port_b,three_way_valve_001.port_1)
        annotation (Line(
        points={{ -0.06883013810072214, -11.022242378017367 }    ,{ -29.80914965848333, -11.022242378017367 }    ,{ -29.80914965848333, 12.14363752550966 }    ,{ -59.54946917886594, 12.14363752550966 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pump_001.port_b,three_way_valve_002.port_1)
        annotation (Line(
        points={{ -0.06883013810072214, -11.022242378017367 }    ,{ 29.6991250713562, -11.022242378017367 }    ,{ 29.6991250713562, 12.019954237183313 }    ,{ 59.46708028081312, 12.019954237183313 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_001.y,control_6.y)
        annotation (Line(
        points={{ -59.54946917886594, 12.14363752550966 }    ,{ -79.77473458943297, 12.14363752550966 }    ,{ -79.77473458943297, 8.68050545237162 }    ,{ -100.0, 8.68050545237162 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_001.port_2,temperature_sensor_001.port_a)
        annotation (Line(
        points={{ -59.54946917886594, 12.14363752550966 }    ,{ -75.42962701319517, 12.14363752550966 }    ,{ -75.42962701319517, -44.045680361155206 }    ,{ -91.3097848475244, -44.045680361155206 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_001.port_3,split_valve_001.port_3)
        annotation (Line(
        points={{ -59.54946917886594, 12.14363752550966 }    ,{ -68.09321517924766, 12.14363752550966 }    ,{ -68.09321517924766, 99.81447506751047 }    ,{ -76.6369611796294, 99.81447506751047 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(control_6.u,temperature_sensor_001.T)
        annotation (Line(
        points={{ -100.0, 8.68050545237162 }    ,{ -95.65489242376219, 8.68050545237162 }    ,{ -95.65489242376219, -44.045680361155206 }    ,{ -91.3097848475244, -44.045680361155206 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_002.y,control_7.y)
        annotation (Line(
        points={{ 59.46708028081312, 12.019954237183313 }    ,{ 75.34637449985878, 12.019954237183313 }    ,{ 75.34637449985878, -44.189977530869285 }    ,{ 91.22566871890444, -44.189977530869285 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_002.port_2,temperature_sensor_002.port_a)
        annotation (Line(
        points={{ 59.46708028081312, 12.019954237183313 }    ,{ 79.73354014040656, 12.019954237183313 }    ,{ 79.73354014040656, 8.643400465873711 }    ,{ 100.0, 8.643400465873711 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_002.port_3,split_valve_002.port_3)
        annotation (Line(
        points={{ 59.46708028081312, 12.019954237183313 }    ,{ 67.99700843665768, 12.019954237183313 }    ,{ 67.99700843665768, 100.0 }    ,{ 76.52693659250224, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(control_7.u,temperature_sensor_002.T)
        annotation (Line(
        points={{ 91.22566871890444, -44.189977530869285 }    ,{ 95.61283435945222, -44.189977530869285 }    ,{ 95.61283435945222, 8.643400465873711 }    ,{ 100.0, 8.643400465873711 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pump_001.port_a,port_a)
        annotation (Line(
        points={{ -0.06883013810072214, -11.022242378017367 }    ,{ -0.03441506905036107, -11.022242378017367 }    ,{ -0.03441506905036107, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve_001.port_2,port_b)
        annotation (Line(
        points={{ -76.6369611796294, 99.81447506751047 }    ,{ -38.3184805898147, 99.81447506751047 }    ,{ -38.3184805898147, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor_001.port_b,port_b1[1])
        annotation (Line(
        points={{ -91.3097848475244, -44.045680361155206 }    ,{ -45.6548924237622, -44.045680361155206 }    ,{ -45.6548924237622, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor_001.port_b,port_b1[2])
        annotation (Line(
        points={{ -91.3097848475244, -44.045680361155206 }    ,{ -45.6548924237622, -44.045680361155206 }    ,{ -45.6548924237622, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve_002.port_2,port_b)
        annotation (Line(
        points={{ 76.52693659250224, 100.0 }    ,{ 38.26346829625112, 100.0 }    ,{ 38.26346829625112, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor_002.port_b,port_b1[3])
        annotation (Line(
        points={{ 100.0, 8.643400465873711 }    ,{ 50.0, 8.643400465873711 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(control_5.dataBus,dataBus)
            ;        
        connect(control_6.dataBus,dataBus)
            ;        
        connect(control_7.dataBus,dataBus)
            ;end distribution;
    model emission


// Define Medium
extends Trano.BaseClasses.Containers.emission ;
package MediumW = AixLib.Media.Water;
// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_a[3] port_a(
  redeclare package Medium = MediumW)
 annotation (Placement(transformation(extent= {{90,44},{110,64}} ), iconTransformation(
      extent= {{90,44},{110,64}} )),iconTransformation(extent=  {{90,-60},{110,-40}} ));
Modelica.Fluid.Interfaces.FluidPort_b[3] port_b(
  redeclare package Medium = MediumW) annotation (Placement(transformation(extent= {{90,-64},{110,-44}} ), 
iconTransformation(extent= {{90,-64},{110,-44}} )),iconTransformation(extent=  {{90,40},{110,60}} ));
// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortCon
  annotation (
      Placement(transformation(extent= {{-108,42},{-88,62}} )),
      iconTransformation(extent= {{-108,42},{-88,62}} )
  );
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortRad
  annotation (
      Placement(transformation(extent= {{-110,-60},{-90,-40}} )),
      iconTransformation(extent= {{-110,-60},{-90,-40}} )
  );

// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
  annotation (
      Placement(transformation(extent= {{-118,68},{-78,108}} )), 
      iconTransformation(extent= {{-228,58},{-208,78}} )
  );
        AixLib.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_003(
                TAir_nominal=293.15,
    dp_nominal=2000.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=5000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=353.15,
    T_b_nominal=333.15,
    mDry=131.5,
    VWat=0.29
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 18.85904612686835, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_003(
                    dpFixed_nominal=5000.0,
    dpValve_nominal=10000.0,
    deltaM=0.02,
    m_flow_nominal=0.02,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { -37.86906423312979, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.EmissionControlControl_1
    control_1(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0,
    k=5.0
) annotation (
    Placement(transformation(origin = { -100.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_001(
                TAir_nominal=293.15,
    dp_nominal=2000.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=5000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=353.15,
    T_b_nominal=333.15,
    mDry=131.5,
    VWat=0.29
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 100.0, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_001(
                    dpFixed_nominal=5000.0,
    dpValve_nominal=10000.0,
    deltaM=0.02,
    m_flow_nominal=0.02,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 43.27188964000186, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.EmissionControlControl_2
    control_2(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0,
    k=5.0
) annotation (
    Placement(transformation(origin = { -18.858008518762546, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_002(
                TAir_nominal=293.15,
    dp_nominal=2000.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=5000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=353.15,
    T_b_nominal=333.15,
    mDry=131.5,
    VWat=0.29
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 90.4540054266904, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        AixLib.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_002(
                    dpFixed_nominal=5000.0,
    dpValve_nominal=10000.0,
    deltaM=0.02,
    m_flow_nominal=0.02,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 33.72589506669226, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.EmissionControlControl_3
    control_3(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0,
    k=5.0
) annotation (
    Placement(transformation(origin = { -28.40400309207216, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(radiator_003.heatPortCon,heatPortCon[1])
        annotation (Line(
        points={{ 18.85904612686835, 0.0 }    ,{ 9.429523063434175, 0.0 }    ,{ 9.429523063434175, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_003.heatPortRad,heatPortRad[1])
        annotation (Line(
        points={{ 18.85904612686835, 0.0 }    ,{ 9.429523063434175, 0.0 }    ,{ 9.429523063434175, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_003.port_b,valve_003.port_a)
        annotation (Line(
        points={{ 18.85904612686835, 0.0 }    ,{ -9.50500905313072, 0.0 }    ,{ -9.50500905313072, 0.0 }    ,{ -37.86906423312979, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_003.y,control_1.y)
        annotation (Line(
        points={{ -37.86906423312979, 0.0 }    ,{ -68.9345321165649, 0.0 }    ,{ -68.9345321165649, 0.0 }    ,{ -100.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_003.port_b,port_b[1])
        annotation (Line(
        points={{ -37.86906423312979, 0.0 }    ,{ -18.934532116564895, 0.0 }    ,{ -18.934532116564895, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_001.heatPortCon,heatPortCon[2])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_001.heatPortRad,heatPortRad[2])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_001.port_b,valve_001.port_a)
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 71.63594482000093, 100.0 }    ,{ 71.63594482000093, 100.0 }    ,{ 43.27188964000186, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_001.y,control_2.y)
        annotation (Line(
        points={{ 43.27188964000186, 100.0 }    ,{ 12.206940560619657, 100.0 }    ,{ 12.206940560619657, 100.0 }    ,{ -18.858008518762546, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_001.port_b,port_b[2])
        annotation (Line(
        points={{ 43.27188964000186, 100.0 }    ,{ 21.63594482000093, 100.0 }    ,{ 21.63594482000093, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_002.heatPortCon,heatPortCon[3])
        annotation (Line(
        points={{ 90.4540054266904, -100.0 }    ,{ 45.2270027133452, -100.0 }    ,{ 45.2270027133452, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_002.heatPortRad,heatPortRad[3])
        annotation (Line(
        points={{ 90.4540054266904, -100.0 }    ,{ 45.2270027133452, -100.0 }    ,{ 45.2270027133452, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_002.port_b,valve_002.port_a)
        annotation (Line(
        points={{ 90.4540054266904, -100.0 }    ,{ 62.08995024669133, -100.0 }    ,{ 62.08995024669133, -100.0 }    ,{ 33.72589506669226, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_002.y,control_3.y)
        annotation (Line(
        points={{ 33.72589506669226, -100.0 }    ,{ 2.6609459873100505, -100.0 }    ,{ 2.6609459873100505, -100.0 }    ,{ -28.40400309207216, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_002.port_b,port_b[3])
        annotation (Line(
        points={{ 33.72589506669226, -100.0 }    ,{ 16.86294753334613, -100.0 }    ,{ 16.86294753334613, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_001.port_a,port_a[1])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_003.port_a,port_a[2])
        annotation (Line(
        points={{ 18.85904612686835, 0.0 }    ,{ 9.429523063434175, 0.0 }    ,{ 9.429523063434175, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_002.port_a,port_a[3])
        annotation (Line(
        points={{ 90.4540054266904, -100.0 }    ,{ 45.2270027133452, -100.0 }    ,{ 45.2270027133452, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(control_1.dataBus,dataBus)
            ;        
        connect(control_2.dataBus,dataBus)
            ;        
        connect(control_3.dataBus,dataBus)
            ;end emission;
    model envelope



// Define Medium Package
extends Trano.BaseClasses.Containers.envelope ;
replaceable package Medium = AixLib.Media.Air
  constrainedby Modelica.Media.Interfaces.PartialMedium

  annotation (choicesAllMatching = true);

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortCon
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[0] heatPortCon1
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{-4,98},{6,108}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortRad

  annotation (
      Placement(transformation(extent= {{90,-62},{110,-42}} )),
      iconTransformation(extent= {{90,-62},{110,-42}} )
  );
  Modelica.Blocks.Interfaces.RealOutput y[3] annotation (Placement(transformation(
      extent= {{-100,-16},{-134,18}} ), iconTransformation(extent= {{-100,-16},{-134,18}} )));
// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
  annotation (Placement(transformation(extent= {{-20,80},{20,120}} )),iconTransformation(extent=  {{-228,58},{-208,78}} ));

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPorts_b[0] ports_b(
  redeclare package Medium = Medium)
  annotation (Placement(transformation(extent= {{-106,34},{-96,78}} ), iconTransformation(
      extent= {{-106,34},{-96,78}} )), iconTransformation(extent=  {{-106,30},{-92,86}} ));

Modelica.Fluid.Interfaces.FluidPorts_a[0] ports_a(
  redeclare package Medium = Medium)
  annotation ( Placement(transformation(extent= {{-104,-80},{-94,-34}} ), iconTransformation(
      extent= {{-104,-80},{-94,-34}} )),iconTransformation(extent=  {{-108,-92},{-94,-40}} ));
    AixLib.ThermalZones.ReducedOrder.ThermalZone.ThermalZone space_001(
    use_moisture_balance=true,
    ROM(extWallRC(thermCapExt(each der_T(fixed=true))), intWallRC(thermCapInt(
            each der_T(fixed=true)))),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    internalGainsMode=1,
    nPorts=2,  
    zoneParam=AixLib.DataBase.ThermalZones.ZoneBaseRecord(
  T_start=293.15,
    AZone=100.0,
    VAir=250.0
,
    hRad=5,
    lat=0.87266462599716,
    nOrientations=3,
    AWin={ 5.0, 0.0, 2.0 },
    ATransparent={ 5.0, 0.0, 2.0 },
    RWin=0.001,
    hConWin=2.7,
    gWin=1,
    UWin=2.1,
    ratioWinConRad=0.09,

    AExt={ 20.0, 30.0, 50.0 },
    hConExt=2.7,
    nExt=1,
    RExt={ 1.726585424234744 },
    RExtRem=1.726585424234744,
    CExt={ 24971200.0 },

    AInt=0,
    hConInt=2.27,
    nInt=1,
    RInt={1.13047235829e-05},
    CInt={1402628013.98},

    AFloor=120.0,
    hConFloor=2.7,
    nFloor=1,
    RFloor={ 0.08928571428571429 },
    RFloorRem=0.08928571428571429,
    CFloor={ 60480000.0 },

    ARoof=0.0,
    hConRoof=2.7,
    nRoof=1,
    RRoof={ 0.001 },
    RRoofRem=0.001,
    CRoof={ 10000 },
    nOrientationsRoof=1,
    tiltRoof={0},
    aziRoof={0},
    wfRoof={1},
    aRoof=0.7,

    aExt=0.7,
    TSoil=283.15,
    hConWallOut=20.0,
    hRadWall=5,
    hConWinOut=20.0,
    hConRoofOut=20,
    hRadRoof=5,
    tiltExtWalls={ 1.5707963267948966, 1.5707963267948966, 1.5707963267948966 },
    aziExtWalls={ 0.0, 1.5707963267948966, 3.141592653589793 },
    wfWall={ 0.2, 0.2, 0.2 },
    wfWin={ 0.25, 0.25, 0.25 },
    wfGro=0.1,
    specificPeople=1/14,
    activityDegree=1.2,
    fixedHeatFlowRatePersons=70,
    ratioConvectiveHeatPeople=0.5,
    internalGainsMoistureNoPeople=0.5,
    internalGainsMachinesSpecific=7.0,
    ratioConvectiveHeatMachines=0.6,
    lightingPowerSpecific=12.5,
    ratioConvectiveHeatLighting=0.6,
    useConstantACHrate=false,
    baseACH=0.2,
    maxUserACH=1,
    maxOverheatingACH={3.0,2.0},
maxSummerACH={1.0,273.15 + 10,273.15 + 17},
winterReduction={0.2,273.15,273.15 + 10},
    withAHU=false,
    minAHU=0,
    maxAHU=12,
    maxIrr = { 100, 100, 100 },
    shadingFactor = { 0.7, 0.7, 0.7 },
    hHeat=167500,
    lHeat=0,
    KRHeat=1000,
    TNHeat=1,
    HeaterOn=false,
    hCool=0,
    lCool=-1,
    heaLoadFacOut=0,
    heaLoadFacGrd=0,
    KRCool=1000,
    TNCool=1,
    CoolerOn=false,
TThresholdHeater=273.15 + 15,
TThresholdCooler=273.15 + 22,
    withIdealThresholds=false))
 annotation (
    Placement(transformation(origin = { -59.79530884780252, -44.28788556101922 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
    
    
    Modelica.Blocks.Sources.CombiTimeTable occupancy_1(
   extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
   tableName = "UserProfiles",
   columns = {2, 3, 4},
   tableOnFile=false,
   table=[0,0,0.1,0,0; 3540,0,0.1,0,0; 3600,0,0.1,0,0; 7140,0,0.1,0,0; 7200,0,
       0.1,0,0; 10740,0,0.1,0,0; 10800,0,0.1,0,0; 14340,0,0.1,0,0; 14400,0,0.1,
       0,0; 17940,0,0.1,0,0; 18000,0,0.1,0,0; 21540,0,0.1,0,0; 21600,0,0.1,0,0;
       25140,0,0.1,0,0; 25200,0,0.1,0,0; 28740,0,0.1,0,0; 28800,0,0.1,0,0;
       32340,0,0.1,0,0; 32400,0.6,0.6,1,1; 35940,0.6,0.6,1,1; 36000,1,1,1,1;
       39540,1,1,1,1; 39600,0.4,0.4,1,1; 43140,0.4,0.4,1,1; 43200,0,0.1,0,0;
       46740,0,0.1,0,0; 46800,0,0.1,0,0; 50340,0,0.1,0,0; 50400,0.6,0.6,1,1;
       53940,0.6,0.6,1,1; 54000,1,1,1,1; 57540,1,1,1,1; 57600,0.4,0.4,1,1;
       61140,0.4,0.4,1,1; 61200,0,0.1,0,0; 64740,0,0.1,0,0; 64800,0,0.1,0,0;
       68340,0,0.1,0,0; 68400,0,0.1,0,0; 71940,0,0.1,0,0; 72000,0,0.1,0,0;
       75540,0,0.1,0,0; 75600,0,0.1,0,0; 79140,0,0.1,0,0; 79200,0,0.1,0,0;
       82740,0,0.1,0,0; 82800,0,0.1,0,0; 86340,0,0.1,0,0; 86400,0,0.1,0,0;
       89940,0,0.1,0,0; 90000,0,0.1,0,0; 93540,0,0.1,0,0; 93600,0,0.1,0,0;
       97140,0,0.1,0,0; 97200,0,0.1,0,0; 100740,0,0.1,0,0; 100800,0,0.1,0,0;
       104340,0,0.1,0,0; 104400,0,0.1,0,0; 107940,0,0.1,0,0; 108000,0,0.1,0,0;
       111540,0,0.1,0,0; 111600,0,0.1,0,0; 115140,0,0.1,0,0; 115200,0,0.1,0,0;
       118740,0,0.1,0,0; 118800,0.6,0.6,1,1; 122340,0.6,0.6,1,1; 122400,1,1,1,
       1; 125940,1,1,1,1; 126000,0.4,0.4,1,1; 129540,0.4,0.4,1,1; 129600,0,0.1,
       0,0; 133140,0,0.1,0,0; 133200,0,0.1,0,0; 136740,0,0.1,0,0; 136800,0.6,
       0.6,1,1; 140340,0.6,0.6,1,1; 140400,1,1,1,1; 143940,1,1,1,1; 144000,0.4,
       0.4,1,1; 147540,0.4,0.4,1,1; 147600,0,0.1,0,0; 151140,0,0.1,0,0; 151200,
       0,0.1,0,0; 154740,0,0.1,0,0; 154800,0,0.1,0,0; 158340,0,0.1,0,0; 158400,
       0,0.1,0,0; 161940,0,0.1,0,0; 162000,0,0.1,0,0; 165540,0,0.1,0,0; 165600,
       0,0.1,0,0; 169140,0,0.1,0,0; 169200,0,0.1,0,0; 172740,0,0.1,0,0; 172800,
       0,0.1,0,0; 176340,0,0.1,0,0; 176400,0,0.1,0,0; 179940,0,0.1,0,0; 180000,
       0,0.1,0,0; 183540,0,0.1,0,0; 183600,0,0.1,0,0; 187140,0,0.1,0,0; 187200,
       0,0.1,0,0; 190740,0,0.1,0,0; 190800,0,0.1,0,0; 194340,0,0.1,0,0; 194400,
       0,0.1,0,0; 197940,0,0.1,0,0; 198000,0,0.1,0,0; 201540,0,0.1,0,0; 201600,
       0,0.1,0,0; 205140,0,0.1,0,0; 205200,0.6,0.6,1,1; 208740,0.6,0.6,1,1;
       208800,1,1,1,1; 212340,1,1,1,1; 212400,0.4,0.4,1,1; 215940,0.4,0.4,1,1;
       216000,0,0.1,0,0; 219540,0,0.1,0,0; 219600,0,0.1,0,0; 223140,0,0.1,0,0;
       223200,0.6,0.6,1,1; 226740,0.6,0.6,1,1; 226800,1,1,1,1; 230340,1,1,1,1;
       230400,0.4,0.4,1,1; 233940,0.4,0.4,1,1; 234000,0,0.1,0,0; 237540,0,0.1,
       0,0; 237600,0,0.1,0,0; 241140,0,0.1,0,0; 241200,0,0.1,0,0; 244740,0,0.1,
       0,0; 244800,0,0.1,0,0; 248340,0,0.1,0,0; 248400,0,0.1,0,0; 251940,0,0.1,
       0,0; 252000,0,0.1,0,0; 255540,0,0.1,0,0; 255600,0,0.1,0,0; 259140,0,0.1,
       0,0; 259200,0,0.1,0,0; 262740,0,0.1,0,0; 262800,0,0.1,0,0; 266340,0,0.1,
       0,0; 266400,0,0.1,0,0; 269940,0,0.1,0,0; 270000,0,0.1,0,0; 273540,0,0.1,
       0,0; 273600,0,0.1,0,0; 277140,0,0.1,0,0; 277200,0,0.1,0,0; 280740,0,0.1,
       0,0; 280800,0,0.1,0,0; 284340,0,0.1,0,0; 284400,0,0.1,0,0; 287940,0,0.1,
       0,0; 288000,0,0.1,0,0; 291540,0,0.1,0,0; 291600,0.6,0.6,1,1; 295140,0.6,
       0.6,1,1; 295200,1,1,1,1; 298740,1,1,1,1; 298800,0.4,0.4,1,1; 302340,0.4,
       0.4,1,1; 302400,0,0.1,0,0; 305940,0,0.1,0,0; 306000,0,0.1,0,0; 309540,0,
       0.1,0,0; 309600,0.6,0.6,1,1; 313140,0.6,0.6,1,1; 313200,1,1,1,1; 316740,
       1,1,1,1; 316800,0.4,0.4,1,1; 320340,0.4,0.4,1,1; 320400,0,0.1,0,0;
       323940,0,0.1,0,0; 324000,0,0.1,0,0; 327540,0,0.1,0,0; 327600,0,0.1,0,0;
       331140,0,0.1,0,0; 331200,0,0.1,0,0; 334740,0,0.1,0,0; 334800,0,0.1,0,0;
       338340,0,0.1,0,0; 338400,0,0.1,0,0; 341940,0,0.1,0,0; 342000,0,0.1,0,0;
       345540,0,0.1,0,0; 345600,0,0.1,0,0; 349140,0,0.1,0,0; 349200,0,0.1,0,0;
       352740,0,0.1,0,0; 352800,0,0.1,0,0; 356340,0,0.1,0,0; 356400,0,0.1,0,0;
       359940,0,0.1,0,0; 360000,0,0.1,0,0; 363540,0,0.1,0,0; 363600,0,0.1,0,0;
       367140,0,0.1,0,0; 367200,0,0.1,0,0; 370740,0,0.1,0,0; 370800,0,0.1,0,0;
       374340,0,0.1,0,0; 374400,0,0.1,0,0; 377940,0,0.1,0,0; 378000,0.6,0.6,1,
       1; 381540,0.6,0.6,1,1; 381600,1,1,1,1; 385140,1,1,1,1; 385200,0.4,0.4,1,
       1; 388740,0.4,0.4,1,1; 388800,0,0.1,0,0; 392340,0,0.1,0,0; 392400,0,0.1,
       0,0; 395940,0,0.1,0,0; 396000,0.6,0.6,1,1; 399540,0.6,0.6,1,1; 399600,1,
       1,1,1; 403140,1,1,1,1; 403200,0.4,0.4,1,1; 406740,0.4,0.4,1,1; 406800,0,
       0.1,0,0; 410340,0,0.1,0,0; 410400,0,0.1,0,0; 413940,0,0.1,0,0; 414000,0,
       0.1,0,0; 417540,0,0.1,0,0; 417600,0,0.1,0,0; 421140,0,0.1,0,0; 421200,0,
       0.1,0,0; 424740,0,0.1,0,0; 424800,0,0.1,0,0; 428340,0,0.1,0,0; 428400,0,
       0.1,0,0; 431940,0,0.1,0,0; 432000,0,0,0,0; 435540,0,0,0,0; 435600,0,0,0,
       0; 439140,0,0,0,0; 439200,0,0,0,0; 442740,0,0,0,0; 442800,0,0,0,0;
       446340,0,0,0,0; 446400,0,0,0,0; 449940,0,0,0,0; 450000,0,0,0,0; 453540,
       0,0,0,0; 453600,0,0,0,0; 457140,0,0,0,0; 457200,0,0,0,0; 460740,0,0,0,0;
       460800,0,0,0,0; 464340,0,0,0,0; 464400,0,0,0,0; 467940,0,0,0,0; 468000,
       0,0,0,0; 471540,0,0,0,0; 471600,0,0,0,0; 475140,0,0,0,0; 475200,0,0,0,0;
       478740,0,0,0,0; 478800,0,0,0,0; 482340,0,0,0,0; 482400,0,0,0,0; 485940,
       0,0,0,0; 486000,0,0,0,0; 489540,0,0,0,0; 489600,0,0,0,0; 493140,0,0,0,0;
       493200,0,0,0,0; 496740,0,0,0,0; 496800,0,0,0,0; 500340,0,0,0,0; 500400,
       0,0,0,0; 503940,0,0,0,0; 504000,0,0,0,0; 507540,0,0,0,0; 507600,0,0,0,0;
       511140,0,0,0,0; 511200,0,0,0,0; 514740,0,0,0,0; 514800,0,0,0,0; 518340,
       0,0,0,0; 518400,0,0,0,0; 521940,0,0,0,0; 522000,0,0,0,0; 525540,0,0,0,0;
       525600,0,0,0,0; 529140,0,0,0,0; 529200,0,0,0,0; 532740,0,0,0,0; 532800,
       0,0,0,0; 536340,0,0,0,0; 536400,0,0,0,0; 539940,0,0,0,0; 540000,0,0,0,0;
       543540,0,0,0,0; 543600,0,0,0,0; 547140,0,0,0,0; 547200,0,0,0,0; 550740,
       0,0,0,0; 550800,0,0,0,0; 554340,0,0,0,0; 554400,0,0,0,0; 557940,0,0,0,0;
       558000,0,0,0,0; 561540,0,0,0,0; 561600,0,0,0,0; 565140,0,0,0,0; 565200,
       0,0,0,0; 568740,0,0,0,0; 568800,0,0,0,0; 572340,0,0,0,0; 572400,0,0,0,0;
       575940,0,0,0,0; 576000,0,0,0,0; 579540,0,0,0,0; 579600,0,0,0,0; 583140,
       0,0,0,0; 583200,0,0,0,0; 586740,0,0,0,0; 586800,0,0,0,0; 590340,0,0,0,0;
       590400,0,0,0,0; 593940,0,0,0,0; 594000,0,0,0,0; 597540,0,0,0,0; 597600,
       0,0,0,0; 601140,0,0,0,0; 601200,0,0,0,0; 604740,0,0,0,0])
 annotation (
    Placement(transformation(origin = { -74.79530884780252, -44.28788556101922 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    AixLib.ThermalZones.ReducedOrder.ThermalZone.ThermalZone space_002(
    use_moisture_balance=true,
    ROM(extWallRC(thermCapExt(each der_T(fixed=true))), intWallRC(thermCapInt(
            each der_T(fixed=true)))),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    internalGainsMode=1,
    nPorts=2,  
    zoneParam=AixLib.DataBase.ThermalZones.ZoneBaseRecord(
  T_start=293.15,
    AZone=70.0,
    VAir=175.0
,
    hRad=5,
    lat=0.87266462599716,
    nOrientations=3,
    AWin={ 5.0, 0.0, 2.0 },
    ATransparent={ 5.0, 0.0, 2.0 },
    RWin=0.001,
    hConWin=2.7,
    gWin=1,
    UWin=2.1,
    ratioWinConRad=0.09,

    AExt={ 25.0, 25.0, 34.0 },
    hConExt=2.7,
    nExt=1,
    RExt={ 2.3125229242347443 },
    RExtRem=2.3125229242347443,
    CExt={ 20978850.144 },

    AInt=0,
    hConInt=2.27,
    nInt=1,
    RInt={1.13047235829e-05},
    CInt={1402628013.98},

    AFloor=60.0,
    hConFloor=2.7,
    nFloor=1,
    RFloor={ 0.08928571428571429 },
    RFloorRem=0.08928571428571429,
    CFloor={ 30240000.0 },

    ARoof=0.0,
    hConRoof=2.7,
    nRoof=1,
    RRoof={ 0.001 },
    RRoofRem=0.001,
    CRoof={ 10000 },
    nOrientationsRoof=1,
    tiltRoof={0},
    aziRoof={0},
    wfRoof={1},
    aRoof=0.7,

    aExt=0.7,
    TSoil=283.15,
    hConWallOut=20.0,
    hRadWall=5,
    hConWinOut=20.0,
    hConRoofOut=20,
    hRadRoof=5,
    tiltExtWalls={ 1.5707963267948966, 1.5707963267948966, 1.5707963267948966 },
    aziExtWalls={ 0.0, 1.5707963267948966, 3.141592653589793 },
    wfWall={ 0.2, 0.2, 0.2 },
    wfWin={ 0.25, 0.25, 0.25 },
    wfGro=0.1,
    specificPeople=1/14,
    activityDegree=1.2,
    fixedHeatFlowRatePersons=70,
    ratioConvectiveHeatPeople=0.5,
    internalGainsMoistureNoPeople=0.5,
    internalGainsMachinesSpecific=7.0,
    ratioConvectiveHeatMachines=0.6,
    lightingPowerSpecific=12.5,
    ratioConvectiveHeatLighting=0.6,
    useConstantACHrate=false,
    baseACH=0.2,
    maxUserACH=1,
    maxOverheatingACH={3.0,2.0},
maxSummerACH={1.0,273.15 + 10,273.15 + 17},
winterReduction={0.2,273.15,273.15 + 10},
    withAHU=false,
    minAHU=0,
    maxAHU=12,
    maxIrr = { 100, 100, 100 },
    shadingFactor = { 0.7, 0.7, 0.7 },
    hHeat=167500,
    lHeat=0,
    KRHeat=1000,
    TNHeat=1,
    HeaterOn=false,
    hCool=0,
    lCool=-1,
    heaLoadFacOut=0,
    heaLoadFacGrd=0,
    KRCool=1000,
    TNCool=1,
    CoolerOn=false,
TThresholdHeater=273.15 + 15,
TThresholdCooler=273.15 + 22,
    withIdealThresholds=false))
 annotation (
    Placement(transformation(origin = { -7.52647747146564, 42.2637460885114 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
    
    
    Modelica.Blocks.Sources.CombiTimeTable occupancy_2(
   extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
   tableName = "UserProfiles",
   columns = {2, 3, 4},
   tableOnFile=false,
   table=[0,0,0.1,0,0; 3540,0,0.1,0,0; 3600,0,0.1,0,0; 7140,0,0.1,0,0; 7200,0,
       0.1,0,0; 10740,0,0.1,0,0; 10800,0,0.1,0,0; 14340,0,0.1,0,0; 14400,0,0.1,
       0,0; 17940,0,0.1,0,0; 18000,0,0.1,0,0; 21540,0,0.1,0,0; 21600,0,0.1,0,0;
       25140,0,0.1,0,0; 25200,0,0.1,0,0; 28740,0,0.1,0,0; 28800,0,0.1,0,0;
       32340,0,0.1,0,0; 32400,0.6,0.6,1,1; 35940,0.6,0.6,1,1; 36000,1,1,1,1;
       39540,1,1,1,1; 39600,0.4,0.4,1,1; 43140,0.4,0.4,1,1; 43200,0,0.1,0,0;
       46740,0,0.1,0,0; 46800,0,0.1,0,0; 50340,0,0.1,0,0; 50400,0.6,0.6,1,1;
       53940,0.6,0.6,1,1; 54000,1,1,1,1; 57540,1,1,1,1; 57600,0.4,0.4,1,1;
       61140,0.4,0.4,1,1; 61200,0,0.1,0,0; 64740,0,0.1,0,0; 64800,0,0.1,0,0;
       68340,0,0.1,0,0; 68400,0,0.1,0,0; 71940,0,0.1,0,0; 72000,0,0.1,0,0;
       75540,0,0.1,0,0; 75600,0,0.1,0,0; 79140,0,0.1,0,0; 79200,0,0.1,0,0;
       82740,0,0.1,0,0; 82800,0,0.1,0,0; 86340,0,0.1,0,0; 86400,0,0.1,0,0;
       89940,0,0.1,0,0; 90000,0,0.1,0,0; 93540,0,0.1,0,0; 93600,0,0.1,0,0;
       97140,0,0.1,0,0; 97200,0,0.1,0,0; 100740,0,0.1,0,0; 100800,0,0.1,0,0;
       104340,0,0.1,0,0; 104400,0,0.1,0,0; 107940,0,0.1,0,0; 108000,0,0.1,0,0;
       111540,0,0.1,0,0; 111600,0,0.1,0,0; 115140,0,0.1,0,0; 115200,0,0.1,0,0;
       118740,0,0.1,0,0; 118800,0.6,0.6,1,1; 122340,0.6,0.6,1,1; 122400,1,1,1,
       1; 125940,1,1,1,1; 126000,0.4,0.4,1,1; 129540,0.4,0.4,1,1; 129600,0,0.1,
       0,0; 133140,0,0.1,0,0; 133200,0,0.1,0,0; 136740,0,0.1,0,0; 136800,0.6,
       0.6,1,1; 140340,0.6,0.6,1,1; 140400,1,1,1,1; 143940,1,1,1,1; 144000,0.4,
       0.4,1,1; 147540,0.4,0.4,1,1; 147600,0,0.1,0,0; 151140,0,0.1,0,0; 151200,
       0,0.1,0,0; 154740,0,0.1,0,0; 154800,0,0.1,0,0; 158340,0,0.1,0,0; 158400,
       0,0.1,0,0; 161940,0,0.1,0,0; 162000,0,0.1,0,0; 165540,0,0.1,0,0; 165600,
       0,0.1,0,0; 169140,0,0.1,0,0; 169200,0,0.1,0,0; 172740,0,0.1,0,0; 172800,
       0,0.1,0,0; 176340,0,0.1,0,0; 176400,0,0.1,0,0; 179940,0,0.1,0,0; 180000,
       0,0.1,0,0; 183540,0,0.1,0,0; 183600,0,0.1,0,0; 187140,0,0.1,0,0; 187200,
       0,0.1,0,0; 190740,0,0.1,0,0; 190800,0,0.1,0,0; 194340,0,0.1,0,0; 194400,
       0,0.1,0,0; 197940,0,0.1,0,0; 198000,0,0.1,0,0; 201540,0,0.1,0,0; 201600,
       0,0.1,0,0; 205140,0,0.1,0,0; 205200,0.6,0.6,1,1; 208740,0.6,0.6,1,1;
       208800,1,1,1,1; 212340,1,1,1,1; 212400,0.4,0.4,1,1; 215940,0.4,0.4,1,1;
       216000,0,0.1,0,0; 219540,0,0.1,0,0; 219600,0,0.1,0,0; 223140,0,0.1,0,0;
       223200,0.6,0.6,1,1; 226740,0.6,0.6,1,1; 226800,1,1,1,1; 230340,1,1,1,1;
       230400,0.4,0.4,1,1; 233940,0.4,0.4,1,1; 234000,0,0.1,0,0; 237540,0,0.1,
       0,0; 237600,0,0.1,0,0; 241140,0,0.1,0,0; 241200,0,0.1,0,0; 244740,0,0.1,
       0,0; 244800,0,0.1,0,0; 248340,0,0.1,0,0; 248400,0,0.1,0,0; 251940,0,0.1,
       0,0; 252000,0,0.1,0,0; 255540,0,0.1,0,0; 255600,0,0.1,0,0; 259140,0,0.1,
       0,0; 259200,0,0.1,0,0; 262740,0,0.1,0,0; 262800,0,0.1,0,0; 266340,0,0.1,
       0,0; 266400,0,0.1,0,0; 269940,0,0.1,0,0; 270000,0,0.1,0,0; 273540,0,0.1,
       0,0; 273600,0,0.1,0,0; 277140,0,0.1,0,0; 277200,0,0.1,0,0; 280740,0,0.1,
       0,0; 280800,0,0.1,0,0; 284340,0,0.1,0,0; 284400,0,0.1,0,0; 287940,0,0.1,
       0,0; 288000,0,0.1,0,0; 291540,0,0.1,0,0; 291600,0.6,0.6,1,1; 295140,0.6,
       0.6,1,1; 295200,1,1,1,1; 298740,1,1,1,1; 298800,0.4,0.4,1,1; 302340,0.4,
       0.4,1,1; 302400,0,0.1,0,0; 305940,0,0.1,0,0; 306000,0,0.1,0,0; 309540,0,
       0.1,0,0; 309600,0.6,0.6,1,1; 313140,0.6,0.6,1,1; 313200,1,1,1,1; 316740,
       1,1,1,1; 316800,0.4,0.4,1,1; 320340,0.4,0.4,1,1; 320400,0,0.1,0,0;
       323940,0,0.1,0,0; 324000,0,0.1,0,0; 327540,0,0.1,0,0; 327600,0,0.1,0,0;
       331140,0,0.1,0,0; 331200,0,0.1,0,0; 334740,0,0.1,0,0; 334800,0,0.1,0,0;
       338340,0,0.1,0,0; 338400,0,0.1,0,0; 341940,0,0.1,0,0; 342000,0,0.1,0,0;
       345540,0,0.1,0,0; 345600,0,0.1,0,0; 349140,0,0.1,0,0; 349200,0,0.1,0,0;
       352740,0,0.1,0,0; 352800,0,0.1,0,0; 356340,0,0.1,0,0; 356400,0,0.1,0,0;
       359940,0,0.1,0,0; 360000,0,0.1,0,0; 363540,0,0.1,0,0; 363600,0,0.1,0,0;
       367140,0,0.1,0,0; 367200,0,0.1,0,0; 370740,0,0.1,0,0; 370800,0,0.1,0,0;
       374340,0,0.1,0,0; 374400,0,0.1,0,0; 377940,0,0.1,0,0; 378000,0.6,0.6,1,
       1; 381540,0.6,0.6,1,1; 381600,1,1,1,1; 385140,1,1,1,1; 385200,0.4,0.4,1,
       1; 388740,0.4,0.4,1,1; 388800,0,0.1,0,0; 392340,0,0.1,0,0; 392400,0,0.1,
       0,0; 395940,0,0.1,0,0; 396000,0.6,0.6,1,1; 399540,0.6,0.6,1,1; 399600,1,
       1,1,1; 403140,1,1,1,1; 403200,0.4,0.4,1,1; 406740,0.4,0.4,1,1; 406800,0,
       0.1,0,0; 410340,0,0.1,0,0; 410400,0,0.1,0,0; 413940,0,0.1,0,0; 414000,0,
       0.1,0,0; 417540,0,0.1,0,0; 417600,0,0.1,0,0; 421140,0,0.1,0,0; 421200,0,
       0.1,0,0; 424740,0,0.1,0,0; 424800,0,0.1,0,0; 428340,0,0.1,0,0; 428400,0,
       0.1,0,0; 431940,0,0.1,0,0; 432000,0,0,0,0; 435540,0,0,0,0; 435600,0,0,0,
       0; 439140,0,0,0,0; 439200,0,0,0,0; 442740,0,0,0,0; 442800,0,0,0,0;
       446340,0,0,0,0; 446400,0,0,0,0; 449940,0,0,0,0; 450000,0,0,0,0; 453540,
       0,0,0,0; 453600,0,0,0,0; 457140,0,0,0,0; 457200,0,0,0,0; 460740,0,0,0,0;
       460800,0,0,0,0; 464340,0,0,0,0; 464400,0,0,0,0; 467940,0,0,0,0; 468000,
       0,0,0,0; 471540,0,0,0,0; 471600,0,0,0,0; 475140,0,0,0,0; 475200,0,0,0,0;
       478740,0,0,0,0; 478800,0,0,0,0; 482340,0,0,0,0; 482400,0,0,0,0; 485940,
       0,0,0,0; 486000,0,0,0,0; 489540,0,0,0,0; 489600,0,0,0,0; 493140,0,0,0,0;
       493200,0,0,0,0; 496740,0,0,0,0; 496800,0,0,0,0; 500340,0,0,0,0; 500400,
       0,0,0,0; 503940,0,0,0,0; 504000,0,0,0,0; 507540,0,0,0,0; 507600,0,0,0,0;
       511140,0,0,0,0; 511200,0,0,0,0; 514740,0,0,0,0; 514800,0,0,0,0; 518340,
       0,0,0,0; 518400,0,0,0,0; 521940,0,0,0,0; 522000,0,0,0,0; 525540,0,0,0,0;
       525600,0,0,0,0; 529140,0,0,0,0; 529200,0,0,0,0; 532740,0,0,0,0; 532800,
       0,0,0,0; 536340,0,0,0,0; 536400,0,0,0,0; 539940,0,0,0,0; 540000,0,0,0,0;
       543540,0,0,0,0; 543600,0,0,0,0; 547140,0,0,0,0; 547200,0,0,0,0; 550740,
       0,0,0,0; 550800,0,0,0,0; 554340,0,0,0,0; 554400,0,0,0,0; 557940,0,0,0,0;
       558000,0,0,0,0; 561540,0,0,0,0; 561600,0,0,0,0; 565140,0,0,0,0; 565200,
       0,0,0,0; 568740,0,0,0,0; 568800,0,0,0,0; 572340,0,0,0,0; 572400,0,0,0,0;
       575940,0,0,0,0; 576000,0,0,0,0; 579540,0,0,0,0; 579600,0,0,0,0; 583140,
       0,0,0,0; 583200,0,0,0,0; 586740,0,0,0,0; 586800,0,0,0,0; 590340,0,0,0,0;
       590400,0,0,0,0; 593940,0,0,0,0; 594000,0,0,0,0; 597540,0,0,0,0; 597600,
       0,0,0,0; 601140,0,0,0,0; 601200,0,0,0,0; 604740,0,0,0,0])
 annotation (
    Placement(transformation(origin = { -22.52647747146564, 42.2637460885114 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    AixLib.ThermalZones.ReducedOrder.ThermalZone.ThermalZone space_003(
    use_moisture_balance=true,
    ROM(extWallRC(thermCapExt(each der_T(fixed=true))), intWallRC(thermCapInt(
            each der_T(fixed=true)))),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    internalGainsMode=1,
    nPorts=2,  
    zoneParam=AixLib.DataBase.ThermalZones.ZoneBaseRecord(
  T_start=293.15,
    AZone=50.0,
    VAir=125.0
,
    hRad=5,
    lat=0.87266462599716,
    nOrientations=3,
    AWin={ 5.0, 5.0, 5.0 },
    ATransparent={ 5.0, 5.0, 5.0 },
    RWin=0.001,
    hConWin=2.7,
    gWin=1,
    UWin=2.1,
    ratioWinConRad=0.09,

    AExt={ 22.0, 17.0, 36.0 },
    hConExt=2.7,
    nExt=1,
    RExt={ 4.285714285714286 },
    RExtRem=4.285714285714286,
    CExt={ 45000000.0 },

    AInt=0,
    hConInt=2.27,
    nInt=1,
    RInt={1.13047235829e-05},
    CInt={1402628013.98},

    AFloor=60.0,
    hConFloor=2.7,
    nFloor=1,
    RFloor={ 0.08928571428571429 },
    RFloorRem=0.08928571428571429,
    CFloor={ 30240000.0 },

    ARoof=0.0,
    hConRoof=2.7,
    nRoof=1,
    RRoof={ 0.001 },
    RRoofRem=0.001,
    CRoof={ 10000 },
    nOrientationsRoof=1,
    tiltRoof={0},
    aziRoof={0},
    wfRoof={1},
    aRoof=0.7,

    aExt=0.7,
    TSoil=283.15,
    hConWallOut=20.0,
    hRadWall=5,
    hConWinOut=20.0,
    hConRoofOut=20,
    hRadRoof=5,
    tiltExtWalls={ 1.5707963267948966, 1.5707963267948966, 1.5707963267948966 },
    aziExtWalls={ 3.141592653589793, 3.141592653589793, 3.141592653589793 },
    wfWall={ 0.2, 0.2, 0.2 },
    wfWin={ 0.25, 0.25, 0.25 },
    wfGro=0.1,
    specificPeople=1/14,
    activityDegree=1.2,
    fixedHeatFlowRatePersons=70,
    ratioConvectiveHeatPeople=0.5,
    internalGainsMoistureNoPeople=0.5,
    internalGainsMachinesSpecific=7.0,
    ratioConvectiveHeatMachines=0.6,
    lightingPowerSpecific=12.5,
    ratioConvectiveHeatLighting=0.6,
    useConstantACHrate=false,
    baseACH=0.2,
    maxUserACH=1,
    maxOverheatingACH={3.0,2.0},
maxSummerACH={1.0,273.15 + 10,273.15 + 17},
winterReduction={0.2,273.15,273.15 + 10},
    withAHU=false,
    minAHU=0,
    maxAHU=12,
    maxIrr = { 100, 100, 100 },
    shadingFactor = { 0.7, 0.7, 0.7 },
    hHeat=167500,
    lHeat=0,
    KRHeat=1000,
    TNHeat=1,
    HeaterOn=false,
    hCool=0,
    lCool=-1,
    heaLoadFacOut=0,
    heaLoadFacGrd=0,
    KRCool=1000,
    TNCool=1,
    CoolerOn=false,
TThresholdHeater=273.15 + 15,
TThresholdCooler=273.15 + 22,
    withIdealThresholds=false))
 annotation (
    Placement(transformation(origin = { 55.75339603530185, -47.45641484130532 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
    
    Modelica.Blocks.Sources.CombiTimeTable occupancy_3(
   extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic,
   tableName = "UserProfiles",
   columns = {2, 3, 4},
   tableOnFile=false,
   table=[0,0,0.1,0,0; 3540,0,0.1,0,0; 3600,0,0.1,0,0; 7140,0,0.1,0,0; 7200,0,
       0.1,0,0; 10740,0,0.1,0,0; 10800,0,0.1,0,0; 14340,0,0.1,0,0; 14400,0,0.1,
       0,0; 17940,0,0.1,0,0; 18000,0,0.1,0,0; 21540,0,0.1,0,0; 21600,0,0.1,0,0;
       25140,0,0.1,0,0; 25200,0,0.1,0,0; 28740,0,0.1,0,0; 28800,0,0.1,0,0;
       32340,0,0.1,0,0; 32400,0.6,0.6,1,1; 35940,0.6,0.6,1,1; 36000,1,1,1,1;
       39540,1,1,1,1; 39600,0.4,0.4,1,1; 43140,0.4,0.4,1,1; 43200,0,0.1,0,0;
       46740,0,0.1,0,0; 46800,0,0.1,0,0; 50340,0,0.1,0,0; 50400,0.6,0.6,1,1;
       53940,0.6,0.6,1,1; 54000,1,1,1,1; 57540,1,1,1,1; 57600,0.4,0.4,1,1;
       61140,0.4,0.4,1,1; 61200,0,0.1,0,0; 64740,0,0.1,0,0; 64800,0,0.1,0,0;
       68340,0,0.1,0,0; 68400,0,0.1,0,0; 71940,0,0.1,0,0; 72000,0,0.1,0,0;
       75540,0,0.1,0,0; 75600,0,0.1,0,0; 79140,0,0.1,0,0; 79200,0,0.1,0,0;
       82740,0,0.1,0,0; 82800,0,0.1,0,0; 86340,0,0.1,0,0; 86400,0,0.1,0,0;
       89940,0,0.1,0,0; 90000,0,0.1,0,0; 93540,0,0.1,0,0; 93600,0,0.1,0,0;
       97140,0,0.1,0,0; 97200,0,0.1,0,0; 100740,0,0.1,0,0; 100800,0,0.1,0,0;
       104340,0,0.1,0,0; 104400,0,0.1,0,0; 107940,0,0.1,0,0; 108000,0,0.1,0,0;
       111540,0,0.1,0,0; 111600,0,0.1,0,0; 115140,0,0.1,0,0; 115200,0,0.1,0,0;
       118740,0,0.1,0,0; 118800,0.6,0.6,1,1; 122340,0.6,0.6,1,1; 122400,1,1,1,
       1; 125940,1,1,1,1; 126000,0.4,0.4,1,1; 129540,0.4,0.4,1,1; 129600,0,0.1,
       0,0; 133140,0,0.1,0,0; 133200,0,0.1,0,0; 136740,0,0.1,0,0; 136800,0.6,
       0.6,1,1; 140340,0.6,0.6,1,1; 140400,1,1,1,1; 143940,1,1,1,1; 144000,0.4,
       0.4,1,1; 147540,0.4,0.4,1,1; 147600,0,0.1,0,0; 151140,0,0.1,0,0; 151200,
       0,0.1,0,0; 154740,0,0.1,0,0; 154800,0,0.1,0,0; 158340,0,0.1,0,0; 158400,
       0,0.1,0,0; 161940,0,0.1,0,0; 162000,0,0.1,0,0; 165540,0,0.1,0,0; 165600,
       0,0.1,0,0; 169140,0,0.1,0,0; 169200,0,0.1,0,0; 172740,0,0.1,0,0; 172800,
       0,0.1,0,0; 176340,0,0.1,0,0; 176400,0,0.1,0,0; 179940,0,0.1,0,0; 180000,
       0,0.1,0,0; 183540,0,0.1,0,0; 183600,0,0.1,0,0; 187140,0,0.1,0,0; 187200,
       0,0.1,0,0; 190740,0,0.1,0,0; 190800,0,0.1,0,0; 194340,0,0.1,0,0; 194400,
       0,0.1,0,0; 197940,0,0.1,0,0; 198000,0,0.1,0,0; 201540,0,0.1,0,0; 201600,
       0,0.1,0,0; 205140,0,0.1,0,0; 205200,0.6,0.6,1,1; 208740,0.6,0.6,1,1;
       208800,1,1,1,1; 212340,1,1,1,1; 212400,0.4,0.4,1,1; 215940,0.4,0.4,1,1;
       216000,0,0.1,0,0; 219540,0,0.1,0,0; 219600,0,0.1,0,0; 223140,0,0.1,0,0;
       223200,0.6,0.6,1,1; 226740,0.6,0.6,1,1; 226800,1,1,1,1; 230340,1,1,1,1;
       230400,0.4,0.4,1,1; 233940,0.4,0.4,1,1; 234000,0,0.1,0,0; 237540,0,0.1,
       0,0; 237600,0,0.1,0,0; 241140,0,0.1,0,0; 241200,0,0.1,0,0; 244740,0,0.1,
       0,0; 244800,0,0.1,0,0; 248340,0,0.1,0,0; 248400,0,0.1,0,0; 251940,0,0.1,
       0,0; 252000,0,0.1,0,0; 255540,0,0.1,0,0; 255600,0,0.1,0,0; 259140,0,0.1,
       0,0; 259200,0,0.1,0,0; 262740,0,0.1,0,0; 262800,0,0.1,0,0; 266340,0,0.1,
       0,0; 266400,0,0.1,0,0; 269940,0,0.1,0,0; 270000,0,0.1,0,0; 273540,0,0.1,
       0,0; 273600,0,0.1,0,0; 277140,0,0.1,0,0; 277200,0,0.1,0,0; 280740,0,0.1,
       0,0; 280800,0,0.1,0,0; 284340,0,0.1,0,0; 284400,0,0.1,0,0; 287940,0,0.1,
       0,0; 288000,0,0.1,0,0; 291540,0,0.1,0,0; 291600,0.6,0.6,1,1; 295140,0.6,
       0.6,1,1; 295200,1,1,1,1; 298740,1,1,1,1; 298800,0.4,0.4,1,1; 302340,0.4,
       0.4,1,1; 302400,0,0.1,0,0; 305940,0,0.1,0,0; 306000,0,0.1,0,0; 309540,0,
       0.1,0,0; 309600,0.6,0.6,1,1; 313140,0.6,0.6,1,1; 313200,1,1,1,1; 316740,
       1,1,1,1; 316800,0.4,0.4,1,1; 320340,0.4,0.4,1,1; 320400,0,0.1,0,0;
       323940,0,0.1,0,0; 324000,0,0.1,0,0; 327540,0,0.1,0,0; 327600,0,0.1,0,0;
       331140,0,0.1,0,0; 331200,0,0.1,0,0; 334740,0,0.1,0,0; 334800,0,0.1,0,0;
       338340,0,0.1,0,0; 338400,0,0.1,0,0; 341940,0,0.1,0,0; 342000,0,0.1,0,0;
       345540,0,0.1,0,0; 345600,0,0.1,0,0; 349140,0,0.1,0,0; 349200,0,0.1,0,0;
       352740,0,0.1,0,0; 352800,0,0.1,0,0; 356340,0,0.1,0,0; 356400,0,0.1,0,0;
       359940,0,0.1,0,0; 360000,0,0.1,0,0; 363540,0,0.1,0,0; 363600,0,0.1,0,0;
       367140,0,0.1,0,0; 367200,0,0.1,0,0; 370740,0,0.1,0,0; 370800,0,0.1,0,0;
       374340,0,0.1,0,0; 374400,0,0.1,0,0; 377940,0,0.1,0,0; 378000,0.6,0.6,1,
       1; 381540,0.6,0.6,1,1; 381600,1,1,1,1; 385140,1,1,1,1; 385200,0.4,0.4,1,
       1; 388740,0.4,0.4,1,1; 388800,0,0.1,0,0; 392340,0,0.1,0,0; 392400,0,0.1,
       0,0; 395940,0,0.1,0,0; 396000,0.6,0.6,1,1; 399540,0.6,0.6,1,1; 399600,1,
       1,1,1; 403140,1,1,1,1; 403200,0.4,0.4,1,1; 406740,0.4,0.4,1,1; 406800,0,
       0.1,0,0; 410340,0,0.1,0,0; 410400,0,0.1,0,0; 413940,0,0.1,0,0; 414000,0,
       0.1,0,0; 417540,0,0.1,0,0; 417600,0,0.1,0,0; 421140,0,0.1,0,0; 421200,0,
       0.1,0,0; 424740,0,0.1,0,0; 424800,0,0.1,0,0; 428340,0,0.1,0,0; 428400,0,
       0.1,0,0; 431940,0,0.1,0,0; 432000,0,0,0,0; 435540,0,0,0,0; 435600,0,0,0,
       0; 439140,0,0,0,0; 439200,0,0,0,0; 442740,0,0,0,0; 442800,0,0,0,0;
       446340,0,0,0,0; 446400,0,0,0,0; 449940,0,0,0,0; 450000,0,0,0,0; 453540,
       0,0,0,0; 453600,0,0,0,0; 457140,0,0,0,0; 457200,0,0,0,0; 460740,0,0,0,0;
       460800,0,0,0,0; 464340,0,0,0,0; 464400,0,0,0,0; 467940,0,0,0,0; 468000,
       0,0,0,0; 471540,0,0,0,0; 471600,0,0,0,0; 475140,0,0,0,0; 475200,0,0,0,0;
       478740,0,0,0,0; 478800,0,0,0,0; 482340,0,0,0,0; 482400,0,0,0,0; 485940,
       0,0,0,0; 486000,0,0,0,0; 489540,0,0,0,0; 489600,0,0,0,0; 493140,0,0,0,0;
       493200,0,0,0,0; 496740,0,0,0,0; 496800,0,0,0,0; 500340,0,0,0,0; 500400,
       0,0,0,0; 503940,0,0,0,0; 504000,0,0,0,0; 507540,0,0,0,0; 507600,0,0,0,0;
       511140,0,0,0,0; 511200,0,0,0,0; 514740,0,0,0,0; 514800,0,0,0,0; 518340,
       0,0,0,0; 518400,0,0,0,0; 521940,0,0,0,0; 522000,0,0,0,0; 525540,0,0,0,0;
       525600,0,0,0,0; 529140,0,0,0,0; 529200,0,0,0,0; 532740,0,0,0,0; 532800,
       0,0,0,0; 536340,0,0,0,0; 536400,0,0,0,0; 539940,0,0,0,0; 540000,0,0,0,0;
       543540,0,0,0,0; 543600,0,0,0,0; 547140,0,0,0,0; 547200,0,0,0,0; 550740,
       0,0,0,0; 550800,0,0,0,0; 554340,0,0,0,0; 554400,0,0,0,0; 557940,0,0,0,0;
       558000,0,0,0,0; 561540,0,0,0,0; 561600,0,0,0,0; 565140,0,0,0,0; 565200,
       0,0,0,0; 568740,0,0,0,0; 568800,0,0,0,0; 572340,0,0,0,0; 572400,0,0,0,0;
       575940,0,0,0,0; 576000,0,0,0,0; 579540,0,0,0,0; 579600,0,0,0,0; 583140,
       0,0,0,0; 583200,0,0,0,0; 586740,0,0,0,0; 586800,0,0,0,0; 590340,0,0,0,0;
       590400,0,0,0,0; 593940,0,0,0,0; 594000,0,0,0,0; 597540,0,0,0,0; 597600,
       0,0,0,0; 601140,0,0,0,0; 601200,0,0,0,0; 604740,0,0,0,0])
 annotation (
    Placement(transformation(origin = { 40.75339603530185, -47.45641484130532 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    AixLib.BoundaryConditions.WeatherData.ReaderTMY3
weather (
    filNam=Modelica.Utilities.Files.loadResource(
        "modelica://AixLib/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"
    )
)
 annotation (
    Placement(transformation(origin = { -0.9762449237505137, -25.29459097004917 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
equation        
        connect(space_001.intGainsConv,heatPortCon[1])
        annotation (Line(
        points={{ -59.79530884780252, -44.28788556101922 }    ,{ -29.89765442390126, -44.28788556101922 }    ,{ -29.89765442390126, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_001.intGainsRad,heatPortRad[1])
        annotation (Line(
        points={{ -59.79530884780252, -44.28788556101922 }    ,{ -29.89765442390126, -44.28788556101922 }    ,{ -29.89765442390126, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_001.intGains,occupancy_1.y)
        annotation (Line(
        points={{ -59.79530884780252, -44.28788556101922 }    ,{ -67.29530884780252, -44.28788556101922 }    ,{ -67.29530884780252, -44.28788556101922 }    ,{ -74.79530884780252, -44.28788556101922 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_001.weaBus,weather.weaBus)
        annotation (Line(
        points={{ -59.79530884780252, -44.28788556101922 }    ,{ -30.385776885776515, -44.28788556101922 }    ,{ -30.385776885776515, -25.29459097004917 }    ,{ -0.9762449237505137, -25.29459097004917 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_002.intGainsConv,heatPortCon[2])
        annotation (Line(
        points={{ -7.52647747146564, 42.2637460885114 }    ,{ -3.76323873573282, 42.2637460885114 }    ,{ -3.76323873573282, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_002.intGainsRad,heatPortRad[2])
        annotation (Line(
        points={{ -7.52647747146564, 42.2637460885114 }    ,{ -3.76323873573282, 42.2637460885114 }    ,{ -3.76323873573282, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_002.intGains,occupancy_2.y)
        annotation (Line(
        points={{ -7.52647747146564, 42.2637460885114 }    ,{ -15.02647747146564, 42.2637460885114 }    ,{ -15.02647747146564, 42.2637460885114 }    ,{ -22.52647747146564, 42.2637460885114 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_002.weaBus,weather.weaBus)
        annotation (Line(
        points={{ -7.52647747146564, 42.2637460885114 }    ,{ -4.251361197608077, 42.2637460885114 }    ,{ -4.251361197608077, -25.29459097004917 }    ,{ -0.9762449237505137, -25.29459097004917 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_003.intGainsConv,heatPortCon[3])
        annotation (Line(
        points={{ 55.75339603530185, -47.45641484130532 }    ,{ 27.876698017650924, -47.45641484130532 }    ,{ 27.876698017650924, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_003.intGainsRad,heatPortRad[3])
        annotation (Line(
        points={{ 55.75339603530185, -47.45641484130532 }    ,{ 27.876698017650924, -47.45641484130532 }    ,{ 27.876698017650924, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_003.intGains,occupancy_3.y)
        annotation (Line(
        points={{ 55.75339603530185, -47.45641484130532 }    ,{ 48.25339603530185, -47.45641484130532 }    ,{ 48.25339603530185, -47.45641484130532 }    ,{ 40.75339603530185, -47.45641484130532 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_003.weaBus,weather.weaBus)
        annotation (Line(
        points={{ 55.75339603530185, -47.45641484130532 }    ,{ 27.388575555775667, -47.45641484130532 }    ,{ 27.388575555775667, -25.29459097004917 }    ,{ -0.9762449237505137, -25.29459097004917 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_001.TAir,y[1])
            ;        
        connect(space_002.TAir,y[2])
            ;        
        connect(space_003.TAir,y[3])
            ;end envelope;
    model production


extends Trano.BaseClasses.Containers.production;
// Define Parameter
parameter Real mRad_flow_nominal = 123;

// Define Medium Package
package MediumW = AixLib.Media.Water;

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_a port_a1(
    redeclare package Medium = MediumW)
    annotation (
        Placement(transformation(extent= {{-110,-58},{-90,-38}} )),
        iconTransformation(extent= {{-110,-58},{-90,-38}} )
    );

Modelica.Fluid.Interfaces.FluidPort_b port_b1(
    redeclare package Medium = MediumW)
    annotation (
        Placement(transformation(extent= {{-110,40},{-90,60}} )),
        iconTransformation(extent= {{-110,38},{-90,58}} )
    );

// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
    annotation (
        Placement(transformation(extent= {{-120,52},{-80,92}} )),  
        iconTransformation(extent= {{-228,58},{-208,78}} )
    );
    three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.
BoilerWithStorageBoiler_001 boiler_001(
    a={0.9},
    dp=5000*{2,1},
    dp_nominal=5000.0,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    deltaM=0.1,
    hTan=2.0,
    show_T=false,
    Q_flow_nominal=20000.0,
    nSeg=4,
    VTan=0.2,
    T_nominal=353.15,
    dIns=0.002,
    linearizeFlowResistance=true,
    useStorageTank=false,
    TSouSet=286.15,
    TSet=323.15,
    nominal_mass_flow_radiator_loop=0.7142857142857143,
    nominal_mass_flow_rate_boiler=0.7142857142857143,
    V_flow=0.7142857142857143/1000*{0.5,1}
,
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.NaturalGasLowerHeatingValue()) "Boiler"  annotation (
    Placement(transformation(origin = { 100.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.BoilerControlControl_4
    control_4(    threshold_outdoor_air_cutoff=288.15,
    threshold_to_switch_off_boiler=288.15,
    TSup_nominal=353.15
) annotation (
    Placement(transformation(origin = { -100.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(boiler_001.dataBus,control_4.dataBus)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ -100.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(boiler_001.port_b,port_b1)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(boiler_001.port_a,port_a1)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(control_4.dataBus,dataBus)
            ;end production;
    model bus


extends Trano.BaseClasses.Containers.bus;
// Define Medium Package
package Medium = AixLib.Media.Air;

// Define Fluid Ports

// Define Heat Transfer Ports
Modelica.Blocks.Interfaces.RealInput[3] u
  annotation (Placement(transformation(extent= {{-138,-20},{-98,20}} )));
// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
  annotation (
      Placement(transformation(extent= {{-118,68},{-78,108}} )), 
      iconTransformation(extent= {{-228,58},{-208,78}} )
  );
  Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p annotation (
  Placement(transformation(extent= {{66,-24},{114,24}} ), iconTransformation(
      extent= {{66,-24},{114,24}} )));
        three_zones_hydronic_reduced_orders_reduced_order.Components.BaseClasses.DataServerReducedOrder
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 0.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(data_bus.u[1],u[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.u[2],u[2])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.u[3],u[3])
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

Components.Containers.distribution distribution1 annotation (Placement(transformation(extent={{-4.0,0.0},{16.0,20.0}})));
Components.Containers.emission emission1 annotation (Placement(transformation(extent={{-44.0,0.0},{-24.0,20.0}})));
Components.Containers.envelope envelope1 annotation (Placement(transformation(extent={{-84.0,0.0},{-64.0,20.0}})));
Components.Containers.production production1 annotation (Placement(transformation(extent={{36.0,0.0},{56.0,20.0}})));
Components.Containers.bus bus1 annotation (Placement(transformation(extent={{-84.0,30.0},{-64.0,50.0}})));

Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p
annotation (Placement(transformation(extent={{-126,-18},{-92,18}}),
iconTransformation(
extent={{-112,-12},{-88,12}})));
equation
connect(term_p, bus1.term_p) annotation (Line(points={{-109,0},{-88,0},
        {-88,-10},{60,-10},{60,64},{-50,64},{-50,40},{-65,40}}, color={
        0,120,120}));
connect(distribution1.port_a1[1],
emission1.port_b[1])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a1[2],
emission1.port_b[2])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a1[3],
emission1.port_b[3])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a,
production1.port_b1)
annotation (Line(points={{15.8,15},{30,15},{30,15},{36,15}},
                                          color={0,127,255}));
connect(distribution1.port_b,
production1.port_a1)
annotation (Line(points={{16,5},{32,5},{32,5.2},{36,5.2}},
                                          color={0,127,255}));
connect(distribution1.port_b1[1],
emission1.port_a[1])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[2],
emission1.port_a[2])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[3],
emission1.port_a[3])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(emission1.heatPortCon[1],
envelope1.heatPortCon[1])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[1],
envelope1.heatPortRad[1])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[2],
envelope1.heatPortCon[2])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[2],
envelope1.heatPortRad[2])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[3],
envelope1.heatPortCon[3])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[3],
envelope1.heatPortRad[3])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(envelope1.y[1],
bus1.u[1])
annotation (Line(points={{-85.7,10.1},{-94,10.1},
          {-94,40},{-85.8,40}}, color={0,0,127}));
connect(envelope1.y[2],
bus1.u[2])
annotation (Line(points={{-85.7,10.1},{-94,10.1},
          {-94,40},{-85.8,40}}, color={0,0,127}));
connect(envelope1.y[3],
bus1.u[3])
annotation (Line(points={{-85.7,10.1},{-94,10.1},
          {-94,40},{-85.8,40}}, color={0,0,127}));

connect(bus1.dataBus, envelope1.dataBus) annotation (Line(points={{-83.8,48.8},{-83.8,56},{-60,56},{-60,26},{-74,26},{-74,20}}, color={255,204,51}, thickness=0.5));
connect(emission1.dataBus, envelope1.dataBus) annotation (Line(points={{-95.8,16.8},{-95.8,26},{-55.8,26},{-55.8,16.8}}, color={255,204,51}, thickness=0.5));
connect(distribution1.dataBus, envelope1.dataBus) annotation (Line(points={{-95.8,16.8},{-95.8,26},{-15.8,26},{-15.8,16.8}}, color={255,204,51}, thickness=0.5));
connect(envelope1.dataBus, production1.dataBus) annotation (Line(points={{-95.8,16.8},{-95.8,26},{24.2,26},{24.2,16.8}}, color={255,204,51}, thickness=0.5));






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
        model EmissionControlControl_1
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
parameter Real k(min=0) = 1 "Gain of controller";
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops
emissionControl
annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); Modelica.Blocks.Interfaces.RealOutput y
annotation (Placement(transformation(extent={{100,-8},{120,12}})));

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
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(emissionControl.yHea, y) annotation (Line(points={{34.4,-21.2},{96,-21.2},
{96,2},{110,2}}, color={0,0,127}));
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
connect(dataBus.TCooSetSpace_001, emissionControl.TCooSet);
connect(dataBus.TZonSpace_001, emissionControl.TZon);
connect(dataBus.yCooValve_003, emissionControl.yCoo);
connect(dataBus.yHeaValve_003, emissionControl.yHea);
end EmissionControlControl_1;
        model EmissionControlControl_2
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
parameter Real k(min=0) = 1 "Gain of controller";
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops
emissionControl
annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); Modelica.Blocks.Interfaces.RealOutput y
annotation (Placement(transformation(extent={{100,-8},{120,12}})));

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
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(emissionControl.yHea, y) annotation (Line(points={{34.4,-21.2},{96,-21.2},
{96,2},{110,2}}, color={0,0,127}));
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
connect(dataBus.TCooSetSpace_002, emissionControl.TCooSet);
connect(dataBus.TZonSpace_002, emissionControl.TZon);
connect(dataBus.yCooValve_001, emissionControl.yCoo);
connect(dataBus.yHeaValve_001, emissionControl.yHea);
end EmissionControlControl_2;
        model EmissionControlControl_3
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
parameter Real k(min=0) = 1 "Gain of controller";
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops
emissionControl
annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); Modelica.Blocks.Interfaces.RealOutput y
annotation (Placement(transformation(extent={{100,-8},{120,12}})));

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
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(emissionControl.yHea, y) annotation (Line(points={{34.4,-21.2},{96,-21.2},
{96,2},{110,2}}, color={0,0,127}));
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
connect(dataBus.TCooSetSpace_003, emissionControl.TCooSet);
connect(dataBus.TZonSpace_003, emissionControl.TZon);
connect(dataBus.yCooValve_002, emissionControl.yCoo);
connect(dataBus.yHeaValve_002, emissionControl.yHea);
end EmissionControlControl_3;
        model CollectorControlControl_5
Buildings.Controls.OBC.CDL.Reals.PIDWithReset
conPum(    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.P,
    k=1.0,
    Nd=10.0,
    Ni=0.9,
    r=1.0,
    Td=0.1,
    Ti=0.5,
    yMax=1.0,
    yMin=0.0
) "Controller for pump"
annotation (Placement(transformation(extent={{54,-10},{74,10}})));Buildings.Controls.OBC.CDL.Reals.MultiMax
mulMax(nin=3)
"Maximum radiator valve position"
annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));Buildings.Controls.OBC.CDL.Reals.Hysteresis
hysPum(uLow=0.01, uHigh=0.5)
"Hysteresis for pump"
annotation (Placement(transformation(extent={{-26,-10},{-6,10}})));Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
"Conversion from boolean to real signal"
annotation (Placement(transformation(extent={{14,-10},{34,10}})));Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(mulMax.y,hysPum. u) annotation (Line(
points={{-54,0},{-28,0}},
color={0,0,127},
smooth=Smooth.None));connect(hysPum.y,conPum. trigger) annotation (Line(points={{-4,0},{4,0},{4,-18},
{58,-18},{58,-12}},     color={255,0,255}));connect(hysPum.y,booToRea. u)
annotation (Line(points={{-4,0},{12,0}},   color={255,0,255}));connect(booToRea.y,conPum. u_s)
annotation (Line(points={{36,0},{52,0}},     color={0,0,127}));annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
coordinateSystem(preserveAspectRatio=false)));connect(dataBus.y_gainPump_001, conPum.u_m);
connect(dataBus.yPump_001, conPum.y);
connect(dataBus.yBoiConPump_001, mulMax.y);
connect(dataBus.yPumBoiPump_001, mulMax.y);
connect(dataBus.yHeaValve_003, mulMax.u[1]);
connect(dataBus.yHeaValve_001, mulMax.u[2]);
connect(dataBus.yHeaValve_002, mulMax.u[3]);
end CollectorControlControl_5;
        model ThreeWayValveControlControl_6
  Buildings.Controls.OBC.CDL.Reals.PIDWithReset
                                      conVal(
        controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.P,
    k=1.0,
    Nd=10.0,
    Ni=0.9,
    r=1.0,
    Td=0.1,
    Ti=0.5,
    yMax=1.0,
    yMin=0.0
) "Controller for pump"
annotation (Placement(transformation(extent={{-12,-10},{8,10}})));  Modelica.Blocks.Interfaces.RealOutput y
annotation (Placement(transformation(extent={{100,-10},{120,10}})));  Modelica.Blocks.Interfaces.RealInput u
annotation (Placement(transformation(extent={{-138,-20},{-98,20}})));        Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.TColSetControl_6, conVal.u_s);
connect(dataBus.triggerControl_6, conVal.trigger);
  connect(conVal.y, y)
annotation (Line(points={{10,0},{110,0}}, color={0,0,127}));  connect(u, conVal.u_m) annotation (Line(points={{-118,0},{-22,0},{-22,-20},{0,
          -20},{0,-16},{-2,-16},{-2,-12}}, color={0,0,127}));annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));end ThreeWayValveControlControl_6;
        model ThreeWayValveControlControl_7
  Buildings.Controls.OBC.CDL.Reals.PIDWithReset
                                      conVal(
        controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.P,
    k=1.0,
    Nd=10.0,
    Ni=0.9,
    r=1.0,
    Td=0.1,
    Ti=0.5,
    yMax=1.0,
    yMin=0.0
) "Controller for pump"
annotation (Placement(transformation(extent={{-12,-10},{8,10}})));  Modelica.Blocks.Interfaces.RealOutput y
annotation (Placement(transformation(extent={{100,-10},{120,10}})));  Modelica.Blocks.Interfaces.RealInput u
annotation (Placement(transformation(extent={{-138,-20},{-98,20}})));        Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.TColSetControl_7, conVal.u_s);
connect(dataBus.triggerControl_7, conVal.trigger);
  connect(conVal.y, y)
annotation (Line(points={{10,0},{110,0}}, color={0,0,127}));  connect(u, conVal.u_m) annotation (Line(points={{-118,0},{-22,0},{-22,-20},{0,
          -20},{0,-16},{-2,-16},{-2,-12}}, color={0,0,127}));annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));end ThreeWayValveControlControl_7;
            model BoilerControlControl_4
    extends three_zones_hydronic_reduced_orders_reduced_order.Trano.Controls.ventilation.PartialBoilerControl;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.TStoTopBoiler_001, sub1.u1);
connect(dataBus.TStoBotBoiler_001, greThr.u);
connect(dataBus.TAirOutBoiler_001, lesThrTOut.u);
connect(dataBus.yBoiConBoiler_001, booToReaBoi.y);
connect(dataBus.yPumBoiBoiler_001, booToReaPum.y);
     end BoilerControlControl_4;
     
        model DataServerReducedOrder
replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
Modelica.Blocks.Interfaces.RealInput[3] u
annotation (Placement(transformation(extent={{-142,20},{-102,60}})));Modelica.Blocks.Routing.RealPassThrough[3] TRoo
annotation (Placement(transformation(extent={{-46,30},{-26,50}})));Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
Modelica.Blocks.Sources.RealExpression
            TAirOutControl_4
            (y=0.0);
Modelica.Blocks.Sources.RealExpression
            TCooSetControl_2
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetControl_1
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TColSetControl_7
            (y=363.15);
Modelica.Blocks.Sources.RealExpression
            TColSetControl_6
            (y=363.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetControl_3
            (y=298.15);
Modelica.Blocks.Sources.BooleanExpression
            triggerControl_6
            (y=true);
Modelica.Blocks.Sources.BooleanExpression
            triggerControl_7
            (y=true);

Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p annotation (
Placement(transformation(extent={{74,-20},{110,20}}), iconTransformation(
    extent={{74,-20},{110,20}})));
Buildings.Electrical.AC.OnePhase.Loads.Resistive loa(linearized=true, mode=
  Buildings.Electrical.Types.Load.VariableZ_P_input)
annotation (Placement(transformation(extent={{-32,-72},{16,-26}})));
Modelica.Blocks.Math.Sum sum1(nin= 1)

annotation (Placement(transformation(extent={{-72,-36},{-52,-16}})));
equation

connect(u[1], TRoo[1].u);
connect(u[2], TRoo[2].u);
connect(u[3], TRoo[3].u);
connect(dataBus.TZonSpace_001, TRoo[1].y);
connect(dataBus.TZonSpace_002, TRoo[2].y);
connect(dataBus.TZonSpace_003, TRoo[3].y);
connect(dataBus.TAirOutBoiler_001, 
    TAirOutControl_4.y);
connect(dataBus.TCooSetSpace_002, 
    TCooSetControl_2.y);
connect(dataBus.TCooSetSpace_001, 
    TCooSetControl_1.y);
connect(dataBus.TColSetControl_7, 
    TColSetControl_7.y);
connect(dataBus.TColSetControl_6, 
    TColSetControl_6.y);
connect(dataBus.TCooSetSpace_003, 
    TCooSetControl_3.y);
connect(dataBus.triggerControl_6, 
    triggerControl_6.y);
connect(dataBus.triggerControl_7, 
    triggerControl_7.y);



connect(term_p, loa.terminal) annotation (Line(points={{92,0},{-32,0},{-32,-51},
{ -28,-51 } }, color={0,120,120}));
connect(dataBus.electricityPump_001, sum1.u[1]) 

annotation (Line(
points={{-100,2},{-48,2},{-48,-40},{-84,-40},{-84,-26},{-74,-26}},
color={255,204,51},
thickness=0.5), Text(
string="%first",
index=-1,
extent={{-6,3},{-6,3}},
horizontalAlignment=TextAlignment.Right));

connect(sum1.y, loa.Pow) annotation (Line(points={{-51,-26},{-38,-26},{-38,-76},
    {24,-76},{24,-49},{16,-49}}, color={0,0,127}));
end DataServerReducedOrder;
            model BoilerWithStorageBoiler_001
    extends three_zones_hydronic_reduced_orders_reduced_order.Trano.Fluid.Boilers.PartialBoilerWithStorage;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.yBoiConBoiler_001, Boiy.u);
connect(dataBus.yPumBoiBoiler_001, pumBoi.y);
connect(dataBus.TStoTopBoiler_001, tanTemTop.T);
connect(dataBus.TStoBotBoiler_001, tanTemBot.T);
     end BoilerWithStorageBoiler_001;
     
        model PumpPump_001
extends three_zones_hydronic_reduced_orders_reduced_order.Trano.Fluid.Ventilation.PartialPump;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.yPump_001, pumRad.y);
connect(dataBus.y_gainPump_001, gain.y);
connect(dataBus.electricityPump_001, pumRad.P);
connect(dataBus.TControl_5, temSup.T);
 end PumpPump_001;
 
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
end three_zones_hydronic_reduced_orders_reduced_order;