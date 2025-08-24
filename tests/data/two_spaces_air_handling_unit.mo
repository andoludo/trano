package two_spaces_air_handling_unit

package Trano
  package Occupancy

    model SimpleOccupancy

      parameter Real occupancy[:]=3600*{7,19}
    "Occupancy table, each entry switching occupancy on or off";

      parameter Real gain[:, :]=[35; 70; 30]
    "Gain to convert from occupancy (per person) to radiant, convective and latent heat in [W/m2] ";

          parameter Real k=1/6/4
    "Heat gain if occupied";

      Buildings.Controls.SetPoints.OccupancySchedule
                                       occSch2(firstEntryOccupied=true,
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
    annotation (Placement(transformation(extent={{-58,-2},{-38,18}})));
  Buildings.HeatTransfer.Sources.FixedTemperature TAmb(T=288.15)
    "Ambient temperature in boiler room"
    annotation (Placement(transformation(extent={{20,80},{40,100}})));
  Buildings.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
MediumW) "Fixed boundary condition, needed to provide a pressure in the system"
    annotation (Placement(transformation(extent={{-74,68},{-54,88}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor1(
      redeclare package Medium = MediumW, m_flow_nominal=nominal_mass_flow_rate_boiler)
                                      "Radiator"  annotation (
    Placement(transformation(origin={-8,5},
    extent = {{-10, -10}, {10, 10}},
        rotation=0)));
      IDEAS.Controls.Continuous.LimPID conPID(controllerType=Modelica.Blocks.Types.SimpleController.P)
                                              annotation (Placement(
            transformation(extent={{-24,42},{-4,62}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=TempSet)
    annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
equation
  connect(
  TAmb.port, boi.heatPort)
           annotation (Line(
      points={{40,90},{44,90},{44,34},{-48,34},{-48,15.2}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(
  bou.ports[1], boi.port_a)
            annotation (Line(
      points={{-54,78},{-50,78},{-50,92},{-78,92},{-78,8},{-58,8}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(boi.port_b, temperature_sensor1.port_a) annotation (Line(points={{-38,8},
          {-22,8},{-22,5},{-18,5}},        color={0,127,255}));
      connect(boi.T, conPID.u_m) annotation (Line(points={{-37,16},{-22,16},{-22,
          32},{-14,32},{-14,40}},              color={0,0,127}));
      connect(conPID.y, boi.y) annotation (Line(points={{-3,52},{0,52},{0,66},{-50,
          66},{-50,24},{-60,24},{-60,16}},          color={0,0,127}));
  connect(temperature_sensor1.port_b, port_b)
    annotation (Line(points={{2,5},{84,5},{84,0},{100,0}}, color={0,127,255}));
  connect(port_a, boi.port_a) annotation (Line(points={{-100,0},{-68,0},{-68,8},
          {-58,8}}, color={0,127,255}));
  connect(realExpression.y, conPID.u_s) annotation (Line(points={{-69,50},{-34,50},
          {-34,52},{-26,52}}, color={0,0,127}));
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

    partial model PartialHeatPump
      replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);
      extends Buildings.Fluid.Interfaces.PartialTwoPort(
                                                redeclare package Medium = MediumW);
    final parameter Modelica.Units.SI.SpecificHeatCapacity cp1_default=
          MediumW.specificHeatCapacityCp(MediumW.setState_pTX(
          MediumW.p_default,
          MediumW.T_default,
          MediumW.X_default))
        "Specific heat capacity of medium 2 at default medium state";

      final parameter Modelica.Units.SI.SpecificHeatCapacity cp2_default=
          MediumW.specificHeatCapacityCp(MediumW.setState_pTX(
          MediumW.p_default,
          MediumW.T_default,
          MediumW.X_default))
        "Specific heat capacity of medium 2 at default medium state";

      parameter Real COP_nominal = 6 "Nominal COP";

      parameter Modelica.Units.SI.Power P_nominal=10E3
        "Nominal compressor power (at y=1)";
      parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-10
        "Temperature difference evaporator outlet-inlet";
      parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
        "Temperature difference condenser outlet-inlet";
      parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal=-P_nominal*(
          COP_nominal - 1)/cp2_default/dTEva_nominal
        "Nominal mass flow rate at chilled water side";
      parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal=P_nominal*
          COP_nominal/cp1_default/dTCon_nominal
        "Nominal mass flow rate at condenser water wide";
      Buildings.Fluid.HeatPumps.Carnot_y heaPum(
        redeclare package Medium1 = MediumW,
        redeclare package Medium2 = MediumW,
        P_nominal=P_nominal,
        dTEva_nominal=dTEva_nominal,
        dTCon_nominal=dTCon_nominal,
        dp1_nominal=6000,
        dp2_nominal=6000,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        show_T=true,
        use_eta_Carnot_nominal=false,
        COP_nominal=COP_nominal,
        TCon_nominal=303.15,
        TEva_nominal=278.15) "Heat pump model"
        annotation (Placement(transformation(extent={{-8,-16},{12,4}})));
      Buildings.Fluid.Sources.MassFlowSource_T sou2(
        nPorts=1,
        redeclare package Medium = MediumW,
        use_T_in=true,
        m_flow=m2_flow_nominal,
        T=291.15)
        annotation (Placement(transformation(extent={{48,-68},{28,-48}})));
      Buildings.Fluid.Sources.Boundary_pT sin2(nPorts=1, redeclare package
            Medium =
            MediumW)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-76,-60})));
      Modelica.Blocks.Sources.Constant const(k=1)
        annotation (Placement(transformation(extent={{-86,44},{-66,64}})));
      Modelica.Blocks.Sources.Constant const2(k=273.15 + 15)
        annotation (Placement(transformation(extent={{14,-100},{34,-80}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort
                                 senTem(redeclare package Medium = MediumW,
          m_flow_nominal=m1_flow_nominal)
        annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort
                                 senTem1(redeclare package Medium = MediumW,
          m_flow_nominal=m1_flow_nominal)
        annotation (Placement(transformation(extent={{46,-10},{66,10}})));
      Buildings.Fluid.Sources.Boundary_pT sin1(nPorts=2, redeclare package
            Medium = MediumW)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-48,-24})));
    equation
      connect(sou2.ports[1],heaPum. port_a2) annotation (Line(
          points={{28,-58},{18,-58},{18,-12},{12,-12}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(sin2.ports[1],heaPum. port_b2) annotation (Line(
          points={{-66,-60},{-14,-60},{-14,-12},{-8,-12}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(const.y,heaPum. y) annotation (Line(points={{-65,54},{-18,54},{-18,3},
              {-10,3}}, color={0,0,127}));
      connect(const2.y,sou2. T_in) annotation (Line(points={{35,-90},{60,-90},{60,-54},
              {50,-54}},
                       color={0,0,127}));
      connect(heaPum.port_b1,senTem1. port_a) annotation (Line(points={{12,0},{46,0}},
                                color={0,127,255}));
      connect(port_a,senTem. port_a) annotation (Line(points={{-100,0},{-78,0}},
                               color={0,127,255}));
      connect(senTem1.port_b, port_b)
        annotation (Line(points={{66,0},{100,0}}, color={0,127,255}));
        connect(senTem.port_b, sin1.ports[1]) annotation (Line(points={{-58,0},{
                -32,0},{-32,-22},{-38,-22}}, color={0,127,255}));
        connect(heaPum.port_a1, sin1.ports[2]) annotation (Line(points={{-8,0},{
                -32,0},{-32,-26},{-38,-26},{-38,-26}}, color={0,127,255}));
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
    end PartialHeatPump;
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
       parameter Modelica.Units.SI.PressureDifference dp_nominal
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
  annotation (uses(Buildings(version = "11.0.0"), Modelica(version = "4.0.0"),
      IDEAS(version="3.0.0")),
  Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
            fillPattern =                                                                            FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
end Trano;



package Components
  package Containers
    model envelope

            parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic double_glazing(
    final glass={
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003,
        k=1.0,
        tauSol={ 0.646 },
        rhoSol_a={ 0.062 },
        rhoSol_b={ 0.063 },
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84)
        ,
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003,
        k=1.0,
        tauSol={ 0.646 },
        rhoSol_a={ 0.062 },
        rhoSol_b={ 0.063 },
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84)
        
    },
    final gas={
            Buildings.HeatTransfer.Data.Gases.Air(x=0.0127)
            
    },
    UFra=1.4)
    annotation (
    defaultComponentPrefixes="parameter",
    defaultComponentName="datGlaSys");
    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        external_wall(
    final nLay=3,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=1.4,
        c=840.0,
        d=2240.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.02,
        k=0.03,
        c=1200.0,
        d=40.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.1,
        k=0.12,
        c=1210.0,
        d=540.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        internal_wall(
    final nLay=1,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=0.89,
        c=790.0,
        d=1920.0)    },
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

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[2] heatPortCon1
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
Modelica.Fluid.Interfaces.FluidPorts_b[6] ports_b(
  redeclare package Medium = Medium)
  annotation (Placement(transformation(extent= {{-106,34},{-96,78}} ), iconTransformation(
      extent= {{-106,34},{-96,78}} )), iconTransformation(extent=  {{-106,30},{-92,86}} ));

Modelica.Fluid.Interfaces.FluidPorts_a[0] ports_a(
  redeclare package Medium = Medium)
  annotation ( Placement(transformation(extent= {{-104,-80},{-94,-34}} ), iconTransformation(
      extent= {{-104,-80},{-94,-34}} )),iconTransformation(extent=  {{-108,-92},{-94,-40}} ));
    Buildings.ThermalZones.Detailed.MixedAir space_1(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=1,
                    datConExt(
                    layers={ external_wall },
    A={ 10.0 },
    til={Buildings.Types.Tilt.Wall},
                    azi={ 135.0 }),
                    nSurBou=1,
                    surBou(
                    A={ 10.0 },
                    til={Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ external_wall },
    A={ 10.0 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=1,
                    datConExtWin(
                    layers={ external_wall },
    A={ 10.0 },
    til={Buildings.Types.Tilt.Wall},
                    glaSys={ double_glazing },
                    wWin={ 1.0 },
                    hWin={ 1.0 },
                    azi={ 45.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 43.993366593531476, 0.04828363184878981 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
        two_spaces_air_handling_unit.Components.BaseClasses.OccupancyOccupancy_0
    occupancy_0(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 28.993366593531476, 0.04828363184878981 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_2(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=0,                    nSurBou=1,
                    surBou(
                    A={ 10.0 },
                    til={Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ external_wall },
    A={ 10.0 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=1,
                    datConExtWin(
                    layers={ external_wall },
    A={ 10.0 },
    til={Buildings.Types.Tilt.Wall},
                    glaSys={ double_glazing },
                    wWin={ 1.0 },
                    hWin={ 1.0 },
                    azi={ 45.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -59.10393113182498, 0.04828363184878981 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
        two_spaces_air_handling_unit.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -74.10393113182498, 0.04828363184878981 },
    extent = {{ 3, -3}, {-3, 3}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_1_space_2(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -8.980719333305387, 46.106269974478664 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather_0(filNam=Modelica.Utilities.Files.loadResource
        ("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
     annotation (
    Placement(transformation(origin = { -8.95276958694933, -45.95911985836801 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(space_1.qGai_flow,occupancy_0.y)
        annotation (Line(
        points={{ 43.993366593531476, 0.04828363184878981 }    ,{ 36.493366593531476, 0.04828363184878981 }    ,{ 36.493366593531476, 0.04828363184878981 }    ,{ 28.993366593531476, 0.04828363184878981 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.ports[1],ports_b[1])
        annotation (Line(
        points={{ 43.993366593531476, 0.04828363184878981 }    ,{ 21.996683296765738, 0.04828363184878981 }    ,{ 21.996683296765738, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
        annotation (Line(
        points={{ 43.993366593531476, 0.04828363184878981 }    ,{ 17.506323630113044, 0.04828363184878981 }    ,{ 17.506323630113044, 46.106269974478664 }    ,{ -8.980719333305387, 46.106269974478664 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ 43.993366593531476, 0.04828363184878981 }    ,{ 17.520298503291073, 0.04828363184878981 }    ,{ 17.520298503291073, -45.95911985836801 }    ,{ -8.95276958694933, -45.95911985836801 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.ports[2],ports_b[2])
        annotation (Line(
        points={{ 43.993366593531476, 0.04828363184878981 }    ,{ 21.996683296765738, 0.04828363184878981 }    ,{ 21.996683296765738, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_2.qGai_flow,occupancy_1.y)
        annotation (Line(
        points={{ -59.10393113182498, 0.04828363184878981 }    ,{ -66.60393113182498, 0.04828363184878981 }    ,{ -66.60393113182498, 0.04828363184878981 }    ,{ -74.10393113182498, 0.04828363184878981 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.ports[1],ports_b[3])
        annotation (Line(
        points={{ -59.10393113182498, 0.04828363184878981 }    ,{ -29.55196556591249, 0.04828363184878981 }    ,{ -29.55196556591249, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_2.surf_surBou[1],internal_space_1_space_2.port_b)
        annotation (Line(
        points={{ -59.10393113182498, 0.04828363184878981 }    ,{ -34.04232523256518, 0.04828363184878981 }    ,{ -34.04232523256518, 46.106269974478664 }    ,{ -8.980719333305387, 46.106269974478664 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -59.10393113182498, 0.04828363184878981 }    ,{ -34.028350359387154, 0.04828363184878981 }    ,{ -34.028350359387154, -45.95911985836801 }    ,{ -8.95276958694933, -45.95911985836801 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.ports[2],ports_b[4])
        annotation (Line(
        points={{ -59.10393113182498, 0.04828363184878981 }    ,{ -29.55196556591249, 0.04828363184878981 }    ,{ -29.55196556591249, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(occupancy_0.dataBus,dataBus)
            ;        
        connect(occupancy_1.dataBus,dataBus)
            ;        
        connect(space_1.heaPorAir,heatPortCon1[1])
            ;        
        connect(space_1.ports[3],ports_b[5])
            ;        
        connect(space_2.heaPorAir,heatPortCon1[2])
            ;        
        connect(space_2.ports[3],ports_b[6])
            ;        
        connect(weather_0.weaBus,dataBus)
        annotation (Line(
        points={{ -8.95276958694933, -45.95911985836801 }    ,{ -8.95276958694933, -45.95911985836801 }    ,{ -8.95276958694933, -45.95911985836801 }    ,{ -8.95276958694933, -45.95911985836801 }    },
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
Modelica.Fluid.Interfaces.FluidPort_b[2] port_b(
  redeclare package Medium = Medium)
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[2] heatPortCon

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
        two_spaces_air_handling_unit.Components.BaseClasses.DataServer
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
        connect(data_bus.port[2],heatPortCon[2])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[2],port_b[2])
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
    model ventilation


extends Trano.BaseClasses.Containers.ventilation;
// Define Medium Package
replaceable package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"})
  constrainedby Modelica.Media.Interfaces.PartialMedium
  annotation (choicesAllMatching = true);

// Define Data Bus
Trano.Controls.BaseClasses.DataBus dataBus 
  annotation (
      Placement(transformation(extent= {{-120,52},{-80,92}} )),  
      iconTransformation(extent= {{-228,58},{-208,78}} )
  );

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPorts_b[2] ports_b(
  redeclare package Medium = Medium)
  annotation (
  Placement(transformation(extent= {{-102,18},{-92,58}} ), iconTransformation(
          extent= {{-102,18},{-92,58}} )),
  iconTransformation(extent=  {{-108,0},{-92,54}} ));

Modelica.Fluid.Interfaces.FluidPorts_a[2] ports_a(
  redeclare package Medium = Medium)
  annotation (
  Placement(transformation(extent= {{-102,-74},{-92,-32}} ), iconTransformation(
          extent= {{-102,-74},{-92,-32}} )),
  iconTransformation(extent=  {{-110,-70},{-94,-22}} ));
      two_spaces_air_handling_unit.Components.BaseClasses.VAVBoxVav_in
     vav_in(
    redeclare package MediumA = Medium,
    mCooAir_flow_nominal=100*1.2/3600,
    mHeaAir_flow_nominal=100*1.2/3600,
    VRoo=100,
    allowFlowReversal=false,
    THeaWatInl_nominal=90,
    THeaWatOut_nominal=60,
    THeaAirInl_nominal=30,
    THeaAirDis_nominal=25
    ) annotation (
    Placement(transformation(origin = { 69.09756499998602, -61.13962300350137 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        two_spaces_air_handling_unit.Components.BaseClasses.VAVControlVav_in_control
    vav_in_control annotation (
    Placement(transformation(origin = { 100.0, -92.29243608806178 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      Buildings.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_out(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { -32.73364897630441, 84.92733464434744 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      Buildings.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_in(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { 33.20319819227748, -22.885989735718738 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      two_spaces_air_handling_unit.Components.BaseClasses.VAVBoxVav_in_2
     vav_in_2(
    redeclare package MediumA = Medium,
    mCooAir_flow_nominal=100*1.2/3600,
    mHeaAir_flow_nominal=100*1.2/3600,
    VRoo=100,
    allowFlowReversal=false,
    THeaWatInl_nominal=90,
    THeaWatOut_nominal=60,
    THeaAirInl_nominal=30,
    THeaAirDis_nominal=25
    ) annotation (
    Placement(transformation(origin = { -67.31081697089019, -80.47796057364862 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        two_spaces_air_handling_unit.Components.BaseClasses.VAVControlVav_in_control_2
    vav_in_control_2 annotation (
    Placement(transformation(origin = { -100.0, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      Buildings.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_out_2(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { -43.81797486974833, 29.82397237277567 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      Buildings.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_in_2(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { -34.59198997698397, -37.6972516667466 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    two_spaces_air_handling_unit.Components.BaseClasses.AhuAhu
    ahu
    (redeclare package MediumA = Medium,

    VRoo={100,100},
    AFlo={20,20},
    mCooVAV_flow_nominal={0.01,0.01}) annotation (
    Placement(transformation(origin = { -5.654166794844201, 29.368315027099612 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        two_spaces_air_handling_unit.Components.BaseClasses.AhuControlAhu_control
    ahu_control annotation (
    Placement(transformation(origin = { 4.66948377552248, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      Buildings.Fluid.Sources.Boundary_pT boundary
    (nPorts=2,redeclare package Medium = Medium) annotation (
    Placement(transformation(origin = { 30.882019816710823, 54.46304379106911 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(pressure_drop_duct_out.port_a,ports_a[1])
        annotation (Line(
        points={{ -32.73364897630441, 84.92733464434744 }    ,{ -16.366824488152204, 84.92733464434744 }    ,{ -16.366824488152204, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(vav_in.dataBus,vav_in_control.dataBus)
        annotation (Line(
        points={{ 69.09756499998602, -61.13962300350137 }    ,{ 84.54878249999301, -61.13962300350137 }    ,{ 84.54878249999301, -92.29243608806178 }    ,{ 100.0, -92.29243608806178 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(vav_in.port_bAir,ports_b[1])
        annotation (Line(
        points={{ 69.09756499998602, -61.13962300350137 }    ,{ 34.54878249999301, -61.13962300350137 }    ,{ 34.54878249999301, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(pressure_drop_duct_out.port_b,ahu.port_a)
        annotation (Line(
        points={{ -32.73364897630441, 84.92733464434744 }    ,{ -19.193907885574305, 84.92733464434744 }    ,{ -19.193907885574305, 29.368315027099612 }    ,{ -5.654166794844201, 29.368315027099612 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pressure_drop_duct_in.port_b,vav_in.port_aAir)
        annotation (Line(
        points={{ 33.20319819227748, -22.885989735718738 }    ,{ 51.15038159613175, -22.885989735718738 }    ,{ 51.15038159613175, -61.13962300350137 }    ,{ 69.09756499998602, -61.13962300350137 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pressure_drop_duct_out_2.port_a,ports_a[2])
        annotation (Line(
        points={{ -43.81797486974833, 29.82397237277567 }    ,{ -21.908987434874167, 29.82397237277567 }    ,{ -21.908987434874167, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(vav_in_2.dataBus,vav_in_control_2.dataBus)
        annotation (Line(
        points={{ -67.31081697089019, -80.47796057364862 }    ,{ -83.6554084854451, -80.47796057364862 }    ,{ -83.6554084854451, -100.0 }    ,{ -100.0, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(vav_in_2.port_bAir,ports_b[2])
        annotation (Line(
        points={{ -67.31081697089019, -80.47796057364862 }    ,{ -33.65540848544509, -80.47796057364862 }    ,{ -33.65540848544509, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(pressure_drop_duct_out_2.port_b,ahu.port_a)
        annotation (Line(
        points={{ -43.81797486974833, 29.82397237277567 }    ,{ -24.736070832296267, 29.82397237277567 }    ,{ -24.736070832296267, 29.368315027099612 }    ,{ -5.654166794844201, 29.368315027099612 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pressure_drop_duct_in_2.port_b,vav_in_2.port_aAir)
        annotation (Line(
        points={{ -34.59198997698397, -37.6972516667466 }    ,{ -50.95140347393708, -37.6972516667466 }    ,{ -50.95140347393708, -80.47796057364862 }    ,{ -67.31081697089019, -80.47796057364862 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(ahu.dataBus,ahu_control.dataBus)
        annotation (Line(
        points={{ -5.654166794844201, 29.368315027099612 }    ,{ -0.49234150966086077, 29.368315027099612 }    ,{ -0.49234150966086077, 100.0 }    ,{ 4.66948377552248, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(ahu.port_b,pressure_drop_duct_in.port_a)
        annotation (Line(
        points={{ -5.654166794844201, 29.368315027099612 }    ,{ 13.774515698716641, 29.368315027099612 }    ,{ 13.774515698716641, -22.885989735718738 }    ,{ 33.20319819227748, -22.885989735718738 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(ahu.port_b,pressure_drop_duct_in_2.port_a)
        annotation (Line(
        points={{ -5.654166794844201, 29.368315027099612 }    ,{ -20.123078385914084, 29.368315027099612 }    ,{ -20.123078385914084, -37.6972516667466 }    ,{ -34.59198997698397, -37.6972516667466 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(boundary.ports,ahu.ports)
        annotation (Line(
        points={{ 30.882019816710823, 54.46304379106911 }    ,{ 12.613926510933311, 54.46304379106911 }    ,{ 12.613926510933311, 29.368315027099612 }    ,{ -5.654166794844201, 29.368315027099612 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(vav_in_control.dataBus,dataBus)
            ;        
        connect(vav_in_control_2.dataBus,dataBus)
            ;        
        connect(ahu_control.dataBus,dataBus)
            ;end ventilation;

model building

Components.Containers.envelope envelope1 annotation (Placement(transformation(extent={{-84.0,0.0},{-64.0,20.0}})));
Components.Containers.bus bus1 annotation (Placement(transformation(extent={{-84.0,30.0},{-64.0,50.0}})));
Components.Containers.ventilation ventilation1 annotation (Placement(transformation(extent={{-44.0,-30.0},{-24.0,-10.0}})));

Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p
annotation (Placement(transformation(extent={{-126,-18},{-92,18}}),
iconTransformation(
extent={{-112,-12},{-88,12}})));
equation
connect(term_p, bus1.term_p) annotation (Line(points={{-109,0},{-88,0},
        {-88,-10},{60,-10},{60,64},{-50,64},{-50,40},{-65,40}}, color={
        0,120,120}));
connect(envelope1.ports_b[1],
ventilation1.ports_a[1])
annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));
connect(envelope1.ports_b[2],
ventilation1.ports_b[1])
annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));
connect(envelope1.ports_b[3],
ventilation1.ports_a[2])
annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));
connect(envelope1.ports_b[4],
ventilation1.ports_b[2])
annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));
connect(envelope1.heatPortCon1[1],
bus1.heatPortCon[1])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[5],
bus1.port_b[1])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[2],
bus1.heatPortCon[2])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[6],
bus1.port_b[2])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));

connect(bus1.dataBus, envelope1.dataBus) annotation (Line(points={{-83.8,48.8},{-83.8,56},{-60,56},{-60,26},{-74,26},{-74,20}}, color={255,204,51}, thickness=0.5));
connect(envelope1.dataBus, ventilation1.dataBus) annotation (Line(points={{-44.1,-32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}}, color={255,204,51}, thickness=0.5));




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
        model OccupancyOccupancy_0
extends two_spaces_air_handling_unit.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_1, occSch2.occupied);
 end OccupancyOccupancy_0;
 
        model VAVControlVav_in_control
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
annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); Trano.Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.TZonSpace_1, rehBoxCon.TZon);
connect(dataBus.TCooSetSpace_1, rehBoxCon.TCooSet);
connect(dataBus.THeaSetSpace_1, rehBoxCon.THeaSet);
connect(dataBus.ppmCO2SetSpace_1, rehBoxCon.ppmCO2Set);
connect(dataBus.ppmCO2Space_1, rehBoxCon.ppmCO2);
connect(dataBus.TSupSetSpace_1, rehBoxCon.TSupSet);
connect(dataBus.uOpeModSpace_1, rehBoxCon.uOpeMod);
connect(dataBus.oveFloSetSpace_1, rehBoxCon.oveFloSet);
connect(dataBus.oveDamPosSpace_1, rehBoxCon.oveDamPos);
connect(dataBus.u1WinSpace_1, rehBoxCon.u1Win);
connect(dataBus.u1OccSpace_1, rehBoxCon.u1Occ);
connect(dataBus.uHeaOffSpace_1, rehBoxCon.uHeaOff);
connect(dataBus.u1FanSpace_1, rehBoxCon.u1Fan);
connect(dataBus.u1HotPlaSpace_1, rehBoxCon.u1HotPla);
connect(dataBus.TDisVav_in_control, rehBoxCon.TDis);
connect(dataBus.VDis_flowVav_in_control, rehBoxCon.VDis_flow);
connect(dataBus.VAdjPopBreZon_flowVav_in_control, rehBoxCon.VAdjPopBreZon_flow);
connect(dataBus.VAdjAreBreZon_flowVav_in_control, rehBoxCon.VAdjAreBreZon_flow);
connect(dataBus.VMinOA_flowVav_in_control, rehBoxCon.VMinOA_flow);
connect(dataBus.yZonTemResReqVav_in_control, rehBoxCon.yZonTemResReq);
connect(dataBus.yZonPreResReqVav_in_control, rehBoxCon.yZonPreResReq);
connect(dataBus.yHeaValResReqVav_in_control, rehBoxCon.yHeaValResReq);
connect(dataBus.TAirSupAhu_control, rehBoxCon.TSup);
connect(dataBus.VSet_flowVav_in, rehBoxCon.VSet_flow);
connect(dataBus.yDamVav_in, rehBoxCon.yDam);
connect(dataBus.yValVav_in, rehBoxCon.yVal);
connect(dataBus.VZonAbsMin_flowVav_in, rehBoxCon.VZonAbsMin_flow);
connect(dataBus.VZonDesMin_flowVav_in, rehBoxCon.VZonDesMin_flow);
connect(dataBus.yCO2Vav_in, rehBoxCon.yCO2);
connect(dataBus.yHotWatPlaReqVav_in, rehBoxCon.yHotWatPlaReq);
connect(dataBus.yLowFloAlaVav_in, rehBoxCon.yLowFloAla);
connect(dataBus.yFloSenAlaVav_in, rehBoxCon.yFloSenAla);
connect(dataBus.yLeaDamAlaVav_in, rehBoxCon.yLeaDamAla);
connect(dataBus.yLeaValAlaVav_in, rehBoxCon.yLeaValAla);
connect(dataBus.yLowTemAlaVav_in, rehBoxCon.yLowTemAla);
end VAVControlVav_in_control;
        model OccupancyOccupancy_1
extends two_spaces_air_handling_unit.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_2, occSch2.occupied);
 end OccupancyOccupancy_1;
 
        model VAVControlVav_in_control_2
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
annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); Trano.Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.TZonSpace_2, rehBoxCon.TZon);
connect(dataBus.TCooSetSpace_2, rehBoxCon.TCooSet);
connect(dataBus.THeaSetSpace_2, rehBoxCon.THeaSet);
connect(dataBus.ppmCO2SetSpace_2, rehBoxCon.ppmCO2Set);
connect(dataBus.ppmCO2Space_2, rehBoxCon.ppmCO2);
connect(dataBus.TSupSetSpace_2, rehBoxCon.TSupSet);
connect(dataBus.uOpeModSpace_2, rehBoxCon.uOpeMod);
connect(dataBus.oveFloSetSpace_2, rehBoxCon.oveFloSet);
connect(dataBus.oveDamPosSpace_2, rehBoxCon.oveDamPos);
connect(dataBus.u1WinSpace_2, rehBoxCon.u1Win);
connect(dataBus.u1OccSpace_2, rehBoxCon.u1Occ);
connect(dataBus.uHeaOffSpace_2, rehBoxCon.uHeaOff);
connect(dataBus.u1FanSpace_2, rehBoxCon.u1Fan);
connect(dataBus.u1HotPlaSpace_2, rehBoxCon.u1HotPla);
connect(dataBus.TDisVav_in_control_2, rehBoxCon.TDis);
connect(dataBus.VDis_flowVav_in_control_2, rehBoxCon.VDis_flow);
connect(dataBus.VAdjPopBreZon_flowVav_in_control_2, rehBoxCon.VAdjPopBreZon_flow);
connect(dataBus.VAdjAreBreZon_flowVav_in_control_2, rehBoxCon.VAdjAreBreZon_flow);
connect(dataBus.VMinOA_flowVav_in_control_2, rehBoxCon.VMinOA_flow);
connect(dataBus.yZonTemResReqVav_in_control_2, rehBoxCon.yZonTemResReq);
connect(dataBus.yZonPreResReqVav_in_control_2, rehBoxCon.yZonPreResReq);
connect(dataBus.yHeaValResReqVav_in_control_2, rehBoxCon.yHeaValResReq);
connect(dataBus.TAirSupAhu_control, rehBoxCon.TSup);
connect(dataBus.VSet_flowVav_in_2, rehBoxCon.VSet_flow);
connect(dataBus.yDamVav_in_2, rehBoxCon.yDam);
connect(dataBus.yValVav_in_2, rehBoxCon.yVal);
connect(dataBus.VZonAbsMin_flowVav_in_2, rehBoxCon.VZonAbsMin_flow);
connect(dataBus.VZonDesMin_flowVav_in_2, rehBoxCon.VZonDesMin_flow);
connect(dataBus.yCO2Vav_in_2, rehBoxCon.yCO2);
connect(dataBus.yHotWatPlaReqVav_in_2, rehBoxCon.yHotWatPlaReq);
connect(dataBus.yLowFloAlaVav_in_2, rehBoxCon.yLowFloAla);
connect(dataBus.yFloSenAlaVav_in_2, rehBoxCon.yFloSenAla);
connect(dataBus.yLeaDamAlaVav_in_2, rehBoxCon.yLeaDamAla);
connect(dataBus.yLeaValAlaVav_in_2, rehBoxCon.yLeaValAla);
connect(dataBus.yLowTemAlaVav_in_2, rehBoxCon.yLowTemAla);
end VAVControlVav_in_control_2;
        model AhuControlAhu_control
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
have_perZonRehBox=false, VUncDesOutAir_flow = VUncDesOutAir_flow,
VDesTotOutAir_flow = VDesTotOutAir_flow)
annotation (Placement(transformation(extent={{-12,-14},{28,74}})));Trano.Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));

Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.SetPoints.OutdoorAirFlow.ASHRAE62_1.SumZone
sumZon(nZon=2, nGro=1,final zonGroMat=[1, 1],
final zonGroMatTra=[1; 1])
annotation (Placement(transformation(extent={{-72,32},{-52,52}})));Buildings.Controls.OBC.CDL.Integers.MultiSum preRetReq(final
nin=2)
annotation (Placement(transformation(extent={{-72,80},{-60,92}})));Buildings.Controls.OBC.CDL.Integers.MultiSum temResReq(final nin=2)
annotation (Placement(transformation(extent={{-72,56},{-60,68}})));equation
connect(dataBus.dpDucAhu_control, mulAHUCon.dpDuc);
connect(dataBus.TOutAhu_control, mulAHUCon.TOut);
connect(dataBus.TAirSupAhu_control, mulAHUCon.TAirSup);
connect(dataBus.VAirOut_flowAhu_control, mulAHUCon.VAirOut_flow);
connect(dataBus.TAirMixAhu_control, mulAHUCon.TAirMix);
connect(dataBus.uAhuOpeModAhu_control, mulAHUCon.uAhuOpeMod);
connect(dataBus.uAhuOpeModAhu_control, sumZon.uOpeMod[1]);
connect(dataBus.u1SupFanAhu_control, mulAHUCon.u1SupFan);
connect(dataBus.VAdjPopBreZon_flowVav_in_control, sumZon.VAdjPopBreZon_flow[1]);
connect(dataBus.VAdjPopBreZon_flowVav_in_control_2, sumZon.VAdjPopBreZon_flow[2]);
connect(dataBus.VAdjAreBreZon_flowVav_in_control, sumZon.VAdjAreBreZon_flow[1]);
connect(dataBus.VAdjAreBreZon_flowVav_in_control_2, sumZon.VAdjAreBreZon_flow[2]);
connect(dataBus.VDis_flowVav_in_control, sumZon.VZonPri_flow[1]);
connect(dataBus.VDis_flowVav_in_control_2, sumZon.VZonPri_flow[2]);
connect(dataBus.VMinOA_flowVav_in_control, sumZon.VMinOA_flow[1]);
connect(dataBus.VMinOA_flowVav_in_control_2, sumZon.VMinOA_flow[2]);
connect(dataBus.yZonPreResReqVav_in_control, preRetReq.u[1]);
connect(dataBus.yZonPreResReqVav_in_control_2, preRetReq.u[2]);
connect(dataBus.yZonTemResReqVav_in_control, temResReq.u[1]);
connect(dataBus.yZonTemResReqVav_in_control_2, temResReq.u[2]);
connect(dataBus.TAirSupSetAhu, mulAHUCon.TAirSupSet);
connect(dataBus.VEffAirOut_flow_minAhu, mulAHUCon.VEffAirOut_flow_min);
connect(dataBus.yMinOutDamAhu, mulAHUCon.yMinOutDam);
connect(dataBus.yRetDamAhu, mulAHUCon.yRetDam);
connect(dataBus.yRelDamAhu, mulAHUCon.yRelDam);
connect(dataBus.yOutDamAhu, mulAHUCon.yOutDam);
connect(dataBus.ySupFanAhu, mulAHUCon.ySupFan);
connect(dataBus.yRetFanAhu, mulAHUCon.yRetFan);
connect(dataBus.yRelFanAhu, mulAHUCon.yRelFan);
connect(dataBus.yCooCoiAhu, mulAHUCon.yCooCoi);
connect(dataBus.yHeaCoiAhu, mulAHUCon.yHeaCoi);
connect(dataBus.yDpBuiAhu, mulAHUCon.yDpBui);
connect(dataBus.dpDisSetAhu, mulAHUCon.dpDisSet);
connect(dataBus.yAlaAhu, mulAHUCon.yAla);
connect(dataBus.yChiWatResReqAhu, mulAHUCon.yChiWatResReq);
connect(dataBus.yChiPlaReqAhu, mulAHUCon.yChiPlaReq);
connect(dataBus.yHotWatResReqAhu, mulAHUCon.yHotWatResReq);
connect(dataBus.yHotWatPlaReqAhu, mulAHUCon.yHotWatPlaReq);
connect(dataBus.y1MinOutDamAhu, mulAHUCon.y1MinOutDam);
connect(dataBus.y1EneCHWPumAhu, mulAHUCon.y1EneCHWPum);
connect(dataBus.y1SupFanAhu, mulAHUCon.y1SupFan);
connect(dataBus.y1RetFanAhu, mulAHUCon.y1RetFan);
connect(dataBus.y1RelFanAhu, mulAHUCon.y1RelFan);
connect(dataBus.y1RelDamAhu, mulAHUCon.y1RelDam);

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
end AhuControlAhu_control;
        model DataServer
replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[2]
TRoo annotation (
Placement(transformation(origin={-544,-226},
extent = {{480, 216}, {500, 236}})));Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[2]
port annotation (
Placement(transformation(extent={{-112,-10},{-92,10}}),
iconTransformation(extent = {{-110, -10}, {-90, 10}})));Buildings.Fluid.Sensors.PPM[2] TRoo1(redeclare
package Medium = Medium)annotation (
Placement(transformation(origin={-542,-268},
extent = {{480, 216}, {500, 236}})));Modelica.Fluid.Interfaces.FluidPort_a[2]
port_a(redeclare package Medium
= Medium)annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));  Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
Modelica.Blocks.Sources.RealExpression
            TSupSetVav_in_control
            (y=293.15);
Modelica.Blocks.Sources.RealExpression
            THeaSetVav_in_control
            (y=293.15);
Modelica.Blocks.Sources.RealExpression
            THeaSetVav_in_control_2
            (y=293.15);
Modelica.Blocks.Sources.RealExpression
            TSupSetVav_in_control_2
            (y=293.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetVav_in_control
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            ppmCO2SetVav_in_control
            (y=0.0);
Modelica.Blocks.Sources.RealExpression
            TCooSetVav_in_control_2
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            ppmCO2SetVav_in_control_2
            (y=0.0);
Modelica.Blocks.Sources.IntegerExpression
            oveDamPosVav_in_control_2
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            oveFloSetVav_in_control
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            oveFloSetVav_in_control_2
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            uOpeModVav_in_control
            (y=1);
Modelica.Blocks.Sources.IntegerExpression
            oveDamPosVav_in_control
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            uAhuOpeModAhu_control
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            uOpeModVav_in_control_2
            (y=1);
Modelica.Blocks.Sources.BooleanExpression
            u1FanVav_in_control
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1FanVav_in_control_2
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1HotPlaVav_in_control
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1OccVav_in_control
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1OccVav_in_control_2
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1WinVav_in_control
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1HotPlaVav_in_control_2
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            uHeaOffVav_in_control
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1SupFanAhu_control
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1WinVav_in_control_2
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            uHeaOffVav_in_control_2
            (y=false);

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
connect(port[1],TRoo[1]. port);
connect(port_a[1], TRoo1[1].port);
connect(port[2],TRoo[2]. port);
connect(port_a[2], TRoo1[2].port);
connect(dataBus.TZonSpace_1, TRoo[1].T);
connect(dataBus.TZonSpace_2, TRoo[2].T);
connect(dataBus.ppmCO2Space_1, TRoo1[1].ppm);
connect(dataBus.ppmCO2Space_2, TRoo1[2].ppm);
connect(dataBus.TSupSetSpace_1,
TSupSetVav_in_control.y);
connect(dataBus.THeaSetSpace_1,
THeaSetVav_in_control.y);
connect(dataBus.THeaSetSpace_2,
THeaSetVav_in_control_2.y);
connect(dataBus.TSupSetSpace_2,
TSupSetVav_in_control_2.y);
connect(dataBus.TCooSetSpace_1,
TCooSetVav_in_control.y);
connect(dataBus.ppmCO2SetSpace_1,
ppmCO2SetVav_in_control.y);
connect(dataBus.TCooSetSpace_2,
TCooSetVav_in_control_2.y);
connect(dataBus.ppmCO2SetSpace_2,
ppmCO2SetVav_in_control_2.y);
connect(dataBus.oveDamPosSpace_2,
oveDamPosVav_in_control_2.y);
connect(dataBus.oveFloSetSpace_1,
oveFloSetVav_in_control.y);
connect(dataBus.oveFloSetSpace_2,
oveFloSetVav_in_control_2.y);
connect(dataBus.uOpeModSpace_1,
uOpeModVav_in_control.y);
connect(dataBus.oveDamPosSpace_1,
oveDamPosVav_in_control.y);
connect(dataBus.uAhuOpeModAhu_control,
uAhuOpeModAhu_control.y);
connect(dataBus.uOpeModSpace_2,
uOpeModVav_in_control_2.y);
connect(dataBus.u1FanSpace_1,
u1FanVav_in_control.y);
connect(dataBus.u1FanSpace_2,
u1FanVav_in_control_2.y);
connect(dataBus.u1HotPlaSpace_1,
u1HotPlaVav_in_control.y);
connect(dataBus.u1OccSpace_1,
u1OccVav_in_control.y);
connect(dataBus.u1OccSpace_2,
u1OccVav_in_control_2.y);
connect(dataBus.u1WinSpace_1,
u1WinVav_in_control.y);
connect(dataBus.u1HotPlaSpace_2,
u1HotPlaVav_in_control_2.y);
connect(dataBus.uHeaOffSpace_1,
uHeaOffVav_in_control.y);
connect(dataBus.u1SupFanAhu_control,
u1SupFanAhu_control.y);
connect(dataBus.u1WinSpace_2,
u1WinVav_in_control_2.y);
connect(dataBus.uHeaOffSpace_2,
uHeaOffVav_in_control_2.y);


connect(term_p, loa.terminal) annotation (Line(points={{92,0},{-32,0},{-32,-51},
{ -28,-51 } }, color={0,120,120}));

        connect(const.y, sum1.u[1]) annotation (Line(points={{-63,42},{-58,42},{-58,14},
    {-76,14},{-76,-20},{-74,-20},{-74,-26}}, color={0,0,127}));
connect(sum1.y, loa.Pow) annotation (Line(points={{-51,-26},{-38,-26},{-38,-76},
    {24,-76},{24,-49},{16,-49}}, color={0,0,127}));
end DataServer;
        model VAVBoxVav_in
extends two_spaces_air_handling_unit.Trano.Fluid.Ventilation.PartialVAVBox;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.yDamVav_in, vav.y);
connect(dataBus.y_actualVav_in, vav.y_actual);
connect(dataBus.VDis_flowVav_in_control, senVolFlo.V_flow);
connect(dataBus.TDisVav_in_control, senTem.T);
 end VAVBoxVav_in;
 
        model VAVBoxVav_in_2
extends two_spaces_air_handling_unit.Trano.Fluid.Ventilation.PartialVAVBox;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.yDamVav_in_2, vav.y);
connect(dataBus.y_actualVav_in_2, vav.y_actual);
connect(dataBus.VDis_flowVav_in_control_2, senVolFlo.V_flow);
connect(dataBus.TDisVav_in_control_2, senTem.T);
 end VAVBoxVav_in_2;
 
            model AhuAhu
    extends two_spaces_air_handling_unit.Trano.Fluid.Ventilation.PartialAhu;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.ySupFanAhu, fanSup1.y);
connect(dataBus.ySupFanAhu, fanSup.y);
connect(dataBus.yRetDamAhu, damRet.y);
connect(dataBus.yOutDamAhu, damOut.y);
connect(dataBus.yOutDamAhu, damExh.y);
connect(dataBus.u1SupFanAhu, fanSup.y_actual);
connect(dataBus.TOutAhu_control, TOut.T);
connect(dataBus.VAirOut_flowAhu_control, VOut1.V_flow);
connect(dataBus.TAirSupAhu_control, TSup.T);
connect(dataBus.TAirMixAhu_control, TMix.T);
connect(dataBus.dpDucAhu_control, dpDisSupFan.p_rel);
     end AhuAhu;
     
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
end two_spaces_air_handling_unit;