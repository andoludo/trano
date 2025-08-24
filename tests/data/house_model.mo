package house_model

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
    model distribution


extends Trano.BaseClasses.Containers.distribution;
parameter Real mRad_flow_nominal = 123;
package MediumW = Buildings.Media.Water;

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

Modelica.Fluid.Interfaces.FluidPort_a[8] port_a1(
    redeclare package Medium = MediumW)
    annotation (
        Placement(transformation(extent= {{-110,-58},{-90,-38}} )),
        iconTransformation(extent= {{-110,-58},{-90,-38}} )
    );

Modelica.Fluid.Interfaces.FluidPort_b[8] port_b1(
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
        Buildings.Fluid.Sensors.TemperatureTwoPort temperaturesensor_0(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { -32.701844767907815, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.FixedResistances.Junction splitvalve_0 (
        dp_nominal={10000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.008*{1,-1,-1},
    linearized=true
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { 100.0, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
      house_model.Components.BaseClasses.PumpPump_0
     pump_0(
         dp_nominal=10000.0,
    m_flow_nominal=0.008
,
    redeclare package Medium = MediumW

    ) annotation (
    Placement(transformation(origin = { -39.12580890150388, 27.59970446292425 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.CollectorControlCollectorcontrol_0
    collectorcontrol_0 annotation (
    Placement(transformation(origin = { -100.0, 99.31146149371986 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             threewayvalve_0(
    redeclare package Medium = MediumW,
          dpFixed_nominal={100,0},
    dpValve_nominal=6000.0,
    fraK=0.7,
    deltaM=0.02,
    m_flow_nominal=0.0078,
    delta0=0.01,
    R=50.0,
    linearized={true, true},
    l={0.01,0.01}
,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve"  annotation (
    Placement(transformation(origin = { 39.32669996185169, 27.854369389904605 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.
    ThreeWayValveControlThreewayvalvecontrol_0
    threewayvalvecontrol_0 annotation (
    Placement(transformation(origin = { 33.593955068999435, -99.5853048905098 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(splitvalve_0.port_1,port_a1[1])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[2])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[3])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[4])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[5])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[6])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[7])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_1,port_a1[8])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[1])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[2])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[3])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[4])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[5])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[6])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[7])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperaturesensor_0.port_b,port_b1[8])
        annotation (Line(
        points={{ -32.701844767907815, -100.0 }    ,{ -16.350922383953908, -100.0 }    ,{ -16.350922383953908, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(splitvalve_0.port_2,port_b)
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(pump_0.dataBus,collectorcontrol_0.dataBus)
        annotation (Line(
        points={{ -39.12580890150388, 27.59970446292425 }    ,{ -69.56290445075194, 27.59970446292425 }    ,{ -69.56290445075194, 99.31146149371986 }    ,{ -100.0, 99.31146149371986 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pump_0.port_b,temperaturesensor_0.port_a)
        annotation (Line(
        points={{ -39.12580890150388, 27.59970446292425 }    ,{ -35.913826834705844, 27.59970446292425 }    ,{ -35.913826834705844, -100.0 }    ,{ -32.701844767907815, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(threewayvalve_0.y,threewayvalvecontrol_0.y)
        annotation (Line(
        points={{ 39.32669996185169, 27.854369389904605 }    ,{ 36.46032751542556, 27.854369389904605 }    ,{ 36.46032751542556, -99.5853048905098 }    ,{ 33.593955068999435, -99.5853048905098 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(threewayvalve_0.port_2,pump_0.port_a)
        annotation (Line(
        points={{ 39.32669996185169, 27.854369389904605 }    ,{ 0.10044553017390001, 27.854369389904605 }    ,{ 0.10044553017390712, 27.59970446292425 }    ,{ -39.12580890150388, 27.59970446292425 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(threewayvalve_0.port_3,splitvalve_0.port_3)
        annotation (Line(
        points={{ 39.32669996185169, 27.854369389904605 }    ,{ 69.66334998092584, 27.854369389904605 }    ,{ 69.66334998092584, 100.0 }    ,{ 100.0, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(threewayvalvecontrol_0.u,temperaturesensor_0.T)
        annotation (Line(
        points={{ 33.593955068999435, -99.5853048905098 }    ,{ 0.4460551505458099, -99.5853048905098 }    ,{ 0.4460551505458099, -100.0 }    ,{ -32.701844767907815, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(threewayvalve_0.port_1,port_a)
        annotation (Line(
        points={{ 39.32669996185169, 27.854369389904605 }    ,{ 19.663349980925844, 27.854369389904605 }    ,{ 19.663349980925844, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(collectorcontrol_0.dataBus,dataBus)
            ;        
        connect(threewayvalvecontrol_0.dataBus,dataBus)
            ;end distribution;
    model emission


// Define Medium
extends Trano.BaseClasses.Containers.emission ;
package MediumW = Buildings.Media.Water;
// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_a[8] port_a(
  redeclare package Medium = MediumW)
 annotation (Placement(transformation(extent= {{90,44},{110,64}} ), iconTransformation(
      extent= {{90,44},{110,64}} )),iconTransformation(extent=  {{90,-60},{110,-40}} ));
Modelica.Fluid.Interfaces.FluidPort_b[8] port_b(
  redeclare package Medium = MediumW) annotation (Placement(transformation(extent= {{90,-64},{110,-44}} ), 
iconTransformation(extent= {{90,-64},{110,-44}} )),iconTransformation(extent=  {{90,40},{110,60}} ));
// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[8] heatPortCon
  annotation (
      Placement(transformation(extent= {{-108,42},{-88,62}} )),
      iconTransformation(extent= {{-108,42},{-88,62}} )
  );
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[8] heatPortRad
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
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_2(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=4000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { -2.8957338897410665, -14.285714285714292 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_2(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { -49.24081466731454, -14.285714285714292 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_2
    emissioncontrol_2(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -100.0, -14.285714285714292 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_1(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=2000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 65.700892729542, 14.285714285714278 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_1(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 19.35739987360141, 14.285714285714278 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_1
    emissioncontrol_1(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -31.402103043410605, 14.285714285714278 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_0(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=2000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 58.07886889166244, -42.85714285714286 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_0(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 11.735376035721856, -42.85714285714286 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_0
    emissioncontrol_0(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -39.02412688129016, -42.85714285714286 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_5(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=4000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 77.13392848636133, 42.85714285714286 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_5(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 30.790435630420745, 42.85714285714286 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_5
    emissioncontrol_5(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -19.969067286591283, 42.85714285714286 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_6(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=2000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 58.07886889166244, -71.42857142857143 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_6(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 11.735376035721856, -71.42857142857143 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_6
    emissioncontrol_6(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -39.02412688129016, -71.42857142857143 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_7(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=2000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 88.56696424318068, 71.42857142857142 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_7(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 42.22347138724007, 71.42857142857142 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_7
    emissioncontrol_7(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -8.536031529771932, 71.42857142857142 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_4(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=4000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 58.07886889166244, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_4(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 11.735376035721856, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_4
    emissioncontrol_4(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -39.02412688129016, -100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 radiator_3(
                TAir_nominal=293.15,
    dp_nominal=0.0,
    n=1.24,
    deltaM=0.01,
    fraRad=0.3,
    Q_flow_nominal=4000.0,
    nEle=1,
    TRad_nominal=293.15,
    linearized=true,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 100.0, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_3(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=true,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 53.648567535894955, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_3
    emissioncontrol_3(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { 2.8970042270473897, 100.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(radiator_2.heatPortRad,heatPortRad[1])
        annotation (Line(
        points={{ -2.8957338897410665, -14.285714285714292 }    ,{ -1.4478669448705332, -14.285714285714292 }    ,{ -1.4478669448705332, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_2.heatPortCon,heatPortCon[1])
        annotation (Line(
        points={{ -2.8957338897410665, -14.285714285714292 }    ,{ -1.4478669448705332, -14.285714285714292 }    ,{ -1.4478669448705332, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_2.port_b,port_b[1])
        annotation (Line(
        points={{ -2.8957338897410665, -14.285714285714292 }    ,{ -1.4478669448705332, -14.285714285714292 }    ,{ -1.4478669448705332, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_2.y,emissioncontrol_2.y)
        annotation (Line(
        points={{ -49.24081466731454, -14.285714285714292 }    ,{ -74.62040733365727, -14.285714285714292 }    ,{ -74.62040733365727, -14.285714285714292 }    ,{ -100.0, -14.285714285714292 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_2.port_b,radiator_2.port_a)
        annotation (Line(
        points={{ -49.24081466731454, -14.285714285714292 }    ,{ -26.068274278527802, -14.285714285714292 }    ,{ -26.068274278527802, -14.285714285714292 }    ,{ -2.8957338897410665, -14.285714285714292 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_1.heatPortRad,heatPortRad[2])
        annotation (Line(
        points={{ 65.700892729542, 14.285714285714278 }    ,{ 32.850446364771, 14.285714285714278 }    ,{ 32.850446364771, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_1.heatPortCon,heatPortCon[2])
        annotation (Line(
        points={{ 65.700892729542, 14.285714285714278 }    ,{ 32.850446364771, 14.285714285714278 }    ,{ 32.850446364771, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_1.port_b,port_b[2])
        annotation (Line(
        points={{ 65.700892729542, 14.285714285714278 }    ,{ 32.850446364771, 14.285714285714278 }    ,{ 32.850446364771, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_1.y,emissioncontrol_1.y)
        annotation (Line(
        points={{ 19.35739987360141, 14.285714285714278 }    ,{ -6.022351584904598, 14.285714285714278 }    ,{ -6.022351584904598, 14.285714285714278 }    ,{ -31.402103043410605, 14.285714285714278 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_1.port_b,radiator_1.port_a)
        annotation (Line(
        points={{ 19.35739987360141, 14.285714285714278 }    ,{ 42.52914630157171, 14.285714285714278 }    ,{ 42.52914630157171, 14.285714285714278 }    ,{ 65.700892729542, 14.285714285714278 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_0.heatPortRad,heatPortRad[3])
        annotation (Line(
        points={{ 58.07886889166244, -42.85714285714286 }    ,{ 29.03943444583122, -42.85714285714286 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_0.heatPortCon,heatPortCon[3])
        annotation (Line(
        points={{ 58.07886889166244, -42.85714285714286 }    ,{ 29.03943444583122, -42.85714285714286 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_0.port_b,port_b[3])
        annotation (Line(
        points={{ 58.07886889166244, -42.85714285714286 }    ,{ 29.03943444583122, -42.85714285714286 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_0.y,emissioncontrol_0.y)
        annotation (Line(
        points={{ 11.735376035721856, -42.85714285714286 }    ,{ -13.644375422784151, -42.85714285714286 }    ,{ -13.644375422784151, -42.85714285714286 }    ,{ -39.02412688129016, -42.85714285714286 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_0.port_b,radiator_0.port_a)
        annotation (Line(
        points={{ 11.735376035721856, -42.85714285714286 }    ,{ 34.90712246369215, -42.85714285714286 }    ,{ 34.90712246369215, -42.85714285714286 }    ,{ 58.07886889166244, -42.85714285714286 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_5.heatPortRad,heatPortRad[4])
        annotation (Line(
        points={{ 77.13392848636133, 42.85714285714286 }    ,{ 38.566964243180664, 42.85714285714286 }    ,{ 38.566964243180664, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_5.heatPortCon,heatPortCon[4])
        annotation (Line(
        points={{ 77.13392848636133, 42.85714285714286 }    ,{ 38.566964243180664, 42.85714285714286 }    ,{ 38.566964243180664, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_5.port_b,port_b[4])
        annotation (Line(
        points={{ 77.13392848636133, 42.85714285714286 }    ,{ 38.566964243180664, 42.85714285714286 }    ,{ 38.566964243180664, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_5.y,emissioncontrol_5.y)
        annotation (Line(
        points={{ 30.790435630420745, 42.85714285714286 }    ,{ 5.410684171914731, 42.85714285714286 }    ,{ 5.410684171914731, 42.85714285714286 }    ,{ -19.969067286591283, 42.85714285714286 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_5.port_b,radiator_5.port_a)
        annotation (Line(
        points={{ 30.790435630420745, 42.85714285714286 }    ,{ 53.962182058391036, 42.85714285714286 }    ,{ 53.962182058391036, 42.85714285714286 }    ,{ 77.13392848636133, 42.85714285714286 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_6.heatPortRad,heatPortRad[5])
        annotation (Line(
        points={{ 58.07886889166244, -71.42857142857143 }    ,{ 29.03943444583122, -71.42857142857143 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_6.heatPortCon,heatPortCon[5])
        annotation (Line(
        points={{ 58.07886889166244, -71.42857142857143 }    ,{ 29.03943444583122, -71.42857142857143 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_6.port_b,port_b[5])
        annotation (Line(
        points={{ 58.07886889166244, -71.42857142857143 }    ,{ 29.03943444583122, -71.42857142857143 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_6.y,emissioncontrol_6.y)
        annotation (Line(
        points={{ 11.735376035721856, -71.42857142857143 }    ,{ -13.644375422784151, -71.42857142857143 }    ,{ -13.644375422784151, -71.42857142857143 }    ,{ -39.02412688129016, -71.42857142857143 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_6.port_b,radiator_6.port_a)
        annotation (Line(
        points={{ 11.735376035721856, -71.42857142857143 }    ,{ 34.90712246369215, -71.42857142857143 }    ,{ 34.90712246369215, -71.42857142857143 }    ,{ 58.07886889166244, -71.42857142857143 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_7.heatPortRad,heatPortRad[6])
        annotation (Line(
        points={{ 88.56696424318068, 71.42857142857142 }    ,{ 44.28348212159034, 71.42857142857142 }    ,{ 44.28348212159034, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_7.heatPortCon,heatPortCon[6])
        annotation (Line(
        points={{ 88.56696424318068, 71.42857142857142 }    ,{ 44.28348212159034, 71.42857142857142 }    ,{ 44.28348212159034, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_7.port_b,port_b[6])
        annotation (Line(
        points={{ 88.56696424318068, 71.42857142857142 }    ,{ 44.28348212159034, 71.42857142857142 }    ,{ 44.28348212159034, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_7.y,emissioncontrol_7.y)
        annotation (Line(
        points={{ 42.22347138724007, 71.42857142857142 }    ,{ 16.843719928734068, 71.42857142857142 }    ,{ 16.843719928734068, 71.42857142857142 }    ,{ -8.536031529771932, 71.42857142857142 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_7.port_b,radiator_7.port_a)
        annotation (Line(
        points={{ 42.22347138724007, 71.42857142857142 }    ,{ 65.39521781521037, 71.42857142857142 }    ,{ 65.39521781521037, 71.42857142857142 }    ,{ 88.56696424318068, 71.42857142857142 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_4.heatPortRad,heatPortRad[7])
        annotation (Line(
        points={{ 58.07886889166244, -100.0 }    ,{ 29.03943444583122, -100.0 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_4.heatPortCon,heatPortCon[7])
        annotation (Line(
        points={{ 58.07886889166244, -100.0 }    ,{ 29.03943444583122, -100.0 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_4.port_b,port_b[7])
        annotation (Line(
        points={{ 58.07886889166244, -100.0 }    ,{ 29.03943444583122, -100.0 }    ,{ 29.03943444583122, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_4.y,emissioncontrol_4.y)
        annotation (Line(
        points={{ 11.735376035721856, -100.0 }    ,{ -13.644375422784151, -100.0 }    ,{ -13.644375422784151, -100.0 }    ,{ -39.02412688129016, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_4.port_b,radiator_4.port_a)
        annotation (Line(
        points={{ 11.735376035721856, -100.0 }    ,{ 34.90712246369215, -100.0 }    ,{ 34.90712246369215, -100.0 }    ,{ 58.07886889166244, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(radiator_3.heatPortRad,heatPortRad[8])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_3.heatPortCon,heatPortCon[8])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(radiator_3.port_b,port_b[8])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_3.y,emissioncontrol_3.y)
        annotation (Line(
        points={{ 53.648567535894955, 100.0 }    ,{ 28.272785881471172, 100.0 }    ,{ 28.272785881471172, 100.0 }    ,{ 2.8970042270473897, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_3.port_b,radiator_3.port_a)
        annotation (Line(
        points={{ 53.648567535894955, 100.0 }    ,{ 76.82428376794748, 100.0 }    ,{ 76.82428376794748, 100.0 }    ,{ 100.0, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_2.port_a,port_a[1])
        annotation (Line(
        points={{ -49.24081466731454, -14.285714285714292 }    ,{ -24.62040733365727, -14.285714285714292 }    ,{ -24.62040733365727, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_1.port_a,port_a[2])
        annotation (Line(
        points={{ 19.35739987360141, 14.285714285714278 }    ,{ 9.678699936800705, 14.285714285714278 }    ,{ 9.678699936800705, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_0.port_a,port_a[3])
        annotation (Line(
        points={{ 11.735376035721856, -42.85714285714286 }    ,{ 5.867688017860928, -42.85714285714286 }    ,{ 5.867688017860928, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_5.port_a,port_a[4])
        annotation (Line(
        points={{ 30.790435630420745, 42.85714285714286 }    ,{ 15.395217815210373, 42.85714285714286 }    ,{ 15.395217815210373, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_6.port_a,port_a[5])
        annotation (Line(
        points={{ 11.735376035721856, -71.42857142857143 }    ,{ 5.867688017860928, -71.42857142857143 }    ,{ 5.867688017860928, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_7.port_a,port_a[6])
        annotation (Line(
        points={{ 42.22347138724007, 71.42857142857142 }    ,{ 21.111735693620034, 71.42857142857142 }    ,{ 21.111735693620034, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_4.port_a,port_a[7])
        annotation (Line(
        points={{ 11.735376035721856, -100.0 }    ,{ 5.867688017860928, -100.0 }    ,{ 5.867688017860928, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_3.port_a,port_a[8])
        annotation (Line(
        points={{ 53.648567535894955, 100.0 }    ,{ 26.824283767947477, 100.0 }    ,{ 26.824283767947477, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emissioncontrol_2.dataBus,dataBus)
            ;        
        connect(emissioncontrol_1.dataBus,dataBus)
            ;        
        connect(emissioncontrol_0.dataBus,dataBus)
            ;        
        connect(emissioncontrol_5.dataBus,dataBus)
            ;        
        connect(emissioncontrol_6.dataBus,dataBus)
            ;        
        connect(emissioncontrol_7.dataBus,dataBus)
            ;        
        connect(emissioncontrol_4.dataBus,dataBus)
            ;        
        connect(emissioncontrol_3.dataBus,dataBus)
            ;end emission;
    model envelope

            parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic simple_glazing(
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
        Construction_1(
    final nLay=4,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.025,
        k=1.4,
        c=840.0,
        d=2100.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.05,
        k=0.046,
        c=940.0,
        d=80.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.24,
        k=0.046,
        c=940.0,
        d=80.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.02,
        k=0.9,
        c=840.0,
        d=975.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        construction_3(
    final nLay=1,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.04,
        k=0.131,
        c=1000.0,
        d=600.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        construction_4(
    final nLay=6,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.18,
        k=0.046,
        c=940.0,
        d=80.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.09,
        k=0.3,
        c=880.0,
        d=850.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.02,
        k=0.025,
        c=1005.0,
        d=1.2),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.05,
        k=0.046,
        c=940.0,
        d=80.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.14,
        k=0.89,
        c=800.0,
        d=1920.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.01,
        k=0.6,
        c=840.0,
        d=975.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        construction_5(
    final nLay=3,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.02,
        k=0.9,
        c=840.0,
        d=975.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.14,
        k=0.84,
        c=840.0,
        d=1400.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.02,
        k=0.9,
        c=840.0,
        d=975.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        construction_6(
    final nLay=5,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=1.4,
        c=840.0,
        d=2100.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=0.036,
        c=1470.0,
        d=26.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.02,
        k=0.036,
        c=1470.0,
        d=26.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.05,
        k=0.9,
        c=840.0,
        d=1100.0),Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.1,
        k=0.11,
        c=1880.0,
        d=550.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));

// Define Medium Package
extends Trano.BaseClasses.Containers.envelope ;
replaceable package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"})
  constrainedby Modelica.Media.Interfaces.PartialMedium

  annotation (choicesAllMatching = true);

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[8] heatPortCon
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[11] heatPortCon1
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{-4,98},{6,108}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[8] heatPortRad

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
Modelica.Fluid.Interfaces.FluidPorts_b[11] ports_b(
  redeclare package Medium = Medium)
  annotation (Placement(transformation(extent= {{-106,34},{-96,78}} ), iconTransformation(
      extent= {{-106,34},{-96,78}} )), iconTransformation(extent=  {{-106,30},{-92,86}} ));

Modelica.Fluid.Interfaces.FluidPorts_a[0] ports_a(
  redeclare package Medium = Medium)
  annotation ( Placement(transformation(extent= {{-104,-80},{-94,-34}} ), iconTransformation(
      extent= {{-104,-80},{-94,-34}} )),iconTransformation(extent=  {{-108,-92},{-94,-40}} ));
    Buildings.ThermalZones.Detailed.MixedAir space_3(
        redeclare package Medium = Medium,
            hRoo=2.4,
    AFlo=11.3,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=0,                    nSurBou=3,
                    surBou(
                    A={ 7.22, 6.5, 1.5 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 11.3 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=1,
                    datConExtWin(
                    layers={ construction_4 },
    A={ 9.024 },
    til={Buildings.Types.Tilt.Wall},
                    glaSys={ simple_glazing },
                    wWin={ 1.1419281938896158 },
                    hWin={ 1.1419281938896158 },
                    azi={ 90.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -25.52870567853961, -61.455908148620644 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_2
    occupancy_2(    gain=[35; 70; 30],
    k=0.15,
    occupancy=3600*{0.1,2,15,24}
) annotation (
    Placement(transformation(origin = { -40.52870567853961, -61.455908148620644 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_2(
        redeclare package Medium = Medium,
            hRoo=3.8,
    AFlo=2.7,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=1,
                    datConExt(
                    layers={ construction_4 },
    A={ 2.37 },
    til={Buildings.Types.Tilt.Wall},
                    azi={ 90.0 }),
                    nSurBou=3,
                    surBou(
                    A={ 3.64, 7.22, 7.22 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 2.7 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=0,        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 22.012009342395558, -53.3886142560995 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 7.012009342395558, -53.3886142560995 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_1(
        redeclare package Medium = Medium,
            hRoo=3.7,
    AFlo=7.07,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=0,                    nSurBou=2,
                    surBou(
                    A={ 6.44, 7.22 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 7.07 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=2,
                    datConExtWin(
                    layers={ construction_4, construction_4 },
    A={ 6.44, 7.224 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    glaSys={ simple_glazing, simple_glazing },
                    wWin={ 1.8346661821704786, 2.0707486568871656 },
                    hWin={ 1.8346661821704786, 2.0707486568871656 },
                    azi={ 90.0, 45.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 60.34110470374483, -44.61170467230108 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_0
    occupancy_0(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 45.34110470374483, -44.61170467230108 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_6(
        redeclare package Medium = Medium,
            hRoo=2.4,
    AFlo=5.7,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=0,                    nSurBou=3,
                    surBou(
                    A={ 6.5, 6.5, 5.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 5.7 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=0,        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -46.355870625894944, -5.314941795566895 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
        house_model.Components.BaseClasses.OccupancyOccupancy_5
    occupancy_5(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -61.355870625894944, -5.314941795566895 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_9(
        redeclare package Medium = Medium,
            hRoo=3.7,
    AFlo=15.7,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=2,
                    datConExt(
                    layers={ Construction_1, Construction_1 },
    A={ 9.0, 9.0 },
    til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Ceiling},
                    azi={ 0.0, 90.0 }),
                    nSurBou=1,
                    surBou(
                    A={ 15.7 },
                    til={Buildings.Types.Tilt.Floor}),
                    nConBou=0,                    nConExtWin=1,
                    datConExtWin(
                    layers={ construction_4 },
    A={ 12.938 },
    til={Buildings.Types.Tilt.Wall},
                    glaSys={ simple_glazing },
                    wWin={ 1.4028542333400145 },
                    hWin={ 1.4028542333400145 },
                    azi={ 45.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 4.2000371756224695, 61.23425291022164 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_8
    occupancy_8(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -10.79996282437753, 61.23425291022164 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_7(
        redeclare package Medium = Medium,
            hRoo=2.4,
    AFlo=5.7,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=0,                    nSurBou=3,
                    surBou(
                    A={ 6.5, 5.0, 6.5 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 5.7 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=0,        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -38.35796891794694, 23.19087864774356 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
        house_model.Components.BaseClasses.OccupancyOccupancy_6
    occupancy_6(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -53.35796891794694, 23.19087864774356 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_8(
        redeclare package Medium = Medium,
            hRoo=6.5,
    AFlo=3.139,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=0,                    nSurBou=5,
                    surBou(
                    A={ 8.0, 1.5, 5.0, 5.0, 12.321 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 3.19 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=0,        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -27.812141146875305, 19.09583798437251 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
        house_model.Components.BaseClasses.OccupancyOccupancy_7
    occupancy_7(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -42.812141146875305, 19.09583798437251 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_10(
        redeclare package Medium = Medium,
            hRoo=3.7,
    AFlo=11.8116,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=2,
                    datConExt(
                    layers={ Construction_1, Construction_1 },
    A={ 9.0, 9.0 },
    til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Ceiling},
                    azi={ 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 12.321, 3.33 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Floor}),
                    nConBou=0,                    nConExtWin=0,        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -65.98841972012487, 0.5118800829213797 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_9
    occupancy_9(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -80.98841972012487, 0.5118800829213797 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_11(
        redeclare package Medium = Medium,
            hRoo=1.6,
    AFlo=6.6,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=2,
                    datConExt(
                    layers={ Construction_1, Construction_1 },
    A={ 9.0, 3.16 },
    til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Ceiling},
                    azi={ 90.0, 45.0 }),
                    nSurBou=0,                    nConBou=0,                    nConExtWin=0,        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -52.15115343613691, -57.6064423536916 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_10
    occupancy_10(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -67.15115343613691, -57.6064423536916 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_5(
        redeclare package Medium = Medium,
            hRoo=3.7,
    AFlo=11.34,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=1,
                    datConExt(
                    layers={ Construction_1 },
    A={ 13.0 },
    til={Buildings.Types.Tilt.Ceiling},
                    azi={ 0.0 }),
                    nSurBou=3,
                    surBou(
                    A={ 11.368, 6.5, 3.33 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Floor}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 11.34 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=1,
                    datConExtWin(
                    layers={ construction_4 },
    A={ 11.34 },
    til={Buildings.Types.Tilt.Wall},
                    glaSys={ simple_glazing },
                    wWin={ 1.6825575770237404 },
                    hWin={ 1.6825575770237404 },
                    azi={ 0.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { -16.027615756880238, -25.62589698612662 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_4
    occupancy_4(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -31.027615756880238, -25.62589698612662 },
    extent = {{ 3, -3}, {-3, 3}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_4(
        redeclare package Medium = Medium,
            hRoo=3.7,
    AFlo=27.11,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=299.15
,nPorts = 3,                    nConExt=1,
                    datConExt(
                    layers={ Construction_1 },
    A={ 13.0 },
    til={Buildings.Types.Tilt.Ceiling},
                    azi={ 0.0 }),
                    nSurBou=5,
                    surBou(
                    A={ 11.368, 3.64, 6.44, 8.0, 15.7 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Floor}),
                    nConBou=1,
                    datConBou(
                    layers={ construction_6 },
    A={ 27.11 },
    til={Buildings.Types.Tilt.Floor},
                    azi={ 90.0 }),
                    nConExtWin=2,
                    datConExtWin(
                    layers={ construction_4, construction_4 },
    A={ 8.664, 18.024 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    glaSys={ simple_glazing, simple_glazing },
                    wWin={ 2.209298531208492, 2.0707486568871656 },
                    hWin={ 2.209298531208492, 2.0707486568871656 },
                    azi={ 0.0, 45.0 }),
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 43.472482325048674, 16.911178440440125 },
    extent = {{ 5, -5}, {-5, 5}}
)));
    
    
    
    
    
    
        house_model.Components.BaseClasses.OccupancyOccupancy_3
    occupancy_3(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{9, 12, 17,22}
) annotation (
    Placement(transformation(origin = { 28.472482325048674, 16.911178440440125 },
    extent = {{ 3, -3}, {-3, 3}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather_0(    filNam=/home/aan/Documents/trano/tests/resources/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos
)
     annotation (
    Placement(transformation(origin = { -7.430202034079599, -10.872269175570096 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_0(A =
            11.368, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 16.063909744053916, -4.519215436134587 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_1(A =
            3.64, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 38.8292029943274, -20.975920905756666 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_2(A =
            6.44, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 65.45458953631126, -12.095359591771654 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_3(A =
            8, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 9.554502327884592, 26.589060755860316 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_4(A =
            7.22, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 46.42596063351377, -63.89252112900654 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_5(A =
            7.22, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 2.2148883225198404, -80.1610588422899 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_6(A =
            6.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -52.111479846918726, -40.29341412852815 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_7(A =
            1.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -32.92415658724846, -21.71583479508851 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_8(A =
            6.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -69.81765577577663, 20.841970977515544 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_9(A =
            5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -51.99833664803722, 16.050071758890127 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_10(A =
            5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -37.92743700531993, 49.08308084834954 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_11(A =
            6.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -16.325902372113262, 6.939881996491778 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_12(A =
            12.321, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -61.293711662639296, 36.65603571998085 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_13(A =
            3.33, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -58.807500071632866, -21.674374103013875 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_14(A =
            15.7, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 29.1047654592916, 49.722532291500556 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(space_3.heaPorRad,heatPortRad[1])
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -12.764352839269804, -61.455908148620644 }    ,{ -12.764352839269804, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_3.heaPorAir,heatPortCon[1])
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -12.764352839269804, -61.455908148620644 }    ,{ -12.764352839269804, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_3.qGai_flow,occupancy_2.y)
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -33.02870567853961, -61.455908148620644 }    ,{ -33.02870567853961, -61.455908148620644 }    ,{ -40.52870567853961, -61.455908148620644 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -16.479453856309604, -61.455908148620644 }    ,{ -16.479453856309604, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.surf_surBou[1],internalelement_5.port_a)
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -11.656908678009884, -61.455908148620644 }    ,{ -11.656908678009884, -80.1610588422899 }    ,{ 2.2148883225198404, -80.1610588422899 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.surf_surBou[2],internalelement_6.port_a)
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -38.82009276272917, -61.455908148620644 }    ,{ -38.82009276272917, -40.29341412852815 }    ,{ -52.111479846918726, -40.29341412852815 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.surf_surBou[3],internalelement_7.port_a)
        annotation (Line(
        points={{ -25.52870567853961, -61.455908148620644 }    ,{ -29.226431132894035, -61.455908148620644 }    ,{ -29.226431132894035, -21.71583479508851 }    ,{ -32.92415658724846, -21.71583479508851 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.heaPorRad,heatPortRad[2])
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 11.006004671197779, -53.3886142560995 }    ,{ 11.006004671197779, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_2.heaPorAir,heatPortCon[2])
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 11.006004671197779, -53.3886142560995 }    ,{ 11.006004671197779, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_2.qGai_flow,occupancy_1.y)
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 14.512009342395558, -53.3886142560995 }    ,{ 14.512009342395558, -53.3886142560995 }    ,{ 7.012009342395558, -53.3886142560995 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 7.2909036541579795, -53.3886142560995 }    ,{ 7.2909036541579795, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.surf_surBou[1],internalelement_1.port_a)
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 30.42060616836148, -53.3886142560995 }    ,{ 30.42060616836148, -20.975920905756666 }    ,{ 38.8292029943274, -20.975920905756666 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.surf_surBou[2],internalelement_4.port_a)
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 34.21898498795466, -53.3886142560995 }    ,{ 34.21898498795466, -63.89252112900654 }    ,{ 46.42596063351377, -63.89252112900654 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.surf_surBou[3],internalelement_5.port_b)
        annotation (Line(
        points={{ 22.012009342395558, -53.3886142560995 }    ,{ 12.1134488324577, -53.3886142560995 }    ,{ 12.1134488324577, -80.1610588422899 }    ,{ 2.2148883225198404, -80.1610588422899 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.heaPorRad,heatPortRad[3])
        annotation (Line(
        points={{ 60.34110470374483, -44.61170467230108 }    ,{ 30.170552351872416, -44.61170467230108 }    ,{ 30.170552351872416, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_1.heaPorAir,heatPortCon[3])
        annotation (Line(
        points={{ 60.34110470374483, -44.61170467230108 }    ,{ 30.170552351872416, -44.61170467230108 }    ,{ 30.170552351872416, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_1.qGai_flow,occupancy_0.y)
        annotation (Line(
        points={{ 60.34110470374483, -44.61170467230108 }    ,{ 52.84110470374483, -44.61170467230108 }    ,{ 52.84110470374483, -44.61170467230108 }    ,{ 45.34110470374483, -44.61170467230108 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ 60.34110470374483, -44.61170467230108 }    ,{ 26.455451334832617, -44.61170467230108 }    ,{ 26.455451334832617, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.surf_surBou[1],internalelement_2.port_a)
        annotation (Line(
        points={{ 60.34110470374483, -44.61170467230108 }    ,{ 62.89784712002805, -44.61170467230108 }    ,{ 62.89784712002805, -12.095359591771654 }    ,{ 65.45458953631126, -12.095359591771654 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.surf_surBou[2],internalelement_4.port_b)
        annotation (Line(
        points={{ 60.34110470374483, -44.61170467230108 }    ,{ 53.3835326686293, -44.61170467230108 }    ,{ 53.3835326686293, -63.89252112900654 }    ,{ 46.42596063351377, -63.89252112900654 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_6.heaPorRad,heatPortRad[4])
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -23.177935312947472, -5.314941795566895 }    ,{ -23.177935312947472, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_6.heaPorAir,heatPortCon[4])
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -23.177935312947472, -5.314941795566895 }    ,{ -23.177935312947472, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_6.qGai_flow,occupancy_5.y)
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -53.855870625894944, -5.314941795566895 }    ,{ -53.855870625894944, -5.314941795566895 }    ,{ -61.355870625894944, -5.314941795566895 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_6.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -26.89303632998727, -5.314941795566895 }    ,{ -26.89303632998727, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_6.surf_surBou[1],internalelement_6.port_b)
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -49.23367523640684, -5.314941795566895 }    ,{ -49.23367523640684, -40.29341412852815 }    ,{ -52.111479846918726, -40.29341412852815 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_6.surf_surBou[2],internalelement_8.port_a)
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -58.08676320083579, -5.314941795566895 }    ,{ -58.08676320083579, 20.841970977515544 }    ,{ -69.81765577577663, 20.841970977515544 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_6.surf_surBou[3],internalelement_9.port_a)
        annotation (Line(
        points={{ -46.355870625894944, -5.314941795566895 }    ,{ -49.17710363696608, -5.314941795566895 }    ,{ -49.17710363696608, 16.050071758890127 }    ,{ -51.99833664803722, 16.050071758890127 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_9.heaPorRad,heatPortRad[5])
        annotation (Line(
        points={{ 4.2000371756224695, 61.23425291022164 }    ,{ 2.1000185878112347, 61.23425291022164 }    ,{ 2.1000185878112347, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_9.heaPorAir,heatPortCon[5])
        annotation (Line(
        points={{ 4.2000371756224695, 61.23425291022164 }    ,{ 2.1000185878112347, 61.23425291022164 }    ,{ 2.1000185878112347, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_9.qGai_flow,occupancy_8.y)
        annotation (Line(
        points={{ 4.2000371756224695, 61.23425291022164 }    ,{ -3.2999628243775305, 61.23425291022164 }    ,{ -3.2999628243775305, 61.23425291022164 }    ,{ -10.79996282437753, 61.23425291022164 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_9.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ 4.2000371756224695, 61.23425291022164 }    ,{ -1.6150824292285648, 61.23425291022164 }    ,{ -1.6150824292285648, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_9.surf_surBou[1],internalelement_14.port_a)
        annotation (Line(
        points={{ 4.2000371756224695, 61.23425291022164 }    ,{ 16.652401317457034, 61.23425291022164 }    ,{ 16.652401317457034, 49.722532291500556 }    ,{ 29.1047654592916, 49.722532291500556 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_7.qGai_flow,occupancy_6.y)
        annotation (Line(
        points={{ -38.35796891794694, 23.19087864774356 }    ,{ -45.85796891794694, 23.19087864774356 }    ,{ -45.85796891794694, 23.19087864774356 }    ,{ -53.35796891794694, 23.19087864774356 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_7.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -38.35796891794694, 23.19087864774356 }    ,{ -22.89408547601327, 23.19087864774356 }    ,{ -22.89408547601327, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_7.surf_surBou[1],internalelement_8.port_b)
        annotation (Line(
        points={{ -38.35796891794694, 23.19087864774356 }    ,{ -54.08781234686178, 23.19087864774356 }    ,{ -54.08781234686178, 20.841970977515544 }    ,{ -69.81765577577663, 20.841970977515544 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_7.surf_surBou[2],internalelement_10.port_a)
        annotation (Line(
        points={{ -38.35796891794694, 23.19087864774356 }    ,{ -38.14270296163343, 23.19087864774356 }    ,{ -38.14270296163343, 49.08308084834954 }    ,{ -37.92743700531993, 49.08308084834954 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_7.surf_surBou[3],internalelement_11.port_a)
        annotation (Line(
        points={{ -38.35796891794694, 23.19087864774356 }    ,{ -27.3419356450301, 23.19087864774356 }    ,{ -27.3419356450301, 6.939881996491778 }    ,{ -16.325902372113262, 6.939881996491778 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.qGai_flow,occupancy_7.y)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -35.312141146875305, 19.09583798437251 }    ,{ -35.312141146875305, 19.09583798437251 }    ,{ -42.812141146875305, 19.09583798437251 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -17.621171590477452, 19.09583798437251 }    ,{ -17.621171590477452, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.surf_surBou[1],internalelement_3.port_a)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -9.128819409495357, 19.09583798437251 }    ,{ -9.128819409495357, 26.589060755860316 }    ,{ 9.554502327884592, 26.589060755860316 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.surf_surBou[2],internalelement_7.port_b)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -30.368148867061883, 19.09583798437251 }    ,{ -30.368148867061883, -21.71583479508851 }    ,{ -32.92415658724846, -21.71583479508851 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.surf_surBou[3],internalelement_9.port_b)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -39.90523889745626, 19.09583798437251 }    ,{ -39.90523889745626, 16.050071758890127 }    ,{ -51.99833664803722, 16.050071758890127 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.surf_surBou[4],internalelement_10.port_b)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -32.86978907609762, 19.09583798437251 }    ,{ -32.86978907609762, 49.08308084834954 }    ,{ -37.92743700531993, 49.08308084834954 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_8.surf_surBou[5],internalelement_12.port_a)
        annotation (Line(
        points={{ -27.812141146875305, 19.09583798437251 }    ,{ -44.5529264047573, 19.09583798437251 }    ,{ -44.5529264047573, 36.65603571998085 }    ,{ -61.293711662639296, 36.65603571998085 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_10.heaPorRad,heatPortRad[6])
        annotation (Line(
        points={{ -65.98841972012487, 0.5118800829213797 }    ,{ -32.99420986006243, 0.5118800829213797 }    ,{ -32.99420986006243, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_10.heaPorAir,heatPortCon[6])
        annotation (Line(
        points={{ -65.98841972012487, 0.5118800829213797 }    ,{ -32.99420986006243, 0.5118800829213797 }    ,{ -32.99420986006243, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_10.qGai_flow,occupancy_9.y)
        annotation (Line(
        points={{ -65.98841972012487, 0.5118800829213797 }    ,{ -73.48841972012487, 0.5118800829213797 }    ,{ -73.48841972012487, 0.5118800829213797 }    ,{ -80.98841972012487, 0.5118800829213797 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_10.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -65.98841972012487, 0.5118800829213797 }    ,{ -36.70931087710223, 0.5118800829213797 }    ,{ -36.70931087710223, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_10.surf_surBou[1],internalelement_12.port_b)
        annotation (Line(
        points={{ -65.98841972012487, 0.5118800829213797 }    ,{ -63.64106569138208, 0.5118800829213797 }    ,{ -63.64106569138208, 36.65603571998085 }    ,{ -61.293711662639296, 36.65603571998085 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_10.surf_surBou[2],internalelement_13.port_a)
        annotation (Line(
        points={{ -65.98841972012487, 0.5118800829213797 }    ,{ -62.39795989587887, 0.5118800829213797 }    ,{ -62.39795989587887, -21.674374103013875 }    ,{ -58.807500071632866, -21.674374103013875 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_11.qGai_flow,occupancy_10.y)
        annotation (Line(
        points={{ -52.15115343613691, -57.6064423536916 }    ,{ -59.65115343613691, -57.6064423536916 }    ,{ -59.65115343613691, -57.6064423536916 }    ,{ -67.15115343613691, -57.6064423536916 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_11.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -52.15115343613691, -57.6064423536916 }    ,{ -29.790677735108254, -57.6064423536916 }    ,{ -29.790677735108254, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_5.heaPorRad,heatPortRad[7])
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ -8.013807878440119, -25.62589698612662 }    ,{ -8.013807878440119, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_5.heaPorAir,heatPortCon[7])
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ -8.013807878440119, -25.62589698612662 }    ,{ -8.013807878440119, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_5.qGai_flow,occupancy_4.y)
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ -23.527615756880238, -25.62589698612662 }    ,{ -23.527615756880238, -25.62589698612662 }    ,{ -31.027615756880238, -25.62589698612662 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_5.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ -11.728908895479918, -25.62589698612662 }    ,{ -11.728908895479918, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_5.surf_surBou[1],internalelement_0.port_a)
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ 0.018146993586839244, -25.62589698612662 }    ,{ 0.018146993586839244, -4.519215436134587 }    ,{ 16.063909744053916, -4.519215436134587 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_5.surf_surBou[2],internalelement_11.port_b)
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ -16.17675906449675, -25.62589698612662 }    ,{ -16.17675906449675, 6.939881996491778 }    ,{ -16.325902372113262, 6.939881996491778 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_5.surf_surBou[3],internalelement_13.port_b)
        annotation (Line(
        points={{ -16.027615756880238, -25.62589698612662 }    ,{ -37.41755791425655, -25.62589698612662 }    ,{ -37.41755791425655, -21.674374103013875 }    ,{ -58.807500071632866, -21.674374103013875 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.heaPorRad,heatPortRad[8])
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 21.736241162524337, 16.911178440440125 }    ,{ 21.736241162524337, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_4.heaPorAir,heatPortCon[8])
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 21.736241162524337, 16.911178440440125 }    ,{ 21.736241162524337, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_4.qGai_flow,occupancy_3.y)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 35.972482325048674, 16.911178440440125 }    ,{ 35.972482325048674, 16.911178440440125 }    ,{ 28.472482325048674, 16.911178440440125 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.weaBus,weather_0.weaBus)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 18.021140145484537, 16.911178440440125 }    ,{ 18.021140145484537, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.surf_surBou[1],internalelement_0.port_b)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 29.768196034551295, 16.911178440440125 }    ,{ 29.768196034551295, -4.519215436134587 }    ,{ 16.063909744053916, -4.519215436134587 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.surf_surBou[2],internalelement_1.port_b)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 41.15084265968804, 16.911178440440125 }    ,{ 41.15084265968804, -20.975920905756666 }    ,{ 38.8292029943274, -20.975920905756666 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.surf_surBou[3],internalelement_2.port_b)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 54.46353593067997, 16.911178440440125 }    ,{ 54.46353593067997, -12.095359591771654 }    ,{ 65.45458953631126, -12.095359591771654 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.surf_surBou[4],internalelement_3.port_b)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 26.513492326466633, 16.911178440440125 }    ,{ 26.513492326466633, 26.589060755860316 }    ,{ 9.554502327884592, 26.589060755860316 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_4.surf_surBou[5],internalelement_14.port_b)
        annotation (Line(
        points={{ 43.472482325048674, 16.911178440440125 }    ,{ 36.288623892170136, 16.911178440440125 }    ,{ 36.288623892170136, 49.722532291500556 }    ,{ 29.1047654592916, 49.722532291500556 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(occupancy_2.dataBus,dataBus)
            ;        
        connect(occupancy_1.dataBus,dataBus)
            ;        
        connect(occupancy_0.dataBus,dataBus)
            ;        
        connect(occupancy_5.dataBus,dataBus)
            ;        
        connect(occupancy_8.dataBus,dataBus)
            ;        
        connect(occupancy_6.dataBus,dataBus)
            ;        
        connect(occupancy_7.dataBus,dataBus)
            ;        
        connect(occupancy_9.dataBus,dataBus)
            ;        
        connect(occupancy_10.dataBus,dataBus)
            ;        
        connect(occupancy_4.dataBus,dataBus)
            ;        
        connect(occupancy_3.dataBus,dataBus)
            ;        
        connect(space_1.heaPorAir,heatPortCon1[1])
            ;        
        connect(space_1.ports[1],ports_b[1])
            ;        
        connect(space_10.heaPorAir,heatPortCon1[2])
            ;        
        connect(space_10.ports[1],ports_b[2])
            ;        
        connect(space_11.heaPorAir,heatPortCon1[3])
            ;        
        connect(space_11.ports[1],ports_b[3])
            ;        
        connect(space_2.heaPorAir,heatPortCon1[4])
            ;        
        connect(space_2.ports[1],ports_b[4])
            ;        
        connect(space_3.heaPorAir,heatPortCon1[5])
            ;        
        connect(space_3.ports[1],ports_b[5])
            ;        
        connect(space_4.heaPorAir,heatPortCon1[6])
            ;        
        connect(space_4.ports[1],ports_b[6])
            ;        
        connect(space_5.heaPorAir,heatPortCon1[7])
            ;        
        connect(space_5.ports[1],ports_b[7])
            ;        
        connect(space_6.heaPorAir,heatPortCon1[8])
            ;        
        connect(space_6.ports[1],ports_b[8])
            ;        
        connect(space_7.heaPorAir,heatPortCon1[9])
            ;        
        connect(space_7.ports[1],ports_b[9])
            ;        
        connect(space_8.heaPorAir,heatPortCon1[10])
            ;        
        connect(space_8.ports[1],ports_b[10])
            ;        
        connect(space_9.heaPorAir,heatPortCon1[11])
            ;        
        connect(space_9.ports[1],ports_b[11])
            ;        
        connect(weather_0.weaBus,dataBus)
        annotation (Line(
        points={{ -7.430202034079599, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    ,{ -7.430202034079599, -10.872269175570096 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;end envelope;
    model production


extends Trano.BaseClasses.Containers.production;
// Define Parameter
parameter Real mRad_flow_nominal = 123;

// Define Medium Package
package MediumW = Buildings.Media.Water;

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
    house_model.Components.BaseClasses.
BoilerWithStorageBoiler_0 boiler_0(
    a={0.9},
    dp=5000*{2,1},
    dp_nominal=5000.0,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    deltaM=0.1,
    hTan=2.0,
    show_T=false,
    Q_flow_nominal=6000.0,
    nSeg=4,
    VTan=0.2,
    T_nominal=353.15,
    dIns=0.002,
    linearizeFlowResistance=true,
    nominal_mass_flow_radiator_loop=0.21428571428571427,
    nominal_mass_flow_rate_boiler=0.21428571428571427,
    V_flow=0.21428571428571427/1000*{0.5,1}
,
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()) "Boiler"  annotation (
    Placement(transformation(origin = { 100.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
        house_model.Components.BaseClasses.CollectorControlCollectorcontrol_1
    collectorcontrol_1 annotation (
    Placement(transformation(origin = { -100.0, 0.0 },
    extent = {{ 5, -5}, {-5, 5}}
)));
equation        
        connect(boiler_0.port_a,port_a1)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(boiler_0.dataBus,collectorcontrol_1.dataBus)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ -100.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(boiler_0.port_b,port_b1)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(collectorcontrol_1.dataBus,dataBus)
            ;end production;
    model bus


extends Trano.BaseClasses.Containers.bus;
// Define Medium Package
package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"});

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_b[11] port_b(
  redeclare package Medium = Medium)
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[11] heatPortCon

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
        house_model.Components.BaseClasses.DataServer
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
        connect(data_bus.port[3],heatPortCon[3])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[3],port_b[3])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[4],heatPortCon[4])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[4],port_b[4])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[5],heatPortCon[5])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[5],port_b[5])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[6],heatPortCon[6])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[6],port_b[6])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[7],heatPortCon[7])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[7],port_b[7])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[8],heatPortCon[8])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[8],port_b[8])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[9],heatPortCon[9])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[9],port_b[9])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[10],heatPortCon[10])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[10],port_b[10])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port[11],heatPortCon[11])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(data_bus.port_a[11],port_b[11])
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
connect(distribution1.port_a1[4],
emission1.port_b[4])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a1[5],
emission1.port_b[5])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a1[6],
emission1.port_b[6])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a1[7],
emission1.port_b[7])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_a1[8],
emission1.port_b[8])
annotation (Line(points={{-24,
          4.6},{-8,4.6},{-8,5.2},{-4,5.2}}, color={0,127,255}));
connect(distribution1.port_b1[1],
emission1.port_a[1])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[2],
emission1.port_a[2])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[3],
emission1.port_a[3])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[4],
emission1.port_a[4])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[5],
emission1.port_a[5])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[6],
emission1.port_a[6])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[7],
emission1.port_a[7])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[8],
emission1.port_a[8])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b,
production1.port_a1)
annotation (Line(points={{16,5},{32,5},{32,5.2},{36,5.2}},
                                          color={0,127,255}));
connect(distribution1.port_a,
production1.port_b1)
annotation (Line(points={{15.8,15},{30,15},{30,15},{36,15}},
                                          color={0,127,255}));
connect(emission1.heatPortRad[1],
envelope1.heatPortRad[1])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[1],
envelope1.heatPortCon[1])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[2],
envelope1.heatPortRad[2])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[2],
envelope1.heatPortCon[2])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[3],
envelope1.heatPortRad[3])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[3],
envelope1.heatPortCon[3])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[4],
envelope1.heatPortRad[4])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[4],
envelope1.heatPortCon[4])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[5],
envelope1.heatPortRad[5])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[5],
envelope1.heatPortCon[5])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[6],
envelope1.heatPortRad[6])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[6],
envelope1.heatPortCon[6])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[7],
envelope1.heatPortRad[7])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[7],
envelope1.heatPortCon[7])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(emission1.heatPortRad[8],
envelope1.heatPortRad[8])
annotation (Line(points
        ={{-64,4.8},{-48,4.8},{-48,5},{-44,5}}, color={191,0,0}));
connect(emission1.heatPortCon[8],
envelope1.heatPortCon[8])
annotation (Line(points={{-64,15},{-62,15.2},{-43.8,15.2}}, color={191,0,0}));
connect(envelope1.heatPortCon1[1],
bus1.heatPortCon[1])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[1],
bus1.port_b[1])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[2],
bus1.heatPortCon[2])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[2],
bus1.port_b[2])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[3],
bus1.heatPortCon[3])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[3],
bus1.port_b[3])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[4],
bus1.heatPortCon[4])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[4],
bus1.port_b[4])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[5],
bus1.heatPortCon[5])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[5],
bus1.port_b[5])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[6],
bus1.heatPortCon[6])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[6],
bus1.port_b[6])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[7],
bus1.heatPortCon[7])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[7],
bus1.port_b[7])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[8],
bus1.heatPortCon[8])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[8],
bus1.port_b[8])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[9],
bus1.heatPortCon[9])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[9],
bus1.port_b[9])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[10],
bus1.heatPortCon[10])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[10],
bus1.port_b[10])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.heatPortCon1[11],
bus1.heatPortCon[11])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));
connect(envelope1.ports_b[11],
bus1.port_b[11])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));

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
        model EmissionControlEmissioncontrol_2
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_3, emissionControl.TCooSet);
connect(dataBus.TZonSpace_3, emissionControl.TZon);
connect(dataBus.yCooValve_2, emissionControl.yCoo);
connect(dataBus.yHeaValve_2, emissionControl.yHea);
end EmissionControlEmissioncontrol_2;
        model OccupancyOccupancy_2
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_3, occSch2.occupied);
 end OccupancyOccupancy_2;
 
        model EmissionControlEmissioncontrol_1
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_2, emissionControl.TCooSet);
connect(dataBus.TZonSpace_2, emissionControl.TZon);
connect(dataBus.yCooValve_1, emissionControl.yCoo);
connect(dataBus.yHeaValve_1, emissionControl.yHea);
end EmissionControlEmissioncontrol_1;
        model OccupancyOccupancy_1
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_2, occSch2.occupied);
 end OccupancyOccupancy_1;
 
        model EmissionControlEmissioncontrol_0
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_1, emissionControl.TCooSet);
connect(dataBus.TZonSpace_1, emissionControl.TZon);
connect(dataBus.yCooValve_0, emissionControl.yCoo);
connect(dataBus.yHeaValve_0, emissionControl.yHea);
end EmissionControlEmissioncontrol_0;
        model OccupancyOccupancy_0
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_1, occSch2.occupied);
 end OccupancyOccupancy_0;
 
        model EmissionControlEmissioncontrol_5
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_6, emissionControl.TCooSet);
connect(dataBus.TZonSpace_6, emissionControl.TZon);
connect(dataBus.yCooValve_5, emissionControl.yCoo);
connect(dataBus.yHeaValve_5, emissionControl.yHea);
end EmissionControlEmissioncontrol_5;
        model OccupancyOccupancy_5
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_6, occSch2.occupied);
 end OccupancyOccupancy_5;
 
        model EmissionControlEmissioncontrol_6
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_9, emissionControl.TCooSet);
connect(dataBus.TZonSpace_9, emissionControl.TZon);
connect(dataBus.yCooValve_6, emissionControl.yCoo);
connect(dataBus.yHeaValve_6, emissionControl.yHea);
end EmissionControlEmissioncontrol_6;
        model OccupancyOccupancy_8
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_9, occSch2.occupied);
 end OccupancyOccupancy_8;
 
        model OccupancyOccupancy_6
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_7, occSch2.occupied);
 end OccupancyOccupancy_6;
 
        model OccupancyOccupancy_7
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_8, occSch2.occupied);
 end OccupancyOccupancy_7;
 
        model EmissionControlEmissioncontrol_7
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_10, emissionControl.TCooSet);
connect(dataBus.TZonSpace_10, emissionControl.TZon);
connect(dataBus.yCooValve_7, emissionControl.yCoo);
connect(dataBus.yHeaValve_7, emissionControl.yHea);
end EmissionControlEmissioncontrol_7;
        model OccupancyOccupancy_9
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_10, occSch2.occupied);
 end OccupancyOccupancy_9;
 
        model OccupancyOccupancy_10
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_11, occSch2.occupied);
 end OccupancyOccupancy_10;
 
        model EmissionControlEmissioncontrol_4
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_5, emissionControl.TCooSet);
connect(dataBus.TZonSpace_5, emissionControl.TZon);
connect(dataBus.yCooValve_4, emissionControl.yCoo);
connect(dataBus.yHeaValve_4, emissionControl.yHea);
end EmissionControlEmissioncontrol_4;
        model OccupancyOccupancy_4
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_5, occSch2.occupied);
 end OccupancyOccupancy_4;
 
        model EmissionControlEmissioncontrol_3
  parameter Real schedule[:]=3600*{7,19};

  parameter   Modelica.Units.SI.Temperature THeaSet= 273.15+24;
    parameter   Modelica.Units.SI.Temperature THeaSetBack= 273.15+16;
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
connect(dataBus.TCooSetSpace_4, emissionControl.TCooSet);
connect(dataBus.TZonSpace_4, emissionControl.TZon);
connect(dataBus.yCooValve_3, emissionControl.yCoo);
connect(dataBus.yHeaValve_3, emissionControl.yHea);
end EmissionControlEmissioncontrol_3;
        model OccupancyOccupancy_3
extends house_model.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_4, occSch2.occupied);
 end OccupancyOccupancy_3;
 
        model CollectorControlCollectorcontrol_0
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
mulMax(nin=8)
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
coordinateSystem(preserveAspectRatio=false)));connect(dataBus.y_gainPump_0, conPum.u_m);
connect(dataBus.yPump_0, conPum.y);
connect(dataBus.yBoiConPump_0, mulMax.y);
connect(dataBus.yPumBoiPump_0, mulMax.y);
connect(dataBus.yHeaValve_2, mulMax.u[1]);
connect(dataBus.yHeaValve_1, mulMax.u[2]);
connect(dataBus.yHeaValve_0, mulMax.u[3]);
connect(dataBus.yHeaValve_5, mulMax.u[4]);
connect(dataBus.yHeaValve_6, mulMax.u[5]);
connect(dataBus.yHeaValve_7, mulMax.u[6]);
connect(dataBus.yHeaValve_4, mulMax.u[7]);
connect(dataBus.yHeaValve_3, mulMax.u[8]);
end CollectorControlCollectorcontrol_0;
        model ThreeWayValveControlThreewayvalvecontrol_0
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
connect(dataBus.TColSetThreewayvalvecontrol_0, conVal.u_s);
connect(dataBus.triggerThreewayvalvecontrol_0, conVal.trigger);
  connect(conVal.y, y)
annotation (Line(points={{10,0},{110,0}}, color={0,0,127}));  connect(u, conVal.u_m) annotation (Line(points={{-118,0},{-22,0},{-22,-20},{0,
          -20},{0,-16},{-2,-16},{-2,-12}}, color={0,0,127}));annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));end ThreeWayValveControlThreewayvalvecontrol_0;
        model CollectorControlCollectorcontrol_1
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
mulMax(nin=8)
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
coordinateSystem(preserveAspectRatio=false)));connect(dataBus.y_gainBoiler_0, conPum.u_m);
connect(dataBus.yBoiler_0, conPum.y);
connect(dataBus.yBoiConBoiler_0, mulMax.y);
connect(dataBus.yPumBoiBoiler_0, mulMax.y);
connect(dataBus.yHeaValve_2, mulMax.u[1]);
connect(dataBus.yHeaValve_1, mulMax.u[2]);
connect(dataBus.yHeaValve_0, mulMax.u[3]);
connect(dataBus.yHeaValve_5, mulMax.u[4]);
connect(dataBus.yHeaValve_6, mulMax.u[5]);
connect(dataBus.yHeaValve_7, mulMax.u[6]);
connect(dataBus.yHeaValve_4, mulMax.u[7]);
connect(dataBus.yHeaValve_3, mulMax.u[8]);
end CollectorControlCollectorcontrol_1;
        model DataServer
replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[11]
TRoo annotation (
Placement(transformation(origin={-544,-226},
extent = {{480, 216}, {500, 236}})));Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[11]
port annotation (
Placement(transformation(extent={{-112,-10},{-92,10}}),
iconTransformation(extent = {{-110, -10}, {-90, 10}})));Buildings.Fluid.Sensors.PPM[11] TRoo1(redeclare
package Medium = Medium)annotation (
Placement(transformation(origin={-542,-268},
extent = {{480, 216}, {500, 236}})));Modelica.Fluid.Interfaces.FluidPort_a[11]
port_a(redeclare package Medium
= Medium)annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));  Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_6
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_2
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_3
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            y_gainCollectorcontrol_1
            (y=0.0);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_0
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TColSetThreewayvalvecontrol_0
            (y=363.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_4
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_1
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_7
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_5
            (y=298.15);
Modelica.Blocks.Sources.BooleanExpression
            triggerThreewayvalvecontrol_0
            (y=true);

Buildings.Electrical.AC.OnePhase.Interfaces.Terminal_p term_p annotation (
Placement(transformation(extent={{74,-20},{110,20}}), iconTransformation(
    extent={{74,-20},{110,20}})));
Buildings.Electrical.AC.OnePhase.Loads.Resistive loa(linearized=true, mode=
  Buildings.Electrical.Types.Load.VariableZ_P_input)
annotation (Placement(transformation(extent={{-32,-72},{16,-26}})));
Modelica.Blocks.Math.Sum sum1(nin= 2)

annotation (Placement(transformation(extent={{-72,-36},{-52,-16}})));
  Modelica.Blocks.Sources.Constant const(k=0)
annotation (Placement(transformation(extent={{-84,32},{-64,52}})));
equation
connect(port[1],TRoo[1]. port);
connect(port_a[1], TRoo1[1].port);
connect(port[2],TRoo[2]. port);
connect(port_a[2], TRoo1[2].port);
connect(port[3],TRoo[3]. port);
connect(port_a[3], TRoo1[3].port);
connect(port[4],TRoo[4]. port);
connect(port_a[4], TRoo1[4].port);
connect(port[5],TRoo[5]. port);
connect(port_a[5], TRoo1[5].port);
connect(port[6],TRoo[6]. port);
connect(port_a[6], TRoo1[6].port);
connect(port[7],TRoo[7]. port);
connect(port_a[7], TRoo1[7].port);
connect(port[8],TRoo[8]. port);
connect(port_a[8], TRoo1[8].port);
connect(port[9],TRoo[9]. port);
connect(port_a[9], TRoo1[9].port);
connect(port[10],TRoo[10]. port);
connect(port_a[10], TRoo1[10].port);
connect(port[11],TRoo[11]. port);
connect(port_a[11], TRoo1[11].port);
connect(dataBus.TZonSpace_1, TRoo[1].T);
connect(dataBus.TZonSpace_10, TRoo[2].T);
connect(dataBus.TZonSpace_11, TRoo[3].T);
connect(dataBus.TZonSpace_2, TRoo[4].T);
connect(dataBus.TZonSpace_3, TRoo[5].T);
connect(dataBus.TZonSpace_4, TRoo[6].T);
connect(dataBus.TZonSpace_5, TRoo[7].T);
connect(dataBus.TZonSpace_6, TRoo[8].T);
connect(dataBus.TZonSpace_7, TRoo[9].T);
connect(dataBus.TZonSpace_8, TRoo[10].T);
connect(dataBus.TZonSpace_9, TRoo[11].T);
connect(dataBus.ppmCO2Space_1, TRoo1[1].ppm);
connect(dataBus.ppmCO2Space_10, TRoo1[2].ppm);
connect(dataBus.ppmCO2Space_11, TRoo1[3].ppm);
connect(dataBus.ppmCO2Space_2, TRoo1[4].ppm);
connect(dataBus.ppmCO2Space_3, TRoo1[5].ppm);
connect(dataBus.ppmCO2Space_4, TRoo1[6].ppm);
connect(dataBus.ppmCO2Space_5, TRoo1[7].ppm);
connect(dataBus.ppmCO2Space_6, TRoo1[8].ppm);
connect(dataBus.ppmCO2Space_7, TRoo1[9].ppm);
connect(dataBus.ppmCO2Space_8, TRoo1[10].ppm);
connect(dataBus.ppmCO2Space_9, TRoo1[11].ppm);
connect(dataBus.TCooSetSpace_9,
TCooSetEmissioncontrol_6.y);
connect(dataBus.TCooSetSpace_3,
TCooSetEmissioncontrol_2.y);
connect(dataBus.TCooSetSpace_4,
TCooSetEmissioncontrol_3.y);
connect(dataBus.y_gainBoiler_0,
y_gainCollectorcontrol_1.y);
connect(dataBus.TCooSetSpace_1,
TCooSetEmissioncontrol_0.y);
connect(dataBus.TColSetThreewayvalvecontrol_0,
TColSetThreewayvalvecontrol_0.y);
connect(dataBus.TCooSetSpace_5,
TCooSetEmissioncontrol_4.y);
connect(dataBus.TCooSetSpace_2,
TCooSetEmissioncontrol_1.y);
connect(dataBus.TCooSetSpace_10,
TCooSetEmissioncontrol_7.y);
connect(dataBus.TCooSetSpace_6,
TCooSetEmissioncontrol_5.y);
connect(dataBus.triggerThreewayvalvecontrol_0,
triggerThreewayvalvecontrol_0.y);


connect(term_p, loa.terminal) annotation (Line(points={{92,0},{-32,0},{-32,-51},
{ -28,-51 } }, color={0,120,120}));
connect(dataBus.electricityPump_0, sum1.u[2]) 


annotation (Line(
points={{-100,2},{-48,2},{-48,-40},{-84,-40},{-84,-26},{-74,-26}},
color={255,204,51},
thickness=0.5), Text(
string="%first",
index=-1,
extent={{-6,3},{-6,3}},
horizontalAlignment=TextAlignment.Right));

        connect(const.y, sum1.u[1]) annotation (Line(points={{-63,42},{-58,42},{-58,14},
    {-76,14},{-76,-20},{-74,-20},{-74,-26}}, color={0,0,127}));
connect(sum1.y, loa.Pow) annotation (Line(points={{-51,-26},{-38,-26},{-38,-76},
    {24,-76},{24,-49},{16,-49}}, color={0,0,127}));
end DataServer;
            model BoilerWithStorageBoiler_0
    extends house_model.Trano.Fluid.Boilers.PartialBoilerWithStorage;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.yBoiConBoiler_0, Boiy.u);
connect(dataBus.yPumBoiBoiler_0, pumBoi.y);
connect(dataBus.TStoTopBoiler_0, tanTemTop.T);
connect(dataBus.TStoBotBoiler_0, tanTemBot.T);
     end BoilerWithStorageBoiler_0;
     
        model PumpPump_0
extends house_model.Trano.Fluid.Ventilation.PartialPump;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.yPump_0, pumRad.y);
connect(dataBus.y_gainPump_0, gain.y);
connect(dataBus.electricityPump_0, pumRad.P);
connect(dataBus.TCollectorcontrol_0, temSup.T);
 end PumpPump_0;
 
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
end house_model;