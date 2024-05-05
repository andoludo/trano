package ideas_free_float_three_zones

package Common
  package Occupancy

    model SimpleOccupancy


      Buildings.Controls.SetPoints.OccupancySchedule
                                           occSch2(firstEntryOccupied=false,
          occupancy=3600*{7,10,12,22})
        "Occupancy schedule"
        annotation (Placement(transformation(extent={{-66,-22},{-46,-2}})));
      Buildings.Controls.OBC.CDL.Reals.Switch switch2
        annotation (Placement(transformation(extent={{-20,-12},{0,8}})));
      Modelica.Blocks.Math.MatrixGain gai2(K=[35; 70; 30])
        "Gain to convert from occupancy (per person) to radiant, convective and latent heat in [W/m2] "
        annotation (Placement(transformation(extent={{18,-12},{38,8}})));
       extends Modelica.Blocks.Interfaces.MO(final nout=3);
      Buildings.Controls.OBC.CDL.Reals.Sources.Constant occ2(k=1/6/4)
        "Heat gain if occupied in room 2"
        annotation (Placement(transformation(extent={{-66,28},{-46,48}})));
      Buildings.Controls.OBC.CDL.Reals.Sources.Constant zero(k=0)
        "Heat gain if occupied in room 2"
        annotation (Placement(transformation(extent={{-62,-68},{-42,-48}})));
    equation
      connect(occSch2.occupied,switch2. u2) annotation (Line(
          points={{-45,-18},{-28,-18},{-28,-2},{-22,-2}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(switch2.y,gai2. u[1]) annotation (Line(
          points={{2,-2},{16,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(occ2.y,switch2. u1) annotation (Line(points={{-44,38},{-40,38},{-40,6},
              {-22,6}},        color={0,0,127}));
      connect(zero.y, switch2.u3)
        annotation (Line(points={{-40,-58},{-22,-58},{-22,-10}}, color={0,0,127}));
      connect(gai2.y, y) annotation (Line(points={{39,-2},{96,-2},{96,0},{110,0}},
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
  end Occupancy;

  package Controls
  package BaseClasses
  annotation (
        Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
                fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Ellipse(lineColor = {128, 128, 128}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-30, -30}, {30, 30}})}));
  end BaseClasses;

    package Interfaces
      partial model BaseSpaceControl
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port annotation (
                Placement(transformation(extent = {{-110, -10}, {-90, 10}}), iconTransformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (
                Placement(transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}})));
      equation

            annotation (
                Icon,
        Diagram);
      end BaseSpaceControl;

      partial model BaseSubstanceSpaceControl
        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);
        Modelica.Fluid.Interfaces.FluidPort_a port_a(
          redeclare final package Medium = Medium)
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{-116,-16},{-82,16}}),
              iconTransformation(extent={{-110,-9},{-90,9}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation (
                Placement(transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}})));
      equation

            annotation (
                Icon,
        Diagram);
      end BaseSubstanceSpaceControl;
      annotation (
        Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
                fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Polygon(origin = {20, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-10, 70}, {10, 70}, {40, 20}, {80, 20}, {80, -20}, {40, -20}, {10, -70}, {-10, -70}, {-10, 70}}), Polygon(fillColor = {102, 102, 102}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-100, 20}, {-60, 20}, {-30, 70}, {-10, 70}, {-10, -70}, {-30, -70}, {-60, -20}, {-100, -20}, {-100, 20}})}));
    end Interfaces;

    package SpaceControls
      model PID
              extends Common.Controls.Interfaces.BaseSpaceControl;

                parameter .Modelica.Blocks.Types.SimpleController controllerType=
                 .Modelica.Blocks.Types.SimpleController.PID "Type of controller";
          parameter Real k(min=0, unit="1") = 1 "Gain of controller";
          parameter Modelica.Units.SI.Time Ti(min=Modelica.Constants.small)=0.5
            "Time constant of Integrator block" annotation (Dialog(enable=
                  controllerType == .Modelica.Blocks.Types.SimpleController.PI or
                  controllerType == .Modelica.Blocks.Types.SimpleController.PID));
          parameter Modelica.Units.SI.Time Td(min=0)=0.1
            "Time constant of Derivative block" annotation (Dialog(enable=
                  controllerType == .Modelica.Blocks.Types.SimpleController.PD or
                  controllerType == .Modelica.Blocks.Types.SimpleController.PID));
          parameter Real yMax(start=1) "Upper limit of output";
          parameter Real yMin=-yMax "Lower limit of output";
          parameter Modelica.Units.SI.Temperature setPoint;
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TRoo annotation (
                Placement(transformation(origin = {-542, -226}, extent = {{480, 216}, {500, 236}})));
        Modelica.Blocks.Continuous.LimPID conRoo(yMax = yMax, yMin = yMin, controllerType = controllerType, k = k, Ti = Ti, Td = Td)  annotation (
                Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=setPoint)
          annotation (Placement(transformation(extent={{-42,36},{-22,56}})));
      equation
              connect(port, TRoo.port) annotation (
                Line(points = {{-100, 0}, {-62, 0}}, color = {191, 0, 0}));
        connect(conRoo.y, y) annotation (
                Line(points={{61,0},{106,0}},      color = {0, 0, 127}));
        connect(TRoo.T, conRoo.u_m) annotation (
                Line(points={{-41,0},{4,0},{4,-36},{50,-36},{50,-12}},            color = {0, 0, 127}));
        connect(realExpression.y, conRoo.u_s) annotation (Line(points={{-21,46},{32,46},
                {32,0},{38,0}}, color={0,0,127}));
            annotation (
                Icon(graphics={  Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Line(points = {{-80, 78}, {-80, -90}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-80, 90}, {-88, 68}, {-72, 68}, {-80, 90}}), Line(points = {{-90, -80}, {82, -80}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{90, -80}, {68, -72}, {68, -88}, {90, -80}}), Line(points = {{-80, -80}, {-80, -20}, {60, 80}}, color = {0, 0, 127}), Text(textColor = {192, 192, 192}, extent = {{-20, -60}, {80, -20}}, textString = "PID")}));
      end PID;

      model PIDSubstance

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);
              extends Common.Controls.Interfaces.BaseSubstanceSpaceControl;

                parameter .Modelica.Blocks.Types.SimpleController controllerType=
                 .Modelica.Blocks.Types.SimpleController.PID "Type of controller";
          parameter Real k(min=0, unit="1") = 1 "Gain of controller";
          parameter Modelica.Units.SI.Time Ti(min=Modelica.Constants.small)=0.5
            "Time constant of Integrator block" annotation (Dialog(enable=
                  controllerType == .Modelica.Blocks.Types.SimpleController.PI or
                  controllerType == .Modelica.Blocks.Types.SimpleController.PID));
          parameter Modelica.Units.SI.Time Td(min=0)=0.1
            "Time constant of Derivative block" annotation (Dialog(enable=
                  controllerType == .Modelica.Blocks.Types.SimpleController.PD or
                  controllerType == .Modelica.Blocks.Types.SimpleController.PID));
          parameter Real yMax(start=1) "Upper limit of output";
          parameter Real yMin=-yMax "Lower limit of output";
          parameter Modelica.Units.SI.Temperature setPoint;
        Buildings.Fluid.Sensors.PPM                             TRoo( redeclare
            package                                                                     Medium = Medium) annotation (
                Placement(transformation(origin = {-542, -226}, extent = {{480, 216}, {500, 236}})));
        Modelica.Blocks.Continuous.LimPID conRoo(yMax = yMax, yMin = yMin, controllerType = controllerType, k = k, Ti = Ti, Td = Td)  annotation (
                Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y=setPoint)
          annotation (Placement(transformation(extent={{-42,38},{-22,58}})));
      equation
        connect(conRoo.y, y) annotation (
                Line(points={{61,0},{106,0}},      color = {0, 0, 127}));
        connect(realExpression.y, conRoo.u_s) annotation (Line(points={{-21,48},
                {32,48},{32,0},{38,0}},
                                color={0,0,127}));
        connect(TRoo.ppm, conRoo.u_m) annotation (Line(points={{-41,0},{30,0},{30,-18},
                {50,-18},{50,-12}}, color={0,0,127}));
        connect(port_a, TRoo.port) annotation (Line(points={{-99,0},{-68,0},{
                -68,-14},{-52,-14},{-52,-10}}, color={0,127,255}));
            annotation (
                Icon(graphics={  Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255},
                  fillPattern =                                                                                FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Line(points = {{-80, 78}, {-80, -90}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-80, 90}, {-88, 68}, {-72, 68}, {-80, 90}}), Line(points = {{-90, -80}, {82, -80}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{90, -80}, {68, -72}, {68, -88}, {90, -80}}), Line(points = {{-80, -80}, {-80, -20}, {60, 80}}, color = {0, 0, 127}), Text(textColor = {192, 192, 192}, extent = {{-20, -60}, {80, -20}}, textString = "PID")}));
      end PIDSubstance;
      annotation (
        Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
                fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
    end SpaceControls;
  annotation (
      Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(origin = {0, 35.1488}, fillColor = {255, 255, 255}, extent = {{-30, -20.1488}, {30, 20.1488}}), Rectangle(origin = {0, -34.8512}, fillColor = {255, 255, 255}, extent = {{-30, -20.1488}, {30, 20.1488}}), Line(origin = {-51.25, 0}, points = {{21.25, -35}, {-13.75, -35}, {-13.75, 35}, {6.25, 35}}), Polygon(origin = {-40, 35}, pattern = LinePattern.None,
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{10, 0}, {-5, 5}, {-5, -5}, {10, 0}}), Line(origin = {51.25, 0}, points = {{-21.25, 35}, {13.75, 35}, {13.75, -35}, {-6.25, -35}}), Polygon(origin = {40, -35}, pattern = LinePattern.None,
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-10, 0}, {5, 5}, {5, -5}, {-10, 0}})}));
  end Controls;

  package Fluid
  package Boilers
  model Simple

  extends Buildings.Fluid.Interfaces.PartialTwoPort;
  Buildings.Fluid.Sources.Boundary_pT bou(use_T_in = true, nPorts = 2, redeclare
            final package                                                                      Medium = Medium)  annotation (
          Placement(transformation(origin = {90, 188}, extent = {{-82, -180}, {-62, -160}})));
  Modelica.Blocks.Sources.Constant constant1(k = 273 + 70)  annotation (
          Placement(transformation(origin = {-32, 20}, extent = {{-10, -10}, {10, 10}})));
  equation
  connect(constant1.y, bou.T_in) annotation (
          Line(points = {{-20, 20}, {6, 20}, {6, 22}}, color = {0, 0, 127}));
  connect(bou.ports[1], port_b) annotation (
          Line(points = {{28, 18}, {100, 18}, {100, 0}}, color = {0, 127, 255}));
  connect(bou.ports[2], port_a) annotation (
          Line(points = {{28, 18}, {-100, 18}, {-100, 0}}, color = {0, 127, 255}));
      annotation (
          Icon(graphics={  Rectangle(fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Rectangle(fillColor = {255, 255, 255},
                  fillPattern =                                                                                                                              FillPattern.Solid, extent = {{-68, 70}, {70, -70}}), Polygon(lineColor = {0, 0, 255}, fillColor = {0, 0, 255},
                  fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-68, 18}, {-68, 18}, {-54, 32}, {-28, 16}, {0, 30}, {26, 16}, {46, 32}, {70, 18}, {70, 18}, {70, -70}, {70, -70}, {-68, -70}, {-68, -70}, {-68, 18}}, smooth = Smooth.Bezier)}));
  end Simple;
  end Boilers;

    package Ventilation
      model SimpleHVAC

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);
        IDEAS.Fluid.Movers.FlowControlled_dp
                                       fanSup(
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          use_inputFilter=false,
          inputType=IDEAS.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=1 + 1) "Supply fan"
          annotation (Placement(transformation(extent={{4,6},{24,26}})));
        IDEAS.Fluid.Movers.FlowControlled_dp
                                       fanRet(
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          use_inputFilter=false,
          inputType=IDEAS.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=1 + 1) "Return fan"
          annotation (Placement(transformation(extent={{24,-34},{4,-14}})));
        IDEAS.Fluid.HeatExchangers.ConstantEffectiveness
                                                   hex(
          redeclare package Medium1 = Medium,
          redeclare package Medium2 = Medium,
          m1_flow_nominal=1,
          m2_flow_nominal=1,
          dp1_nominal=100,
          dp2_nominal=100) "Heat exchanger with constant heat recovery effectivity"
          annotation (Placement(transformation(extent={{-26,-14},{-6,6}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(
          redeclare final package Medium = Medium)
          "Fluid connector b (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{118,1},{86,31}}),
              iconTransformation(extent={{110,31},{90,49}})));

        Modelica.Fluid.Interfaces.FluidPort_a port_a(
          redeclare final package Medium = Medium)
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{84,-40},{118,-8}}),
              iconTransformation(extent={{90,-49},{110,-31}})));
        IDEAS.Fluid.Sources.OutsideAir outsideAir(
          azi=0,                                  nPorts=2, redeclare package
            Medium = Medium) annotation (
          Placement(transformation(origin = {-64, 2}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(hex.port_b1, fanSup.port_a) annotation (
          Line(points = {{-6, 2}, {-6, 16}, {4, 16}}, color = {0, 127, 255}));
        connect(hex.port_a2, fanRet.port_b) annotation (
          Line(points = {{-6, -10}, {-6, -24}, {4, -24}}, color = {0, 127, 255}));
        connect(fanSup.port_b, port_b) annotation (
          Line(points = {{24, 16}, {102, 16}}, color = {0, 127, 255}));
        connect(fanRet.port_a, port_a) annotation (
          Line(points = {{24, -24}, {101, -24}}, color = {0, 127, 255}));
        connect(
          outsideAir.ports[1], hex.port_a1) annotation (
          Line(points={{-54,4},{-40,4},{-40,2},{-26,2}},
                                              color = {0, 127, 255}));
        connect(
          outsideAir.ports[2], hex.port_b2) annotation (
          Line(points={{-54,0},{-26,0},{-26,-10}},        color = {0, 127, 255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},
                  {100,60}}), graphics={Rectangle(
                extent={{-100,60},{100,-60}},
                lineColor={255,128,0},
                fillColor={255,128,0},
                fillPattern=FillPattern.Forward)}), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-60},{100,60}})));
      end SimpleHVAC;

      model SimpleVAV
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end SimpleVAV;

      model SimpleHVACBuildings

        replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
          "Medium model" annotation (choicesAllMatching=true);
        Buildings.Fluid.Movers.FlowControlled_dp
                                       fanSup(
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          use_inputFilter=false,
          inputType=Buildings.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=1 + 1) "Supply fan"
          annotation (Placement(transformation(extent={{4,6},{24,26}})));
        Buildings.Fluid.Movers.FlowControlled_dp
                                       fanRet(
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          use_inputFilter=false,
          inputType=Buildings.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=1 + 1) "Return fan"
          annotation (Placement(transformation(extent={{24,-34},{4,-14}})));
        Buildings.Fluid.HeatExchangers.ConstantEffectiveness
                                                   hex(
          redeclare package Medium1 = Medium,
          redeclare package Medium2 = Medium,
          m1_flow_nominal=1,
          m2_flow_nominal=1,
          dp1_nominal=100,
          dp2_nominal=100) "Heat exchanger with constant heat recovery effectivity"
          annotation (Placement(transformation(extent={{-26,-14},{-6,6}})));
        Modelica.Fluid.Interfaces.FluidPort_b port_b(
          redeclare final package Medium = Medium)
          "Fluid connector b (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{118,1},{86,31}}),
              iconTransformation(extent={{110,31},{90,49}})));

        Modelica.Fluid.Interfaces.FluidPort_a port_a(
          redeclare final package Medium = Medium)
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{84,-40},{118,-8}}),
              iconTransformation(extent={{90,-49},{110,-31}})));
        Buildings.Fluid.Sources.Boundary_pT bou(T=295.15, nPorts=2,    redeclare
            package                                                                      Medium = Medium)
          annotation (Placement(transformation(extent={{-78,-14},{-58,6}})));
      equation
        connect(hex.port_b1, fanSup.port_a) annotation (
          Line(points = {{-6, 2}, {-6, 16}, {4, 16}}, color = {0, 127, 255}));
        connect(hex.port_a2, fanRet.port_b) annotation (
          Line(points = {{-6, -10}, {-6, -24}, {4, -24}}, color = {0, 127, 255}));
        connect(fanSup.port_b, port_b) annotation (
          Line(points = {{24, 16}, {102, 16}}, color = {0, 127, 255}));
        connect(fanRet.port_a, port_a) annotation (
          Line(points = {{24, -24}, {101, -24}}, color = {0, 127, 255}));
        connect(bou.ports[1], hex.port_b2) annotation (Line(points={{-58,-2},{-32,-2},
                {-32,-10},{-26,-10}}, color={0,127,255}));
        connect(bou.ports[2], hex.port_a1) annotation (Line(points={{-58,-6},{-32,-6},
                {-32,2},{-26,2}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},
                  {100,60}}), graphics={Rectangle(
                extent={{-100,60},{100,-60}},
                lineColor={255,128,0},
                fillColor={255,128,0},
                fillPattern=FillPattern.Forward)}), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-60},{100,60}})));
      end SimpleHVACBuildings;
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
  annotation (uses(Buildings(version = "11.0.0"), Modelica(version = "4.0.0"),
      IDEAS(version="3.0.0")),
  Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
            fillPattern =                                                                            FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
end Common;

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
    record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_free_float_three_zones.Data.Materials.id_100
        (d=0.003),ideas_free_float_three_zones.Data.Materials.Air
        (d=0.0127),ideas_free_float_three_zones.Data.Materials.id_100
        (d=0.003)    },
    final SwTrans=[0, 0.721;
                    10, 0.720;
                    20, 0.718;
                    30, 0.711;
                    40, 0.697;
                    50, 0.665;
                    60, 0.596;
                    70, 0.454;
                    80, 0.218;
                    90, 0.000],
      final SwAbs=[0, 0.082, 0, 0.062;
                  10, 0.082, 0, 0.062;
                  20, 0.084, 0, 0.063;
                  30, 0.086, 0, 0.065;
                  40, 0.090, 0, 0.067;
                  50, 0.094, 0, 0.068;
                  60, 0.101, 0, 0.067;
                  70, 0.108, 0, 0.061;
                  80, 0.112, 0, 0.045;
                  90, 0.000, 0, 0.000],
      final SwTransDif=0.619,
      final SwAbsDif={0.093, 0,  0.063},
      final U_value=2.9,
      final g_value=0.78

    ) "ideas_free_float_three_zones";
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
      epsLw=0.88,
      epsSw=0.55);    record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);    record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.0,
      c=840.0,
      rho=2500.0,
      epsLw=0.88,
      epsSw=0.55);    record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);    record brick = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=790.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);    record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record internal_wall
    "internal_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_three_zones.Data.Materials.brick
        (d=0.2)    });
    end internal_wall;      record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_three_zones.Data.Materials.concrete
        (d=0.2),ideas_free_float_three_zones.Data.Materials.insulation_board
        (d=0.02),ideas_free_float_three_zones.Data.Materials.plywood
        (d=0.1)    });
    end external_wall;
end Constructions;
end Data;
model building



replaceable package Medium = IDEAS.Media.Air
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);  inner IDEAS.BoundaryConditions.SimInfoManager
sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));

    IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,    V=10,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=9,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 3200, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_1_w2_1_w3_1_w4_1(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -147.1143502881811, 147.23280894204024 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[2]
    merged_win1_1_win2_1(
    redeclare parameter ideas_free_float_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 10, 10 },
    final azi={ 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 192.9066426622929, 17.83922651128474 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_1(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -11.659499571635111, 195.3249109853301 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_2(
    mSenFac=0.822,    V=10,
    n50=0.822*0.5*space_2.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 3400, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[3]
    merged_w1_2_w2_2_w3_2(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10 },
    final azi={ 135, 0, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 189.72685833773542, -62.15170081921953 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[2]
    merged_win1_2_win2_2(
    redeclare parameter ideas_free_float_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 10, 10 },
    final azi={ 0, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -63.173051873613005, -169.0630740214997 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -195.7540968910603, -13.199617833003577 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_3(
    mSenFac=0.822,    V=10,
    n50=0.822*0.5*space_3.n50toAch,
    redeclare package Medium = Medium,
    nSurf=7,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 3600, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[3]
    merged_w1_3_w2_3_w3_3(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10 },
    final azi={ 135, 0, 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -134.2927238636985, -157.98642662410435 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_w4_3(
    redeclare parameter ideas_free_float_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 10 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 11.827440227192598, -188.24006455496456 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_3(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { 90.97672885776035, -193.011608957264 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_1_space_2
    (redeclare parameter ideas_free_float_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 3300.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_1_space_3
    (redeclare parameter ideas_free_float_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 3400.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_2_space_3
    (redeclare parameter ideas_free_float_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 3500.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));



equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 3200.0, 50.0 }    ,{ 1526.4428248559095, 50.0 }    ,{ 1526.4428248559093, 147.23280894204024 }    ,{ -147.1143502881811, 147.23280894204024 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5:6],merged_win1_1_win2_1[1:2].propsBus_a)
annotation (Line(
points={{ 3200.0, 50.0 }    ,{ 1696.4533213311465, 50.0 }    ,{ 1696.4533213311463, 17.83922651128474 }    ,{ 192.9066426622929, 17.83922651128474 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[7],floor_1.propsBus_a)
annotation (Line(
points={{ 3200.0, 50.0 }    ,{ 1594.1702502141825, 50.0 }    ,{ 1594.1702502141825, 195.3249109853301 }    ,{ -11.659499571635111, 195.3249109853301 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[8],internal_space_1_space_2.propsBus_a)
annotation (Line(
points={{ 3200.0, 50.0 }    ,{ 3250.0, 50.0 }    ,{ 3250.0, 50.0 }    ,{ 3300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[9],internal_space_1_space_3.propsBus_a)
annotation (Line(
points={{ 3200.0, 50.0 }    ,{ 3300.0, 50.0 }    ,{ 3300.0, 50.0 }    ,{ 3400.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[1:3],merged_w1_2_w2_2_w3_2[1:3].propsBus_a)
annotation (Line(
points={{ 3400.0, 50.0 }    ,{ 1794.8634291688677, 50.0 }    ,{ 1794.8634291688677, -62.15170081921953 }    ,{ 189.72685833773542, -62.15170081921953 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[4:5],merged_win1_2_win2_2[1:2].propsBus_a)
annotation (Line(
points={{ 3400.0, 50.0 }    ,{ 1668.4134740631935, 50.0 }    ,{ 1668.4134740631935, -169.0630740214997 }    ,{ -63.173051873613005, -169.0630740214997 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 3400.0, 50.0 }    ,{ 1602.1229515544699, 50.0 }    ,{ 1602.1229515544699, -13.199617833003577 }    ,{ -195.7540968910603, -13.199617833003577 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[7],internal_space_1_space_2.propsBus_b)
annotation (Line(
points={{ 3400.0, 50.0 }    ,{ 3350.0, 50.0 }    ,{ 3350.0, 50.0 }    ,{ 3300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[8],internal_space_2_space_3.propsBus_a)
annotation (Line(
points={{ 3400.0, 50.0 }    ,{ 3450.0, 50.0 }    ,{ 3450.0, 50.0 }    ,{ 3500.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[1:3],merged_w1_3_w2_3_w3_3[1:3].propsBus_a)
annotation (Line(
points={{ 3600.0, 50.0 }    ,{ 1732.8536380681508, 50.0 }    ,{ 1732.8536380681508, -157.98642662410435 }    ,{ -134.2927238636985, -157.98642662410435 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[4],merged_w4_3[1].propsBus_a)
annotation (Line(
points={{ 3600.0, 50.0 }    ,{ 1805.9137201135964, 50.0 }    ,{ 1805.9137201135964, -188.24006455496456 }    ,{ 11.827440227192598, -188.24006455496456 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[5],floor_3.propsBus_a)
annotation (Line(
points={{ 3600.0, 50.0 }    ,{ 1845.4883644288802, 50.0 }    ,{ 1845.4883644288802, -193.011608957264 }    ,{ 90.97672885776035, -193.011608957264 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[6],internal_space_1_space_3.propsBus_b)
annotation (Line(
points={{ 3600.0, 50.0 }    ,{ 3500.0, 50.0 }    ,{ 3500.0, 50.0 }    ,{ 3400.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[7],internal_space_2_space_3.propsBus_b)
annotation (Line(
points={{ 3600.0, 50.0 }    ,{ 3550.0, 50.0 }    ,{ 3550.0, 50.0 }    ,{ 3500.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));end building;


end ideas_free_float_three_zones;
