package ideas_simple_hydronic_three_zones

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
      model DataServer
      replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
    "Medium model" annotation (choicesAllMatching=true);
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[3] TRoo annotation (
                Placement(transformation(origin={-544,-226},    extent = {{480, 216}, {500, 236}})));        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] port annotation (
                Placement(transformation(extent={{-112,-10},{-92,10}}),      iconTransformation(extent = {{-110, -10}, {-90, 10}})));
        BaseClasses.DataBus controlBus annotation (Placement(transformation(
                extent={{80,-20},{120,20}}), iconTransformation(extent={{90,-10},
                  {110,10}})));      Buildings.Fluid.Sensors.PPM[3]                             TRoo1(redeclare
      package Medium = Medium)                                                                     annotation (
          Placement(transformation(origin={-542,-268},    extent = {{480, 216}, {500, 236}})));  Modelica.Fluid.Interfaces.FluidPort_a[3] port_a(redeclare package Medium
      = Medium)
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
  iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));        equation
              connect(port[1],TRoo[1]. port)
annotation (
                Line(points={{-102,0},{-64,0}},      color = {191, 0, 0}));
        connect(TRoo[1].T, controlBus.temperature_space_1)

            annotation (Line(points={{-43,0},{100,0}}, color={0,0,127}), Text(
            string="%second",
            index=1,
            extent={{6,3},{6,3}},
            horizontalAlignment=TextAlignment.Left));
          connect(port_a[1], TRoo1[1].port)
annotation (Line(points={{-99,-42},{-68,-42},{-68,
          -56},{-52,-56},{-52,-52}}, color={0,127,255}));  connect(TRoo1[1].ppm, controlBus.ppm_space_1)
annotation (Line(points={{-41,-42},{74,
          -42},{74,0},{100,0}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));              connect(port[2],TRoo[2]. port)
annotation (
                Line(points={{-102,0},{-64,0}},      color = {191, 0, 0}));
        connect(TRoo[2].T, controlBus.temperature_space_2)

            annotation (Line(points={{-43,0},{100,0}}, color={0,0,127}), Text(
            string="%second",
            index=1,
            extent={{6,3},{6,3}},
            horizontalAlignment=TextAlignment.Left));
          connect(port_a[2], TRoo1[2].port)
annotation (Line(points={{-99,-42},{-68,-42},{-68,
          -56},{-52,-56},{-52,-52}}, color={0,127,255}));  connect(TRoo1[2].ppm, controlBus.ppm_space_2)
annotation (Line(points={{-41,-42},{74,
          -42},{74,0},{100,0}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));              connect(port[3],TRoo[3]. port)
annotation (
                Line(points={{-102,0},{-64,0}},      color = {191, 0, 0}));
        connect(TRoo[3].T, controlBus.temperature_space_3)

            annotation (Line(points={{-43,0},{100,0}}, color={0,0,127}), Text(
            string="%second",
            index=1,
            extent={{6,3},{6,3}},
            horizontalAlignment=TextAlignment.Left));
          connect(port_a[3], TRoo1[3].port)
annotation (Line(points={{-99,-42},{-68,-42},{-68,
          -56},{-52,-56},{-52,-52}}, color={0,127,255}));  connect(TRoo1[3].ppm, controlBus.ppm_space_3)
annotation (Line(points={{-41,-42},{74,
          -42},{74,0},{100,0}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
                extent={{-102,100},{100,-100}},
                lineColor={28,108,200},
                fillColor={255,255,85},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                preserveAspectRatio=false)));
      end DataServer;

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
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          inputType=IDEAS.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=2*100*1.2/3600) "Supply fan"
          annotation (Placement(transformation(extent={{4,6},{24,26}})));
        IDEAS.Fluid.Movers.FlowControlled_dp
                                       fanRet(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          inputType=IDEAS.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=2*100*1.2/3600) "Return fan"
          annotation (Placement(transformation(extent={{24,-34},{4,-14}})));
        IDEAS.Fluid.HeatExchangers.ConstantEffectiveness
                                                   hex(
          redeclare package Medium1 = Medium,
          redeclare package Medium2 = Medium,
          m1_flow_nominal=2*100*1.2/3600,
          m2_flow_nominal=2*100*1.2/3600,
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
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          use_inputFilter=false,
          inputType=Buildings.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=2*100*1.2/3600) "Supply fan"
          annotation (Placement(transformation(extent={{4,6},{24,26}})));
        Buildings.Fluid.Movers.FlowControlled_dp
                                       fanRet(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          use_inputFilter=false,
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
          mats={ideas_simple_hydronic_three_zones.Data.Materials.id_100
        (d=0.003),ideas_simple_hydronic_three_zones.Data.Materials.Air
        (d=0.0127),ideas_simple_hydronic_three_zones.Data.Materials.id_100
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

    ) "ideas_simple_hydronic_three_zones";
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);    record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.0,
      c=840.0,
      rho=2500.0,
      epsLw=0.88,
      epsSw=0.55);    record brick = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=790.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);    record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);    record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
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
      mats={ideas_simple_hydronic_three_zones.Data.Materials.brick
        (d=0.2)    });
    end internal_wall;      record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_simple_hydronic_three_zones.Data.Materials.concrete
        (d=0.2),ideas_simple_hydronic_three_zones.Data.Materials.insulation_board
        (d=0.02),ideas_simple_hydronic_three_zones.Data.Materials.plywood
        (d=0.1)    });
    end external_wall;
end Constructions;
end Data;
model building



replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);
package MediumW = IDEAS.Media.Water "Medium model";
 parameter Integer nRoo = 2 "Number of rooms";
  parameter Modelica.Units.SI.Volume VRoo=4*6*3 "Volume of one room";
  parameter Modelica.Units.SI.Power Q_flow_nominal=2200
    "Nominal power of heating plant";
 // Due to the night setback, in which the radiator do not provide heat input into the room,
 // we scale the design power of the radiator loop
 parameter Real scaFacRad = 1.5
    "Scaling factor to scale the power (and mass flow rate) of the radiator loop";
  parameter Modelica.Units.SI.Temperature TSup_nominal=273.15 + 50 + 5
    "Nominal supply temperature for radiators";
  parameter Modelica.Units.SI.Temperature TRet_nominal=273.15 + 40 + 5
    "Nominal return temperature for radiators";
  parameter Modelica.Units.SI.Temperature dTRad_nominal=TSup_nominal -
      TRet_nominal "Nominal temperature difference for radiator loop";
  parameter Modelica.Units.SI.Temperature dTBoi_nominal=20
    "Nominal temperature difference for boiler loop";
  parameter Modelica.Units.SI.MassFlowRate mRad_flow_nominal=scaFacRad*
      Q_flow_nominal/dTRad_nominal/4200
    "Nominal mass flow rate of radiator loop";
  parameter Modelica.Units.SI.MassFlowRate mBoi_flow_nominal=scaFacRad*
      Q_flow_nominal/dTBoi_nominal/4200 "Nominal mass flow rate of boiler loop";
  parameter Modelica.Units.SI.PressureDifference dpPip_nominal=10000
    "Pressure difference of pipe (without valve)";
  parameter Modelica.Units.SI.PressureDifference dpVal_nominal=6000
    "Pressure difference of valve";
  parameter Modelica.Units.SI.PressureDifference dpRoo_nominal=6000
    "Pressure difference of flow leg that serves a room";
  parameter Modelica.Units.SI.PressureDifference dpThrWayVal_nominal=6000
    "Pressure difference of three-way valve";
  parameter Modelica.Units.SI.PressureDifference dp_nominal=dpPip_nominal +
      dpVal_nominal + dpRoo_nominal + dpThrWayVal_nominal
    "Pressure difference of loop";
  inner IDEAS.BoundaryConditions.SimInfoManager sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));


    IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,nPorts = 1,    V=100,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_1_w2_1_w3_1_w4_1(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -147.48410530257257, 89.88690529484455 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_1(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -78.30930741685818, -172.5806353245702 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -20.538451922117, 194.11319895573044 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission(
    redeclare package Medium = MediumW, allowFlowReversal = false, Q_flow_nominal = 500,
    T_a_nominal = 318.15, T_b_nominal = 308.15, m_flow_nominal = 100*1.2/3600,
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"
    annotation (
    Placement(transformation(origin = { 30, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1, dpValve_nominal = 200,
    allowFlowReversal = false, dpFixed_nominal = 200, linearized = false) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 0, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Controls.SpaceControls.PID
    space_control(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { -50, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_2(
    mSenFac=0.822,nPorts = 1,    V=100,
    n50=0.822*0.5*space_2.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 200, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_2_w2_2_w3_2_w4_2(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -177.45784223189415, 63.12099932454199 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_2(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -140.43460949911088, -150.14134071148172 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_3(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -178.5482625602294, 28.77839187546705 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission_2(
    redeclare package Medium = MediumW, allowFlowReversal = false, Q_flow_nominal = 500,
    T_a_nominal = 318.15, T_b_nominal = 308.15, m_flow_nominal = 100*1.2/3600,
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"
    annotation (
    Placement(transformation(origin = { 230, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve_2(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1, dpValve_nominal = 200,
    allowFlowReversal = false, dpFixed_nominal = 200, linearized = false) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 200, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Controls.SpaceControls.PID
    space_control_2(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { 150, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_3(
    mSenFac=0.822,nPorts = 1,    V=100,
    n50=0.822*0.5*space_3.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 400, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_3_w2_3_w3_3_w4_3(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -186.2703940782089, -41.289979507721405 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_3(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 44.459637436607046, 183.9567311481972 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_4(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { 142.32435086541454, -153.1215886369992 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission_3(
    redeclare package Medium = MediumW, allowFlowReversal = false, Q_flow_nominal = 500,
    T_a_nominal = 318.15, T_b_nominal = 308.15, m_flow_nominal = 100*1.2/3600,
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"
    annotation (
    Placement(transformation(origin = { 430, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve_3(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1, dpValve_nominal = 200,
    allowFlowReversal = false, dpFixed_nominal = 200, linearized = false) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 400, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Controls.SpaceControls.PID
    space_control_3(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { 350, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Occupancy.SimpleOccupancy occupancy_2 annotation (
    Placement(transformation(origin = { 350, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_1_space_2
    (redeclare parameter ideas_simple_hydronic_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 100.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_1_space_3
    (redeclare parameter ideas_simple_hydronic_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 200.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_2_space_3
    (redeclare parameter ideas_simple_hydronic_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 300.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));

        IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1,
    dpValve_nominal = 1000) "Three-way valve"
    annotation (
    Placement(transformation(origin = { -100, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        Modelica.Blocks.Sources.Constant three_way_valve_control(k= 1)
    annotation (
    Placement(transformation(origin = { -150, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve_2(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1,
    dpValve_nominal = 1000) "Three-way valve"
    annotation (
    Placement(transformation(origin = { 300, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        Modelica.Blocks.Sources.Constant three_way_valve_control_2(k= 1)
    annotation (
    Placement(transformation(origin = { 250, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.FixedResistances.Junction split_valve (
    redeclare package Medium = MediumW, m_flow_nominal = {0.1, 0.1, 0.1},
    dp_nominal = {40, 40, 40})
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 130, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.FixedResistances.Junction split_valve_2 (
    redeclare package Medium = MediumW, m_flow_nominal = {0.1, 0.1, 0.1},
    dp_nominal = {40, 40, 40})
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 530, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Fluid.Boilers.Simple boiler(
    redeclare package Medium = MediumW) "Boiler"
    annotation (
    Placement(transformation(origin = { 230, -225 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Movers.FlowControlled_m_flow pump(
    redeclare package Medium = MediumW, m_flow_nominal = 1, dp_nominal = 100)
    annotation (
    Placement(transformation(origin = { -200, -225 },
    extent = {{-10, -10}, {10, 10}}
)));
        Modelica.Blocks.Sources.Constant pump_control(k= 1)
    annotation (
    Placement(transformation(origin = { -250, -225 },
    extent = {{-10, -10}, {10, 10}}
)));
        ideas_simple_hydronic_three_zones.Common.Controls.SpaceControls.DataServer
    data_bus (redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { -107.03667654977146, -154.49940732963856 },
    extent = {{-10, -10}, {10, 10}}
)));


equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -73.74205265128629, 50.0 }    ,{ -73.74205265128629, 89.88690529484455 }    ,{ -147.48410530257257, 89.88690529484455 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -39.15465370842909, 50.0 }    ,{ -39.15465370842909, -172.5806353245702 }    ,{ -78.30930741685818, -172.5806353245702 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -10.2692259610585, 50.0 }    ,{ -10.2692259610585, 194.11319895573044 }    ,{ -20.538451922117, 194.11319895573044 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainCon,emission.heatPortCon)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 15.0, 50.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainRad,emission.heatPortRad)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 15.0, 50.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.yOcc,occupancy_0.y)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -50.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[7],internal_space_1_space_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 100.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[8],internal_space_1_space_3.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainCon,data_bus.port[1])
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -53.51833827488573, 50.0 }    ,{ -53.51833827488573, -154.49940732963856 }    ,{ -107.03667654977146, -154.49940732963856 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[1],data_bus.port_a[1])
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -53.51833827488573, 50.0 }    ,{ -53.51833827488573, -154.49940732963856 }    ,{ -107.03667654977146, -154.49940732963856 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(emission.port_b,split_valve.port_1)
annotation (Line(
points={{ 30.0, -25.0 }    ,{ 80.0, -25.0 }    ,{ 80.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(valve.port_b,emission.port_a)
annotation (Line(
points={{ 0.0, -25.0 }    ,{ 15.0, -25.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.port,space_1.gainCon)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.y,valve.y)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[1:4],merged_w1_2_w2_2_w3_2_w4_2[1:4].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 11.27107888405294, 50.0 }    ,{ 11.271078884052912, 63.12099932454199 }    ,{ -177.45784223189415, 63.12099932454199 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[5],merged_win1_2[1].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 29.78269525044456, 50.0 }    ,{ 29.78269525044456, -150.14134071148172 }    ,{ -140.43460949911088, -150.14134071148172 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[6],floor_3.propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 10.725868719885284, 50.0 }    ,{ 10.725868719885312, 28.77839187546705 }    ,{ -178.5482625602294, 28.77839187546705 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.gainCon,emission_2.heatPortCon)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 215.0, 50.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.gainRad,emission_2.heatPortRad)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 215.0, 50.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.yOcc,occupancy_1.y)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 175.0, 50.0 }    ,{ 175.0, 50.0 }    ,{ 150.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[7],internal_space_1_space_2.propsBus_b)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 150.0, 50.0 }    ,{ 150.0, 50.0 }    ,{ 100.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[8],internal_space_2_space_3.propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 250.0, 50.0 }    ,{ 250.0, 50.0 }    ,{ 300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.gainCon,data_bus.port[2])
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 46.481661725114265, 50.0 }    ,{ 46.48166172511428, -154.49940732963856 }    ,{ -107.03667654977146, -154.49940732963856 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.ports[1],data_bus.port_a[2])
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 46.481661725114265, 50.0 }    ,{ 46.48166172511428, -154.49940732963856 }    ,{ -107.03667654977146, -154.49940732963856 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(emission_2.port_b,split_valve.port_1)
annotation (Line(
points={{ 230.0, -25.0 }    ,{ 180.0, -25.0 }    ,{ 180.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(valve_2.port_b,emission_2.port_a)
annotation (Line(
points={{ 200.0, -25.0 }    ,{ 215.0, -25.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_2.port,space_2.gainCon)
annotation (Line(
points={{ 150.0, 0.0 }    ,{ 175.0, 0.0 }    ,{ 175.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_2.y,valve_2.y)
annotation (Line(
points={{ 150.0, 0.0 }    ,{ 175.0, 0.0 }    ,{ 175.0, -25.0 }    ,{ 200.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[1:4],merged_w1_3_w2_3_w3_3_w4_3[1:4].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 106.86480296089553, 50.0 }    ,{ 106.86480296089556, -41.289979507721405 }    ,{ -186.2703940782089, -41.289979507721405 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[5],merged_win1_3[1].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 222.22981871830353, 50.0 }    ,{ 222.2298187183035, 183.9567311481972 }    ,{ 44.459637436607046, 183.9567311481972 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[6],floor_4.propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 271.1621754327073, 50.0 }    ,{ 271.1621754327073, -153.1215886369992 }    ,{ 142.32435086541454, -153.1215886369992 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.gainCon,emission_3.heatPortCon)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 415.0, 50.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.gainRad,emission_3.heatPortRad)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 415.0, 50.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.yOcc,occupancy_2.y)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 375.0, 50.0 }    ,{ 375.0, 50.0 }    ,{ 350.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[7],internal_space_1_space_3.propsBus_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[8],internal_space_2_space_3.propsBus_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.gainCon,data_bus.port[3])
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 146.48166172511426, 50.0 }    ,{ 146.4816617251143, -154.49940732963856 }    ,{ -107.03667654977146, -154.49940732963856 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.ports[1],data_bus.port_a[3])
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 146.48166172511426, 50.0 }    ,{ 146.4816617251143, -154.49940732963856 }    ,{ -107.03667654977146, -154.49940732963856 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(emission_3.port_b,split_valve_2.port_1)
annotation (Line(
points={{ 430.0, -25.0 }    ,{ 480.0, -25.0 }    ,{ 480.0, -125.0 }    ,{ 530.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(valve_3.port_b,emission_3.port_a)
annotation (Line(
points={{ 400.0, -25.0 }    ,{ 415.0, -25.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_3.port,space_3.gainCon)
annotation (Line(
points={{ 350.0, 0.0 }    ,{ 375.0, 0.0 }    ,{ 375.0, 50.0 }    ,{ 400.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_3.y,valve_3.y)
annotation (Line(
points={{ 350.0, 0.0 }    ,{ 375.0, 0.0 }    ,{ 375.0, -25.0 }    ,{ 400.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.y,three_way_valve_control.y)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ -125.0, -125.0 }    ,{ -125.0, -125.0 }    ,{ -150.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_2,valve.port_a)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ -50.0, -125.0 }    ,{ -50.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_2,valve_2.port_a)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ 50.0, -125.0 }    ,{ 50.0, -25.0 }    ,{ 200.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_3,split_valve.port_3)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_2,split_valve.port_1)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.y,three_way_valve_control_2.y)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 275.0, -125.0 }    ,{ 275.0, -125.0 }    ,{ 250.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.port_2,valve_3.port_a)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 350.0, -125.0 }    ,{ 350.0, -25.0 }    ,{ 400.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.port_3,split_valve_2.port_3)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 415.0, -125.0 }    ,{ 415.0, -125.0 }    ,{ 530.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.port_2,split_valve_2.port_1)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 415.0, -125.0 }    ,{ 415.0, -125.0 }    ,{ 530.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(split_valve.port_2,boiler.port_a)
annotation (Line(
points={{ 130.0, -125.0 }    ,{ 180.0, -125.0 }    ,{ 180.0, -225.0 }    ,{ 230.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(split_valve_2.port_2,boiler.port_a)
annotation (Line(
points={{ 530.0, -125.0 }    ,{ 380.0, -125.0 }    ,{ 380.0, -225.0 }    ,{ 230.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(boiler.port_b,pump.port_a)
annotation (Line(
points={{ 230.0, -225.0 }    ,{ 15.0, -225.0 }    ,{ 15.0, -225.0 }    ,{ -200.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.m_flow_in,pump_control.y)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ -225.0, -225.0 }    ,{ -225.0, -225.0 }    ,{ -250.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.port_b,three_way_valve.port_1)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ -150.0, -225.0 }    ,{ -150.0, -125.0 }    ,{ -100.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.port_b,three_way_valve_2.port_1)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ 50.0, -225.0 }    ,{ 50.0, -125.0 }    ,{ 300.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));end building;


end ideas_simple_hydronic_three_zones;
