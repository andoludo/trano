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
      extends
           Modelica.Blocks.Interfaces.MO(final nout=3);
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
              extends Trano.Controls.Interfaces.BaseSpaceControl;

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
              extends Trano.Controls.Interfaces.BaseSubstanceSpaceControl;

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


    package ventilation
      model AHU_G36
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
              annotation (Placement(transformation(extent={{-12,-14},{28,74}})));

            BaseClasses.DataBus dataBus annotation (Placement(transformation(
                    extent={{-120,-20},{-80,20}}), iconTransformation(extent={{
                      -112,-10},{-92,10}})));
      equation
            connect(mulAHUCon.ySupFan, dataBus.ySupFan) annotation (Line(points={
                    {30,27.8},{36,27.8},{36,-20},{-74,-20},{-74,0},{-100,0}},
                  color={0,0,127}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(mulAHUCon.y1SupFan, dataBus.y1SupFan) annotation (Line(points
                  ={{30,30},{38,30},{38,-22},{-76,-22},{-76,0},{-100,0}}, color={
                    255,0,255}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(mulAHUCon.yOutDam, dataBus.yOutDam) annotation (Line(points={
                    {30,37},{36,37},{36,20},{34,20},{34,-18},{-70,-18},{-70,0},{
                    -100,0}}, color={0,0,127}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(mulAHUCon.yRetDam, dataBus.yRetDam) annotation (Line(points={
                    {30,43},{36,43},{36,78},{-74,78},{-74,0},{-100,0}}, color={0,
                    0,127}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(mulAHUCon.yMinOutDam, dataBus.yMinOutDam) annotation (Line(
                  points={{30,49},{38,49},{38,80},{-78,80},{-78,24},{-76,24},{-76,
                    0},{-100,0}}, color={0,0,127}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(mulAHUCon.VEffAirOut_flow_min, dataBus.VEffAirOut_flow_min)
              annotation (Line(points={{30,53},{30,58},{34,58},{34,76},{-72,76},{
                    -72,0},{-100,0}}, color={0,0,127}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(mulAHUCon.TAirSupSet, dataBus.TAirSupSet) annotation (Line(
                  points={{30,64},{34,64},{34,70},{32,70},{32,76},{-20,76},{-20,0},
                    {-100,0}}, color={0,0,127}), Text(
                string="%second",
                index=1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.VAirOut_flow, mulAHUCon.VAirOut_flow) annotation (
                Line(
                points={{-100,0},{-22,0},{-22,37},{-14,37}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.uOutAirFra_max, mulAHUCon.uOutAirFra_max) annotation (
               Line(
                points={{-100,0},{-24,0},{-24,47},{-14,47}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.VSumZonPri_flow, mulAHUCon.VSumZonPri_flow)
              annotation (Line(
                points={{-100,0},{-26,0},{-26,50},{-14,50}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.VSumAdjAreBreZon_flow, mulAHUCon.VSumAdjAreBreZon_flow)
              annotation (Line(
                points={{-100,0},{-28,0},{-28,53},{-14,53}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.TAirSup, mulAHUCon.TAirSup) annotation (Line(
                points={{-100,0},{-30,0},{-30,58},{-14,58}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.u1SupFan, mulAHUCon.u1SupFan) annotation (Line(
                points={{-100,0},{-32,0},{-32,61},{-14,61}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.uZonTemResReq, mulAHUCon.uZonTemResReq) annotation (
                Line(
                points={{-100,0},{-34,0},{-34,63},{-14,63}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.TOut, mulAHUCon.TOut) annotation (Line(
                points={{-100,0},{-36,0},{-36,66},{-14,66}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.dpDuc, mulAHUCon.dpDuc) annotation (Line(
                points={{-100,0},{-38,0},{-38,68},{-14,68}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.uZonPreResReq, mulAHUCon.uZonPreResReq) annotation (
                Line(
                points={{-100,0},{-40,0},{-40,71},{-14,71}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.uAhuOpeMod, mulAHUCon.uAhuOpeMod) annotation (Line(
                points={{-100,0},{-42,0},{-42,73},{-14,73}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-6,3},{-6,3}},
                horizontalAlignment=TextAlignment.Right));
            connect(dataBus.VSumAdjPopBreZon_flow, mulAHUCon.VSumAdjPopBreZon_flow)
              annotation (Line(
                points={{-100,0},{-14,0},{-14,55}},
                color={255,204,51},
                thickness=0.5), Text(
                string="%first",
                index=-1,
                extent={{-3,-6},{-3,-6}},
                horizontalAlignment=TextAlignment.Right));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                  graphics={Rectangle(
                    extent={{-100,98},{98,-100}},
                    lineColor={28,108,200},
                    fillColor={28,108,200},
                    fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
                    preserveAspectRatio=false)));
      end AHU_G36;


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
                        annotation (Line(points={{116,77},{116,5.66667},
          {146,5.66667}},
           color={255,0,255}));
        connect(
          boiOn.active, pumOnSig.u[2])
                       annotation (Line(points={{56,77},{56,8},{146,8}},
        color={255,0,255}));
        connect(
          pumOn.active, pumOnSig.u[3])
                       annotation (Line(points={{-4,77},{-4,10.3333},{146,10.3333}},
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
  end Controls;

  package Fluid
  package Boilers


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
        use_inputFilter=false,
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
      Buildings.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
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
          points={{32,-52.3},{32,-52},{64,-52},{64,-24},{68,-24}},
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



  end Boilers;

    package Ventilation

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
          redeclare package                                                        Medium = Medium)
          annotation (Placement(transformation(extent={{-78,-14},{-58,6}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TSup(
          redeclare package Medium = Medium,
          m_flow_nominal=2*100*1.2/3600,
          allowFlowReversal=false)
          annotation (Placement(transformation(extent={{48,6},{68,26}})));
        Controls.BaseClasses.DataBus dataBus annotation (Placement(
              transformation(
          extent={{-120,22},{-80,62}}), iconTransformation(extent={{-208,22},{-188,
            42}})));
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
          extent={{-100,60},{100,-60}},
          lineColor={255,128,0},
          fillColor={255,128,0},
          fillPattern=FillPattern.Forward)}), Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-60},{100,60}})));
      end SimpleHVACBuildings;

    model AhuWithEconomizer
    replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
    "Medium model" annotation (choicesAllMatching=true);

      constant Integer numZon(min=2)=2 "Total number of served VAV boxes";

        parameter Modelica.Units.SI.Volume VRoo[numZon] "Room volume per zone";
        parameter Modelica.Units.SI.Area AFlo[numZon] "Floor area per zone";

        final parameter Modelica.Units.SI.Area ATot=sum(AFlo)
    "Total floor area for all zone";

        constant Real conv=1.2/3600
          "Conversion factor for nominal mass flow rate";


        parameter Modelica.Units.SI.HeatFlowRate QHeaAHU_flow_nominal(min=0) =
          mHeaAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir
          *(THeaAirSup_nominal - THeaAirMix_nominal)
    "Nominal heating heat flow rate of air handler unit coil";

        parameter Modelica.Units.SI.HeatFlowRate QCooAHU_flow_nominal(max=0) = 1.3
          *mCooAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir
          *(TCooAirSup_nominal - TCooAirMix_nominal)
    "Nominal total cooling heat flow rate of air handler unit coil (negative number)";

        parameter Modelica.Units.SI.MassFlowRate mCooVAV_flow_nominal[numZon]
    "Design mass flow rate per zone for cooling"
    annotation (Dialog(group="Nominal mass flow rate"));

        parameter Modelica.Units.SI.MassFlowRate mHeaVAV_flow_nominal[numZon]=0.3
            *mCooVAV_flow_nominal
    "Design mass flow rate per zone for heating"
    annotation (Dialog(group="Nominal mass flow rate"));

        parameter Modelica.Units.SI.MassFlowRate mAir_flow_nominal=0.01
    "Nominal mass flow rate for fan"
    annotation (Dialog(group="Nominal mass flow rate"));
        parameter Modelica.Units.SI.MassFlowRate mCooAir_flow_nominal=0.7*sum(
            mCooVAV_flow_nominal)
    "Nominal mass flow rate for fan"
    annotation (Dialog(group="Nominal mass flow rate"));
        parameter Modelica.Units.SI.MassFlowRate mHeaAir_flow_nominal=0.7*sum(
            mHeaVAV_flow_nominal)
    "Nominal mass flow rate for fan"
    annotation (Dialog(group="Nominal mass flow rate"));

        parameter Modelica.Units.SI.MassFlowRate mHeaWat_flow_nominal=
      QHeaAHU_flow_nominal/Buildings.Utilities.Psychrometrics.Constants.cpWatLiq/10
    "Nominal water mass flow rate for heating coil in AHU"
    annotation (Dialog(group="Nominal mass flow rate"));
        parameter Modelica.Units.SI.MassFlowRate mCooWat_flow_nominal=
      QCooAHU_flow_nominal/Buildings.Utilities.Psychrometrics.Constants.cpWatLiq/(-6)
    "Nominal water mass flow rate for cooling coil"
    annotation (Dialog(group="Nominal mass flow rate"));

        parameter Real ratOAFlo_A(final unit="m3/(s.m2)") = 0.3e-3
    "Outdoor airflow rate required per unit area";
        parameter Real ratOAFlo_P=2.5e-3
    "Outdoor airflow rate required per person";
        parameter Real ratP_A=5e-2
    "Occupant density";
        parameter Real effZ(final unit="1") = 0.8
    "Zone air distribution effectiveness (limiting value)";
        parameter Real divP(final unit="1") = 0.7
    "Occupant diversity ratio";

        parameter Modelica.Units.SI.VolumeFlowRate VZonOA_flow_nominal[numZon]=
                                                                         (
      ratOAFlo_P*ratP_A + ratOAFlo_A)*AFlo/effZ
    "Zone outdoor air flow rate of each VAV box";

        parameter Modelica.Units.SI.VolumeFlowRate Vou_flow_nominal=
                                                              (divP*ratOAFlo_P*
      ratP_A + ratOAFlo_A)*sum(AFlo) "System uncorrected outdoor air flow rate";
        parameter Real effVen(final unit="1") = if divP < 0.6 then
    0.88 * divP + 0.22 else 0.75
    "System ventilation efficiency";
        parameter Modelica.Units.SI.VolumeFlowRate Vot_flow_nominal=
            Vou_flow_nominal/
      effVen "System design outdoor air flow rate";

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


        parameter Modelica.Units.SI.Temperature TCooAirMix_nominal(displayUnit="degC")
           = 303.15
    "Mixed air temperature during cooling nominal conditions (used to size cooling coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature TCooAirSup_nominal(displayUnit="degC")
           = 285.15
    "Supply air temperature during cooling nominal conditions (used to size cooling coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.MassFraction wCooAirMix_nominal=0.017
    "Humidity ratio of mixed air at a nominal conditions used to size cooling coil (in kg/kg dry total)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature TCooWatInl_nominal(displayUnit="degC")
           = 279.15
    "Cooling coil nominal inlet water temperature"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));


        parameter Modelica.Units.SI.Temperature THeaAirMix_nominal(displayUnit="degC")
           = 277.15
    "Mixed air temperature during heating nominal conditions (used to size heating coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature THeaAirSup_nominal(displayUnit="degC")
           = 285.15
    "Supply air temperature during heating nominal conditions (used to size heating coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature THeaWatInl_nominal(displayUnit="degC")
    "Reheat coil nominal inlet water temperature"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));

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
            =
          MediumA, m_flow_nominal=mAir_flow_nominal)
    "Outside air volume flow rate"
    annotation (Placement(transformation(extent={{-68,-80},{-48,-60}})));
        Buildings.Fluid.Actuators.Dampers.Exponential
                                      damRet(
      redeclare package Medium = MediumA,
      m_flow_nominal=mAir_flow_nominal,
      from_dp=false,
      riseTime=15,
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
      riseTime=15,
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
      linearized=true)
    "Flow splitter"
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={52,-70})));
        Buildings.Fluid.Actuators.Dampers.Exponential damExh(
      from_dp=false,
      riseTime=15,
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
            Medium =
        MediumA)
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{556,-5},{524,25}}),
        iconTransformation(extent={{552,11},{532,29}})));
        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package
            Medium =
        MediumA)
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{522,-90},{556,-58}}),
        iconTransformation(extent={{532,-69},{552,-51}})));
        Modelica.Fluid.Interfaces.FluidPorts_b ports[2](redeclare each package
            Medium =
               MediumA, each m_flow(max=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Leaving
           then 0 else +Modelica.Constants.inf, min=if flowDirection ==
          Modelica.Fluid.Types.PortFlowDirection.Entering then 0 else -Modelica.Constants.inf))
    "Fluid ports"
    annotation (Placement(transformation(extent={{-110,26},{-90,-54}}),
        iconTransformation(extent={{-110,26},{-90,-54}})));
        Controls.BaseClasses.DataBus dataBus annotation (Placement(
              transformation(
          extent={{-90,78},{-50,118}}), iconTransformation(extent={{-84,54},{-34,
            96}})));
        Modelica.Blocks.Math.RealToBoolean u1SupFan(threshold=0.2)
    "Convert real to integer"
    annotation (Placement(transformation(extent={{316,30},{356,70}})));
        Buildings.Controls.OBC.CDL.Integers.Sources.Constant opeMod(final k=
              Buildings.Controls.OBC.ASHRAE.G36.Types.OperationModes.occupied)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-66,-170},{-46,-150}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant sumDesPopBreZon(
            final k=0.0125)
    "Sum of the population component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{200,-160},{220,-140}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant sumDesAreBreZon(
            final k=0.03)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{324,-160},{344,-140}})));
        Buildings.Fluid.Sensors.RelativePressure dpDisSupFan(redeclare package
            Medium =
        MediumA) "Supply fan static discharge pressure" annotation (Placement(
        transformation(
        extent={{-18,22},{18,-22}},
        rotation=90,
        origin={404,-28})));
        Buildings.Controls.OBC.CDL.Integers.Sources.Constant ducPreResReq(
            final k=2)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-64,-218},{-44,-198}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TOut(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAir_flow_nominal,
    allowFlowReversal=allowFlowReversal,
    transferHeat=true) "Mixed air temperature sensor"
    annotation (Placement(transformation(extent={{-32,-80},{-12,-60}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant sumDesPopBreZon1(
            final k=0.04)
    "Sum of the population component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{264,-160},{284,-140}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant VSumZonPri_flow(
            final k=0.03)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{444,-112},{464,-92}})));
        Buildings.Controls.OBC.CDL.Reals.Sources.Constant uOutAirFra_max(final
            k=0.5)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{498,-122},{518,-102}})));
        Buildings.Controls.OBC.CDL.Integers.Sources.Constant maxSupResReq(
            final k=6)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-74,-268},{-54,-248}})));

      protected
        parameter Modelica.Fluid.Types.PortFlowDirection flowDirection=Modelica.Fluid.Types.PortFlowDirection.Bidirectional
    "Allowed flow direction" annotation (Evaluate=true, Dialog(tab="Advanced"));
    equation
        connect(
          TSup.port_a,fanSup. port_b) annotation (Line(
      points={{276,-72},{266,-72}},
      color={0,127,255},
      smooth=Smooth.None,
      thickness=0.5));
        connect(
          TSup.port_b,senSupFlo. port_a)
    annotation (Line(points={{296,-72},{346,-72}}, color={0,127,255}));
        connect(
          dpSupDuc.port_b,fanSup. port_a)
    annotation (Line(points={{216,-72},{246,-72}}, color={0,127,255}));
        connect(
          damOut.port_b,splRetOut. port_1)
    annotation (Line(points={{22,-70},{42,-70}},   color={0,127,255}));
        connect(
          splRetOut.port_2,TMix. port_a)
    annotation (Line(points={{62,-70},{72,-70},{72,-72},{82,-72}},
                                                 color={0,127,255}));
        connect(
          damRet.port_b,splRetOut. port_3) annotation (Line(points={{52,-16},{52,
          -60}},                                    color={0,127,255}));
    connect(dpSupDuc.port_a, TMix.port_b) annotation (Line(points={{196,-72},{102,
          -72}},               color={0,127,255}));
        connect(
          senRetFlo.port_b,TRet. port_a) annotation (Line(points={{214,10},{138,
          10}},                color={0,127,255}));
    connect(TRet.port_b, damRet.port_a) annotation (Line(points={{118,10},{52,10},
          {52,4}},                 color={0,127,255}));
    connect(TRet.port_b, damExh.port_a) annotation (Line(points={{118,10},{-6,10},
          {-6,-4},{-16,-4}},                                        color=
           {0,127,255}));
    connect(senRetFlo.port_a, fanSup1.port_b)
      annotation (Line(points={{234,10},{258,10}},   color={0,127,255}));
    connect(fanSup1.port_a, dpRetDuc.port_b)
      annotation (Line(points={{278,10},{348,10}},   color={0,127,255}));
        connect(
          senSupFlo.port_b, port_a) annotation (Line(points={{366,-72},{516,-72},
          {516,-74},{539,-74}}, color={0,127,255}));
        connect(
          dpRetDuc.port_a, port_b)
    annotation (Line(points={{368,10},{540,10}}, color={0,127,255}));
        connect(
          damExh.port_b, ports[1]) annotation (Line(points={{-36,-4},{-84,-4},{-84,
          6},{-100,6}}, color={0,127,255}));
        connect(
          VOut1.port_a, ports[2]) annotation (Line(points={{-68,-70},{-84,-70},{
          -84,-34},{-100,-34}}, color={0,127,255}));
        connect(
          dataBus.yRetDam, damRet.y) annotation (Line(
      points={{-70,98},{-70,-24},{32,-24},{32,-6},{40,-6}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          dataBus.yOutDam, damOut.y) annotation (Line(
      points={{-70,98},{-70,-48},{12,-48},{12,-58}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          dataBus.ySupFan, fanSup1.y) annotation (Line(
      points={{-70,98},{-70,32},{268,32},{268,22}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          dataBus.ySupFan, fanSup.y) annotation (Line(
      points={{-70,98},{-70,32},{248,32},{248,-52},{256,-52},{256,-60}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          dataBus.yOutDam, damExh.y) annotation (Line(
      points={{-70,98},{-70,18},{-26,18},{-26,8}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          opeMod.y, dataBus.uAhuOpeMod) annotation (Line(points={{-44,-160},{-34,
          -160},{-34,-158},{-36,-158},{-36,-154},{-34,-154},{-34,-86},{-38,-86},
          {-38,-52},{-70,-52},{-70,98}}, color={255,127,0}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
        connect(
          dpDisSupFan.port_a, port_a) annotation (Line(points={{404,-46},{404,-72},
          {516,-72},{516,-74},{539,-74}}, color={0,127,255}));
        connect(
          VOut1.port_a, dpDisSupFan.port_b) annotation (Line(points={{-68,-70},{
          -84,-70},{-84,-26},{376,-26},{376,0},{404,0},{404,-10}}, color={0,127,
          255}));
        connect(
          dpDisSupFan.p_rel, dataBus.dpDuc) annotation (Line(points={{384.2,-28},
          {-70,-28},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          ducPreResReq.y, dataBus.uZonPreResReq) annotation (Line(points={{-42,-208},
          {-38,-208},{-38,-204},{-40,-204},{-40,-200},{-38,-200},{-38,-178},{-70,
          -178},{-70,98}}, color={255,127,0}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          VOut1.port_b, TOut.port_a)
    annotation (Line(points={{-48,-70},{-32,-70}}, color={0,127,255}));
        connect(
          TOut.port_b, damOut.port_a)
    annotation (Line(points={{-12,-70},{2,-70}}, color={0,127,255}));
        connect(
          TSup.T, dataBus.TAirSup) annotation (Line(points={{286,-61},{136,-61},
          {136,38},{-70,38},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          dataBus.ySupFan, u1SupFan.u) annotation (Line(
      points={{-70,98},{-70,50},{312,50}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          u1SupFan.y, dataBus.u1SupFan) annotation (Line(points={{358,50},{366,50},
          {366,76},{-44,76},{-44,72},{-70,72},{-70,98}}, color={255,0,255}),
      Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          sumDesPopBreZon.y, dataBus.VSumAdjPopBreZon_flow) annotation (Line(
        points={{222,-150},{230,-150},{230,-6},{200,-6},{200,48},{-70,48},{-70,98}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          sumDesAreBreZon.y, dataBus.VSumAdjAreBreZon_flow) annotation (Line(
        points={{346,-150},{364,-150},{364,-26},{-70,-26},{-70,98}}, color={0,0,
          127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
        connect(
          TOut.T, dataBus.TOut) annotation (Line(points={{-22,-59},{-22,-50},{-70,
          -50},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          VOut1.V_flow, dataBus.VAirOut_flow) annotation (Line(points={{-58,-59},
          {-38,-59},{-38,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
        connect(
          VSumZonPri_flow.y, dataBus.VSumZonPri_flow) annotation (Line(points={{
          466,-102},{200,-102},{200,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          uOutAirFra_max.y, dataBus.uOutAirFra_max) annotation (Line(points={{520,
          -112},{226,-112},{226,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
        connect(
          maxSupResReq.y, dataBus.uZonTemResReq) annotation (Line(points={{-52,-258},
          {-44,-258},{-44,-224},{-70,-224},{-70,98}}, color={255,127,0}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},
            {540,100}}), graphics={Rectangle(
          extent={{-98,102},{542,-138}},
          lineColor={28,108,200},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid)}),
                                         Diagram(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-180},{540,100}})));
    end AhuWithEconomizer;

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

        parameter Modelica.Units.SI.HeatFlowRate QHeaAHU_flow_nominal(min=0) =
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

        parameter Modelica.Units.SI.Temperature TCooAirMix_nominal(displayUnit="degC")
           = 303.15
          "Mixed air temperature during cooling nominal conditions (used to size cooling coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature TCooAirSup_nominal(displayUnit="degC")
           = 285.15
          "Supply air temperature during cooling nominal conditions (used to size cooling coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.MassFraction wCooAirMix_nominal=0.017
          "Humidity ratio of mixed air at a nominal conditions used to size cooling coil (in kg/kg dry total)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature TCooWatInl_nominal(displayUnit="degC")
           = 279.15 "Cooling coil nominal inlet water temperature" annotation (
            Dialog(group="Air handler unit nominal temperatures and humidity"));

        parameter Modelica.Units.SI.Temperature THeaAirMix_nominal(displayUnit="degC")
           = 277.15
          "Mixed air temperature during heating nominal conditions (used to size heating coil)"
          annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
        parameter Modelica.Units.SI.Temperature THeaAirSup_nominal(displayUnit="degC")
           = 285.15
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
          riseTime=15,
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
          riseTime=15,
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
          riseTime=15,
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
              iconTransformation(extent={{552,11},{532,29}})));
        Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package
            Medium = MediumA)
          "Fluid connector a (positive design flow direction is from port_a to port_b)"
          annotation (Placement(transformation(extent={{522,-90},{556,-58}}),
              iconTransformation(extent={{532,-69},{552,-51}})));
        Modelica.Fluid.Interfaces.FluidPorts_b ports[2](redeclare each package
            Medium =
           MediumA, each m_flow(max=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Leaving then 0
                 else +Modelica.Constants.inf, min=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Entering
                 then 0 else -Modelica.Constants.inf)) "Fluid ports"
          annotation (Placement(transformation(extent={{-110,26},{-90,-54}}),
              iconTransformation(extent={{-110,26},{-90,-54}})));
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
                               annotation (Line(points={{-36,-4},{-84,-4},{-84,6},{-100,
                6}}, color={0,127,255}));
        connect(VOut1.port_a, ports[2])
                              annotation (Line(points={{-68,-70},{-84,-70},{-84,-34},{
                -100,-34}}, color={0,127,255}));
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
                extent={{-98,102},{542,-138}},
                lineColor={28,108,200},
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid)}),
                                     Diagram(coordinateSystem(
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
       Buildings.Fluid.Sensors.TemperatureTwoPort temSup(redeclare package Medium =
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
        extends
      Modelica.Blocks.Icons.Block;
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

  annotation (uses(Buildings(version = "11.0.0"), Modelica(version = "4.0.0"),
      IDEAS(version="3.0.0")),
  Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
            fillPattern =                                                                            FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
end Trano;


package Components
  package Containers
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
conPum(    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
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
        controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
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
conPum(    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
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
            TCooSetEmissioncontrol_3
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_7
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_2
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
            TCooSetEmissioncontrol_5
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            y_gainCollectorcontrol_1
            (y=0.0);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmissioncontrol_0
            (y=298.15);
Modelica.Blocks.Sources.BooleanExpression
            triggerThreewayvalvecontrol_0
            (y=true);
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
connect(dataBus.TCooSetSpace_4,
TCooSetEmissioncontrol_3.y);
connect(dataBus.TCooSetSpace_10,
TCooSetEmissioncontrol_7.y);
connect(dataBus.TCooSetSpace_3,
TCooSetEmissioncontrol_2.y);
connect(dataBus.TColSetThreewayvalvecontrol_0,
TColSetThreewayvalvecontrol_0.y);
connect(dataBus.TCooSetSpace_5,
TCooSetEmissioncontrol_4.y);
connect(dataBus.TCooSetSpace_2,
TCooSetEmissioncontrol_1.y);
connect(dataBus.TCooSetSpace_6,
TCooSetEmissioncontrol_5.y);
connect(dataBus.y_gainBoiler_0,
y_gainCollectorcontrol_1.y);
connect(dataBus.TCooSetSpace_1,
TCooSetEmissioncontrol_0.y);
connect(dataBus.triggerThreewayvalvecontrol_0,
triggerThreewayvalvecontrol_0.y);
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
connect(dataBus.TCollectorcontrol_0, temSup.T);
 end PumpPump_0;
 
  end BaseClasses;
end Components;


model building
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



package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";
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
    Placement(transformation(origin = { 0.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 30.0, -75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_2(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 0.0, -75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_2
    emissioncontrol_2(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 527.2655079632606, 244.19848462338433 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_2
    occupancy_2(    gain=[35; 70; 30],
    k=0.15,
    occupancy=3600*{0.1,2,15,24}
) annotation (
    Placement(transformation(origin = { -50.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 250.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 280.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_1(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 250.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_1
    emissioncontrol_1(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 375.7388114843199, 442.98766899420593 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 200.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 500.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 530.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_0(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 500.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_0
    emissioncontrol_0(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 294.12439086281995, 367.84281681770915 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_0
    occupancy_0(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 450.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 0.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 30.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_5(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 0.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_5
    emissioncontrol_5(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 460.9533543002585, 259.5453870153023 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_5
    occupancy_5(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -50.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 250.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 280.0, 225.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_6(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 250.0, 225.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_6
    emissioncontrol_6(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 240.40217908755426, 374.22374090031195 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_8
    occupancy_8(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 200.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 500.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_6
    occupancy_6(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 450.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 0.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_7
    occupancy_7(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -50.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 250.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=52.6,
    VWat=0.116
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 280.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_7(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 250.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_7
    emissioncontrol_7(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 299.3463156768894, 279.2007131184074 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_9
    occupancy_9(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 200.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 500.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_10
    occupancy_10(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 450.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 0.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 30.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_4(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 0.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_4
    emissioncontrol_4(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 434.1027342054251, 414.7749220026742 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_4
    occupancy_4(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -50.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
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
    Placement(transformation(origin = { 250.0, 600.0 },
    extent = {{10, -10}, {-10, 10}}
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
    linearized=false,
    from_dp=false,
    T_a_nominal=363.15,
    T_b_nominal=353.15,
    mDry=105.2,
    VWat=0.232
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator"  annotation (
    Placement(transformation(origin = { 280.0, 525.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve_3(
                    dpFixed_nominal=6000.0,
    dpValve_nominal=6000.0,
    deltaM=0.02,
    m_flow_nominal=0.01,
    delta0=0.01,
    R=50.0,
    linearized=false,
    from_dp=true,
    l=0.0001
,
    redeclare package Medium = MediumW

    ) "Radiator valve"  annotation (
    Placement(transformation(origin = { 250.0, 525.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.EmissionControlEmissioncontrol_3
    emissioncontrol_3(    schedule=3600*{7, 19},
    THeaSet=24.0,
    THeaSetBack=16.0
) annotation (
    Placement(transformation(origin = { 314.43886677940054, 470.8958549992571 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.OccupancyOccupancy_3
    occupancy_3(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{9, 12, 17,22}
) annotation (
    Placement(transformation(origin = { 200.0, 600.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather_19(    filNam=/home/aan/Documents/trano/tests/resources/BEL_VLG_Uccle.064470_TMYx.2007-2021.mos
)
     annotation (
    Placement(transformation(origin = { -100.0, 200.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_1(A =
            11.368, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 125.0, 525.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_2(A =
            3.64, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 0.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_3(A =
            6.44, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -125.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_4(A =
            8, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 125.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_5(A =
            7.22, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 125.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_6(A =
            7.22, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -125.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_7(A =
            6.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 0.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_8(A =
            1.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 0.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_9(A =
            6.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -250.0, 225.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_10(A =
            5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 0.0, 225.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_11(A =
            5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 250.0, 300.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_12(A =
            6.5, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 250.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_13(A =
            12.321, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 125.0, 375.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_14(A =
            3.33, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 125.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internalelement_15(A =
            15.7, layers =
    construction_5, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 0.0, 450.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperaturesensor_0(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { 247.60941400496466, 241.70999851433663 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.FixedResistances.Junction splitvalve_0 (
        dp_nominal={10000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.008*{1,-1,-1},
    linearized=false
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { 388.33574390058914, 284.51938790670033 },
    extent = {{10, -10}, {-10, 10}}
)));
      house_model.Components.BaseClasses.PumpPump_0
     pump_0(
         dp_nominal=10000.0,
    m_flow_nominal=0.008
,
    redeclare package Medium = MediumW

    ) annotation (
    Placement(transformation(origin = { 236.71817055072083, 50.38924379735552 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.CollectorControlCollectorcontrol_0
    collectorcontrol_0 annotation (
    Placement(transformation(origin = { 367.1404613888643, 174.84029119001636 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             threewayvalve_0(
    redeclare package Medium = MediumW,
    use_inputFilter=false,
        dpFixed_nominal={100,0},
    dpValve_nominal=6000.0,
    fraK=0.7,
    deltaM=0.02,
    m_flow_nominal=0.0078,
    delta0=0.01,
    R=50.0,
    linearized={false, false},
    l={0.01,0.01}
,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve"  annotation (
    Placement(transformation(origin = { 328.6925544579649, 61.92987668994206 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.
    ThreeWayValveControlThreewayvalvecontrol_0
    threewayvalvecontrol_0 annotation (
    Placement(transformation(origin = { 325.10641332059186, 212.1081562917843 },
    extent = {{10, -10}, {-10, 10}}
)));
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
    linearizeFlowResistance=false,
    nominal_mass_flow_radiator_loop=0.21428571428571427,
    nominal_mass_flow_rate_boiler=0.21428571428571427,
    V_flow=0.21428571428571427/1000*{0.5,1}
,
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()) "Boiler"  annotation (
    Placement(transformation(origin = { 420.93257844945873, 44.52161640172337 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.CollectorControlCollectorcontrol_1
    collectorcontrol_1 annotation (
    Placement(transformation(origin = { 498.2338429661652, 166.498291487149 },
    extent = {{10, -10}, {-10, 10}}
)));
        house_model.Components.BaseClasses.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 493.82281946191097, 416.29029861833305 },
    extent = {{10, -10}, {-10, 10}}
)));


equation            
        connect(space_3.heaPorRad,radiator_2.heatPortRad)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 15.0, 0.0 }    ,{ 15.0, -75.0 }    ,{ 30.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.heaPorAir,radiator_2.heatPortCon)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 15.0, 0.0 }    ,{ 15.0, -75.0 }    ,{ 30.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.qGai_flow,occupancy_2.y)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -50.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -50.0, 0.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.surf_surBou[1],internalelement_6.port_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -62.5, 0.0 }    ,{ -62.5, 75.0 }    ,{ -125.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.surf_surBou[2],internalelement_7.port_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 75.0 }    ,{ 0.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.surf_surBou[3],internalelement_8.port_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 150.0 }    ,{ 0.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_2.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 30.0, -75.0 }    ,{ 209.16787195029457, -75.0 }    ,{ 209.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_2.y,emissioncontrol_2.y)
        annotation (Line(
        points={{ 0.0, -75.0 }    ,{ 263.6327539816303, -75.0 }    ,{ 263.6327539816303, 244.19848462338433 }    ,{ 527.2655079632606, 244.19848462338433 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_2.port_b,radiator_2.port_a)
        annotation (Line(
        points={{ 0.0, -75.0 }    ,{ 15.0, -75.0 }    ,{ 15.0, -75.0 }    ,{ 30.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.heaPorRad,radiator_1.heatPortRad)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 265.0, 150.0 }    ,{ 265.0, 75.0 }    ,{ 280.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.heaPorAir,radiator_1.heatPortCon)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 265.0, 150.0 }    ,{ 265.0, 75.0 }    ,{ 280.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.qGai_flow,occupancy_1.y)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 225.0, 150.0 }    ,{ 225.0, 150.0 }    ,{ 200.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 75.0, 150.0 }    ,{ 75.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.surf_surBou[1],internalelement_2.port_a)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 125.0, 150.0 }    ,{ 125.0, 375.0 }    ,{ 0.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.surf_surBou[2],internalelement_5.port_a)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 187.5, 150.0 }    ,{ 187.5, 150.0 }    ,{ 125.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.surf_surBou[3],internalelement_6.port_b)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 62.5, 150.0 }    ,{ 62.5, 75.0 }    ,{ -125.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_1.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 280.0, 75.0 }    ,{ 334.16787195029457, 75.0 }    ,{ 334.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_1.y,emissioncontrol_1.y)
        annotation (Line(
        points={{ 250.0, 75.0 }    ,{ 312.86940574216, 75.0 }    ,{ 312.86940574216, 442.98766899420593 }    ,{ 375.7388114843199, 442.98766899420593 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_1.port_b,radiator_1.port_a)
        annotation (Line(
        points={{ 250.0, 75.0 }    ,{ 265.0, 75.0 }    ,{ 265.0, 75.0 }    ,{ 280.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.heaPorRad,radiator_0.heatPortRad)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 515.0, 150.0 }    ,{ 515.0, 75.0 }    ,{ 530.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.heaPorAir,radiator_0.heatPortCon)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 515.0, 150.0 }    ,{ 515.0, 75.0 }    ,{ 530.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.qGai_flow,occupancy_0.y)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 475.0, 150.0 }    ,{ 475.0, 150.0 }    ,{ 450.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 200.0, 150.0 }    ,{ 200.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.surf_surBou[1],internalelement_3.port_a)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 187.5, 150.0 }    ,{ 187.5, 375.0 }    ,{ -125.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.surf_surBou[2],internalelement_5.port_b)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 312.5, 150.0 }    ,{ 312.5, 150.0 }    ,{ 125.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_0.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 530.0, 75.0 }    ,{ 459.16787195029457, 75.0 }    ,{ 459.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_0.y,emissioncontrol_0.y)
        annotation (Line(
        points={{ 500.0, 75.0 }    ,{ 397.06219543141, 75.0 }    ,{ 397.06219543141, 367.84281681770915 }    ,{ 294.12439086281995, 367.84281681770915 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_0.port_b,radiator_0.port_a)
        annotation (Line(
        points={{ 500.0, 75.0 }    ,{ 515.0, 75.0 }    ,{ 515.0, 75.0 }    ,{ 530.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.heaPorRad,radiator_5.heatPortRad)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ 15.0, 150.0 }    ,{ 15.0, 75.0 }    ,{ 30.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.heaPorAir,radiator_5.heatPortCon)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ 15.0, 150.0 }    ,{ 15.0, 75.0 }    ,{ 30.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.qGai_flow,occupancy_5.y)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ -25.0, 150.0 }    ,{ -25.0, 150.0 }    ,{ -50.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ -50.0, 150.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.surf_surBou[1],internalelement_7.port_b)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ 0.0, 150.0 }    ,{ 0.0, 75.0 }    ,{ 0.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.surf_surBou[2],internalelement_9.port_a)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ -125.0, 150.0 }    ,{ -125.0, 225.0 }    ,{ -250.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.surf_surBou[3],internalelement_10.port_a)
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ 0.0, 150.0 }    ,{ 0.0, 225.0 }    ,{ 0.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_5.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 30.0, 75.0 }    ,{ 209.16787195029457, 75.0 }    ,{ 209.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_5.y,emissioncontrol_5.y)
        annotation (Line(
        points={{ 0.0, 75.0 }    ,{ 230.47667715012926, 75.0 }    ,{ 230.47667715012926, 259.5453870153023 }    ,{ 460.9533543002585, 259.5453870153023 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_5.port_b,radiator_5.port_a)
        annotation (Line(
        points={{ 0.0, 75.0 }    ,{ 15.0, 75.0 }    ,{ 15.0, 75.0 }    ,{ 30.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.heaPorRad,radiator_6.heatPortRad)
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 265.0, 300.0 }    ,{ 265.0, 225.0 }    ,{ 280.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.heaPorAir,radiator_6.heatPortCon)
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 265.0, 300.0 }    ,{ 265.0, 225.0 }    ,{ 280.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.qGai_flow,occupancy_8.y)
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 225.0, 300.0 }    ,{ 225.0, 300.0 }    ,{ 200.0, 300.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 75.0, 300.0 }    ,{ 75.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.surf_surBou[1],internalelement_15.port_a)
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 125.0, 300.0 }    ,{ 125.0, 450.0 }    ,{ 0.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_6.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 280.0, 225.0 }    ,{ 334.16787195029457, 225.0 }    ,{ 334.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_6.y,emissioncontrol_6.y)
        annotation (Line(
        points={{ 250.0, 225.0 }    ,{ 245.20108954377713, 225.0 }    ,{ 245.20108954377713, 374.22374090031195 }    ,{ 240.40217908755426, 374.22374090031195 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_6.port_b,radiator_6.port_a)
        annotation (Line(
        points={{ 250.0, 225.0 }    ,{ 265.0, 225.0 }    ,{ 265.0, 225.0 }    ,{ 280.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.qGai_flow,occupancy_6.y)
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 475.0, 300.0 }    ,{ 475.0, 300.0 }    ,{ 450.0, 300.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 200.0, 300.0 }    ,{ 200.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.surf_surBou[1],internalelement_9.port_b)
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 125.0, 300.0 }    ,{ 125.0, 225.0 }    ,{ -250.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.surf_surBou[2],internalelement_11.port_a)
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 375.0, 300.0 }    ,{ 375.0, 300.0 }    ,{ 250.0, 300.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.surf_surBou[3],internalelement_12.port_a)
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 375.0, 300.0 }    ,{ 375.0, 375.0 }    ,{ 250.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.qGai_flow,occupancy_7.y)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ -25.0, 300.0 }    ,{ -25.0, 300.0 }    ,{ -50.0, 300.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ -50.0, 300.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.surf_surBou[1],internalelement_4.port_a)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 62.5, 300.0 }    ,{ 62.5, 450.0 }    ,{ 125.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.surf_surBou[2],internalelement_8.port_b)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 0.0, 300.0 }    ,{ 0.0, 150.0 }    ,{ 0.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.surf_surBou[3],internalelement_10.port_b)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 0.0, 300.0 }    ,{ 0.0, 225.0 }    ,{ 0.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.surf_surBou[4],internalelement_11.port_b)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 125.0, 300.0 }    ,{ 125.0, 300.0 }    ,{ 250.0, 300.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.surf_surBou[5],internalelement_13.port_a)
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 62.5, 300.0 }    ,{ 62.5, 375.0 }    ,{ 125.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.heaPorRad,radiator_7.heatPortRad)
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 265.0, 450.0 }    ,{ 265.0, 375.0 }    ,{ 280.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.heaPorAir,radiator_7.heatPortCon)
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 265.0, 450.0 }    ,{ 265.0, 375.0 }    ,{ 280.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.qGai_flow,occupancy_9.y)
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 225.0, 450.0 }    ,{ 225.0, 450.0 }    ,{ 200.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 75.0, 450.0 }    ,{ 75.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.surf_surBou[1],internalelement_13.port_b)
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 187.5, 450.0 }    ,{ 187.5, 375.0 }    ,{ 125.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.surf_surBou[2],internalelement_14.port_a)
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 187.5, 450.0 }    ,{ 187.5, 450.0 }    ,{ 125.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_7.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 280.0, 375.0 }    ,{ 334.16787195029457, 375.0 }    ,{ 334.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_7.y,emissioncontrol_7.y)
        annotation (Line(
        points={{ 250.0, 375.0 }    ,{ 274.6731578384447, 375.0 }    ,{ 274.6731578384447, 279.2007131184074 }    ,{ 299.3463156768894, 279.2007131184074 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_7.port_b,radiator_7.port_a)
        annotation (Line(
        points={{ 250.0, 375.0 }    ,{ 265.0, 375.0 }    ,{ 265.0, 375.0 }    ,{ 280.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_11.qGai_flow,occupancy_10.y)
        annotation (Line(
        points={{ 500.0, 450.0 }    ,{ 475.0, 450.0 }    ,{ 475.0, 450.0 }    ,{ 450.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_11.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 500.0, 450.0 }    ,{ 200.0, 450.0 }    ,{ 200.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.heaPorRad,radiator_4.heatPortRad)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 15.0, 450.0 }    ,{ 15.0, 375.0 }    ,{ 30.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.heaPorAir,radiator_4.heatPortCon)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 15.0, 450.0 }    ,{ 15.0, 375.0 }    ,{ 30.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.qGai_flow,occupancy_4.y)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ -25.0, 450.0 }    ,{ -25.0, 450.0 }    ,{ -50.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ -50.0, 450.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.surf_surBou[1],internalelement_1.port_a)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 62.5, 450.0 }    ,{ 62.5, 525.0 }    ,{ 125.0, 525.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.surf_surBou[2],internalelement_12.port_b)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 125.0, 450.0 }    ,{ 125.0, 375.0 }    ,{ 250.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.surf_surBou[3],internalelement_14.port_b)
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 62.5, 450.0 }    ,{ 62.5, 450.0 }    ,{ 125.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_4.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 30.0, 375.0 }    ,{ 209.16787195029457, 375.0 }    ,{ 209.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_4.y,emissioncontrol_4.y)
        annotation (Line(
        points={{ 0.0, 375.0 }    ,{ 217.05136710271256, 375.0 }    ,{ 217.05136710271256, 414.7749220026742 }    ,{ 434.1027342054251, 414.7749220026742 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_4.port_b,radiator_4.port_a)
        annotation (Line(
        points={{ 0.0, 375.0 }    ,{ 15.0, 375.0 }    ,{ 15.0, 375.0 }    ,{ 30.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.heaPorRad,radiator_3.heatPortRad)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 265.0, 600.0 }    ,{ 265.0, 525.0 }    ,{ 280.0, 525.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.heaPorAir,radiator_3.heatPortCon)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 265.0, 600.0 }    ,{ 265.0, 525.0 }    ,{ 280.0, 525.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.qGai_flow,occupancy_3.y)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 225.0, 600.0 }    ,{ 225.0, 600.0 }    ,{ 200.0, 600.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.weaBus,weather_19.weaBus)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 75.0, 600.0 }    ,{ 75.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.surf_surBou[1],internalelement_1.port_b)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 187.5, 600.0 }    ,{ 187.5, 525.0 }    ,{ 125.0, 525.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.surf_surBou[2],internalelement_2.port_b)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 125.0, 600.0 }    ,{ 125.0, 375.0 }    ,{ 0.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.surf_surBou[3],internalelement_3.port_b)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 62.5, 600.0 }    ,{ 62.5, 375.0 }    ,{ -125.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.surf_surBou[4],internalelement_4.port_b)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 187.5, 600.0 }    ,{ 187.5, 450.0 }    ,{ 125.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.surf_surBou[5],internalelement_15.port_b)
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 125.0, 600.0 }    ,{ 125.0, 450.0 }    ,{ 0.0, 450.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(radiator_3.port_b,splitvalve_0.port_1)
        annotation (Line(
        points={{ 280.0, 525.0 }    ,{ 334.16787195029457, 525.0 }    ,{ 334.16787195029457, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_3.y,emissioncontrol_3.y)
        annotation (Line(
        points={{ 250.0, 525.0 }    ,{ 282.21943338970027, 525.0 }    ,{ 282.21943338970027, 470.8958549992571 }    ,{ 314.43886677940054, 470.8958549992571 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_3.port_b,radiator_3.port_a)
        annotation (Line(
        points={{ 250.0, 525.0 }    ,{ 265.0, 525.0 }    ,{ 265.0, 525.0 }    ,{ 280.0, 525.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_2.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 123.80470700248233, 241.70999851433663 }    ,{ 123.80470700248233, -75.0 }    ,{ 0.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_1.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 248.80470700248233, 241.70999851433663 }    ,{ 248.80470700248233, 75.0 }    ,{ 250.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_0.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 373.80470700248236, 241.70999851433663 }    ,{ 373.80470700248236, 75.0 }    ,{ 500.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_5.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 123.80470700248233, 241.70999851433663 }    ,{ 123.80470700248233, 75.0 }    ,{ 0.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_6.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 248.80470700248233, 241.70999851433663 }    ,{ 248.80470700248233, 225.0 }    ,{ 250.0, 225.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_7.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 248.80470700248233, 241.70999851433663 }    ,{ 248.80470700248233, 375.0 }    ,{ 250.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_4.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 123.80470700248233, 241.70999851433663 }    ,{ 123.80470700248233, 375.0 }    ,{ 0.0, 375.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperaturesensor_0.port_b,valve_3.port_a)
        annotation (Line(
        points={{ 247.60941400496466, 241.70999851433663 }    ,{ 248.80470700248233, 241.70999851433663 }    ,{ 248.80470700248233, 525.0 }    ,{ 250.0, 525.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(splitvalve_0.port_2,boiler_0.port_a)
        annotation (Line(
        points={{ 388.33574390058914, 284.51938790670033 }    ,{ 404.6341611750239, 284.51938790670033 }    ,{ 404.6341611750239, 44.52161640172337 }    ,{ 420.93257844945873, 44.52161640172337 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(pump_0.dataBus,collectorcontrol_0.dataBus)
        annotation (Line(
        points={{ 236.71817055072083, 50.38924379735552 }    ,{ 301.9293159697926, 50.38924379735552 }    ,{ 301.9293159697926, 174.84029119001636 }    ,{ 367.1404613888643, 174.84029119001636 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(pump_0.port_b,temperaturesensor_0.port_a)
        annotation (Line(
        points={{ 236.71817055072083, 50.38924379735552 }    ,{ 242.16379227784273, 50.38924379735552 }    ,{ 242.16379227784273, 241.70999851433663 }    ,{ 247.60941400496466, 241.70999851433663 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(threewayvalve_0.y,threewayvalvecontrol_0.y)
        annotation (Line(
        points={{ 328.6925544579649, 61.92987668994206 }    ,{ 326.8994838892784, 61.92987668994206 }    ,{ 326.8994838892784, 212.1081562917843 }    ,{ 325.10641332059186, 212.1081562917843 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(threewayvalve_0.port_2,pump_0.port_a)
        annotation (Line(
        points={{ 328.6925544579649, 61.92987668994206 }    ,{ 282.7053625043429, 61.92987668994206 }    ,{ 282.7053625043429, 50.38924379735552 }    ,{ 236.71817055072083, 50.38924379735552 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(threewayvalve_0.port_3,splitvalve_0.port_3)
        annotation (Line(
        points={{ 328.6925544579649, 61.92987668994206 }    ,{ 358.514149179277, 61.92987668994206 }    ,{ 358.514149179277, 284.51938790670033 }    ,{ 388.33574390058914, 284.51938790670033 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(threewayvalvecontrol_0.u,temperaturesensor_0.T)
        annotation (Line(
        points={{ 325.10641332059186, 212.1081562917843 }    ,{ 286.3579136627783, 212.1081562917843 }    ,{ 286.3579136627783, 241.70999851433663 }    ,{ 247.60941400496466, 241.70999851433663 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(boiler_0.dataBus,collectorcontrol_1.dataBus)
        annotation (Line(
        points={{ 420.93257844945873, 44.52161640172337 }    ,{ 459.583210707812, 44.52161640172337 }    ,{ 459.583210707812, 166.498291487149 }    ,{ 498.2338429661652, 166.498291487149 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(boiler_0.port_b,threewayvalve_0.port_1)
        annotation (Line(
        points={{ 420.93257844945873, 44.52161640172337 }    ,{ 374.81256645371184, 44.52161640172337 }    ,{ 374.81256645371184, 61.92987668994206 }    ,{ 328.6925544579649, 61.92987668994206 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_2.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 527.2655079632606, 244.19848462338433 }    ,{ 510.54416371258577, 244.19848462338433 }    ,{ 510.54416371258577, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_2.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ -50.0, 0.0 }    ,{ 221.91140973095548, 0.0 }    ,{ 221.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_1.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 375.7388114843199, 442.98766899420593 }    ,{ 434.78081547311547, 442.98766899420593 }    ,{ 434.78081547311547, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_1.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 200.0, 150.0 }    ,{ 346.9114097309555, 150.0 }    ,{ 346.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_0.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 294.12439086281995, 367.84281681770915 }    ,{ 393.97360516236546, 367.84281681770915 }    ,{ 393.97360516236546, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_0.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 450.0, 150.0 }    ,{ 471.9114097309555, 150.0 }    ,{ 471.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_5.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 460.9533543002585, 259.5453870153023 }    ,{ 477.38808688108475, 259.5453870153023 }    ,{ 477.38808688108475, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_5.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ -50.0, 150.0 }    ,{ 221.91140973095548, 150.0 }    ,{ 221.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_6.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 240.40217908755426, 374.22374090031195 }    ,{ 367.1124992747326, 374.22374090031195 }    ,{ 367.1124992747326, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_8.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 200.0, 300.0 }    ,{ 346.9114097309555, 300.0 }    ,{ 346.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_6.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 450.0, 300.0 }    ,{ 471.9114097309555, 300.0 }    ,{ 471.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_7.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ -50.0, 300.0 }    ,{ 221.91140973095548, 300.0 }    ,{ 221.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_7.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 299.3463156768894, 279.2007131184074 }    ,{ 396.5845675694002, 279.2007131184074 }    ,{ 396.5845675694002, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_9.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 200.0, 450.0 }    ,{ 346.9114097309555, 450.0 }    ,{ 346.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_10.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 450.0, 450.0 }    ,{ 471.9114097309555, 450.0 }    ,{ 471.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_4.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 434.1027342054251, 414.7749220026742 }    ,{ 463.96277683366804, 414.7749220026742 }    ,{ 463.96277683366804, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_4.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ -50.0, 450.0 }    ,{ 221.91140973095548, 450.0 }    ,{ 221.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emissioncontrol_3.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 314.43886677940054, 470.8958549992571 }    ,{ 404.13084312065575, 470.8958549992571 }    ,{ 404.13084312065575, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_3.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 200.0, 600.0 }    ,{ 346.9114097309555, 600.0 }    ,{ 346.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(collectorcontrol_0.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 367.1404613888643, 174.84029119001636 }    ,{ 430.48164042538764, 174.84029119001636 }    ,{ 430.48164042538764, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(threewayvalvecontrol_0.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 325.10641332059186, 212.1081562917843 }    ,{ 409.4646163912514, 212.1081562917843 }    ,{ 409.4646163912514, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(collectorcontrol_1.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 498.2338429661652, 166.498291487149 }    ,{ 496.0283312140381, 166.498291487149 }    ,{ 496.0283312140381, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.heaPorAir,data_bus.port[1])
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 496.9114097309555, 150.0 }    ,{ 496.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.ports[1],data_bus.port_a[1])
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 496.9114097309555, 150.0 }    ,{ 496.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.heaPorAir,data_bus.port[2])
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 371.9114097309555, 450.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_10.ports[1],data_bus.port_a[2])
        annotation (Line(
        points={{ 250.0, 450.0 }    ,{ 371.9114097309555, 450.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_11.heaPorAir,data_bus.port[3])
        annotation (Line(
        points={{ 500.0, 450.0 }    ,{ 496.9114097309555, 450.0 }    ,{ 496.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_11.ports[1],data_bus.port_a[3])
        annotation (Line(
        points={{ 500.0, 450.0 }    ,{ 496.9114097309555, 450.0 }    ,{ 496.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.heaPorAir,data_bus.port[4])
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 371.9114097309555, 150.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.ports[1],data_bus.port_a[4])
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 371.9114097309555, 150.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.heaPorAir,data_bus.port[5])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 246.91140973095548, 0.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.ports[1],data_bus.port_a[5])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 246.91140973095548, 0.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.heaPorAir,data_bus.port[6])
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 371.9114097309555, 600.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_4.ports[1],data_bus.port_a[6])
        annotation (Line(
        points={{ 250.0, 600.0 }    ,{ 371.9114097309555, 600.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.heaPorAir,data_bus.port[7])
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 246.91140973095548, 450.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_5.ports[1],data_bus.port_a[7])
        annotation (Line(
        points={{ 0.0, 450.0 }    ,{ 246.91140973095548, 450.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.heaPorAir,data_bus.port[8])
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ 246.91140973095548, 150.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_6.ports[1],data_bus.port_a[8])
        annotation (Line(
        points={{ 0.0, 150.0 }    ,{ 246.91140973095548, 150.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.heaPorAir,data_bus.port[9])
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 496.9114097309555, 300.0 }    ,{ 496.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_7.ports[1],data_bus.port_a[9])
        annotation (Line(
        points={{ 500.0, 300.0 }    ,{ 496.9114097309555, 300.0 }    ,{ 496.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.heaPorAir,data_bus.port[10])
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 246.91140973095548, 300.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_8.ports[1],data_bus.port_a[10])
        annotation (Line(
        points={{ 0.0, 300.0 }    ,{ 246.91140973095548, 300.0 }    ,{ 246.91140973095548, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.heaPorAir,data_bus.port[11])
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 371.9114097309555, 300.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_9.ports[1],data_bus.port_a[11])
        annotation (Line(
        points={{ 250.0, 300.0 }    ,{ 371.9114097309555, 300.0 }    ,{ 371.9114097309555, 416.29029861833305 }    ,{ 493.82281946191097, 416.29029861833305 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;annotation (Diagram(coordinateSystem(extent={{-50,-50},{1000,1000}})), Icon(
        coordinateSystem(extent={{-50,-50},{1000,1000}})));
end building;


end house_model;