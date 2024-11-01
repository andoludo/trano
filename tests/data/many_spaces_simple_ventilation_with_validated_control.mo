within ;
package many_spaces_simple_ventilation_with_validated_control

package Trano
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
              package                                                                   Medium = Medium) annotation (
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
        Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[2] TRoo annotation (
                Placement(transformation(origin={-544,-226},    extent = {{480, 216}, {500, 236}})));
                                                                                                             Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[2] port annotation (
                Placement(transformation(extent={{-112,-10},{-92,10}}),      iconTransformation(extent = {{-110, -10}, {-90, 10}})));
        BaseClasses.DataBus controlBus annotation (Placement(transformation(
                extent={{80,-20},{120,20}}), iconTransformation(extent={{90,-10},
                  {110,10}})));
                                     Buildings.Fluid.Sensors.PPM[2]                             TRoo1(redeclare
              package
              Medium = Medium)                                                                     annotation (
          Placement(transformation(origin={-542,-268},    extent = {{480, 216}, {500, 236}})));
                                                                                                 Modelica.Fluid.Interfaces.FluidPort_a[2] port_a(redeclare
              package                                                                                                                                              Medium =
        Medium)
            "Fluid connector a (positive design flow direction is from port_a to port_b)"
            annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
                iconTransformation(origin={-2,-42}, extent={{-110,-9},{-90,9}})));
          Modelica.Blocks.Sources.Constant const(k= 0.2)
            annotation (Placement(transformation(extent={{-32,36},{-12,56}})));
      equation
              connect(port[1],TRoo[1]. port) annotation (
                Line(points={{-102,0},{-64,0}},      color = {191, 0, 0}));
        connect(TRoo[1].T, controlBus.temperature_space_1)
            annotation (Line(points={{-43,0},{100,0}}, color={0,0,127}), Text(
            string="%second",
            index=1,
            extent={{6,3},{6,3}},
            horizontalAlignment=TextAlignment.Left));

          connect(port_a[1], TRoo1[1].port) annotation (Line(points={{-99,-42},
                  {-68,-42},{-68,
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

          connect(port_a[2], TRoo1[2].port) annotation (Line(points={{-99,-42},
                  {-68,-42},{-68,
          -56},{-52,-56},{-52,-52}}, color={0,127,255}));  connect(TRoo1[2].ppm, controlBus.ppm_space_2)
            annotation (Line(points={{-41,-42},{74,
          -42},{74,0},{100,0}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
          connect(const.y, controlBus.yVAVvav_in_1) annotation (Line(points={{
                  -11,46},{42.5,46},{42.5,2},{96,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{6,3},{6,3}},
              horizontalAlignment=TextAlignment.Left));
          connect(const.y, controlBus.yVAVvav_in_2) annotation (Line(points={{
                  -11,46},{43.5,46},{43.5,-2},{98,-2}}, color={0,0,127}), Text(
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
            connect(mulAHUCon.y1SupFan, dataBus.y1SupFan) annotation (Line(points=
                   {{30,30},{38,30},{38,-22},{-76,-22},{-76,0},{-100,0}}, color={
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
  model Simple

  extends Buildings.Fluid.Interfaces.PartialTwoPort;
  Buildings.Fluid.Sources.Boundary_pT bou(use_T_in = true, nPorts = 2, redeclare
              final package                                                                    Medium = Medium)  annotation (
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
          m_flow_nominal=10*100*1.2/3600) "Supply fan"
          annotation (Placement(transformation(extent={{4,6},{24,26}})));
        Buildings.Fluid.Movers.FlowControlled_dp
                                       fanRet(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          use_inputFilter=false,
          inputType=Buildings.Fluid.Types.InputType.Constant,
          nominalValuesDefineDefaultPressureCurve=true,
          redeclare package Medium = Medium,
          dp_nominal=200,
          m_flow_nominal=10*100*1.2/3600) "Return fan"
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
              package                                                                    Medium = Medium)
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

    model AhuWithEconomizer
    replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
    "Medium model" annotation (choicesAllMatching=true);

      constant Integer numZon(min=2)=2 "Total number of served VAV boxes";

          parameter Modelica.Units.SI.Volume VRoo[numZon]
            "Room volume per zone";
          parameter Modelica.Units.SI.Area AFlo[numZon] "Floor area per zone";

          final parameter Modelica.Units.SI.Area ATot=sum(AFlo)
    "Total floor area for all zone";

          constant Real conv=1.2/3600
            "Conversion factor for nominal mass flow rate";


          parameter Modelica.Units.SI.HeatFlowRate QHeaAHU_flow_nominal(min=0)
             = mHeaAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir
            *(THeaAirSup_nominal - THeaAirMix_nominal)
    "Nominal heating heat flow rate of air handler unit coil";

          parameter Modelica.Units.SI.HeatFlowRate QCooAHU_flow_nominal(max=0)
             = 1.3*mCooAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir
            *(TCooAirSup_nominal - TCooAirMix_nominal)
    "Nominal total cooling heat flow rate of air handler unit coil (negative number)";

          parameter Modelica.Units.SI.MassFlowRate mCooVAV_flow_nominal[numZon]
    "Design mass flow rate per zone for cooling"
    annotation (Dialog(group="Nominal mass flow rate"));

          parameter Modelica.Units.SI.MassFlowRate mHeaVAV_flow_nominal[numZon]
            =0.3*mCooVAV_flow_nominal
    "Design mass flow rate per zone for heating"
    annotation (Dialog(group="Nominal mass flow rate"));

          parameter Modelica.Units.SI.MassFlowRate mAir_flow_nominal=
              mCooAir_flow_nominal
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

          parameter Modelica.Units.SI.VolumeFlowRate VZonOA_flow_nominal[numZon]
            =                                                            (
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
          parameter Modelica.Units.SI.PressureDifference dpBuiStaSet(min=0) =
            12
    "Building static pressure";
          parameter Real yFanMin=0.1 "Minimum fan speed";


          parameter Modelica.Units.SI.Temperature TCooAirMix_nominal(
              displayUnit="degC") = 303.15
    "Mixed air temperature during cooling nominal conditions (used to size cooling coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
          parameter Modelica.Units.SI.Temperature TCooAirSup_nominal(
              displayUnit="degC") = 285.15
    "Supply air temperature during cooling nominal conditions (used to size cooling coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
          parameter Modelica.Units.SI.MassFraction wCooAirMix_nominal=0.017
    "Humidity ratio of mixed air at a nominal conditions used to size cooling coil (in kg/kg dry total)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
          parameter Modelica.Units.SI.Temperature TCooWatInl_nominal(
              displayUnit="degC") = 279.15
    "Cooling coil nominal inlet water temperature"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));


          parameter Modelica.Units.SI.Temperature THeaAirMix_nominal(
              displayUnit="degC") = 277.15
    "Mixed air temperature during heating nominal conditions (used to size heating coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
          parameter Modelica.Units.SI.Temperature THeaAirSup_nominal(
              displayUnit="degC") = 285.15
    "Supply air temperature during heating nominal conditions (used to size heating coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
          parameter Modelica.Units.SI.Temperature THeaWatInl_nominal(
              displayUnit="degC")
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
          Modelica.Fluid.Interfaces.FluidPorts_b ports[2](redeclare each
              package Medium =
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
          Buildings.Fluid.Sensors.RelativePressure dpDisSupFan(redeclare
              package Medium =
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
          Buildings.Controls.OBC.CDL.Reals.Sources.Constant uOutAirFra_max(
              final k=0.5)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{498,-122},{518,-102}})));
          Buildings.Controls.OBC.CDL.Integers.Sources.Constant maxSupResReq(
              final k=6)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-74,-268},{-54,-248}})));

        protected
          parameter Modelica.Fluid.Types.PortFlowDirection flowDirection=
              Modelica.Fluid.Types.PortFlowDirection.Bidirectional
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
          damRet.port_b,splRetOut. port_3) annotation (Line(points={{52,-16},{
                  52,
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
          damExh.port_b, ports[1]) annotation (Line(points={{-36,-4},{-84,-4},{
                  -84,
          6},{-100,6}}, color={0,127,255}));
          connect(
          VOut1.port_a, ports[2]) annotation (Line(points={{-68,-70},{-84,-70},
                  {
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
          VOut1.port_a, dpDisSupFan.port_b) annotation (Line(points={{-68,-70},
                  {
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
          ducPreResReq.y, dataBus.uZonPreResReq) annotation (Line(points={{-42,
                  -208},
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
          u1SupFan.y, dataBus.u1SupFan) annotation (Line(points={{358,50},{366,
                  50},
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
          VSumZonPri_flow.y, dataBus.VSumZonPri_flow) annotation (Line(points={
                  {
          466,-102},{200,-102},{200,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
          connect(
          uOutAirFra_max.y, dataBus.uOutAirFra_max) annotation (Line(points={{
                  520,
          -112},{226,-112},{226,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
          connect(
          maxSupResReq.y, dataBus.uZonTemResReq) annotation (Line(points={{-52,
                  -258},
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
          parameter Modelica.Units.SI.Temperature THeaWatInl_nominal(start=55
                 + 273.15, displayUnit="degC")
            "Reheat coil nominal inlet water temperature";
          parameter Modelica.Units.SI.Temperature THeaWatOut_nominal(start=
    THeaWatInl_nominal - 10, displayUnit="degC")
            "Reheat coil nominal outlet water temperature";
          parameter Modelica.Units.SI.Temperature THeaAirInl_nominal(start=12
                 + 273.15, displayUnit="degC")
            "Inlet air nominal temperature into VAV box during heating";
          parameter Modelica.Units.SI.Temperature THeaAirDis_nominal(start=28
                 + 273.15, displayUnit="degC")
            "Discharge air temperature from VAV box during heating";
          parameter Modelica.Units.SI.HeatFlowRate QHea_flow_nominal=
              mHeaAir_flow_nominal*cpAir*(THeaAirDis_nominal -
              THeaAirInl_nominal) "Nominal heating heat flow rate";
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
            allowFlowReversal=allowFlowReversal)
            "Supply air temperature sensor" annotation (Placement(
                transformation(
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
      senTem.port_b, senVolFlo.port_a) annotation (Line(points={{0,50},{0,70},{
                  -6.66134e-16,70}},     color={0,127,255}));
          connect(
      senVolFlo.port_b, port_bAir) annotation (Line(points={{4.44089e-16,90},{0,
                  90},{0,100}},                  color={0,127,255}));
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

        model VAVBoxVav_in_1

          extends
          many_spaces_simple_ventilation_with_validated_control.Trano.Fluid.Ventilation.PartialVAVBox;
          Controls.BaseClasses.DataBus dataBus annotation (Placement(
                transformation(
          extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-368,-4},{-348,
            16}})));
        equation
          connect(
          senVolFlo.V_flow, dataBus.VSup_flowvav_in_1) annotation (Line(points=
                  {{11,80},
          {14,80},{14,114},{-74,114},{-74,2},{-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          connect(senTem.T, dataBus.TSupvav_in_1) annotation (Line(points={{11,
                  40},{16,40},{16,24},
          {-70,24},{-70,2},{-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          connect(vav.y_actual, dataBus.y_actualvav_in_1) annotation (Line(
                points={{-7,15},{-68,
          15},{-68,2},{-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          connect(vav.y, dataBus.yVAVvav_in_1) annotation (Line(points={{-12,10},
                  {-66,10},{-66,2},
          {-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
        end VAVBoxVav_in_1;

        model VAVBoxVav_in_2

          extends
          many_spaces_simple_ventilation_with_validated_control.Trano.Fluid.Ventilation.PartialVAVBox;
          Controls.BaseClasses.DataBus dataBus annotation (Placement(
                transformation(
          extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-368,-4},{-348,
            16}})));
        equation
          connect(
          senVolFlo.V_flow, dataBus.VSup_flowvav_in_2) annotation (Line(points=
                  {{11,80},
          {14,80},{14,114},{-74,114},{-74,2},{-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          connect(senTem.T, dataBus.TSupvav_in_2) annotation (Line(points={{11,
                  40},{16,40},{16,24},
          {-70,24},{-70,2},{-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          connect(vav.y_actual, dataBus.y_actualvav_in_2) annotation (Line(
                points={{-7,15},{-68,
          15},{-68,2},{-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          connect(vav.y, dataBus.yVAVvav_in_2) annotation (Line(points={{-12,10},
                  {-66,10},{-66,2},
          {-100,2}}, color={0,0,127}), Text(
              string="%second",
              index=1,
              extent={{-6,3},{-6,3}},
              horizontalAlignment=TextAlignment.Right));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
        end VAVBoxVav_in_2;

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

model building
            parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic double_glazing(
    final glass={
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003,
        k=1.0,
        tauSol={ 0.646},
        rhoSol_a={ 0.062},
        rhoSol_b={ 0.063},
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84),
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003,
        k=1.0,
        tauSol={ 0.646},
        rhoSol_a={ 0.062},
        rhoSol_b={ 0.063},
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84)},
    final gas={
            Buildings.HeatTransfer.Data.Gases.Air(x=0.0127)},
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
        d=540.0)},
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));
                                                                         parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
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
        d=1920.0)},
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));


package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
    package MediumW = Buildings.Media.Water "Medium model";

    Buildings.ThermalZones.Detailed.MixedAir space_1(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,nPorts=3,                  nConExt=1,
                datConExt(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                azi={ 45.0}),
                nSurBou=1,
                surBou(
                A={ 10.0},
                til={Buildings.Types.Tilt.Wall}),
                nConBou=1,
                datConBou(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Floor},
                azi={ 90.0}),
                nConExtWin=1,
                datConExtWin(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                glaSys={ double_glazing},
                wWin={ 1.0},
                hWin={ 1.0}),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 0, 50},
    extent = {{-10, -10}, {10, 10}})));
        many_spaces_simple_ventilation_with_validated_control.Trano.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50},
    extent = {{-10, -10}, {10, 10}})));
      many_spaces_simple_ventilation_with_validated_control.Trano.Fluid.Ventilation.VAVBoxVav_in_1
     vav_in_1(
    redeclare package MediumA = Medium,
      mCooAir_flow_nominal=100*1.2/3600,
      mHeaAir_flow_nominal=100*1.2/3600,
    VRoo=100,
      THeaWatInl_nominal=353.15,
      THeaWatOut_nominal=343.15,
      THeaAirInl_nominal=295.15,
      THeaAirDis_nominal=295.15,
    QHea_flow_nominal=2000,
    allowFlowReversal=false) annotation (
    Placement(transformation(origin = { -153.04481633429498, 127.54788990417522},
    extent = {{-10, -10}, {10, 10}})));
    Buildings.ThermalZones.Detailed.MixedAir space_2(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,nPorts=3,                  nConExt=1,
                datConExt(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                azi={ 45.0}),
                nSurBou=1,
                surBou(
                A={ 10.0},
                til={Buildings.Types.Tilt.Wall}),
                nConBou=1,
                datConBou(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Floor},
                azi={ 90.0}),
                nConExtWin=1,
                datConExtWin(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                glaSys={ double_glazing},
                wWin={ 1.0},
                hWin={ 1.0}),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) annotation (
    Placement(transformation(origin = { 200, 50},
    extent = {{-10, -10}, {10, 10}})));
        many_spaces_simple_ventilation_with_validated_control.Trano.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50},
    extent = {{-10, -10}, {10, 10}})));
      many_spaces_simple_ventilation_with_validated_control.Trano.Fluid.Ventilation.VAVBoxVav_in_2
     vav_in_2(
    redeclare package MediumA = Medium,
      mCooAir_flow_nominal=100*1.2/3600,
      mHeaAir_flow_nominal=100*1.2/3600,
    VRoo=1000,
      THeaWatInl_nominal=353.15,
      THeaWatOut_nominal=343.15,
      THeaAirInl_nominal=295.15,
      THeaAirDis_nominal=295.15,
    QHea_flow_nominal=2000,
    allowFlowReversal=false) annotation (
    Placement(transformation(origin={214,-152},
    extent = {{-10, -10}, {10, 10}})));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_1_space_2(A=
            10, layers=
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { 100.0, 50},
    extent = {{-10, -10}, {10, 10}})));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
            weather(filNam=
    Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
 annotation (
    Placement(transformation(origin = { -100, 200},
    extent = {{-10, -10}, {10, 10}})));
    many_spaces_simple_ventilation_with_validated_control.Trano.Fluid.Ventilation.SimpleHVACBuildings
    ahu(
     redeclare package Medium = Medium) annotation (
    Placement(transformation(origin = { 100, -50},
    extent = {{-10, -10}, {10, 10}})));
        many_spaces_simple_ventilation_with_validated_control.Trano.Controls.SpaceControls.DataServer
    data_bus( redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin={19.8297,181.103},
    extent = {{-10, -10}, {10, 10}})));


equation
            connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points={{-10.8,54},{-25,54},{-25,50},{-39,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
annotation (Line(
points={{-1.9,43},{50,43},{50,50},{90,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.weaBus,weather.weaBus)
annotation (Line(
points={{8.95,58.95},{-50,58.95},{-50,200},{-90,200}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[1],ahu.port_a)
annotation (Line(
points={{-7.5,43.6667},{50,43.6667},{50,-56.6667},{110,-56.6667}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.heaPorAir,data_bus.port[1])
annotation (Line(
points={{-0.5,50},{-0.5,118},{9.8297,118},{9.8297,180.603}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[2],data_bus.port_a[1])
annotation (Line(
points={{-7.5,45},{14.9149,45},{14.9149,176.903},{9.1297,176.903}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in_1.port_bAir,space_1.ports[3])
annotation (Line(
points={{-153.045,137.548},{-76.5224,137.548},{-76.5224,46.3333},{-7.5,46.3333}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.qGai_flow,occupancy_1.y)
annotation (Line(
points={{189.2,54},{175,54},{175,50},{161,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.surf_surBou[1],internal_space_1_space_2.port_b)
annotation (Line(
points={{198.1,43},{150,43},{150,50},{110,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.weaBus,weather.weaBus)
annotation (Line(
points={{208.95,58.95},{50,58.95},{50,200},{-90,200}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.ports[1],ahu.port_a)
annotation (Line(
points={{192.5,43.6667},{150,43.6667},{150,-56.6667},{110,-56.6667}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.heaPorAir,data_bus.port[2])
annotation (Line(
points={{199.5,50},{118,50},{118,62},{112,62},{112,66},{9.8297,66},{9.8297,181.603}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.ports[2],data_bus.port_a[2])
annotation (Line(
points={{192.5,45},{118,45},{118,52},{116,52},{116,64},{14,64},{14,118},{10.1297,
          118},{10.1297,176.903}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in_2.port_bAir,space_2.ports[3])
annotation (Line(
points={{214,-142},{118,-142},{118,46.3333},{192.5,46.3333}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,vav_in_1.port_aAir)
annotation (Line(
points={{110,-43.3333},{-26.5224,-43.3333},{-26.5224,117.548},{-153.045,117.548}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,vav_in_2.port_aAir)
annotation (Line(
points={{110,-43.3333},{48,-43.3333},{48,-120},{114,-120},{114,-162},{214,-162}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));
  connect(vav_in_1.dataBus, data_bus.controlBus) annotation (Line(
      points={{-188.845,128.148},{-120,128.148},{-120,118},{2,118},{2,116},{
            29.8297,116},{29.8297,181.103}},
      color={255,204,51},
      thickness=0.5));
  connect(vav_in_2.dataBus, data_bus.controlBus) annotation (Line(
      points={{178.2,-151.4},{118,-151.4},{118,181.103},{29.8297,181.103}},
      color={255,204,51},
      thickness=0.5));
     annotation (
    Placement(transformation(origin = { -182.79274783988745, 63.5066046496308},
    extent = {{-10, -10}, {10, 10}})),
    Placement(transformation(origin = { 191.00032154274217, 11.383774063399553},
    extent = {{-10, -10}, {10, 10}})),
    Placement(transformation(origin = { -31.242516196177128, 183.19984191224242},
    extent = {{-10, -10}, {10, 10}})),
    Placement(transformation(origin = { 140.93747237617055, -110.27153306887236},
    extent = {{-10, -10}, {10, 10}})),
    Placement(transformation(origin = { -182.31178337442503, -3.5371622104869593},
    extent = {{-10, -10}, {10, 10}})),
    Placement(transformation(origin = { -102.50500685941628, -169.8101506092406},
    extent = {{-10, -10}, {10, 10}})));
end building;


  annotation (uses(Modelica(version="4.0.0")));
end many_spaces_simple_ventilation_with_validated_control;
