package buildings_simple_hydronic_three_zones

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
            package Medium =                                                                     Medium) annotation (
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
  annotation (
      Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                                                                                                                                                        FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(origin = {0, 35.1488}, fillColor = {255, 255, 255}, extent = {{-30, -20.1488}, {30, 20.1488}}), Rectangle(origin = {0, -34.8512}, fillColor = {255, 255, 255}, extent = {{-30, -20.1488}, {30, 20.1488}}), Line(origin = {-51.25, 0}, points = {{21.25, -35}, {-13.75, -35}, {-13.75, 35}, {6.25, 35}}), Polygon(origin = {-40, 35}, pattern = LinePattern.None,
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{10, 0}, {-5, 5}, {-5, -5}, {10, 0}}), Line(origin = {51.25, 0}, points = {{-21.25, 35}, {13.75, 35}, {13.75, -35}, {-6.25, -35}}), Polygon(origin = {40, -35}, pattern = LinePattern.None,
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-10, 0}, {5, 5}, {5, -5}, {-10, 0}})}));
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
                -32,0},{-32,-25},{-38,-25}}, color={0,127,255}));
        connect(heaPum.port_a1, sin1.ports[2]) annotation (Line(points={{-8,0},{
                -32,0},{-32,-26},{-38,-26},{-38,-23}}, color={0,127,255}));
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
        Buildings.Fluid.Sensors.VolumeFlowRate VOut1(redeclare package Medium =
          MediumA, m_flow_nominal=mAir_flow_nominal)
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
      linearized=true)
    "Flow splitter"
    annotation (Placement(transformation(
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
        Buildings.Fluid.Sensors.VolumeFlowRate VOut1(redeclare package Medium =
              MediumA, m_flow_nominal=mAir_flow_nominal)
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
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { -84.13039464186795, 100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             three_way_valve(
    redeclare package Medium = MediumW,
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
    Placement(transformation(origin = { -61.10989877964849, 18.495766229453764 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.
    ThreeWayValveControlThree_way_valve_control
    three_way_valve_control annotation (
    Placement(transformation(origin = { -100.0, 53.97642370911507 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor_2(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { 84.1371090930455, 99.99169848912504 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             three_way_valve_2(
    redeclare package Medium = MediumW,
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
    Placement(transformation(origin = { 61.02261091434042, 18.558027561016104 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.
    ThreeWayValveControlThree_way_valve_control_2
    three_way_valve_control_2 annotation (
    Placement(transformation(origin = { 100.0, 53.8435995351154 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.FixedResistances.Junction split_valve (
        dp_nominal={10000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.008*{1,-1,-1},
    linearized=false
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { -90.25229550299632, -56.641208699983395 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.FixedResistances.Junction split_valve_2 (
        dp_nominal={10000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.008*{1,-1,-1},
    linearized=false
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { 90.09618451311837, -56.624605678233436 },
    extent = {{10, -10}, {-10, 10}}
)));
      buildings_simple_hydronic_three_zones.Components.BaseClasses.PumpPump
     pump(
         dp_nominal=10000.0,
    m_flow_nominal=0.008
,
    redeclare package Medium = MediumW

    ) annotation (
    Placement(transformation(origin = { -0.01846474073823856, -5.711439481985707 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.CollectorControlPump_control
    pump_control annotation (
    Placement(transformation(origin = { 0.015107515149509254, -100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
equation        
        connect(split_valve.port_1,port_a1[1])
        annotation (Line(
        points={{ -90.25229550299632, -56.641208699983395 }    ,{ -45.12614775149816, -56.641208699983395 }    ,{ -45.12614775149816, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve.port_1,port_a1[2])
        annotation (Line(
        points={{ -90.25229550299632, -56.641208699983395 }    ,{ -45.12614775149816, -56.641208699983395 }    ,{ -45.12614775149816, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve_2.port_1,port_a1[3])
        annotation (Line(
        points={{ 90.09618451311837, -56.624605678233436 }    ,{ 45.04809225655919, -56.624605678233436 }    ,{ 45.04809225655919, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor.port_b,port_b1[1])
        annotation (Line(
        points={{ -84.13039464186795, 100.0 }    ,{ -42.065197320933976, 100.0 }    ,{ -42.065197320933976, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor.port_b,port_b1[2])
        annotation (Line(
        points={{ -84.13039464186795, 100.0 }    ,{ -42.065197320933976, 100.0 }    ,{ -42.065197320933976, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor.T,three_way_valve_control.u)
        annotation (Line(
        points={{ -84.13039464186795, 100.0 }    ,{ -92.06519732093398, 100.0 }    ,{ -92.06519732093398, 53.97642370911507 }    ,{ -100.0, 53.97642370911507 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve.y,three_way_valve_control.y)
        annotation (Line(
        points={{ -61.10989877964849, 18.495766229453764 }    ,{ -80.55494938982424, 18.495766229453764 }    ,{ -80.55494938982424, 53.97642370911507 }    ,{ -100.0, 53.97642370911507 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve.port_2,temperature_sensor.port_a)
        annotation (Line(
        points={{ -61.10989877964849, 18.495766229453764 }    ,{ -72.62014671075822, 18.495766229453764 }    ,{ -72.62014671075822, 100.0 }    ,{ -84.13039464186795, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve.port_3,split_valve.port_3)
        annotation (Line(
        points={{ -61.10989877964849, 18.495766229453764 }    ,{ -75.6810971413224, 18.495766229453764 }    ,{ -75.6810971413224, -56.641208699983395 }    ,{ -90.25229550299632, -56.641208699983395 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor_2.port_b,port_b1[3])
        annotation (Line(
        points={{ 84.1371090930455, 99.99169848912504 }    ,{ 42.06855454652275, 99.99169848912504 }    ,{ 42.06855454652275, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(temperature_sensor_2.T,three_way_valve_control_2.u)
        annotation (Line(
        points={{ 84.1371090930455, 99.99169848912504 }    ,{ 92.06855454652275, 99.99169848912504 }    ,{ 92.06855454652275, 53.8435995351154 }    ,{ 100.0, 53.8435995351154 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_2.y,three_way_valve_control_2.y)
        annotation (Line(
        points={{ 61.02261091434042, 18.558027561016104 }    ,{ 80.51130545717021, 18.558027561016104 }    ,{ 80.51130545717021, 53.8435995351154 }    ,{ 100.0, 53.8435995351154 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_2.port_2,temperature_sensor_2.port_a)
        annotation (Line(
        points={{ 61.02261091434042, 18.558027561016104 }    ,{ 72.57986000369296, 18.558027561016104 }    ,{ 72.57986000369296, 99.99169848912504 }    ,{ 84.1371090930455, 99.99169848912504 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_2.port_3,split_valve_2.port_3)
        annotation (Line(
        points={{ 61.02261091434042, 18.558027561016104 }    ,{ 75.5593977137294, 18.558027561016104 }    ,{ 75.5593977137294, -56.624605678233436 }    ,{ 90.09618451311837, -56.624605678233436 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(split_valve.port_2,port_b)
        annotation (Line(
        points={{ -90.25229550299632, -56.641208699983395 }    ,{ -45.12614775149816, -56.641208699983395 }    ,{ -45.12614775149816, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(split_valve_2.port_2,port_b)
        annotation (Line(
        points={{ 90.09618451311837, -56.624605678233436 }    ,{ 45.04809225655919, -56.624605678233436 }    ,{ 45.04809225655919, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(pump.port_a,port_a)
        annotation (Line(
        points={{ -0.01846474073823856, -5.711439481985707 }    ,{ -0.00923237036911928, -5.711439481985707 }    ,{ -0.00923237036911928, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(pump.dataBus,pump_control.dataBus)
        annotation (Line(
        points={{ -0.01846474073823856, -5.711439481985707 }    ,{ -0.0016786127943646534, -5.711439481985707 }    ,{ -0.0016786127943646534, -100.0 }    ,{ 0.015107515149509254, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pump.port_b,three_way_valve.port_1)
        annotation (Line(
        points={{ -0.01846474073823856, -5.711439481985707 }    ,{ -30.564181760193364, -5.711439481985707 }    ,{ -30.564181760193364, 18.495766229453764 }    ,{ -61.10989877964849, 18.495766229453764 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(pump.port_b,three_way_valve_2.port_1)
        annotation (Line(
        points={{ -0.01846474073823856, -5.711439481985707 }    ,{ 30.50207308680109, -5.711439481985707 }    ,{ 30.50207308680109, 18.558027561016104 }    ,{ 61.02261091434042, 18.558027561016104 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(three_way_valve_control.dataBus,dataBus)
            ;        
        connect(three_way_valve_control_2.dataBus,dataBus)
            ;        
        connect(pump_control.dataBus,dataBus)
            ;end distribution;
    model emission


// Define Medium
extends Trano.BaseClasses.Containers.emission ;
package MediumW = Buildings.Media.Water;
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
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 emission(
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
    Placement(transformation(origin = { 78.10871435076649, -100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve(
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
    Placement(transformation(origin = { 27.841397804125265, -100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.EmissionControlEmission_valve_control
    emission_valve_control(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -27.208945406709063, -100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 emission_2(
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
    Placement(transformation(origin = { 16.691683672608917, 0.0 },
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
    Placement(transformation(origin = { -39.00291771372676, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.EmissionControlEmission_valve_control_2
    emission_valve_control_2(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -100.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 emission_3(
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
    Placement(transformation(origin = { 100.0, 100.0 },
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
    Placement(transformation(origin = { 44.31214477257009, 100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.EmissionControlEmission_valve_control_3
    emission_valve_control_3(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { -16.684937513703147, 100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
equation        
        connect(emission.heatPortRad,heatPortRad[1])
        annotation (Line(
        points={{ 78.10871435076649, -100.0 }    ,{ 39.054357175383245, -100.0 }    ,{ 39.054357175383245, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission.heatPortCon,heatPortCon[1])
        annotation (Line(
        points={{ 78.10871435076649, -100.0 }    ,{ 39.054357175383245, -100.0 }    ,{ 39.054357175383245, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission.port_b,port_b[1])
        annotation (Line(
        points={{ 78.10871435076649, -100.0 }    ,{ 39.054357175383245, -100.0 }    ,{ 39.054357175383245, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve.y,emission_valve_control.y)
        annotation (Line(
        points={{ 27.841397804125265, -100.0 }    ,{ 0.316226198708101, -100.0 }    ,{ 0.316226198708101, -100.0 }    ,{ -27.208945406709063, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve.port_b,emission.port_a)
        annotation (Line(
        points={{ 27.841397804125265, -100.0 }    ,{ 52.97505607744588, -100.0 }    ,{ 52.97505607744588, -100.0 }    ,{ 78.10871435076649, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(emission_2.heatPortRad,heatPortRad[2])
        annotation (Line(
        points={{ 16.691683672608917, 0.0 }    ,{ 8.345841836304459, 0.0 }    ,{ 8.345841836304459, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission_2.heatPortCon,heatPortCon[2])
        annotation (Line(
        points={{ 16.691683672608917, 0.0 }    ,{ 8.345841836304459, 0.0 }    ,{ 8.345841836304459, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission_2.port_b,port_b[2])
        annotation (Line(
        points={{ 16.691683672608917, 0.0 }    ,{ 8.345841836304459, 0.0 }    ,{ 8.345841836304459, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_2.y,emission_valve_control_2.y)
        annotation (Line(
        points={{ -39.00291771372676, 0.0 }    ,{ -69.50145885686338, 0.0 }    ,{ -69.50145885686338, 0.0 }    ,{ -100.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_2.port_b,emission_2.port_a)
        annotation (Line(
        points={{ -39.00291771372676, 0.0 }    ,{ -11.155617020558921, 0.0 }    ,{ -11.155617020558921, 0.0 }    ,{ 16.691683672608917, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(emission_3.heatPortRad,heatPortRad[3])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission_3.heatPortCon,heatPortCon[3])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission_3.port_b,port_b[3])
        annotation (Line(
        points={{ 100.0, 100.0 }    ,{ 50.0, 100.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_3.y,emission_valve_control_3.y)
        annotation (Line(
        points={{ 44.31214477257009, 100.0 }    ,{ 13.813603629433473, 100.0 }    ,{ 13.813603629433473, 100.0 }    ,{ -16.684937513703147, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve_3.port_b,emission_3.port_a)
        annotation (Line(
        points={{ 44.31214477257009, 100.0 }    ,{ 72.15607238628505, 100.0 }    ,{ 72.15607238628505, 100.0 }    ,{ 100.0, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(valve.port_a,port_a[1])
        annotation (Line(
        points={{ 27.841397804125265, -100.0 }    ,{ 13.920698902062632, -100.0 }    ,{ 13.920698902062632, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_2.port_a,port_a[2])
        annotation (Line(
        points={{ -39.00291771372676, 0.0 }    ,{ -19.50145885686338, 0.0 }    ,{ -19.50145885686338, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(valve_3.port_a,port_a[3])
        annotation (Line(
        points={{ 44.31214477257009, 100.0 }    ,{ 22.156072386285047, 100.0 }    ,{ 22.156072386285047, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(emission_valve_control.dataBus,dataBus)
            ;        
        connect(emission_valve_control_2.dataBus,dataBus)
            ;        
        connect(emission_valve_control_3.dataBus,dataBus)
            ;end emission;
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
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortCon
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortCon1
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{-4,98},{6,108}} )
  );

Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortRad

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
Modelica.Fluid.Interfaces.FluidPorts_b[3] ports_b(
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
,nPorts = 3,                    nConExt=3,
                    datConExt(
                    layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    azi={ 135.0, 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 10.0, 10.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
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
    Placement(transformation(origin = { 49.1192183637327, 26.53276955602537 },
    extent = {{10, -10}, {-10, 10}}
)));
    
    
    
    
    
    
        buildings_simple_hydronic_three_zones.Components.BaseClasses.OccupancyOccupancy_0
    occupancy_0(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 85.7050961332285, -3.31142568250759 },
    extent = {{10, -10}, {-10, 10}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_2(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=3,
                    datConExt(
                    layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    azi={ 135.0, 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 10.0, 10.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
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
    Placement(transformation(origin = { -51.737000997239726, 53.12069124000368 },
    extent = {{10, -10}, {-10, 10}}
)));
    
    
    
    
    
    
        buildings_simple_hydronic_three_zones.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -43.144175450658196, 100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_3(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=3,
                    datConExt(
                    layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    azi={ 135.0, 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 10.0, 10.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
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
    Placement(transformation(origin = { -24.070689434137364, -48.51778656126482 },
    extent = {{10, -10}, {-10, 10}}
)));
    
    
    
    
    
    
        buildings_simple_hydronic_three_zones.Components.BaseClasses.OccupancyOccupancy_2
    occupancy_2(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -59.40633037338062, -39.24533504917732 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_1_space_2(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -125.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_1_space_3(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -250.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_2_space_3(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -125.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather_5(filNam=Modelica.Utilities.Files.loadResource
        ("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
     annotation (
    Placement(transformation(origin = { -8.884055849962508, 10.35711002849527 },
    extent = {{10, -10}, {-10, 10}}
)));
equation        
        connect(space_1.heaPorRad,heatPortRad[1])
        annotation (Line(
        points={{ 49.1192183637327, 26.53276955602537 }    ,{ 24.55960918186635, 26.53276955602537 }    ,{ 24.55960918186635, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_1.heaPorAir,heatPortCon[1])
        annotation (Line(
        points={{ 49.1192183637327, 26.53276955602537 }    ,{ 24.55960918186635, 26.53276955602537 }    ,{ 24.55960918186635, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_1.qGai_flow,occupancy_0.y)
        annotation (Line(
        points={{ 49.1192183637327, 26.53276955602537 }    ,{ 67.4121572484806, 26.53276955602537 }    ,{ 67.4121572484806, -3.31142568250759 }    ,{ 85.7050961332285, -3.31142568250759 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
        annotation (Line(
        points={{ 49.1192183637327, 26.53276955602537 }    ,{ -37.94039081813365, 26.53276955602537 }    ,{ -37.94039081813365, 75.0 }    ,{ -125.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.surf_surBou[2],internal_space_1_space_3.port_a)
        annotation (Line(
        points={{ 49.1192183637327, 26.53276955602537 }    ,{ -100.44039081813364, 26.53276955602537 }    ,{ -100.44039081813366, 75.0 }    ,{ -250.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_1.weaBus,weather_5.weaBus)
        annotation (Line(
        points={{ 49.1192183637327, 26.53276955602537 }    ,{ 20.117581256885096, 26.53276955602537 }    ,{ 20.117581256885096, 10.35711002849527 }    ,{ -8.884055849962508, 10.35711002849527 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.heaPorRad,heatPortRad[2])
        annotation (Line(
        points={{ -51.737000997239726, 53.12069124000368 }    ,{ -25.868500498619863, 53.12069124000368 }    ,{ -25.868500498619863, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_2.heaPorAir,heatPortCon[2])
        annotation (Line(
        points={{ -51.737000997239726, 53.12069124000368 }    ,{ -25.868500498619863, 53.12069124000368 }    ,{ -25.868500498619863, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_2.qGai_flow,occupancy_1.y)
        annotation (Line(
        points={{ -51.737000997239726, 53.12069124000368 }    ,{ -47.440588223948964, 53.12069124000368 }    ,{ -47.440588223948964, 100.0 }    ,{ -43.144175450658196, 100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.surf_surBou[1],internal_space_1_space_2.port_b)
        annotation (Line(
        points={{ -51.737000997239726, 53.12069124000368 }    ,{ -88.36850049861985, 53.12069124000368 }    ,{ -88.36850049861987, 75.0 }    ,{ -125.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.surf_surBou[2],internal_space_2_space_3.port_a)
        annotation (Line(
        points={{ -51.737000997239726, 53.12069124000368 }    ,{ -88.36850049861985, 53.12069124000368 }    ,{ -88.36850049861987, 150.0 }    ,{ -125.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_2.weaBus,weather_5.weaBus)
        annotation (Line(
        points={{ -51.737000997239726, 53.12069124000368 }    ,{ -30.310528423601117, 53.12069124000368 }    ,{ -30.310528423601117, 10.35711002849527 }    ,{ -8.884055849962508, 10.35711002849527 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.heaPorRad,heatPortRad[3])
        annotation (Line(
        points={{ -24.070689434137364, -48.51778656126482 }    ,{ -12.035344717068682, -48.51778656126482 }    ,{ -12.035344717068682, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_3.heaPorAir,heatPortCon[3])
        annotation (Line(
        points={{ -24.070689434137364, -48.51778656126482 }    ,{ -12.035344717068682, -48.51778656126482 }    ,{ -12.035344717068682, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(space_3.qGai_flow,occupancy_2.y)
        annotation (Line(
        points={{ -24.070689434137364, -48.51778656126482 }    ,{ -41.738509903758995, -48.51778656126482 }    ,{ -41.738509903758995, -39.24533504917732 }    ,{ -59.40633037338062, -39.24533504917732 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.surf_surBou[1],internal_space_1_space_3.port_b)
        annotation (Line(
        points={{ -24.070689434137364, -48.51778656126482 }    ,{ -137.03534471706868, -48.51778656126482 }    ,{ -137.03534471706868, 75.0 }    ,{ -250.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.surf_surBou[2],internal_space_2_space_3.port_b)
        annotation (Line(
        points={{ -24.070689434137364, -48.51778656126482 }    ,{ -74.53534471706868, -48.51778656126482 }    ,{ -74.53534471706868, 150.0 }    ,{ -125.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(space_3.weaBus,weather_5.weaBus)
        annotation (Line(
        points={{ -24.070689434137364, -48.51778656126482 }    ,{ -16.477372642049936, -48.51778656126482 }    ,{ -16.477372642049936, 10.35711002849527 }    ,{ -8.884055849962508, 10.35711002849527 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(occupancy_0.dataBus,dataBus)
            ;        
        connect(occupancy_1.dataBus,dataBus)
            ;        
        connect(occupancy_2.dataBus,dataBus)
            ;        
        connect(space_1.heaPorAir,heatPortCon1[1])
            ;        
        connect(space_1.ports[1],ports_b[1])
            ;        
        connect(space_2.heaPorAir,heatPortCon1[2])
            ;        
        connect(space_2.ports[1],ports_b[2])
            ;        
        connect(space_3.heaPorAir,heatPortCon1[3])
            ;        
        connect(space_3.ports[1],ports_b[3])
            ;        
        connect(weather_5.weaBus,dataBus)
        annotation (Line(
        points={{ -8.884055849962508, 10.35711002849527 }    ,{ -8.884055849962508, 10.35711002849527 }    ,{ -8.884055849962508, 10.35711002849527 }    ,{ -8.884055849962508, 10.35711002849527 }    },
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
    buildings_simple_hydronic_three_zones.Components.BaseClasses.
BoilerWithStorageBoiler boiler(
    a={0.9},
    dp=5000*{2,1},
    dp_nominal=5000.0,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    deltaM=0.1,
    hTan=2.0,
    show_T=false,
    Q_flow_nominal=2000.0,
    nSeg=4,
    VTan=0.2,
    T_nominal=353.15,
    dIns=0.002,
    linearizeFlowResistance=false,
    nominal_mass_flow_radiator_loop=0.07142857142857142,
    nominal_mass_flow_rate_boiler=0.07142857142857142,
    V_flow=0.07142857142857142/1000*{0.5,1}
,
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()) "Boiler"  annotation (
    Placement(transformation(origin = { 100.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.BoilerControlBoiler_control
    boiler_control(    threshold_outdoor_air_cutoff=288.15,
    threshold_to_switch_off_boiler=288.15,
    TSup_nominal=353.15
) annotation (
    Placement(transformation(origin = { -100.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
equation        
        connect(boiler.port_a,port_a1)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(boiler.dataBus,boiler_control.dataBus)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ -100.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;        
        connect(boiler.port_b,port_b1)
        annotation (Line(
        points={{ 100.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 50.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern =
        LinePattern.Dash,
        smooth=Smooth.None))
            ;        
        connect(boiler_control.dataBus,dataBus)
            ;end production;
    model bus


extends Trano.BaseClasses.Containers.bus;
// Define Medium Package
package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"});

// Define Fluid Ports
Modelica.Fluid.Interfaces.FluidPort_b[3] port_b(
  redeclare package Medium = Medium)
  annotation (
      Placement(transformation(extent= {{90,40},{110,60}} )),
      iconTransformation(extent= {{90,40},{110,60}} )
  );

// Define Heat Transfer Ports
Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3] heatPortCon

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
        buildings_simple_hydronic_three_zones.Components.BaseClasses.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 0.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
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
connect(distribution1.port_b1[1],
emission1.port_a[1])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[2],
emission1.port_a[2])
annotation (Line(points={{-24,15.4},{-22,15},{-4,15}}, color={0,127,255}));
connect(distribution1.port_b1[3],
emission1.port_a[3])
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
        model EmissionControlEmission_valve_control
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
connect(dataBus.yCooValve, emissionControl.yCoo);
connect(dataBus.yHeaValve, emissionControl.yHea);
end EmissionControlEmission_valve_control;
        model OccupancyOccupancy_0
extends buildings_simple_hydronic_three_zones.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_1, occSch2.occupied);
 end OccupancyOccupancy_0;
 
        model EmissionControlEmission_valve_control_2
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
connect(dataBus.yCooValve_2, emissionControl.yCoo);
connect(dataBus.yHeaValve_2, emissionControl.yHea);
end EmissionControlEmission_valve_control_2;
        model OccupancyOccupancy_1
extends buildings_simple_hydronic_three_zones.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_2, occSch2.occupied);
 end OccupancyOccupancy_1;
 
        model EmissionControlEmission_valve_control_3
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
connect(dataBus.yCooValve_3, emissionControl.yCoo);
connect(dataBus.yHeaValve_3, emissionControl.yHea);
end EmissionControlEmission_valve_control_3;
        model OccupancyOccupancy_2
extends buildings_simple_hydronic_three_zones.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_3, occSch2.occupied);
 end OccupancyOccupancy_2;
 
        model ThreeWayValveControlThree_way_valve_control
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
connect(dataBus.TColSetThree_way_valve_control, conVal.u_s);
connect(dataBus.triggerThree_way_valve_control, conVal.trigger);
  connect(conVal.y, y)
annotation (Line(points={{10,0},{110,0}}, color={0,0,127}));  connect(u, conVal.u_m) annotation (Line(points={{-118,0},{-22,0},{-22,-20},{0,
          -20},{0,-16},{-2,-16},{-2,-12}}, color={0,0,127}));annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));end ThreeWayValveControlThree_way_valve_control;
        model ThreeWayValveControlThree_way_valve_control_2
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
connect(dataBus.TColSetThree_way_valve_control_2, conVal.u_s);
connect(dataBus.triggerThree_way_valve_control_2, conVal.trigger);
  connect(conVal.y, y)
annotation (Line(points={{10,0},{110,0}}, color={0,0,127}));  connect(u, conVal.u_m) annotation (Line(points={{-118,0},{-22,0},{-22,-20},{0,
          -20},{0,-16},{-2,-16},{-2,-12}}, color={0,0,127}));annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));end ThreeWayValveControlThree_way_valve_control_2;
            model BoilerControlBoiler_control
    extends buildings_simple_hydronic_three_zones.Trano.Controls.ventilation.PartialBoilerControl;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.TStoTopBoiler, sub1.u1);
connect(dataBus.TStoBotBoiler, greThr.u);
connect(dataBus.TAirOutBoiler, lesThrTOut.u);
connect(dataBus.yBoiConBoiler, booToReaBoi.y);
connect(dataBus.yPumBoiBoiler, booToReaPum.y);
     end BoilerControlBoiler_control;
     
        model CollectorControlPump_control
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
coordinateSystem(preserveAspectRatio=false)));connect(dataBus.y_gainPump, conPum.u_m);
connect(dataBus.yPump, conPum.y);
connect(dataBus.yBoiConPump, mulMax.y);
connect(dataBus.yPumBoiPump, mulMax.y);
connect(dataBus.yHeaValve, mulMax.u[1]);
connect(dataBus.yHeaValve_2, mulMax.u[2]);
connect(dataBus.yHeaValve_3, mulMax.u[3]);
end CollectorControlPump_control;
        model DataServer
replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;
Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[3]
TRoo annotation (
Placement(transformation(origin={-544,-226},
extent = {{480, 216}, {500, 236}})));Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[3]
port annotation (
Placement(transformation(extent={{-112,-10},{-92,10}}),
iconTransformation(extent = {{-110, -10}, {-90, 10}})));Buildings.Fluid.Sensors.PPM[3] TRoo1(redeclare
package Medium = Medium)annotation (
Placement(transformation(origin={-542,-268},
extent = {{480, 216}, {500, 236}})));Modelica.Fluid.Interfaces.FluidPort_a[3]
port_a(redeclare package Medium
= Medium)annotation (Placement(transformation(extent={{-116,-58},{-82,-26}}),
iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));  Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
Modelica.Blocks.Sources.RealExpression
            TColSetThree_way_valve_control_2
            (y=363.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmission_valve_control_2
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TColSetThree_way_valve_control
            (y=363.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmission_valve_control_3
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            TAirOutBoiler_control
            (y=0.0);
Modelica.Blocks.Sources.RealExpression
            TCooSetEmission_valve_control
            (y=298.15);
Modelica.Blocks.Sources.BooleanExpression
            triggerThree_way_valve_control_2
            (y=true);
Modelica.Blocks.Sources.BooleanExpression
            triggerThree_way_valve_control
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
connect(dataBus.TZonSpace_1, TRoo[1].T);
connect(dataBus.TZonSpace_2, TRoo[2].T);
connect(dataBus.TZonSpace_3, TRoo[3].T);
connect(dataBus.ppmCO2Space_1, TRoo1[1].ppm);
connect(dataBus.ppmCO2Space_2, TRoo1[2].ppm);
connect(dataBus.ppmCO2Space_3, TRoo1[3].ppm);
connect(dataBus.TColSetThree_way_valve_control_2,
TColSetThree_way_valve_control_2.y);
connect(dataBus.TCooSetSpace_2,
TCooSetEmission_valve_control_2.y);
connect(dataBus.TColSetThree_way_valve_control,
TColSetThree_way_valve_control.y);
connect(dataBus.TCooSetSpace_3,
TCooSetEmission_valve_control_3.y);
connect(dataBus.TAirOutBoiler,
TAirOutBoiler_control.y);
connect(dataBus.TCooSetSpace_1,
TCooSetEmission_valve_control.y);
connect(dataBus.triggerThree_way_valve_control_2,
triggerThree_way_valve_control_2.y);
connect(dataBus.triggerThree_way_valve_control,
triggerThree_way_valve_control.y);


connect(term_p, loa.terminal) annotation (Line(points={{92,0},{-32,0},{-32,-51},
{ -28,-51 } }, color={0,120,120}));
connect(dataBus.electricityPump, sum1.u[2]) 


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
            model BoilerWithStorageBoiler
    extends buildings_simple_hydronic_three_zones.Trano.Fluid.Boilers.PartialBoilerWithStorage;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.yBoiConBoiler, Boiy.u);
connect(dataBus.yPumBoiBoiler, pumBoi.y);
connect(dataBus.TStoTopBoiler, tanTemTop.T);
connect(dataBus.TStoBotBoiler, tanTemBot.T);
     end BoilerWithStorageBoiler;
     
        model PumpPump
extends buildings_simple_hydronic_three_zones.Trano.Fluid.Ventilation.PartialPump;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.yPump, pumRad.y);
connect(dataBus.y_gainPump, gain.y);
connect(dataBus.electricityPump, pumRad.P);
connect(dataBus.TPump_control, temSup.T);
 end PumpPump;
 
  end BaseClasses;
end Components;


model building
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


    Buildings.ThermalZones.Detailed.MixedAir space_1(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=3,
                    datConExt(
                    layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    azi={ 135.0, 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 10.0, 10.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
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
    Placement(transformation(origin = { 0.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 emission(
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
    Placement(transformation(origin = { 30.0, -75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            valve(
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
        buildings_simple_hydronic_three_zones.Components.BaseClasses.EmissionControlEmission_valve_control
    emission_valve_control(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { 544.706391361582, 787.5842736861489 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.OccupancyOccupancy_0
    occupancy_0(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -50.0, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_2(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=3,
                    datConExt(
                    layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    azi={ 135.0, 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 10.0, 10.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
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
    Placement(transformation(origin = { 250.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 emission_2(
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
    Placement(transformation(origin = { 250.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.EmissionControlEmission_valve_control_2
    emission_valve_control_2(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { 454.4374994246578, 701.2593736892181 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 200.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
    Buildings.ThermalZones.Detailed.MixedAir space_3(
        redeclare package Medium = Medium,
            hRoo=2.0,
    AFlo=20.0,
    linearizeRadiation=true,
    m_flow_nominal=0.01,
    mSenFac=1.0,
    T_start=294.15
,nPorts = 3,                    nConExt=3,
                    datConExt(
                    layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                    azi={ 135.0, 0.0, 90.0 }),
                    nSurBou=2,
                    surBou(
                    A={ 10.0, 10.0 },
                    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
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
    Placement(transformation(origin = { 500.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 emission_3(
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
    Placement(transformation(origin = { 500.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.EmissionControlEmission_valve_control_3
    emission_valve_control_3(    schedule=3600*{7, 19},
    THeaSet=297.0,
    THeaSetBack=289.0
) annotation (
    Placement(transformation(origin = { 265.1176920032035, 441.5480782018886 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.OccupancyOccupancy_2
    occupancy_2(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { 450.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_1_space_2(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -125.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_1_space_3(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -250.0, 75.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.HeatTransfer.Conduction.MultiLayer
                internal_space_2_space_3(A =
            10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms"  annotation (
    Placement(transformation(origin = { -125.0, 150.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather_5(filNam=Modelica.Utilities.Files.loadResource
        ("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
     annotation (
    Placement(transformation(origin = { -100.0, 200.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { 400.55785181025675, 1000.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             three_way_valve(
    redeclare package Medium = MediumW,
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
    Placement(transformation(origin = { 272.85949682871376, 931.4659273430386 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.
    ThreeWayValveControlThree_way_valve_control
    three_way_valve_control annotation (
    Placement(transformation(origin = { 358.8018153197522, 838.3377494961483 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Sensors.TemperatureTwoPort temperature_sensor_2(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator"  annotation (
    Placement(transformation(origin = { 0.0, 521.141313799912 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             three_way_valve_2(
    redeclare package Medium = MediumW,
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
    Placement(transformation(origin = { 45.87088400180427, 658.3424555229316 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.
    ThreeWayValveControlThree_way_valve_control_2
    three_way_valve_control_2 annotation (
    Placement(transformation(origin = { 185.11290515598682, 561.1936939241101 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.FixedResistances.Junction split_valve (
        dp_nominal={10000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.008*{1,-1,-1},
    linearized=false
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { 410.37088860454196, 791.7071623681545 },
    extent = {{10, -10}, {-10, 10}}
)));
        Buildings.Fluid.FixedResistances.Junction split_valve_2 (
        dp_nominal={10000,-1,-1},
    deltaM=0.3,
    m_flow_nominal=0.008*{1,-1,-1},
    linearized=false
,
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"  annotation (
    Placement(transformation(origin = { 97.12697112242364, 531.0239700451164 },
    extent = {{10, -10}, {-10, 10}}
)));
    buildings_simple_hydronic_three_zones.Components.BaseClasses.
BoilerWithStorageBoiler boiler(
    a={0.9},
    dp=5000*{2,1},
    dp_nominal=5000.0,
    effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
    deltaM=0.1,
    hTan=2.0,
    show_T=false,
    Q_flow_nominal=2000.0,
    nSeg=4,
    VTan=0.2,
    T_nominal=353.15,
    dIns=0.002,
    linearizeFlowResistance=false,
    nominal_mass_flow_radiator_loop=0.07142857142857142,
    nominal_mass_flow_rate_boiler=0.07142857142857142,
    V_flow=0.07142857142857142/1000*{0.5,1}
,
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()) "Boiler"  annotation (
    Placement(transformation(origin = { 205.46621130248272, 701.034302843054 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.BoilerControlBoiler_control
    boiler_control(    threshold_outdoor_air_cutoff=288.15,
    threshold_to_switch_off_boiler=288.15,
    TSup_nominal=353.15
) annotation (
    Placement(transformation(origin = { 317.5520799771704, 590.3301380093507 },
    extent = {{10, -10}, {-10, 10}}
)));
      buildings_simple_hydronic_three_zones.Components.BaseClasses.PumpPump
     pump(
         dp_nominal=10000.0,
    m_flow_nominal=0.008
,
    redeclare package Medium = MediumW

    ) annotation (
    Placement(transformation(origin = { 149.06426342388454, 804.3418212323652 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.CollectorControlPump_control
    pump_control annotation (
    Placement(transformation(origin = { 297.8339516344322, 707.7250452699316 },
    extent = {{10, -10}, {-10, 10}}
)));
        buildings_simple_hydronic_three_zones.Components.BaseClasses.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 473.0325597665492, 557.4186420043582 },
    extent = {{10, -10}, {-10, 10}}
)));


equation            
        connect(space_1.heaPorRad,emission.heatPortRad)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 15.0, 0.0 }    ,{ 15.0, -75.0 }    ,{ 30.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.heaPorAir,emission.heatPortCon)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 15.0, 0.0 }    ,{ 15.0, -75.0 }    ,{ 30.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.qGai_flow,occupancy_0.y)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -50.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -62.5, 0.0 }    ,{ -62.5, 75.0 }    ,{ -125.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.surf_surBou[2],internal_space_1_space_3.port_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -125.0, 0.0 }    ,{ -125.0, 75.0 }    ,{ -250.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.weaBus,weather_5.weaBus)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -50.0, 0.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emission.port_b,split_valve.port_1)
        annotation (Line(
        points={{ 30.0, -75.0 }    ,{ 220.18544430227098, -75.0 }    ,{ 220.18544430227098, 791.7071623681545 }    ,{ 410.37088860454196, 791.7071623681545 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve.y,emission_valve_control.y)
        annotation (Line(
        points={{ 0.0, -75.0 }    ,{ 272.353195680791, -75.0 }    ,{ 272.353195680791, 787.5842736861489 }    ,{ 544.706391361582, 787.5842736861489 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve.port_b,emission.port_a)
        annotation (Line(
        points={{ 0.0, -75.0 }    ,{ 15.0, -75.0 }    ,{ 15.0, -75.0 }    ,{ 30.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.heaPorRad,emission_2.heatPortRad)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 265.0, 150.0 }    ,{ 265.0, 75.0 }    ,{ 280.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.heaPorAir,emission_2.heatPortCon)
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
        connect(space_2.surf_surBou[1],internal_space_1_space_2.port_b)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 62.5, 150.0 }    ,{ 62.5, 75.0 }    ,{ -125.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.surf_surBou[2],internal_space_2_space_3.port_a)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 62.5, 150.0 }    ,{ 62.5, 150.0 }    ,{ -125.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.weaBus,weather_5.weaBus)
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 75.0, 150.0 }    ,{ 75.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emission_2.port_b,split_valve.port_1)
        annotation (Line(
        points={{ 280.0, 75.0 }    ,{ 345.185444302271, 75.0 }    ,{ 345.185444302271, 791.7071623681545 }    ,{ 410.37088860454196, 791.7071623681545 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_2.y,emission_valve_control_2.y)
        annotation (Line(
        points={{ 250.0, 75.0 }    ,{ 352.2187497123289, 75.0 }    ,{ 352.2187497123289, 701.2593736892181 }    ,{ 454.4374994246578, 701.2593736892181 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_2.port_b,emission_2.port_a)
        annotation (Line(
        points={{ 250.0, 75.0 }    ,{ 265.0, 75.0 }    ,{ 265.0, 75.0 }    ,{ 280.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.heaPorRad,emission_3.heatPortRad)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 515.0, 150.0 }    ,{ 515.0, 75.0 }    ,{ 530.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.heaPorAir,emission_3.heatPortCon)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 515.0, 150.0 }    ,{ 515.0, 75.0 }    ,{ 530.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.qGai_flow,occupancy_2.y)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 475.0, 150.0 }    ,{ 475.0, 150.0 }    ,{ 450.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.surf_surBou[1],internal_space_1_space_3.port_b)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 125.0, 150.0 }    ,{ 125.0, 75.0 }    ,{ -250.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.surf_surBou[2],internal_space_2_space_3.port_b)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 187.5, 150.0 }    ,{ 187.5, 150.0 }    ,{ -125.0, 150.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.weaBus,weather_5.weaBus)
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 200.0, 150.0 }    ,{ 200.0, 200.0 }    ,{ -100.0, 200.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emission_3.port_b,split_valve_2.port_1)
        annotation (Line(
        points={{ 530.0, 75.0 }    ,{ 313.5634855612118, 75.0 }    ,{ 313.5634855612118, 531.0239700451164 }    ,{ 97.12697112242364, 531.0239700451164 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_3.y,emission_valve_control_3.y)
        annotation (Line(
        points={{ 500.0, 75.0 }    ,{ 382.55884600160175, 75.0 }    ,{ 382.55884600160175, 441.5480782018886 }    ,{ 265.1176920032035, 441.5480782018886 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(valve_3.port_b,emission_3.port_a)
        annotation (Line(
        points={{ 500.0, 75.0 }    ,{ 515.0, 75.0 }    ,{ 515.0, 75.0 }    ,{ 530.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperature_sensor.port_b,valve.port_a)
        annotation (Line(
        points={{ 400.55785181025675, 1000.0 }    ,{ 200.27892590512838, 1000.0 }    ,{ 200.27892590512838, -75.0 }    ,{ 0.0, -75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperature_sensor.port_b,valve_2.port_a)
        annotation (Line(
        points={{ 400.55785181025675, 1000.0 }    ,{ 325.2789259051284, 1000.0 }    ,{ 325.2789259051284, 75.0 }    ,{ 250.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperature_sensor.T,three_way_valve_control.u)
        annotation (Line(
        points={{ 400.55785181025675, 1000.0 }    ,{ 379.6798335650045, 1000.0 }    ,{ 379.6798335650045, 838.3377494961483 }    ,{ 358.8018153197522, 838.3377494961483 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve.y,three_way_valve_control.y)
        annotation (Line(
        points={{ 272.85949682871376, 931.4659273430386 }    ,{ 315.830656074233, 931.4659273430386 }    ,{ 315.830656074233, 838.3377494961483 }    ,{ 358.8018153197522, 838.3377494961483 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve.port_2,temperature_sensor.port_a)
        annotation (Line(
        points={{ 272.85949682871376, 931.4659273430386 }    ,{ 336.7086743194852, 931.4659273430386 }    ,{ 336.7086743194852, 1000.0 }    ,{ 400.55785181025675, 1000.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve.port_3,split_valve.port_3)
        annotation (Line(
        points={{ 272.85949682871376, 931.4659273430386 }    ,{ 341.61519271662786, 931.4659273430386 }    ,{ 341.61519271662786, 791.7071623681545 }    ,{ 410.37088860454196, 791.7071623681545 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperature_sensor_2.port_b,valve_3.port_a)
        annotation (Line(
        points={{ 0.0, 521.141313799912 }    ,{ 250.0, 521.141313799912 }    ,{ 250.0, 75.0 }    ,{ 500.0, 75.0 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(temperature_sensor_2.T,three_way_valve_control_2.u)
        annotation (Line(
        points={{ 0.0, 521.141313799912 }    ,{ 92.55645257799341, 521.141313799912 }    ,{ 92.55645257799341, 561.1936939241101 }    ,{ 185.11290515598682, 561.1936939241101 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve_2.y,three_way_valve_control_2.y)
        annotation (Line(
        points={{ 45.87088400180427, 658.3424555229316 }    ,{ 115.49189457889555, 658.3424555229316 }    ,{ 115.49189457889554, 561.1936939241101 }    ,{ 185.11290515598682, 561.1936939241101 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve_2.port_2,temperature_sensor_2.port_a)
        annotation (Line(
        points={{ 45.87088400180427, 658.3424555229316 }    ,{ 22.935442000902135, 658.3424555229316 }    ,{ 22.935442000902135, 521.141313799912 }    ,{ 0.0, 521.141313799912 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve_2.port_3,split_valve_2.port_3)
        annotation (Line(
        points={{ 45.87088400180427, 658.3424555229316 }    ,{ 71.49892756211395, 658.3424555229316 }    ,{ 71.49892756211395, 531.0239700451164 }    ,{ 97.12697112242364, 531.0239700451164 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(split_valve.port_2,boiler.port_a)
        annotation (Line(
        points={{ 410.37088860454196, 791.7071623681545 }    ,{ 307.91854995351235, 791.7071623681545 }    ,{ 307.91854995351235, 701.034302843054 }    ,{ 205.46621130248272, 701.034302843054 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(split_valve_2.port_2,boiler.port_a)
        annotation (Line(
        points={{ 97.12697112242364, 531.0239700451164 }    ,{ 151.29659121245317, 531.0239700451164 }    ,{ 151.29659121245317, 701.034302843054 }    ,{ 205.46621130248272, 701.034302843054 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(boiler.dataBus,boiler_control.dataBus)
        annotation (Line(
        points={{ 205.46621130248272, 701.034302843054 }    ,{ 261.50914563982656, 701.034302843054 }    ,{ 261.50914563982656, 590.3301380093507 }    ,{ 317.5520799771704, 590.3301380093507 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(boiler.port_b,pump.port_a)
        annotation (Line(
        points={{ 205.46621130248272, 701.034302843054 }    ,{ 177.26523736318364, 701.034302843054 }    ,{ 177.26523736318364, 804.3418212323652 }    ,{ 149.06426342388454, 804.3418212323652 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(pump.dataBus,pump_control.dataBus)
        annotation (Line(
        points={{ 149.06426342388454, 804.3418212323652 }    ,{ 223.44910752915837, 804.3418212323652 }    ,{ 223.44910752915837, 707.7250452699316 }    ,{ 297.8339516344322, 707.7250452699316 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(pump.port_b,three_way_valve.port_1)
        annotation (Line(
        points={{ 149.06426342388454, 804.3418212323652 }    ,{ 210.96188012629915, 804.3418212323652 }    ,{ 210.96188012629915, 931.4659273430386 }    ,{ 272.85949682871376, 931.4659273430386 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(pump.port_b,three_way_valve_2.port_1)
        annotation (Line(
        points={{ 149.06426342388454, 804.3418212323652 }    ,{ 97.4675737128444, 804.3418212323652 }    ,{ 97.4675737128444, 658.3424555229316 }    ,{ 45.87088400180427, 658.3424555229316 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emission_valve_control.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 544.706391361582, 787.5842736861489 }    ,{ 508.86947556406557, 787.5842736861489 }    ,{ 508.86947556406557, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_0.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ -50.0, 0.0 }    ,{ 211.5162798832746, 0.0 }    ,{ 211.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emission_valve_control_2.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 454.4374994246578, 701.2593736892181 }    ,{ 463.73502959560346, 701.2593736892181 }    ,{ 463.73502959560346, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_1.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 200.0, 150.0 }    ,{ 336.5162798832746, 150.0 }    ,{ 336.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(emission_valve_control_3.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 265.1176920032035, 441.5480782018886 }    ,{ 369.07512588487634, 441.5480782018886 }    ,{ 369.07512588487634, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(occupancy_2.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 450.0, 150.0 }    ,{ 461.5162798832746, 150.0 }    ,{ 461.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve_control.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 358.8018153197522, 838.3377494961483 }    ,{ 415.9171875431507, 838.3377494961483 }    ,{ 415.9171875431507, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(three_way_valve_control_2.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 185.11290515598682, 561.1936939241101 }    ,{ 329.072732461268, 561.1936939241101 }    ,{ 329.072732461268, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(boiler_control.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 317.5520799771704, 590.3301380093507 }    ,{ 395.29231987185983, 590.3301380093507 }    ,{ 395.29231987185983, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(pump_control.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 297.8339516344322, 707.7250452699316 }    ,{ 385.4332557004907, 707.7250452699316 }    ,{ 385.4332557004907, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.heaPorAir,data_bus.port[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 236.5162798832746, 0.0 }    ,{ 236.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_1.ports[1],data_bus.port_a[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 236.5162798832746, 0.0 }    ,{ 236.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.heaPorAir,data_bus.port[2])
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 361.5162798832746, 150.0 }    ,{ 361.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_2.ports[1],data_bus.port_a[2])
        annotation (Line(
        points={{ 250.0, 150.0 }    ,{ 361.5162798832746, 150.0 }    ,{ 361.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.heaPorAir,data_bus.port[3])
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 486.5162798832746, 150.0 }    ,{ 486.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;            
        connect(space_3.ports[1],data_bus.port_a[3])
        annotation (Line(
        points={{ 500.0, 150.0 }    ,{ 486.5162798832746, 150.0 }    ,{ 486.5162798832746, 557.4186420043582 }    ,{ 473.0325597665492, 557.4186420043582 }    },
        color={255,204,51},
        thickness=0.1,pattern =
        LinePattern.Solid,
        smooth=Smooth.None))
            ;annotation (Diagram(coordinateSystem(extent={{-50,-50},{1000,1000}})), Icon(
        coordinateSystem(extent={{-50,-50},{1000,1000}})));
end building;

model building_with_grid
  Components.Containers.building building
    annotation (Placement(transformation(extent={{12,-24},{60,24}})));
  Buildings.Electrical.AC.OnePhase.Sources.Grid gri
    annotation (Placement(transformation(extent={{-46,28},{-26,48}})));
equation
  connect(gri.terminal, building.term_p)
    annotation (Line(points={{-36,28},{-36,0},{12,0}}, color={0,120,120}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end building_with_grid;
end buildings_simple_hydronic_three_zones;