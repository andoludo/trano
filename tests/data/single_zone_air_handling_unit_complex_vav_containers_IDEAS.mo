package single_zone_air_handling_unit_complex_vav_containers_IDEAS

package Trano
  package Occupancy

    model SimpleOccupancy

  parameter Real occupancy[:]=3600*{7, 19}
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
   extends Modelica.Blocks.Interfaces.MO(final nout=3);
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant occ2(k=k)
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
      connect(dataBus.uOutAirFra_max, mulAHUCon.uOutAirFra_max) annotation
        (Line(
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
  parameter Modelica.Units.SI.Temperature TSup_nominal=80 + 273.15 "Check for temperature at the bottom of the tank";
    parameter Modelica.Units.SI.Temperature threshold_outdoor_air_cutoff=15 + 273.15 "Output true if outdoor air is below heating cut-off limit";
    parameter Modelica.Units.SI.Temperature threshold_to_switch_off_boiler=15 + 273.15 "Threshold to switch boiler off";
  Buildings.Controls.OBC.CDL.Reals.GreaterThreshold greThr(t=
        TSup_nominal + 5)
    "Check for temperature at the bottom of the tank" annotation (
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
  Modelica.Blocks.Logical.LessThreshold lesThrTOut(threshold=threshold_outdoor_air_cutoff)
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
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant dTThr1(k=threshold_to_switch_off_boiler) "Threshold to switch boiler off" annotation (Placement(
        transformation(extent={{-208,-22},{-188,-2}})));
equation
  connect(booToReaPum.u, pumOnSig.y)
                     annotation (Line(
      points={{-92,-22},{176,-22},{176,8},{167.5,8}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(sub1.y, lesThr.u2)
             annotation (Line(
      points={{-142,-78},{-116,-78}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(dTThr.y, sub1.u2)
            annotation (Line(
      points={{-182,-102},{-174,-102},{-174,-84},{-166,-84}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(lesThr.y, and1.u2)
             annotation (Line(
      points={{-92,-70},{-84,-70},{-84,40},{-76,40}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(lesThrTOut.y, and1.u1)
                 annotation (Line(
      points={{-93,48},{-76,48}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(and1.y, T1.condition)
                annotation (Line(points={{-52,48},{-44,48},{-44,42},{-36,
          42},{-36,58},{-34,58},{-34,76}},
color={255,0,255}));
  connect(greThr.y, T2.condition)
                  annotation (Line(points={{-92,-132},{86,-132},{86,76}},
        color={255,0,255}));
  connect(boiOn.active, booToReaBoi.u)
                       annotation (Line(points={{56,77},{56,8},{-92,8}},
        color={255,0,255}));
  connect(pumOn2.active, pumOnSig.u[1])
                        annotation (Line(points={{116,77},{116,5.66667},
          {146,5.66667}},
           color={255,0,255}));
  connect(boiOn.active, pumOnSig.u[2])
                       annotation (Line(points={{56,77},{56,8},{146,8}},
        color={255,0,255}));
  connect(pumOn.active, pumOnSig.u[3])
                       annotation (Line(points={{-4,77},{-4,10.3333},{146,10.3333}},
                         color={255,0,255}));
  connect(off.outPort[1], T1.inPort)
    annotation (Line(points={{-53.5,88},{-38,88}}, color={0,0,0}));
  connect(T1.outPort, pumOn.inPort[1])
    annotation (Line(points={{-32.5,88},{-15,88}}, color={0,0,0}));
  connect(pumOn.outPort[1], T3.inPort)
                       annotation (Line(points={{6.5,88},{22,88}},
                     color={0,0,0}));
  connect(T3.outPort, boiOn.inPort[1])
    annotation (Line(points={{27.5,88},{45,88}}, color={0,0,0}));
  connect(boiOn.outPort[1], T2.inPort)
    annotation (Line(points={{66.5,88},{82,88}}, color={0,0,0}));
  connect(T2.outPort, pumOn2.inPort[1])
    annotation (Line(points={{87.5,88},{105,88}}, color={0,0,0}));
  connect(pumOn2.outPort[1], T4.inPort)
    annotation (Line(points={{126.5,88},{142,88}}, color={0,0,0}));
  connect(T4.outPort, off.inPort[1])
                     annotation (Line(points={{147.5,88},{166,88},{166,118},
          {-94,118},{-94,88},{-75,88}},
                                color={0,0,0}));
  connect(
  dTThr1.y, lesThr.u1) annotation (Line(points={{-186,-12},{-124,-12},{
          -124,
  -70},{-116,-70}}, color={0,0,127}));
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
  Buildings.Fluid.Sensors.TemperatureTwoPort TSup(
    redeclare package Medium = Medium,
    m_flow_nominal=2*100*1.2/3600,
    allowFlowReversal=false)
annotation (Placement(transformation(extent={{48,6},{68,26}})));
  Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
          extent={{-120,22},{-80,62}}), iconTransformation(extent={{-208,22},{-188,
            42}})));
equation
  connect(hex.port_b1, fanSup.port_a) annotation (
    Line(points = {{-6, 2}, {-6, 16}, {4, 16}}, color = {0, 127, 255}));
  connect(hex.port_a2, fanRet.port_b) annotation (
    Line(points = {{-6, -10}, {-6, -24}, {4, -24}}, color = {0, 127, 255}));
  connect(fanRet.port_a, port_a) annotation (
    Line(points = {{24, -24}, {101, -24}}, color = {0, 127, 255}));
  connect(bou.ports[1], hex.port_b2) annotation (Line(points={{-58,-2},{-32,-2},
          {-32,-10},{-26,-10}}, color={0,127,255}));
  connect(bou.ports[2], hex.port_a1) annotation (Line(points={{-58,-6},{-32,-6},
          {-32,2},{-26,2}}, color={0,127,255}));
  connect(fanSup.port_b, TSup.port_a)
    annotation (Line(points={{24,16},{48,16}}, color={0,127,255}));
  connect(TSup.port_b, port_b)
    annotation (Line(points={{68,16},{102,16}}, color={0,127,255}));
  connect(TSup.T, dataBus.TSupAhu) annotation (Line(points={{58,27},{58,42},{
          -100,42}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
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

  parameter Modelica.Units.SI.Volume VRoo[numZon] "Room volume per zone";
  parameter Modelica.Units.SI.Area AFlo[numZon] "Floor area per zone";

  final parameter Modelica.Units.SI.Area ATot=sum(AFlo)
    "Total floor area for all zone";

  constant Real conv=1.2/3600 "Conversion factor for nominal mass flow rate";


  parameter Modelica.Units.SI.HeatFlowRate QHeaAHU_flow_nominal(min=0) = mHeaAir_flow_nominal * Buildings.Utilities.Psychrometrics.Constants.cpAir * (THeaAirSup_nominal-THeaAirMix_nominal)
    "Nominal heating heat flow rate of air handler unit coil";

  parameter Modelica.Units.SI.HeatFlowRate QCooAHU_flow_nominal(max=0) = 1.3 * mCooAir_flow_nominal * Buildings.Utilities.Psychrometrics.Constants.cpAir *(TCooAirSup_nominal-TCooAirMix_nominal)
    "Nominal total cooling heat flow rate of air handler unit coil (negative number)";

  parameter Modelica.Units.SI.MassFlowRate mCooVAV_flow_nominal[numZon]
    "Design mass flow rate per zone for cooling"
    annotation (Dialog(group="Nominal mass flow rate"));

  parameter Modelica.Units.SI.MassFlowRate mHeaVAV_flow_nominal[numZon] = 0.3*mCooVAV_flow_nominal
    "Design mass flow rate per zone for heating"
    annotation (Dialog(group="Nominal mass flow rate"));

  parameter Modelica.Units.SI.MassFlowRate mAir_flow_nominal=0.01
    "Nominal mass flow rate for fan"
    annotation (Dialog(group="Nominal mass flow rate"));
  parameter Modelica.Units.SI.MassFlowRate mCooAir_flow_nominal=0.7*sum(mCooVAV_flow_nominal)
    "Nominal mass flow rate for fan"
    annotation (Dialog(group="Nominal mass flow rate"));
  parameter Modelica.Units.SI.MassFlowRate mHeaAir_flow_nominal = 0.7*sum(mHeaVAV_flow_nominal)
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
  parameter Real ratOAFlo_P = 2.5e-3
    "Outdoor airflow rate required per person";
  parameter Real ratP_A = 5e-2
    "Occupant density";
  parameter Real effZ(final unit="1") = 0.8
    "Zone air distribution effectiveness (limiting value)";
  parameter Real divP(final unit="1") = 0.7
    "Occupant diversity ratio";

  parameter Modelica.Units.SI.VolumeFlowRate VZonOA_flow_nominal[numZon]=(
      ratOAFlo_P*ratP_A + ratOAFlo_A)*AFlo/effZ
    "Zone outdoor air flow rate of each VAV box";

  parameter Modelica.Units.SI.VolumeFlowRate Vou_flow_nominal=(divP*ratOAFlo_P*
      ratP_A + ratOAFlo_A)*sum(AFlo) "System uncorrected outdoor air flow rate";
  parameter Real effVen(final unit="1") = if divP < 0.6 then
    0.88 * divP + 0.22 else 0.75
    "System ventilation efficiency";
  parameter Modelica.Units.SI.VolumeFlowRate Vot_flow_nominal=Vou_flow_nominal/
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
  parameter Real yFanMin = 0.1 "Minimum fan speed";


  parameter Modelica.Units.SI.Temperature TCooAirMix_nominal(displayUnit="degC")=303.15
    "Mixed air temperature during cooling nominal conditions (used to size cooling coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
  parameter Modelica.Units.SI.Temperature TCooAirSup_nominal(displayUnit="degC")=285.15
    "Supply air temperature during cooling nominal conditions (used to size cooling coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
  parameter Modelica.Units.SI.MassFraction wCooAirMix_nominal = 0.017
    "Humidity ratio of mixed air at a nominal conditions used to size cooling coil (in kg/kg dry total)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
   parameter Modelica.Units.SI.Temperature TCooWatInl_nominal(displayUnit="degC") = 279.15
    "Cooling coil nominal inlet water temperature"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));


  parameter Modelica.Units.SI.Temperature THeaAirMix_nominal(displayUnit="degC")=277.15
    "Mixed air temperature during heating nominal conditions (used to size heating coil)"
    annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
  parameter Modelica.Units.SI.Temperature THeaAirSup_nominal(displayUnit="degC")=285.15
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
  Buildings.Fluid.Sensors.VolumeFlowRate senSupFlo(redeclare package Medium =
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
  Buildings.Fluid.Sensors.VolumeFlowRate senRetFlo(redeclare package Medium =
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
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare final package Medium =
        MediumA)
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{556,-5},{524,25}}),
        iconTransformation(extent={{552,11},{532,29}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package Medium =
        MediumA)
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{522,-90},{556,-58}}),
        iconTransformation(extent={{532,-69},{552,-51}})));
  Modelica.Fluid.Interfaces.FluidPorts_b ports[2](redeclare each package Medium =
               MediumA, each m_flow(max=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Leaving
           then 0 else +Modelica.Constants.inf, min=if flowDirection ==
          Modelica.Fluid.Types.PortFlowDirection.Entering then 0 else -Modelica.Constants.inf))
    "Fluid ports"
    annotation (Placement(transformation(extent={{-110,26},{-90,-54}}),
        iconTransformation(extent={{-110,26},{-90,-54}})));
  Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
          extent={{-90,78},{-50,118}}), iconTransformation(extent={{-84,54},{-34,
            96}})));
  Modelica.Blocks.Math.RealToBoolean u1SupFan(threshold=0.2)
    "Convert real to integer"
    annotation (Placement(transformation(extent={{316,30},{356,70}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant opeMod(final k=Buildings.Controls.OBC.ASHRAE.G36.Types.OperationModes.occupied)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-66,-170},{-46,-150}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant sumDesPopBreZon(final k=0.0125)
    "Sum of the population component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{200,-160},{220,-140}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant sumDesAreBreZon(final k=0.03)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{324,-160},{344,-140}})));
  Buildings.Fluid.Sensors.RelativePressure dpDisSupFan(redeclare package Medium =
        MediumA) "Supply fan static discharge pressure" annotation (Placement(
        transformation(
        extent={{-18,22},{18,-22}},
        rotation=90,
        origin={404,-28})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant ducPreResReq(final k=2)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-64,-218},{-44,-198}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TOut(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAir_flow_nominal,
    allowFlowReversal=allowFlowReversal,
    transferHeat=true) "Mixed air temperature sensor"
    annotation (Placement(transformation(extent={{-32,-80},{-12,-60}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant sumDesPopBreZon1(final k=0.04)
    "Sum of the population component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{264,-160},{284,-140}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant VSumZonPri_flow(final k=0.03)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{444,-112},{464,-92}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant uOutAirFra_max(final k=0.5)
    "Sum of the area component design breathing zone flow rate"
    annotation (Placement(transformation(extent={{498,-122},{518,-102}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant maxSupResReq(final k=6)
    "AHU operation mode is occupied"
    annotation (Placement(transformation(extent={{-74,-268},{-54,-248}})));

  protected
  parameter Modelica.Fluid.Types.PortFlowDirection flowDirection=Modelica.Fluid.Types.PortFlowDirection.Bidirectional
    "Allowed flow direction" annotation (Evaluate=true, Dialog(tab="Advanced"));
equation
  connect(TSup.port_a,fanSup. port_b) annotation (Line(
      points={{276,-72},{266,-72}},
      color={0,127,255},
      smooth=Smooth.None,
      thickness=0.5));
  connect(TSup.port_b,senSupFlo. port_a)
    annotation (Line(points={{296,-72},{346,-72}}, color={0,127,255}));
  connect(dpSupDuc.port_b,fanSup. port_a)
    annotation (Line(points={{216,-72},{246,-72}}, color={0,127,255}));
  connect(damOut.port_b,splRetOut. port_1)
    annotation (Line(points={{22,-70},{42,-70}},   color={0,127,255}));
  connect(splRetOut.port_2,TMix. port_a)
    annotation (Line(points={{62,-70},{72,-70},{72,-72},{82,-72}},
                                                 color={0,127,255}));
  connect(damRet.port_b,splRetOut. port_3) annotation (Line(points={{52,-16},{52,
          -60}},                                    color={0,127,255}));
    connect(dpSupDuc.port_a, TMix.port_b) annotation (Line(points={{196,-72},{102,
          -72}},               color={0,127,255}));
  connect(senRetFlo.port_b,TRet. port_a) annotation (Line(points={{214,10},{138,
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
  connect(senSupFlo.port_b, port_a) annotation (Line(points={{366,-72},{516,-72},
          {516,-74},{539,-74}}, color={0,127,255}));
  connect(dpRetDuc.port_a, port_b)
    annotation (Line(points={{368,10},{540,10}}, color={0,127,255}));
  connect(damExh.port_b, ports[1]) annotation (Line(points={{-36,-4},{-84,-4},{-84,
          6},{-100,6}}, color={0,127,255}));
  connect(VOut1.port_a, ports[2]) annotation (Line(points={{-68,-70},{-84,-70},{
          -84,-34},{-100,-34}}, color={0,127,255}));
  connect(dataBus.yRetDam, damRet.y) annotation (Line(
      points={{-70,98},{-70,-24},{32,-24},{32,-6},{40,-6}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(dataBus.yOutDam, damOut.y) annotation (Line(
      points={{-70,98},{-70,-48},{12,-48},{12,-58}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(dataBus.ySupFan, fanSup1.y) annotation (Line(
      points={{-70,98},{-70,32},{268,32},{268,22}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(dataBus.ySupFan, fanSup.y) annotation (Line(
      points={{-70,98},{-70,32},{248,32},{248,-52},{256,-52},{256,-60}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(dataBus.yOutDam, damExh.y) annotation (Line(
      points={{-70,98},{-70,18},{-26,18},{-26,8}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(opeMod.y, dataBus.uAhuOpeMod) annotation (Line(points={{-44,-160},{-34,
          -160},{-34,-158},{-36,-158},{-36,-154},{-34,-154},{-34,-86},{-38,-86},
          {-38,-52},{-70,-52},{-70,98}}, color={255,127,0}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(dpDisSupFan.port_a, port_a) annotation (Line(points={{404,-46},{404,-72},
          {516,-72},{516,-74},{539,-74}}, color={0,127,255}));
  connect(VOut1.port_a, dpDisSupFan.port_b) annotation (Line(points={{-68,-70},{
          -84,-70},{-84,-26},{376,-26},{376,0},{404,0},{404,-10}}, color={0,127,
          255}));
  connect(dpDisSupFan.p_rel, dataBus.dpDuc) annotation (Line(points={{384.2,-28},
          {-70,-28},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(ducPreResReq.y, dataBus.uZonPreResReq) annotation (Line(points={{-42,-208},
          {-38,-208},{-38,-204},{-40,-204},{-40,-200},{-38,-200},{-38,-178},{-70,
          -178},{-70,98}}, color={255,127,0}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(VOut1.port_b, TOut.port_a)
    annotation (Line(points={{-48,-70},{-32,-70}}, color={0,127,255}));
  connect(TOut.port_b, damOut.port_a)
    annotation (Line(points={{-12,-70},{2,-70}}, color={0,127,255}));
  connect(TSup.T, dataBus.TAirSup) annotation (Line(points={{286,-61},{136,-61},
          {136,38},{-70,38},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(dataBus.ySupFan, u1SupFan.u) annotation (Line(
      points={{-70,98},{-70,50},{312,50}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(u1SupFan.y, dataBus.u1SupFan) annotation (Line(points={{358,50},{366,50},
          {366,76},{-44,76},{-44,72},{-70,72},{-70,98}}, color={255,0,255}),
      Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(sumDesPopBreZon.y, dataBus.VSumAdjPopBreZon_flow) annotation (Line(
        points={{222,-150},{230,-150},{230,-6},{200,-6},{200,48},{-70,48},{-70,98}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(sumDesAreBreZon.y, dataBus.VSumAdjAreBreZon_flow) annotation (Line(
        points={{346,-150},{364,-150},{364,-26},{-70,-26},{-70,98}}, color={0,0,
          127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(TOut.T, dataBus.TOut) annotation (Line(points={{-22,-59},{-22,-50},{-70,
          -50},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(VOut1.V_flow, dataBus.VAirOut_flow) annotation (Line(points={{-58,-59},
          {-38,-59},{-38,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(VSumZonPri_flow.y, dataBus.VSumZonPri_flow) annotation (Line(points={{
          466,-102},{200,-102},{200,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(uOutAirFra_max.y, dataBus.uOutAirFra_max) annotation (Line(points={{520,
          -112},{226,-112},{226,98},{-70,98}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(maxSupResReq.y, dataBus.uZonTemResReq) annotation (Line(points={{-52,-258},
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

  constant Integer numZon(min=2)=2 "Total number of served VAV boxes";

  parameter Modelica.Units.SI.Volume VRoo[numZon] "Room volume per zone";
  parameter Modelica.Units.SI.Area AFlo[numZon] "Floor area per zone";

  final parameter Modelica.Units.SI.Area ATot=sum(AFlo)
"Total floor area for all zone";

  constant Real conv=1.2/3600 "Conversion factor for nominal mass flow rate";

  parameter Modelica.Units.SI.HeatFlowRate QHeaAHU_flow_nominal(min=0)=
    mHeaAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir*(
    THeaAirSup_nominal - THeaAirMix_nominal)
"Nominal heating heat flow rate of air handler unit coil";

  parameter Modelica.Units.SI.HeatFlowRate QCooAHU_flow_nominal(max=0) = 1.3*
    mCooAir_flow_nominal*Buildings.Utilities.Psychrometrics.Constants.cpAir*(
    TCooAirSup_nominal - TCooAirMix_nominal)
"Nominal total cooling heat flow rate of air handler unit coil (negative number)";

  parameter Modelica.Units.SI.MassFlowRate mCooVAV_flow_nominal[numZon]
"Design mass flow rate per zone for cooling"
annotation (Dialog(group="Nominal mass flow rate"));

  parameter Modelica.Units.SI.MassFlowRate mHeaVAV_flow_nominal[numZon]=0.3*
      mCooVAV_flow_nominal
"Design mass flow rate per zone for heating"
annotation (Dialog(group="Nominal mass flow rate"));

  parameter Modelica.Units.SI.MassFlowRate mAir_flow_nominal=
      0.01
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
  parameter Modelica.Units.SI.VolumeFlowRate Vot_flow_nominal=Vou_flow_nominal/
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
       279.15
"Cooling coil nominal inlet water temperature"
annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));

  parameter Modelica.Units.SI.Temperature THeaAirMix_nominal(displayUnit="degC")=
       277.15
"Mixed air temperature during heating nominal conditions (used to size heating coil)"
annotation (Dialog(group="Air handler unit nominal temperatures and humidity"));
  parameter Modelica.Units.SI.Temperature THeaAirSup_nominal(displayUnit="degC")=
       285.15
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
  Buildings.Fluid.Sensors.VolumeFlowRate senSupFlo(redeclare package Medium =
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
  Buildings.Fluid.Sensors.VolumeFlowRate senRetFlo(redeclare package Medium =
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
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare final package Medium =
    MediumA)
"Fluid connector b (positive design flow direction is from port_a to port_b)"
annotation (Placement(transformation(extent={{556,-5},{524,25}}),
    iconTransformation(extent={{552,11},{532,29}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare final package Medium =
    MediumA)
"Fluid connector a (positive design flow direction is from port_a to port_b)"
annotation (Placement(transformation(extent={{522,-90},{556,-58}}),
    iconTransformation(extent={{532,-69},{552,-51}})));
  Modelica.Fluid.Interfaces.FluidPorts_b ports[2](redeclare each package Medium =
           MediumA, each m_flow(max=if flowDirection == Modelica.Fluid.Types.PortFlowDirection.Leaving
       then 0 else +Modelica.Constants.inf, min=if flowDirection ==
      Modelica.Fluid.Types.PortFlowDirection.Entering then 0 else -Modelica.Constants.inf))
"Fluid ports"
annotation (Placement(transformation(extent={{-110,26},{-90,-54}}),
    iconTransformation(extent={{-110,26},{-90,-54}})));
  Buildings.Fluid.Sensors.RelativePressure dpDisSupFan(redeclare package Medium =
    MediumA) "Supply fan static discharge pressure" annotation (Placement(
    transformation(
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
      dpDisSupFan.port_a, port_a) annotation (Line(points={{404,-46},{404,-72},
      {516,-72},{516,-74},{539,-74}}, color={0,127,255}));
  connect(
      VOut1.port_a, dpDisSupFan.port_b) annotation (Line(points={{-68,-70},{
      -84,-70},{-84,-26},{376,-26},{376,0},{404,0},{404,-10}}, color={0,127,
      255}));
  connect(
      VOut1.port_b, TOut.port_a)
annotation (Line(points={{-48,-70},{-32,-70}}, color={0,127,255}));
  connect(
      TOut.port_b, damOut.port_a)
annotation (Line(points={{-12,-70},{2,-70}}, color={0,127,255}));
annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},
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
  QHea_flow_nominal/(cpWatLiq*(THeaWatInl_nominal - THeaWatOut_nominal))
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
  mHeaAir_flow_nominal * cpAir * (THeaAirDis_nominal-THeaAirInl_nominal)
"Nominal heating heat flow rate";
  Modelica.Fluid.Interfaces.FluidPort_a port_aAir(
redeclare package Medium=MediumA)
"Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
    iconTransformation(extent={{-10,-110},{10,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_bAir(
redeclare package Medium=MediumA)
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
      senTem.port_b, senVolFlo.port_a)
annotation (Line(points={{0,50},{0,70},{-6.66134e-16,70}},
                                         color={0,127,255}));
  connect(
      senVolFlo.port_b, port_bAir)
annotation (Line(points={{4.44089e-16,90},{0,90},{0,100}},
                                                 color={0,127,255}));
  connect(
      vav.port_a, port_aAir)
annotation (Line(points={{-5.55112e-16,0},{0,-100}}, color={0,127,255}));
  annotation (Icon(
graphics={
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

    package BoundaryConditions
  package WeatherData
    model ReadearTMYReducedOrder
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                                weaDat(
        calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
        computeWetBulbTemperature=false,
        filNam=Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        "Weather data reader"
        annotation (Placement(transformation(extent={{-84,56},{-64,76}})));

      Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez
                                                       HDifTil[2](
        each outSkyCon=true,
        each outGroCon=true,
        each til=1.5707963267949,
        azi={3.1415926535898,4.7123889803847})
        "Calculates diffuse solar radiation on titled surface for both directions"
        annotation (Placement(transformation(extent={{-56,24},{-36,44}})));
      Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
                                                              HDirTil[2](each til=1.5707963267949,
          azi={3.1415926535898,4.7123889803847})
        "Calculates direct solar radiation on titled surface for both directions"
        annotation (Placement(transformation(extent={{-56,56},{-36,76}})));
      Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane
                                      corGDouPan(UWin=2.1, n=2)
        "Correction factor for solar transmission"
        annotation (Placement(transformation(extent={{18,50},{38,70}})));
      Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow
                                                 eqAirTemp(
        wfGro=0,
        withLongwave=true,
        aExt=0.7,
        hConWallOut=20,
        hRad=5,
        hConWinOut=20,
        n=2,
        wfWall={0.3043478260869566,0.6956521739130435},
        wfWin={0.5,0.5},
        TGro=285.15) "Computes equivalent air temperature"
        annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      Modelica.Blocks.Math.Add solRad[2]
        "Sums up solar radiation of both directions"
        annotation (Placement(transformation(extent={{-26,10},{-16,20}})));
      Buildings.HeatTransfer.Sources.PrescribedTemperature preTem
        "Prescribed temperature for exterior walls outdoor surface temperature"
        annotation (Placement(transformation(extent={{20,-2},{32,10}})));
      Buildings.HeatTransfer.Sources.PrescribedTemperature preTem1
        "Prescribed temperature for windows outdoor surface temperature"
        annotation (Placement(transformation(extent={{20,18},{32,30}})));
      Modelica.Thermal.HeatTransfer.Components.Convection theConWin
        "Outdoor convective heat transfer of windows"
        annotation (Placement(transformation(extent={{50,20},{40,30}})));
      Modelica.Thermal.HeatTransfer.Components.Convection theConWall
        "Outdoor convective heat transfer of walls"
        annotation (Placement(transformation(extent={{48,10},{38,0}})));
      Modelica.Blocks.Sources.Constant const[2](each k=0)
        "Sets sunblind signal to zero (open)"
        annotation (Placement(transformation(extent={{-8,18},{-2,24}})));
      Modelica.Blocks.Sources.Constant hConWall(k=25*11.5)
        "Outdoor coefficient of heat transfer for walls"
        annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90,
        origin={42,-12})));
      Modelica.Blocks.Sources.Constant hConWin(k=20*14)
        "Outdoor coefficient of heat transfer for windows"
        annotation (Placement(transformation(extent={{4,-4},{-4,4}},
        rotation=90,origin={44,42})));
      Buildings.HeatTransfer.Sources.PrescribedTemperature preTemFloor
        "Prescribed temperature for floor plate outdoor surface temperature"
        annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=90,origin={35,-32})));
      Modelica.Blocks.Sources.Constant TSoil(k=283.15)
        "Outdoor surface temperature for floor plate"
        annotation (Placement(transformation(extent={{-4,-4},{4,4}},
        rotation=180,origin={-8,-36})));
      Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007
                                       eqAirTempVDI(
        aExt=0.7,
        n=1,
        wfWall={1},
        wfWin={0},
        wfGro=0,
        hConWallOut=20,
        hRad=5,
        TGro=285.15) "Computes equivalent air temperature for roof"
        annotation (Placement(transformation(extent={{18,78},{38,98}})));
      Buildings.HeatTransfer.Sources.PrescribedTemperature preTemRoof
        "Prescribed temperature for roof outdoor surface temperature"
        annotation (Placement(transformation(extent={{-6,-6},{6,6}},rotation=-90,
        origin={57,68})));
      Modelica.Thermal.HeatTransfer.Components.Convection theConRoof
        "Outdoor convective heat transfer of roof"
        annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,
        origin={59,47})));
      Modelica.Blocks.Sources.Constant hConRoof(k=25*11.5)
        "Outdoor coefficient of heat transfer for roof"
        annotation (Placement(transformation(extent={{4,-4},{-4,4}},origin={78,49})));
      Modelica.Blocks.Sources.Constant const1(k=0)
        "Sets sunblind signal to zero (open)"
        annotation (Placement(transformation(extent={{80,94},{74,100}})));
      Buildings.BoundaryConditions.WeatherData.Bus
                                         weaBus "Weather data bus"
        annotation (Placement(transformation(extent={{-88,-6},{-54,26}}),
        iconTransformation(extent={{-96,70},{-66,100}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ConvRoof annotation (
          Placement(transformation(extent={{-10,-10},{10,10}}, origin={102,70}),
            iconTransformation(extent={{98,66},{118,86}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ConvWin annotation (
          Placement(transformation(extent={{-10,-10},{10,10}}, origin={102,40}),
            iconTransformation(extent={{98,22},{118,42}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a ConvWall annotation (
          Placement(transformation(extent={{-10,-10},{10,10}}, origin={102,-20}),
            iconTransformation(extent={{98,-40},{118,-20}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a TempFloor annotation (
          Placement(transformation(extent={{-10,-10},{10,10}}, origin={102,-80}),
            iconTransformation(extent={{98,-82},{118,-62}})));
      Modelica.Blocks.Interfaces.RealOutput[2] solarRadTrans annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-110}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-110})));
    equation
      connect(eqAirTemp.TEqAirWin,preTem1. T)
        annotation (Line(points={{9,3.8},{12,3.8},{12,24},{18.8,24}},
        color={0,0,127}));
      connect(eqAirTemp.TEqAir,preTem. T)
        annotation (Line(points={{9,0},{16,0},{16,4},{18.8,4}},
        color={0,0,127}));
      connect(weaDat.weaBus,weaBus)
        annotation (Line(points={{-64,66},{-62,66},{-62,22},{-72,22},{-72,16},{-71,16},
              {-71,10}},  color={255,204,51},
        thickness=0.5), Text(textString="%second",index=1,extent={{6,3},{6,3}}));
      connect(weaBus.TDryBul,eqAirTemp. TDryBul)
        annotation (Line(points={{-70.915,10.08},{-70.915,2},{-26,2},{-26,-6},{-14,-6}},
        color={255,204,51},
        thickness=0.5), Text(textString="%first",index=-1,extent={{-6,3},{-6,3}}));
      connect(const.y,eqAirTemp. sunblind)
        annotation (Line(points={{-1.7,21},{0,21},{0,12},{-2,12}},
        color={0,0,127}));
      connect(HDifTil.HSkyDifTil,corGDouPan. HSkyDifTil)
        annotation (Line(points={{-35,40},{6,40},{6,62},{12,62},{12,61.8},{16,61.8},
              {16,62}},
        color={0,0,127}));
      connect(HDirTil.H,corGDouPan. HDirTil)
        annotation (Line(points={{-35,66},{16,66}},        color={0,0,127}));
      connect(HDirTil.H,solRad. u1)
        annotation (Line(points={{-35,66},{-30,66},{-30,18},{-27,18}},
        color={0,0,127}));
      connect(HDifTil.H,solRad. u2)
        annotation (Line(points={{-35,34},{-32,34},{-32,12},{-27,12}},
        color={0,0,127}));
      connect(HDifTil.HGroDifTil,corGDouPan. HGroDifTil)
        annotation (Line(points={{-35,28},{8,28},{8,58},{16,58}},
        color={0,0,127}));
      connect(solRad.y,eqAirTemp. HSol)
        annotation (Line(points={{-15.5,15},{-14,15},{-14,6}},
        color={0,0,127}));
      connect(weaDat.weaBus,HDifTil [1].weaBus)
        annotation (Line(points={{-64,66},{-62,66},{-62,34},{-56,34}},
        color={255,204,51},thickness=0.5));
      connect(weaDat.weaBus,HDifTil [2].weaBus)
        annotation (Line(points={{-64,66},{-62,66},{-62,34},{-56,34}},
        color={255,204,51},thickness=0.5));
      connect(weaDat.weaBus,HDirTil [1].weaBus)
        annotation (Line(
        points={{-64,66},{-56,66}},
        color={255,204,51},
        thickness=0.5));
      connect(weaDat.weaBus,HDirTil [2].weaBus)
        annotation (Line(
        points={{-64,66},{-56,66}},
        color={255,204,51},
        thickness=0.5));
      connect(preTem1.port,theConWin. fluid)
        annotation (Line(points={{32,24},{40,24},{40,25}}, color={191,0,0}));
      connect(theConWall.fluid,preTem. port)
        annotation (Line(points={{38,5},{36,5},{36,4},{32,4}}, color={191,0,0}));
      connect(hConWall.y,theConWall. Gc)
        annotation (Line(points={{42,-7.6},{42,0},{43,0}},    color={0,0,127}));
      connect(hConWin.y,theConWin. Gc)
        annotation (Line(points={{44,37.6},{44,30},{45,30}}, color={0,0,127}));
      connect(weaBus.TBlaSky,eqAirTemp. TBlaSky)
        annotation (Line(
        points={{-70.915,10.08},{-46,10.08},{-46,6},{-20,6},{-20,0},{-14,0}},
        color={255,204,51},
        thickness=0.5), Text(
        textString="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
      connect(TSoil.y,preTemFloor. T)
      annotation (Line(points={{-12.4,-36},{-16,-36},{-16,-44},{35,-44},{35,-39.2}},
                                                                color={0,0,127}));
      connect(preTemRoof.port,theConRoof. fluid)
        annotation (Line(points={{57,62},{59,62},{59,52}}, color={191,0,0}));
      connect(eqAirTempVDI.TEqAir,preTemRoof. T)
        annotation (Line(
        points={{39,88},{57,88},{57,75.2}}, color={0,0,127}));
      connect(theConRoof.Gc,hConRoof. y)
        annotation (Line(points={{64,47},{64,49},{73.6,49}},  color={0,0,127}));
      connect(eqAirTempVDI.TDryBul,eqAirTemp. TDryBul)
        annotation (Line(points={{16,82},{-92,82},{-92,10},{-72,10},{-72,6},{-70,6},
              {-70,2},{-26,2},{-26,-6},{-14,-6}},
        color={0,0,127}));
      connect(eqAirTempVDI.TBlaSky,eqAirTemp. TBlaSky)
        annotation (Line(points={{16,88},{8,88},{8,80},{-88,80},{-88,30},{-72,30},{-72,
              10},{-46,10},{-46,6},{-20,6},{-20,0},{-14,0}},
        color={0,0,127}));
      connect(eqAirTempVDI.HSol[1],weaBus. HGloHor)
        annotation (Line(points={{16,94},{-94,94},{-94,-10},{-72,-10},{-72,10.08},{-70.915,
              10.08}},
        color={0,0,127}),Text(
        textString="%second",
        index=1,
        extent={{6,3},{6,3}}));
      connect(HDirTil.inc,corGDouPan. inc)
        annotation (Line(points={{-35,62},{2,62},{2,54},{16,54}},
        color={0,0,127}));
      connect(const1.y,eqAirTempVDI. sunblind[1])
        annotation (Line(points={{73.7,97},{42,97},{42,110},{28,110},{28,100}},
                                          color={0,0,127}));
      connect(preTemFloor.port, TempFloor) annotation (Line(points={{35,-26},{35,-22},
              {86,-22},{86,-80},{102,-80}}, color={191,0,0}));
      connect(theConWall.solid, ConvWall) annotation (Line(points={{48,5},{88,5},{88,
              -20},{102,-20}}, color={191,0,0}));
      connect(theConWin.solid, ConvWin) annotation (Line(points={{50,25},{86,25},{86,
              40},{102,40}}, color={191,0,0}));
      connect(theConRoof.solid, ConvRoof) annotation (Line(points={{59,42},{59,38},{
              82,38},{82,42},{86,42},{86,70},{102,70}}, color={191,0,0}));
      connect(corGDouPan.solarRadWinTrans, solarRadTrans) annotation (Line(
            points={{39,60},{44,60},{44,-2},{0,-2},{0,-110}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={124,142,255},
              fillColor={124,142,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-74,68},{6,-6}},
              lineColor={255,220,220},
              lineThickness=1,
              fillPattern=FillPattern.Sphere,
              fillColor={255,255,0}),
            Polygon(
              points={{32,8},{40,4},{50,0},{60,-2},{62,-4},{74,-14},{66,-30},{62,
                  -44},{54,-46},{-40,-46},{-66,-30},{-56,6},{-16,14},{-10.211,12.158},
                  {-14,6},{-2,16},{12,18},{28,14},{32,8}},
              lineColor={220,220,220},
              lineThickness=0.1,
              fillPattern=FillPattern.Sphere,
              smooth=Smooth.Bezier,
              fillColor={230,230,230}),
            Text(
              extent={{212,-192},{-54,-278}},
              textColor={255,255,255},
              textString=DynamicSelect("", String(weaBus.TDryBul-273.15, format=".1f")))}),
          Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ReadearTMYReducedOrder;
  end WeatherData;
end BoundaryConditions;
  annotation (uses(Buildings(version = "11.0.0"), Modelica(version = "4.0.0"),
      IDEAS(version="3.0.0")),
  Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
            fillPattern =                                                                            FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
end Trano;


package Components
  package Containers
    model envelope

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record gypsum_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.38,
      c=840.0,
      rho=1120.0,
      epsLw=0.88,
      epsSw=0.55);    record rockwool_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.035,
      c=800.0,
      rho=100.0,
      epsLw=0.88,
      epsSw=0.55);    record concrete_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=900.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);    record brick_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=800.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);    record brickhollow_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.3,
      c=880.0,
      rho=850.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record concreteslab_001
    "concreteslab_001"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.concrete_001
        (d=0.125),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.concrete_001
        (d=0.125)    });
    end concreteslab_001;      record cavitywall_001
    "cavitywall_001"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.brick_001
        (d=0.08),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.rockwool_001
        (d=0.1),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.brickhollow_001
        (d=0.14),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.gypsum_001
        (d=0.015)    });
    end cavitywall_001;
end Constructions;
end Data;

replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[0] heatPortCon
    "Nodes for convective heat gains"
    annotation (Placement(transformation(extent={{90,40},{110,60}}),
        iconTransformation(extent={{90,40},{110,60}})));
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[1]  heatPortCon1
"Nodes for convective heat gains"
annotation (Placement(transformation(extent={{90,40},{110,60}}),
    iconTransformation(extent={{-4,98},{6,108}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[0] heatPortRad
    "Nodes for radiative heat gains"
    annotation (Placement(transformation(extent={{90,-62},{110,-42}}),
        iconTransformation(extent={{90,-62},{110,-42}})));
              Trano.Controls.BaseClasses.DataBus
                                                 dataBus annotation (Placement(
    transformation(extent={{-120,52},{-80,92}}),  iconTransformation(extent
      ={{-228,58},{-208,78}})));
    Modelica.Fluid.Interfaces.FluidPorts_b[3] ports_b(redeclare package Medium
      =
Medium) annotation (Placement(
    transformation(extent={{-106,30},{-92,86}}),  iconTransformation(extent={{-106,30},
            {-92,86}})));
    Modelica.Fluid.Interfaces.FluidPorts_a[0] ports_a(redeclare package Medium
      = Medium)
        annotation (Placement(
    transformation(extent={{-110,-100},{-90,-20}}),
                                                  iconTransformation(extent={{-108,
            -92},{-94,-40}})));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={255,128,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
    IDEAS.Buildings.Components.Zone space_001(
    mSenFac=0.822,nPorts = 3,        hZone=2.5,
    V=250.0
,
    n50=0.822*0.5*space_001.n50toAch,
    redeclare package Medium = Medium,
    nSurf=4,
    T_start=293.15) annotation (
    Placement(transformation(origin = { 0.18899973683581095, -0.09759999999999991 },
    extent = {{10, -10}, {-10, 10}}
)));
        IDEAS.Buildings.Components.OuterWall[3]
    merged_externalwall_17_externalwall_18_externalwall_19(
    redeclare parameter single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Constructions.
    cavitywall_001
    constructionType,
    A={ 20.0, 30.0, 50.0 },
    final azi={ 0.0, 90.0, 180.0 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall }) annotation (
    Placement(transformation(origin = { -100.0, 1.7584000000000088 },
    extent = {{10, -10}, {-10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround flooronground_8(
    redeclare parameter single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Constructions.
    concreteslab_001 constructionType,
    redeclare package Medium = Medium,
    A=50.0) annotation (
    Placement(transformation(origin = { -0.2974552820242735, -100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[40; 75; 40],
    k=1/7/3,
    occupancy=3600*{9, 17}
) annotation (
    Placement(transformation(origin = { 100.0, -2.2095999999999947 },
    extent = {{10, -10}, {-10, 10}}
)));
            inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort) "Data reader"
annotation (Placement(transformation(extent={{-96,76},{-76,96}})));     annotation (
    Placement(transformation(origin = { 3.667551855307565, 100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
equation
        
        
        connect(space_001.propsBus[1:3],merged_externalwall_17_externalwall_18_externalwall_19[1:3].propsBus_a)
        annotation (Line(
        points={{ 0.18899973683581095, -0.09759999999999991 }    ,{ -49.905500131582095, -0.09759999999999991 }    ,{ -49.905500131582095, 1.7584000000000088 }    ,{ -100.0, 1.7584000000000088 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(space_001.propsBus[4],flooronground_8.propsBus_a)
        annotation (Line(
        points={{ 0.18899973683581095, -0.09759999999999991 }    ,{ -0.05422777259423128, -0.09759999999999991 }    ,{ -0.05422777259423128, -100.0 }    ,{ -0.2974552820242735, -100.0 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(space_001.yOcc,occupancy_1.y)
        annotation (Line(
        points={{ 0.18899973683581095, -0.09759999999999991 }    ,{ 50.094499868417905, -0.09759999999999991 }    ,{ 50.094499868417905, -2.2095999999999947 }    ,{ 100.0, -2.2095999999999947 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(space_001.ports[1],ports_b[1])
        annotation (Line(
        points={{ 0.18899973683581095, -0.09759999999999991 }    ,{ 0.09449986841790547, -0.09759999999999991 }    ,{ 0.09449986841790547, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern = LinePattern.Dash,
        smooth=Smooth.None))
            ;
        
        
        connect(space_001.ports[2],ports_b[2])
        annotation (Line(
        points={{ 0.18899973683581095, -0.09759999999999991 }    ,{ 0.09449986841790547, -0.09759999999999991 }    ,{ 0.09449986841790547, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern = LinePattern.Dash,
        smooth=Smooth.None))
            ;
        
        
        connect(occupancy_1.dataBus,dataBus)
            ;
        
        
        connect(space_001.gainCon,heatPortCon1[1])
            ;
        
        
        connect(space_001.ports[3],ports_b[3])
            ;
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={238,46,47},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
end envelope;
    model bus



    package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"});
      Modelica.Fluid.Interfaces.FluidPort_b[1] port_b(redeclare package Medium =
            Medium)
        annotation (Placement(transformation(extent={{90,40},{110,60}}),
            iconTransformation(extent={{90,40},{110,60}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[1] heatPortCon
        "Nodes for convective heat gains"
        annotation (Placement(transformation(extent={{-108,42},{-88,62}}),
            iconTransformation(extent={{-108,42},{-88,62}})));

     
    
                      Trano.Controls.BaseClasses.DataBus dataBus annotation (Placement(
            transformation(extent={{-118,68},{-78,108}}), iconTransformation(extent
              ={{-228,58},{-208,78}})));
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
              extent={{-60,100},{60,-100}},
              lineColor={255,128,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Forward)}));             
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 0, 0 },
    extent = {{10, -10}, {-10, 10}}
)));
equation
        
        
        connect(data_bus.port[1],heatPortCon[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern = LinePattern.Dash,
        smooth=Smooth.None))
            ;
        
        
        connect(data_bus.port_a[1],port_b[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern = LinePattern.Dash,
        smooth=Smooth.None))
            ;
        
        
        connect(data_bus.dataBus,dataBus)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={238,46,47},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
end bus;
    model ventilation



  replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
  constrainedby Modelica.Media.Interfaces.PartialMedium
  "Medium in the component"
  annotation (choicesAllMatching = true);

          Trano.Controls.BaseClasses.DataBus
                                             dataBus annotation (Placement(
transformation(extent={{-120,52},{-80,92}}),  iconTransformation(extent
  ={{-228,58},{-208,78}})));
    Modelica.Fluid.Interfaces.FluidPorts_b[3] ports_b(redeclare package Medium
      =
Medium) annotation (Placement(
    transformation(extent={{-108,0},{-92,54}}),   iconTransformation(extent={{-108,0},
            {-92,54}})));
    Modelica.Fluid.Interfaces.FluidPorts_a[3] ports_a(redeclare package Medium
      =
Medium) annotation (Placement(
    transformation(extent={{-110,-76},{-90,-6}}), iconTransformation(extent={{-110,
            -70},{-94,-22}})));
  annotation (
Icon(
  coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
    graphics={Rectangle(
      extent={{-60,100},{60,-100}},
      lineColor={255,128,0},
      fillColor={215,215,215},
      fillPattern=FillPattern.Forward)}));


  annotation (
Icon(
  coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
    graphics={Rectangle(
      extent={{-60,100},{60,-100}},
      lineColor={238,46,47},
      fillColor={215,215,215},
      fillPattern=FillPattern.Forward)}));
      single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.VAVBoxVav_001
     vav_001(
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
    Placement(transformation(origin = { 54.88869906426865, 0.23955837933549162 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.VAVControlVav_control_001
    vav_control_001 annotation (
    Placement(transformation(origin = { 100.0, 0.4228726174357007 },
    extent = {{10, -10}, {-10, 10}}
)));
      IDEAS.Fluid.FixedResistances.PressureDrop
    duct_002(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { -61.54335217277681, 100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
      IDEAS.Fluid.FixedResistances.PressureDrop
    duct_001(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { 4.661884646438665, 0.09790646807623204 },
    extent = {{10, -10}, {-10, 10}}
)));
    single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.AhuAhu_001
    ahu_001
    (redeclare package MediumA = Medium,

    VRoo={100,100},
    AFlo={20,20},
    mCooVAV_flow_nominal={0.01,0.01}) annotation (
    Placement(transformation(origin = { -50.86571728353167, -0.022914279762517253 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.AhuControlAhu_control_001
    ahu_control_001 annotation (
    Placement(transformation(origin = { -100.0, 0.07290907197166518 },
    extent = {{10, -10}, {-10, 10}}
)));
      Buildings.Fluid.Sources.Boundary_pT boundary
    (nPorts=2,redeclare package Medium = Medium) annotation (
    Placement(transformation(origin = { -60.91342047063503, -100.0 },
    extent = {{10, -10}, {-10, 10}}
)));
equation
        
        
        connect(duct_002.port_a,ports_a[1])
        annotation (Line(
        points={{ -61.54335217277681, 100.0 }    ,{ -30.771676086388403, 100.0 }    ,{ -30.771676086388403, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern = LinePattern.Dash,
        smooth=Smooth.None))
            ;
        
        
        connect(vav_001.dataBus,vav_control_001.dataBus)
        annotation (Line(
        points={{ 54.88869906426865, 0.23955837933549162 }    ,{ 77.44434953213432, 0.23955837933549162 }    ,{ 77.44434953213432, 0.4228726174357007 }    ,{ 100.0, 0.4228726174357007 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(vav_001.port_bAir,ports_b[1])
        annotation (Line(
        points={{ 54.88869906426865, 0.23955837933549162 }    ,{ 27.444349532134325, 0.23955837933549162 }    ,{ 27.444349532134325, 0.0 }    ,{ 0.0, 0.0 }    },
        color={0, 0, 139},
        thickness=0.1,pattern = LinePattern.Dash,
        smooth=Smooth.None))
            ;
        
        
        connect(duct_002.port_b,ahu_001.port_a)
        annotation (Line(
        points={{ -61.54335217277681, 100.0 }    ,{ -56.20453472815424, 100.0 }    ,{ -56.20453472815424, -0.022914279762517253 }    ,{ -50.86571728353167, -0.022914279762517253 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(duct_001.port_b,vav_001.port_aAir)
        annotation (Line(
        points={{ 4.661884646438665, 0.09790646807623204 }    ,{ 29.775291855353657, 0.09790646807623204 }    ,{ 29.775291855353657, 0.23955837933549162 }    ,{ 54.88869906426865, 0.23955837933549162 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(ahu_001.dataBus,ahu_control_001.dataBus)
        annotation (Line(
        points={{ -50.86571728353167, -0.022914279762517253 }    ,{ -75.43285864176583, -0.022914279762517253 }    ,{ -75.43285864176583, 0.07290907197166518 }    ,{ -100.0, 0.07290907197166518 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(ahu_001.port_b,duct_001.port_a)
        annotation (Line(
        points={{ -50.86571728353167, -0.022914279762517253 }    ,{ -23.101916318546504, -0.022914279762517253 }    ,{ -23.101916318546504, 0.09790646807623204 }    ,{ 4.661884646438665, 0.09790646807623204 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(boundary.ports,ahu_001.ports)
        annotation (Line(
        points={{ -60.91342047063503, -100.0 }    ,{ -55.88956887708335, -100.0 }    ,{ -55.88956887708335, -0.022914279762517253 }    ,{ -50.86571728353167, -0.022914279762517253 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;
        
        
        connect(vav_control_001.dataBus,dataBus)
            ;
        
        
        connect(ahu_control_001.dataBus,dataBus)
            ;
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
        graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={238,46,47},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward)}));
end ventilation;
  end Containers;

  package BaseClasses
        model OccupancyOccupancy_1
extends single_zone_air_handling_unit_complex_vav_containers_IDEAS.Trano.Occupancy.SimpleOccupancy ;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_001, occSch2.occupied);
 end OccupancyOccupancy_1;
 
        model VAVControlVav_control_001
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
annotation (Placement(transformation(extent={{-36,-36},{28,38}}))); Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.TZonSpace_001, rehBoxCon.TZon);
connect(dataBus.TCooSetSpace_001, rehBoxCon.TCooSet);
connect(dataBus.THeaSetSpace_001, rehBoxCon.THeaSet);
connect(dataBus.ppmCO2SetSpace_001, rehBoxCon.ppmCO2Set);
connect(dataBus.ppmCO2Space_001, rehBoxCon.ppmCO2);
connect(dataBus.TSupSetSpace_001, rehBoxCon.TSupSet);
connect(dataBus.uOpeModSpace_001, rehBoxCon.uOpeMod);
connect(dataBus.oveFloSetSpace_001, rehBoxCon.oveFloSet);
connect(dataBus.oveDamPosSpace_001, rehBoxCon.oveDamPos);
connect(dataBus.u1WinSpace_001, rehBoxCon.u1Win);
connect(dataBus.u1OccSpace_001, rehBoxCon.u1Occ);
connect(dataBus.uHeaOffSpace_001, rehBoxCon.uHeaOff);
connect(dataBus.u1FanSpace_001, rehBoxCon.u1Fan);
connect(dataBus.u1HotPlaSpace_001, rehBoxCon.u1HotPla);
connect(dataBus.TDisVav_control_001, rehBoxCon.TDis);
connect(dataBus.VDis_flowVav_control_001, rehBoxCon.VDis_flow);
connect(dataBus.VAdjPopBreZon_flowVav_control_001, rehBoxCon.VAdjPopBreZon_flow);
connect(dataBus.VAdjAreBreZon_flowVav_control_001, rehBoxCon.VAdjAreBreZon_flow);
connect(dataBus.VMinOA_flowVav_control_001, rehBoxCon.VMinOA_flow);
connect(dataBus.yZonTemResReqVav_control_001, rehBoxCon.yZonTemResReq);
connect(dataBus.yZonPreResReqVav_control_001, rehBoxCon.yZonPreResReq);
connect(dataBus.yHeaValResReqVav_control_001, rehBoxCon.yHeaValResReq);
connect(dataBus.TAirSupAhu_control_001, rehBoxCon.TSup);
connect(dataBus.VSet_flowVav_001, rehBoxCon.VSet_flow);
connect(dataBus.yDamVav_001, rehBoxCon.yDam);
connect(dataBus.yValVav_001, rehBoxCon.yVal);
connect(dataBus.VZonAbsMin_flowVav_001, rehBoxCon.VZonAbsMin_flow);
connect(dataBus.VZonDesMin_flowVav_001, rehBoxCon.VZonDesMin_flow);
connect(dataBus.yCO2Vav_001, rehBoxCon.yCO2);
connect(dataBus.yHotWatPlaReqVav_001, rehBoxCon.yHotWatPlaReq);
connect(dataBus.yLowFloAlaVav_001, rehBoxCon.yLowFloAla);
connect(dataBus.yFloSenAlaVav_001, rehBoxCon.yFloSenAla);
connect(dataBus.yLeaDamAlaVav_001, rehBoxCon.yLeaDamAla);
connect(dataBus.yLeaValAlaVav_001, rehBoxCon.yLeaValAla);
connect(dataBus.yLowTemAlaVav_001, rehBoxCon.yLowTemAla);
end VAVControlVav_control_001;
        model AhuControlAhu_control_001
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
annotation (Placement(transformation(extent={{-12,-14},{28,74}})));Controls.BaseClasses.DataBus dataBus annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));

Buildings.Controls.OBC.ASHRAE.G36.AHUs.MultiZone.VAV.SetPoints.OutdoorAirFlow.ASHRAE62_1.SumZone
sumZon(nZon=1, nGro=1,final zonGroMat=[1],
final zonGroMatTra=[1])
annotation (Placement(transformation(extent={{-72,32},{-52,52}})));Buildings.Controls.OBC.CDL.Integers.MultiSum preRetReq(final
nin=1)
annotation (Placement(transformation(extent={{-72,80},{-60,92}})));Buildings.Controls.OBC.CDL.Integers.MultiSum temResReq(final nin=1)
annotation (Placement(transformation(extent={{-72,56},{-60,68}})));equation
connect(dataBus.dpDucAhu_control_001, mulAHUCon.dpDuc);
connect(dataBus.TOutAhu_control_001, mulAHUCon.TOut);
connect(dataBus.TAirSupAhu_control_001, mulAHUCon.TAirSup);
connect(dataBus.VAirOut_flowAhu_control_001, mulAHUCon.VAirOut_flow);
connect(dataBus.TAirMixAhu_control_001, mulAHUCon.TAirMix);
connect(dataBus.uAhuOpeModAhu_control_001, mulAHUCon.uAhuOpeMod);
connect(dataBus.uAhuOpeModAhu_control_001, sumZon.uOpeMod[1]);
connect(dataBus.u1SupFanAhu_control_001, mulAHUCon.u1SupFan);
connect(dataBus.VAdjPopBreZon_flowVav_control_001, sumZon.VAdjPopBreZon_flow[1]);
connect(dataBus.VAdjAreBreZon_flowVav_control_001, sumZon.VAdjAreBreZon_flow[1]);
connect(dataBus.VDis_flowVav_control_001, sumZon.VZonPri_flow[1]);
connect(dataBus.VMinOA_flowVav_control_001, sumZon.VMinOA_flow[1]);
connect(dataBus.yZonPreResReqVav_control_001, preRetReq.u[1]);
connect(dataBus.yZonTemResReqVav_control_001, temResReq.u[1]);
connect(dataBus.TAirSupSetAhu_001, mulAHUCon.TAirSupSet);
connect(dataBus.VEffAirOut_flow_minAhu_001, mulAHUCon.VEffAirOut_flow_min);
connect(dataBus.yMinOutDamAhu_001, mulAHUCon.yMinOutDam);
connect(dataBus.yRetDamAhu_001, mulAHUCon.yRetDam);
connect(dataBus.yRelDamAhu_001, mulAHUCon.yRelDam);
connect(dataBus.yOutDamAhu_001, mulAHUCon.yOutDam);
connect(dataBus.ySupFanAhu_001, mulAHUCon.ySupFan);
connect(dataBus.yRetFanAhu_001, mulAHUCon.yRetFan);
connect(dataBus.yRelFanAhu_001, mulAHUCon.yRelFan);
connect(dataBus.yCooCoiAhu_001, mulAHUCon.yCooCoi);
connect(dataBus.yHeaCoiAhu_001, mulAHUCon.yHeaCoi);
connect(dataBus.yDpBuiAhu_001, mulAHUCon.yDpBui);
connect(dataBus.dpDisSetAhu_001, mulAHUCon.dpDisSet);
connect(dataBus.yAlaAhu_001, mulAHUCon.yAla);
connect(dataBus.yChiWatResReqAhu_001, mulAHUCon.yChiWatResReq);
connect(dataBus.yChiPlaReqAhu_001, mulAHUCon.yChiPlaReq);
connect(dataBus.yHotWatResReqAhu_001, mulAHUCon.yHotWatResReq);
connect(dataBus.yHotWatPlaReqAhu_001, mulAHUCon.yHotWatPlaReq);
connect(dataBus.y1MinOutDamAhu_001, mulAHUCon.y1MinOutDam);
connect(dataBus.y1EneCHWPumAhu_001, mulAHUCon.y1EneCHWPum);
connect(dataBus.y1SupFanAhu_001, mulAHUCon.y1SupFan);
connect(dataBus.y1RetFanAhu_001, mulAHUCon.y1RetFan);
connect(dataBus.y1RelFanAhu_001, mulAHUCon.y1RelFan);
connect(dataBus.y1RelDamAhu_001, mulAHUCon.y1RelDam);

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
end AhuControlAhu_control_001;
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
Modelica.Blocks.Sources.RealExpression
            THeaSetVav_control_001
            (y=293.15);
Modelica.Blocks.Sources.RealExpression
            TCooSetVav_control_001
            (y=298.15);
Modelica.Blocks.Sources.RealExpression
            ppmCO2SetVav_control_001
            (y=0.0);
Modelica.Blocks.Sources.RealExpression
            TSupSetVav_control_001
            (y=293.15);
Modelica.Blocks.Sources.IntegerExpression
            oveDamPosVav_control_001
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            uOpeModVav_control_001
            (y=1);
Modelica.Blocks.Sources.IntegerExpression
            uAhuOpeModAhu_control_001
            (y=0);
Modelica.Blocks.Sources.IntegerExpression
            oveFloSetVav_control_001
            (y=0);
Modelica.Blocks.Sources.BooleanExpression
            u1FanVav_control_001
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1WinVav_control_001
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1SupFanAhu_control_001
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1OccVav_control_001
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            uHeaOffVav_control_001
            (y=false);
Modelica.Blocks.Sources.BooleanExpression
            u1HotPlaVav_control_001
            (y=false);
equation
connect(port[1],TRoo[1]. port);
connect(port_a[1], TRoo1[1].port);
connect(dataBus.TZonSpace_001, TRoo[1].T);
connect(dataBus.ppmCO2Space_001, TRoo1[1].ppm);
connect(dataBus.THeaSetSpace_001,
THeaSetVav_control_001.y);
connect(dataBus.TCooSetSpace_001,
TCooSetVav_control_001.y);
connect(dataBus.ppmCO2SetSpace_001,
ppmCO2SetVav_control_001.y);
connect(dataBus.TSupSetSpace_001,
TSupSetVav_control_001.y);
connect(dataBus.oveDamPosSpace_001,
oveDamPosVav_control_001.y);
connect(dataBus.uOpeModSpace_001,
uOpeModVav_control_001.y);
connect(dataBus.uAhuOpeModAhu_control_001,
uAhuOpeModAhu_control_001.y);
connect(dataBus.oveFloSetSpace_001,
oveFloSetVav_control_001.y);
connect(dataBus.u1FanSpace_001,
u1FanVav_control_001.y);
connect(dataBus.u1WinSpace_001,
u1WinVav_control_001.y);
connect(dataBus.u1SupFanAhu_control_001,
u1SupFanAhu_control_001.y);
connect(dataBus.u1OccSpace_001,
u1OccVav_control_001.y);
connect(dataBus.uHeaOffSpace_001,
uHeaOffVav_control_001.y);
connect(dataBus.u1HotPlaSpace_001,
u1HotPlaVav_control_001.y);
end DataServer;
      
        model VAVBoxVav_001
extends single_zone_air_handling_unit_complex_vav_containers_IDEAS.Trano.Fluid.Ventilation.PartialVAVBox;
Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.yDamVav_001, vav.y);
connect(dataBus.y_actualVav_001, vav.y_actual);
connect(dataBus.VDis_flowVav_control_001, senVolFlo.V_flow);
connect(dataBus.TDisVav_control_001, senTem.T);
 end VAVBoxVav_001;
 
            model AhuAhu_001
    extends single_zone_air_handling_unit_complex_vav_containers_IDEAS.Trano.Fluid.Ventilation.PartialAhu;
    Trano.Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
    equation
    connect(dataBus.ySupFanAhu_001, fanSup1.y);
connect(dataBus.ySupFanAhu_001, fanSup.y);
connect(dataBus.yRetDamAhu_001, damRet.y);
connect(dataBus.yOutDamAhu_001, damOut.y);
connect(dataBus.yOutDamAhu_001, damExh.y);
connect(dataBus.u1SupFanAhu_001, fanSup.y_actual);
connect(dataBus.TOutAhu_control_001, TOut.T);
connect(dataBus.VAirOut_flowAhu_control_001, VOut1.V_flow);
connect(dataBus.TAirSupAhu_control_001, TSup.T);
connect(dataBus.TAirMixAhu_control_001, TMix.T);
connect(dataBus.dpDucAhu_control_001, dpDisSupFan.p_rel);
     end AhuAhu_001;
     
  end BaseClasses;
end Components;


package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record gypsum_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.38,
      c=840.0,
      rho=1120.0,
      epsLw=0.88,
      epsSw=0.55);    record rockwool_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.035,
      c=800.0,
      rho=100.0,
      epsLw=0.88,
      epsSw=0.55);    record concrete_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=900.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);    record brick_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=800.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);    record brickhollow_001 = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.3,
      c=880.0,
      rho=850.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record concreteslab_001
    "concreteslab_001"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.concrete_001
        (d=0.125),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.concrete_001
        (d=0.125)    });
    end concreteslab_001;      record cavitywall_001
    "cavitywall_001"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.brick_001
        (d=0.08),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.rockwool_001
        (d=0.1),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.brickhollow_001
        (d=0.14),single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Materials.gypsum_001
        (d=0.015)    });
    end cavitywall_001;
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



    IDEAS.Buildings.Components.Zone space_001(
    mSenFac=0.822,nPorts = 3,        hZone=2.5,
    V=250.0
,
    n50=0.822*0.5*space_001.n50toAch,
    redeclare package Medium = Medium,
    nSurf=4,
    T_start=293.15) annotation (
    Placement(transformation(origin = { 0, 0 },
    extent = {{10, -10}, {-10, 10}}
)));
        IDEAS.Buildings.Components.OuterWall[3]
    merged_externalwall_17_externalwall_18_externalwall_19(
    redeclare parameter single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Constructions.
    cavitywall_001
    constructionType,
    A={ 20.0, 30.0, 50.0 },
    final azi={ 0.0, 90.0, 180.0 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall }) annotation (
    Placement(transformation(origin = { 46.21116043119848, 867.6953818827709 },
    extent = {{10, -10}, {-10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround flooronground_8(
    redeclare parameter single_zone_air_handling_unit_complex_vav_containers_IDEAS.Data.Constructions.
    concreteslab_001 constructionType,
    redeclare package Medium = Medium,
    A=50.0) annotation (
    Placement(transformation(origin = { 0.0, 527.4755772646537 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.OccupancyOccupancy_1
    occupancy_1(    gain=[40; 75; 40],
    k=1/7/3,
    occupancy=3600*{9, 17}
) annotation (
    Placement(transformation(origin = { -50, 0 },
    extent = {{10, -10}, {-10, 10}}
)));
      single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.VAVBoxVav_001
     vav_001(
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
    Placement(transformation(origin = { 750.2289861199184, 670.8925399644761 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.VAVControlVav_control_001
    vav_control_001 annotation (
    Placement(transformation(origin = { 867.0119072782359, 1000.0 },
    extent = {{10, -10}, {-10, 10}}
)));
      IDEAS.Fluid.FixedResistances.PressureDrop
    duct_002(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { 497.1640949763969, 358.6922735346359 },
    extent = {{10, -10}, {-10, 10}}
)));
      IDEAS.Fluid.FixedResistances.PressureDrop
    duct_001(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct"  annotation (
    Placement(transformation(origin = { 1000.0, 399.7446714031971 },
    extent = {{10, -10}, {-10, 10}}
)));
            inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort) "Data reader"
annotation (Placement(transformation(extent={{-96,76},{-76,96}})));     annotation (
    Placement(transformation(origin = { -100, 200 },
    extent = {{10, -10}, {-10, 10}}
)));
    single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.AhuAhu_001
    ahu_001
    (redeclare package MediumA = Medium,

    VRoo={100,100},
    AFlo={20,20},
    mCooVAV_flow_nominal={0.01,0.01}) annotation (
    Placement(transformation(origin = { 755.4252096103716, 378.1527531083481 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.AhuControlAhu_control_001
    ahu_control_001 annotation (
    Placement(transformation(origin = { 924.610723596139, 693.4502664298401 },
    extent = {{10, -10}, {-10, 10}}
)));
      Buildings.Fluid.Sources.Boundary_pT boundary
    (nPorts=2,redeclare package Medium = Medium) annotation (
    Placement(transformation(origin = { 582.540689072078, 0.0 },
    extent = {{10, -10}, {-10, 10}}
)));
        single_zone_air_handling_unit_complex_vav_containers_IDEAS.Components.BaseClasses.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { 643.2308180088777, 792.173623445826 },
    extent = {{10, -10}, {-10, 10}}
)));


equation    
        
        
        connect(space_001.propsBus[1:3],merged_externalwall_17_externalwall_18_externalwall_19[1:3].propsBus_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 23.10558021559924, 0.0 }    ,{ 23.10558021559924, 867.6953818827709 }    ,{ 46.21116043119848, 867.6953818827709 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(space_001.propsBus[4],flooronground_8.propsBus_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 0.0, 0.0 }    ,{ 0.0, 527.4755772646537 }    ,{ 0.0, 527.4755772646537 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(space_001.yOcc,occupancy_1.y)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -50.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(space_001.ports[1],duct_002.port_a)
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 248.58204748819844, 0.0 }    ,{ 248.58204748819844, 358.6922735346359 }    ,{ 497.1640949763969, 358.6922735346359 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(vav_001.dataBus,vav_control_001.dataBus)
        annotation (Line(
        points={{ 750.2289861199184, 670.8925399644761 }    ,{ 808.6204466990771, 670.8925399644761 }    ,{ 808.6204466990771, 1000.0 }    ,{ 867.0119072782359, 1000.0 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(vav_001.port_bAir,space_001.ports[2])
        annotation (Line(
        points={{ 750.2289861199184, 670.8925399644761 }    ,{ 375.1144930599592, 670.8925399644761 }    ,{ 375.1144930599592, 0.0 }    ,{ 0.0, 0.0 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(duct_002.port_b,ahu_001.port_a)
        annotation (Line(
        points={{ 497.1640949763969, 358.6922735346359 }    ,{ 626.2946522933842, 358.6922735346359 }    ,{ 626.2946522933842, 378.1527531083481 }    ,{ 755.4252096103716, 378.1527531083481 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(duct_001.port_b,vav_001.port_aAir)
        annotation (Line(
        points={{ 1000.0, 399.7446714031971 }    ,{ 875.1144930599592, 399.7446714031971 }    ,{ 875.1144930599592, 670.8925399644761 }    ,{ 750.2289861199184, 670.8925399644761 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(ahu_001.dataBus,ahu_control_001.dataBus)
        annotation (Line(
        points={{ 755.4252096103716, 378.1527531083481 }    ,{ 840.0179666032552, 378.1527531083481 }    ,{ 840.0179666032552, 693.4502664298401 }    ,{ 924.610723596139, 693.4502664298401 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(ahu_001.port_b,duct_001.port_a)
        annotation (Line(
        points={{ 755.4252096103716, 378.1527531083481 }    ,{ 877.7126048051857, 378.1527531083481 }    ,{ 877.7126048051857, 399.7446714031971 }    ,{ 1000.0, 399.7446714031971 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(boundary.ports,ahu_001.ports)
        annotation (Line(
        points={{ 582.540689072078, 0.0 }    ,{ 668.9829493412248, 0.0 }    ,{ 668.9829493412248, 378.1527531083481 }    ,{ 755.4252096103716, 378.1527531083481 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(occupancy_1.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ -50.0, 0.0 }    ,{ 296.61540900443885, 0.0 }    ,{ 296.61540900443885, 792.173623445826 }    ,{ 643.2308180088777, 792.173623445826 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(vav_control_001.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 867.0119072782359, 1000.0 }    ,{ 755.1213626435567, 1000.0 }    ,{ 755.1213626435567, 792.173623445826 }    ,{ 643.2308180088777, 792.173623445826 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(ahu_control_001.dataBus,data_bus.dataBus)
        annotation (Line(
        points={{ 924.610723596139, 693.4502664298401 }    ,{ 783.9207708025083, 693.4502664298401 }    ,{ 783.9207708025083, 792.173623445826 }    ,{ 643.2308180088777, 792.173623445826 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(space_001.gainCon,data_bus.port[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 321.61540900443885, 0.0 }    ,{ 321.61540900443885, 792.173623445826 }    ,{ 643.2308180088777, 792.173623445826 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;    
        
        
        connect(space_001.ports[3],data_bus.port_a[1])
        annotation (Line(
        points={{ 0.0, 0.0 }    ,{ 321.61540900443885, 0.0 }    ,{ 321.61540900443885, 792.173623445826 }    ,{ 643.2308180088777, 792.173623445826 }    },
        color={255,204,51},
        thickness=0.1,pattern = LinePattern.Solid,
        smooth=Smooth.None))
            ;annotation (Diagram(coordinateSystem(extent={{-50,-50},{1000,1000}})), Icon(
        coordinateSystem(extent={{-50,-50},{1000,1000}})));
end building;

model building_container

Components.Containers.envelope envelope1 annotation (Placement(transformation(extent={{-84.0,0.0},{-64.0,20.0}})));
Components.Containers.bus bus1 annotation (Placement(transformation(extent={{-84.0,30.0},{-64.0,50.0}})));
Components.Containers.ventilation ventilation1 annotation (Placement(transformation(extent={{-44.0,-30.0},{-24.0,-10.0}})));

equation

connect(envelope1.ports_b[1],ventilation1.ports_a[1])
annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));;
connect(envelope1.ports_b[2],ventilation1.ports_b[1])
annotation (Line(points={{-44.1,
          -32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}},
        color={0,127,255}));;
connect(envelope1.heatPortCon1[1],bus1.heatPortCon[1])
annotation (Line(points={{-24,
    5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));;
connect(envelope1.ports_b[3],bus1.port_b[1])
annotation (Line(points={{-24,
      5},{-20,5},{-20,10},{-8,10},{-8,14.8},{-4,14.8}}, color={0,127,255}));;

connect(bus1.dataBus, envelope1.dataBus) annotation (Line(points={{-95.8,46.8},{-95.8,16.8}}, color={255,204,51}, thickness=0.5));
connect(envelope1.dataBus, ventilation1.dataBus) annotation (Line(points={{-44.1,-32.6},{-50,-32.6},{-50,-16},{-90,-16},{-90,15.8},{-83.9,15.8}}, color={255,204,51}, thickness=0.5));

annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
coordinateSystem(preserveAspectRatio=false)));

end building_container;

end single_zone_air_handling_unit_complex_vav_containers_IDEAS;