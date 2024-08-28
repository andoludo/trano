package buildings_free_float_single_zone_with_data

package Common
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
        model OccupancyOccupancy_0
extends buildings_free_float_single_zone_with_data.Common.Occupancy.SimpleOccupancy ;
Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
equation
connect(dataBus.OccupiedSpace_1, occSch2.occupied);
 end OccupancyOccupancy_0;

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
iconTransformation(origin = {-2, -42}, extent = {{-110, -9}, {-90, 9}})));  Controls.BaseClasses.DataBus dataBus
    annotation (Placement(transformation(
  extent={{-120,-18},{-80,22}}), iconTransformation(extent={{-120,62},{-78,98}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
    tableOnFile=false,
    table=[0.0,13.6833333333333,50.15,22.2426666666667;3600.0,13.7333333333333,56.3833333333333,22.0083333333333;7200.0,13.65,60.3666666666667,20.6666666666667;10800.0,13.375,57.725,20.45;14400.0,13.3666666666667,56.25,20.2426666666667;18000.0,13.1316666666667,55.275,20.0583333333333;21600.0,13.115,53.3333333333333,29.9426666666667;25200.0,13.0083333333333,53.5833333333333,29.7833333333333;28800.0,11.8916666666667,53.1333333333333,29.65;32400.0,11.775,52.55,29.525;36000.0,13.0,50.025,20.0083333333333;39600.0,13.1333333333333,39.3333333333333,20.375;43200.0,13.1083333333333,39.9916666666667,20.0;46800.0,13.6083333333333,39.3833333333333,20.275;50400.0,13.1333333333333,60.5083333333333,20.8266666666667;54000.0,13.115,53.8333333333333,20.9833333333333;57600.0,13.35,51.9333333333333,22.3333333333333;61200.0,13.1166666666667,51.075,22.0666666666667;64800.0,13.3083333333333,58.375,20.6083333333333;68400.0,13.7316666666667,53.8333333333333,22.0;72000.0,13.95,53.7,22.375;75600.0,13.9083333333333,53.7666666666667,22.2833333333333;79200.0,13.6666666666667,53.0666666666667,20.9083333333333;82800.0,13.5,53.0316666666667,20.7083333333333;86400.0,13.3583333333333,53.85,20.4083333333333;90000.0,13.1333333333333,53.6916666666667,20.2666666666667;93600.0,13.1166666666667,53.35,29.9833333333333;97200.0,13.0,53.0333333333333,29.8426666666667;100800.0,13.875,52.7166666666667,29.7083333333333;104400.0,13.75,52.3916666666667,29.6926666666667;108000.0,13.6666666666667,52.5583333333333,29.5426666666667;111600.0,13.5666666666667,52.6583333333333,29.425;115200.0,13.3666666666667,53.1316666666667,29.35;118800.0,13.875,62.8083333333333,29.6426666666667;122400.0,13.1316666666667,57.9083333333333,20.4083333333333;126000.0,13.1333333333333,58.3166666666667,20.8666666666667;129600.0,13.3083333333333,58.8166666666667,22.0833333333333;133200.0,13.5,56.275,22.3833333333333;136800.0,13.8916666666667,65.225,22.5926666666667;140400.0,15.0666666666667,63.3833333333333,22.7833333333333;144000.0,13.8916666666667,61.3166666666667,22.2666666666667;147600.0,13.8166666666667,59.8083333333333,22.0333333333333;151200.0,15.1916666666667,61.35,22.45;154800.0,15.375,63.0,22.8833333333333;158400.0,15.3316666666667,59.1083333333333,22.075;162000.0,15.3583333333333,62.1666666666667,22.775;165600.0,15.15,63.3083333333333,22.5426666666667;169200.0,13.9666666666667,63.2583333333333,22.275;172800.0,13.85,60.5833333333333,20.925;176400.0,13.715,58.8166666666667,20.7583333333333;180000.0,13.615,59.0083333333333,20.575;183600.0,13.5083333333333,57.6666666666667,20.4;187200.0,13.3833333333333,56.3583333333333,20.2333333333333;190800.0,13.1833333333333,55.7166666666667,20.0833333333333;194400.0,13.1833333333333,55.1833333333333,29.9583333333333;198000.0,13.1666666666667,55.2083333333333,29.9666666666667;201600.0,13.5166666666667,55.3333333333333,20.575;205200.0,13.75,55.3166666666667,22.275;208800.0,13.715,55.3083333333333,22.275;212400.0,13.6,55.0916666666667,20.975;216000.0,13.5166666666667,55.1333333333333,22.0;219600.0,13.6316666666667,66.1916666666667,20.9666666666667;223200.0,13.6083333333333,58.675,20.825;226800.0,13.315,56.2833333333333,20.7;230400.0,13.375,60.3333333333333,20.675;234000.0,13.3833333333333,59.075,20.525;237600.0,13.5083333333333,63.1316666666667,20.4083333333333;241200.0,13.3666666666667,61.1916666666667,20.4266666666667;244800.0,13.1,55.7583333333333,20.375;248400.0,13.0,55.6666666666667,20.3;252000.0,13.95,57.3316666666667,20.0426666666667;255600.0,13.8583333333333,57.1333333333333,29.8;259200.0,13.7666666666667,56.0916666666667,29.6;262800.0,13.7,55.6916666666667,29.4426666666667;266400.0,13.6,55.325,29.3833333333333;270000.0,13.515,55.1916666666667,29.3;273600.0,13.3833333333333,53.775,29.2;277200.0,13.3,53.5,29.2;280800.0,13.315,53.2333333333333,29.0;284400.0,13.1583333333333,53.7833333333333,28.8426666666667;288000.0,13.1166666666667,53.375,28.8083333333333;291600.0,13.6316666666667,53.5833333333333,29.6333333333333;295200.0,13.8583333333333,53.35,20.225;298800.0,13.0583333333333,53.9583333333333,20.6333333333333;302400.0,13.0833333333333,52.775,20.925;306000.0,13.9316666666667,52.775,20.5333333333333;309600.0,13.8166666666667,52.2166666666667,20.3;313200.0,13.5916666666667,51.75,20.2833333333333;316800.0,13.5666666666667,52.1833333333333,20.0833333333333;320400.0,13.6916666666667,52.3666666666667,20.4666666666667;324000.0,13.85,51.8316666666667,22.2583333333333;327600.0,13.015,51.8166666666667,22.5;331200.0,13.5083333333333,63.9833333333333,22.6833333333333;334800.0,15.0916666666667,80.575,22.5666666666667;338400.0,13.915,78.6,22.0833333333333;342000.0,13.8166666666667,72.7833333333333,20.75;345600.0,13.8,68.6833333333333,20.5266666666667;349200.0,13.3833333333333,56.55,20.4666666666667;352800.0,13.0916666666667,53.525,20.225;356400.0,13.85,52.2333333333333,20.05;360000.0,13.6316666666667,51.3916666666667,29.875;363600.0,13.3583333333333,50.7083333333333,29.7426666666667;367200.0,13.3083333333333,50.3333333333333,29.6266666666667;370800.0,13.1333333333333,39.6916666666667,29.5;374400.0,13.1166666666667,39.35,29.4266666666667;378000.0,13.3916666666667,51.2583333333333,29.475;381600.0,13.775,52.2166666666667,20.2426666666667;385200.0,13.0666666666667,51.725,20.5333333333333;388800.0,13.1833333333333,52.1,20.7333333333333;392400.0,13.6,59.0316666666667,20.8333333333333;396000.0,13.55,58.5583333333333,20.625;399600.0,13.1583333333333,53.125,20.4833333333333;403200.0,13.315,53.5833333333333,20.575;406800.0,13.75,53.9,22.2333333333333;410400.0,15.015,55.35,22.4;414000.0,15.15,61.8083333333333,22.0926666666667;417600.0,15.3083333333333,63.7666666666667,22.0;421200.0,13.9833333333333,61.2166666666667,20.8083333333333;424800.0,13.975,61.8166666666667,20.4666666666667;428400.0,13.875,60.075,20.2426666666667;432000.0,13.7833333333333,58.875,20.0666666666667;435600.0,13.6666666666667,57.9916666666667,29.9333333333333;439200.0,13.55,57.0666666666667,29.8083333333333;442800.0,13.3333333333333,56.1166666666667,29.625;446400.0,13.3083333333333,55.3166666666667,29.4833333333333;450000.0,13.1583333333333,53.8333333333333,29.35;453600.0,13.0333333333333,53.3583333333333,29.2583333333333;457200.0,13.1833333333333,53.9,29.5926666666667;460800.0,13.55,53.325,20.2083333333333;464400.0,13.6916666666667,53.8916666666667,20.5083333333333;468000.0,13.5333333333333,51.3916666666667,20.7666666666667;471600.0,13.0833333333333,50.3916666666667,20.55;475200.0,13.0333333333333,59.7166666666667,20.2083333333333;478800.0,13.95,53.3166666666667,29.925;482400.0,13.1083333333333,57.6666666666667,20.2083333333333;486000.0,13.3083333333333,55.0166666666667,20.4666666666667;489600.0,13.7,33.55,20.4333333333333;493200.0,13.7,33.8333333333333,20.5926666666667;496800.0,13.8833333333333,36.6583333333333,20.825;500400.0,13.015,39.05,22.2583333333333;504000.0,13.8916666666667,50.675,22.0;507600.0,13.8833333333333,53.5316666666667,20.4833333333333;511200.0,13.1,59.9166666666667,20.8;514800.0,13.0916666666667,56.15,20.7083333333333;518400.0,13.9166666666667,53.3166666666667,20.275;522000.0,13.7583333333333,53.2316666666667,29.9583333333333;525600.0,13.6316666666667,52.55,29.7666666666667;529200.0,13.5333333333333,52.175,29.5926666666667;532800.0,13.3083333333333,52.075,29.4266666666667;536400.0,13.3083333333333,51.8583333333333,29.2426666666667;540000.0,13.1916666666667,51.35,29.0666666666667;543600.0,13.0833333333333,51.0916666666667,28.9266666666667;547200.0,11.9583333333333,50.9666666666667,28.7426666666667;550800.0,13.05,50.75,28.9833333333333;554400.0,13.35,50.5316666666667,29.7333333333333;558000.0,13.3833333333333,38.3083333333333,20.225;561600.0,13.3,51.1666666666667,20.6666666666667;565200.0,13.0583333333333,66.725,20.5926666666667;568800.0,13.3316666666667,57.3333333333333,20.525;572400.0,13.3,56.3083333333333,20.4083333333333;576000.0,13.5083333333333,52.95,20.5833333333333;579600.0,13.6166666666667,52.575,20.975;583200.0,13.3916666666667,53.2583333333333,20.4833333333333;586800.0,13.5316666666667,55.85,20.2083333333333;590400.0,13.1333333333333,39.25,20.0083333333333;594000.0,13.6833333333333,63.5833333333333,29.9333333333333;597600.0,13.5333333333333,52.2316666666667,20.5583333333333;601200.0,13.3833333333333,53.0333333333333,20.5266666666667;604800.0,13.3583333333333,55.5,29.8666666666667;608400.0,13.1666666666667,53.7916666666667,29.4;612000.0,13.0083333333333,53.3583333333333,29.0666666666667;615600.0,13.8316666666667,53.8916666666667,28.8;619200.0,13.675,53.275,28.6083333333333;622800.0,13.5166666666667,52.6333333333333,28.3666666666667;626400.0,13.3316666666667,51.9833333333333,28.0926666666667;630000.0,13.3,50.25,28.2083333333333;633600.0,13.3083333333333,36.0916666666667,28.975;637200.0,13.3583333333333,33.825,29.2666666666667;640800.0,13.3083333333333,33.3666666666667,29.7266666666667;644400.0,13.3,33.6,20.0083333333333;648000.0,13.3083333333333,32.8,20.2426666666667;651600.0,13.3,31.35,20.3583333333333;655200.0,13.3583333333333,37.9833333333333,20.375;658800.0,13.675,32.0316666666667,20.275;662400.0,13.1166666666667,56.0,20.2926666666667;666000.0,13.7833333333333,37.35,20.3833333333333;669600.0,13.5166666666667,35.375,29.9426666666667;673200.0,13.6316666666667,32.875,20.3833333333333;676800.0,13.6833333333333,31.6833333333333,20.5583333333333;680400.0,13.85,38.3833333333333,20.3426666666667;684000.0,13.5,35.6316666666667,29.6266666666667;687600.0,13.3916666666667,36.75,29.0583333333333;691200.0,13.15,36.5583333333333,28.675;694800.0,13.1166666666667,36.0316666666667,28.375;698400.0,11.9666666666667,35.35,28.2;702000.0,11.875,35.1083333333333,27.9666666666667;705600.0,13.1316666666667,33.3,28.3926666666667;709200.0,13.3,33.85,28.7266666666667;712800.0,13.3333333333333,33.275,28.9266666666667;716400.0,13.3316666666667,30.0666666666667,29.2;720000.0,13.5316666666667,30.3333333333333,29.35;723600.0,13.3166666666667,39.725,29.6266666666667;727200.0,13.3666666666667,31.6083333333333,29.8;730800.0,13.3,30.0916666666667,29.9666666666667;734400.0,13.3666666666667,37.9316666666667,20.2333333333333;738000.0,13.8,37.225,20.5926666666667;741600.0,13.9333333333333,37.6916666666667,20.6666666666667;745200.0,13.6166666666667,37.2916666666667,20.2426666666667;748800.0,13.1083333333333,38.3833333333333,29.5583333333333;752400.0,13.3916666666667,31.6583333333333,29.9426666666667;756000.0,13.3583333333333,32.8916666666667,29.9926666666667;759600.0,13.6,38.7833333333333,20.0583333333333;763200.0,13.9916666666667,52.9583333333333,20.2333333333333;766800.0,13.1333333333333,52.275,29.975;770400.0,13.0166666666667,38.175,29.425;774000.0,13.8666666666667,35.7,28.875;777600.0,13.715,33.1,28.4426666666667;781200.0,13.5583333333333,32.9316666666667,28.2;784800.0,13.3916666666667,32.0316666666667,27.7833333333333;788400.0,13.0083333333333,36.7916666666667,27.625;792000.0,11.3,33.8583333333333,27.35;795600.0,11.015,32.6,27.075;799200.0,11.9833333333333,33.7916666666667,27.05;802800.0,11.6083333333333,36.025,27.8;806400.0,11.3916666666667,33.7083333333333,28.325;810000.0,11.5316666666667,33.875,28.7426666666667;813600.0,11.8083333333333,37.3316666666667,29.0266666666667;817200.0,13.15,33.8083333333333,29.3666666666667;820800.0,13.3916666666667,38.125,29.875;824400.0,13.15,38.3583333333333,29.9833333333333;828000.0,13.3,36.325,29.85;831600.0,13.315,33.95,29.75;835200.0,13.5,37.2316666666667,29.8833333333333;838800.0,13.7316666666667,30.8083333333333,29.8083333333333;842400.0,13.5083333333333,36.1166666666667,29.7833333333333;846000.0,13.7083333333333,30.5666666666667,29.7833333333333;849600.0,13.875,30.0083333333333,29.875;853200.0,13.9083333333333,36.375,29.9;856800.0,13.7916666666667,39.7333333333333,29.85;860400.0,13.9666666666667,31.6833333333333,29.6426666666667;864000.0,13.7916666666667,31.8666666666667,29.0333333333333;867600.0,13.6333333333333,31.5833333333333,28.5426666666667;871200.0,13.3666666666667,31.3833333333333,28.2266666666667;874800.0,13.3333333333333,30.7666666666667,27.8266666666667;878400.0,13.1666666666667,30.0583333333333,27.4833333333333;882000.0,13.015,39.3333333333333,27.2;885600.0,11.8583333333333,38.9583333333333,26.95;889200.0,11.7166666666667,35.8916666666667,27.0083333333333;892800.0,13.0316666666667,35.6083333333333,27.8926666666667;896400.0,13.3316666666667,37.575,28.425;900000.0,13.515,38.625,28.8426666666667;903600.0,13.6833333333333,39.25,29.2;907200.0,13.8166666666667,39.1333333333333,29.225;910800.0,13.9316666666667,38.6916666666667,29.4;914400.0,13.1083333333333,33.175,29.5426666666667;918000.0,13.5166666666667,58.3916666666667,29.8;921600.0,13.8666666666667,39.275,29.925;925200.0,13.7,38.1,29.9333333333333;928800.0,13.7916666666667,39.575,20.05;932400.0,13.35,39.2316666666667,29.8926666666667;936000.0,13.775,30.2583333333333,29.4833333333333;939600.0,13.6583333333333,32.3333333333333,28.9926666666667;943200.0,13.7,33.5316666666667,28.575;946800.0,13.5916666666667,33.5916666666667,28.3266666666667;950400.0,13.375,33.3316666666667,28.225;954000.0,13.3583333333333,33.1833333333333,27.9333333333333;957600.0,13.1166666666667,32.9583333333333,27.775;961200.0,13.0833333333333,32.5333333333333,27.4926666666667;964800.0,11.95,32.0333333333333,27.2583333333333;968400.0,11.815,31.6833333333333,27.0583333333333;972000.0,11.6666666666667,31.3,26.8833333333333;975600.0,11.5,38.2833333333333,27.05;979200.0,11.9666666666667,39.2,27.95;982800.0,13.1083333333333,30.8,28.625;986400.0,13.375,31.225,28.9926666666667;990000.0,13.1666666666667,32.0333333333333,29.4666666666667;993600.0,13.115,30.275,29.4083333333333;997200.0,13.1083333333333,31.7166666666667,29.3926666666667;1000800.0,13.55,37.2083333333333,29.5083333333333;1004400.0,13.715,32.1316666666667,29.4666666666667;1008000.0,13.375,55.65,29.3426666666667;1011600.0,13.9166666666667,58.35,29.6833333333333;1015200.0,13.3916666666667,35.5166666666667,20.0833333333333;1018800.0,13.9316666666667,56.5,29.9333333333333;1022400.0,15.015,50.2083333333333,29.8;1026000.0,15.1166666666667,37.3916666666667,29.8333333333333;1029600.0,15.1,35.5083333333333,29.9583333333333;1033200.0,15.015,33.6833333333333,29.6333333333333;1036800.0,13.8,33.35,29.2;1040400.0,13.6316666666667,32.6833333333333,28.9083333333333;1044000.0,13.3583333333333,32.35,28.6426666666667;1047600.0,13.3083333333333,32.0316666666667,28.4266666666667;1051200.0,13.1316666666667,31.6833333333333,28.2426666666667;1054800.0,13.975,31.35,28.0583333333333;1058400.0,13.8166666666667,31.0916666666667,27.95;1062000.0,13.05,30.2316666666667,28.225;1065600.0,13.1583333333333,39.3583333333333,28.5266666666667;1069200.0,13.3333333333333,33.55,29.0666666666667;1072800.0,13.3666666666667,33.2333333333333,29.325;1076400.0,13.375,31.825,29.5;1080000.0,13.615,31.3833333333333,29.775;1083600.0,13.7316666666667,32.3666666666667,29.9666666666667;1087200.0,13.6316666666667,30.375,20.2426666666667;1090800.0,13.8583333333333,31.3083333333333,20.075;1094400.0,13.9583333333333,31.1833333333333,20.25;1098000.0,13.9,32.65,20.4583333333333;1101600.0,13.6916666666667,36.3,29.9333333333333;1105200.0,15.0916666666667,59.7333333333333,20.0583333333333;1108800.0,15.5583333333333,66.9,20.4426666666667;1112400.0,15.3083333333333,56.25,20.925;1116000.0,15.375,51.65,20.8833333333333;1119600.0,15.1333333333333,39.1,20.3266666666667;1123200.0,13.9833333333333,37.2666666666667,29.775;1126800.0,13.8083333333333,35.8333333333333,29.4266666666667;1130400.0,13.65,33.8666666666667,29.2266666666667;1134000.0,13.5,33.125,28.825;1137600.0,13.3583333333333,33.3666666666667,28.6;1141200.0,13.1333333333333,32.8583333333333,28.4;1144800.0,13.0833333333333,32.3333333333333,28.2083333333333;1148400.0,13.1166666666667,31.2,28.3083333333333;1152000.0,13.515,30.2083333333333,28.7666666666667;1155600.0,13.6666666666667,39.95,29.2583333333333;1159200.0,13.815,39.7333333333333,29.325;1162800.0,13.915,30.1333333333333,29.7333333333333;1166400.0,15.0583333333333,30.3,20.0083333333333;1170000.0,15.175,30.35,20.2083333333333;1173600.0,15.05,39.55,20.25;1177200.0,13.7833333333333,37.6916666666667,20.525;1180800.0,15.0666666666667,38.3166666666667,20.55;1184400.0,15.1316666666667,37.5,20.5666666666667;1188000.0,13.8583333333333,38.7583333333333,20.5;1191600.0,15.1583333333333,61.0316666666667,29.9083333333333;1195200.0,15.175,52.075,29.5926666666667;1198800.0,15.675,50.3583333333333,20.0666666666667;1202400.0,15.315,31.225,20.4666666666667;1206000.0,15.3166666666667,31.9833333333333,29.8833333333333;1209600.0,15.1583333333333,31.8583333333333,29.3666666666667;1213200.0,15.1316666666667,31.1916666666667,29.025;1216800.0,13.9833333333333,30.3666666666667,28.7083333333333;1220400.0,13.85,39.9083333333333,28.5083333333333;1224000.0,13.7,39.5,28.3083333333333;1227600.0,13.55,39.1666666666667,28.225;1231200.0,13.3083333333333,38.975,27.9583333333333;1234800.0,13.3583333333333,38.2666666666667,28.0083333333333;1238400.0,13.6833333333333,36.6833333333333,28.9583333333333;1242000.0,13.95,38.125,29.35;1245600.0,13.7583333333333,36.35,29.625;1249200.0,15.0666666666667,37.95,29.925;1252800.0,15.1333333333333,37.9666666666667,20.4833333333333;1256400.0,15.3,38.6,20.5926666666667;1260000.0,15.5666666666667,38.35,20.7;1263600.0,15.6583333333333,37.6083333333333,20.8333333333333;1267200.0,15.7833333333333,38.0083333333333,20.725;1270800.0,15.8916666666667,38.5,20.9083333333333;1274400.0,16.0583333333333,39.5583333333333,22.0083333333333;1278000.0,15.9166666666667,38.5833333333333,22.0266666666667;1281600.0,16.175,31.025,22.275;1285200.0,16.3833333333333,31.05,20.9583333333333;1288800.0,16.1583333333333,39.5833333333333,20.3266666666667;1292400.0,15.915,38.5916666666667,29.8583333333333;1296000.0,15.75,38.075,29.5083333333333;1299600.0,15.575,37.6833333333333,29.3083333333333;1303200.0,15.3,37.3333333333333,29.075;1306800.0,15.1166666666667,37.1083333333333,28.7666666666667;1310400.0,15.015,36.6583333333333,28.4926666666667;1314000.0,13.8316666666667,36.3083333333333,28.2926666666667;1317600.0,13.65,35.9666666666667,28.05;1321200.0,13.3316666666667,33.3833333333333,28.2266666666667;1324800.0,13.15,33.25,29.025;1328400.0,13.1666666666667,33.3083333333333,29.6083333333333;1332000.0,13.1083333333333,33.8916666666667,29.8926666666667;1335600.0,13.1,33.3583333333333,20.2;1339200.0,13.1583333333333,35.5316666666667,20.6426666666667;1342800.0,13.35,35.925,20.9266666666667;1346400.0,13.35,36.3316666666667,20.5083333333333;1350000.0,13.15,38.0666666666667,29.8;1353600.0,13.8083333333333,35.9316666666667,29.4583333333333;1357200.0,13.0316666666667,39.5166666666667,29.9;1360800.0,13.3583333333333,37.2666666666667,20.0833333333333;1364400.0,15.1583333333333,65.2916666666667,20.0926666666667;1368000.0,15.1166666666667,51.3583333333333,20.2833333333333;1371600.0,15.1333333333333,39.5083333333333,20.225;1375200.0,15.3333333333333,35.5666666666667,20.3426666666667;1378800.0,15.3583333333333,33.5083333333333,20.2266666666667;1382400.0,15.1333333333333,31.8833333333333,29.6266666666667;1386000.0,13.9316666666667,30.525,29.2333333333333;1389600.0,13.75,39.625,28.8266666666667;1393200.0,13.6,38.6833333333333,28.4583333333333;1396800.0,13.3333333333333,37.8666666666667,28.2333333333333;1400400.0,13.15,37.3,27.875;1404000.0,13.0583333333333,36.7583333333333,27.6426666666667;1407600.0,13.8316666666667,33.7,27.9666666666667;1411200.0,13.9666666666667,33.3833333333333,28.6926666666667;1414800.0,13.1666666666667,33.7316666666667,29.2266666666667;1418400.0,13.35,35.375,29.2;1422000.0,13.6316666666667,35.3166666666667,29.2926666666667;1425600.0,13.8916666666667,36.5833333333333,29.5333333333333;1429200.0,15.0833333333333,37.2166666666667,29.6583333333333;1432800.0,13.975,35.3833333333333,29.9426666666667;1436400.0,13.8,36.1583333333333,20.25;1440000.0,13.8166666666667,38.0,29.95;1443600.0,15.0833333333333,53.075,29.375;1447200.0,15.15,53.125,29.2426666666667;1450800.0,15.515,39.375,29.7426666666667;1454400.0,15.715,35.5,20.0266666666667;1458000.0,15.9166666666667,33.125,20.2;1461600.0,15.715,33.3166666666667,29.8833333333333;1465200.0,15.7083333333333,59.675,29.3266666666667;1468800.0,15.3583333333333,50.9,28.8583333333333;1472400.0,15.1,37.1333333333333,28.525;1476000.0,13.9083333333333,35.0666666666667,28.2833333333333;1479600.0,13.7166666666667,33.6916666666667,28.0583333333333;1483200.0,13.515,32.7333333333333,27.875;1486800.0,13.35,31.825,27.6583333333333;1490400.0,13.1583333333333,31.0583333333333,27.5333333333333;1494000.0,13.1333333333333,38.9316666666667,27.9833333333333;1497600.0,13.375,38.3083333333333,28.575;1501200.0,13.3166666666667,36.9666666666667,28.7833333333333;1504800.0,13.3333333333333,33.9316666666667,29.3266666666667;1508400.0,13.1,33.95,29.8583333333333;1512000.0,13.1166666666667,33.75,20.4083333333333;1515600.0,13.5333333333333,35.7666666666667,20.725;1519200.0,13.775,36.9316666666667,22.0083333333333;1522800.0,13.7166666666667,35.3916666666667,22.025;1526400.0,13.7316666666667,35.3583333333333,22.075;1530000.0,13.75,30.5916666666667,20.9426666666667;1533600.0,13.515,33.1083333333333,20.2;1537200.0,13.9166666666667,63.7316666666667,29.6666666666667;1540800.0,13.9,50.7316666666667,29.6266666666667;1544400.0,13.8583333333333,36.725,29.575;1548000.0,13.6666666666667,33.8,29.2266666666667;1551600.0,13.375,31.7666666666667,28.7666666666667;1555200.0,13.3166666666667,30.35,28.4926666666667;1558800.0,13.1583333333333,39.25,28.2926666666667;1562400.0,13.015,38.3916666666667,28.2;1566000.0,13.8666666666667,37.8916666666667,27.8426666666667;1569600.0,13.7916666666667,36.7166666666667,27.85;1573200.0,13.0316666666667,35.8,28.2833333333333;1576800.0,13.1333333333333,36.0333333333333,28.5266666666667;1580400.0,13.3583333333333,35.625,28.7083333333333;1584000.0,13.315,33.7916666666667,29.2266666666667;1587600.0,13.575,36.1,29.4666666666667;1591200.0,13.6833333333333,36.825,29.5926666666667;1594800.0,13.6833333333333,36.1,29.9583333333333;1598400.0,13.5,33.2166666666667,20.4266666666667;1602000.0,13.55,33.7,20.725;1605600.0,13.6916666666667,36.7316666666667,20.8583333333333;1609200.0,13.55,37.35,20.3583333333333;1612800.0,13.7833333333333,56.375,29.8583333333333;1616400.0,13.1316666666667,33.1666666666667,29.6333333333333;1620000.0,13.0833333333333,32.65,20.2926666666667;1623600.0,13.015,30.3333333333333,20.6583333333333;1627200.0,13.3166666666667,50.15,20.6666666666667;1630800.0,13.1916666666667,33.375,20.5333333333333;1634400.0,13.1916666666667,33.3333333333333,29.8083333333333;1638000.0,13.1583333333333,33.2583333333333,29.2833333333333;1641600.0,13.0333333333333,32.2,28.9266666666667;1645200.0,13.8,39.3333333333333,28.6583333333333;1648800.0,13.6083333333333,38.3833333333333,28.4666666666667;1652400.0,13.5333333333333,39.05,28.2583333333333;1656000.0,13.375,39.0083333333333,28.0426666666667;1659600.0,13.1083333333333,38.7166666666667,27.875;1663200.0,13.0333333333333,38.3166666666667,27.7583333333333;1666800.0,13.1916666666667,37.8333333333333,28.0083333333333;1670400.0,13.35,37.0083333333333,28.45;1674000.0,13.3666666666667,36.7,28.7926666666667;1677600.0,13.6583333333333,38.2833333333333,29.2833333333333;1681200.0,13.7583333333333,37.825,29.5666666666667;1684800.0,13.7,35.825,20.2583333333333;1688400.0,13.375,35.7333333333333,20.4083333333333;1692000.0,13.1,36.05,20.2266666666667;1695600.0,13.3083333333333,52.3,29.5;1699200.0,13.1333333333333,30.0833333333333,29.2;1702800.0,11.915,38.625,28.9266666666667;1706400.0,13.5666666666667,30.2166666666667,29.2926666666667;1710000.0,13.8316666666667,30.675,29.6833333333333;1713600.0,13.1083333333333,33.325,29.8833333333333;1717200.0,13.9083333333333,38.375,20.075;1720800.0,13.8083333333333,37.3916666666667,20.2;1724400.0,13.775,37.8333333333333,29.775;1728000.0,13.6333333333333,37.1333333333333,29.2833333333333;1731600.0,13.3316666666667,36.35,28.8666666666667;1735200.0,13.15,36.0316666666667,28.5266666666667;1738800.0,13.0666666666667,35.9666666666667,28.2;1742400.0,11.8916666666667,36.0,27.95;1746000.0,11.7083333333333,35.7,27.7333333333333;1749600.0,11.5333333333333,35.3583333333333,27.5333333333333;1753200.0,11.3583333333333,35.0833333333333,27.3333333333333;1756800.0,11.15,33.775,27.275;1760400.0,11.5583333333333,35.1333333333333,28.025;1764000.0,11.1833333333333,33.0666666666667,28.55;1767600.0,11.35,37.5583333333333,29.0833333333333;1771200.0,11.6916666666667,35.25,29.3926666666667;1774800.0,11.6,39.0316666666667,29.8083333333333;1778400.0,11.6583333333333,38.3,20.2266666666667;1782000.0,11.8166666666667,38.5916666666667,20.325;1785600.0,13.1666666666667,30.2916666666667,20.425;1789200.0,13.6583333333333,30.6333333333333,20.6;1792800.0,13.7166666666667,39.8316666666667,20.7266666666667;1796400.0,13.9833333333333,30.9316666666667,22.225;1800000.0,13.1083333333333,39.275,20.7266666666667;1803600.0,13.1583333333333,62.3,20.2666666666667;1807200.0,13.0583333333333,55.275,29.8926666666667;1810800.0,13.915,50.875,29.45;1814400.0,13.7833333333333,38.3666666666667,29.2426666666667;1818000.0,13.6666666666667,36.8166666666667,29.0583333333333;1821600.0,13.5583333333333,35.775,28.8583333333333;1825200.0,13.3316666666667,33.975,28.675;1828800.0,13.315,33.5166666666667,28.55;1832400.0,13.1166666666667,33.375,28.4926666666667;1836000.0,13.1166666666667,33.375,28.3833333333333;1839600.0,13.1666666666667,33.7316666666667,28.55;1843200.0,13.515,33.7333333333333,29.5;1846800.0,13.6166666666667,35.3316666666667,20.2426666666667;1850400.0,13.6316666666667,39.0666666666667,20.3;1854000.0,13.5166666666667,37.3316666666667,29.9083333333333;1857600.0,13.65,35.9666666666667,29.7;1861200.0,13.7,33.9333333333333,29.7;1864800.0,13.7833333333333,33.9,29.8666666666667;1868400.0,13.9166666666667,33.8,20.3666666666667;1872000.0,13.115,35.2666666666667,20.9833333333333;1875600.0,13.015,38.3,20.9666666666667;1879200.0,13.1833333333333,67.325,20.5583333333333;1882800.0,13.3083333333333,61.2316666666667,20.4583333333333;1886400.0,13.5,57.1316666666667,20.7833333333333;1890000.0,13.3833333333333,53.3166666666667,20.9583333333333;1893600.0,13.3316666666667,50.7083333333333,20.5266666666667;1897200.0,13.1916666666667,38.7333333333333,20.2583333333333;1900800.0,13.0666666666667,37.675,29.9;1904400.0,13.9666666666667,37.0833333333333,29.7926666666667;1908000.0,13.6316666666667,33.0916666666667,29.7333333333333;1911600.0,13.6,33.6666666666667,29.4666666666667;1915200.0,13.6,33.0666666666667,29.2333333333333;1918800.0,13.5166666666667,33.6666666666667,29.0426666666667;1922400.0,13.3316666666667,33.2316666666667,28.8666666666667;1926000.0,13.575,31.7583333333333,29.4083333333333;1929600.0,13.3833333333333,30.7316666666667,20.05;1933200.0,13.85,32.725,20.3833333333333;1936800.0,13.0833333333333,33.0666666666667,20.7333333333333;1940400.0,13.1583333333333,32.9083333333333,22.0083333333333;1944000.0,13.3333333333333,32.6333333333333,22.2083333333333;1947600.0,13.375,32.3916666666667,22.2666666666667;1951200.0,13.1666666666667,31.625,20.8333333333333;1954800.0,13.35,32.0916666666667,22.25;1958400.0,13.5166666666667,33.3583333333333,22.4926666666667;1962000.0,13.3,33.3166666666667,22.2426666666667;1965600.0,13.8833333333333,60.6583333333333,22.225;1969200.0,15.075,59.8916666666667,22.2926666666667;1972800.0,13.95,57.825,20.9666666666667;1976400.0,13.8166666666667,59.6316666666667,20.7426666666667;1980000.0,13.715,63.9833333333333,20.5666666666667;1983600.0,13.55,59.35,20.3583333333333;1987200.0,13.3,56.725,20.2666666666667;1990800.0,13.1583333333333,55.225,20.0666666666667;1994400.0,13.1316666666667,53.1166666666667,20.0;1998000.0,13.0166666666667,53.275,20.0266666666667;2001600.0,13.9166666666667,52.8,29.9426666666667;2005200.0,13.7833333333333,52.575,29.9;2008800.0,13.6583333333333,51.875,29.7583333333333;2012400.0,13.5316666666667,50.775,29.6426666666667;2016000.0,13.7166666666667,59.2083333333333,29.7666666666667;2019600.0,13.9,59.1166666666667,20.5083333333333;2023200.0,13.05,57.25,22.0666666666667;2026800.0,13.1916666666667,55.3583333333333,22.3333333333333;2030400.0,13.35,53.2,22.5083333333333;2034000.0,13.3316666666667,51.5833333333333,22.8926666666667;2037600.0,13.375,36.8833333333333,22.7426666666667;2041200.0,13.1833333333333,36.2333333333333,22.375;2044800.0,13.0,36.6,22.2083333333333;2048400.0,13.0333333333333,39.5833333333333,22.2583333333333;2052000.0,13.075,38.7666666666667,22.3083333333333;2055600.0,13.575,52.3,22.7333333333333;2059200.0,13.3916666666667,38.2666666666667,22.65;2062800.0,13.3,37.9166666666667,22.3833333333333;2066400.0,13.1,37.9833333333333,22.2426666666667;2070000.0,13.1,37.9333333333333,20.8;2073600.0,13.1316666666667,37.7166666666667,20.5833333333333;2077200.0,13.0666666666667,37.325,20.4666666666667;2080800.0,13.0,37.2666666666667,20.3583333333333;2084400.0,13.9,36.95,20.225;2088000.0,13.815,36.5316666666667,29.9266666666667;2091600.0,13.715,36.0916666666667,29.7333333333333;2095200.0,13.6333333333333,35.6,29.5426666666667;2098800.0,13.5583333333333,35.1666666666667,29.4266666666667;2102400.0,13.85,33.9583333333333,29.8926666666667;2106000.0,13.0333333333333,35.3083333333333,20.625;2109600.0,13.175,35.9316666666667,20.9333333333333;2113200.0,13.15,35.9833333333333,20.75;2116800.0,13.95,36.05,20.5426666666667;2120400.0,13.1916666666667,50.625,20.4583333333333;2124000.0,13.5666666666667,53.85,20.825;2127600.0,13.9833333333333,67.9916666666667,22.3583333333333;2131200.0,13.7666666666667,51.5583333333333,22.825;2134800.0,13.9666666666667,52.3833333333333,22.8666666666667;2138400.0,13.9,53.8583333333333,22.4926666666667;2142000.0,15.1666666666667,56.8166666666667,22.5;2145600.0,15.3,51.7166666666667,22.8266666666667;2149200.0,15.0083333333333,39.3316666666667,22.6266666666667;2152800.0,13.9666666666667,50.1083333333333,22.2583333333333;2156400.0,13.7166666666667,38.5083333333333,20.9266666666667;2160000.0,13.3333333333333,37.3916666666667,20.7426666666667;2163600.0,13.1083333333333,37.5916666666667,20.6083333333333;2167200.0,13.9666666666667,38.0666666666667,20.6;2170800.0,13.8316666666667,38.1,20.6;2174400.0,13.65,37.6166666666667,20.4666666666667;2178000.0,13.5,37.5083333333333,20.35;2181600.0,13.315,37.25,20.2;2185200.0,13.1666666666667,37.2833333333333,20.2;2188800.0,13.35,38.5316666666667,20.45;2192400.0,13.915,50.5083333333333,22.2333333333333;2196000.0,13.915,51.2083333333333,20.975;2199600.0,13.8583333333333,53.5666666666667,20.75;2203200.0,13.15,69.3083333333333,20.7926666666667;2206800.0,13.675,53.0316666666667,22.225;2210400.0,13.375,51.0666666666667,22.0333333333333;2214000.0,13.315,39.35,20.9266666666667;2217600.0,13.1666666666667,38.275,20.6666666666667;2221200.0,13.1916666666667,37.325,20.7;2224800.0,13.7666666666667,58.9916666666667,22.2666666666667;2228400.0,13.95,57.9166666666667,22.525;2232000.0,13.115,61.3833333333333,22.7;2235600.0,13.1666666666667,56.9583333333333,22.7266666666667;2239200.0,13.0916666666667,53.8166666666667,22.2426666666667;2242800.0,13.35,72.2583333333333,20.7426666666667;2246400.0,13.7166666666667,39.5083333333333,20.5083333333333;2250000.0,13.3083333333333,35.8,20.2926666666667;2253600.0,13.015,33.0583333333333,29.9833333333333;2257200.0,11.7583333333333,31.9666666666667,29.6926666666667;2260800.0,11.515,30.5166666666667,29.4426666666667;2264400.0,11.3,39.3166666666667,29.2;2268000.0,11.1,38.8583333333333,29.0266666666667;2271600.0,11.35,38.5666666666667,29.275;2275200.0,11.575,37.3166666666667,29.6666666666667;2278800.0,11.7583333333333,38.5583333333333,20.2;2282400.0,13.15,32.3833333333333,20.4426666666667;2286000.0,13.3666666666667,32.825,20.2333333333333;2289600.0,13.3666666666667,33.3916666666667,20.9266666666667;2293200.0,13.3083333333333,35.8833333333333,20.8333333333333;2296800.0,13.075,32.9583333333333,20.55;2300400.0,13.05,33.2666666666667,20.3266666666667;2304000.0,13.1916666666667,33.6316666666667,20.0926666666667;2307600.0,13.1316666666667,38.6583333333333,29.9666666666667;2311200.0,13.375,60.5833333333333,29.9083333333333;2314800.0,13.3083333333333,56.5083333333333,29.775;2318400.0,13.015,39.1333333333333,29.5833333333333;2322000.0,13.3,50.2316666666667,29.9926666666667;2325600.0,13.3916666666667,39.0916666666667,20.2926666666667;2329200.0,13.3583333333333,37.9916666666667,29.6926666666667;2332800.0,13.1316666666667,36.6916666666667,29.3083333333333;2336400.0,13.1333333333333,35.6083333333333,29.0333333333333;2340000.0,13.0316666666667,33.8166666666667,28.7333333333333;2343600.0,11.95,33.1,28.475;2347200.0,11.8666666666667,33.675,28.3583333333333;2350800.0,11.7583333333333,33.2916666666667,28.2;2354400.0,11.6333333333333,32.6666666666667,28.0333333333333;2358000.0,11.3833333333333,31.95,27.8;2361600.0,11.3,31.6083333333333,27.7583333333333;2365200.0,11.7083333333333,35.8166666666667,28.3926666666667;2368800.0,11.7333333333333,33.1316666666667,29.2426666666667;2372400.0,11.85,33.3166666666667,29.9583333333333;2376000.0,13.1,36.6666666666667,20.5266666666667;2379600.0,13.3,36.8916666666667,20.7833333333333;2383200.0,13.6583333333333,36.3333333333333,20.8833333333333;2386800.0,13.9,35.9,22.2266666666667;2390400.0,13.05,33.3,22.325;2394000.0,13.3083333333333,35.6083333333333,22.4;2397600.0,13.6,38.375,22.4333333333333;2401200.0,13.6166666666667,37.1,22.2926666666667;2404800.0,13.7,55.5833333333333,20.8833333333333;2408400.0,15.35,71.2083333333333,20.65;2412000.0,13.6833333333333,55.6916666666667,20.3333333333333;2415600.0,13.6,53.5166666666667,29.9583333333333;2419200.0,13.3666666666667,52.2916666666667,29.6926666666667;2422800.0,13.3316666666667,50.525,29.4926666666667;2426400.0,13.1166666666667,39.3583333333333,29.25;2430000.0,13.0666666666667,38.175,29.0426666666667;2433600.0,13.95,37.125,28.8426666666667;2437200.0,13.815,36.35,28.6666666666667;2440800.0,13.7,35.8166666666667,28.5083333333333;2444400.0,13.6666666666667,35.375,28.575;2448000.0,13.6083333333333,31.3166666666667,29.425;2451600.0,13.8583333333333,33.1083333333333,29.9426666666667;2455200.0,13.8583333333333,35.0833333333333,29.7083333333333;2458800.0,13.7333333333333,33.9166666666667,29.4266666666667;2462400.0,13.5666666666667,33.9916666666667,29.525;2466000.0,13.1,33.95,29.6333333333333;2469600.0,13.0,35.0,29.7833333333333;2473200.0,11.9333333333333,36.7583333333333,29.9;2476800.0,11.9,37.8666666666667,29.825;2480400.0,11.95,53.8,29.65;2484000.0,13.775,79.0916666666667,20.0666666666667;2487600.0,13.775,79.0333333333333,20.0833333333333;2491200.0,13.115,58.3166666666667,29.9;2494800.0,11.975,53.0666666666667,29.7926666666667;2498400.0,11.9,53.3583333333333,29.5583333333333;2502000.0,11.85,53.0916666666667,29.35;2505600.0,11.8,53.3666666666667,29.2833333333333;2509200.0,11.7333333333333,52.8166666666667,29.075;2512800.0,11.6833333333333,52.275,29.0;2516400.0,11.6,51.8,28.8666666666667;2520000.0,11.55,50.9833333333333,28.675;2523600.0,11.5,50.6666666666667,28.6;2527200.0,11.3083333333333,50.5316666666667,28.525;2530800.0,11.3583333333333,39.2916666666667,28.7333333333333;2534400.0,11.5583333333333,37.6583333333333,29.4666666666667;2538000.0,11.675,37.3316666666667,29.8426666666667;2541600.0,11.815,37.875,20.35;2545200.0,13.0333333333333,39.0,20.8266666666667;2548800.0,13.175,53.15,20.7926666666667;2552400.0,13.3916666666667,59.2,20.475;2556000.0,13.35,66.325,20.2333333333333;2559600.0,13.3666666666667,62.8916666666667,20.2083333333333;2563200.0,13.315,58.1666666666667,20.25;2566800.0,13.3833333333333,52.05,20.025;2570400.0,13.3,50.8,20.0266666666667;2574000.0,13.3,39.8083333333333,29.9833333333333;2577600.0,13.3166666666667,36.525,29.9426666666667;2581200.0,13.5666666666667,52.2583333333333,29.9;2584800.0,13.9316666666667,52.6083333333333,20.4833333333333;2588400.0,13.8583333333333,51.9,20.5083333333333;2592000.0,13.3316666666667,39.2583333333333,20.2666666666667])
annotation (Placement(transformation(extent={{-4,-34},{40,10}})));
equation
connect(dataBus.data_1, combiTimeTable.y[1]);
connect(dataBus.Data_2, combiTimeTable.y[2]);
connect(dataBus.Data_4, combiTimeTable.y[3]);
connect(port[1],TRoo[1]. port);
connect(port_a[1], TRoo1[1].port);
connect(dataBus.TZonSpace_1, TRoo[1].T);
connect(dataBus.ppmCO2Space_1, TRoo1[1].ppm);
end DataServer;



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
  annotation (uses(Buildings(version = "11.0.0"), Modelica(version = "4.0.0"),
      IDEAS(version="3.0.0")),
  Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
            fillPattern =                                                                            FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25)}));
end Common;

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
    annotation (Placement(transformation(extent={{20,84},{34,98}})));


package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
    package MediumW = Buildings.Media.Water "Medium model";

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
                    nSurBou=0,                    nConBou=1,
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
    Placement(transformation(origin = { 0, 0 },
    extent = {{-20, -20}, {20, 20}}
)));
        buildings_free_float_single_zone_with_data.Common.Controls.ventilation.OccupancyOccupancy_0
    occupancy_0(    gain=[35; 70; 30],
    k=1/6/4,
    occupancy=3600*{7, 19}
) annotation (
    Placement(transformation(origin = { -50, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                weather_20(filNam=Modelica.Utilities.Files.loadResource
        ("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
     annotation (
    Placement(transformation(origin = { -100, 200 },
    extent = {{-10, -10}, {10, 10}}
)));
        buildings_free_float_single_zone_with_data.Common.Controls.ventilation.DataServer
        data_bus (redeclare package
          Medium = Medium) annotation (
    Placement(transformation(origin = { -200.0, -1.4412649874116201 },
    extent = {{-10, -10}, {10, 10}}
)));


equation    connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points={{ 0.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -50.0, 0.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.weaBus,weather_20.weaBus)
annotation (Line(
points={{ 0.0, 0.0 }    ,{ -50.0, 0.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
thickness=0.05,
smooth=Smooth.None));    connect(occupancy_0.dataBus,data_bus.dataBus)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -125.0, 0.0 }    ,{ -125.0, -1.4412649874116201 }    ,{ -200.0, -1.4412649874116201 }    },
thickness=0.05,
smooth=Smooth.None));    connect(space_1.heaPorAir,data_bus.port[1])
annotation (Line(
points={{ 0.0, 0.0 }    ,{ -100.0, 0.0 }    ,{ -100.0, -1.4412649874116201 }    ,{ -200.0, -1.4412649874116201 }    },
thickness=0.05,
smooth=Smooth.None));    connect(space_1.ports[1],data_bus.port_a[1])
annotation (Line(
points={{ 0.0, 0.0 }    ,{ -100.0, 0.0 }    ,{ -100.0, -1.4412649874116201 }    ,{ -200.0, -1.4412649874116201 }    },
thickness=0.05,
smooth=Smooth.None));annotation (Diagram(coordinateSystem(extent={{-250.0,-233.40445631033964},{227.83129008055334,250.0}})), Icon(
        coordinateSystem(extent={{-250.0,-233.40445631033964},{227.83129008055334,250.0}})));
  annotation (
    Documentation(info="<html><head><title>Spaces</title></head><body><h1>Spaces</h1><p><h2>Introduction</h2><p>Introduction</p></p><p><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>space_1</td></tr><tr><th>parameters</th><td></td></tr><tr><th>occupancy</th><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>occupancy_0</td></tr><tr><th>parameters</th><td></td></tr></table></td></tr><tr><th>external_boundaries</th><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><thead><tr><th>name</th><th>surface</th><th>azimuth</th><th>tilt</th><th>construction</th></tr></thead><tbody><tr><td>w1_1</td><td>10.0</td><td>135.0</td><td>wall</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>external_wall</td></tr></table></td></tr><tr><td>w2_1</td><td>10.0</td><td>0.0</td><td>wall</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>external_wall</td></tr></table></td></tr><tr><td>w3_1</td><td>10.0</td><td>45.0</td><td>wall</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>external_wall</td></tr></table></td></tr><tr><td>w4_1</td><td>10.0</td><td>90.0</td><td>wall</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>external_wall</td></tr></table></td></tr><tr><td>floor_2</td><td>10.0</td><td>90.0</td><td>floor</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>external_wall</td></tr></table></td></tr><tr><td>win1_1</td><td>1.0</td><td>45.0</td><td>wall</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>double_glazing</td></tr></table></td></tr></tbody></table></td></tr></table></p><p><h2>Conclusions</h2><p>Conclusions</p></p></body><body><h1>Spaces</h1><p><h2>Introduction</h2><p>Introduction</p></p><p><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>external_wall</td></tr><tr><th>layers</th><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><thead><tr><th>material</th><th>thickness</th></tr></thead><tbody><tr><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>concrete</td></tr><tr><th>k</th><td>1.4</td></tr><tr><th>c</th><td>840.0</td></tr><tr><th>rho</th><td>2240.0</td></tr><tr><th>epsLw</th><td>0.85</td></tr><tr><th>epsSw</th><td>0.65</td></tr></table></td><td>0.2</td></tr><tr><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>insulation_board</td></tr><tr><th>k</th><td>0.03</td></tr><tr><th>c</th><td>1200.0</td></tr><tr><th>rho</th><td>40.0</td></tr><tr><th>epsLw</th><td>0.85</td></tr><tr><th>epsSw</th><td>0.65</td></tr></table></td><td>0.02</td></tr><tr><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>plywood</td></tr><tr><th>k</th><td>0.12</td></tr><tr><th>c</th><td>1210.0</td></tr><tr><th>rho</th><td>540.0</td></tr><tr><th>epsLw</th><td>0.85</td></tr><tr><th>epsSw</th><td>0.65</td></tr></table></td><td>0.1</td></tr></tbody></table></td></tr></table><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>double_glazing</td></tr><tr><th>layers</th><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><thead><tr><th>thickness</th><th>material</th><th>layer_type</th></tr></thead><tbody><tr><td>0.003</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>id_100</td></tr><tr><th>k</th><td>1.0</td></tr><tr><th>c</th><td>840.0</td></tr><tr><th>rho</th><td>2500.0</td></tr><tr><th>epsLw</th><td>0.85</td></tr><tr><th>epsSw</th><td>0.65</td></tr><tr><th>solar_transmittance</th><td><ul><li>0.646</li></ul></td></tr><tr><th>solar_reflectance_outside_facing</th><td><ul><li>0.062</li></ul></td></tr><tr><th>solar_reflectance_room_facing</th><td><ul><li>0.063</li></ul></td></tr><tr><th>infrared_transmissivity</th><td>0.0</td></tr><tr><th>infrared_absorptivity_outside_facing</th><td>0.84</td></tr><tr><th>infrared_absorptivity_room_facing</th><td>0.84</td></tr></table></td><td>glass</td></tr><tr><td>0.0127</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>Air</td></tr><tr><th>k</th><td>0.025</td></tr><tr><th>c</th><td>1005.0</td></tr><tr><th>rho</th><td>1.2</td></tr><tr><th>epsLw</th><td>0.85</td></tr><tr><th>epsSw</th><td>0.65</td></tr></table></td><td>gas</td></tr><tr><td>0.003</td><td><table border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse; margin-top: 20px; margin-bottom: 20px;'><tr><th>name</th><td>id_100</td></tr><tr><th>k</th><td>1.0</td></tr><tr><th>c</th><td>840.0</td></tr><tr><th>rho</th><td>2500.0</td></tr><tr><th>epsLw</th><td>0.85</td></tr><tr><th>epsSw</th><td>0.65</td></tr><tr><th>solar_transmittance</th><td><ul><li>0.646</li></ul></td></tr><tr><th>solar_reflectance_outside_facing</th><td><ul><li>0.062</li></ul></td></tr><tr><th>solar_reflectance_room_facing</th><td><ul><li>0.063</li></ul></td></tr><tr><th>infrared_transmissivity</th><td>0.0</td></tr><tr><th>infrared_absorptivity_outside_facing</th><td>0.84</td></tr><tr><th>infrared_absorptivity_room_facing</th><td>0.84</td></tr></table></td><td>glass</td></tr></tbody></table></td></tr><tr><th>u_value_frame</th><td>1.4</td></tr></table></p><p><h2>Conclusions</h2><p>Conclusions</p></p></body><body><h1>Spaces</h1><p><h2>Introduction</h2><p>Introduction</p></p><p></p><p><h2>Conclusions</h2><p>Conclusions</p></p></body></html>"));
end building;


end buildings_free_float_single_zone_with_data;
