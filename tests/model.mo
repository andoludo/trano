model mymodel
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Air "Medium model";
  parameter Buildings.HeatTransfer.Data.Solids.Plywood matWoo(x = 0.01, k = 0.11, d = 544, nStaRef = 1) "Wood for exterior construction" annotation(
    Placement(transformation(extent = {{40, 110}, {60, 130}})));
  parameter Buildings.HeatTransfer.Data.Solids.Concrete matCon(x = 0.1, k = 1.311, c = 836, nStaRef = 5) "Concrete" annotation(
    Placement(transformation(extent = {{40, 140}, {60, 160}})));
  parameter Buildings.HeatTransfer.Data.Solids.Generic matIns(x = 0.087, k = 0.049, c = 836.8, d = 265, nStaRef = 5) "Steelframe construction with insulation" annotation(
    Placement(transformation(extent = {{80, 110}, {100, 130}})));
  parameter Buildings.HeatTransfer.Data.Solids.GypsumBoard matGyp(x = 0.0127, k = 0.16, c = 830, d = 784, nStaRef = 2) "Gypsum board" annotation(
    Placement(transformation(extent = {{40, 80}, {60, 100}})));
  parameter Buildings.HeatTransfer.Data.Solids.GypsumBoard matGyp2(x = 0.025, k = 0.16, c = 830, d = 784, nStaRef = 2) "Gypsum board" annotation(
    Placement(transformation(extent = {{80, 80}, {100, 100}})));
  parameter Buildings.HeatTransfer.Data.Solids.Plywood matFur(x = 0.15, nStaRef = 5) "Material for furniture" annotation(
    Placement(transformation(extent = {{80, 170}, {100, 190}})));
  parameter Buildings.HeatTransfer.Data.Solids.Plywood matCarTra(x = 0.215/0.11, k = 0.11, d = 544, nStaRef = 1) "Wood for floor" annotation(
    Placement(transformation(extent = {{40, 170}, {60, 190}})));
  parameter Buildings.HeatTransfer.Data.Resistances.Carpet matCar "Carpet" annotation(
    Placement(transformation(extent = {{120, 140}, {140, 160}})));
  parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear glaSys(UFra = 2, shade = Buildings.HeatTransfer.Data.Shades.Gray(), haveInteriorShade = false, haveExteriorShade = false) "Data record for the glazing system" annotation(
    Placement(transformation(extent = {{80, 140}, {100, 160}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conExtWal(final nLay = 3, material = {matWoo, matIns, matGyp}) "Exterior construction" annotation(
    Placement(transformation(extent = {{120, 110}, {140, 130}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conIntWal(final nLay = 1, material = {matGyp2}) "Interior wall construction" annotation(
    Placement(transformation(extent = {{160, 110}, {180, 130}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conFlo(final nLay = 1, material = {matCon}) "Floor construction (opa_a is carpet)" annotation(
    Placement(transformation(extent = {{120, 80}, {140, 100}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic conFur(final nLay = 1, material = {matFur}) "Construction for internal mass of furniture" annotation(
    Placement(transformation(extent = {{160, 80}, {180, 100}})));
  parameter String weaFil = Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos") "Weather data file";
  parameter Modelica.Units.SI.Volume VRoo = 4555.7 "Room volum";
  parameter Modelica.Units.SI.Height hRoo = 2.74 "Room height";
  parameter Modelica.Units.SI.Length hWin = 1.5 "Height of windows";
  parameter Real winWalRat(min = 0.01, max = 0.99) = 0.33 "Window to wall ratio for exterior walls";
  parameter Modelica.Units.SI.Area AFlo = VRoo/hRoo "Floor area";
  Buildings.ThermalZones.Detailed.MixedAir roo(redeclare package Medium = Medium, AFlo = AFlo, hRoo = hRoo, nConExt = 0, nConExtWin = 4, datConExtWin(layers = {conExtWal, conExtWal, conExtWal, conExtWal}, A = {49.91*hRoo, 49.91*hRoo, 33.27*hRoo, 33.27*hRoo}, glaSys = {glaSys, glaSys, glaSys, glaSys}, wWin = {winWalRat/hWin*49.91*hRoo, winWalRat/hWin*49.91*hRoo, winWalRat/hWin*33.27*hRoo, winWalRat/hWin*33.27*hRoo}, each hWin = hWin, fFra = {0.1, 0.1, 0.1, 0.1}, til = {Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall}, azi = {Buildings.Types.Azimuth.N, Buildings.Types.Azimuth.S, Buildings.Types.Azimuth.W, Buildings.Types.Azimuth.E}), nConPar = 3, datConPar(layers = {conFlo, conFur, conIntWal}, A = {AFlo, AFlo*2, (6.47*2 + 40.76 + 24.13)*2*hRoo}, til = {Buildings.Types.Tilt.Floor, Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall}), nConBou = 0,     nSurBou=1,
    surBou(
      each A=4*3,
      each absIR=0.9,
      each absSol=0.9,
      each til=Buildings.Types.Tilt.Wall), energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Floor" annotation(
    Placement(transformation(extent = {{48, -62}, {88, -22}})));
  Buildings.ThermalZones.Detailed.MixedAir roo2(redeclare package Medium = Medium, AFlo = AFlo, hRoo = hRoo, nConExt = 0, nConExtWin = 4, datConExtWin(layers = {conExtWal, conExtWal, conExtWal, conExtWal}, A = {49.91*hRoo, 49.91*hRoo, 33.27*hRoo, 33.27*hRoo}, glaSys = {glaSys, glaSys, glaSys, glaSys}, wWin = {winWalRat/hWin*49.91*hRoo, winWalRat/hWin*49.91*hRoo, winWalRat/hWin*33.27*hRoo, winWalRat/hWin*33.27*hRoo}, each hWin = hWin, fFra = {0.1, 0.1, 0.1, 0.1}, til = {Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall}, azi = {Buildings.Types.Azimuth.N, Buildings.Types.Azimuth.S, Buildings.Types.Azimuth.W, Buildings.Types.Azimuth.E}), nConPar = 3, datConPar(layers = {conFlo, conFur, conIntWal}, A = {AFlo, AFlo*2, (6.47*2 + 40.76 + 24.13)*2*hRoo}, til = {Buildings.Types.Tilt.Floor, Buildings.Types.Tilt.Wall, Buildings.Types.Tilt.Wall}), nConBou = 0,     nSurBou=1,
    surBou(
      each A=4*3,
      each absIR=0.9,
      each absSol=0.9,
      each til=Buildings.Types.Tilt.Wall), energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Floor" annotation(
    Placement(transformation(origin = {92, -28}, extent = {{48, -62}, {88, -22}})));
  Modelica.Blocks.Sources.Constant qConGai_flow(k = 0) "Convective heat gain" annotation(
    Placement(transformation(origin = {-64, 8}, extent = {{-60, -40}, {-40, -20}})));
  Modelica.Blocks.Sources.Constant qRadGai_flow(k = 0) "Radiative heat gain" annotation(
    Placement(transformation(origin = {0, 28}, extent = {{-60, 0}, {-40, 20}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3_1 "Multiplex" annotation(
    Placement(transformation(origin = {2, -34}, extent = {{-20, -40}, {0, -20}})));
  Modelica.Blocks.Sources.Constant qLatGai_flow(k = 0) "Latent heat gain" annotation(
    Placement(transformation(origin = {-52, -4}, extent = {{-60, -80}, {-40, -60}})));
  Modelica.Blocks.Sources.Constant qConGai_flow_1(k = 0) "Convective heat gain" annotation(
    Placement(transformation(origin = {-6, -100}, extent = {{-60, -40}, {-40, -20}})));
  Modelica.Blocks.Sources.Constant qRadGai_flow_1(k = 0) "Radiative heat gain" annotation(
    Placement(transformation(origin = {22, -114}, extent = {{-60, 0}, {-40, 20}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3_2 "Multiplex" annotation(
    Placement(transformation(origin = {90, -70}, extent = {{-20, -40}, {0, -20}})));
  Modelica.Blocks.Sources.Constant qLatGai_flow_1(k = 0) "Latent heat gain" annotation(
    Placement(transformation(origin = {-46, -84}, extent = {{-60, -80}, {-40, -60}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam = weaFil, computeWetBulbTemperature = false) annotation(
    Placement(transformation(origin = {40, 46}, extent = {{-60, 40}, {-40, 60}})));
  Buildings.HeatTransfer.Conduction.MultiLayer parWal(A = 4*3, layers = conIntWal, stateAtSurface_a = true, stateAtSurface_b = true) "Partition wall between the two rooms" annotation(
    Placement(transformation(origin = {186, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(qRadGai_flow.y, multiplex3_1.u1[1]) annotation(
    Line(points = {{-39, 38}, {-52, 38}, {-52, -57}, {-20, -57}}, color = {0, 0, 127}));
  connect(qConGai_flow.y, multiplex3_1.u2[1]) annotation(
    Line(points = {{-103, -22}, {-52.5, -22}, {-52.5, -64}, {-20, -64}}, color = {0, 0, 127}));
  connect(qLatGai_flow.y, multiplex3_1.u3[1]) annotation(
    Line(points = {{-91, -74}, {-52, -74}, {-52, -71}, {-20, -71}}, color = {0, 0, 127}));
  connect(qRadGai_flow_1.y, multiplex3_2.u1[1]) annotation(
    Line(points = {{-17, -104}, {68, -104}, {68, -93}}, color = {0, 0, 127}));
  connect(qConGai_flow_1.y, multiplex3_2.u2[1]) annotation(
    Line(points = {{-45, -130}, {-1.5, -130}, {-1.5, -100}, {68, -100}}, color = {0, 0, 127}));
  connect(qLatGai_flow_1.y, multiplex3_2.u3[1]) annotation(
    Line(points = {{-85, -154}, {68, -154}, {68, -107}}, color = {0, 0, 127}));
  connect(multiplex3_1.y, roo.qGai_flow) annotation(
    Line(points = {{3, -64}, {13.7, -64}, {13.7, -34}, {46.4, -34}}, color = {0, 0, 127}));
  connect(weaDat.weaBus, roo.weaBus) annotation(
    Line(points = {{0, 96}, {0, -24}, {85.9, -24}, {85.9, -24.1}}, color = {255, 204, 51}, thickness = 0.5));
  connect(weaDat.weaBus, roo2.weaBus) annotation(
    Line(points = {{0, 96}, {0, -24}, {178, -24}, {178, -52}}, color = {255, 204, 51}, thickness = 0.5));
  connect(multiplex3_2.y, roo2.qGai_flow) annotation(
    Line(points = {{92, -100}, {138, -100}, {138, -62}}, color = {0, 0, 127}, thickness = 0.5));
  connect(roo.surf_surBou[1], parWal.port_b) annotation(
    Line(points = {{64, -56}, {116, -56}, {116, 22}, {76, 22}, {76, 41}, {122, 41}, {122, 11.5}, {186, 11.5}, {186, 6}}, color = {191, 0, 0}));
  connect(roo2.surf_surBou[1], parWal.port_a) annotation(
    Line(points = {{156, -84}, {186, -84}, {186, -14}}, color = {191, 0, 0}));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {200, 200}})),
    __Dymola_Commands(file = "modelica://Buildings/Resources/Scripts/Dymola/ThermalZones/Detailed/Examples/MixedAirFreeResponse.mos" "Simulate and plot"),
    Documentation(info = "<html>
<p>
This model illustrates the use of the room model
<a href=\"modelica://Buildings.ThermalZones.Detailed.MixedAir\">
Buildings.ThermalZones.Detailed.MixedAir</a>.
</p>
<p>
The geometry, materials and constructions of the model are consistent with those of
<a href=\"modelica://Buildings.Examples.VAVReheat.BaseClasses.Floor\">
Buildings.Examples.VAVReheat.BaseClasses.Floor</a>
but here they are modeled as a single thermal zone.
The model is representative for one floor of the
new construction medium office building for Chicago, IL,
as described in the set of DOE Commercial Building Benchmarks.
There are four perimeter zones and one core zone.
The envelope thermal properties meet ASHRAE Standard 90.1-2004.
</p>
<p>
For a comparison between the one-zone and five-zone model, see
<a href=\"modelica://Buildings.ThermalZones.Detailed.Validation.SingleZoneFloorWithHeating\">
Buildings.ThermalZones.Detailed.Validation.SingleZoneFloorWithHeating</a>.
</p>
</html>", revisions = "<html>
<ul>
<li>
April 10, 2020, by Michael Wetter:<br/>
Changed room model geometry and construction material.
</li>
<li>
October 29, 2016, by Michael Wetter:<br/>
Changed example to place a state at the surface,
and removed computation of the wet bulb temperature
as it is not needed.<br/>
Added thermal resistance of soil, because at the connector
<code>surf_conBou</code>, there is now a state variable, and
hence the temperature cannot be prescribed if its initial value
is specified.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/565\">issue 565</a>.
</li>
<li>
March 26, 2015, by Michael Wetter:<br/>
Set initialization of <code>conOut</code>
to be steady-state initialization.
</li>
<li>
February 12, 2015, by Michael Wetter:<br/>
Set initial temperature to be <i>22</i>&deg;C to add
propagation of the initial temperature to this test case.
</li>
<li>
December 22, 2014 by Michael Wetter:<br/>
Removed <code>Modelica.Fluid.System</code>
to address issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
</li>
<li>
September 11, 2014, by Michael Wetter:<br/>
Changed assignment of <code>layers</code> in <code>conOut</code>
as <code>layers</code> is no longer replaceable.
</li>
<li>
May 1, 2013, by Michael Wetter:<br/>
Declared the parameter record to be a parameter, as declaring its elements
to be parameters does not imply that the whole record has the variability of a parameter.
</li>
<li>
December 14, 2010, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    experiment(StopTime = 172800, Tolerance = 1e-06),
    uses(Buildings(version = "10.0.0")));
end mymodel;