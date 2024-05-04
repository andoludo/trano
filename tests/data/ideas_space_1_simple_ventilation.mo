
model ideas_space_1_simple_ventilation

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
    record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_space_1_simple_ventilation.Data.Materials.id_100(
         d=0.003),ideas_space_1_simple_ventilation.Data.Materials.Air(
         d=0.0127),ideas_space_1_simple_ventilation.Data.Materials.id_100(
         d=0.003)},
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
      final g_value=0.78)
      "ideas_space_1_simple_ventilation";

end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;
                                                     record plywood =
          IDEAS.Buildings.Data.Interfaces.Material (
          k=0.12,
          c=1210.0,
          rho=540.0,
          epsLw=0.88,
          epsSw=0.55);
  record Air = IDEAS.Buildings.Data.Interfaces.Material (
   k=0.025,
        c=1005.0,
        rho=1.2,
        epsLw=0.88,
        epsSw=0.55);
                      record id_100 = IDEAS.Buildings.Data.Interfaces.Material
          (
          k=1.0,
          c=840.0,
          rho=2500.0,
          epsLw=0.88,
          epsSw=0.55);
  record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
   k=0.03,
        c=1200.0,
        rho=40.0,
        epsLw=0.88,
        epsSw=0.55);
                      record concrete =
          IDEAS.Buildings.Data.Interfaces.Material (
          k=1.4,
          c=840.0,
          rho=2240.0,
          epsLw=0.88,
          epsSw=0.55);
end Materials;

package Constructions "Library of building envelope constructions"
                                                                        record external_wall
        "external_wall"
        extends IDEAS.Buildings.Data.Interfaces.Construction(mats={
              ideas_space_1_simple_ventilation.Data.Materials.concrete(d=0.2),
              ideas_space_1_simple_ventilation.Data.Materials.insulation_board(
              d=0.02),ideas_space_1_simple_ventilation.Data.Materials.plywood(d
              =0.1)});
                                                                        end
        external_wall;
end Constructions;
end Data;


replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);
                                         inner IDEAS.BoundaryConditions.SimInfoManager
sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));

    IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,nPorts = 3,    V=100,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=4,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50},
    extent={{-20,-20},{20,20}})));
        IDEAS.Buildings.Components.OuterWall[2]
    merged_w1_1_w2_1(
    redeclare parameter ideas_space_1_simple_ventilation.Data.Constructions.external_wall
    constructionType,
    A={ 10, 10},
    final azi={ 135, 45},
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall})   annotation (
    Placement(transformation(origin = { -180.72345984489368, 91.69467077054897},  extent=
{{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_1(
    redeclare parameter ideas_space_1_simple_ventilation.Data.Glazing.double_glazing
                   glazing,
    A={ 1},
    final azi={ 45},
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall})   annotation (
    Placement(transformation(origin = { 10.368493261509817, 194.6808385549267},
      extent = {{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_space_1_simple_ventilation.Data.Constructions.external_wall
                  constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation (
    Placement(transformation(origin = { -156.0320260116334, 8.570211134413826},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.Actuators.Dampers.Exponential vav_in(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -114.78723013171653, -154.65474702290442},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.Actuators.Dampers.Exponential vav_out(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -11.54355386320691, -200.0},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.FixedResistances.PressureDrop pressure_drop_duct_out(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 188.06563048433438, 85.66631975434214},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.FixedResistances.PressureDrop pressure_drop_duct_in(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 149.38201399815586, -102.62849577235458},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Controls.SpaceControls.PIDSubstance
    ventilation_control(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { 114.69659563180845, 167.7278647930241},
    extent = {{-10, -10}, {10, 10}})));

    Neosim.Fluid.Ventilation.SimpleHVAC ahu(
     redeclare package Medium = Medium)
    annotation (
    Placement(transformation(origin = { 198.54497460920115, -24.378667229834853},
    extent = {{-10, -10}, {10, 10}})));


equation
            connect(space_1.propsBus[1:2],merged_w1_1_w2_1[1:2].propsBus_a)
annotation (Line(
points={{-20,59},{-90.3617,59},{-90.3617,93.6947},{-172.39,93.6947}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[3],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{-20,57},{5.18425,57},{5.18425,196.681},{18.7018,196.681}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[4],floor_2.propsBus_a)
annotation (Line(
points={{-20,55},{-78.016,55},{-78.016,10.5702},{-147.699,10.5702}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.yOcc,occupancy_0.y)
annotation (Line(
points={{24,58},{-25,58},{-25,50},{-39,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[1],vav_out.port_a)
annotation (Line(
points={{5.33333,70},{-5.77178,70},{-5.77178,-200},{-21.5436,-200}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in.port_b,space_1.ports[2])
annotation (Line(
points={{-104.787,-154.655},{-57.3936,-154.655},{-57.3936,70},{0,70}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_out.port_b,pressure_drop_duct_out.port_a)
annotation (Line(
points={{-1.54355,-200},{88.261,-200},{88.261,85.6663},{178.066,85.6663}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_out.port_b,ahu.port_a)
annotation (Line(
points={{198.066,85.6663},{193.305,85.6663},{193.305,-31.0454},{208.545,
          -31.0454}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_in.port_b,vav_in.port_a)
annotation (Line(
points={{159.382,-102.628},{17.2974,-102.628},{17.2974,-154.655},{-124.787,
          -154.655}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.port_a,space_1.ports[3])
annotation (Line(
points={{104.697,167.728},{57.3483,167.728},{57.3483,70},{-5.33333,70}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_in.y)
annotation (Line(
points={{125.297,167.728},{-0.0453172,167.728},{-0.0453172,-142.655},{-114.787,
          -142.655}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_out.y)
annotation (Line(
points={{125.297,167.728},{51.5765,167.728},{51.5765,-188},{-11.5436,-188}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,pressure_drop_duct_in.port_a)
annotation (Line(
points={{208.545,-17.712},{173.963,-17.712},{173.963,-102.628},{139.382,
          -102.628}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end ideas_space_1_simple_ventilation;
