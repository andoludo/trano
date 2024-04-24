model ideas_free_float_three_zones

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
        record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_free_float_three_zones.Data.Materials.id_100(d=0.003),ideas_free_float_three_zones.Data.Materials.Air(d=0.0127),ideas_free_float_three_zones.Data.Materials.id_100(d=0.003)    },
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
extends Modelica.Icons.MaterialPropertiesPackage;record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.0,
      c=840.0,
      rho=2500.0,
      epsLw=0.88,
      epsSw=0.55);record brick = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=790.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
      epsLw=0.88,
      epsSw=0.55);record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"record internal_wall
    "internal_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_three_zones.Data.Materials.brick(d=0.2)    });
    end internal_wall;record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_three_zones.Data.Materials.concrete(d=0.2),ideas_free_float_three_zones.Data.Materials.insulation_board(d=0.02),ideas_free_float_three_zones.Data.Materials.plywood(d=0.1)    });
    end external_wall;
end Constructions;
end Data;
  replaceable package Medium = IDEAS.Media.Air
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
  annotation (choicesAllMatching = true);
  inner IDEAS.BoundaryConditions.SimInfoManager sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,
    V=10,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=9,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50 },extent={{-20,-20},{20,20}}
)));    IDEAS.Buildings.Components.OuterWall[4] merged_w1_1_w2_1_w3_1_w4_1(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.external_wall constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 69.38990475210109, 175.4927963715315 }, extent = {{-10, -10}, {10, 10}}
)));    IDEAS.Buildings.Components.Window[2] merged_win1_1_win2_1(
    redeclare parameter ideas_free_float_three_zones.Data.Glazing.double_glazing glazing,
    A={ 10, 10 },
    final azi={ 45, 90 },
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 126.95492949079542, 146.37293808016764 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.SlabOnGround floor_1(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.external_wall constructionType,
    A=10)  annotation(
    Placement(transformation(origin = { -54.30099982124037, -166.05109833124234 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.Zone space_2(
    mSenFac=0.822,
    V=10,
    n50=0.822*0.5*space_2.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 200, 50 },extent={{-20,-20},{20,20}}
)));    IDEAS.Buildings.Components.OuterWall[3] merged_w1_2_w2_2_w3_2(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.external_wall constructionType,
    A={ 10, 10, 10 },
    final azi={ 135, 0, 90 },
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 13.03921130835067, -156.5475212582107 }, extent = {{-10, -10}, {10, 10}}
)));    IDEAS.Buildings.Components.Window[2] merged_win1_2_win2_2(
    redeclare parameter ideas_free_float_three_zones.Data.Glazing.double_glazing glazing,
    A={ 10, 10 },
    final azi={ 0, 90 },
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 76.39240022652896, -151.90067719011267 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.external_wall constructionType,
    A=10)  annotation(
    Placement(transformation(origin = { -109.66250712645258, 163.20261778271495 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.Zone space_3(
    mSenFac=0.822,
    V=10,
    n50=0.822*0.5*space_3.n50toAch,
    redeclare package Medium = Medium,
    nSurf=7,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 400, 50 },extent={{-20,-20},{20,20}}
)));    IDEAS.Buildings.Components.OuterWall[3] merged_w1_3_w2_3_w3_3(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.external_wall constructionType,
    A={ 10, 10, 10 },
    final azi={ 135, 0, 45 },
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -50.9932238464634, 173.38469008391866 }, extent = {{-10, -10}, {10, 10}}
)));    IDEAS.Buildings.Components.Window[1] merged_w4_3(
    redeclare parameter ideas_free_float_three_zones.Data.Glazing.double_glazing glazing,
    A={ 10 },
    final azi={ 45 },
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 186.98938097898335, -37.5849523212697 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.SlabOnGround floor_3(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.external_wall constructionType,
    A=10)  annotation(
    Placement(transformation(origin = { -184.12641926487314, -6.2552845095723955 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.InternalWall internal_space_1_space_2(redeclare parameter ideas_free_float_three_zones.Data.Constructions.internal_wall constructionType,
    A = 10, inc = IDEAS.Types.Tilt.Wall, azi = 10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 100.0, 50 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.InternalWall internal_space_1_space_3(redeclare parameter ideas_free_float_three_zones.Data.Constructions.internal_wall constructionType,
    A = 10, inc = IDEAS.Types.Tilt.Wall, azi = 10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 200.0, 50 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.InternalWall internal_space_2_space_3(redeclare parameter ideas_free_float_three_zones.Data.Constructions.internal_wall constructionType,
    A = 10, inc = IDEAS.Types.Tilt.Wall, azi = 10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 300.0, 50 }, extent = {{-10, -10}, {10, 10}}
)));
equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 34.694952376050544, 50.0 }    ,{ 34.694952376050544, 175.4927963715315 }    ,{ 69.38990475210109, 175.4927963715315 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5:6],merged_win1_1_win2_1[1:2].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 63.47746474539771, 50.0 }    ,{ 63.47746474539771, 146.37293808016764 }    ,{ 126.95492949079542, 146.37293808016764 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[7],floor_1.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -27.150499910620184, 50.0 }    ,{ -27.150499910620184, -166.05109833124234 }    ,{ -54.30099982124037, -166.05109833124234 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[8],internal_space_1_space_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 100.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[9],internal_space_1_space_3.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[1:3],merged_w1_2_w2_2_w3_2[1:3].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 106.51960565417534, 50.0 }    ,{ 106.51960565417534, -156.5475212582107 }    ,{ 13.03921130835067, -156.5475212582107 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[4:5],merged_win1_2_win2_2[1:2].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 138.19620011326447, 50.0 }    ,{ 138.19620011326447, -151.90067719011267 }    ,{ 76.39240022652896, -151.90067719011267 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 45.168746436773716, 50.0 }    ,{ 45.1687464367737, 163.20261778271495 }    ,{ -109.66250712645258, 163.20261778271495 }    },
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
smooth=Smooth.None));    connect(space_3.propsBus[1:3],merged_w1_3_w2_3_w3_3[1:3].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 174.5033880767683, 50.0 }    ,{ 174.50338807676832, 173.38469008391866 }    ,{ -50.9932238464634, 173.38469008391866 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[4],merged_w4_3[1].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 293.4946904894917, 50.0 }    ,{ 293.4946904894917, -37.5849523212697 }    ,{ 186.98938097898335, -37.5849523212697 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[5],floor_3.propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 107.93679036756345, 50.0 }    ,{ 107.93679036756342, -6.2552845095723955 }    ,{ -184.12641926487314, -6.2552845095723955 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[6],internal_space_1_space_3.propsBus_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[7],internal_space_2_space_3.propsBus_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end ideas_free_float_three_zones;
