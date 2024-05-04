model ideas_free_float_three_zones

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
extends Modelica.Icons.MaterialPropertiesPackage;    record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);    record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);    record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
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
      epsSw=0.55);    record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_three_zones.Data.Materials.concrete
        (d=0.2),ideas_free_float_three_zones.Data.Materials.insulation_board
        (d=0.02),ideas_free_float_three_zones.Data.Materials.plywood
        (d=0.1)    });
    end external_wall;      record internal_wall
    "internal_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_three_zones.Data.Materials.brick
        (d=0.2)    });
    end internal_wall;
end Constructions;
end Data;


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
    annotation (Placement(transformation(origin={ 0, 50 },
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
    Placement(transformation(origin = { -43.8132552688748, -200.0 }, extent =
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
    Placement(transformation(origin = { 36.31751199001921, 198.55757441730006 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_1(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -121.25119207288044, -169.01631549646098 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_2(
    mSenFac=0.822,    V=10,
    n50=0.822*0.5*space_2.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 200, 50 },
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
    Placement(transformation(origin = { -193.31727247487748, -27.098295866997713 }, extent =
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
    Placement(transformation(origin = { 168.35201114682948, -118.0606456893097 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -169.7405552017221, -104.63210689711714 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_3(
    mSenFac=0.822,    V=10,
    n50=0.822*0.5*space_3.n50toAch,
    redeclare package Medium = Medium,
    nSurf=7,
    hZone=10,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 400, 50 },
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
    Placement(transformation(origin = { 176.92827374405152, 115.19140932920983 }, extent =
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
    Placement(transformation(origin = { 114.55613492846682, 172.52358202038116 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_3(
    redeclare parameter ideas_free_float_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { 28.73810019667843, -182.30873054139946 },
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
    Placement(transformation(origin = { 100.0, 50 },
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
    Placement(transformation(origin = { 200.0, 50 },
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
    Placement(transformation(origin = { 300.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));



equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -21.9066276344374, 50.0 }    ,{ -21.9066276344374, -200.0 }    ,{ -43.8132552688748, -200.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5:6],merged_win1_1_win2_1[1:2].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 18.158755995009606, 50.0 }    ,{ 18.158755995009606, 198.55757441730006 }    ,{ 36.31751199001921, 198.55757441730006 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[7],floor_1.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -60.62559603644022, 50.0 }    ,{ -60.62559603644022, -169.01631549646098 }    ,{ -121.25119207288044, -169.01631549646098 }    },
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
points={{ 200.0, 50.0 }    ,{ 3.341363762561258, 50.0 }    ,{ 3.341363762561258, -27.098295866997713 }    ,{ -193.31727247487748, -27.098295866997713 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[4:5],merged_win1_2_win2_2[1:2].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 184.17600557341473, 50.0 }    ,{ 184.17600557341473, -118.0606456893097 }    ,{ 168.35201114682948, -118.0606456893097 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 15.129722399138956, 50.0 }    ,{ 15.129722399138956, -104.63210689711714 }    ,{ -169.7405552017221, -104.63210689711714 }    },
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
points={{ 400.0, 50.0 }    ,{ 288.46413687202573, 50.0 }    ,{ 288.46413687202573, 115.19140932920983 }    ,{ 176.92827374405152, 115.19140932920983 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[4],merged_w4_3[1].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 257.2780674642334, 50.0 }    ,{ 257.2780674642334, 172.52358202038116 }    ,{ 114.55613492846682, 172.52358202038116 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[5],floor_3.propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 214.36905009833922, 50.0 }    ,{ 214.36905009833922, -182.30873054139946 }    ,{ 28.73810019667843, -182.30873054139946 }    },
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
