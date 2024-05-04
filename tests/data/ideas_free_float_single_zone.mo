model ideas_free_float_single_zone

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
    record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_free_float_single_zone.Data.Materials.id_100
        (d=0.003),ideas_free_float_single_zone.Data.Materials.Air
        (d=0.0127),ideas_free_float_single_zone.Data.Materials.id_100
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

    ) "ideas_free_float_single_zone";
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
      epsLw=0.88,
      epsSw=0.55);    record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);    record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);    record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);    record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.0,
      c=840.0,
      rho=2500.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_free_float_single_zone.Data.Materials.concrete
        (d=0.2),ideas_free_float_single_zone.Data.Materials.insulation_board
        (d=0.02),ideas_free_float_single_zone.Data.Materials.plywood
        (d=0.1)    });
    end external_wall;
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
    mSenFac=0.822,    V=100,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=6,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_1_w2_1_w3_1_w4_1(
    redeclare parameter ideas_free_float_single_zone.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -50.92983207066687, 200.0 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_1(
    redeclare parameter ideas_free_float_single_zone.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 159.35469447154446, -115.20858822146121 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_free_float_single_zone.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -175.68778631964003, 18.723529223377973 },
    extent = {{-10, -10}, {10, 10}}
)));



equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.464916035333435, 50.0 }    ,{ -25.464916035333435, 200.0 }    ,{ -50.92983207066687, 200.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 79.67734723577223, 50.0 }    ,{ 79.67734723577223, -115.20858822146121 }    ,{ 159.35469447154446, -115.20858822146121 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -87.84389315982001, 50.0 }    ,{ -87.84389315982001, 18.723529223377973 }    ,{ -175.68778631964003, 18.723529223377973 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end ideas_free_float_single_zone;
