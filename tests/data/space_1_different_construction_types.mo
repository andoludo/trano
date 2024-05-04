model space_1_different_construction_types

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
    record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={space_1_different_construction_types.Data.Materials.id_100
        (d=0.003),space_1_different_construction_types.Data.Materials.Air
        (d=0.0127),space_1_different_construction_types.Data.Materials.id_100
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

    ) "space_1_different_construction_types";
    record  simple_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={space_1_different_construction_types.Data.Materials.id_100
        (d=0.003),space_1_different_construction_types.Data.Materials.Air
        (d=0.0127),space_1_different_construction_types.Data.Materials.id_100
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

    ) "space_1_different_construction_types";
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);    record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);    record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.0,
      c=840.0,
      rho=2500.0,
      epsLw=0.88,
      epsSw=0.55);    record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
      epsLw=0.88,
      epsSw=0.55);    record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);    record brick = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=790.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={space_1_different_construction_types.Data.Materials.concrete
        (d=0.2),space_1_different_construction_types.Data.Materials.insulation_board
        (d=0.02),space_1_different_construction_types.Data.Materials.plywood
        (d=0.1)    });
    end external_wall;      record test_wall
    "test_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={space_1_different_construction_types.Data.Materials.concrete
        (d=0.4),space_1_different_construction_types.Data.Materials.insulation_board
        (d=0.2),space_1_different_construction_types.Data.Materials.plywood
        (d=0.4)    });
    end test_wall;      record internal_wall
    "internal_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={space_1_different_construction_types.Data.Materials.brick
        (d=0.2)    });
    end internal_wall;
end Constructions;
end Data;


replaceable package Medium = IDEAS.Media.Air
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
  inner IDEAS.BoundaryConditions.SimInfoManager sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));


    IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,    V=100,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=7,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[2]
    merged_w1_1_w3_1(
    redeclare parameter space_1_different_construction_types.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10 },
    final azi={ 135, 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -172.7701079869989, -107.11026448036343 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.OuterWall[1]
    merged_w2_1(
    redeclare parameter space_1_different_construction_types.Data.Constructions.
    internal_wall
    constructionType,
    A={ 10 },
    final azi={ 0 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -189.0273939224997, 3.3504870204799153 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.OuterWall[1]
    merged_w4_1(
    redeclare parameter space_1_different_construction_types.Data.Constructions.
    test_wall
    constructionType,
    A={ 10 },
    final azi={ 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 25.920260175765964, 188.5925804108925 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_1(
    redeclare parameter space_1_different_construction_types.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 110.02359197164235, -131.86319826158845 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_2(
    redeclare parameter space_1_different_construction_types.Data.Glazing.
    simple_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 126.71404715556504, 144.4771561243453 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter space_1_different_construction_types.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { 200.0, 56.66971283490549 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.HeatTransfer.IdealHeatingSystem.IdealHeatEmission emission
    annotation (
    Placement(transformation(origin = { 0, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Controls.SpaceControls.PID space_control(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { -50, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50 },
    extent = {{-10, -10}, {10, 10}}
)));



equation    connect(space_1.propsBus[1:2],merged_w1_1_w3_1[1:2].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -86.38505399349945, 50.0 }    ,{ -86.38505399349945, -107.11026448036343 }    ,{ -172.7701079869989, -107.11026448036343 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[3],merged_w2_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -94.51369696124985, 50.0 }    ,{ -94.51369696124985, 3.3504870204799153 }    ,{ -189.0273939224997, 3.3504870204799153 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[4],merged_w4_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 12.960130087882982, 50.0 }    ,{ 12.960130087882982, 188.5925804108925 }    ,{ 25.920260175765964, 188.5925804108925 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 55.01179598582117, 50.0 }    ,{ 55.01179598582117, -131.86319826158845 }    ,{ 110.02359197164235, -131.86319826158845 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[6],merged_win1_2[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 63.35702357778252, 50.0 }    ,{ 63.35702357778252, 144.4771561243453 }    ,{ 126.71404715556504, 144.4771561243453 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[7],floor_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 100.0, 56.66971283490549 }    ,{ 200.0, 56.66971283490549 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainCon,emission.heatPortCon)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 0.0, 50.0 }    ,{ 0.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainRad,emission.heatPortRad)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 0.0, 50.0 }    ,{ 0.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.yOcc,occupancy_0.y)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -50.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.port,space_1.gainCon)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.y,emission.y)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end space_1_different_construction_types;
