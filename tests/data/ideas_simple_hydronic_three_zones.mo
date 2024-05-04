model ideas_simple_hydronic_three_zones

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
    record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_simple_hydronic_three_zones.Data.Materials.id_100
        (d=0.003),ideas_simple_hydronic_three_zones.Data.Materials.Air
        (d=0.0127),ideas_simple_hydronic_three_zones.Data.Materials.id_100
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

    ) "ideas_simple_hydronic_three_zones";
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;    record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);    record brick = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.89,
      c=790.0,
      rho=1920.0,
      epsLw=0.88,
      epsSw=0.55);    record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
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
      epsSw=0.55);    record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"      record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_simple_hydronic_three_zones.Data.Materials.concrete
        (d=0.2),ideas_simple_hydronic_three_zones.Data.Materials.insulation_board
        (d=0.02),ideas_simple_hydronic_three_zones.Data.Materials.plywood
        (d=0.1)    });
    end external_wall;      record internal_wall
    "internal_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_simple_hydronic_three_zones.Data.Materials.brick
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
    nSurf=8,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_1_w2_1_w3_1_w4_1(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -197.7089768976076, -28.678557961877107 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_1(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 164.35017621976314, 76.20771365473658 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -114.59762947889783, 176.09936976112866 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission(
    redeclare package Medium = MediumW) "Radiator"
    annotation (
    Placement(transformation(origin = { 30, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve(
    redeclare package Medium = MediumW) "Radiator valve"
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
    IDEAS.Buildings.Components.Zone space_2(
    mSenFac=0.822,    V=100,
    n50=0.822*0.5*space_2.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 200, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_2_w2_2_w3_2_w4_2(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -43.71351319245662, 173.31095094332986 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_2(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -30.44578310586229, -197.67799074426588 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_3(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -75.77752884597741, 186.22411162177804 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission_2(
    redeclare package Medium = MediumW) "Radiator"
    annotation (
    Placement(transformation(origin = { 230, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve_2(
    redeclare package Medium = MediumW) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 200, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Controls.SpaceControls.PID space_control_2(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { 150, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
    IDEAS.Buildings.Components.Zone space_3(
    mSenFac=0.822,    V=100,
    n50=0.822*0.5*space_3.n50toAch,
    redeclare package Medium = Medium,
    nSurf=8,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 400, 50 },
    extent={{-20,-20},{20,20}}
)));
        IDEAS.Buildings.Components.OuterWall[4]
    merged_w1_3_w2_3_w3_3_w4_3(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall
    constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 78.85855554179987, 185.13531929415498 }, extent =
{{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_3(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Glazing.
    double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 96.73244440240222, -181.446179219494 }
    , extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.SlabOnGround floor_4(
    redeclare parameter ideas_simple_hydronic_three_zones.Data.Constructions.
    external_wall constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation(
    Placement(transformation(origin = { -193.64250065223771, 71.5903994721793 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission_3(
    redeclare package Medium = MediumW) "Radiator"
    annotation (
    Placement(transformation(origin = { 430, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve_3(
    redeclare package Medium = MediumW) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 400, -25 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Controls.SpaceControls.PID space_control_3(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { 350, 0 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Occupancy.SimpleOccupancy occupancy_2 annotation (
    Placement(transformation(origin = { 350, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Buildings.Components.InternalWall internal_space_1_space_2
    (redeclare parameter ideas_simple_hydronic_three_zones.
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
    (redeclare parameter ideas_simple_hydronic_three_zones.
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
    (redeclare parameter ideas_simple_hydronic_three_zones.
    Data.Constructions.internal_wall constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi =
    10) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 300.0, 50 },
    extent = {{-10, -10}, {10, 10}}
)));

        IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve(
    redeclare package Medium = MediumW) "Three-way valve"
    annotation (
    Placement(transformation(origin = { -100, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        Modelica.Blocks.Sources.Constant three_way_valve_control(k= 1)
    annotation (
    Placement(transformation(origin = { -150, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve_2(
    redeclare package Medium = MediumW) "Three-way valve"
    annotation (
    Placement(transformation(origin = { 300, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        Modelica.Blocks.Sources.Constant three_way_valve_control_2(k= 1)
    annotation (
    Placement(transformation(origin = { 250, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.FixedResistances.Junction split_valve (
    redeclare package Medium = MediumW)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 130, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.FixedResistances.Junction split_valve_2 (
    redeclare package Medium = MediumW)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 530, -125 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Fluid.Boilers.Simple boiler(
    redeclare package Medium = MediumW) "Boiler"
    annotation (
    Placement(transformation(origin = { 230, -225 },
    extent = {{-10, -10}, {10, 10}}
)));
        IDEAS.Fluid.Movers.FlowControlled_m_flow pump(
    redeclare package Medium = MediumW, m_flow_nominal = 1, dp_nominal = 100)
    annotation (
    Placement(transformation(origin = { -200, -225 },
    extent = {{-10, -10}, {10, 10}}
)));
        Modelica.Blocks.Sources.Constant pump_control(k= 1)
    annotation (
    Placement(transformation(origin = { -250, -225 },
    extent = {{-10, -10}, {10, 10}}
)));


equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -98.8544884488038, 50.0 }    ,{ -98.8544884488038, -28.678557961877107 }    ,{ -197.7089768976076, -28.678557961877107 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 82.17508810988157, 50.0 }    ,{ 82.17508810988157, 76.20771365473658 }    ,{ 164.35017621976314, 76.20771365473658 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -57.298814739448915, 50.0 }    ,{ -57.298814739448915, 176.09936976112866 }    ,{ -114.59762947889783, 176.09936976112866 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainCon,emission.heatPortCon)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 15.0, 50.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.gainRad,emission.heatPortRad)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 15.0, 50.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.yOcc,occupancy_0.y)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -50.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[7],internal_space_1_space_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 100.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[8],internal_space_1_space_3.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(emission.port_b,split_valve.port_1)
annotation (Line(
points={{ 30.0, -25.0 }    ,{ 80.0, -25.0 }    ,{ 80.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(valve.port_b,emission.port_a)
annotation (Line(
points={{ 0.0, -25.0 }    ,{ 15.0, -25.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.port,space_1.gainCon)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.y,valve.y)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[1:4],merged_w1_2_w2_2_w3_2_w4_2[1:4].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 78.14324340377169, 50.0 }    ,{ 78.14324340377169, 173.31095094332986 }    ,{ -43.71351319245662, 173.31095094332986 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[5],merged_win1_2[1].propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 84.77710844706885, 50.0 }    ,{ 84.77710844706885, -197.67799074426588 }    ,{ -30.44578310586229, -197.67799074426588 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[6],floor_3.propsBus_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 62.111235577011286, 50.0 }    ,{ 62.1112355770113, 186.22411162177804 }    ,{ -75.77752884597741, 186.22411162177804 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.gainCon,emission_2.heatPortCon)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 215.0, 50.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.gainRad,emission_2.heatPortRad)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 215.0, 50.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.yOcc,occupancy_1.y)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 175.0, 50.0 }    ,{ 175.0, 50.0 }    ,{ 150.0, 50.0 }    },
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
smooth=Smooth.None));    connect(emission_2.port_b,split_valve.port_1)
annotation (Line(
points={{ 230.0, -25.0 }    ,{ 180.0, -25.0 }    ,{ 180.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(valve_2.port_b,emission_2.port_a)
annotation (Line(
points={{ 200.0, -25.0 }    ,{ 215.0, -25.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_2.port,space_2.gainCon)
annotation (Line(
points={{ 150.0, 0.0 }    ,{ 175.0, 0.0 }    ,{ 175.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_2.y,valve_2.y)
annotation (Line(
points={{ 150.0, 0.0 }    ,{ 175.0, 0.0 }    ,{ 175.0, -25.0 }    ,{ 200.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[1:4],merged_w1_3_w2_3_w3_3_w4_3[1:4].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 239.42927777089994, 50.0 }    ,{ 239.42927777089994, 185.13531929415498 }    ,{ 78.85855554179987, 185.13531929415498 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[5],merged_win1_3[1].propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 248.3662222012011, 50.0 }    ,{ 248.3662222012011, -181.446179219494 }    ,{ 96.73244440240222, -181.446179219494 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[6],floor_4.propsBus_a)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 103.17874967388116, 50.0 }    ,{ 103.17874967388113, 71.5903994721793 }    ,{ -193.64250065223771, 71.5903994721793 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.gainCon,emission_3.heatPortCon)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 415.0, 50.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.gainRad,emission_3.heatPortRad)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 415.0, 50.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.yOcc,occupancy_2.y)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 375.0, 50.0 }    ,{ 375.0, 50.0 }    ,{ 350.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[7],internal_space_1_space_3.propsBus_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.propsBus[8],internal_space_2_space_3.propsBus_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(emission_3.port_b,split_valve_2.port_1)
annotation (Line(
points={{ 430.0, -25.0 }    ,{ 480.0, -25.0 }    ,{ 480.0, -125.0 }    ,{ 530.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(valve_3.port_b,emission_3.port_a)
annotation (Line(
points={{ 400.0, -25.0 }    ,{ 415.0, -25.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_3.port,space_3.gainCon)
annotation (Line(
points={{ 350.0, 0.0 }    ,{ 375.0, 0.0 }    ,{ 375.0, 50.0 }    ,{ 400.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_3.y,valve_3.y)
annotation (Line(
points={{ 350.0, 0.0 }    ,{ 375.0, 0.0 }    ,{ 375.0, -25.0 }    ,{ 400.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.y,three_way_valve_control.y)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ -125.0, -125.0 }    ,{ -125.0, -125.0 }    ,{ -150.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_2,valve.port_a)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ -50.0, -125.0 }    ,{ -50.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_2,valve_2.port_a)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ 50.0, -125.0 }    ,{ 50.0, -25.0 }    ,{ 200.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_3,split_valve.port_3)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.y,three_way_valve_control_2.y)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 275.0, -125.0 }    ,{ 275.0, -125.0 }    ,{ 250.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.port_2,valve_3.port_a)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 350.0, -125.0 }    ,{ 350.0, -25.0 }    ,{ 400.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve_2.port_3,split_valve_2.port_3)
annotation (Line(
points={{ 300.0, -125.0 }    ,{ 415.0, -125.0 }    ,{ 415.0, -125.0 }    ,{ 530.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(split_valve.port_2,boiler.port_a)
annotation (Line(
points={{ 130.0, -125.0 }    ,{ 180.0, -125.0 }    ,{ 180.0, -225.0 }    ,{ 230.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(split_valve_2.port_2,boiler.port_a)
annotation (Line(
points={{ 530.0, -125.0 }    ,{ 380.0, -125.0 }    ,{ 380.0, -225.0 }    ,{ 230.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(boiler.port_b,pump.port_a)
annotation (Line(
points={{ 230.0, -225.0 }    ,{ 15.0, -225.0 }    ,{ 15.0, -225.0 }    ,{ -200.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.m_flow_in,pump_control.y)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ -225.0, -225.0 }    ,{ -225.0, -225.0 }    ,{ -250.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.port_b,three_way_valve.port_1)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ -150.0, -225.0 }    ,{ -150.0, -125.0 }    ,{ -100.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.port_b,three_way_valve_2.port_1)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ 50.0, -225.0 }    ,{ 50.0, -125.0 }    ,{ 300.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end ideas_simple_hydronic_three_zones;
