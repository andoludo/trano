model ideas_simple_hydronic_no_occupancy


package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
        record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_simple_hydronic_no_occupancy.Data.Materials.id_100(d=0.003),ideas_simple_hydronic_no_occupancy.Data.Materials.Air(d=0.0127),ideas_simple_hydronic_no_occupancy.Data.Materials.id_100(d=0.003)    },
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

    ) "ideas_simple_hydronic_no_occupancy";

end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;record plywood = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.12,
      c=1210.0,
      rho=540.0,
      epsLw=0.88,
      epsSw=0.55);record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.0,
      c=840.0,
      rho=2500.0,
      epsLw=0.88,
      epsSw=0.55);record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.03,
      c=1200.0,
      rho=40.0,
      epsLw=0.88,
      epsSw=0.55);record Air = IDEAS.Buildings.Data.Interfaces.Material (
 k=0.025,
      c=1005.0,
      rho=1.2,
      epsLw=0.88,
      epsSw=0.55);record concrete = IDEAS.Buildings.Data.Interfaces.Material (
 k=1.4,
      c=840.0,
      rho=2240.0,
      epsLw=0.88,
      epsSw=0.55);end Materials;
package Constructions "Library of building envelope constructions"record external_wall
    "external_wall"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
      mats={ideas_simple_hydronic_no_occupancy.Data.Materials.concrete(d=0.2),ideas_simple_hydronic_no_occupancy.Data.Materials.insulation_board(d=0.02),ideas_simple_hydronic_no_occupancy.Data.Materials.plywood(d=0.1)    });
    end external_wall;
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
  inner IDEAS.BoundaryConditions.SimInfoManager sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,
    V=100,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=6,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50 },extent={{-20,-20},{20,20}}
)));    IDEAS.Buildings.Components.OuterWall[4] merged_w1_1_w2_1_w3_1_w4_1(
    redeclare parameter ideas_simple_hydronic_no_occupancy.Data.Constructions.external_wall constructionType,
    A={ 10, 10, 10, 10 },
    final azi={ 135, 0, 45, 90 },
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { -177.08118681069303, 27.202423455535115 }, extent = {{-10, -10}, {10, 10}}
)));    IDEAS.Buildings.Components.Window[1] merged_win1_1(
    redeclare parameter ideas_simple_hydronic_no_occupancy.Data.Glazing.double_glazing glazing,
    A={ 1 },
    final azi={ 45 },
    final inc={ IDEAS.Types.Tilt.Wall })  annotation(
    Placement(transformation(origin = { 146.3678117981924, 152.64655396609103 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_simple_hydronic_no_occupancy.Data.Constructions.external_wall constructionType,
    A=10)  annotation(
    Placement(transformation(origin = { 162.51472671889198, 57.30877756904761 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission(
    redeclare package Medium = MediumW) "Radiator"
    annotation (
    Placement(transformation(origin = { 30, -25 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage valve(
    redeclare package Medium = MediumW) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 0, -25 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Controls.SpaceControls.PID space_control(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { -50, 0 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve(
    redeclare package Medium = MediumW) "Three-way valve"
    annotation (
    Placement(transformation(origin = { -100, -125 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Fluid.FixedResistances.Junction split_valve (
    redeclare package Medium = MediumW)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 130, -125 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Fluid.Boilers.Simple boiler(
    redeclare package Medium = MediumW) "Boiler"
    annotation (
    Placement(transformation(origin = { 230, -225 }, extent = {{-10, -10}, {10, 10}}
)));IDEAS.Fluid.Movers.FlowControlled_m_flow pump(
    redeclare package Medium = MediumW, m_flow_nominal = 1, dp_nominal = 100)
    annotation (
    Placement(transformation(origin = { -200, -225 }, extent = {{-10, -10}, {10, 10}}
)));Modelica.Blocks.Sources.Constant pump_control(k= 1)
    annotation (
    Placement(transformation(origin = { 189.93722889469845, -22.44475858928548 }, extent = {{-10, -10}, {10, 10}}
)));Modelica.Blocks.Sources.Constant three_way_valve_control(k= 1)
    annotation (
    Placement(transformation(origin = { -70.10367456887504, -187.19251899765217 }, extent = {{-10, -10}, {10, 10}}
)));equation    connect(space_1.propsBus[1:4],merged_w1_1_w2_1_w3_1_w4_1[1:4].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -88.54059340534651, 50.0 }    ,{ -88.54059340534651, 27.202423455535115 }    ,{ -177.08118681069303, 27.202423455535115 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 73.1839058990962, 50.0 }    ,{ 73.1839058990962, 152.64655396609103 }    ,{ 146.3678117981924, 152.64655396609103 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[6],floor_2.propsBus_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 81.25736335944599, 50.0 }    ,{ 81.25736335944599, 57.30877756904761 }    ,{ 162.51472671889198, 57.30877756904761 }    },
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
smooth=Smooth.None));    connect(three_way_valve.port_2,valve.port_a)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ -50.0, -125.0 }    ,{ -50.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.port_3,split_valve.port_3)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 15.0, -125.0 }    ,{ 130.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(three_way_valve.y,three_way_valve_control.y)
annotation (Line(
points={{ -100.0, -125.0 }    ,{ -85.05183728443751, -125.0 }    ,{ -85.05183728443751, -187.19251899765217 }    ,{ -70.10367456887504, -187.19251899765217 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(split_valve.port_2,boiler.port_a)
annotation (Line(
points={{ 130.0, -125.0 }    ,{ 180.0, -125.0 }    ,{ 180.0, -225.0 }    ,{ 230.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(boiler.port_b,pump.port_a)
annotation (Line(
points={{ 230.0, -225.0 }    ,{ 15.0, -225.0 }    ,{ 15.0, -225.0 }    ,{ -200.0, -225.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.port_b,three_way_valve.port_1)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ -150.0, -225.0 }    ,{ -150.0, -125.0 }    ,{ -100.0, -125.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pump.m_flow_in,pump_control.y)
annotation (Line(
points={{ -200.0, -225.0 }    ,{ -5.031385552650761, -225.0 }    ,{ -5.031385552650789, -22.44475858928548 }    ,{ 189.93722889469845, -22.44475858928548 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end ideas_simple_hydronic_no_occupancy;