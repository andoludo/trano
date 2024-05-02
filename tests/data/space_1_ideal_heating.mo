model space_1_ideal_heating

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


package Medium = Buildings.Media.Air "Medium model";
package MediumW = Buildings.Media.Water "Medium model";
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


    Buildings.ThermalZones.Detailed.MixedAir space_1(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,                nConExt=4,
                datConExt(
                layers={ external_wall, external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 0.0, 45.0, 90.0 }),
                nSurBou=0,                nConBou=1,
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
                hWin={ 1.0 }),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(origin=
    { 0, 50 },extent={{-20,-20},{20,20}}
)));






        Neosim.HeatTransfer.IdealHeatingSystem.IdealHeatEmission emission
    annotation (
    Placement(transformation(origin = {22, -33 },
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
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
            weather(filNam =
    Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/
    USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    annotation (
    Placement(transformation(origin = { -100, 200 },
    extent = {{-10, -10}, {10, 10}}
)));


equation    connect(space_1.heaPorAir,emission.heatPortCon)
annotation (Line(
points = {{0, 50}, {0, 15}, {20, 15}, {20, -26}},
color={255,204,51},
thickness=0.5));    connect(space_1.heaPorRad,emission.heatPortRad)
annotation (Line(
points = {{0, 50}, {0, 12.5}, {24, 12.5}, {24, -26}},
color={255,204,51},
thickness=0.5));    connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -50.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.weaBus,weather.weaBus)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -50.0, 50.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.port,space_1.heaPorAir)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.y,emission.y)
annotation (Line(
points = {{-50, 0}, {-25, 0}, {-25, -33}, {10, -33}},
color={255,204,51},
thickness=0.5));
end space_1_ideal_heating;
