model buildings_simple_hydronic_three_zones
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
        absIR_b=0.84),
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


        parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic external_wall(
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

        parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic internal_wall(
    final nLay=1,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=0.89,
        c=790.0,
        d=1920.0)    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));

Buildings.ThermalZones.Detailed.MixedAir space_1(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,                nConExt=4,
                datConExt(
                layers={ external_wall, external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 0.0, 45.0, 90.0 }),
                nSurBou=2,
                surBou(
                A={ 10.0, 10.0 },
                til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                nConBou=1,
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
    annotation (Placement(transformation(origin={ 0, 50 },extent={{-20,-20},{20,20}}
)));Buildings.ThermalZones.Detailed.MixedAir space_2(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,                nConExt=4,
                datConExt(
                layers={ external_wall, external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 0.0, 45.0, 90.0 }),
                nSurBou=2,
                surBou(
                A={ 10.0, 10.0 },
                til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                nConBou=1,
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
    annotation (Placement(transformation(origin={ 200, 50 },extent={{-20,-20},{20,20}}
)));Buildings.ThermalZones.Detailed.MixedAir space_3(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,                nConExt=4,
                datConExt(
                layers={ external_wall, external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 0.0, 45.0, 90.0 }),
                nSurBou=2,
                surBou(
                A={ 10.0, 10.0 },
                til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall}),
                nConBou=1,
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
    annotation (Placement(transformation(origin={ 400, 50 },extent={{-20,-20},{20,20}}
)));Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission(
    redeclare package Medium = MediumW,
    Q_flow_nominal=scaFacRad*Q_flow_nominal/nRoo,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=323.15,
    T_b_nominal=313.15) "Radiator"
    annotation (
    Placement(transformation(origin = { 30, -25 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valve(
    redeclare package Medium = MediumW,
    dpValve_nominal(displayUnit="Pa") = dpVal_nominal,
    m_flow_nominal=mRad_flow_nominal/nRoo,
    dpFixed_nominal=dpRoo_nominal,
    from_dp=true,
    use_inputFilter=false) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 0, -25 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Controls.SpaceControls.PID space_control(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { -50, 0 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = {-80, 48 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission_2(
    redeclare package Medium = MediumW,
    Q_flow_nominal=scaFacRad*Q_flow_nominal/nRoo,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=323.15,
    T_b_nominal=313.15) "Radiator"
    annotation (
    Placement(transformation(origin = { 230, -25 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valve_2(
    redeclare package Medium = MediumW,
    dpValve_nominal(displayUnit="Pa") = dpVal_nominal,
    m_flow_nominal=mRad_flow_nominal/nRoo,
    dpFixed_nominal=dpRoo_nominal,
    from_dp=true,
    use_inputFilter=false) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 200, -25 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Controls.SpaceControls.PID space_control_2(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { 150, 0 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 emission_3(
    redeclare package Medium = MediumW,
    Q_flow_nominal=scaFacRad*Q_flow_nominal/nRoo,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=323.15,
    T_b_nominal=313.15) "Radiator"
    annotation (
    Placement(transformation(origin = { 430, -25 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valve_3(
    redeclare package Medium = MediumW,
    dpValve_nominal(displayUnit="Pa") = dpVal_nominal,
    m_flow_nominal=mRad_flow_nominal/nRoo,
    dpFixed_nominal=dpRoo_nominal,
    from_dp=true,
    use_inputFilter=false) "Radiator valve"
    annotation (
    Placement(transformation(origin = { 400, -25 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Controls.SpaceControls.PID space_control_3(setPoint = 295.15, yMax = 1, yMin = 0)
    annotation (
    Placement(transformation(origin = { 350, 0 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Occupancy.SimpleOccupancy occupancy_2 annotation (
    Placement(transformation(origin = { 350, 50 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.HeatTransfer.Conduction.MultiLayer internal_space_1_space_2(A = 10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 100.0, 50 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.HeatTransfer.Conduction.MultiLayer internal_space_1_space_3(A = 10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 200.0, 50 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.HeatTransfer.Conduction.MultiLayer internal_space_2_space_3(A = 10, layers =
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = { 300.0, 50 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weather(filNam =
    Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    annotation (
    Placement(transformation(origin = { -100, 200 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve(
    redeclare package Medium = MediumW,
    dpValve_nominal=dpThrWayVal_nominal,
    l={0.01,0.01},
    tau=10,
    m_flow_nominal=mRad_flow_nominal,
    dpFixed_nominal={100,0},
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-way valve"
    annotation (
    Placement(transformation(origin = { -100, -125 }, extent = {{-10, -10}, {10, 10}}
)));Modelica.Blocks.Sources.Constant three_way_valve_control(k= 1)
    annotation (
    Placement(transformation(origin = { -150, -125 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear three_way_valve_2(
    redeclare package Medium = MediumW,
    dpValve_nominal=dpThrWayVal_nominal,
    l={0.01,0.01},
    tau=10,
    m_flow_nominal=mRad_flow_nominal,
    dpFixed_nominal={100,0},
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-way valve"
    annotation (
    Placement(transformation(origin = { 300, -125 }, extent = {{-10, -10}, {10, 10}}
)));Modelica.Blocks.Sources.Constant three_way_valve_control_2(k= 1)
    annotation (
    Placement(transformation(origin = { 250, -125 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.FixedResistances.Junction split_valve (
    dp_nominal={0,0,0},
    m_flow_nominal=mRad_flow_nominal*{1,-1,-1},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 130, -125 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.FixedResistances.Junction split_valve_2 (
    dp_nominal={0,0,0},
    m_flow_nominal=mRad_flow_nominal*{1,-1,-1},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = { 530, -125 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Fluid.Boilers.Simple boiler(
    redeclare package Medium = MediumW) "Boiler"
    annotation (
    Placement(transformation(origin = { 230, -225 }, extent = {{-10, -10}, {10, 10}}
)));Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y pump(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal,
    dp_nominal=dp_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (
    Placement(transformation(origin = { -200, -225 }, extent = {{-10, -10}, {10, 10}}
)));Modelica.Blocks.Sources.Constant pump_control(k= 1)
    annotation (
    Placement(transformation(origin = { -250, -225 }, extent = {{-10, -10}, {10, 10}}
)));

equation    connect(space_1.heaPorAir,emission.heatPortCon)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 15.0, 50.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.heaPorRad,emission.heatPortRad)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 15.0, 50.0 }    ,{ 15.0, -25.0 }    ,{ 30.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points = {{0, 50}, {-21.5, 50}, {-21.5, 48}, {-69, 48}},
color={255,204,51},
thickness=0.5));    connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 100.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.surf_surBou[2],internal_space_1_space_3.port_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 100.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.weaBus,weather.weaBus)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -50.0, 50.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
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
smooth=Smooth.None));    connect(space_control.port,space_1.heaPorAir)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control.y,valve.y)
annotation (Line(
points={{ -50.0, 0.0 }    ,{ -25.0, 0.0 }    ,{ -25.0, -25.0 }    ,{ 0.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.heaPorAir,emission_2.heatPortCon)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 215.0, 50.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.heaPorRad,emission_2.heatPortRad)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 215.0, 50.0 }    ,{ 215.0, -25.0 }    ,{ 230.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.qGai_flow,occupancy_1.y)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 175.0, 50.0 }    ,{ 175.0, 50.0 }    ,{ 150.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.surf_surBou[1],internal_space_1_space_2.port_b)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 150.0, 50.0 }    ,{ 150.0, 50.0 }    ,{ 100.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.surf_surBou[2],internal_space_2_space_3.port_a)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 250.0, 50.0 }    ,{ 250.0, 50.0 }    ,{ 300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.weaBus,weather.weaBus)
annotation (Line(
points={{ 200.0, 50.0 }    ,{ 50.0, 50.0 }    ,{ 50.0, 200.0 }    ,{ -100.0, 200.0 }    },
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
smooth=Smooth.None));    connect(space_control_2.port,space_2.heaPorAir)
annotation (Line(
points={{ 150.0, 0.0 }    ,{ 175.0, 0.0 }    ,{ 175.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_control_2.y,valve_2.y)
annotation (Line(
points={{ 150.0, 0.0 }    ,{ 175.0, 0.0 }    ,{ 175.0, -25.0 }    ,{ 200.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.heaPorAir,emission_3.heatPortCon)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 415.0, 50.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.heaPorRad,emission_3.heatPortRad)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 415.0, 50.0 }    ,{ 415.0, -25.0 }    ,{ 430.0, -25.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.qGai_flow,occupancy_2.y)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 375.0, 50.0 }    ,{ 375.0, 50.0 }    ,{ 350.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.surf_surBou[1],internal_space_1_space_3.port_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 300.0, 50.0 }    ,{ 200.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.surf_surBou[2],internal_space_2_space_3.port_b)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 350.0, 50.0 }    ,{ 300.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_3.weaBus,weather.weaBus)
annotation (Line(
points={{ 400.0, 50.0 }    ,{ 150.0, 50.0 }    ,{ 150.0, 200.0 }    ,{ -100.0, 200.0 }    },
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
smooth=Smooth.None));    connect(space_control_3.port,space_3.heaPorAir)
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
smooth=Smooth.None));    connect(pump.y,pump_control.y)
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
end buildings_simple_hydronic_three_zones;
