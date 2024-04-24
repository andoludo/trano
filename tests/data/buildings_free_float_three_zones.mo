model buildings_free_float_three_zones
package Medium = Buildings.Media.Air "Medium model";
package MediumW = Buildings.Media.Water "Medium model";

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
    AFlo=10,
    hRoo=10,                nConExt=4,
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
                nConExtWin=2,
                datConExtWin(
                layers={ external_wall, external_wall },
    A={ 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                glaSys={ double_glazing, double_glazing },
                wWin={ 1.0, 1.0 },
                hWin={ 1.0, 1.0 }),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(origin={ 0, 50 },extent={{-20,-20},{20,20}}
)));Buildings.ThermalZones.Detailed.MixedAir space_2(
    redeclare package Medium = Medium,
    AFlo=10,
    hRoo=10,                nConExt=3,
                datConExt(
                layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 0.0, 90.0 }),
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
                nConExtWin=2,
                datConExtWin(
                layers={ external_wall, external_wall },
    A={ 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                glaSys={ double_glazing, double_glazing },
                wWin={ 1.0, 1.0 },
                hWin={ 1.0, 1.0 }),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(origin={ 200, 50 },extent={{-20,-20},{20,20}}
)));Buildings.ThermalZones.Detailed.MixedAir space_3(
    redeclare package Medium = Medium,
    AFlo=10,
    hRoo=10,                nConExt=3,
                datConExt(
                layers={ external_wall, external_wall, external_wall },
    A={ 10.0, 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 0.0, 45.0 }),
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
)));Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50 }, extent = {{-10, -10}, {10, 10}}
)));Neosim.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50 }, extent = {{-10, -10}, {10, 10}}
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
)));equation    connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -50.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
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
smooth=Smooth.None));
end buildings_free_float_three_zones;
