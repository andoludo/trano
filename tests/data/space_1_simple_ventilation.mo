model space_1_simple_ventilation

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

package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";

    Buildings.ThermalZones.Detailed.MixedAir space_1(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,nPorts = 3,                nConExt=2,
                datConExt(
                layers={ external_wall, external_wall },
    A={ 10.0, 10.0 },
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 45.0 }),
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




        Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50 },
    extent = {{-10, -10}, {10, 10}}
)));
      Buildings.Fluid.Actuators.Dampers.Exponential vav_in(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { 114.95094984848205, 127.53647272239847 },
    extent = {{-10, -10}, {10, 10}}
)));
      Buildings.Fluid.Actuators.Dampers.Exponential vav_out(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -136.66822719532144, 148.03690441039566 },
    extent = {{-10, -10}, {10, 10}}
)));
      Buildings.Fluid.FixedResistances.PressureDrop pressure_drop_duct_out(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 163.56001971085777, -106.73976150294159 },
    extent = {{-10, -10}, {10, 10}}
)));
      Buildings.Fluid.FixedResistances.PressureDrop pressure_drop_duct_in(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 200.0, -15.389808933980332 },
    extent = {{-10, -10}, {10, 10}}
)));
        Neosim.Controls.SpaceControls.PIDSubstance
    ventilation_control(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { -87.24263898896751, -168.19638874978267 },
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
    Neosim.Fluid.Ventilation.SimpleHVACBuildings ahu
    (redeclare package Medium = Medium)
    annotation (
    Placement(transformation(origin = { -171.38275785446933, 62.67843122291192 },
    extent = {{-10, -10}, {10, 10}}
)));


equation    connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -25.0, 50.0 }    ,{ -50.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[1],vav_out.port_a)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -68.33411359766072, 50.0 }    ,{ -68.33411359766072, 148.03690441039566 }    ,{ -136.66822719532144, 148.03690441039566 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.weaBus,weather.weaBus)
annotation (Line(
points={{ 0.0, 50.0 }    ,{ -50.0, 50.0 }    ,{ -50.0, 200.0 }    ,{ -100.0, 200.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in.port_b,space_1.ports[2])
annotation (Line(
points={{ 114.95094984848205, 127.53647272239847 }    ,{ 57.47547492424103, 127.53647272239847 }    ,{ 57.47547492424103, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_out.port_b,pressure_drop_duct_out.port_a)
annotation (Line(
points={{ -136.66822719532144, 148.03690441039566 }    ,{ 13.445896257768169, 148.03690441039566 }    ,{ 13.445896257768169, -106.73976150294159 }    ,{ 163.56001971085777, -106.73976150294159 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_out.port_b,ahu.port_a)
annotation (Line(
points={{ 163.56001971085777, -106.73976150294159 }    ,{ -3.9113690718057796, -106.73976150294159 }    ,{ -3.9113690718057796, 62.67843122291192 }    ,{ -171.38275785446933, 62.67843122291192 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_in.port_b,vav_in.port_a)
annotation (Line(
points={{ 200.0, -15.389808933980332 }    ,{ 157.47547492424104, -15.389808933980332 }    ,{ 157.47547492424104, 127.53647272239847 }    ,{ 114.95094984848205, 127.53647272239847 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.port_a,space_1.ports[3])
annotation (Line(
points={{ -87.24263898896751, -168.19638874978267 }    ,{ -43.62131949448376, -168.19638874978267 }    ,{ -43.62131949448376, 50.0 }    ,{ 0.0, 50.0 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_in.y)
annotation (Line(
points={{ -87.24263898896751, -168.19638874978267 }    ,{ 13.85415542975727, -168.19638874978267 }    ,{ 13.85415542975727, 127.53647272239847 }    ,{ 114.95094984848205, 127.53647272239847 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_out.y)
annotation (Line(
points={{ -87.24263898896751, -168.19638874978267 }    ,{ -111.95543309214447, -168.19638874978267 }    ,{ -111.95543309214447, 148.03690441039566 }    ,{ -136.66822719532144, 148.03690441039566 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,pressure_drop_duct_in.port_a)
annotation (Line(
points={{ -171.38275785446933, 62.67843122291192 }    ,{ 14.308621072765334, 62.67843122291192 }    ,{ 14.308621072765334, -15.389808933980332 }    ,{ 200.0, -15.389808933980332 }    },
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end space_1_simple_ventilation;
