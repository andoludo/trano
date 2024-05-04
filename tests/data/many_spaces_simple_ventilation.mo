model many_spaces_simple_ventilation

            parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic double_glazing(
    final glass={
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003,
        k=1.0,
        tauSol={ 0.646},
        rhoSol_a={ 0.062},
        rhoSol_b={ 0.063},
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84),
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x=0.003,
        k=1.0,
        tauSol={ 0.646},
        rhoSol_a={ 0.062},
        rhoSol_b={ 0.063},
        tauIR=0.0,
        absIR_a=0.84,
        absIR_b=0.84)},
    final gas={
            Buildings.HeatTransfer.Data.Gases.Air(x=0.0127)},
    UFra=1.4)
    annotation (
    defaultComponentPrefixes="parameter",
    defaultComponentName="datGlaSys");

    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        internal_wall(
    final nLay=1,
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={Buildings.HeatTransfer.Data.Solids.Generic(
        x=0.2,
        k=0.89,
        c=790.0,
        d=1920.0)},
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));
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
        d=540.0)},
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={{20,84},{34,98}})));

package Medium = Buildings.Media.Air(extraPropertiesNames={"CO2"}) "Medium model";
package MediumW = Buildings.Media.Water "Medium model";

    Buildings.ThermalZones.Detailed.MixedAir space_1(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,nPorts = 3,                nConExt=2,
                datConExt(
                layers={ external_wall, external_wall},
    A={ 10.0, 10.0},
    til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
                azi={ 135.0, 45.0}),
                nSurBou=1,
                surBou(
                A={ 10.0},
                til={Buildings.Types.Tilt.Wall}),
                nConBou=1,
                datConBou(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Floor},
                azi={ 90.0}),
                nConExtWin=1,
                datConExtWin(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                glaSys={ double_glazing},
                wWin={ 1.0},
                hWin={ 1.0}),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(origin=
    { 0, 50}, extent={{-20,-20},{20,20}})));




        Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.Actuators.Dampers.Exponential vav_in(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { 4.989433438682065, 177.5946975301676},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.Actuators.Dampers.Exponential vav_out(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -181.57004985453727, -108.82559130029718},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.FixedResistances.PressureDrop pressure_drop_duct_out(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 73.66943843979595, 191.7410706200839},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.FixedResistances.PressureDrop pressure_drop_duct_in(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { -177.00415038359915, 14.554657841850414},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Controls.SpaceControls.PIDSubstance
    ventilation_control(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { 78.96641578897973, -172.18829299412963},
    extent = {{-10, -10}, {10, 10}})));
    Buildings.ThermalZones.Detailed.MixedAir space_2(
    redeclare package Medium = Medium,
    AFlo=50,
    hRoo=2,nPorts = 3,                nConExt=1,
                datConExt(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                azi={ 45.0}),
                nSurBou=1,
                surBou(
                A={ 10.0},
                til={Buildings.Types.Tilt.Wall}),
                nConBou=1,
                datConBou(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Floor},
                azi={ 90.0}),
                nConExtWin=1,
                datConExtWin(
                layers={ external_wall},
    A={ 10.0},
    til={Buildings.Types.Tilt.Wall},
                glaSys={ double_glazing},
                wWin={ 1.0},
                hWin={ 1.0}),
    nConPar=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    annotation (Placement(transformation(origin=
    { 200, 50}, extent={{-20,-20},{20,20}})));
        Neosim.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.Actuators.Dampers.Exponential vav_in_2(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { 24.5498700155019, -199.24967483028686},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.Actuators.Dampers.Exponential vav_out_2(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -182.12271304253724, -45.903240833027155},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.FixedResistances.PressureDrop pressure_drop_duct_out_2(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 131.36922435266223, -140.89249616347502},
    extent = {{-10, -10}, {10, 10}})));
      Buildings.Fluid.FixedResistances.PressureDrop pressure_drop_duct_in_1(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { -42.451833139062835, -193.81811872521473},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Controls.SpaceControls.PIDSubstance
    ventilation_control_2(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { 181.4588013781593, -94.4809260301075},
    extent = {{-10, -10}, {10, 10}})));
        Buildings.HeatTransfer.Conduction.MultiLayer internal_space_1_space_2(A=
            10, layers=
    internal_wall, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms" annotation (
    Placement(transformation(origin = { 100.0, 50},
    extent = {{-10, -10}, {10, 10}})));
        Buildings.BoundaryConditions.WeatherData.ReaderTMY3
            weather(filNam=
    Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/
    USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    annotation (
    Placement(transformation(origin = { -100, 200},
    extent = {{-10, -10}, {10, 10}})));
    Neosim.Fluid.Ventilation.SimpleHVACBuildings ahu(
     redeclare package Medium = Medium)
    annotation (
    Placement(transformation(origin = { -56.38670152351546, 184.9422783799691},
    extent = {{-10, -10}, {10, 10}})));


equation
            connect(space_1.qGai_flow,occupancy_0.y)
annotation (Line(
points={{-21.6,58},{-25,58},{-25,50},{-39,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[1],vav_out.port_a)
annotation (Line(
points={{-15,37.3333},{-90.785,37.3333},{-90.785,-108.826},{-191.57,-108.826}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.surf_surBou[1],internal_space_1_space_2.port_a)
annotation (Line(
points={{-3.8,36},{50,36},{50,50},{90,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.weaBus,weather.weaBus)
annotation (Line(
points={{17.9,67.9},{-50,67.9},{-50,200},{-90,200}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in.port_b,space_1.ports[2])
annotation (Line(
points={{14.9894,177.595},{2.49472,177.595},{2.49472,40},{-15,40}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_out.port_b,pressure_drop_duct_out.port_a)
annotation (Line(
points={{-171.57,-108.826},{-53.9503,-108.826},{-53.9503,191.741},{63.6694,
          191.741}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_out.port_b,ahu.port_a)
annotation (Line(
points={{83.6694,191.741},{8.64137,191.741},{8.64137,178.275},{-46.3867,178.275}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_in.port_b,vav_in.port_a)
annotation (Line(
points={{-167.004,14.5547},{-86.0074,14.5547},{-86.0074,177.595},{-5.01057,
          177.595}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.port_a,space_1.ports[3])
annotation (Line(
points={{68.9664,-172.188},{39.4832,-172.188},{39.4832,42.6667},{-15,42.6667}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_in.y)
annotation (Line(
points={{89.5664,-172.188},{41.9779,-172.188},{41.9779,189.595},{4.98943,
          189.595}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_out.y)
annotation (Line(
points={{89.5664,-172.188},{-51.3018,-172.188},{-51.3018,-96.826},{-181.57,
          -96.826}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.qGai_flow,occupancy_1.y)
annotation (Line(
points={{178.4,58},{175,58},{175,50},{161,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.ports[1],vav_out_2.port_a)
annotation (Line(
points={{185,37.3333},{8.93864,37.3333},{8.93864,-45.9032},{-192.123,-45.9032}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.surf_surBou[1],internal_space_1_space_2.port_b)
annotation (Line(
points={{196.2,36},{150,36},{150,50},{110,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.weaBus,weather.weaBus)
annotation (Line(
points={{217.9,67.9},{50,67.9},{50,200},{-90,200}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in_2.port_b,space_2.ports[2])
annotation (Line(
points={{34.5499,-199.25},{112.275,-199.25},{112.275,40},{185,40}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_out_2.port_b,pressure_drop_duct_out_2.port_a)
annotation (Line(
points={{-172.123,-45.9032},{-25.3767,-45.9032},{-25.3767,-140.892},{121.369,
          -140.892}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_out_2.port_b,ahu.port_a)
annotation (Line(
points={{141.369,-140.892},{37.4913,-140.892},{37.4913,178.275},{-46.3867,
          178.275}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_in_1.port_b,vav_in_2.port_a)
annotation (Line(
points={{-32.4518,-193.818},{-8.95098,-193.818},{-8.95098,-199.25},{14.5499,
          -199.25}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control_2.port_a,space_2.ports[3])
annotation (Line(
points={{171.459,-94.4809},{190.729,-94.4809},{190.729,42.6667},{185,42.6667}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control_2.y,vav_in_2.y)
annotation (Line(
points={{192.059,-94.4809},{103.004,-94.4809},{103.004,-187.25},{24.5499,
          -187.25}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control_2.y,vav_out_2.y)
annotation (Line(
points={{192.059,-94.4809},{-0.331956,-94.4809},{-0.331956,-33.9032},{-182.123,
          -33.9032}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,pressure_drop_duct_in.port_a)
annotation (Line(
points={{-46.3867,191.609},{-116.695,191.609},{-116.695,14.5547},{-187.004,
          14.5547}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,pressure_drop_duct_in_1.port_a)
annotation (Line(
points={{-46.3867,191.609},{-49.4193,191.609},{-49.4193,-193.818},{-52.4518,
          -193.818}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end many_spaces_simple_ventilation;
