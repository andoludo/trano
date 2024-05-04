
model ideas_many_spaces_simple_ventilation

package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
    record  double_glazing = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
      final checkLowPerformanceGlazing=false,
          mats={ideas_many_spaces_simple_ventilation.Data.Materials.id_100(
         d=0.003),ideas_many_spaces_simple_ventilation.Data.Materials.Air(
         d=0.0127),ideas_many_spaces_simple_ventilation.Data.Materials.id_100(
         d=0.003)},
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
      final g_value=0.78)
      "ideas_many_spaces_simple_ventilation";

end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;
                                                     record concrete =
          IDEAS.Buildings.Data.Interfaces.Material (
          k=1.4,
          c=840.0,
          rho=2240.0,
          epsLw=0.88,
          epsSw=0.55);
  record brick = IDEAS.Buildings.Data.Interfaces.Material (
   k=0.89,
        c=790.0,
        rho=1920.0,
        epsLw=0.88,
        epsSw=0.55);
                      record Air = IDEAS.Buildings.Data.Interfaces.Material (
          k=0.025,
          c=1005.0,
          rho=1.2,
          epsLw=0.88,
          epsSw=0.55);
  record id_100 = IDEAS.Buildings.Data.Interfaces.Material (
   k=1.0,
        c=840.0,
        rho=2500.0,
        epsLw=0.88,
        epsSw=0.55);
                      record plywood = IDEAS.Buildings.Data.Interfaces.Material
          (
          k=0.12,
          c=1210.0,
          rho=540.0,
          epsLw=0.88,
          epsSw=0.55);
  record insulation_board = IDEAS.Buildings.Data.Interfaces.Material (
   k=0.03,
        c=1200.0,
        rho=40.0,
        epsLw=0.88,
        epsSw=0.55);
end Materials;

package Constructions "Library of building envelope constructions"
                                                                        record internal_wall
        "internal_wall"
        extends IDEAS.Buildings.Data.Interfaces.Construction(mats={
              ideas_many_spaces_simple_ventilation.Data.Materials.brick(d=0.2)});
                                                                        end
        internal_wall;

  record external_wall
      "external_wall"
     extends IDEAS.Buildings.Data.Interfaces.Construction(
        mats={ideas_many_spaces_simple_ventilation.Data.Materials.concrete(
           d=0.2),ideas_many_spaces_simple_ventilation.Data.Materials.insulation_board(
           d=0.02),ideas_many_spaces_simple_ventilation.Data.Materials.plywood(
           d=0.1)});
  end external_wall;
end Constructions;
end Data;


replaceable package Medium = IDEAS.Media.Air(extraPropertiesNames={"CO2"})
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);
                                         inner IDEAS.BoundaryConditions.SimInfoManager
sim(interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));

    IDEAS.Buildings.Components.Zone space_1(
    mSenFac=0.822,nPorts = 3,    V=100,
    n50=0.822*0.5*space_1.n50toAch,
    redeclare package Medium = Medium,
    nSurf=5,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 0, 50},
    extent={{-20,-20},{20,20}})));
        IDEAS.Buildings.Components.OuterWall[2]
    merged_w1_1_w2_1(
    redeclare parameter ideas_many_spaces_simple_ventilation.Data.Constructions.external_wall
    constructionType,
    A={ 10, 10},
    final azi={ 135, 45},
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall, IDEAS.Types.Tilt.Wall})   annotation (
    Placement(transformation(origin = { -65.79231841400437, 193.22909173927894},  extent=
{{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_1(
    redeclare parameter ideas_many_spaces_simple_ventilation.Data.Glazing.double_glazing
                   glazing,
    A={ 1},
    final azi={ 45},
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall})   annotation (
    Placement(transformation(origin = { 41.40522510771586, 191.96588457383507},
      extent = {{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.SlabOnGround floor_2(
    redeclare parameter ideas_many_spaces_simple_ventilation.Data.Constructions.external_wall
                  constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation (
    Placement(transformation(origin = { 177.63468487315103, -108.29923582820068},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Occupancy.SimpleOccupancy occupancy_0 annotation (
    Placement(transformation(origin = { -50, 50},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.Actuators.Dampers.Exponential
    vav_in(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -9.236976761769942, -181.82934898342617},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.Actuators.Dampers.Exponential
    vav_out(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -112.06993538381604, 158.11255754099793},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_out(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { -191.75389294879727, 72.51463298041074},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_in(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 130.22960319952256, -132.83548582685046},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Controls.SpaceControls.PIDSubstance
    ventilation_control(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { 98.66206110275296, -174.5611378203883},
    extent = {{-10, -10}, {10, 10}})));
    IDEAS.Buildings.Components.Zone space_2(
    mSenFac=0.822,nPorts = 3,    V=100,
    n50=0.822*0.5*space_2.n50toAch,
    redeclare package Medium = Medium,
    nSurf=4,
    hZone=2,
    T_start=293.15)
    annotation (Placement(transformation(origin={ 200, 50},
    extent={{-20,-20},{20,20}})));
        IDEAS.Buildings.Components.OuterWall[1]
    merged_w2_2(
    redeclare parameter ideas_many_spaces_simple_ventilation.Data.Constructions.external_wall
    constructionType,
    A={ 10},
    final azi={ 45},
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall})   annotation (
    Placement(transformation(origin = { 102.15690506708246, 182.81853182740426},  extent=
{{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.Window[1]
    merged_win1_2(
    redeclare parameter ideas_many_spaces_simple_ventilation.Data.Glazing.double_glazing
                   glazing,
    A={ 1},
    final azi={ 45},
    redeclare package Medium = Medium,
    final inc={ IDEAS.Types.Tilt.Wall})   annotation (
    Placement(transformation(origin = { -194.57173427696108, -79.96498210903249},
      extent = {{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.SlabOnGround floor_3(
    redeclare parameter ideas_many_spaces_simple_ventilation.Data.Constructions.external_wall
                  constructionType,
    redeclare package Medium = Medium,
    A=10)  annotation (
    Placement(transformation(origin = { -178.64278193690072, 27.242474873717182},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Occupancy.SimpleOccupancy occupancy_1 annotation (
    Placement(transformation(origin = { 150, 50},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.Actuators.Dampers.Exponential
    vav_in_2(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { -12.713418066237496, 182.042701029512},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.Actuators.Dampers.Exponential
    vav_out_2(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    dpDamper_nominal=20,
    allowFlowReversal=false,
    dpFixed_nominal=130) "VAV box for room" annotation (
    Placement(transformation(origin = { 190.78257726442752, 47.375274563083174},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_out_2(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 190.66839018628332, -51.307920680886255},
    extent = {{-10, -10}, {10, 10}})));
      IDEAS.Fluid.FixedResistances.PressureDrop
    pressure_drop_duct_in_1(
    m_flow_nominal=1,
    redeclare package Medium = Medium,
    dp_nominal=40) "Pressure drop for return duct" annotation (
    Placement(transformation(origin = { 179.7588149054584, -1.600762832381784},
    extent = {{-10, -10}, {10, 10}})));
        Neosim.Controls.SpaceControls.PIDSubstance
    ventilation_control_2(redeclare package
      Medium = Medium) annotation (
    Placement(transformation(origin = { 136.0665570197406, 140.0535623687668},
    extent = {{-10, -10}, {10, 10}})));
        IDEAS.Buildings.Components.InternalWall internal_space_1_space_2(
     redeclare parameter ideas_many_spaces_simple_ventilation.Data.Constructions.internal_wall
                                     constructionType,
    redeclare package Medium = Medium,
    A = 10, inc = IDEAS.Types.Tilt.
    Wall, azi=
    10) "Partition wall between the two
    rooms" annotation (
    Placement(transformation(origin = { 100.0, 50},
    extent = {{-10, -10}, {10, 10}})));

    Neosim.Fluid.Ventilation.SimpleHVAC ahu(
     redeclare package Medium = Medium)
    annotation (
    Placement(transformation(origin = { -120.04929540345854, -173.32360895887476},
    extent = {{-10, -10}, {10, 10}})));


equation
            connect(space_1.propsBus[1:2],merged_w1_1_w2_1[1:2].propsBus_a)
annotation (Line(
points={{-20,59.6},{-32.8962,59.6},{-32.8962,195.229},{-57.459,195.229}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[3],merged_win1_1[1].propsBus_a)
annotation (Line(
points={{-20,58},{20.7026,58},{20.7026,193.966},{49.7386,193.966}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[4],floor_2.propsBus_a)
annotation (Line(
points={{-20,56.4},{88.8173,56.4},{88.8173,-106.299},{185.968,-106.299}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.yOcc,occupancy_0.y)
annotation (Line(
points={{24,58},{-25,58},{-25,50},{-39,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.ports[1],vav_out.port_a)
annotation (Line(
points={{5.33333,70},{-56.035,70},{-56.035,158.113},{-122.07,158.113}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_1.propsBus[5],internal_space_1_space_2.propsBus_a)
annotation (Line(
points={{-20,54.8},{50,54.8},{50,52},{108.333,52}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in.port_b,space_1.ports[2])
annotation (Line(
points={{0.763023,-181.829},{-4.61849,-181.829},{-4.61849,70},{0,70}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_out.port_b,pressure_drop_duct_out.port_a)
annotation (Line(
points={{-102.07,158.113},{-151.912,158.113},{-151.912,72.5146},{-201.754,
          72.5146}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_out.port_b,ahu.port_a)
annotation (Line(
points={{-181.754,72.5146},{-155.902,72.5146},{-155.902,-179.99},{-110.049,
          -179.99}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_in.port_b,vav_in.port_a)
annotation (Line(
points={{140.23,-132.835},{60.4963,-132.835},{60.4963,-181.829},{-19.237,
          -181.829}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.port_a,space_1.ports[3])
annotation (Line(
points={{88.6621,-174.561},{49.331,-174.561},{49.331,70},{-5.33333,70}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_in.y)
annotation (Line(
points={{109.262,-174.561},{44.7125,-174.561},{44.7125,-169.829},{-9.23698,
          -169.829}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control.y,vav_out.y)
annotation (Line(
points={{109.262,-174.561},{-6.70394,-174.561},{-6.70394,170.113},{-112.07,
          170.113}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[1],merged_w2_2[1].propsBus_a)
annotation (Line(
points={{180,61},{151.078,61},{151.078,184.819},{110.49,184.819}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[2],merged_win1_2[1].propsBus_a)
annotation (Line(
points={{180,59},{2.71413,59},{2.71413,-77.965},{-186.238,-77.965}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[3],floor_3.propsBus_a)
annotation (Line(
points={{180,57},{10.6786,57},{10.6786,29.2425},{-170.309,29.2425}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.yOcc,occupancy_1.y)
annotation (Line(
points={{224,58},{175,58},{175,50},{161,50}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.ports[1],vav_out_2.port_a)
annotation (Line(
points={{205.333,70},{195.391,70},{195.391,47.3753},{180.783,47.3753}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(space_2.propsBus[4],internal_space_1_space_2.propsBus_b)
annotation (Line(
points={{180,55},{150,55},{150,52},{91.6667,52}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_in_2.port_b,space_2.ports[2])
annotation (Line(
points={{-2.71342,182.043},{93.6433,182.043},{93.6433,70},{200,70}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(vav_out_2.port_b,pressure_drop_duct_out_2.port_a)
annotation (Line(
points={{200.783,47.3753},{190.725,47.3753},{190.725,-51.3079},{180.668,
          -51.3079}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_out_2.port_b,ahu.port_a)
annotation (Line(
points={{200.668,-51.3079},{35.3095,-51.3079},{35.3095,-179.99},{-110.049,
          -179.99}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(pressure_drop_duct_in_1.port_b,vav_in_2.port_a)
annotation (Line(
points={{189.759,-1.60076},{83.5227,-1.60076},{83.5227,182.043},{-22.7134,
          182.043}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control_2.port_a,space_2.ports[3])
annotation (Line(
points={{126.067,140.054},{168.033,140.054},{168.033,70},{194.667,70}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control_2.y,vav_in_2.y)
annotation (Line(
points={{146.667,140.054},{61.6766,140.054},{61.6766,194.043},{-12.7134,194.043}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ventilation_control_2.y,vav_out_2.y)
annotation (Line(
points={{146.667,140.054},{163.425,140.054},{163.425,59.3753},{190.783,59.3753}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,pressure_drop_duct_in.port_a)
annotation (Line(
points={{-110.049,-166.657},{5.09015,-166.657},{5.09015,-132.835},{120.23,
          -132.835}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));    connect(ahu.port_b,pressure_drop_duct_in_1.port_a)
annotation (Line(
points={{-110.049,-166.657},{29.8548,-166.657},{29.8548,-1.60076},{169.759,
          -1.60076}},
color={255,204,51},
thickness=0.5,
smooth=Smooth.None));


end ideas_many_spaces_simple_ventilation;
