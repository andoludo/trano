
## Occupancy

### Variant: default from REDUCED_ORDER
The following template is used for this component:
```jinja

    {{package_name}}.Trano.Occupancy.SimpleOccupancy
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



## Space

### Variant: default from REDUCED_ORDER
The following template is used for this component:
```jinja
{%- set zone = element.parameters %}
{%- set occ = element.occupancy.parameters if element.occupancy else none %}
{%- set hInt = zone.interior_convection_coefficient if zone.interior_convection_coefficient is not none else 2.7 %}
{%- set hExt = zone.exterior_convection_coefficient if zone.exterior_convection_coefficient is not none else 20.0 %}
{%- set shading = zone.shading_reduction_factor if zone.shading_reduction_factor is not none else 0.7 %}
{%- set maxIrr = zone.sunblind_irradiance_threshold if zone.sunblind_irradiance_threshold is not none else 100 %}
{%- set activity = occ.activity_degree if occ and occ.activity_degree is not none else 1.2 %}
{%- set lighting = occ.lighting_power_density if occ and occ.lighting_power_density is not none else 0 %}
{%- set lightingRad = occ.lighting_radiant_fraction if occ and occ.lighting_radiant_fraction is not none else 0.4 %}
{%- set equipment = occ.equipment_power_density if occ and occ.equipment_power_density is not none else 0 %}
{%- set equipmentRad = occ.equipment_radiant_fraction if occ and occ.equipment_radiant_fraction is not none else 0.4 %}
AixLib.ThermalZones.ReducedOrder.ThermalZone.ThermalZone {{ element.name }}(
{#- A constant outdoor air exchange is dry: AixLib's moisture balance fails on the humidity it brings in. #}
    use_moisture_balance={{ "false" if element.variant == "infiltration" else "true" }},
    {#- The air exchange of the record only exists when the natural air exchange is switched on. #}
    {%- if element.variant == "infiltration" %}
    use_NaturalAirExchange=true,
    {%- endif %}
    ROM(extWallRC(thermCapExt(each der_T(fixed=true))), intWallRC(thermCapInt(
            each der_T(fixed=true)))),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    internalGainsMode=2,
    nPorts=2,  
    zoneParam=AixLib.DataBase.ThermalZones.ZoneBaseRecord(
  T_start=293.15,
{{ macros.render_parameters(parameters) | safe }},
    hRad=5,
    lat=0.87266462599716,
    nOrientations={{ element.aggregated_envelope.orientations | length }},
    AWin={{ macros.join_list(element.aggregated_envelope.window_areas) }},
    ATransparent={{ macros.join_list(element.aggregated_envelope.window_areas) }},
    RWin={{ element.aggregated_envelope.windows.resistance }},
    hConWin={{ hInt }},
    gWin={{ element.aggregated_envelope.windows.g_value }},
    UWin={{ element.aggregated_envelope.windows.u_value }},
    ratioWinConRad=0.09,

    AExt={{ macros.join_list(element.aggregated_envelope.opaque_areas) }},
    hConExt={{ hInt }},
    nExt=1,
    RExt={ {{- element.aggregated_envelope.exterior_walls.resistance -}} },
    RExtRem={{ element.aggregated_envelope.exterior_walls.resistance_remaining }},
    CExt={ {{- element.aggregated_envelope.exterior_walls.capacitance -}} },

    AInt=0,
    hConInt=2.27,
    nInt=1,
    RInt={1.13047235829e-05},
    CInt={1402628013.98},

    AFloor={{ element.aggregated_envelope.floor.area }},
    hConFloor={{ hInt }},
    nFloor=1,
    RFloor={ {{- element.aggregated_envelope.floor.resistance -}} },
    RFloorRem={{ element.aggregated_envelope.floor.resistance_remaining }},
    CFloor={ {{- element.aggregated_envelope.floor.capacitance -}} },

    ARoof={{ element.aggregated_envelope.roof.area }},
    hConRoof={{ hInt }},
    nRoof=1,
    RRoof={ {{- element.aggregated_envelope.roof.resistance -}} },
    RRoofRem={{ element.aggregated_envelope.roof.resistance_remaining }},
    CRoof={ {{- element.aggregated_envelope.roof.capacitance -}} },
    nOrientationsRoof={{ element.aggregated_envelope.roof_orientations | length }},
    tiltRoof={{ macros.join_list(element.aggregated_envelope.roof_tilts) }},
    aziRoof={{ macros.join_list(element.aggregated_envelope.roof_azimuths) }},
    wfRoof={{ macros.join_list(element.aggregated_envelope.roof_weighting_factors) }},
    aRoof=0.7,

    aExt=0.7,
    TSoil={{ element.aggregated_envelope.ground_temperature }},
    hConWallOut={{ hExt }},
    hRadWall=5,
    hConWinOut={{ hExt }},
    hConRoofOut={{ hExt }},
    hRadRoof=5,
    tiltExtWalls={{ macros.join_list(element.aggregated_envelope.tilts) }},
    aziExtWalls={{ macros.join_list(element.aggregated_envelope.azimuths) }},
    wfWall={{ macros.join_list(element.aggregated_envelope.wall_weighting_factors) }},
    wfWin={{ macros.join_list(element.aggregated_envelope.window_weighting_factors) }},
    wfGro=0,
{% if element.occupancy %}
    specificPeople={{ element.occupancy.parameters.heat_gain_if_occupied }},
    fixedHeatFlowRatePersons={{ element.occupancy.gains_per_person.sensible }},
    ratioConvectiveHeatPeople={{ (1 - element.occupancy.gains_per_person.radiant_fraction) | round(6) }},
{% else %}
    specificPeople=0,
    fixedHeatFlowRatePersons=0,
    ratioConvectiveHeatPeople=0.5,
{% endif %}
    activityDegree={{ activity }},
    internalGainsMoistureNoPeople=0.5,
    internalGainsMachinesSpecific={{ equipment }},
    ratioConvectiveHeatMachines={{ (1 - equipmentRad) | round(6) }},
    lightingPowerSpecific={{ lighting }},
    ratioConvectiveHeatLighting={{ (1 - lightingRad) | round(6) }},
{%- if element.variant == "infiltration" and element.parameters.ach is not none %}
    useConstantACHrate=true,
    baseACH={{ element.parameters.ach }},
{%- else %}
    useConstantACHrate=false,
    baseACH=0.2,
{%- endif %}
    maxUserACH=1,
    maxOverheatingACH={3.0,2.0},
maxSummerACH={1.0,273.15 + 10,273.15 + 17},
winterReduction={0.2,273.15,273.15 + 10},
    withAHU=false,
    minAHU=0,
    maxAHU=12,
    maxIrr = {{ macros.join_list([maxIrr] * (element.aggregated_envelope.orientations | length)) }},
    shadingFactor = {{ macros.join_list([shading] * (element.aggregated_envelope.orientations | length)) }},
    hHeat=167500,
    lHeat=0,
    KRHeat=1000,
    TNHeat=1,
    HeaterOn=false,
    hCool=0,
    lCool=-1,
    heaLoadFacOut=0,
    heaLoadFacGrd=0,
    KRCool=1000,
    TNCool=1,
    CoolerOn=false,
TThresholdHeater=273.15 + 15,
TThresholdCooler=273.15 + 22,
    withIdealThresholds=false))

```



### Variant: infiltration from REDUCED_ORDER
The following template is used for this component:
```jinja
{%- set zone = element.parameters %}
{%- set occ = element.occupancy.parameters if element.occupancy else none %}
{%- set hInt = zone.interior_convection_coefficient if zone.interior_convection_coefficient is not none else 2.7 %}
{%- set hExt = zone.exterior_convection_coefficient if zone.exterior_convection_coefficient is not none else 20.0 %}
{%- set shading = zone.shading_reduction_factor if zone.shading_reduction_factor is not none else 0.7 %}
{%- set maxIrr = zone.sunblind_irradiance_threshold if zone.sunblind_irradiance_threshold is not none else 100 %}
{%- set activity = occ.activity_degree if occ and occ.activity_degree is not none else 1.2 %}
{%- set lighting = occ.lighting_power_density if occ and occ.lighting_power_density is not none else 0 %}
{%- set lightingRad = occ.lighting_radiant_fraction if occ and occ.lighting_radiant_fraction is not none else 0.4 %}
{%- set equipment = occ.equipment_power_density if occ and occ.equipment_power_density is not none else 0 %}
{%- set equipmentRad = occ.equipment_radiant_fraction if occ and occ.equipment_radiant_fraction is not none else 0.4 %}
AixLib.ThermalZones.ReducedOrder.ThermalZone.ThermalZone {{ element.name }}(
{#- A constant outdoor air exchange is dry: AixLib's moisture balance fails on the humidity it brings in. #}
    use_moisture_balance={{ "false" if element.variant == "infiltration" else "true" }},
    {#- The air exchange of the record only exists when the natural air exchange is switched on. #}
    {%- if element.variant == "infiltration" %}
    use_NaturalAirExchange=true,
    {%- endif %}
    ROM(extWallRC(thermCapExt(each der_T(fixed=true))), intWallRC(thermCapInt(
            each der_T(fixed=true)))),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    internalGainsMode=2,
    nPorts=2,  
    zoneParam=AixLib.DataBase.ThermalZones.ZoneBaseRecord(
  T_start=293.15,
{{ macros.render_parameters(parameters) | safe }},
    hRad=5,
    lat=0.87266462599716,
    nOrientations={{ element.aggregated_envelope.orientations | length }},
    AWin={{ macros.join_list(element.aggregated_envelope.window_areas) }},
    ATransparent={{ macros.join_list(element.aggregated_envelope.window_areas) }},
    RWin={{ element.aggregated_envelope.windows.resistance }},
    hConWin={{ hInt }},
    gWin={{ element.aggregated_envelope.windows.g_value }},
    UWin={{ element.aggregated_envelope.windows.u_value }},
    ratioWinConRad=0.09,

    AExt={{ macros.join_list(element.aggregated_envelope.opaque_areas) }},
    hConExt={{ hInt }},
    nExt=1,
    RExt={ {{- element.aggregated_envelope.exterior_walls.resistance -}} },
    RExtRem={{ element.aggregated_envelope.exterior_walls.resistance_remaining }},
    CExt={ {{- element.aggregated_envelope.exterior_walls.capacitance -}} },

    AInt=0,
    hConInt=2.27,
    nInt=1,
    RInt={1.13047235829e-05},
    CInt={1402628013.98},

    AFloor={{ element.aggregated_envelope.floor.area }},
    hConFloor={{ hInt }},
    nFloor=1,
    RFloor={ {{- element.aggregated_envelope.floor.resistance -}} },
    RFloorRem={{ element.aggregated_envelope.floor.resistance_remaining }},
    CFloor={ {{- element.aggregated_envelope.floor.capacitance -}} },

    ARoof={{ element.aggregated_envelope.roof.area }},
    hConRoof={{ hInt }},
    nRoof=1,
    RRoof={ {{- element.aggregated_envelope.roof.resistance -}} },
    RRoofRem={{ element.aggregated_envelope.roof.resistance_remaining }},
    CRoof={ {{- element.aggregated_envelope.roof.capacitance -}} },
    nOrientationsRoof={{ element.aggregated_envelope.roof_orientations | length }},
    tiltRoof={{ macros.join_list(element.aggregated_envelope.roof_tilts) }},
    aziRoof={{ macros.join_list(element.aggregated_envelope.roof_azimuths) }},
    wfRoof={{ macros.join_list(element.aggregated_envelope.roof_weighting_factors) }},
    aRoof=0.7,

    aExt=0.7,
    TSoil={{ element.aggregated_envelope.ground_temperature }},
    hConWallOut={{ hExt }},
    hRadWall=5,
    hConWinOut={{ hExt }},
    hConRoofOut={{ hExt }},
    hRadRoof=5,
    tiltExtWalls={{ macros.join_list(element.aggregated_envelope.tilts) }},
    aziExtWalls={{ macros.join_list(element.aggregated_envelope.azimuths) }},
    wfWall={{ macros.join_list(element.aggregated_envelope.wall_weighting_factors) }},
    wfWin={{ macros.join_list(element.aggregated_envelope.window_weighting_factors) }},
    wfGro=0,
{% if element.occupancy %}
    specificPeople={{ element.occupancy.parameters.heat_gain_if_occupied }},
    fixedHeatFlowRatePersons={{ element.occupancy.gains_per_person.sensible }},
    ratioConvectiveHeatPeople={{ (1 - element.occupancy.gains_per_person.radiant_fraction) | round(6) }},
{% else %}
    specificPeople=0,
    fixedHeatFlowRatePersons=0,
    ratioConvectiveHeatPeople=0.5,
{% endif %}
    activityDegree={{ activity }},
    internalGainsMoistureNoPeople=0.5,
    internalGainsMachinesSpecific={{ equipment }},
    ratioConvectiveHeatMachines={{ (1 - equipmentRad) | round(6) }},
    lightingPowerSpecific={{ lighting }},
    ratioConvectiveHeatLighting={{ (1 - lightingRad) | round(6) }},
{%- if element.variant == "infiltration" and element.parameters.ach is not none %}
    useConstantACHrate=true,
    baseACH={{ element.parameters.ach }},
{%- else %}
    useConstantACHrate=false,
    baseACH=0.2,
{%- endif %}
    maxUserACH=1,
    maxOverheatingACH={3.0,2.0},
maxSummerACH={1.0,273.15 + 10,273.15 + 17},
winterReduction={0.2,273.15,273.15 + 10},
    withAHU=false,
    minAHU=0,
    maxAHU=12,
    maxIrr = {{ macros.join_list([maxIrr] * (element.aggregated_envelope.orientations | length)) }},
    shadingFactor = {{ macros.join_list([shading] * (element.aggregated_envelope.orientations | length)) }},
    hHeat=167500,
    lHeat=0,
    KRHeat=1000,
    TNHeat=1,
    HeaterOn=false,
    hCool=0,
    lCool=-1,
    heaLoadFacOut=0,
    heaLoadFacGrd=0,
    KRCool=1000,
    TNCool=1,
    CoolerOn=false,
TThresholdHeater=273.15 + 15,
TThresholdCooler=273.15 + 22,
    withIdealThresholds=false))

```



## DataBus

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    {{package_name}}.Components.BaseClasses.DataServer
        {{ element.name }} (redeclare package
          Medium = Medium)
```



## ExternalWall

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{ macros.convert_tilts(element.tilts, library_name) }})
```



## ExternalDoor

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{ macros.convert_tilts(element.tilts, library_name) }})
```



## Window

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.Window[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Glazing.
    {{ element.constructions[0].name }} glazing,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    frac={{ macros.join_list(element.frame_fractions) }},
    redeclare parameter IDEAS.Buildings.Data.Frames.Wood fraType(each U_value={{ element.constructions[0].u_value_frame }}),
    {%- if element.overhang and element.side_fins %}
    redeclare IDEAS.Buildings.Components.Shading.Box shaType(
        each hWin={{ element.heights[0] }}, each wWin={{ element.widths[0] }},
        each wLeft={{ element.overhang.width_left }}, each wRight={{ element.overhang.width_right }},
        each ovDep={{ element.overhang.depth }}, each ovGap={{ element.overhang.gap }},
        each hFin={{ element.side_fins.height }}, each finDep={{ element.side_fins.depth }},
        each finGap={{ element.side_fins.gap }}),
    {%- elif element.overhang %}
    redeclare IDEAS.Buildings.Components.Shading.Overhang shaType(
        each hWin={{ element.heights[0] }}, each wWin={{ element.widths[0] }},
        each wLeft={{ element.overhang.width_left }}, each wRight={{ element.overhang.width_right }},
        each dep={{ element.overhang.depth }}, each gap={{ element.overhang.gap }}),
    {%- elif element.side_fins %}
    redeclare IDEAS.Buildings.Components.Shading.SideFins shaType(
        each hWin={{ element.heights[0] }}, each wWin={{ element.widths[0] }},
        each hFin={{ element.side_fins.height }}, each dep={{ element.side_fins.depth }},
        each gap={{ element.side_fins.gap }}),
    {%- endif %}
    final inc={{ macros.convert_tilts(element.tilts, library_name) }})
```



## FloorOnGround

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.SlabOnGround {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    {%- if element.perimeter is not none %}
    PWall={{ element.perimeter }},
    {%- endif %}
    {%- if element.average_indoor_temperature is not none %}
    TiAvg={{ element.average_indoor_temperature }},
    {%- endif %}
    A={{  element.surface}})
```



### Variant: outdoor_air from IDEAS
The following template is used for this component:
```jinja

    {{ package_name }}.Trano.ThermalZones.BoundaryWallOutdoorAir {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    inc=IDEAS.Types.Tilt.Floor,
    azi=0,
    A={{  element.surface}})
```



## AdiabaticWall

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.BoundaryWall {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    use_T_in=false,
    use_Q_in=false,
    inc={{ macros.convert_tilt(element.tilt, library_name) }},
    azi={{ element.azimuth }},
    A={{ element.surface }})
```



## InternalElement

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.InternalWall {{ element.name }}
    (redeclare parameter {{ package_name }}.
    Data.Constructions.{{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    A = {{ element.surface }}, inc = {{ macros.convert_tilt(element.tilt, library_name) }}, azi =
    {{ element.azimuth }}) "Partition wall between the two
    rooms"
```



## Photovoltaic

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{ package_name }}.Components.BaseClasses.Photovoltaic{{ element.name | capitalize }} {{ element.name }}({{ macros.render_parameters(parameters) | safe }})

```



## EmissionControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.EmissionControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



### Variant: onoff from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.EmissionControlOnOff{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



### Variant: PController from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.EmissionControlPController{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



### Variant: MonitoredSetpoint from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.EmissionControlMonitoredSetpoint{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



## BoilerControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



### Variant: complex from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



### Variant: simple from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}
```



## ThreeWayValveControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.
    ThreeWayValveControl{{ element.name | capitalize}}
    {{ element.name }}
```



## CollectorControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.CollectorControl{{ element.name | capitalize}}
    {{ element.name }}
```



## AhuControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.AhuControl{{ element.name | capitalize}}
    {{ element.name }}
```



## Radiator

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    {{library_name}}.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 {{ element.name }}(
            {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator" 
```



### Variant: idealbus from DEFAULT
The following template is used for this component:
```jinja

      {{ package_name }}.Components.BaseClasses.IdealHeatingBus{{ element.name | capitalize }}
         {{ element.name }}({{ macros.render_parameters(parameters) | safe }})
```



### Variant: ideal from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Trano.HeatTransfer.IdealHeatingSystem.IdealHeatEmission
    {{ element.name }}({{ macros.render_parameters(parameters) | safe }})
```



## Valve

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{ library_name }}.Fluid.Actuators.Valves.TwoWayEqualPercentage
        {{ element.name }}(
            {{ macros.render_parameters(parameters) | safe}},
{%- if element.parameters.flow_coefficient_type %}
CvData={{ library_name }}.Fluid.Types.CvTypes.{{ element.parameters.flow_coefficient_type }},
{%- endif %}
redeclare package Medium = MediumW

) "Radiator valve" 
```



## ThreeWayValve

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{library_name}}.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
         {{ element.name }}(
redeclare package Medium = MediumW,
  {{ macros.render_parameters(parameters) | safe}},
{%- if element.parameters.flow_coefficient_type %}
CvData={{ library_name }}.Fluid.Types.CvTypes.{{ element.parameters.flow_coefficient_type }},
{%- endif %}
energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve" 
```



## SplitValve

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    {{library_name}}.Fluid.FixedResistances.Junction {{ element.name }} (
    {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter" 
```



## Pump

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
  {{ package_name }}.Components.BaseClasses.Pump{{ element.name | capitalize }}
     {{ element.name }}(
     {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW

    )
```



## TemperatureSensor

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    Buildings.Fluid.Sensors.TemperatureTwoPort {{ element.name }}(
    redeclare package Medium = MediumW,
    {{ macros.render_parameters(parameters) | safe }})
```



## HeatMeterSensor

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    Buildings.Fluid.Sensors.HeatMeter {{ element.name }}(
    redeclare package Medium = MediumW,
    {{ macros.render_parameters(parameters) | safe }})
```



## Boiler

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

{{package_name}}.Components.BaseClasses.
BoilerWithStorage{{ element.name | capitalize}} {{ element.name }}(
{{ macros.render_parameters(parameters) | safe}},
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.{{ element.parameters.fuel or 'NaturalGasLowerHeatingValue' }}()) "Boiler" 
```



### Variant: without_storage from DEFAULT
The following template is used for this component:
```jinja

{{package_name}}.Components.BaseClasses.
BoilerWithoutStorage{{ element.name | capitalize}} {{ element.name }}(
{{ macros.render_parameters(parameters) | safe}},
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.{{ element.parameters.fuel or 'NaturalGasLowerHeatingValue' }}()) "Boiler" 
```



### Variant: air_water_heat_pump from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Components.BaseClasses.AirWaterHeatPump{{ element.name | capitalize}} {{ element.name }}(
{{ macros.heat_pump(element, parameters, air=true) }},
{{ macros.render_parameters(parameters) | safe}},
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.{{ element.parameters.fuel or 'NaturalGasLowerHeatingValue' }}()) "Air-water heat pump"
```



### Variant: ground_source_heat_pump from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Components.BaseClasses.GroundSourceHeatPump{{ element.name | capitalize}} {{ element.name }}(
{{ macros.heat_pump(element, parameters, air=false) }},
{{ macros.render_parameters(parameters) | safe}},
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.{{ element.parameters.fuel or 'NaturalGasLowerHeatingValue' }}()) "Ground-source heat pump"
```



### Variant: water_water_heat_pump from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Components.BaseClasses.WaterWaterHeatPump{{ element.name | capitalize}} {{ element.name }}(
{{ macros.heat_pump(element, parameters, air=false) }},
{{ macros.render_parameters(parameters) | safe}},
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.{{ element.parameters.fuel or 'NaturalGasLowerHeatingValue' }}()) "Water-water heat pump"
```



## VAV

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{ library_name }}.Fluid.Actuators.Dampers.PressureIndependent
    {{ element.name }}(
    redeclare package Medium = Medium,
    {{ macros.render_parameters(parameters) | safe }},
    allowFlowReversal=false) "VAV box for room" 
```



### Variant: complex from DEFAULT
The following template is used for this component:
```jinja
  {{ package_name }}.Components.BaseClasses.VAVBox{{ element.name | capitalize }}
     {{ element.name }}(
    redeclare package MediumA = Medium,
    {{ macros.render_parameters(parameters) | safe }},
    allowFlowReversal=false
    )
```



## VAVControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Components.BaseClasses.VAVControl{{ element.name | capitalize}}
    {{ element.name }}
```



### Variant: constant from DEFAULT
The following template is used for this component:
```jinja
    Modelica.Blocks.Sources.RealExpression {{ element.name }}(y=1)
```



## PowerSensor

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{ package_name }}.Components.BaseClasses.PowerSensor{{ element.name | capitalize}}
       {{ element.name }}
```



## AirHandlingUnit

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Components.BaseClasses.Ahu{{ element.name | capitalize}}
    {{ element.name }}
    (redeclare package MediumA = Medium,
    {%- for key, value in parameters.items() if value is not none %}
    {{ key }}={{ value | safe }},
    {%- endfor %}
    {% raw %}
    VRoo={100,100},
    AFlo={20,20},
    mCooVAV_flow_nominal={0.01,0.01}{% endraw %})
```



### Variant: test from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Trano.Fluid.
    Ventilation.SimpleHVACBuildings
    {{ element.name }}
    (redeclare package Medium = Medium, {{ macros.render_parameters(parameters) | safe}})
```



### Variant: system_d from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Components.BaseClasses.SystemD{{ element.name | capitalize}}
    {{ element.name }}
    (redeclare package Medium = Medium, {{ macros.render_parameters(parameters) | safe}},
    usePostHeater={{ 'true' if element.parameters.supply_air_minimum_temperature is not none else 'false' }})
```



### Variant: system_c from DEFAULT
The following template is used for this component:
```jinja
{%- set rendered = macros.render_parameters(parameters) %}
{{package_name}}.Components.BaseClasses.SystemC{{ element.name | capitalize}}
    {{ element.name }}
    (redeclare package Medium = Medium{{ (', ' ~ rendered) | safe if rendered }})
```



## Weather

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    inner IDEAS.BoundaryConditions.SimInfoManager {{ element.name }}(
    interZonalAirFlowType=IDEAS.BoundaryConditions.Types.InterZonalAirFlow.{{ element.interzonal_airflow }},
    linIntRad={{ "true" if element.linearize_radiation else "false" }},
    linExtRad={{ "true" if element.linearize_radiation else "false" }}
    {%- for key, value in parameters.items() if value is not none %},
    {{ key }}={{ value | safe }}
    {%- endfor %}) "Data reader"
```



## Boundary

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
  {{ library_name }}.Fluid.Sources.Outside {{ element.name }}
    (nPorts=2,redeclare package Medium = Medium)
```



## IdealHeatingCooling

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{ package_name }}.Trano.HeatTransfer.IdealHeatingSystem.IdealHeatingCooling
    {{ element.name }}({{ macros.render_parameters(parameters) | safe }})
```



## Duct

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{ library_name }}.Fluid.FixedResistances.PressureDrop
    {{ element.name }}(
    {{ macros.render_parameters(parameters) | safe }},
    redeclare package Medium = Medium,
    allowFlowReversal = false) "Pressure drop for return duct" 
```



## MergedExternalWall

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{ macros.convert_tilts(element.tilts, library_name) }})
```



## MergedWindows

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.Window[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Glazing.
    {{ element.constructions[0].name }} glazing,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    frac={{ macros.join_list(element.frame_fractions) }},
    redeclare parameter IDEAS.Buildings.Data.Frames.Wood fraType(each U_value={{ element.constructions[0].u_value_frame }}),
    {%- if element.overhang and element.side_fins %}
    redeclare IDEAS.Buildings.Components.Shading.Box shaType(
        each hWin={{ element.heights[0] }}, each wWin={{ element.widths[0] }},
        each wLeft={{ element.overhang.width_left }}, each wRight={{ element.overhang.width_right }},
        each ovDep={{ element.overhang.depth }}, each ovGap={{ element.overhang.gap }},
        each hFin={{ element.side_fins.height }}, each finDep={{ element.side_fins.depth }},
        each finGap={{ element.side_fins.gap }}),
    {%- elif element.overhang %}
    redeclare IDEAS.Buildings.Components.Shading.Overhang shaType(
        each hWin={{ element.heights[0] }}, each wWin={{ element.widths[0] }},
        each wLeft={{ element.overhang.width_left }}, each wRight={{ element.overhang.width_right }},
        each dep={{ element.overhang.depth }}, each gap={{ element.overhang.gap }}),
    {%- elif element.side_fins %}
    redeclare IDEAS.Buildings.Components.Shading.SideFins shaType(
        each hWin={{ element.heights[0] }}, each wWin={{ element.widths[0] }},
        each hFin={{ element.side_fins.height }}, each dep={{ element.side_fins.depth }},
        each gap={{ element.side_fins.gap }}),
    {%- endif %}
    final inc={{ macros.convert_tilts(element.tilts, library_name) }})
```


