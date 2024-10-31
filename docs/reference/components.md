
## Weather

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    {% if macros.render_parameters(parameters) | safe %}
    inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort, {{ macros.render_parameters(parameters) | safe}}) "Data reader"
    {% raw %}annotation (Placement(transformation(extent={{-96,76},{-76,96}})));{% endraw %}
    {% else %}
        inner IDEAS.BoundaryConditions.SimInfoManager
    sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort) "Data reader"
    {% raw %}annotation (Placement(transformation(extent={{-96,76},{-76,96}})));{% endraw %}
    {% endif %}
    
```



### Variant: default from BUILDINGS
The following template is used for this component:
```jinja

    {% if macros.render_parameters(parameters) | safe %}
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
    {% else %}
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                {{ element.name }}(filNam=Modelica.Utilities.Files.loadResource
        ("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    {% endif %}
    
```



## InternalElement

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.InternalWall {{ element.name }}
    (redeclare parameter {{ package_name }}.
    Data.Constructions.{{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    A = {{ element.surface }}, inc = IDEAS.Types.Tilt.
    {{ element.tilt.value | capitalize }}, azi =
    {{ element.azimuth }}) "Partition wall between the two
    rooms" 
```



### Variant: default from BUILDINGS
The following template is used for this component:
```jinja
    Buildings.HeatTransfer.Conduction.MultiLayer
                {{ element.name }}(A =
            {{ element.surface }}, layers =
    {{ element.construction.name }}, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms" 
```



## Window

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.Window[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Glazing.
    {{ element.constructions[0].name }} glazing,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{macros.join_list(tilts)}})
```



## Pump

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
  {{ package_name }}.Common.
    Fluid.Ventilation.Pump{{ element.name | capitalize }}
     {{ element.name }}(
     {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW

    )
```



## Valve

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    {{ library_name }}.Fluid.Actuators.Valves.TwoWayEqualPercentage
            {{ element.name }}(
                {{ macros.render_parameters(parameters) | safe}},
    redeclare package Medium = MediumW

    ) "Radiator valve" 
```



## DataBus

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    {{package_name}}.Common.Controls.ventilation.DataServer
        {{ element.name }} (redeclare package
          Medium = Medium)
```



## FloorOnGround

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    IDEAS.Buildings.Components.SlabOnGround {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    A={{  element.surface}})
```



## ThreeWayValve

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    {{library_name}}.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             {{ element.name }}(
    redeclare package Medium = MediumW,
    use_inputFilter=false,
    {{ macros.render_parameters(parameters) | safe}},
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-wayvalve" 
```



## Occupancy

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.Occupancy{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



## CollectorControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.CollectorControl{{ element.name | capitalize}}
    {{ element.name }}
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



## VAVControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.VAVControl{{ element.name | capitalize}}
    {{ element.name }}
```



### Variant: constant from DEFAULT
The following template is used for this component:
```jinja
    Modelica.Blocks.Sources.RealExpression {{ element.name }}(y=1)
```



## AhuControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.AhuControl{{ element.name | capitalize}}
    {{ element.name }}
```



## MergedWindows

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.Window[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Glazing.
    {{ element.constructions[0].name }} glazing,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{macros.join_list(tilts)}})
```



## BoilerControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.BoilerControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



## Duct

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
  {{ library_name }}.Fluid.FixedResistances.PressureDrop
    {{ element.name }}(
    m_flow_nominal=100*1.2/3600,
    redeclare package Medium = Medium,
    allowFlowReversal = false,
    dp_nominal=40) "Pressure drop for return duct" 
```



## AirHandlingUnit

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Common.Fluid.
    Ventilation.Ahu{{ element.name | capitalize}}
    {{ element.name }}
    (redeclare package MediumA = Medium,
    {% raw %}
    VRoo={100,100},
    AFlo={20,20},
    mCooVAV_flow_nominal={0.01,0.01}{% endraw %})
```



### Variant: test from DEFAULT
The following template is used for this component:
```jinja
{{package_name}}.Common.Fluid.
    Ventilation.SimpleHVACBuildings
    {{ element.name }}
    (redeclare package Medium = Medium)
```



## Boundary

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
  Buildings.Fluid.Sources.Outside {{ element.name }}
    (nPorts=2,redeclare package Medium = Medium)
```



## VAV

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
  {{ library_name }}.Fluid.Actuators.Dampers.PressureIndependent
    {{ element.name }}(
    redeclare package Medium = Medium,
    m_flow_nominal=100*1.2/3600,
    dpDamper_nominal=50,
    allowFlowReversal=false,
    dpFixed_nominal=50) "VAV box for room" 
```



### Variant: complex from DEFAULT
The following template is used for this component:
```jinja
  {{ package_name }}.Common.
    Fluid.Ventilation.VAVBox{{ element.name | capitalize }}
     {{ element.name }}(
    redeclare package MediumA = Medium,
    mCooAir_flow_nominal=100*1.2/3600,
    mHeaAir_flow_nominal=100*1.2/3600,
    VRoo=100,
    allowFlowReversal=false,
    THeaWatInl_nominal=90,
    THeaWatOut_nominal=60,
    THeaAirInl_nominal=30,
    THeaAirDis_nominal=25
    )
```



## EmissionControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.EmissionControl{{ element.name | capitalize}}
    {{ element.name }}({{ macros.render_parameters(parameters) | safe}})
```



## Boiler

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

{{package_name}}.Common.Fluid.Boilers.
BoilerWithStorage{{ element.name | capitalize}} {{ element.name }}(
{{ macros.render_parameters(parameters) | safe}},
redeclare package MediumW = MediumW, fue = Buildings.Fluid.Data.Fuels.HeatingOilLowerHeatingValue()) "Boiler" 
```



## TemperatureSensor

### Variant: default from DEFAULT
The following template is used for this component:
```jinja
    Buildings.Fluid.Sensors.TemperatureTwoPort {{ element.name }}(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal) "Radiator" 
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



### Variant: ideal from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.HeatTransfer.IdealHeatingSystem.IdealHeatEmission
    {{ element.name }}
```



## Space

### Variant: default from IDEAS
The following template is used for this component:
```jinja
IDEAS.Buildings.Components.Zone {{ element.name }}(
    mSenFac=0.822,
        {%- if element.number_ventilation_ports != 0 -%}
    nPorts = {{ element.number_ventilation_ports }},
    {%- endif %}
    {{ macros.render_parameters(parameters) | safe}},
    n50=0.822*0.5*{{ element.name }}.n50toAch,
    redeclare package Medium = Medium,
    nSurf={{ element.number_merged_external_boundaries }},
    T_start=293.15)
```



### Variant: default from BUILDINGS
The following template is used for this component:
```jinja
Buildings.ThermalZones.Detailed.MixedAir {{ element.name }}(
        redeclare package Medium = Medium,
        {{ macros.render_parameters(parameters) | safe}},
        {%- if element.number_ventilation_ports != 0 -%}
        nPorts = {{ element.number_ventilation_ports }},
        {%- endif %}
        {%- for boundary in element.boundaries -%}
            {%- if boundary.type == 'ExternalWall' -%}
                {%- if boundary.number %}
                    nConExt={{ boundary.number }},
                    datConExt(
                    {{ macros.element_parameters(boundary) }},
                    azi={{ macros.join_list(boundary.azimuths) }}),
                {% else %}
                    nConExt=0,
                {%- endif %}
            {%- endif %}
            {%- if boundary.type == "InternalElement"-%}
                {%- if boundary.number %}
                    nSurBou={{ boundary.number }},
                    surBou(
                    A={{ macros.join_list(boundary.surfaces) }},
                    til={{ macros.convert_tilts(boundary.tilts) }}),
                {% else %}
                    nSurBou=0,
                {%- endif %}
            {%- endif %}
            {%- if boundary.type == "WindowedWall" -%}
                {%- if boundary.number %}
                    nConExtWin={{ boundary.number }},
                    datConExtWin(
                    {{ macros.element_parameters(boundary) }},
                    glaSys={{ macros.join_list(boundary.window_layers) }},
                    wWin={{ macros.join_list(boundary.window_width) }},
                    hWin={{ macros.join_list(boundary.window_height) }},
                    azi={{ macros.join_list(boundary.azimuths) }}),
                {% else %}
                    nConExtWin=0,
                {%- endif %}
            {%- endif %}
            {%- if boundary.type == "FloorOnGround" -%}
                {%- if boundary.number %}
                    nConBou={{ boundary.number }},
                    datConBou(
                    {{ macros.element_parameters(boundary) }},
                    azi={{ macros.join_list(boundary.azimuths) }}),
                {% else %}
                    nConBou=0,
                {%- endif %}
            {%- endif %}
        {%- endfor %}
        nConPar=0,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
```



## ExternalWall

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{macros.join_list(tilts)}})
```



## MergedExternalWall

### Variant: default from IDEAS
The following template is used for this component:
```jinja

    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{macros.join_list(tilts)}})
```



## ThreeWayValveControl

### Variant: default from DEFAULT
The following template is used for this component:
```jinja

    {{package_name}}.Common.Controls.ventilation.
    ThreeWayValveControl{{ element.name | capitalize}}
    {{ element.name }}
```


