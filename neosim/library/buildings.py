from typing import List, Optional, Union

from networkx.classes.reportviews import NodeView
from pydantic import Field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.library.base import (
    BaseEmission,
    BaseInternalElement,
    BasePump,
    BaseSpace,
    BaseSplitValve,
    BaseThreeWayValve,
    BaseValve,
    BaseWeather,
    DefaultLibrary,
    LibraryData,
)
from neosim.library.data.base import BaseConstructionData, BaseData
from neosim.material import Material
from neosim.models.elements.wall import BaseSimpleWall

BUILDINGS_CONSTANTS = """
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
"""


class BuildingsConstruction(BaseData):
    template: str = Field(
        default="""    parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic
        {{ construction.name }}(
    final nLay={{ construction.layers|length }},
    absIR_a=0.9,
    absIR_b=0.9,
    absSol_a=0.6,
    absSol_b=0.6,
    material={
    {%- for layer in construction.layers -%}
        Buildings.HeatTransfer.Data.Solids.Generic(
        x={{ layer.thickness }},
        k={{ layer.material.thermal_conductivity }},
        c={{ layer.material.specific_heat_capacity }},
        d={{ layer.material.density }}){{ "," if not loop.last }}
    {%- endfor %}
    },
    roughness_a=Buildings.HeatTransfer.Types.SurfaceRoughness.Rough)
    annotation (Placement(transformation(extent={% raw %}{{20,84},{34,98}}{% endraw %})));"""
    )


class BuildingsMaterial(BaseData):
    template: Optional[str] = None
    constructions: List[Union[Construction, Material, Glass]] = Field(default=[])


class BuildingsGlazing(BaseData):
    template: str = Field(
        default="""    {% set glass_layers =    construction.layers |
        selectattr('layer_type', 'eq', "glass") | list %}
    {% set gas_layers =    construction.layers | selectattr('layer_type', 'eq', "gas") | list %}
    parameter Buildings.HeatTransfer.Data.GlazingSystems.Generic {{ construction.name }}(
    final glass={
    {% for layer in glass_layers %}
        Buildings.HeatTransfer.Data.Glasses.Generic(
        x={{ layer.thickness }},
        k={{ layer.material.thermal_conductivity }},
        tauSol={{ macros.join_list(layer.material.solar_transmittance ) }},
        rhoSol_a={{ macros.join_list(layer.material.solar_reflectance_outside_facing ) }},
        rhoSol_b={{ macros.join_list(layer.material.solar_reflectance_room_facing ) }},
        tauIR={{ layer.material.infrared_transmissivity }},
        absIR_a={{ layer.material.infrared_absorptivity_outside_facing }},
        absIR_b={{ layer.material.infrared_absorptivity_room_facing }})
        {{ "," if not loop.last }}
    {% endfor %}
    },
    final gas={
    {% for layer in gas_layers %}
        {% if layer.layer_type == 'gas' %}
            Buildings.HeatTransfer.Data.Gases.Air(x={{ layer.thickness }})
            {{ "," if not loop.last }}
        {% endif %}
    {% endfor %}
    },
    UFra={{ construction.u_value_frame }})
    annotation (
    defaultComponentPrefixes="parameter",
    defaultComponentName="datGlaSys");"""
    )


class BuildingsData(BaseConstructionData):
    template: str = """
{% for g in glazing %}
    {{ g|safe }}
{% endfor %}
{%- for c in construction -%}
    {{ c|safe}}
{%- endfor %}
"""
    construction: BuildingsConstruction
    material: BuildingsMaterial
    glazing: BuildingsGlazing


class BuildingsLibrary(DefaultLibrary):
    template: str = "buildings.jinja2"
    constants: str = BUILDINGS_CONSTANTS
    internalelement: LibraryData = Field(
        default=BaseInternalElement(
            template="""    Buildings.HeatTransfer.Conduction.MultiLayer {{ element.name }}(A =
            {{ element.surface }}, layers =
    {{ element.construction.name }}, stateAtSurface_a = true, stateAtSurface_b = true)
    "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    weather: LibraryData = Field(
        default=BaseWeather(
            template="""    Buildings.BoundaryConditions.WeatherData.ReaderTMY3
            {{ element.name }}(filNam =
    Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/
    USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    threewayvalve: LibraryData = Field(
        default=BaseThreeWayValve(
            template="""    Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear
             {{ element.name }}(
    redeclare package Medium = MediumW,
    dpValve_nominal=dpThrWayVal_nominal,
    l={0.01,0.01},
    tau=10,
    m_flow_nominal=mRad_flow_nominal,
    dpFixed_nominal={100,0},
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Three-way valve"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
     extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    splitvalve: LibraryData = Field(
        default=BaseSplitValve(
            template="""    Buildings.Fluid.FixedResistances.Junction {{ element.name }} (
    dp_nominal={0,0,0},
    m_flow_nominal=mRad_flow_nominal*{1,-1,-1},
    redeclare package Medium = MediumW,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    pump: LibraryData = Field(
        default=BasePump(
            template="""    Buildings.Fluid.Movers.Preconfigured.SpeedControlled_y
            {{ element.name }}(
    redeclare package Medium = MediumW,
    m_flow_nominal=mRad_flow_nominal,
    dp_nominal=dp_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    valve: LibraryData = Field(
        default=BaseValve(
            template="""    Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage
            {{ element.name }}(
    redeclare package Medium = MediumW,
    dpValve_nominal(displayUnit="Pa") = dpVal_nominal,
    m_flow_nominal=mRad_flow_nominal/nRoo,
    dpFixed_nominal=dpRoo_nominal,
    from_dp=true,
    use_inputFilter=false) "Radiator valve"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }}
    , extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    emission: LibraryData = Field(
        default=BaseEmission(
            template="""    Buildings.Fluid.HeatExchangers.Radiators.
            RadiatorEN442_2 {{ element.name }}(
    redeclare package Medium = MediumW,
    Q_flow_nominal=scaFacRad*Q_flow_nominal/nRoo,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=323.15,
    T_b_nominal=313.15) "Radiator"
    annotation (
    Placement(transformation(origin =
    {{ macros.join_list(element.position) }}, extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    space: LibraryData = Field(
        default=BaseSpace(
            template="""Buildings.ThermalZones.Detailed.MixedAir {{ element.name }}(
    redeclare package Medium = Medium,
    AFlo={{ element.floor_area }},
    hRoo={{ element.height }},
    {%- for boundary in element.boundaries -%}
        {%- if boundary.type == 'ExternalWall' and boundary.number -%}
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
                hWin={{ macros.join_list(boundary.window_height) }}),
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
    annotation (Placement(transformation(origin=
    {{ macros.join_list(element.position) }},extent={% raw %}{{-20,-20},{20,20}}
    {% endraw %})));"""
        )
    )

    def extract_data(self, package_name: str, nodes: NodeView) -> str:  # noqa: PLR6301

        constructions = {
            node.construction
            for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
        }
        wall_constructions = [c for c in constructions if isinstance(c, Construction)]
        glazing = [c for c in constructions if isinstance(c, Glass)]
        buildings_data = BuildingsData(
            construction=BuildingsConstruction(constructions=wall_constructions),
            glazing=BuildingsGlazing(constructions=glazing),
            material=BuildingsMaterial(),
        )
        return buildings_data.generate_data(package_name)
