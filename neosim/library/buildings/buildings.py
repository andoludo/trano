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
from neosim.library.buildings.constants import BUILDINGS_CONSTANTS
from neosim.library.buildings.data import (
    BuildingsConstruction,
    BuildingsData,
    BuildingsGlazing,
    BuildingsMaterial,
)
from neosim.models.elements.wall import BaseSimpleWall


class BuildingsLibrary(DefaultLibrary):
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