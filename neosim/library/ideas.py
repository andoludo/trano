from typing import Callable, Dict, List

from jinja2 import BaseLoader, Environment
from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, Field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.library.base import (
    BaseData,
    BaseEmission,
    BaseSpace,
    BaseSplitValve,
    BaseThreeWayValve,
    BaseValve,
    DefaultLibrary,
    LibraryData,
)
from neosim.models.constants import Flow
from neosim.models.elements.base import Port
from neosim.models.elements.control import Control, SpaceControl
from neosim.models.elements.merged_wall import MergedBaseWall, MergedExternalWall
from neosim.models.elements.space import Space
from neosim.models.elements.system import Emission, Occupancy
from neosim.models.elements.wall import BaseSimpleWall, BaseWall

CONSTANTS = """
replaceable package Medium = IDEAS.Media.Air
constrainedby Modelica.Media.Interfaces.PartialMedium
"Medium in the component"
annotation (choicesAllMatching = true);
package MediumW = IDEAS.Media.Water "Medium model";
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
  inner IDEAS.BoundaryConditions.SimInfoManager sim(interZonalAirFlowType=
  IDEAS.BoundaryConditions.Types.
  InterZonalAirFlow.OnePort)
                                              "Data reader"
    annotation (Placement(transformation(extent={{-96,76},{-76,96}})));
"""


class IdeasSpace(BaseSpace):
    template: str = """IDEAS.Buildings.Components.Zone {{ element.name }}(
    mSenFac=0.822,
    V={{ element.volume }},
    n50=0.822*0.5*{{ element.name }}.n50toAch,
    redeclare package Medium = Medium,
    nSurf={{ element.number_merged_external_boundaries }},
    hZone={{ element.height }},
    T_start=293.15)
    annotation (Placement(transformation(origin={{ macros.join_list(element.position) }},
    extent={% raw %}{{-20,-20},{20,20}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=BaseWall,
                names=["propsBus"],
                multi_connection=True,
                bus_connection=True,
            ),
            Port(target=Occupancy, names=["yOcc"]),
            Port(target=Emission, names=["gainCon", "gainRad"]),
            Port(target=SpaceControl, names=["gainCon"]),
        ]
    )


class IdeasMergedExternalWall(LibraryData):
    template: str = """
    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.OuterWall[{{ element.constructions | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    final inc={{macros.join_list(tilts)}})  annotation(
    Placement(transformation(origin = {{ macros.join_list(element.position) }}, extent =
    {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=Space,
                names=["propsBus_a"],
                multi_connection=True,
                multi_object=True,
                bus_connection=True,
            ),
        ]
    )


class IdeasMergedWindows(LibraryData):
    template: str = """
    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.Window[{{ element.constructions | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Glazing.
    {{ element.constructions[0].name }} glazing,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    final inc={{macros.join_list(tilts)}})  annotation(
    Placement(transformation(origin = {{ macros.join_list(element.position) }}
    , extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                target=Space,
                names=["propsBus_a"],
                multi_connection=True,
                multi_object=True,
                bus_connection=True,
            ),
        ]
    )


class IdeasFloorOnGround(LibraryData):
    template: str = """
    IDEAS.Buildings.Components.SlabOnGround {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.construction.name }} constructionType,
    A={{  element.surface}})  annotation(
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["propsBus_a"], multi_connection=False),
        ]
    )


class IdeasInternalElement(LibraryData):
    template: str = """
    IDEAS.Buildings.Components.InternalWall {{ element.name }}
    (redeclare parameter {{ package_name }}.
    Data.Constructions.{{ element.construction.name }} constructionType,
    A = {{ element.surface }}, inc = IDEAS.Types.Tilt.
    {{ element.tilt.value | capitalize }}, azi =
    {{ element.azimuth }}) "Partition wall between the two
    rooms" annotation(
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(target=Space, names=["propsBus_a"], multi_connection=False),
            Port(target=Space, names=["propsBus_b"], multi_connection=False),
        ]
    )


class IdeasPump(LibraryData):
    template: str = """
    IDEAS.Fluid.Movers.FlowControlled_m_flow {{ element.name }}(
    redeclare package Medium = MediumW, m_flow_nominal = 1, dp_nominal = 100)
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                names=["port_a"],
                flow=Flow.inlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                names=["port_b"],
                flow=Flow.outlet,
                multi_connection=True,
                use_counter=False,
            ),
            Port(target=Control, names=["m_flow_in"]),
        ]
    )


class IdeasConstruction(BaseData):
    template: str = Field(
        default="""      record {{ construction.name }}
    "{{ construction.name }}"
   extends IDEAS.Buildings.Data.Interfaces.Construction(
{#      incLastLay = IDEAS.Types.Tilt.Wall,#}
      mats={
        {%- for layer in construction.layers -%}
        {{ package_name }}.Data.Materials.{{ layer.material.name }}
        (d={{ layer.thickness }}){{ "," if not loop.last }}
        {%- endfor %}
    });
    end {{ construction.name }};"""
    )


class IdeasMaterial(BaseData):
    template: str = Field(
        default="""
    record {{ construction.name }} = IDEAS.Buildings.Data.Interfaces.Material (
 k={{construction.thermal_conductivity}},
      c={{construction.specific_heat_capacity}},
      rho={{construction.density}},
      epsLw=0.88,
      epsSw=0.55);"""
    )


class IdeasGlazing(BaseData):
    template: str = Field(
        default="""record  {{ construction.name }} = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay={{ construction.layers|length }},
      final checkLowPerformanceGlazing=false,
          mats={
        {%- for layer in construction.layers -%}
        {{ package_name }}.Data.Materials.{{ layer.material.name }}
        (d={{ layer.thickness }}){{ "," if not loop.last }}
        {%- endfor %}
    },
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
      final g_value=0.78

    ) "{{ package_name }}";"""
    )


class IdeasData(BaseModel):
    template: str = """package Data "Data for transient thermal building simulation"
extends Modelica.Icons.MaterialPropertiesPackage;

package Glazing "Library of building glazing systems"
extends Modelica.Icons.MaterialPropertiesPackage;
{% for g in glazing %}
    {{ g|safe }}
{% endfor %}
end Glazing;

package Materials "Library of construction materials"
extends Modelica.Icons.MaterialPropertiesPackage;
{%- for m in material -%}
    {{ m|safe }}
{%- endfor %}
end Materials;
package Constructions "Library of building envelope constructions"
{%- for c in construction -%}
    {{ c|safe}}
{%- endfor %}

end Constructions;
end Data;"""
    construction: IdeasConstruction
    material: IdeasMaterial
    glazing: IdeasGlazing

    def generate_data(self, package_name: str) -> str:
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=BaseLoader(),
            autoescape=True,
        )
        models = {"material": [], "construction": [], "glazing": []}
        for construction_type_name in models:
            construction_type = getattr(self, construction_type_name)
            for construction in construction_type.constructions:
                template = environment.from_string(construction_type.template)
                model = template.render(
                    construction=construction, package_name=package_name
                )
                models[construction_type_name].append(model)
        template = environment.from_string(self.template)
        model = template.render(**models, package_name=package_name)
        return model


def tilts_processing_ideas(element: MergedExternalWall) -> List[str]:
    return [f"IDEAS.Types.Tilt.{tilt.value.capitalize()}" for tilt in element.tilts]


class IdeasLibrary(DefaultLibrary):
    template: str = "ideas.jinja2"
    constants: str = CONSTANTS
    merged_external_boundaries: bool = True
    functions: Dict[str, Callable] = {  # noqa: RUF012
        "tilts_processing_ideas": tilts_processing_ideas
    }
    space: LibraryData = Field(default=IdeasSpace())
    externalwall: LibraryData = Field(default=IdeasMergedExternalWall())
    flooronground: LibraryData = Field(default=IdeasFloorOnGround())
    mergedexternalwall: LibraryData = Field(default=IdeasMergedExternalWall())
    mergedwindows: LibraryData = Field(default=IdeasMergedWindows())
    internalelement: LibraryData = Field(default=IdeasInternalElement())
    pump: LibraryData = Field(default=IdeasPump())
    threewayvalve: LibraryData = Field(
        default=BaseThreeWayValve(
            template="""
    IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear {{ element.name }}(
    redeclare package Medium = MediumW) "Three-way valve"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    splitvalve: LibraryData = Field(
        default=BaseSplitValve(
            template="""
    IDEAS.Fluid.FixedResistances.Junction {{ element.name }} (
    redeclare package Medium = MediumW)
    "Flow splitter"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    valve: LibraryData = Field(
        default=BaseValve(
            template="""
    IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage {{ element.name }}(
    redeclare package Medium = MediumW) "Radiator valve"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )
    emission: LibraryData = Field(
        default=BaseEmission(
            template="""
    IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 {{ element.name }}(
    redeclare package Medium = MediumW) "Radiator"
    annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
        )
    )

    def extract_data(self, package_name: str, nodes: NodeView) -> str:  # noqa: PLR6301

        merged_constructions = {
            construction
            for node in [node_ for node_ in nodes if isinstance(node_, MergedBaseWall)]
            for construction in node.constructions
        }

        constructions = {
            node.construction
            for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
        }
        merged_constructions.update(constructions)
        wall_constructions = [
            c for c in merged_constructions if isinstance(c, Construction)
        ]
        glazing = [c for c in merged_constructions if isinstance(c, Glass)]
        materials = {
            layer.material
            for construction in merged_constructions
            for layer in construction.layers
        }
        ideas_data = IdeasData(
            material=IdeasMaterial(constructions=list(materials)),
            construction=IdeasConstruction(constructions=wall_constructions),
            glazing=IdeasGlazing(constructions=glazing),
        )
        return ideas_data.generate_data(package_name)
