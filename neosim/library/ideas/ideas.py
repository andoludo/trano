from typing import Any, Callable, Dict, List

from networkx.classes.reportviews import NodeView
from pydantic import Field

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.library.base import (
    BaseAirHandlingUnit,
    BaseEmission,
    BaseIdealHeatingEmission,
    BaseSpace,
    BaseSplitValve,
    BaseThreeWayValve,
    BaseValve,
    DefaultLibrary,
    MaterialProperties,
)
from neosim.library.ideas.constants import CONSTANTS
from neosim.library.ideas.data import (
    IdeasConstruction,
    IdeasData,
    IdeasGlazing,
    IdeasMaterial,
)
from neosim.models.constants import Flow
from neosim.models.elements.base import LibraryData, Port
from neosim.models.elements.bus import DataBus
from neosim.models.elements.control import SpaceControl
from neosim.models.elements.controls.base import Control
from neosim.models.elements.envelope.base import BaseSimpleWall, BaseWall
from neosim.models.elements.merged_wall import MergedBaseWall, MergedExternalWall
from neosim.models.elements.occupancy import Occupancy
from neosim.models.elements.space import Space
from neosim.models.elements.system import Emission, Ventilation


class IdeasSpace(BaseSpace):
    template: str = """IDEAS.Buildings.Components.Zone {{ element.name }}(
    mSenFac=0.822,
        {%- if element.number_ventilation_ports != 0 -%}
    nPorts = {{ element.number_ventilation_ports }},
    {%- endif %}
    V={{ element.volume }},
    n50=0.822*0.5*{{ element.name }}.n50toAch,
    redeclare package Medium = Medium,
    nSurf={{ element.number_merged_external_boundaries }},
    hZone={{ element.height }},
    T_start=293.15)"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[BaseWall],
                names=["propsBus"],
                multi_connection=True,
                bus_connection=True,
            ),
            Port(targets=[Occupancy], names=["yOcc"]),
            Port(targets=[Emission], names=["gainCon", "gainRad"]),
            Port(
                targets=[SpaceControl, DataBus],
                names=["gainCon"],
                multi_connection=True,
                use_counter=False,
            ),
            Port(
                targets=[Ventilation, Control, DataBus],
                names=["ports"],
                multi_connection=True,
                flow=Flow.inlet_or_outlet,
            ),
        ]
    )


class IdeasMergedExternalWall(LibraryData):
    template: str = """
    {% set tilts = tilts_processing_ideas(element) %}
    IDEAS.Buildings.Components.OuterWall[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Constructions.
    {{ element.constructions[0].name }}
    constructionType,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{macros.join_list(tilts)}})"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space],
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
    IDEAS.Buildings.Components.Window[{{ element.surfaces | length }}]
    {{ element.name }}(
    redeclare parameter {{ package_name }}.Data.Glazing.
    {{ element.constructions[0].name }} glazing,
    A={{  macros.join_list(element.surfaces)}},
    final azi={{macros.join_list(element.azimuths)}},
    redeclare package Medium = Medium,
    final inc={{macros.join_list(tilts)}})"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space],
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
    redeclare package Medium = Medium,
    A={{  element.surface}})"""
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(targets=[Space], names=["propsBus_a"], multi_connection=False),
        ]
    )


class IdeasInternalElement(LibraryData):
    template: str = """
    IDEAS.Buildings.Components.InternalWall {{ element.name }}
    (redeclare parameter {{ package_name }}.
    Data.Constructions.{{ element.construction.name }} constructionType,
    redeclare package Medium = Medium,
    A = {{ element.surface }}, inc = IDEAS.Types.Tilt.
    {{ element.tilt.value | capitalize }}, azi =
    {{ element.azimuth }}) "Partition wall between the two
    rooms" """
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[Space],
                names=["propsBus_a"],
                multi_connection=False,
                flow=Flow.interchangeable_port,
            ),
            Port(
                targets=[Space],
                names=["propsBus_b"],
                multi_connection=False,
                flow=Flow.interchangeable_port,
            ),
        ]
    )


class IdeasPump(LibraryData):
    template: str = """
    IDEAS.Fluid.Movers.FlowControlled_m_flow {{ element.name }}(
    redeclare package Medium = MediumW, m_flow_nominal = 1, dp_nominal = 100)"""
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
            Port(targets=[Control], names=["m_flow_in"]),
        ]
    )


def tilts_processing_ideas(element: MergedExternalWall) -> List[str]:
    return [f"IDEAS.Types.Tilt.{tilt.value.capitalize()}" for tilt in element.tilts]


class IdeasLibrary(DefaultLibrary):
    library_name: str = "IDEAS"
    constants: str = CONSTANTS
    merged_external_boundaries: bool = True
    functions: Dict[str, Callable[[Any], Any]] = {  # noqa: RUF012
        "tilts_processing_ideas": tilts_processing_ideas
    }
    space: List[LibraryData] = Field(default=[IdeasSpace()])
    externalwall: List[LibraryData] = Field(default=[IdeasMergedExternalWall()])
    flooronground: List[LibraryData] = Field(default=[IdeasFloorOnGround()])
    mergedexternalwall: List[LibraryData] = Field(default=[IdeasMergedExternalWall()])
    mergedwindows: List[LibraryData] = Field(default=[IdeasMergedWindows()])
    internalelement: List[LibraryData] = Field(default=[IdeasInternalElement()])
    pump: List[LibraryData] = Field(default=[IdeasPump()])
    threewayvalve: List[LibraryData] = Field(
        default=[
            BaseThreeWayValve(
                template="""
    IDEAS.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear {{ element.name }}(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1,
    dpValve_nominal = 1000) "Three-way valve" """
            )
        ]
    )
    splitvalve: List[LibraryData] = Field(
        default=[
            BaseSplitValve(
                template="""
    IDEAS.Fluid.FixedResistances.Junction {{ element.name }} (
    redeclare package Medium = MediumW, m_flow_nominal = {0.1, 0.1, 0.1},
    dp_nominal = {40, 40, 40})
    "Flow splitter" """
            )
        ]
    )
    valve: List[LibraryData] = Field(
        default=[
            BaseValve(
                template="""
    IDEAS.Fluid.Actuators.Valves.TwoWayEqualPercentage {{ element.name }}(
    redeclare package Medium = MediumW, m_flow_nominal = 0.1, dpValve_nominal = 200,
    allowFlowReversal = false, dpFixed_nominal = 200, linearized = false) "Radiator valve"
    """
            )
        ]
    )
    emission: List[LibraryData] = Field(
        default=[
            BaseEmission(
                template="""
    IDEAS.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 {{ element.name }}(
    redeclare package Medium = MediumW, allowFlowReversal = false, Q_flow_nominal = 500,
    T_a_nominal = 318.15, T_b_nominal = 308.15, m_flow_nominal = 100*1.2/3600,
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Radiator" """
            ),
            BaseIdealHeatingEmission(),
        ]
    )
    airhandlingunit: List[LibraryData] = Field(
        default=[
            BaseAirHandlingUnit(
                template="""{{ package_name }}.Common.Fluid.
            Ventilation.SimpleHVAC {{ element.name }}
    (redeclare package Medium = Medium)"""
            )
        ]
    )

    def extract_data(
        self, package_name: str, nodes: NodeView  # noqa: PLR6301
    ) -> MaterialProperties:

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
        return MaterialProperties(
            data=ideas_data.generate_data(package_name), is_package=True
        )
