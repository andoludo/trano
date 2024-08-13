from functools import partial
from math import ceil
from typing import Any, Callable, ClassVar, Dict, List, Literal, Optional, Union

from networkx import Graph
from pydantic import Field, computed_field

from trano.models.constants import Flow
from trano.models.elements.base import (
    AvailableLibraries,
    Axis,
    BaseElement,
    BaseParameter,
    Figure,
    LibraryData,
    Line,
    Port,
    exclude_parameters,
    modify_alias,
)
from trano.models.elements.bus import DataBus
from trano.models.elements.controls.base import Control
from trano.models.elements.controls.vav import VAVControl
from trano.models.elements.envelope.base import (
    BaseExternalWall,
    BaseFloorOnGround,
    BaseInternalElement,
    BaseWall,
    BaseWindow,
    MergedBaseWall,
)
from trano.models.elements.system import (
    BaseOccupancy,
    BaseWeather,
    Emission,
    System,
    Ventilation,
)
from trano.models.parameters import WallParameters, WindowedWallParameters

MAX_X_SPACES = 3


def _get_controllable_element(elements: List[System]) -> Optional["System"]:
    controllable_elements = []
    for element in elements:
        controllable_ports = element.get_controllable_ports()
        if controllable_ports:
            controllable_elements.append(element)
    if len(controllable_elements) > 1:
        raise NotImplementedError
    if not controllable_elements:
        return None
    return controllable_elements[0]


class SpaceParameter(BaseParameter):
    sensible_thermal_mass_scaling_factor: float = Field(
        1,
        description="Factor for scaling the sensible thermal mass of the zone air volume",
        alias="mSenFac",
    )
    floor_area: float = Field(20, description="Floor area [m2]", alias="AFlo")
    average_room_height: float = Field(
        2, description="Average room height [m]", alias="hRoo"
    )
    linearize_emissive_power: Literal["true", "false"] = Field(
        default="true",
        description="Set to true to linearize emissive power",
        alias="linearizeRadiation",
    )
    nominal_mass_flow_rate: float = Field(
        0.01, description="Nominal mass flow rate [kg/s]", alias="m_flow_nominal"
    )
    temperature_initial: float = Field(
        273.15 + 21, description="Initial temperature [K]", alias="T_start"
    )

    @computed_field
    def volume(self) -> float:
        return self.floor_area * self.average_room_height


class BuildingsSpace(LibraryData):
    template: str = """Buildings.ThermalZones.Detailed.MixedAir {{ element.name }}(
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
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)"""
    parameter_processing: Callable[[SpaceParameter], Dict[str, Any]] = partial(
        exclude_parameters, exclude_parameters={"volume"}
    )
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[BaseInternalElement],
                names=["surf_surBou"],
                multi_connection=True,
            ),
            Port(targets=[BaseOccupancy], names=["qGai_flow"]),
            Port(targets=[BaseWeather], names=["weaBus"]),
            Port(targets=[Emission], names=["heaPorAir", "heaPorRad"]),
            Port(targets=[DataBus], names=["heaPorAir"]),
            Port(targets=[VAVControl], names=["heaPorAir"]),
            Port(
                targets=[Ventilation, Control, DataBus],
                names=["ports"],
                multi_connection=True,
                flow=Flow.inlet_or_outlet,
            ),
        ]
    )

    figures: List[Figure] = Field(
        default=[
            Figure(
                right_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.occupancy.name }}.occSch2.occupied",
                            label="Occupied zone [-]",
                            color="grey",
                            line_style="dashed",
                        )
                    ],
                    label="Occupancy schedule [-]",
                ),
                left_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.name }}.air.vol.T",
                            label="Air temperature [K]",
                            color="blue",
                            line_width=1.5,
                        ),
                        Line(
                            template="data_bus.dataBus.T{{ element.name | capitalize}}",
                            label="Measured Air temperature [K]",
                            color="black",
                            line_width=2,
                        ),
                    ],
                    label="Zone air temperature [K]",
                ),
            ),
            Figure(
                left_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.name }}.heaPorRad.Q_flow",
                            label="Radiative heat flow rate [W]",
                            color="red",
                        ),
                        Line(
                            template="{{ element.name }}.heaPorAir.Q_flow",
                            label="Convective heat flow rate [W]",
                            color="blue",
                        ),
                        Line(
                            template="{{ element.name }}.air.QLat_flow",
                            label="Latent heat gain [W]",
                            color="yellow",
                        ),
                        Line(
                            template="{{ element.name }}.air.QCon_flow",
                            label="Convective heat gain [W]",
                        ),
                    ],
                    label="Heat flow rate [W]",
                ),
            ),
            Figure(
                right_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.emissions[0].control.name }}."
                            "emissionControl.conHea.u_s",
                            label="Zone controller setpoint [K]",
                            color="blue",
                        ),
                        Line(
                            template="{{ element.emissions[0].control.name }}."
                            "emissionControl.conHea.u_m",
                            label="Zone controller measured [K]",
                            color="red",
                        ),
                    ],
                    label="Zone controller input [K]",
                ),
                left_axis=Axis(
                    lines=[
                        Line(
                            template="{{ element.emissions[0].control.name  }}.y",
                            label="Control signal [-]",
                            color="grey",
                            line_style="dashed",
                        ),
                    ],
                    label="Control signal [-]",
                ),
            ),
        ]
    )


class IdeasSpace(LibraryData):
    template: str = """IDEAS.Buildings.Components.Zone {{ element.name }}(
    mSenFac=0.822,
        {%- if element.number_ventilation_ports != 0 -%}
    nPorts = {{ element.number_ventilation_ports }},
    {%- endif %}
    {{ macros.render_parameters(parameters) | safe}},
    n50=0.822*0.5*{{ element.name }}.n50toAch,
    redeclare package Medium = Medium,
    nSurf={{ element.number_merged_external_boundaries }},
    T_start=293.15)"""
    parameter_processing: Callable[[SpaceParameter], Dict[str, Any]] = partial(
        modify_alias, mapping={"average_room_height": "hZone", "volume": "V"}
    )
    ports_factory: Callable[[], List[Port]] = Field(
        default=lambda: [
            Port(
                targets=[BaseWall],
                names=["propsBus"],
                multi_connection=True,
                bus_connection=True,
            ),
            Port(targets=[BaseOccupancy], names=["yOcc"]),
            Port(targets=[Emission], names=["gainCon", "gainRad"]),
            Port(
                targets=[VAVControl, DataBus],
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


class Space(BaseElement):
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-20, -20}, {20, 20}}
    {% endraw %})));"""
    counter: ClassVar[int] = 0
    parameters: SpaceParameter = Field(default=SpaceParameter())
    name: str
    external_boundaries: list[
        Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround"]
    ]
    internal_elements: List["BaseInternalElement"] = Field(default=[])
    boundaries: Optional[List[WallParameters]] = None
    emissions: List[System] = Field(default=[])
    ventilation_inlets: List[System] = Field(default=[])
    ventilation_outlets: List[System] = Field(default=[])
    occupancy: Optional[BaseOccupancy] = None
    # libraries_data: AvailableLibraries = AvailableLibraries(
    #     ideas=[IdeasSpace], buildings=[BuildingsSpace]
    # )

    def model_post_init(self, __context) -> None:  # type: ignore # noqa: ANN001
        self._assign_space()

    def _assign_space(self) -> None:
        for emission in (
            self.emissions + self.ventilation_inlets + self.ventilation_outlets
        ):
            if emission.control:
                emission.control.space_name = self.name
        if self.occupancy:
            self.occupancy.space_name = self.name

    # @computed_field  # type: ignore
    @property
    def number_merged_external_boundaries(self) -> int:
        return sum(
            [
                boundary.length
                for boundary in self.merged_external_boundaries + self.internal_elements
            ]
        )

    # @computed_field  # type: ignore
    @property
    def number_ventilation_ports(self) -> int:
        return 2 + 1  # databus

    # @computed_field  # type: ignore
    @property
    def merged_external_boundaries(
        self,
    ) -> List[
        Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround", "MergedBaseWall"]
    ]:
        from trano.models.elements.envelope.merged_wall import MergedExternalWall
        from trano.models.elements.envelope.merged_windows import MergedWindows

        external_walls = [
            boundary
            for boundary in self.external_boundaries
            if boundary.type in ["ExternalWall", "ExternalDoor"]
        ]
        windows = [
            boundary
            for boundary in self.external_boundaries
            if boundary.type == "Window"
        ]
        merged_external_walls = MergedExternalWall.from_base_elements(external_walls)
        merged_windows = MergedWindows.from_base_windows(windows)  # type: ignore
        external_boundaries: list[
            BaseExternalWall | BaseWindow | BaseFloorOnGround | MergedBaseWall
        ] = (
            merged_external_walls
            + merged_windows
            + [
                boundary
                for boundary in self.external_boundaries
                if boundary.type not in ["ExternalWall", "Window", "ExternalDoor"]
            ]
        )
        return external_boundaries

    def get_controllable_emission(self) -> Optional["System"]:
        return _get_controllable_element(self.emissions)

    def assign_position(self) -> None:
        self.position = [
            250 * (Space.counter % MAX_X_SPACES),
            150 * ceil(Space.counter / MAX_X_SPACES),
        ]
        Space.counter += 1
        x = self.position[0]
        y = self.position[1]
        for i, emission in enumerate(self.emissions):
            emission.position = [x + i * 30, y - 75]
        if self.occupancy:
            self.occupancy.position = [x - 50, y]

    def find_emission(self) -> Optional["Emission"]:
        emissions = [
            emission for emission in self.emissions if isinstance(emission, Emission)
        ]
        if not emissions:
            return None
        if len(emissions) != 1:
            raise NotImplementedError
        return emissions[0]

    def first_emission(self) -> Optional["System"]:
        if self.emissions:
            return self.emissions[0]
        return None

    def last_emission(self) -> Optional["System"]:
        if self.emissions:
            return self.emissions[-1]
        return None

    def get_ventilation_inlet(self) -> Optional["System"]:
        if self.ventilation_inlets:
            return self.ventilation_inlets[-1]
        return None

    def get_last_ventilation_inlet(self) -> Optional["System"]:
        if self.ventilation_inlets:
            return self.ventilation_inlets[0]
        return None

    def get_ventilation_outlet(self) -> Optional["System"]:
        if self.ventilation_outlets:
            return self.ventilation_outlets[0]
        return None

    def get_last_ventilation_outlet(self) -> Optional["System"]:
        if self.ventilation_outlets:
            return self.ventilation_outlets[-1]
        return None

    def get_neighhors(self, graph: Graph) -> None:
        from trano.models.elements.envelope.external_wall import ExternalWall
        from trano.models.elements.envelope.floor_on_ground import FloorOnGround
        from trano.models.elements.envelope.internal_element import InternalElement

        neighbors = list(graph.neighbors(self))  # type: ignore
        self.boundaries = []
        windowed_wall_parameters = WindowedWallParameters.from_neighbors(neighbors)
        for wall in [ExternalWall, BaseWindow, InternalElement, FloorOnGround]:
            self.boundaries.append(
                WallParameters.from_neighbors(
                    neighbors,
                    wall,  # type: ignore
                    filter=windowed_wall_parameters.included_external_walls,
                )
            )
        self.boundaries += [windowed_wall_parameters]

    def __add__(self, other: "Space") -> "Space":
        self.name = (
            f"merge_{self.name.replace('merge', '')}_{other.name.replace('merge', '')}"
        )
        self.volume: float = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        return self
