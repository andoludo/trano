from functools import cached_property
from math import ceil
from typing import ClassVar, Optional, Union, TYPE_CHECKING

from networkx import Graph
from pydantic import Field, PrivateAttr, model_validator

from trano.elements.aggregated_envelope import AggregatedEnvelope
from trano.elements.base import BaseElement
from trano.elements.construction import Construction, Glass
from trano.elements.envelope import (
    BaseExternalWall,
    BaseFloorOnGround,
    BaseSimpleWall,
    BaseInternalElement,
    BaseWindow,
    ExternalWall,
    FloorOnGround,
    InternalElement,
    MergedBaseWall,
    MergedExternalWall,
    MergedWindows,
    WallParameters,
    WindowedWallParameters,
    assign_windows_to_walls,
)
from trano.elements.common_base import BaseParameter
from trano.elements.library.parameters import param_from_config
from trano.elements.system import AirHandlingUnit, BaseOccupancy, Emission, Occupancy, System
from trano.elements.types import BaseVariant, ContainerTypes
from trano.elements.zone_template import RectangularZone
from trano.exceptions import UnknownComponentVariantError

if TYPE_CHECKING:
    from trano.elements.library.library import Library
    from trano.topology import Network

MAX_X_SPACES = 3

ExternalBoundary = Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround"]
EnvelopeComponent = Union[ExternalBoundary, "MergedBaseWall"]


def _zero_occupancy_parameters() -> BaseParameter:
    """Parameters of an occupancy with no one in and no gains, for zones whose gain input must be connected."""
    parameters = param_from_config("Occupancy")
    if parameters is None:
        raise UnknownComponentVariantError("No occupancy parameters are defined.")
    return parameters(gain="[0; 0; 0]", heat_gain_if_occupied="0")


class SpaceVariant(BaseVariant):
    infiltration: str = "infiltration"
    # IDEAS only: the envelope is rendered with IDEAS.Buildings.Components.RectangularZoneTemplate.
    rectangular_zone: str = "rectangular_zone"


def merge_external_boundaries(boundaries: list[ExternalBoundary]) -> list[EnvelopeComponent]:
    """Lump the walls (and windows) sharing a construction into array components."""
    external_walls = [boundary for boundary in boundaries if boundary.type in ["ExternalWall", "ExternalDoor"]]
    windows = [boundary for boundary in boundaries if boundary.type == "Window"]
    merged_external_walls = MergedExternalWall.from_base_elements(external_walls)
    merged_windows = MergedWindows.from_base_windows(windows)  # type: ignore
    return (
        merged_external_walls
        + merged_windows
        + [boundary for boundary in boundaries if boundary.type not in ["ExternalWall", "Window", "ExternalDoor"]]
    )


def _get_controllable_element(elements: list[System]) -> Optional["System"]:
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


class BaseSpace(BaseElement):
    counter: ClassVar[int] = 0
    name: str
    external_boundaries: list[Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround"]]
    internal_elements: list["BaseInternalElement"] = Field(default=[])
    boundaries: list[WallParameters] | None = None
    emissions: list[System] = Field(default=[])
    ventilation_inlets: list[System] = Field(default=[])
    ventilation_outlets: list[System] = Field(default=[])
    occupancy: BaseOccupancy | None = None
    container_type: ContainerTypes = "envelope"
    merged_external_boundaries: list[Union["BaseExternalWall", "BaseWindow", "BaseFloorOnGround", "MergedBaseWall"]] = (
        Field(default_factory=list)
    )
    _merged_for_variant: str | None = PrivateAttr(default=None)

    def model_post_init(self, __context) -> None:  # type: ignore # noqa: ANN001
        self._assign_space()

    def _assign_space(self) -> None:
        for emission in self.emissions + self.ventilation_inlets + self.ventilation_outlets:
            if emission.control:
                emission.control.space_name = self.name
        if self.occupancy:
            self.occupancy.space_name = self.name

    @property
    def number_merged_external_boundaries(self) -> int:
        return sum([boundary.length for boundary in self.merged_external_boundaries + self.internal_elements])

    @property
    def number_ventilation_ports(self) -> int:
        return 2 + 1  # databus

    @model_validator(mode="after")
    def _merged_external_boundaries_validator(
        self,
    ) -> "BaseSpace":
        # Runs again on every assignment (validate_assignment): only rebuild when the variant
        # changed, which keeps the merged components stable and avoids re-entering here.
        if self._merged_for_variant == self.variant:
            return self
        self._merged_for_variant = self.variant
        self.merged_external_boundaries = self._merged_envelope()
        return self

    def _merged_envelope(self) -> list[EnvelopeComponent]:
        """Array components of the envelope; with the zone template only the surfaces it cannot hold."""
        assign_windows_to_walls(self.external_boundaries)  # type: ignore[arg-type]
        if self.uses_zone_template:
            return merge_external_boundaries(self.rectangular_zone.external_surfaces)
        return merge_external_boundaries(self.external_boundaries)

    def _zone_requires_occupancy(self, network: "Network") -> bool:
        """Whether the zone of the library takes its gains from an input that must be connected."""
        library_data = self.get_library_data(network.library)
        return bool(library_data and library_data.requires_occupancy)

    @property
    def uses_zone_template(self) -> bool:
        return self.variant == SpaceVariant.rectangular_zone

    @cached_property
    def rectangular_zone(self) -> RectangularZone:
        """Envelope mapped onto IDEAS' RectangularZoneTemplate (variant `rectangular_zone`)."""
        return RectangularZone.from_boundaries(
            self.external_boundaries,
            height=self.parameters.average_room_height,  # type: ignore[union-attr]
            floor_area=self.parameters.floor_area,  # type: ignore[union-attr]
        )

    def envelope_components(self, library: "Library") -> list[EnvelopeComponent]:
        """Envelope elements rendered as components of their own next to the space."""
        if self.uses_zone_template or library.merged_external_boundaries:
            return self.merged_external_boundaries
        return list(self.external_boundaries)

    def template_constructions(self) -> set[Construction | Glass]:
        """Constructions rendered inside the space component rather than by a wall component."""
        return self.rectangular_zone.constructions() if self.uses_zone_template else set()

    def get_controllable_emission(self) -> Optional["System"]:
        return _get_controllable_element(self.emissions)

    def assign_position(self) -> None:
        x, y = [
            250 * (Space.counter % MAX_X_SPACES),
            150 * ceil(Space.counter / MAX_X_SPACES),
        ]
        self.position.set_global(x, y)
        Space.counter += 1

        for i, emission in enumerate(self.emissions):
            emission.position.set_global(x + i * 30, y - 75)
        if self.occupancy:
            self.occupancy.position.set_global(x - 50, y)

    def set_child_position(self) -> None:
        if self.occupancy:
            self.occupancy.position.set_global(self.position.x_global - 15, self.position.y_global)
            self.occupancy.position.set_container(self.position.x_container - 15, self.position.y_container)
        for i, ext in enumerate(self.merged_external_boundaries):
            ext.position.set_global(self.position.x_global + 15, self.position.y_global + 10 * i)
            ext.position.set_container(self.position.x_container + 15, self.position.y_container + 10 * i)

    def find_emission(self) -> Optional["Emission"]:
        emissions = [emission for emission in self.emissions if isinstance(emission, Emission)]
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
        """Constructions of the Buildings zone, grouped from the envelope elements connected to the space."""
        neighbors = list(graph.neighbors(self))  # type: ignore
        windowed_walls = WindowedWallParameters.from_neighbors(neighbors)
        kinds: list[type[BaseSimpleWall]] = [ExternalWall, InternalElement, FloorOnGround]
        self.boundaries = [
            WallParameters.from_neighbors(self.name, neighbors, kind, filter=windowed_walls.included_external_walls)
            for kind in kinds
        ]
        self.boundaries.append(windowed_walls)

    @cached_property
    def aggregated_envelope(self) -> AggregatedEnvelope:
        """Envelope per orientation and lumped RC elements for the AixLib reduced-order and ISO 13790 zones."""
        return AggregatedEnvelope.from_boundaries(self.external_boundaries)

    def __add__(self, other: "BaseSpace") -> "BaseSpace":
        self.name = f"merge_{self.name.replace('merge', '')}_{other.name.replace('merge', '')}"
        self.volume: float = self.volume + other.volume
        self.external_boundaries += other.external_boundaries
        assign_windows_to_walls(self.external_boundaries)  # type: ignore[arg-type]
        # Views derived from the envelope are cached: drop them so they are rebuilt from the merged envelope.
        for derived_envelope in ("aggregated_envelope", "rectangular_zone"):
            self.__dict__.pop(derived_envelope, None)
        return self


class Space(BaseSpace):
    def add_to_network(self, network: "Network") -> None:
        network.add_node(self)
        if not self.template:
            raise UnknownComponentVariantError(
                f"No Space component template for variant '{self.variant}' in library {network.library.name}."
            )
        for boundary in self.envelope_components(network.library):
            network.add_node(boundary)
            network.graph.add_edge(
                self,
                boundary,
            )
        emission = self.find_emission()
        if emission:
            network.add_node(emission)
            network.graph.add_edge(
                self,
                emission,
            )
            network._add_subsequent_systems(self.emissions)
        if self.occupancy is None and self._zone_requires_occupancy(network):
            self.occupancy = Occupancy(name=f"no_occupancy_{self.name}", parameters=_zero_occupancy_parameters())
            self.occupancy.space_name = self.name
        if self.occupancy:
            network.add_node(self.occupancy)
            network.connect_system(self, self.occupancy)
        # Assumption: first element always the one connected to the space.
        if self.get_ventilation_inlet():
            network.add_node(self.get_ventilation_inlet())  # type: ignore
            network.graph.add_edge(self.get_ventilation_inlet(), self)
        if self.get_ventilation_outlet():
            network.add_node(self.get_ventilation_outlet())  # type: ignore
            network.graph.add_edge(self, self.get_ventilation_outlet())
        # The rest is connected to each other
        network._add_subsequent_systems(self.ventilation_outlets)
        network._add_subsequent_systems(self.ventilation_inlets)
        self.assign_position()  # TODO: this is not relevant anymore?

    def processing(self, network: "Network", include_container: bool = False) -> None:
        from trano.elements import VAVControl

        _neighbors = []
        if self.get_last_ventilation_inlet():
            _neighbors += list(
                network.graph.predecessors(self.get_last_ventilation_inlet())  # type: ignore
            )
        if self.get_last_ventilation_outlet():
            _neighbors += list(
                network.graph.predecessors(self.get_last_ventilation_outlet())  # type: ignore
            )
        neighbors = list(set(_neighbors))
        controllable_ventilation_elements = list(
            filter(
                None,
                [
                    _get_controllable_element(self.ventilation_inlets),
                    _get_controllable_element(self.ventilation_outlets),
                ],
            )
        )
        for controllable_element in controllable_ventilation_elements:
            if controllable_element.control and isinstance(controllable_element.control, VAVControl):
                controllable_element.control.ahu = next((n for n in neighbors if isinstance(n, AirHandlingUnit)), None)

        self.get_neighhors(network.graph)
        self.process_figures(include_container=include_container)
