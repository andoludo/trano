"""The components of a model, as written in the YAML file, and the flow between them."""

from collections import defaultdict
from dataclasses import dataclass, field
from itertools import pairwise
from typing import Any, Literal

import networkx as nx

from trano.validation.issues import Issue

WATER = frozenset(
    {"boiler", "pump", "split_valve", "three_way_valve", "valve", "temperature_sensor", "heat_meter_sensor"}
)
AIR = frozenset({"air_handling_unit", "duct", "vav"})
EMITTERS = frozenset({"radiator", "ideal_heating_cooling"})
IDEAL_RADIATOR_VARIANTS = frozenset({"ideal", "idealbus"})
SPACE_COMPONENT_KEYS = ("emissions", "ventilation_inlets", "ventilation_outlets")

Medium = Literal["water", "air"]


@dataclass
class Component:
    """A system, emission or ventilation component of the model."""

    id: str
    kind: str
    data: dict[str, Any]
    space: str | None = None

    @property
    def variant(self) -> str:
        return str(self.data.get("variant") or "default")

    @property
    def medium(self) -> Medium | None:
        """The fluid flowing through the component, None for the components without a fluid port."""
        if self.kind == "radiator":
            if self.variant in IDEAL_RADIATOR_VARIANTS:
                return None
            from trano.elements.library.components import external_variant_has_fluid_ports

            external_fluid = external_variant_has_fluid_ports(self.kind, self.variant)
            if external_fluid is not None:
                return "water" if external_fluid else None
            return "water"
        if self.kind in WATER:
            return "water"
        return "air" if self.kind in AIR else None

    @property
    def control_kind(self) -> str | None:
        control = self.data.get("control")
        return next(iter(control), None) if isinstance(control, dict) else None

    @property
    def control(self) -> dict[str, Any]:
        control = self.data.get("control")
        if not isinstance(control, dict) or not control:
            return {}
        return next(iter(control.values())) or {}


@dataclass
class ModelGraph:
    """The components of a model and the flow edges between them.

    Declared edges come from the `inlets`/`outlets` of the systems; implicit edges from the order of the
    emissions and ventilation components of each space (radiator -> valve, inlets -> space -> outlets).
    """

    components: dict[str, Component] = field(default_factory=dict)
    declared_edges: list[tuple[str, str]] = field(default_factory=list)
    implicit_edges: list[tuple[str, str]] = field(default_factory=list)
    spaces: list[str] = field(default_factory=list)

    def is_measurement(self, source: str, target: str) -> bool:
        """The temperature a heat meter reads from a sensor: a signal, not a flow of water."""
        kinds = {self.components[node].kind for node in (source, target) if node in self.components}
        return kinds == {"heat_meter_sensor", "temperature_sensor"}

    def flow_graph(self) -> nx.DiGraph:
        graph = nx.DiGraph()
        graph.add_nodes_from(self.components)
        graph.add_nodes_from(self.spaces)
        graph.add_edges_from(
            (source, target)
            for source, target in [*self.declared_edges, *self.implicit_edges]
            if source in graph and target in graph and not self.is_measurement(source, target)
        )
        return graph


def entry_component(entry: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    """The YAML key and the content of a one-key entry such as `- pump: {id: PUMP:001}`."""
    kind, data = next(iter(entry.items()))
    return kind, data or {}


def space_components(space: dict[str, Any], key: str) -> list[tuple[str, dict[str, Any]]]:
    return [entry_component(entry) for entry in space.get(key) or [] if entry]


def _add_space(graph: ModelGraph, space: dict[str, Any], occurrences: dict[str, list[Component]]) -> None:
    space_id = space["id"]
    graph.spaces.append(space_id)
    for key in SPACE_COMPONENT_KEYS:
        for kind, data in space_components(space, key):
            occurrences[data["id"]].append(Component(data["id"], kind, data, space_id))
    emissions, inlets, outlets = (
        [data["id"] for _, data in space_components(space, key)] for key in SPACE_COMPONENT_KEYS
    )
    graph.implicit_edges += [*pairwise(emissions), *pairwise(inlets), *pairwise(outlets)]
    graph.implicit_edges += [(inlets[-1], space_id)] if inlets else []
    graph.implicit_edges += [(space_id, outlets[0])] if outlets else []


def _is_transfer_duct(components: list[Component]) -> bool:
    """The same ventilation component listed in two spaces: the air leaving one space enters the other."""
    first = components[0]
    return first.kind in AIR and all(
        component.space is not None and component.kind == first.kind and component.data == first.data
        for component in components
    )


def build_graph(data: dict[str, Any]) -> tuple[ModelGraph, list[Issue]]:
    """Gather the components and edges of the model, reporting the ids used by several components."""
    graph = ModelGraph()
    occurrences: dict[str, list[Component]] = defaultdict(list)
    for space in data.get("spaces") or []:
        _add_space(graph, space, occurrences)
    for entry in data.get("systems") or []:
        kind, component_data = entry_component(entry)
        component = Component(component_data["id"], kind, component_data)
        occurrences[component.id].append(component)
        graph.declared_edges += [(component.id, outlet) for outlet in component_data.get("outlets") or []]
        graph.declared_edges += [(inlet, component.id) for inlet in component_data.get("inlets") or []]

    issues = []
    for id_, components in occurrences.items():
        graph.components[id_] = components[0]
        if len(components) > 1 and not _is_transfer_duct(components):
            places = ", ".join(f"{component.kind} in {component.space or 'systems'}" for component in components)
            issues.append(
                Issue(
                    "error",
                    id_,
                    f"the id is used by {len(components)} components ({places}). Give each component its own id.",
                )
            )
    return graph, issues
