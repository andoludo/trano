"""Checks of the connections of a model: media, circuits, emitters and controls."""

from collections.abc import Iterator
from typing import Any

import networkx as nx

from trano.validation.graph import (
    EMITTERS,
    IDEAL_RADIATOR_VARIANTS,
    Component,
    Medium,
    ModelGraph,
    space_components,
)
from trano.validation.issues import Issue

GENERATORS = frozenset({"boiler"})
RC_ONLY = frozenset({"chiller", "dhw_tank", "battery", "ev_charger"})
REQUIRED_CONTROLS = {
    "valve": "emission_control",
    "three_way_valve": "three_way_valve_control",
    "boiler": "boiler_control",
    "pump": "collector_control",
    "vav": "vav_control",
}
# VAV variant -> the variant of its vav_control: the default box takes a damper signal (y), the complex one
# exchanges its signals with the control through the data bus
VAV_CONTROL_VARIANTS = {"default": "constant", "complex": "default"}
CIRCUIT_SOURCES: dict[Medium, tuple[frozenset[str], str]] = {
    "water": (GENERATORS, "a generator (boiler or heat pump)"),
    "air": (frozenset({"air_handling_unit"}), "an air handling unit"),
}


def check_edges(graph: ModelGraph) -> Iterator[Issue]:
    """Self-connections, contradictory directions and connections between incompatible components."""
    declared = set(graph.declared_edges)
    for source, target in sorted(declared):
        if source == target:
            yield Issue("error", source, "is connected to itself. Remove its own id from its inlets/outlets.")
            continue
        if (target, source) in declared and source < target:
            yield Issue(
                "error",
                source,
                f"is declared both upstream and downstream of {target}. "
                "Keep only the direction in which the fluid flows.",
            )
        if source in graph.components and target in graph.components:
            yield from _check_pair(graph.components[source], graph.components[target])


def _check_pair(first: Component, second: Component) -> Iterator[Issue]:
    for component, other in ((first, second), (second, first)):
        if component.kind in EMITTERS and component.medium is None and other.medium is not None:
            yield Issue(
                "error",
                component.id,
                f"has no water circuit (variant '{component.variant}'), but "
                f"{first.id} -> {second.id} connects it to one. Remove it from the "
                "inlets/outlets, or use the variant 'default' followed by a valve.",
            )
    if first.medium and second.medium and first.medium != second.medium:
        yield Issue(
            "error",
            first.id,
            f"carries {first.medium} but feeds {second.id}, which carries "
            f"{second.medium}. Water and air components cannot be connected.",
        )


def check_spaces(data: dict[str, Any]) -> Iterator[Issue]:
    """One emitter per space; a water radiator always followed by its valve."""
    for space in data.get("spaces") or []:
        emissions = space_components(space, "emissions")
        emitters = [(kind, data_) for kind, data_ in emissions if kind in EMITTERS]
        if len(emitters) > 1:
            listed = ", ".join(data_["id"] for _, data_ in emitters)
            yield Issue(
                "error",
                space["id"],
                f"has {len(emitters)} heat emitters ({listed}); a space takes one. "
                "Keep one emitter and size it for the space.",
            )
        water_radiators = [
            data_
            for kind, data_ in emitters
            if kind == "radiator" and (data_.get("variant") or "default") not in IDEAL_RADIATOR_VARIANTS
        ]
        has_valve = any(kind == "valve" for kind, _ in emissions)
        if has_valve and not water_radiators:
            yield Issue(
                "error",
                space["id"],
                "has a valve in its emissions but no water radiator: the space gets "
                "no heat. Add a radiator before the valve, or remove the valve.",
            )
        if water_radiators and not has_valve:
            yield Issue(
                "error",
                water_radiators[0]["id"],
                "is a water radiator without a valve. Add a valve with "
                f"an emission_control after it in the emissions of "
                f"{space['id']}.",
            )


def check_controls(graph: ModelGraph, is_rc: bool) -> Iterator[Issue]:
    """The components that cannot work without their control, or with a control of another variant."""
    for component in graph.components.values():
        if component.control_kind:
            yield from _check_control_variant(component)
            continue
        required = REQUIRED_CONTROLS.get(component.kind)
        if required and not is_rc:
            yield Issue(
                "error",
                component.id,
                f"a {component.kind} needs a control to drive it. Add `control: {{{required}: }}`.",
            )
        if component.kind == "radiator" and component.variant in IDEAL_RADIATOR_VARIANTS:
            yield Issue(
                "error",
                component.id,
                f"an ideal radiator (variant '{component.variant}') needs a "
                "control. Add `control: {emission_control: }`.",
            )
        if component.kind == "air_handling_unit" and component.variant == "default":
            yield Issue(
                "error",
                component.id,
                "an air handling unit of variant 'default' needs a control. Add `control: {ahu_control: }`.",
            )


def _check_control_variant(component: Component) -> Iterator[Issue]:
    expected = VAV_CONTROL_VARIANTS.get(component.variant) if component.kind == "vav" else None
    given = str(component.control.get("variant") or "default")
    if expected is not None and component.control_kind == "vav_control" and given != expected:
        yield Issue(
            "error",
            component.id,
            f"a VAV of variant '{component.variant}' is driven by a vav_control of "
            f"variant '{expected}', not '{given}'. Set `variant: {expected}` on its "
            "vav_control.",
        )


def check_topology(graph: ModelGraph) -> Iterator[Issue]:
    """Orphans, port counts, closed circuits and three-way valve sensors (not for the flat RC models)."""
    flow = graph.flow_graph()
    yield from _check_orphans(graph, flow)
    for component in graph.components.values():
        if component.id in flow:
            yield from _check_degree(graph, flow, component)
    for medium in CIRCUIT_SOURCES:
        yield from _check_circuits(graph, flow, medium)


def _check_orphans(graph: ModelGraph, flow: nx.DiGraph) -> Iterator[Issue]:
    for component in graph.components.values():
        if component.kind in RC_ONLY:
            yield Issue(
                "error",
                component.id,
                f"a {component.kind} is only modelled by the 'mpc' library. "
                "Remove it, or translate the model with the 'mpc' library.",
            )
        elif component.space is None and flow.degree(component.id) == 0:
            yield Issue(
                "error",
                component.id,
                f"the {component.kind} is not connected to anything and would be "
                "left out of the model. Add it to the inlets/outlets of its "
                "neighbours, or remove it.",
            )


def _neighbours(graph: ModelGraph, flow: nx.DiGraph, component: Component) -> list[Component]:
    nodes = {*flow.predecessors(component.id), *flow.successors(component.id)}  # type: ignore[no-untyped-call]
    return [graph.components[node] for node in nodes if node in graph.components]


def _check_degree(graph: ModelGraph, flow: nx.DiGraph, component: Component) -> Iterator[Issue]:
    inlets, outlets = flow.in_degree(component.id), flow.out_degree(component.id)
    neighbours = _neighbours(graph, flow, component)
    if component.kind == "three_way_valve":
        if inlets + outlets != 3:
            yield Issue(
                "error",
                component.id,
                f"has {inlets + outlets} connections; a three-way valve has "
                "exactly three: the supply (port_1), the outlet to the emitters "
                "(port_2) and the bypass to the return (port_3).",
            )
        sensors = [
            node
            for node in flow.successors(component.id)  # type: ignore[no-untyped-call]
            if graph.components.get(node, None) and graph.components[node].kind == "temperature_sensor"
        ]
        if component.control_kind == "three_way_valve_control" and not sensors:
            yield Issue(
                "error",
                component.id,
                "its three_way_valve_control measures the temperature after the "
                "valve, but no temperature_sensor follows it. Add a "
                "temperature_sensor to its outlets, ahead of the emitters.",
            )
    if component.kind == "split_valve" and sum(n.kind == "three_way_valve" for n in neighbours) != 1:
        yield Issue(
            "error",
            component.id,
            "a split_valve joins the returns of the emitters (port_1) with the "
            "bypass of exactly one three_way_valve (port_3). Connect it to one "
            "three_way_valve.",
        )
    if component.kind == "heat_meter_sensor":
        measured = any(graph.is_measurement(*edge) for edge in graph.declared_edges if component.id in edge)
        if not measured or any(n.kind not in {"pump", "boiler"} for n in neighbours):
            yield Issue(
                "error",
                component.id,
                "a heat_meter_sensor sits between a boiler and a pump and reads the "
                "return temperature of a temperature_sensor: connect it to a boiler "
                "or pump only, and list one temperature_sensor in its inlets.",
            )
    if component.kind in {"valve", "radiator"} and component.medium == "water" and (inlets > 1 or outlets > 1):
        yield Issue(
            "error",
            component.id,
            f"has {inlets} inlets and {outlets} outlets; a {component.kind} has one "
            "of each. Join the branches with a split_valve.",
        )


def _check_circuits(graph: ModelGraph, flow: nx.DiGraph, medium: Medium) -> Iterator[Issue]:
    """Every component of the medium on a loop through its source, and every water loop with a pump."""
    sources, source_label = CIRCUIT_SOURCES[medium]
    nodes = [component.id for component in graph.components.values() if component.medium == medium]
    subgraph = flow.subgraph([*nodes, *(graph.spaces if medium == "air" else [])])  # type: ignore[no-untyped-call]
    for circuit in nx.strongly_connected_components(subgraph):
        members = sorted(node for node in circuit if node in graph.components)
        if not members or all(subgraph.degree(node) == 0 for node in circuit):
            continue  # a space without ventilation, or an orphan already reported
        if len(circuit) == 1:
            node = members[0]
            missing = (
                "nothing upstream"
                if not list(subgraph.predecessors(node))
                else "nothing downstream"
                if not list(subgraph.successors(node))
                else "no path back to it"
            )
            yield Issue(
                "error",
                node,
                f"is not on a closed {medium} circuit ({missing}). Every {medium} component "
                f"must be on a loop through {source_label}: complete its inlets/outlets.",
            )
            continue
        kinds = {graph.components[node].kind for node in members}
        if not kinds & sources:
            yield Issue(
                "error",
                members[0],
                f"the {medium} loop {', '.join(members)} does not go through {source_label}. Connect the loop to one.",
            )
        elif medium == "water" and "pump" not in kinds:
            yield Issue(
                "error",
                members[0],
                f"the water loop {', '.join(members)} has no pump: nothing drives the "
                "flow to the emitters. Add a pump after the generator.",
            )
