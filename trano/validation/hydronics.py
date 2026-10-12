"""Physical checks of the water circuits, each one backed by a simulated root cause.

The thresholds come from simulations of `tests/models/single_zone_hydronic.yaml` changed one parameter at a
time (Buildings 13, three winter days):

- the storage tank of the default boiler is charged until its bottom is 5 K above the supply set point, so
  its top, which feeds the circuit, ends about 15 K above the set point (70 -> 85, 80 -> 95, 95 -> 110 degC);
- a three-way valve whose pump is upstream (the pump feeds port_1 and port_3 returns to the boiler) diverts
  the water to the return: it can reduce the flow to the emitters but not their supply temperature. When the
  supply exceeds the set point of its control, the valve closes and the rooms go cold (16 degC on average in
  `three_zones_hydronic.yaml` with the default 80 degC boiler set point and the 90 degC valve set point).
"""

from collections.abc import Iterator
from typing import Any

import networkx as nx

from trano.validation.graph import Component, ModelGraph
from trano.validation.issues import HydronicLayoutWarning, Issue
from trano.validation.parameters import parameter_classes

WATER_SPECIFIC_HEAT = 4184.0  # J/(kg.K)
BOILING_TEMPERATURE = 373.15  # K, water at 1 bar
STORAGE_OVERSHOOT = 15.0  # K, top of the tank of the default boiler above its supply set point (simulated)
THREE_WAY_VALVE_SETPOINT = 363.15  # K, template default of the three-way valve control
SETPOINT_MARGIN = 5.0  # K, above the highest supply when the set point is raised
MINIMUM_VALVE_AUTHORITY = 0.25
UNDERSIZED_FLOW_RATIO = 0.5  # nominal pump flow below half of the flow the emitters need at their nominal dT
BOUNDARY_KINDS = frozenset({"boiler", "split_valve"})


def _value(class_name: str, parameters: dict[str, Any], name: str) -> float | None:
    return parameter_classes()[class_name].value(parameters, name)


def _parameters(component: Component) -> dict[str, Any]:
    return component.data.get("parameters") or {}


def supply_set_point(boiler: Component) -> float | None:
    if boiler.control_kind != "boiler_control":
        return None
    return _value("BoilerControlParameters", boiler.control.get("parameters") or {}, "tsup_nominal")


def highest_supply(boiler: Component) -> float | None:
    """The highest temperature the boiler delivers, when known from simulations (default variant only)."""
    set_point = supply_set_point(boiler)
    if boiler.variant != "default" or set_point is None:
        return None
    return set_point + STORAGE_OVERSHOOT


def _upstream_boilers(graph: ModelGraph, flow: nx.DiGraph, node: str) -> list[Component]:
    ancestors = nx.ancestors(flow, node)  # type: ignore[no-untyped-call]
    return [
        graph.components[id_]
        for id_ in sorted(ancestors)
        if graph.components.get(id_, None) and graph.components[id_].kind == "boiler"
    ]


def _downstream_until_return(graph: ModelGraph, flow: nx.DiGraph, node: str) -> list[Component]:
    """The components after `node`, up to the return junctions and the generators."""
    seen: set[str] = set()
    stack = list(flow.successors(node))  # type: ignore[no-untyped-call]
    while stack:
        current = stack.pop()
        component = graph.components.get(current)
        if current in seen or component is None or component.kind in BOUNDARY_KINDS:
            continue
        seen.add(current)
        stack += list(flow.successors(current))  # type: ignore[no-untyped-call]
    return [graph.components[id_] for id_ in sorted(seen)]


def is_mixing(graph: ModelGraph, flow: nx.DiGraph, valve: Component) -> bool:
    """A pump after the valve draws the return through port_3: the valve mixes and sets the supply temperature."""
    return any(component.kind == "pump" for component in _downstream_until_return(graph, flow, valve.id))


def _three_way_valve_set_point(valve: Component) -> float | None:
    return (valve.control.get("parameters") or {}).get("supply_temperature_setpoint")


def derived_three_way_valve_set_point(graph: ModelGraph, flow: nx.DiGraph, valve: Component) -> float | None:
    """The set point of a diverting valve whose generator exceeds the template default, None otherwise.

    A diverting valve cannot lower the supply temperature, so its set point is kept above the supply instead
    of closing the valve.
    """
    if valve.control_kind != "three_way_valve_control" or _three_way_valve_set_point(valve) is not None:
        return None
    if is_mixing(graph, flow, valve):
        return None
    supplies = [supply for boiler in _upstream_boilers(graph, flow, valve.id) if (supply := highest_supply(boiler))]
    if not supplies or max(supplies) + SETPOINT_MARGIN <= THREE_WAY_VALVE_SETPOINT:
        return None
    return round(max(supplies) + SETPOINT_MARGIN, 2)


def derive_defaults(data: dict[str, Any], graph: ModelGraph) -> None:
    """Raise the set point of the diverting three-way valves above the boiler supply (the workaround).

    `check_three_way_valve_layouts` reports it in the warning on the layout of each such valve.
    """
    flow = graph.flow_graph()
    for valve in [component for component in graph.components.values() if component.kind == "three_way_valve"]:
        set_point = derived_three_way_valve_set_point(graph, flow, valve)
        if set_point is not None:
            valve.control.setdefault("parameters", {})["supply_temperature_setpoint"] = set_point


def check_supply_temperatures(graph: ModelGraph) -> Iterator[Issue]:
    for boiler in (component for component in graph.components.values() if component.kind == "boiler"):
        set_point, highest = supply_set_point(boiler), highest_supply(boiler)
        if set_point is not None and set_point >= BOILING_TEMPERATURE:
            yield Issue(
                "error",
                boiler.id,
                f"its supply set point ({set_point} K) is at or above the boiling "
                "point of water (373.15 K); the water media of the libraries are "
                "liquid only. Use at most 363.15 K.",
            )
        elif highest is not None and highest >= BOILING_TEMPERATURE:
            yield Issue(
                "warning",
                boiler.id,
                f"its storage tank is charged to about {highest} K (the supply set "
                f"point {set_point} K plus {STORAGE_OVERSHOOT} K), above the boiling "
                "point of water. Lower tsup_nominal to 353.15 K or less.",
            )


def _upstream_pumps(graph: ModelGraph, flow: nx.DiGraph, node: str) -> list[str]:
    ancestors = nx.ancestors(flow, node)  # type: ignore[no-untyped-call]
    return sorted(id_ for id_ in ancestors if graph.components.get(id_, None) and graph.components[id_].kind == "pump")


def _layout_consequence(graph: ModelGraph, flow: nx.DiGraph, valve: Component) -> str:
    supplies = [supply for boiler in _upstream_boilers(graph, flow, valve.id) if (supply := highest_supply(boiler))]
    derived = derived_three_way_valve_set_point(graph, flow, valve)
    explicit = _three_way_valve_set_point(valve)
    if derived is not None:
        return (
            f"WORKAROUND APPLIED: the boiler supply reaches about {max(supplies)} K, above the "
            f"{THREE_WAY_VALVE_SETPOINT} K set point of the valve, which would close it and leave the rooms cold; the "
            "set point is raised to "
            f"{derived} K so that the valve stays open, and the radiators get the boiler water unmixed."
        )
    if explicit is not None and supplies and max(supplies) > explicit:
        return (
            f"With its set point of {explicit} K below the boiler supply (about {max(supplies)} K), this valve "
            "WILL CLOSE and the rooms will go cold."
        )
    return "As long as the supply stays below its set point, the valve stays open and controls nothing."


def check_three_way_valve_layouts(graph: ModelGraph, flow: nx.DiGraph) -> Iterator[Issue]:
    """A temperature-controlled three-way valve fed by a pump: it diverts instead of mixing."""
    for valve in (component for component in graph.components.values() if component.kind == "three_way_valve"):
        if valve.control_kind != "three_way_valve_control" or is_mixing(graph, flow, valve):
            continue
        pumps = ", ".join(_upstream_pumps(graph, flow, valve.id)) or "the generator"
        yield Issue(
            "warning",
            valve.id,
            f"WRONG HYDRONIC LAYOUT. {pumps} pushes the water into this temperature-controlled three-way valve, so "
            "the valve sends the water back to the boiler through port_3 instead of mixing the return water in: "
            "it can reduce the flow to the radiators but never lower their supply temperature. "
            f"{_layout_consequence(graph, flow, valve)} "
            "FIX: place the pump after the valve and its temperature sensor (boiler -> three_way_valve -> "
            "temperature_sensor -> pump -> radiators, the return split valve in the inlets of the valve), as in "
            "the tutorial 'Hydronic heating with mixing circuits'.",
            category=HydronicLayoutWarning,
        )


def _radiator_flow(radiator: Component) -> float | None:
    parameters = _parameters(radiator)
    power = _value("RadiatorParameter", parameters, "nominal_heating_power_positive_for_heating")
    inlet = _value("RadiatorParameter", parameters, "water_inlet_temperature_at_nominal_condition")
    outlet = _value("RadiatorParameter", parameters, "water_outlet_temperature_at_nominal_condition")
    if power is None or inlet is None or outlet is None or inlet <= outlet:
        return None
    return power / (WATER_SPECIFIC_HEAT * (inlet - outlet))


def _served_radiators(graph: ModelGraph, flow: nx.DiGraph, node: str) -> list[Component]:
    return [
        component
        for component in _downstream_until_return(graph, flow, node)
        if component.kind == "radiator" and component.medium == "water"
    ]


def check_pump_flows(graph: ModelGraph, flow: nx.DiGraph) -> Iterator[Issue]:
    for pump in (component for component in graph.components.values() if component.kind == "pump"):
        given = _parameters(pump).get("m_flow_nominal")
        flows = [flow_ for radiator in _served_radiators(graph, flow, pump.id) if (flow_ := _radiator_flow(radiator))]
        if not isinstance(given, (int, float)) or not flows:
            continue
        needed = sum(flows)
        if given < UNDERSIZED_FLOW_RATIO * needed:
            yield Issue(
                "warning",
                pump.id,
                f"its nominal flow ({given} kg/s) is less than half of the {needed:.3f} "
                "kg/s its radiators need at their nominal temperatures (power / "
                "(cp x dT)): they will not deliver their power.",
            )


def check_valve_authorities(graph: ModelGraph) -> Iterator[Issue]:
    for valve in (component for component in graph.components.values() if component.kind == "valve"):
        parameters = _parameters(valve)
        valve_dp = _value("ValveParameters", parameters, "dp_valve_nominal")
        fixed_dp = _value("ValveParameters", parameters, "dp_fixed_nominal")
        if not valve_dp or fixed_dp is None:
            continue
        authority = valve_dp / (valve_dp + fixed_dp)
        if authority < MINIMUM_VALVE_AUTHORITY:
            yield Issue(
                "warning",
                valve.id,
                f"its authority dp_valve / (dp_valve + dp_fixed) is {authority:.2f}, "
                f"below {MINIMUM_VALVE_AUTHORITY}: the valve barely changes the flow "
                "until it is almost closed. Raise dp_valve_nominal.",
            )


def check_hydronics(graph: ModelGraph) -> Iterator[Issue]:
    flow = graph.flow_graph()
    yield from check_supply_temperatures(graph)
    yield from check_three_way_valve_layouts(graph, flow)
    yield from check_pump_flows(graph, flow)
    yield from check_valve_authorities(graph)
