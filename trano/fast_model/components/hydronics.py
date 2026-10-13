"""Hydronic circuits of the ``FastModel`` library, built from the system graph of the YAML.

The graph trano builds for the Buildings library (boiler, pumps, three-way valves, split valves, temperature
sensors, radiators and their valves, edges in the flow direction) is mapped onto ``FastModel.Components.Hydronics``:

* every element becomes a two-port component (the three-way valve has a supply and a bypass inlet);
* a split valve, and any element with several inlets or outlets, becomes an ideal ``Mixer`` and/or ``Splitter``
  (Buildings joins the ports of a connection set the same way);
* one ``Reference`` per closed circuit, on the return of its production, drops the redundant mass balance;
* the flows are set (decision 2 of the plan): each radiator valve sets the flow of its branch, the three-way valve
  the share of supply water, mass balances the rest. The branch flow is the control by default
  (``FlowValve``); ``valve_input="opening"`` uses the equal-percentage ``Valve`` instead;
* a boiler is ``Hydronics.Boiler`` (Buildings ``BoilerPolynomial``, its part load ratio is the control) or, with
  ``production="supply_temperature"``, an ideal production whose supply temperature is the control.

Radiators follow ``Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2`` with the parameters trano gives it:
the conductance of each element is solved from the nominal conditions as Buildings does at initialization.
"""

import math
import re
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

import networkx as nx

from pydantic import BaseModel, ConfigDict, Field

from trano.fast_model.components.model import Instance, TopInput
from trano.mpc.estimation import sanitize_name

if TYPE_CHECKING:
    from trano.elements.base import BaseElement
    from trano.elements.space import Space
    from trano.elements.system import Boiler, Radiator, System
    from trano.topology import Network

WATER_HEAT_CAPACITY = 4184.0  # [J/(kg.K)] Buildings.Media.Water
WATER_DENSITY = 995.586  # [kg/m3] Buildings.Media.Water
DRY_MASS_HEAT_CAPACITY = 500.0  # [J/(kg.K)] of the dry mass lumped to the water (RadiatorEN442_2.mSenFac)
# defaults of trano's Buildings boiler template (PartialBoilerWithoutStorage)
BOILER_WATER_VOLUME = 1.5e-6  # [m3/W]
BOILER_DRY_MASS = 1.5e-3  # [kg/W]
BOILER_LOSS_CONDUCTANCE = 1e-4  # [W/K per W]
BOILER_ROOM_TEMPERATURE = 288.15  # [K]


class HydronicSettings(BaseModel):
    model_config = ConfigDict(frozen=True)

    valve_input: Literal["flow", "opening"] = Field(
        "flow", description="Control of a radiator valve: the flow of its branch or its opening"
    )
    production: Literal["boiler", "supply_temperature"] = Field(
        "boiler", description="Boiler model, part load ratio as control, or ideal supply temperature as control"
    )


@dataclass
class HydronicCircuit:
    """Instances, connections and inputs of the hydronic circuits of a building."""

    instances: list[Instance] = field(default_factory=list)
    connections: list[tuple[str, str]] = field(default_factory=list)
    inputs: list[TopInput] = field(default_factory=list)
    heated_zones: set[str] = field(default_factory=set)  # zones whose emitters are in a circuit


# ------------------------------------------------------------------------------------------- radiators
def _root(function: Callable[[float], float], low: float, high: float, tolerance: float = 1e-10) -> float:
    """Root of a monotonic function bracketed by ``[low, high]``, by bisection."""
    f_low = function(low)
    if f_low * function(high) > 0:
        raise ValueError(f"No root between {low} and {high}.")
    while high - low > tolerance * max(1.0, abs(low)):
        middle = (low + high) / 2
        f_middle = function(middle)
        if (f_middle > 0) == (f_low > 0):
            low, f_low = middle, f_middle
        else:
            high = middle
    return (low + high) / 2


def radiator_conductance(  # noqa: PLR0913
    power: float,
    t_inlet: float,
    t_outlet: float,
    t_air: float = 293.15,
    t_radiant: float = 293.15,
    radiant_fraction: float = 0.35,
    exponent: float = 1.24,
    elements: int = 1,
) -> float:
    """``UAEle`` of ``RadiatorEN442_2``: the conductance of each element giving the nominal power.

    As the initial equations of Buildings: the nominal flow cools by ``QEle/(m*cp)`` in each element, whose water
    is at its outlet temperature, and each element emits ``UA*(f*(TW - TRad)^n + (1 - f)*(TW - TAir)^n)``.
    """
    flow = power / WATER_HEAT_CAPACITY / (t_inlet - t_outlet)

    def element_power(conductance: float, t_water: float) -> float:
        return conductance * float(
            radiant_fraction * max(t_water - t_radiant, 0.0) ** exponent
            + (1 - radiant_fraction) * max(t_water - t_air, 0.0) ** exponent
        )

    def total_power(conductance: float) -> float:
        t_upstream, total = t_inlet, 0.0
        for _ in range(elements):
            # the water of the element is at the temperature it leaves with: solve TW = Tup - Q(TW)/(m*cp)
            def balance(t_water: float, t_up: float = t_upstream) -> float:
                return t_water - t_up + element_power(conductance, t_water) / flow / WATER_HEAT_CAPACITY

            t_water = _root(balance, min(t_air, t_radiant), t_upstream)
            total += element_power(conductance, t_water)
            t_upstream = t_water
        return total

    mean_difference = (t_inlet + t_outlet) / 2 - ((1 - radiant_fraction) * t_air + radiant_fraction * t_radiant)
    guess = power / mean_difference**exponent / elements
    return _root(lambda conductance: total_power(conductance) - power, guess / 10, guess * 10)


def radiator_instance(radiator: "Radiator", name: str, zone: str | None) -> Instance:
    parameters = radiator.parameters
    power = float(parameters.nominal_heating_power_positive_for_heating)  # type: ignore[union-attr]
    elements = int(parameters.number_of_elements_used_in_the_discretization or 1)  # type: ignore[union-attr]
    radiant_fraction = float(parameters.fraction_radiant_heat_transfer)  # type: ignore[union-attr]
    exponent = float(parameters.exponent_for_heat_transfer)  # type: ignore[union-attr]
    water_volume = parameters.water_volume_of_radiator or 5.8e-6 * power  # type: ignore[union-attr]
    dry_mass = parameters.dry_mass_of_radiator_that_will_be_lumped_to_water_heat_capacity or 0.0263 * power  # type: ignore[union-attr]
    conductance = radiator_conductance(
        power,
        t_inlet=float(parameters.water_inlet_temperature_at_nominal_condition),  # type: ignore[union-attr]
        t_outlet=float(parameters.water_outlet_temperature_at_nominal_condition),  # type: ignore[union-attr]
        t_air=float(parameters.air_temperature_at_nominal_condition),  # type: ignore[union-attr]
        t_radiant=float(parameters.radiative_temperature_at_nominal_condition),  # type: ignore[union-attr]
        radiant_fraction=radiant_fraction,
        exponent=exponent,
        elements=elements,
    )
    capacity = WATER_DENSITY * float(water_volume) * WATER_HEAT_CAPACITY + DRY_MASS_HEAT_CAPACITY * float(dry_mass)
    return Instance(
        name=name,
        component="Hydronics.Radiator",
        description=f"Radiator {radiator.name} (EN 442, {power:g} W nominal)",
        zone=zone,
        system=name,
        parameters={
            "nEle": elements,
            "UAEle": conductance,
            "n": exponent,
            "fraRad": radiant_fraction,
            "C": capacity,
        },
    )


# ------------------------------------------------------------------------------------------- boilers
def boiler_efficiency(curve: str, coefficients: list[float]) -> list[float]:
    """Coefficients a1..a6 of ``eta = a1 + a2*y + a3*y^2 + (a4 + a5*y + a6*y^2)*T`` for a Buildings curve."""
    kind = curve.rsplit(".", 1)[-1]
    if kind == "Constant":
        terms = coefficients[:1]
    elif kind == "Polynomial":
        if len(coefficients) > 3:
            raise NotImplementedError("Boiler efficiency polynomials of degree above 2 in the part load ratio.")
        terms = coefficients
    elif kind == "QuadraticLinear":
        terms = coefficients[:6]
    else:
        raise ValueError(f"Unknown boiler efficiency curve {curve}.")
    return [*terms, *[0.0] * (6 - len(terms))]


def _numbers(value: object) -> list[float]:
    return [float(number) for number in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(value))]


def boiler_instance(boiler: "Boiler", name: str, control: str) -> Instance:
    """``Hydronics.Boiler`` with the parameters of trano's Buildings boiler (``PartialBoilerWithoutStorage``)."""
    parameters = boiler.parameters
    variant = getattr(boiler, "variant", "default")
    if variant not in ("default", "without_storage"):
        raise NotImplementedError(
            f"Production {boiler.name} ({variant}): the FastModel library models boilers and air-to-water heat pumps; "
            "use HydronicSettings(production='supply_temperature') for an ideal production."
        )
    if str(parameters.use_storage_tank).lower() == "true":  # type: ignore[union-attr]
        raise NotImplementedError(f"Boiler {boiler.name} has a storage tank: the FastModel library has no tank yet.")
    power = float(parameters.nominal_heating_power)  # type: ignore[union-attr]
    a = boiler_efficiency(
        str(parameters.effcur),  # type: ignore[union-attr]
        _numbers(parameters.coefficients_for_efficiency_curve),  # type: ignore[union-attr]
    )
    t_nominal = float(parameters.temperature_used_to_compute_nominal_efficiency)  # type: ignore[union-attr]
    eta_nominal = a[0] + a[1] + a[2] + (a[3] + a[4] + a[5]) * t_nominal
    water_volume = parameters.water_volume or BOILER_WATER_VOLUME * power  # type: ignore[union-attr]
    dry_mass = parameters.dry_mass or BOILER_DRY_MASS * power  # type: ignore[union-attr]
    return Instance(
        name=name,
        component="Hydronics.Boiler",
        description=f"Boiler {boiler.name} ({power:g} W)",
        system=name,
        parameters={
            "Q_flow_nominal": power,
            **{f"a{i}": value for i, value in enumerate(a, start=1)},
            "eta_nominal": eta_nominal,
            # Buildings 13 BoilerPolynomial subtracts its UA loss twice (see the boiler template of trano)
            "UA": 2 * float(parameters.loss_conductance or BOILER_LOSS_CONDUCTANCE * power),  # type: ignore[union-attr]
            "TAmb": BOILER_ROOM_TEMPERATURE,
            "C": WATER_DENSITY * float(water_volume) * WATER_HEAT_CAPACITY + DRY_MASS_HEAT_CAPACITY * float(dry_mass),
        },
        inputs={"y": control},
    )


# ------------------------------------------------------------------------------------------- the graph
def _hydronic_classes() -> tuple[type, ...]:
    from trano.elements.system import (
        Boiler,
        HeatMeterSensor,
        Pump,
        Radiator,
        SplitValve,
        TemperatureSensor,
        ThreeWayValve,
        Valve,
    )

    return Boiler, HeatMeterSensor, Pump, Radiator, SplitValve, TemperatureSensor, ThreeWayValve, Valve


def hydronic_graph(network: "Network") -> nx.DiGraph:
    """The water circuits: hydronic elements joined in the flow direction."""
    classes = _hydronic_classes()
    nodes = [node for node in network.graph.nodes if isinstance(node, classes)]
    graph: nx.DiGraph = network.graph.subgraph(nodes).copy()  # type: ignore[no-untyped-call]
    graph.remove_nodes_from([node for node in list(graph.nodes) if graph.degree(node) == 0])
    return graph


def zone_of(element: "System", network: "Network") -> "Space | None":
    from trano.elements.space import Space

    return next(
        (node for node in network.graph.nodes if isinstance(node, Space) and element in node.emissions),
        None,
    )


def hydronic_emitters(network: "Network") -> list["Radiator"]:
    from trano.elements.system import Radiator

    return [node for node in hydronic_graph(network).nodes if isinstance(node, Radiator)]


def _by_name(elements: "Iterable[BaseElement]") -> list["BaseElement"]:
    """Elements in the order of their names: the generated model does not depend on the order of the graph."""
    return sorted(elements, key=lambda element: str(element.name))


@dataclass
class _Node:
    """The component of an element and its fluid ports, in the order of its predecessors and successors."""

    element: "BaseElement"
    inlets: list[str]
    outlets: list[str]


class _Builder:
    def __init__(
        self,
        network: "Network",
        zone_ports: Callable[[str], tuple[str, str]],
        settings: HydronicSettings,
    ) -> None:
        self.network = network
        self.graph = hydronic_graph(network)
        self.zone_ports = zone_ports
        self.settings = settings
        self.circuit = HydronicCircuit()
        self.nodes: dict[BaseElement, _Node] = {}

    def add(self, instance: Instance) -> str:
        self.circuit.instances.append(instance)
        return instance.name

    def control(  # noqa: PLR0913
        self,
        name: str,
        unit: str,
        description: str,
        system: str,
        zone: str | None = None,
        bounds: tuple[float, float] = (0.0, 1.0),
    ) -> str:
        minimum, maximum = bounds
        self.circuit.inputs.append(
            TopInput(
                name=name,
                unit=unit,
                description=description,
                control=True,
                system=system,
                zone=zone,
                minimum=minimum,
                maximum=maximum,
            )
        )
        return name

    def branch_zone(self, valve: "BaseElement") -> str | None:
        """Zone of the radiator next to a valve (the zone whose heating the valve controls)."""
        from trano.elements.system import Radiator

        for neighbor in [*self.upstream(valve), *self.downstream(valve)]:
            if isinstance(neighbor, Radiator) and (space := zone_of(neighbor, self.network)):
                return sanitize_name(str(space.name))
        return None

    def build(self) -> HydronicCircuit:
        for element in _by_name(self.graph.nodes):
            self.nodes[element] = self.element(element)
        references = self.references()
        for upstream in _by_name(self.graph.nodes):
            for downstream in self.downstream(upstream):
                self.connect(upstream, downstream, references)
        return self.circuit

    def upstream(self, element: "BaseElement") -> list["BaseElement"]:
        """Elements feeding ``element``, in the order of its inlets."""
        return _by_name(self.graph.pred[element])

    def downstream(self, element: "BaseElement") -> list["BaseElement"]:
        """Elements fed by ``element``, in the order of its outlets."""
        return _by_name(self.graph.succ[element])

    def connect(
        self, upstream: "BaseElement", downstream: "BaseElement", references: set[tuple["BaseElement", "BaseElement"]]
    ) -> None:
        outlet = self.nodes[upstream].outlets[self.downstream(upstream).index(downstream)]
        inlet = self.nodes[downstream].inlets[self.upstream(downstream).index(upstream)]
        if (upstream, downstream) not in references:
            self.circuit.connections.append((outlet, inlet))
            return
        reference = self.add(
            Instance(
                name=f"{sanitize_name(str(upstream.name))}_ref",
                component="Hydronics.Reference",
                description="Reference of the circuit (expansion vessel)",
            )
        )
        self.circuit.connections += [(outlet, f"{reference}.inlet"), (f"{reference}.outlet", inlet)]

    # ------------------------------------------------------------------ elements
    def element(self, element: "BaseElement") -> _Node:
        from trano.elements.system import (
            Boiler,
            Pump,
            Radiator,
            SplitValve,
            ThreeWayValve,
            Valve,
        )

        name = sanitize_name(str(element.name))
        n_in, n_out = self.graph.in_degree(element), self.graph.out_degree(element)
        if isinstance(element, SplitValve):
            return self.junction(name, n_in, n_out, element)
        if isinstance(element, ThreeWayValve):
            return self.three_way_valve(element, name, n_in, n_out)
        if isinstance(element, Radiator):
            space = zone_of(element, self.network)
            zone = sanitize_name(str(space.name)) if space else None
            self.add(radiator_instance(element, name, zone))
            if zone:
                convective, radiative = self.zone_ports(zone)
                self.circuit.connections += [(f"{name}.heatPortCon", convective), (f"{name}.heatPortRad", radiative)]
                self.circuit.heated_zones.add(zone)
        elif isinstance(element, Valve):
            self.valve(element, name)
        elif isinstance(element, Pump):
            parameters = element.parameters
            self.add(
                Instance(
                    name=name,
                    component="Hydronics.Pump",
                    description=f"Pump {element.name}",
                    system=name,
                    parameters={"dp_nominal": float(parameters.dp_nominal)},  # type: ignore[union-attr]
                )
            )
        elif isinstance(element, Boiler):
            self.production(element, name)
        else:  # temperature sensors, heat meters
            self.add(
                Instance(
                    name=name,
                    component="Hydronics.TemperatureSensor",
                    description=f"Sensor {element.name}",
                    system=name,
                )
            )
        node = _Node(element, [f"{name}.inlet"], [f"{name}.outlet"])
        return self.fan_out(self.fan_in(node, n_in), n_out)

    def three_way_valve(self, element: "BaseElement", name: str, n_in: int, n_out: int) -> _Node:
        """A mixing valve (supply and bypass inlets) or a diverting one (emitter and bypass outlets)."""
        position = self.control(f"{name}_y", "1", f"Position of {element.name} (share to the emitters)", name)
        if n_in == 2 and n_out == 1:
            self.add(
                Instance(
                    name=name,
                    component="Hydronics.ThreeWayValve",
                    description=f"Three-way mixing valve {element.name}",
                    system=name,
                    inputs={"y": position},
                )
            )
            supply, bypass = sorted(self.upstream(element), key=self.from_production)
            ports = {supply: f"{name}.port_1", bypass: f"{name}.port_3"}
            return _Node(element, [ports[upstream] for upstream in self.upstream(element)], [f"{name}.port_2"])
        if n_in == 1 and n_out == 2:
            self.add(
                Instance(
                    name=name,
                    component="Hydronics.DivertingValve",
                    description=f"Three-way diverting valve {element.name}",
                    system=name,
                    parameters={"m_flow_nominal": float(element.parameters.m_flow_nominal)},  # type: ignore[union-attr]
                    inputs={"y": position},
                )
            )
            emitters, bypass = sorted(self.downstream(element), key=self.to_production, reverse=True)
            ports = {emitters: f"{name}.port_2", bypass: f"{name}.port_3"}
            return _Node(element, [f"{name}.port_1"], [ports[downstream] for downstream in self.downstream(element)])
        raise ValueError(f"Three-way valve {element.name} has {n_in} inlets and {n_out} outlets: 2 and 1 or 1 and 2.")

    def production(self, element: "Boiler", name: str) -> None:
        if self.settings.production == "supply_temperature":
            self.add(
                Instance(
                    name=name,
                    component="Hydronics.PrescribedSupply",
                    description=f"Production {element.name}: supply temperature prescribed",
                    system=name,
                    inputs={
                        "TSup": self.control(
                            f"{name}_TSup", "K", f"Supply temperature of {element.name}", name, bounds=(283.15, 363.15)
                        )
                    },
                )
            )
            return
        if getattr(element, "variant", "default") == "air_water_heat_pump":
            from trano.fast_model.components.heat_pump import heat_pump_instance

            speed = self.control(f"{name}_y", "1", f"Compressor speed of {element.name}", name)
            self.add(heat_pump_instance(element, name, speed))
            return
        self.add(
            boiler_instance(element, name, self.control(f"{name}_y", "1", f"Part load ratio of {element.name}", name))
        )

    def valve(self, element: "BaseElement", name: str) -> None:
        parameters = element.parameters
        nominal = float(parameters.m_flow_nominal)  # type: ignore[union-attr]
        zone = self.branch_zone(element)
        if self.settings.valve_input == "flow":
            self.add(
                Instance(
                    name=name,
                    component="Hydronics.FlowValve",
                    description=f"Valve {element.name}: sets the flow of its branch (at most {nominal:g} kg/s)",
                    system=name,
                    inputs={
                        "m_flow": self.control(
                            f"{name}_m_flow", "kg/s", f"Flow through {element.name}", name, zone, (0.0, nominal)
                        )
                    },
                )
            )
            return
        valve_drop = float(parameters.dp_valve_nominal)  # type: ignore[union-attr]
        fixed_drop = float(parameters.dp_fixed_nominal or 0.0)  # type: ignore[union-attr]
        linearized = str(parameters.use_linear_relation_between_m_flow_and_dp_for_any_flow_rate).lower() == "true"  # type: ignore[union-attr]
        self.add(
            Instance(
                name=name,
                component="Hydronics.Valve",
                description=f"Equal-percentage valve {element.name}",
                system=name,
                parameters={
                    "m_flow_nominal": nominal,
                    "R": float(parameters.rangeability),  # type: ignore[union-attr]
                    "l": float(parameters.valve_leakage),  # type: ignore[union-attr]
                    "a": valve_drop / (valve_drop + fixed_drop),
                    "p": 1.0 if linearized else 2.0,
                },
                inputs={"y": self.control(f"{name}_y", "1", f"Opening of {element.name}", name, zone)},
            )
        )

    def junction(self, name: str, n_in: int, n_out: int, element: "BaseElement") -> _Node:
        inlets, outlets = [f"{name}_mix.inlet[{i}]" for i in range(1, n_in + 1)], [f"{name}_mix.outlet"]
        if n_in > 1 or n_out == 1:
            self.add(
                Instance(
                    name=f"{name}_mix",
                    component="Hydronics.Mixer",
                    description=f"{element.name}: mixing",
                    parameters={"n": n_in},
                )
            )
        else:
            inlets, outlets = [], []
        if n_out > 1:
            self.add(
                Instance(
                    name=f"{name}_spl",
                    component="Hydronics.Splitter",
                    description=f"{element.name}: split",
                    parameters={"n": n_out},
                )
            )
            if outlets:
                self.circuit.connections.append((outlets[0], f"{name}_spl.inlet"))
            else:
                inlets = [f"{name}_spl.inlet"]
            outlets = [f"{name}_spl.outlet[{i}]" for i in range(1, n_out + 1)]
        return _Node(element, inlets, outlets)

    def fan_in(self, node: _Node, n_in: int) -> _Node:
        if n_in <= 1:
            return node
        name = self.add(
            Instance(
                name=f"{sanitize_name(str(node.element.name))}_mix",
                component="Hydronics.Mixer",
                description=f"Inlets of {node.element.name}",
                parameters={"n": n_in},
            )
        )
        self.circuit.connections.append((f"{name}.outlet", node.inlets[0]))
        return _Node(node.element, [f"{name}.inlet[{i}]" for i in range(1, n_in + 1)], node.outlets)

    def fan_out(self, node: _Node, n_out: int) -> _Node:
        if n_out <= 1:
            return node
        name = self.add(
            Instance(
                name=f"{sanitize_name(str(node.element.name))}_spl",
                component="Hydronics.Splitter",
                description=f"Outlets of {node.element.name}",
                parameters={"n": n_out},
            )
        )
        self.circuit.connections.append((node.outlets[0], f"{name}.inlet"))
        return _Node(node.element, node.inlets, [f"{name}.outlet[{i}]" for i in range(1, n_out + 1)])

    # ------------------------------------------------------------------ topology
    def productions(self) -> list["BaseElement"]:
        from trano.elements.system import Boiler

        return [node for node in _by_name(self.graph.nodes) if isinstance(node, Boiler)]

    def from_production(self, node: "BaseElement") -> float:
        """Length of the shortest path from a production to ``node`` (the supply is closer than the return)."""
        lengths = [
            nx.shortest_path_length(self.graph, production, node)
            for production in self.productions()
            if nx.has_path(self.graph, production, node)  # type: ignore[no-untyped-call]
        ]
        return min(lengths, default=math.inf)

    def to_production(self, node: "BaseElement") -> float:
        """Length of the shortest path from ``node`` back to a production (a bypass returns sooner)."""
        lengths = [
            nx.shortest_path_length(self.graph, node, production)
            for production in self.productions()
            if nx.has_path(self.graph, node, production)  # type: ignore[no-untyped-call]
        ]
        return min(lengths, default=math.inf)

    def references(self) -> set[tuple["BaseElement", "BaseElement"]]:
        """One edge per circuit, the return into its production, gets the ``Reference``."""
        references = set()
        productions = set(self.productions())
        for circuit in nx.weakly_connected_components(self.graph):
            members = _by_name(circuit)
            edges = [(upstream, node) for node in members if node in productions for upstream in self.upstream(node)]
            edges = edges or [(node, downstream) for node in members for downstream in self.downstream(node)]
            if edges:
                references.add(edges[0])
        return references


def hydronic_circuit(
    network: "Network", zone_ports: Callable[[str], tuple[str, str]], settings: HydronicSettings | None = None
) -> HydronicCircuit:
    """The hydronic circuits of ``network``; ``zone_ports(zone)`` gives the convective and radiative heat ports."""
    return _Builder(network, zone_ports, settings or HydronicSettings()).build()
