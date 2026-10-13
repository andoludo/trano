"""Mechanical ventilation of the ``FastModel`` library, after trano's Buildings air handling unit ``system_d``.

The Buildings unit (``PartialSystemD``) blows the supply and extract air with two constant-pressure fans through a
constant-effectiveness heat exchanger; the ducts of the YAML lead the supply air to each zone and the extract air
back. ``Ventilation.HeatRecoveryUnit`` keeps that physics with balanced flows: each zone receives its nominal duct
flow (scaled by the speed ``y``, the control) at ``TOut + eps*(TExtract - TOut)``, the extract air being the zone air
mixed by flow; the fans heat the supply air and draw electricity.
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING

from trano.fast_model.components.model import Instance, TopInput
from trano.mpc.estimation import sanitize_name

if TYPE_CHECKING:
    from trano.elements.system import AirHandlingUnit
    from trano.topology import Network

AIR_DENSITY = 1.2  # [kg/m3]
AIR_HEAT_CAPACITY = 1006.0  # [J/(kg.K)]
FAN_EFFICIENCY = 0.5  # overall efficiency of the constant-pressure fans
DEFAULT_EFFECTIVENESS = 0.8  # eps of PartialSystemD
DESIGN_SPEED = 1.0  # bounds of the unit speed in the interface: the design flow


@dataclass(frozen=True)
class VentilatedZone:
    zone: str
    nominal_flow: float  # [kg/s] of its supply duct


def _number(value: object, default: float) -> float:
    try:
        return float(str(value).strip("'\" "))
    except ValueError:
        return default


def ventilation_units(network: "Network") -> list["AirHandlingUnit"]:
    from trano.elements.system import AirHandlingUnit

    return sorted(
        (node for node in network.graph.nodes if isinstance(node, AirHandlingUnit)), key=lambda unit: str(unit.name)
    )


def ventilated_zones(unit: "AirHandlingUnit", network: "Network") -> list[VentilatedZone]:
    """The zones a unit supplies through its supply ducts, with the nominal flow of each duct."""
    from trano.elements.space import Space
    from trano.elements.system import Duct

    zones = []
    for duct in sorted(network.graph.succ[unit], key=lambda node: str(node.name)):
        if not isinstance(duct, Duct):
            continue
        for space in network.graph.succ[duct]:
            if isinstance(space, Space):
                flow = getattr(duct.parameters, "nominal_mass_flow_rate", None)
                zones.append(VentilatedZone(zone=sanitize_name(str(space.name)), nominal_flow=_number(flow, 0.0)))
    if zones and all(zone.nominal_flow <= 0 for zone in zones):  # no duct flows: share the unit flow evenly
        total = _number(unit.parameters.m_flow_nominal, 0.0)  # type: ignore[union-attr]
        zones = [VentilatedZone(zone=zone.zone, nominal_flow=total / len(zones)) for zone in zones]
    return zones


def ventilation_instance(unit: "AirHandlingUnit", zones: list[VentilatedZone], control: str) -> Instance:
    parameters = unit.parameters
    flow = sum(zone.nominal_flow for zone in zones)
    effectiveness = _number(getattr(parameters, "heat_exchanger_effectiveness", None), DEFAULT_EFFECTIVENESS)
    pressure = _number(getattr(parameters, "dp_nominal", None), 200.0)
    supply_rise = _number(getattr(parameters, "supply_dp_nominal", None), pressure)
    return_rise = _number(getattr(parameters, "return_dp_nominal", None), pressure)
    supply_fan = supply_rise * flow / AIR_DENSITY / FAN_EFFICIENCY
    return_fan = return_rise * flow / AIR_DENSITY / FAN_EFFICIENCY
    # the fans dissipate their power into the air: the supply fan after the heat exchanger, the return fan before it
    heating = (supply_fan + effectiveness * return_fan) / (flow * AIR_HEAT_CAPACITY) if flow > 0 else 0.0
    name = sanitize_name(str(unit.name))
    return Instance(
        name=name,
        component="Ventilation.HeatRecoveryUnit",
        description=f"Balanced ventilation {unit.name} with heat recovery (eps {effectiveness:g})",
        system=name,
        parameters={
            "n": len(zones),
            "mNom": [zone.nominal_flow for zone in zones],
            "eps": effectiveness,
            "dTFan": heating,
            "PNom": supply_fan + return_fan,
        },
        inputs={"y": control, "TOut": "TOut"},
    )


@dataclass
class VentilationSystems:
    instances: list[Instance]
    connections: list[tuple[str, str]]
    inputs: list[TopInput]


def ventilation_systems(network: "Network", zone_port: str = "{zone}_air.port") -> VentilationSystems:
    """The ventilation units of ``network``, connected to the air of their zones."""
    systems = VentilationSystems(instances=[], connections=[], inputs=[])
    for unit in ventilation_units(network):
        zones = ventilated_zones(unit, network)
        if not zones:
            continue
        name = sanitize_name(str(unit.name))
        # held at the design flow by default, as the Buildings unit runs: without an indoor air quality model, an
        # optimizer free to lower it would stop the ventilation to save heat (relax the minimum to modulate it)
        control = TopInput(
            name=f"{name}_y",
            unit="1",
            description=f"Air flow of {unit.name} relative to the nominal one",
            control=True,
            system=name,
            minimum=DESIGN_SPEED,
            maximum=DESIGN_SPEED,
        )
        systems.inputs.append(control)
        systems.instances.append(ventilation_instance(unit, zones, control.name))
        systems.connections += [
            (f"{name}.zone[{index}]", zone_port.format(zone=zone.zone)) for index, zone in enumerate(zones, start=1)
        ]
    return systems
