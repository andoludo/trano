"""Building descriptions of the systems validation: one well-insulated zone and the systems under study.

Every case is built from the same 100 m2 zone (``zone``), so that the systems can be compared on the same load, and
from blocks that describe one system each: a radiator and its valve, an ideal heater, a heat pump or a boiler with
the mixing circuit that trano needs to connect a generator to its emitters, a ventilation unit. The blocks return the
dictionaries trano reads; ``render`` writes them as the YAML of a case.
"""

from __future__ import annotations

import copy
from typing import Any

import yaml

KELVIN = 273.15
FLOOR_AREA = 100.0  # [m2]
HEIGHT = 2.7  # [m]
# Set points of the rooms: 21 degC from 7 to 22 h, 16 degC at night (the schedule of EN 12831 dwellings).
HEATING_SETPOINT = 21.0 + KELVIN
HEATING_SETBACK = 16.0 + KELVIN
OCCUPIED_HOURS = (7, 22)
RADIATOR_POWER = 3000.0  # [W] at its rating temperatures, for a design heat loss of about 2.5 kW at -8 degC
RADIATOR_FLOW = RADIATOR_POWER / 4180 / 10  # [kg/s] at a 10 K temperature drop
VENTILATION_FLOW = 0.05  # [kg/s] 150 m3/h, about 0.55 air changes per hour

ZONE = "ZONE:001"
RADIATOR = "RADIATOR:001"
VALVE = "VALVE:001"
HEATING = "HEATING:001"
GENERATOR = "GENERATOR:001"
PUMP = "PUMP:001"
THREE_WAY_VALVE = "THREE_WAY_VALVE:001"
TEMPERATURE_SENSOR = "TEMPERATURE_SENSOR:001"
SPLIT_VALVE = "SPLIT_VALVE:001"
SUPPLY_DUCT, EXTRACT_DUCT = "DUCT:001", "DUCT:002"
AHU = "AHU:001"


def zone(n50: float = 3.0, window_area: float = 10.0) -> dict[str, Any]:
    """A 10 m x 10 m x 2.7 m zone of the default constructions: filled cavity walls (U 0.28), a warm flat roof
    (U 0.17), an insulated slab, low-e double glazing facing south, airtight to ``n50``, occupied from 7 to 19 h.

    Design heat loss at -8 degC outdoor and 21 degC inside (EN 12831, simplified): walls 0.8 kW, roof 0.5 kW,
    windows 0.3 kW, floor 0.4 kW, infiltration 0.4 kW: about 2.4 kW without ventilation.
    """
    return {
        "id": ZONE,
        "variant": "infiltration",
        "occupancy": {"parameters": {"occupancy": "3600*{7, 19}"}},
        "parameters": {"floor_area": FLOOR_AREA, "average_room_height": HEIGHT, "n50": n50},
        "external_boundaries": {
            "external_walls": [
                {"surface": 30.0, "azimuth": 0, "tilt": "wall", "construction": "CAVITYWALL:001"},
                {"surface": 27.0, "azimuth": 1.57, "tilt": "wall", "construction": "CAVITYWALL:001"},
                {"surface": 30.0 - window_area, "azimuth": 3.14, "tilt": "wall", "construction": "CAVITYWALL:001"},
                {"surface": 27.0, "azimuth": 4.71, "tilt": "wall", "construction": "CAVITYWALL:001"},
                {"surface": FLOOR_AREA, "azimuth": 0, "tilt": "ceiling", "construction": "FLATROOF:001"},
            ],
            "windows": [
                {
                    "surface": window_area,
                    "azimuth": 3.14,
                    "tilt": "wall",
                    "construction": "INS2AR2020:001",
                    "width": window_area / 2.0,
                    "height": 2.0,
                }
            ],
            "floor_on_grounds": [{"surface": FLOOR_AREA, "construction": "FLOORONGROUND:001"}],
        },
    }


def schedule(hours: tuple[int, int] = OCCUPIED_HOURS) -> str:
    return f"3600*{{{hours[0]}, {hours[1]}}}"


def radiator_and_valve(  # noqa: PLR0913
    supply: float,
    return_: float,
    power: float = RADIATOR_POWER,
    control_variant: str = "PController",
    valve_parameters: dict[str, Any] | None = None,
    control_parameters: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    """A radiator rated at ``supply``/``return_`` [K] in a 20 degC room, followed by its two-way valve, driven by an
    emission control on the zone temperature (``PController`` by default)."""
    control: dict[str, Any] = {
        "id": "EMISSION_CONTROL:001",
        "variant": control_variant,
        "parameters": {
            "schedule": schedule(),
            "temperature_heating_setpoint": HEATING_SETPOINT,
            "temperature_heating_setback": HEATING_SETBACK,
            **(control_parameters or {}),
        },
    }
    return [
        {
            "radiator": {
                "id": RADIATOR,
                "parameters": {
                    "nominal_heating_power_positive_for_heating": power,
                    "water_inlet_temperature_at_nominal_condition": supply,
                    "water_outlet_temperature_at_nominal_condition": return_,
                    "air_temperature_at_nominal_condition": 20.0 + KELVIN,
                },
            }
        },
        {
            "valve": {
                "id": VALVE,
                "parameters": {"m_flow_nominal": round(RADIATOR_FLOW, 4), **(valve_parameters or {})},
                "control": {"emission_control": control},
            }
        },
    ]


def ideal_heating(maximum_power: float = 8000.0) -> list[dict[str, Any]]:
    """An ideal heater holding the set points of the rooms (no cooling): the heating energy it delivers is the
    load of the zone, which isolates the effect of a ventilation system."""
    start, end = (hour * 3600 for hour in OCCUPIED_HOURS)
    heating = (
        f"[0, {HEATING_SETBACK}; {start}, {HEATING_SETBACK}; {start}, {HEATING_SETPOINT}; "
        f"{end}, {HEATING_SETPOINT}; {end}, {HEATING_SETBACK}; 86400, {HEATING_SETBACK}]"
    )
    return [
        {
            "ideal_heating_cooling": {
                "id": HEATING,
                "parameters": {
                    "heating_setpoint_schedule": heating,
                    "cooling_setpoint_schedule": f"[0, {50.0 + KELVIN}]",
                    "maximum_heating_power": maximum_power,
                    "maximum_cooling_power": 0.0,
                },
            }
        }
    ]


def air_water_heat_pump(
    power: float = 4000.0,
    supply: float = 45.0 + KELVIN,
    buffer_volume: float | None = 0.1,
    parameters: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """An air-to-water heat pump of ``power`` [W] at the design outdoor temperature and ``supply`` [K], charging a
    buffer tank of ``buffer_volume`` [m3] (a hydraulic separator when None), with trano's simple generator control
    (on when a room needs heat)."""
    block: dict[str, Any] = {
        "id": GENERATOR,
        "variant": "air_water_heat_pump",
        "parameters": {
            "nominal_heating_power": power,
            "temperature_supply_setpoint_heat_pump": supply,
            "use_storage_tank": buffer_volume is not None,
            **({"tank_volume": buffer_volume, "height_of_tank_without_insulation": 0.8} if buffer_volume else {}),
            **(parameters or {}),
        },
        "control": {"boiler_control": {"id": "GENERATOR_CONTROL:001", "variant": "simple"}},
    }
    return block


def gas_boiler(
    power: float = 6000.0,
    supply: float = 70.0 + KELVIN,
    storage: bool = True,
    parameters: dict[str, Any] | None = None,
    control_parameters: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """A gas boiler of ``power`` [W] holding ``supply`` [K]: with its storage tank and the tank-temperature control
    of trano (``storage``), or without, firing on demand under a supply temperature controller."""
    if storage:
        return {
            "id": GENERATOR,
            "parameters": {"nominal_heating_power": power, **(parameters or {})},
            "control": {
                "boiler_control": {
                    "id": "GENERATOR_CONTROL:001",
                    "parameters": {"tsup_nominal": supply, **(control_parameters or {})},
                }
            },
        }
    return {
        "id": GENERATOR,
        "variant": "without_storage",
        "parameters": {
            "nominal_heating_power": power,
            "supply_temperature_setpoint": supply,
            **(parameters or {}),
        },
        "control": {"boiler_control": {"id": "GENERATOR_CONTROL:001", "variant": "simple"}},
    }


def mixing_circuit(generator: dict[str, Any], supply: float, flow: float = RADIATOR_FLOW) -> list[dict[str, Any]]:
    """The generator feeding the radiator through a mixing circuit, the layout of the tutorial "Hydronic heating with
    mixing circuits": generator -> three-way valve -> temperature sensor -> pump -> radiator -> valve -> split
    valve, which returns the water to the generator and to the bypass of the three-way valve. The valve holds
    ``supply`` [K] after it under a PI controller; the pump runs when the radiator valve opens."""
    generator = copy.deepcopy(generator)
    generator["outlets"] = [THREE_WAY_VALVE]
    flow = round(flow, 4)
    return [
        {"boiler": generator},
        {
            "three_way_valve": {
                "id": THREE_WAY_VALVE,
                "parameters": {"m_flow_nominal": flow},
                "control": {
                    "three_way_valve_control": {
                        "id": "THREE_WAY_VALVE_CONTROL:001",
                        "parameters": {
                            "supply_temperature_setpoint": supply,
                            "controller_type": "Buildings.Controls.OBC.CDL.Types.SimpleController.PI",
                            "k": 0.1,
                            "ti": 300,
                        },
                    }
                },
                "inlets": [SPLIT_VALVE],
                "outlets": [TEMPERATURE_SENSOR],
            }
        },
        {"temperature_sensor": {"id": TEMPERATURE_SENSOR, "outlets": [PUMP]}},
        {
            "pump": {
                "id": PUMP,
                "parameters": {"m_flow_nominal": flow},
                "control": {"collector_control": None},
                "outlets": [RADIATOR],
            }
        },
        {"split_valve": {"id": SPLIT_VALVE, "inlets": [VALVE], "outlets": [GENERATOR]}},
    ]


def ventilation_unit(
    variant: str = "system_d",
    flow: float = VENTILATION_FLOW,
    effectiveness: float = 0.8,
    pressure: float = 200.0,
    parameters: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """A balanced ventilation unit of ``flow`` [kg/s] with heat recovery: the ducts of the zone and the unit."""
    ducts = {
        "ventilation_inlets": [{"duct": {"id": SUPPLY_DUCT, "parameters": {"nominal_mass_flow_rate": flow}}}],
        "ventilation_outlets": [{"duct": {"id": EXTRACT_DUCT, "parameters": {"nominal_mass_flow_rate": flow}}}],
    }
    unit = {
        "air_handling_unit": {
            "id": AHU,
            "variant": variant,
            "parameters": {
                "m_flow_nominal": flow,
                "dp_nominal": pressure,
                "heat_exchanger_effectiveness": effectiveness,
                **(parameters or {}),
            },
            "inlets": [EXTRACT_DUCT],
            "outlets": [SUPPLY_DUCT],
        }
    }
    return ducts, [unit]


def building(
    space: dict[str, Any],
    emissions: list[dict[str, Any]] | None = None,
    systems: list[dict[str, Any]] | None = None,
    ventilation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    space = copy.deepcopy(space)
    if emissions:
        space["emissions"] = emissions
    if ventilation:
        space.update(ventilation)
    description: dict[str, Any] = {"spaces": [space]}
    if systems:
        description["systems"] = systems
    return description


def render(description: dict[str, Any], header: str) -> str:
    """The YAML of a building description, on trano's default library of materials and constructions."""
    comment = "".join(f"# {line}\n" for line in header.splitlines())
    return comment + "default: !include_default\n" + yaml.safe_dump(description, sort_keys=False, width=120)
