"""Checks of the parameter values: strictly positive quantities and consistent pairs of values.

The range a single value must lie in is in the schema (``minimum_value``/``maximum_value``). LinkML's bounds
are inclusive, so a quantity that must be strictly positive is marked ``positive: true`` in
``parameters.yaml`` and checked here, with the relations between several values of the same component.
"""

import re
from collections.abc import Iterator
from contextlib import suppress
from dataclasses import dataclass, field
from functools import cache
from pathlib import Path
from typing import Any

import yaml

from trano.validation.graph import SPACE_COMPONENT_KEYS, entry_component, space_components
from trano.validation.issues import Issue

PARAMETERS_PATH = Path(__file__).parents[1].joinpath("data_models", "parameters.yaml")
_IFABSENT = re.compile(r"^(?:float|int)\((.*)\)$")

# YAML key of an object with parameters -> its parameter class in parameters.yaml
PARAMETER_CLASSES = {
    "space": "SpaceParameter",
    "boiler": "BoilerParameters",
    "pump": "PumpParameters",
    "radiator": "RadiatorParameter",
    "valve": "ValveParameters",
    "three_way_valve": "ThreeWayValveParameters",
    "split_valve": "SplitValveParameters",
    "temperature_sensor": "SensorParameters",
    "heat_meter_sensor": "SensorParameters",
    "ideal_heating_cooling": "IdealHeatingCoolingParameter",
    "air_handling_unit": "AhuParameters",
    "vav": "VavParameters",
    "duct": "DuctParameters",
    "chiller": "ChillerParameters",
    "dhw_tank": "DhwTankParameters",
    "battery": "BatteryParameters",
    "ev_charger": "EvChargerParameters",
    "photovoltaic": "PhotovoltaicParameters",
    "emission_control": "EmissionControlParameters",
    "boiler_control": "BoilerControlParameters",
}


@dataclass
class ParameterClass:
    """The strictly positive attributes and the defaults of a parameter class; `names` maps short names too."""

    positive: set[str] = field(default_factory=set)
    defaults: dict[str, float] = field(default_factory=dict)
    names: dict[str, str] = field(default_factory=dict)

    def value(self, parameters: dict[str, Any], name: str) -> float | None:
        """The value given for `name`, under its name or its short name, else its default."""
        for key, canonical in self.names.items():
            if canonical == name and isinstance(parameters.get(key), (int, float)):
                return float(parameters[key])
        return self.defaults.get(name)


@cache
def parameter_classes(path: Path = PARAMETERS_PATH) -> dict[str, ParameterClass]:
    classes: dict[str, ParameterClass] = {}
    for class_name, definition in yaml.safe_load(path.read_text()).items():
        spec = ParameterClass()
        for name, attribute in (definition.get("attributes") or {}).items():
            spec.names[name] = name
            if attribute.get("short_name"):
                spec.names[attribute["short_name"]] = name
            if attribute.get("positive"):
                spec.positive.add(name)
            match = _IFABSENT.match(str(attribute.get("ifabsent", "")))
            if match and attribute.get("range") in {"float", "integer"}:
                with suppress(ValueError):
                    spec.defaults[name] = float(match.group(1))
        classes[class_name] = spec
    return classes


def parameters_of(data: dict[str, Any]) -> Iterator[tuple[str, str, dict[str, Any]]]:
    """(id, YAML key, parameters) of every space, component and control of the model."""
    entries: list[tuple[str, dict[str, Any]]] = []
    for space in data.get("spaces") or []:
        yield space["id"], "space", space.get("parameters") or {}
        entries += [entry for key in SPACE_COMPONENT_KEYS for entry in space_components(space, key)]
    entries += [entry_component(entry) for key in ("systems", "solar") for entry in data.get(key) or [] if entry]
    for kind, component in entries:
        yield component.get("id", kind), kind, component.get("parameters") or {}
        for control_kind, control in (component.get("control") or {}).items():
            yield (control or {}).get("id", control_kind), control_kind, (control or {}).get("parameters") or {}


def _is_not_positive(value: Any) -> bool:  # noqa: ANN401
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value <= 0


def check_positive_parameters(data: dict[str, Any]) -> Iterator[Issue]:
    for owner, kind, parameters in parameters_of(data):
        spec = parameter_classes().get(PARAMETER_CLASSES.get(kind, ""))
        if spec is None:
            continue
        for key, value in parameters.items():
            if spec.names.get(key) in spec.positive and _is_not_positive(value):
                yield Issue("error", owner, f"{key} = {value}, but it must be strictly positive.")


def check_surfaces(data: dict[str, Any]) -> Iterator[Issue]:
    for space in data.get("spaces") or []:
        for key, boundaries in (space.get("external_boundaries") or {}).items():
            for index, boundary in enumerate(boundaries or []):
                if _is_not_positive(boundary.get("surface")):
                    yield Issue(
                        "error",
                        space["id"],
                        f"{key}[{index}] has a surface of {boundary['surface']} m2; it must be strictly positive.",
                    )
    for wall in data.get("internal_walls") or []:
        if _is_not_positive(wall.get("surface")):
            yield Issue(
                "error",
                f"internal wall {wall.get('space_1')}-{wall.get('space_2')}",
                f"has a surface of {wall['surface']} m2; it must be strictly positive.",
            )


def check_layers(data: dict[str, Any]) -> Iterator[Issue]:
    """Layers with a thickness, materials that conduct heat (density and capacity may be 0: a massless layer)."""
    for source in (data, data.get("default") or {}):
        for key in ("constructions", "glazings"):
            for construction in source.get(key) or []:
                for index, layer in enumerate(construction.get("layers") or []):
                    if _is_not_positive(layer.get("thickness")):
                        yield Issue(
                            "error",
                            construction["id"],
                            f"layer {index} is {layer['thickness']} m thick; the thickness must be strictly positive.",
                        )
        for key in ("material", "gas", "glass_material"):
            for material in source.get(key) or []:
                if _is_not_positive(material.get("thermal_conductivity")):
                    yield Issue(
                        "error",
                        material["id"],
                        f"thermal_conductivity = {material['thermal_conductivity']}, but it must be strictly positive.",
                    )


def _radiator(owner: str, parameters: dict[str, Any]) -> Iterator[Issue]:
    spec = parameter_classes()["RadiatorParameter"]
    inlet = spec.value(parameters, "water_inlet_temperature_at_nominal_condition")
    outlet = spec.value(parameters, "water_outlet_temperature_at_nominal_condition")
    air = spec.value(parameters, "air_temperature_at_nominal_condition")
    if inlet is not None and outlet is not None and inlet <= outlet:
        yield Issue(
            "error",
            owner,
            f"its nominal water inlet temperature ({inlet} K) must be above its nominal "
            f"outlet temperature ({outlet} K), e.g. 353.15 and 333.15 for 80/60 degC.",
        )
    elif outlet is not None and air is not None and outlet <= air:
        yield Issue(
            "error",
            owner,
            f"its nominal water outlet temperature ({outlet} K) must be above the nominal "
            f"air temperature ({air} K): the radiator would not heat.",
        )


def _emission_control(owner: str, parameters: dict[str, Any]) -> Iterator[Issue]:
    spec = parameter_classes()["EmissionControlParameters"]
    setpoint = spec.value(parameters, "temperature_heating_setpoint")
    setback = spec.value(parameters, "temperature_heating_setback")
    if setpoint is not None and setback is not None and setback > setpoint:
        yield Issue(
            "error",
            owner,
            f"its setback temperature ({setback} K) is above its heating set point "
            f"({setpoint} K); the setback is the lower, unoccupied set point.",
        )


def _boiler_control(owner: str, parameters: dict[str, Any]) -> Iterator[Issue]:
    spec = parameter_classes()["BoilerControlParameters"]
    supply = spec.value(parameters, "tsup_nominal")
    switch_off = spec.value(parameters, "threshold_to_switch_off_boiler")
    if supply is not None and switch_off is not None and switch_off <= supply:
        # simulated: the boiler still heats the rooms, but cycles on a tank charged below the set point
        yield Issue(
            "warning",
            owner,
            f"threshold_to_switch_off_boiler ({switch_off} K) is not above tsup_nominal "
            f"({supply} K): the tank is charged below the supply set point. Use "
            "tsup_nominal + 5 K (the default).",
        )


CONSISTENCY_CHECKS = {"radiator": _radiator, "emission_control": _emission_control, "boiler_control": _boiler_control}


def check_consistency(data: dict[str, Any]) -> Iterator[Issue]:
    for owner, kind, parameters in parameters_of(data):
        if kind in CONSISTENCY_CHECKS:
            yield from CONSISTENCY_CHECKS[kind](owner, parameters)


def check_parameters(data: dict[str, Any]) -> list[Issue]:
    return [*check_positive_parameters(data), *check_surfaces(data), *check_layers(data), *check_consistency(data)]
