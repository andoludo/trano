"""Inventory of the YAML fields of a specification: can each one be changed at run time?

For every numeric field the inventory gives the chain YAML field -> Modelica parameter path(s) ->
run-time name(s) in ``<Model>_init.xml`` (:mod:`trano.runtime.resolution`), the attributes of those
names in the init file, and a status:

- ``changeable``: the run-time parameters take the YAML value and pass the run-time checks;
- ``derived``: trano transforms the value in Python; a calibration sets the run-time parameters listed
  (with the transformation) instead, and they pass the checks;
- ``frozen``: the executable cannot take the change (evaluated, bound or structural parameters);
- ``no_impact``: the change is taken but does not move the outputs of the scenario;
- ``not_used``: the library's model does not take the field;
- ``missing``: a run-time parameter could not be found.

The static part (no simulation) is here; the run-time checks that settle ``changeable`` are in
:mod:`trano.runtime.checks`.
"""

import csv
import enum
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, fields
from pathlib import Path

from trano.runtime.init_xml import ModelVariable, ParameterKind
from trano.runtime.resolution import Resolution, ResolutionKind
from trano.runtime.specification import FieldValue, YamlField, perturbed

RELATIVE_PERTURBATION = 0.2
TEMPERATURE_PERTURBATION = 2.0  # [K]
SCHEDULE_SHIFT = 1800.0  # [s]
ZERO_PERTURBATION = 0.05  # absolute change of a bounded field at zero
AZIMUTH_PERTURBATION = 0.2  # [rad] absolute change of an azimuth at zero
_BOUNDED = re.compile(
    r"(efficiency|effectiveness|fraction|transmittance|reflectance|absorptivity|emissivity|transmissivity)"
)


class Status(enum.StrEnum):
    changeable = "changeable"
    derived = "derived"
    frozen = "frozen"
    no_impact = "no_impact"
    not_used = "not_used"
    missing = "missing"


class Priority(enum.StrEnum):
    high = "high"
    medium = "medium"
    low = "low"


@dataclass(frozen=True)
class Group:
    name: str
    pattern: re.Pattern[str]
    priority: Priority
    high: re.Pattern[str] | None = None  # fields of the group with a high priority (calibration targets)


GROUPS = (
    Group(
        "Ground",
        re.compile(r"floor_on_grounds\[[^]]+\]\.(ground_temperature|perimeter|average_indoor_temperature)$"),
        Priority.medium,
        re.compile(r"ground_temperature$"),
    ),
    Group(
        "Opaque materials",
        re.compile(r"^material\["),
        Priority.medium,
        re.compile(r"^material\[(EPS|MINERAL_WOOL|PUR)[^]]*\]\.thermal_conductivity$"),
    ),
    Group("Layers", re.compile(r"^constructions\[.*\.thickness$"), Priority.medium),
    Group(
        "Glazing",
        re.compile(r"^(glass_material|gas|glazings)\[|\.windows\[[^]]+\]\.frame_fraction$"),
        Priority.medium,
        re.compile(r"(solar_transmittance|u_value_frame)$"),
    ),
    Group("PV", re.compile(r"^solar\["), Priority.medium),
    Group(
        "Geometry",
        re.compile(r"\.(surface|azimuth)$|^internal_walls\["),
        Priority.low,
    ),
    Group(
        "Space",
        re.compile(r"^spaces\[[^]]+\]\.(parameters|occupancy)\."),
        Priority.medium,
        re.compile(r"\.n50$"),
    ),
    Group(
        "Controls", re.compile(r"\.control\..*emission_control\."), Priority.medium, re.compile(r"temperature_heating")
    ),
    Group("Emission", re.compile(r"\.emissions\["), Priority.medium),
    Group(
        "Heat pump",
        re.compile(r"^systems\[[^]]+\]\.boiler\."),
        Priority.medium,
        re.compile(r"(nominal_heating_power|temperature_supply_setpoint_heat_pump)$"),
    ),
    Group("Hydronics", re.compile(r"^systems\[[^]]+\]\.(pump|three_way_valve)\."), Priority.low),
    Group(
        "Ventilation",
        re.compile(r"^systems\[[^]]+\]\.air_handling_unit\.|\.ventilation_(inlets|outlets)\["),
        Priority.medium,
        re.compile(r"(heat_exchanger_effectiveness|m_flow_nominal|nominal_mass_flow_rate)$"),
    ),
)


def group_of(path: str) -> tuple[str, Priority]:
    """The parameter group of a YAML path and its priority for the run-time checks."""
    for group in GROUPS:
        if group.pattern.search(path):
            priority = Priority.high if group.high is not None and group.high.search(path) else group.priority
            return group.name, priority
    return "Other", Priority.low


def perturbation(yaml_field: YamlField) -> FieldValue:
    """The perturbed value of a field, chosen to stay physical.

    +20 % by default; -20 % for quantities bounded by one (efficiencies, optical properties, fractions);
    +2 K for temperatures (-2 K for a setback); schedules shifted by half an hour; an absolute change for a
    field at zero.
    """
    name, value = yaml_field.name, yaml_field.value
    if isinstance(value, str):
        return perturbed(value, 0.0, shift=SCHEDULE_SHIFT)
    if "temperature" in name and isinstance(value, int | float):
        return value - TEMPERATURE_PERTURBATION if "setback" in name else value + TEMPERATURE_PERTURBATION
    if _BOUNDED.search(name):
        if isinstance(value, list):
            return [item * (1 - RELATIVE_PERTURBATION) if item else ZERO_PERTURBATION for item in value]
        return value * (1 - RELATIVE_PERTURBATION) if value else ZERO_PERTURBATION
    if name == "azimuth" and value == 0:
        return AZIMUTH_PERTURBATION
    return perturbed(value, RELATIVE_PERTURBATION)


@dataclass
class InventoryRow:
    """One row of ``parameter_inventory_<library>.csv``."""

    yaml_path: str
    modelica_paths: str = ""
    runtime_names: str = ""
    unit: str = ""
    default: str = ""
    causality: str = ""
    variability: str = ""
    is_value_changeable: str = ""
    readback_ok: str = ""
    impact_metric: str = ""
    equivalence_ok: str = ""
    status: str = ""
    reason: str = ""
    group: str = field(default="", metadata={"csv": False})
    priority: str = field(default="", metadata={"csv": False})

    @classmethod
    def columns(cls) -> list[str]:
        return [item.name for item in fields(cls) if item.metadata.get("csv", True)]

    def as_csv(self) -> dict[str, str]:
        return {column: getattr(self, column) for column in self.columns()}


def _join(values: Iterable[str]) -> str:
    return ";".join(dict.fromkeys(values))


def _format(value: FieldValue) -> str:
    if isinstance(value, list):
        return "[" + ", ".join(f"{item:g}" for item in value) + "]"
    return str(value)


def frozen_mechanism(variable: ModelVariable) -> str:
    """Why the executable refuses an override, from the attributes of the init file."""
    if variable.kind == ParameterKind.evaluated:
        return "evaluated at translation (calculatedParameter with a literal value)"
    if variable.kind == ParameterKind.bound:
        return "bound to other parameters (calculatedParameter computed at initialisation)"
    if variable.kind == ParameterKind.alias:
        return f"alias of {variable.alias_variable}"
    return f"not a parameter ({variable.variability})"


def static_row(
    resolution: Resolution,
    variables: Mapping[str, ModelVariable],
    mechanisms: Mapping[str, str] | None = None,
) -> InventoryRow:
    """The row of a resolved field, classified from the init file only (no simulation).

    ``mechanisms`` gives, by regular expression of run-time names, the reason a library evaluates a
    parameter (``Evaluate=true`` in its source, a ``final`` modifier, ...).
    """
    group, priority = group_of(resolution.path)
    row = InventoryRow(
        yaml_path=resolution.path,
        modelica_paths=_join(resolution.modelica_paths),
        runtime_names=_join(resolution.runtime_names),
        default=_format(resolution.value),
        group=group,
        priority=priority,
    )
    if resolution.kind == ResolutionKind.not_used:
        row.status = Status.not_used
        row.reason = resolution.not_used_reason or "the model of the library does not change with the field"
        return row
    if resolution.kind == ResolutionKind.structural:
        row.status = Status.frozen
        row.reason = (
            f"structural in trano: {resolution.error}"
            if resolution.error
            else f"structural: run-time parameters appear or disappear ({_join(resolution.structural_names[:5])})"
        )
        return row
    if resolution.kind == ResolutionKind.unresolved:
        row.status = Status.missing
        row.reason = "the model changes with the field but no run-time parameter of the adapter does"
        return row
    present = [variables[name] for name in resolution.runtime_names if name in variables]
    absent = [name for name in resolution.runtime_names if name not in variables]
    row.unit = _join(variable.unit for variable in present if variable.unit)
    row.causality = _join(variable.causality for variable in present)
    row.variability = _join(variable.variability for variable in present)
    changeable = {variable.is_value_changeable for variable in present}
    row.is_value_changeable = "mixed" if len(changeable) > 1 else str(changeable.pop()).lower() if changeable else ""
    if absent:
        row.status = Status.missing
        row.reason = f"not in <Model>_init.xml: {_join(absent[:5])}{' ...' if len(absent) > 5 else ''}"
        return row
    not_free = [variable for variable in present if variable.kind != ParameterKind.free]
    blocked = "; ".join(
        f"{variable.name.rsplit('.', 1)[-1]}: {_mechanism(variable, mechanisms)}"
        for variable in {variable.name.rsplit(".", 1)[-1]: variable for variable in not_free}.values()
    )
    if len(not_free) == len(present):
        row.status = Status.frozen
        row.reason = blocked
        return row
    row.status = Status.derived if resolution.kind == ResolutionKind.derived else Status.changeable
    reasons = []
    if resolution.kind == ResolutionKind.derived:
        reasons.append(f"derived: set {_join(resolution.modelica_paths)} instead; {resolution.transformation}")
    if not_free:
        reasons.append(
            f"{len(not_free)} of {len(present)} run-time names cannot be overridden ({blocked}); "
            "only the free ones are set, the checks tell whether the others matter"
        )
    row.reason = " | ".join(reasons)
    return row


def free_overrides(resolution: Resolution, variables: Mapping[str, ModelVariable]) -> dict[str, float]:
    """The overrides of a resolved field restricted to the free parameters of the executable."""
    return {
        name: value
        for name, value in resolution.overrides.items()
        if name in variables and variables[name].kind == ParameterKind.free
    }


def _mechanism(variable: ModelVariable, mechanisms: Mapping[str, str] | None) -> str:
    for pattern, mechanism in (mechanisms or {}).items():
        if re.search(pattern, variable.name):
            return f"{frozen_mechanism(variable)}, {mechanism}"
    return frozen_mechanism(variable)


def write_inventory(rows: Iterable[InventoryRow], path: Path) -> Path:
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=InventoryRow.columns())
        writer.writeheader()
        writer.writerows(row.as_csv() for row in rows)
    return path


def read_inventory(path: Path) -> list[InventoryRow]:
    with path.open(newline="") as file:
        return [InventoryRow(**row) for row in csv.DictReader(file)]


__all__ = [
    "GROUPS",
    "InventoryRow",
    "Priority",
    "Status",
    "free_overrides",
    "frozen_mechanism",
    "group_of",
    "perturbation",
    "read_inventory",
    "static_row",
    "write_inventory",
]
