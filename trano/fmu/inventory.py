"""Which parameters of a YAML building description can be set in the exported FMU.

The chain resolved here is **YAML field → Modelica parameter path(s) → FMU variable(s)**:

- the Modelica names of the parameters of the systems, the controls, the occupancies and the
  zones come from the parameter schema (``data_models/parameters.yaml``, through
  :mod:`trano.elements.library.parameters`), the same source as the documentation;
- the envelope (materials, layers, glazings, walls, windows, floors) is rendered into records and
  arrays whose layout is the library's: a :class:`EnvelopeResolver` per library (Buildings:
  :mod:`trano.fmu.buildings`) resolves it.

A field whose value reaches Modelica unchanged is ``direct``. A field transformed in Python
first (an airtightness turned into an infiltration rate, a window area turned into a width and a
height, a schedule expression expanded into an array, a nominal power that also sizes a water
volume) is ``derived``: the transformation is recorded, with every FMU parameter a calibration
would have to set instead. A field the library does not take is ``not_used``.

:func:`classify` then looks the resolved paths up in the ``modelDescription.xml`` of the FMU: a
parameter is only usable when every path is present with ``causality = parameter`` and a
``fixed`` or ``tunable`` variability. OpenModelica exports almost every parameter as ``fixed``
(settable before initialisation only), which is enough for a calibration that re-instantiates
the FMU at every run.
"""

import csv
import re
from collections.abc import Iterable, Iterator, Sequence
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

from pydantic import BaseModel, ConfigDict, Field

from trano.elements.base import BaseElement
from trano.elements.common_base import BaseParameter
from trano.elements.library.parameters import ParameterSpec, library_parameters, parameter_specs
from trano.elements.space import BaseSpace
from trano.fmu.model_description import FmuVariable, ModelDescription
from trano.topology import Network

TOP_INSTANCE = "building"
"""The generated ``building`` model instantiates ``Components.Containers.building`` under this name."""
PERTURBATION = 1.2
"""Factor applied to a value to find the parameters computed from it."""
_ARRAY = re.compile(
    r"^\s*(?:(?P<factor>[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?(?:\s*[*/]\s*\d+(?:\.\d+)?)*)\s*\*\s*)?\{(?P<items>[^}]*)\}\s*$"
)
_NUMBER = re.compile(r"^[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?$")
Number = float | int

Kind = str  # direct, derived, not_used, unknown


class Status(str, Enum):
    calibratable = "calibratable"
    derived = "derived"
    frozen = "frozen"
    no_impact = "no_impact"
    not_used = "not_used"
    missing = "missing"


class Target(BaseModel):
    """One Modelica parameter a YAML field ends up in."""

    model_config = ConfigDict(frozen=True)
    path: str = Field(description="Parameter path from the top model, the FMU variable name when exported")
    value: Number | None = Field(default=None, description="Value the YAML gives it; None when not a number")
    transformation: str | None = Field(
        default=None, description="How the YAML value is turned into this one; None when it is written as is"
    )

    @property
    def derived(self) -> bool:
        return self.transformation is not None


class Location(BaseModel):
    """Where a field sits in the YAML and the inventory group it belongs to."""

    model_config = ConfigDict(frozen=True)
    yaml_path: str
    group: str

    def child(self, suffix: str, group: str | None = None) -> "Location":
        return Location(yaml_path=f"{self.yaml_path}.{suffix}", group=group or self.group)


class ResolutionContext(BaseModel):
    """The library whose names are resolved and the fields the YAML gives for the component."""

    model_config = ConfigDict(frozen=True)
    library: str
    given: frozenset[str] = frozenset()


class ResolvedParameter(BaseModel):
    """A numeric field of the YAML with the Modelica parameters it sets."""

    model_config = ConfigDict(frozen=True)
    yaml_path: str
    group: str
    value: Any = None
    unit: str | None = None
    element: str | None = Field(default=None, description="Name of the Modelica component holding the parameter")
    targets: tuple[Target, ...] = ()
    note: str | None = None

    @property
    def kind(self) -> Kind:
        if not self.targets:
            return "not_used"
        if any(target.derived for target in self.targets):
            return "derived"
        return "direct"

    @property
    def paths(self) -> list[str]:
        return [target.path for target in self.targets]


class InventoryRow(BaseModel):
    """One line of ``parameter_inventory_<library>.csv``."""

    yaml_path: str
    modelica_paths: str
    fmu_names: str
    unit: str
    default: str
    causality: str
    variability: str
    readback_ok: str = ""
    impact_metric: str = ""
    equivalence_ok: str = ""
    status: str
    reason: str
    group: str = ""

    @classmethod
    def columns(cls) -> list[str]:
        return [name for name in cls.model_fields if name != "group"] + ["group"]


class EnvelopeResolver(Protocol):
    """Resolves the envelope fields of the YAML for one library."""

    def resolve(self, network: Network, data: dict[str, Any]) -> list[ResolvedParameter]: ...


def clean_name(identifier: str) -> str:
    """The Modelica component name of a YAML id (``RADIATOR_LIVING:001`` → ``radiator_living_001``)."""
    return identifier.lower().replace(":", "_") if ":" in identifier else identifier


def parse_modelica_array(text: str) -> list[float] | None:
    """The elements of a Modelica array literal, ``{7, 23}`` or ``3600*{7, 23}``; None for anything else."""
    match = _ARRAY.match(text)
    if match is None:
        return None
    factor = _product(match.group("factor")) if match.group("factor") else 1.0
    items = [item.strip() for item in match.group("items").split(",") if item.strip()]
    if not all(_NUMBER.match(item) for item in items):
        return None
    return [factor * float(item) for item in items]


def _product(expression: str) -> float:
    """The value of a chain of numbers multiplied and divided (``0.05/1000*2``)."""
    tokens = re.findall(r"[*/]|[^*/]+", expression.replace(" ", ""))
    value = float(tokens[0])
    for operator, operand in zip(tokens[1::2], tokens[2::2], strict=True):
        value = value * float(operand) if operator == "*" else value / float(operand)
    return value


def is_numeric(value: Any) -> bool:  # noqa: ANN401
    """Numbers, and the strings holding a Modelica array (schedules, ``3600*{7, 19}``)."""
    if isinstance(value, bool):
        return False
    if isinstance(value, int | float):
        return True
    return isinstance(value, str) and (
        _NUMBER.match(value.strip()) is not None or parse_modelica_array(value) is not None
    )


def find_element(network: Network, name: str) -> BaseElement | None:
    return next((node for node in network.graph.nodes if node.name == name), None)


def component_prefix(element: BaseElement) -> str:
    """``building.<container>1.<name>``: the path of a component in the generated building."""
    return f"{TOP_INSTANCE}.{element.container_type}1.{element.name}"


def _without_computed(parameters: BaseParameter) -> dict[str, Any]:
    computed = set(type(parameters).model_computed_fields)
    return {name: value for name, value in parameters.model_dump().items() if name not in computed}


def _perturbed(parameters: BaseParameter, field: str, value: Any, given: set[str]) -> BaseParameter:  # noqa: ANN401
    """The parameters with ``field`` perturbed, re-validated so that the computed fields follow.

    The fields derived from others (``default_from``) that the YAML did not give are dropped, so that the
    validation derives them again from the perturbed value.
    """
    if isinstance(value, str):
        array = parse_modelica_array(value)
        perturbed_value: Any = (
            re.sub(r"\{[^}]*\}", "{" + ", ".join(f"{item * PERTURBATION:g}" for item in array) + "}", value)
            if array
            else value
        )
    else:
        perturbed_value = value * PERTURBATION if value else 1.0
    derived = {name for name, spec in parameter_specs(parameters).items() if spec.default_from and name not in given}
    data = {
        name: item for name, item in _without_computed(parameters).items() if name not in derived and item is not None
    }
    return type(parameters).model_validate(data | {field: perturbed_value})


def _spec_of(specs: dict[str, ParameterSpec], field: str) -> tuple[str, ParameterSpec] | None:
    if field in specs:
        return field, specs[field]
    return next(((name, spec) for name, spec in specs.items() if spec.short_name == field), None)


def _value_targets(path: str, value: Any, transformation: str | None) -> list[Target]:  # noqa: ANN401
    """Targets for a rendered value: one per array element for a Modelica array, else one."""
    if isinstance(value, str):
        if _NUMBER.match(value.strip()):
            return [Target(path=path, value=float(value), transformation=transformation)]
        array = parse_modelica_array(value)
        if array is None:
            return [Target(path=path, value=None, transformation=transformation)]
        note = f"Modelica array {value!r} evaluated at translation"
        return [
            Target(path=f"{path}[{index}]", value=item, transformation=transformation or note)
            for index, item in enumerate(array, start=1)
        ]
    if isinstance(value, bool):
        return [Target(path=path, value=float(value), transformation=transformation)]
    return [Target(path=path, value=value, transformation=transformation)]


def resolve_element_parameter(
    element: BaseElement,
    field: str,
    value: Any,  # noqa: ANN401
    context: ResolutionContext,
    location: Location,
) -> ResolvedParameter:
    """Resolve one field of the ``parameters`` of a component through the parameter schema.

    The field's own mapping gives the direct target; the fields the schema computes from it
    (``func`` or ``default_from``) are found by perturbing the value and rendering again.
    """
    parameters = element.parameters
    library = context.library
    if parameters is None:
        return ResolvedParameter(
            yaml_path=location.yaml_path, group=location.group, value=value, note="component without parameters"
        )
    specs = parameter_specs(parameters)
    found = _spec_of(specs, field)
    name, spec = found or (field, None)
    rendered = library_parameters(parameters, library, element.variant)
    prefix = component_prefix(element)
    targets: list[Target] = []
    mapping = spec.mapping(library, element.variant, given=True) if spec else None
    direct_name = mapping.name if mapping and mapping.applies(element.variant, True) else (None if spec else field)
    if direct_name is not None and direct_name in rendered:
        targets += _value_targets(f"{prefix}.{direct_name}", rendered[direct_name], None)
    try:
        perturbed = library_parameters(
            _perturbed(parameters, name, value, set(context.given) or {name}), library, element.variant
        )
    except Exception:
        perturbed = rendered
    for rendered_name, rendered_value in perturbed.items():
        if rendered_name == direct_name or rendered.get(rendered_name) == rendered_value:
            continue
        source = next((s for s in specs.values() if s.mapping(library, element.variant).name == rendered_name), None)
        transformation = (source.func or source.default_from or "computed") if source else "computed"
        targets += _value_targets(
            f"{prefix}.{rendered_name}", rendered.get(rendered_name), f"{rendered_name} = {transformation}"
        )
    note = None
    if not targets and spec is not None:
        note = f"the {library} library does not take {name}" + (
            f" ({mapping.name} handled by the template)" if mapping and mapping.handled else ""
        )
    return ResolvedParameter(
        yaml_path=location.yaml_path,
        group=location.group,
        value=value,
        unit=spec.unit if spec else None,
        element=element.name,
        targets=tuple(targets),
        note=note,
    )


def _numeric_items(parameters: dict[str, Any]) -> Iterator[tuple[str, Any]]:
    for field, value in parameters.items():
        if is_numeric(value):
            yield field, value


def resolve_component(
    network: Network, component: dict[str, Any], yaml_path: str, group: str, element: BaseElement | None = None
) -> list[ResolvedParameter]:
    """Resolve the numeric ``parameters`` of a component dict and of its ``control``."""
    if element is None:
        identifier = component.get("id")
        element = find_element(network, clean_name(identifier)) if identifier else None
    resolved: list[ResolvedParameter] = []
    context = ResolutionContext(
        library=network.library.parameter_library, given=frozenset((component.get("parameters") or {}).keys())
    )
    for field, value in _numeric_items(component.get("parameters") or {}):
        location = Location(yaml_path=f"{yaml_path}.parameters.{field}", group=group)
        if element is None:
            resolved.append(
                ResolvedParameter(yaml_path=location.yaml_path, group=group, value=value, note="component not found")
            )
        else:
            resolved.append(resolve_element_parameter(element, field, value, context, location))
    for control_type, control in (component.get("control") or {}).items():
        if not isinstance(control, dict):
            continue
        control_element = find_element(network, clean_name(control["id"])) if control.get("id") else None
        if control_element is None and element is not None:
            control_element = getattr(element, "control", None)
        resolved += resolve_component(
            network, control, f"{yaml_path}.control.{control_type}", "Controls", element=control_element
        )
    return resolved


def _systems(data: dict[str, Any]) -> Iterator[tuple[str, str, dict[str, Any]]]:
    """``(yaml path, component type, component)`` of every system, emission, duct and solar component."""
    for index, space in enumerate(data.get("spaces", [])):
        space_path = f"spaces[{space.get('id', index)}]"
        for section in ("emissions", "ventilation_inlets", "ventilation_outlets"):
            for position, item in enumerate(space.get(section) or []):
                for component_type, component in item.items():
                    yield f"{space_path}.{section}[{position}].{component_type}", component_type, component
    for section in ("systems", "solar"):
        for position, item in enumerate(data.get(section) or []):
            for component_type, component in item.items():
                yield f"{section}[{position}].{component_type}", component_type, component


SYSTEM_GROUPS = {
    "radiator": "Emission",
    "valve": "Emission",
    "boiler": "Heat pump",
    "pump": "Hydronics",
    "three_way_valve": "Hydronics",
    "split_valve": "Hydronics",
    "air_handling_unit": "Ventilation",
    "duct": "Ventilation",
    "photovoltaic": "PV",
}


def resolve_systems(network: Network, data: dict[str, Any]) -> list[ResolvedParameter]:
    resolved = []
    for yaml_path, component_type, component in _systems(data):
        resolved += resolve_component(network, component, yaml_path, SYSTEM_GROUPS.get(component_type, "Systems"))
    return resolved


def resolve_spaces(network: Network, data: dict[str, Any]) -> list[ResolvedParameter]:
    """The ``parameters`` and the ``occupancy`` of each space."""
    resolved = []
    for index, space_data in enumerate(data.get("spaces", [])):
        identifier = space_data.get("id", str(index))
        yaml_path = f"spaces[{identifier}]"
        space = find_element(network, clean_name(identifier))
        if not isinstance(space, BaseSpace):
            continue
        resolved += resolve_component(network, {"parameters": space_data.get("parameters")}, yaml_path, "Space", space)
        occupancy = space_data.get("occupancy")
        if isinstance(occupancy, dict) and space.occupancy is not None:
            resolved += resolve_component(network, occupancy, f"{yaml_path}.occupancy", "Occupancy", space.occupancy)
    return resolved


def resolve_all(network: Network, data: dict[str, Any], envelope: EnvelopeResolver) -> list[ResolvedParameter]:
    """Every numeric field of the YAML: envelope, spaces, systems and controls."""
    return envelope.resolve(network, data) + resolve_spaces(network, data) + resolve_systems(network, data)


def _attribute_summary(variables: Sequence[FmuVariable], attribute: str) -> str:
    values = sorted({str(value) for variable in variables if (value := getattr(variable, attribute)) is not None})
    return "/".join(values)


def frozen_reason(variable: FmuVariable) -> str | None:
    """Why the FMU does not take a value for the variable, from its FMI attributes; None when it does."""
    if variable.settable:
        return None
    if variable.causality == "calculatedParameter":
        return "calculatedParameter: bound to other parameters by an equation in the FMU"
    if variable.variability == "constant":
        return "constant: evaluated at translation (final, Evaluate=true or structural use)"
    if variable.causality == "local":
        return f"local {variable.variability} variable, not exposed as a parameter"
    return f"causality {variable.causality}, variability {variable.variability}"


def classify(resolved: ResolvedParameter, description: ModelDescription) -> InventoryRow:
    """The inventory row of a resolved parameter before the simulation checks."""
    by_name = description.by_name()
    found = [by_name[path] for path in resolved.paths if path in by_name]
    missing = [path for path in resolved.paths if path not in by_name]
    reasons = []
    if resolved.kind == "not_used":
        status = Status.not_used
        reasons.append(resolved.note or "the library does not take this field")
    elif missing:
        status = Status.missing
        reasons.append(f"not in modelDescription.xml: {', '.join(missing)}")
    elif any(frozen_reason(variable) for variable in found):
        status = Status.frozen
        reasons += sorted({f"{v.name}: {frozen_reason(v)}" for v in found if frozen_reason(v)})
    elif resolved.kind == "derived":
        status = Status.derived
        reasons += sorted({t.transformation for t in resolved.targets if t.transformation})
    else:
        status = Status.calibratable  # pending the simulation checks
    return InventoryRow(
        yaml_path=resolved.yaml_path,
        modelica_paths="; ".join(resolved.paths),
        fmu_names="; ".join(variable.name for variable in found),
        unit=resolved.unit or _attribute_summary(found, "unit") or "",
        default=str(resolved.value),
        causality=_attribute_summary(found, "causality"),
        variability=_attribute_summary(found, "variability"),
        status=status.value,
        reason="; ".join(reasons),
        group=resolved.group,
    )


def write_inventory(rows: Iterable[InventoryRow], path: Path) -> Path:
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=InventoryRow.columns())
        writer.writeheader()
        for row in rows:
            writer.writerow(row.model_dump())
    return path


def read_inventory(path: Path) -> list[InventoryRow]:
    with path.open(newline="") as file:
        return [InventoryRow.model_validate(row) for row in csv.DictReader(file)]
