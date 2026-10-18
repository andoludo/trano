"""The variables of an OpenModelica executable, read from its ``<Model>_init.xml``.

OpenModelica writes every scalar of the translated model to ``<Model>_init.xml`` (an extension of the
FMI 1.0 model description). Four attributes tell how a parameter behaves at run time; what they mean in
practice was checked on the executables (see RUNTIME_PARAMETERS.md):

- ``causality="parameter"`` with ``isValueChangeable="true"``: a free parameter, its start value is read
  from the file (or from ``-override``) at initialisation. Free does not mean effective: a free parameter
  the translation used to size arrays (``nSeg``) or to evaluate other parameters keeps a stale effect.
- ``causality="calculatedParameter"`` with a start value: OpenModelica evaluated it at translation (an
  ``Evaluate=true`` or ``final`` parameter, or one structural parameters depend on). Overrides are refused.
- ``causality="calculatedParameter"`` without a start value: computed from its binding at initialisation;
  it follows the free parameters it depends on, overrides are refused.
- ``alias`` other than ``noAlias``: the variable is another one (``aliasVariable``) under another name.
"""

import enum
import xml.etree.ElementTree as ET
from collections.abc import Collection, Iterator
from dataclasses import dataclass
from pathlib import Path

VALUE_TYPES = ("Real", "Integer", "Boolean", "String")


class ParameterKind(enum.StrEnum):
    """How a variable of the init file takes a value at run time."""

    free = "free"  # changeable parameter: read from the init file or an override
    evaluated = "evaluated"  # calculated parameter with a literal value: fixed at translation
    bound = "bound"  # calculated parameter computed from its binding at initialisation
    alias = "alias"  # another variable under another name
    variable = "variable"  # not a parameter (state, algebraic, discrete or constant)


@dataclass(frozen=True)
class ModelVariable:
    """One ``ScalarVariable`` of the init file."""

    name: str
    causality: str
    variability: str
    is_value_changeable: bool
    value_type: str
    start: str | None = None
    unit: str | None = None
    alias: str = "noAlias"
    alias_variable: str | None = None

    @property
    def kind(self) -> ParameterKind:
        if self.alias != "noAlias":
            return ParameterKind.alias
        if self.variability != "parameter":
            return ParameterKind.variable
        if self.causality == "parameter" and self.is_value_changeable:
            return ParameterKind.free
        if self.causality == "calculatedParameter":
            return ParameterKind.bound if self.start is None else ParameterKind.evaluated
        return ParameterKind.evaluated

    @property
    def start_value(self) -> float | None:
        """The start value as a number (booleans as 0/1); None without a numeric start value."""
        if self.start is None or self.value_type == "String":
            return None
        if self.value_type == "Boolean":
            return float(self.start.lower() == "true")
        return float(self.start)


def _variable(element: ET.Element) -> ModelVariable:
    value = next((child for child in element if child.tag in VALUE_TYPES), None)
    return ModelVariable(
        name=element.attrib["name"],
        causality=element.attrib.get("causality", "local"),
        variability=element.attrib.get("variability", "continuous"),
        is_value_changeable=element.attrib.get("isValueChangeable", "false") == "true",
        value_type=value.tag if value is not None else "Real",
        start=value.attrib.get("start") if value is not None else None,
        unit=value.attrib.get("unit") if value is not None else None,
        alias=element.attrib.get("alias", "noAlias"),
        alias_variable=element.attrib.get("aliasVariable"),
    )


def iter_model_variables(path: Path) -> Iterator[ModelVariable]:
    """The variables of the init file in their order; streamed, the file of a large model is tens of MB."""
    for _, element in ET.iterparse(path, events=("end",)):  # noqa: S314 - written by OpenModelica
        if element.tag == "ScalarVariable":
            yield _variable(element)
            element.clear()


def read_model_variables(path: Path, names: Collection[str] | None = None) -> dict[str, ModelVariable]:
    """The variables of the init file by name; only those in ``names`` when given."""
    wanted = set(names) if names is not None else None
    return {
        variable.name: variable for variable in iter_model_variables(path) if wanted is None or variable.name in wanted
    }


def read_default_experiment(path: Path) -> dict[str, str]:
    """The ``DefaultExperiment`` attributes (startTime, stopTime, stepSize, tolerance, solver, ...)."""
    for event, element in ET.iterparse(path, events=("start",)):  # noqa: S314 - written by OpenModelica
        if element.tag == "DefaultExperiment":
            return dict(element.attrib)
        if event == "start" and element.tag == "ModelVariables":
            break
    return {}


__all__ = [
    "ModelVariable",
    "ParameterKind",
    "iter_model_variables",
    "read_default_experiment",
    "read_model_variables",
]
