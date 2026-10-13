"""Catalogue of the primitive components of ``FastModel.Components``, read from the Modelica source.

The Modelica file ``trano/fast_model/library/Components.mo`` is the single source of the physics. trano
only needs to know, for each component class, its parameters (in declaration order, with their
defaults), its inputs and its states: that is enough to render instances and to describe the
CasADi vectors of an assembled model, whose order is the declaration order of the instances and,
within an instance, the declaration order of its class (checked against rumoca by the tests).

The file is written in a regular layout that this module parses: one declaration per line,
``package``/``model``/``connector`` blocks closed by ``end <name>;``.
"""

import re
from functools import cache
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field

LIBRARY_FILE = Path(__file__).parent.parent / "library" / "Components.mo"
ROOT_PACKAGE = "FastModel.Components"

_BLOCK = re.compile(r"^\s*(?:partial\s+)?(package|model|connector)\s+(\w+)\b")
_END = re.compile(r"^\s*end\s+(\w+)\s*;")
_ATTRIBUTES = r'((?:[^()"]|"[^"]*")*)'  # modifiers in parentheses; quoted strings may hold parentheses ("J/(kg.K)")
_PARAMETER = re.compile(
    r"^\s*parameter\s+(Real|Integer)\s+(\w+)(?:\[(\w+)\])?(?:\("
    + _ATTRIBUTES
    + r'\))?(?:\s*=\s*([^"\s;]+))?\s*(?:"([^"]*)")?\s*;'
)
_INPUT = re.compile(r"^\s*input\s+Real\s+(\w+)(?:\(" + _ATTRIBUTES + r'\))?\s*(?:"([^"]*)")?\s*;')
_STATE = re.compile(r"^\s*Real\s+(\w+)(?:\[(\w+)\])?\(" + _ATTRIBUTES + r'\)\s*(?:"([^"]*)")?\s*;')
_UNIT = re.compile(r'unit\s*=\s*"([^"]*)"')
_START = re.compile(r"start\s*=\s*(\w+)")


class ComponentParameter(BaseModel):
    model_config = ConfigDict(frozen=True)

    name: str
    unit: str = ""
    default: float | None = None
    description: str = ""
    integer: bool = False
    dimension: str | None = Field(None, description="Integer parameter giving the size of an array parameter")


class ComponentInput(BaseModel):
    model_config = ConfigDict(frozen=True)

    name: str
    unit: str = ""
    description: str = ""


class ComponentState(BaseModel):
    model_config = ConfigDict(frozen=True)

    name: str
    unit: str = ""
    start_parameter: str | None = None
    description: str = ""
    dimension: str | None = Field(None, description="Integer parameter giving the size of an array state")


class ComponentClass(BaseModel):
    """What trano needs to know about a component class of the library."""

    model_config = ConfigDict(frozen=True)

    path: str  # relative to ``FastModel.Components``, e.g. ``Thermal.HeatCapacitor``
    parameters: tuple[ComponentParameter, ...] = ()
    inputs: tuple[ComponentInput, ...] = ()
    states: tuple[ComponentState, ...] = ()

    @property
    def qualified_name(self) -> str:
        return f"{ROOT_PACKAGE}.{self.path}"

    def parameter(self, name: str) -> ComponentParameter:
        for parameter in self.parameters:
            if parameter.name == name:
                return parameter
        raise KeyError(f"{self.path} has no parameter {name}.")


def _unit(attributes: str | None) -> str:
    match = _UNIT.search(attributes or "")
    return match.group(1) if match else ""


def parse_library(source: str) -> dict[str, ComponentClass]:
    """The ``model`` classes of the library source, keyed by their path relative to the root package."""
    classes: dict[str, ComponentClass] = {}
    stack: list[tuple[str, str]] = []  # (kind, name)
    parameters: list[ComponentParameter] = []
    inputs: list[ComponentInput] = []
    states: list[ComponentState] = []
    for line in source.splitlines():
        if block := _BLOCK.match(line):
            kind, name = block.groups()
            stack.append((kind, name))
            if kind == "model":
                parameters, inputs, states = [], [], []
            continue
        if (end := _END.match(line)) and stack and stack[-1][1] == end.group(1):
            kind, name = stack.pop()
            if kind == "model":
                path = ".".join([*(item[1] for item in stack[1:]), name])
                classes[path] = ComponentClass(
                    path=path, parameters=tuple(parameters), inputs=tuple(inputs), states=tuple(states)
                )
            continue
        if not stack or stack[-1][0] != "model":
            continue
        if parameter := _PARAMETER.match(line):
            kind, name, dimension, attributes, default, description = parameter.groups()
            parameters.append(
                ComponentParameter(
                    name=name,
                    unit=_unit(attributes),
                    default=float(default) if default is not None else None,
                    description=description or "",
                    integer=kind == "Integer",
                    dimension=dimension,
                )
            )
        elif input_ := _INPUT.match(line):
            name, attributes, description = input_.groups()
            inputs.append(ComponentInput(name=name, unit=_unit(attributes), description=description or ""))
        elif (state := _STATE.match(line)) and re.search(r"fixed\s*=\s*true", state.group(3)):
            name, dimension, attributes, description = state.groups()
            start = _START.search(attributes)
            states.append(
                ComponentState(
                    name=name,
                    unit=_unit(attributes),
                    start_parameter=start.group(1) if start else None,
                    description=description or "",
                    dimension=dimension,
                )
            )
    return classes


def library_source() -> str:
    """The Modelica source of ``FastModel.Components``."""
    return LIBRARY_FILE.read_text()


@cache
def catalogue() -> dict[str, ComponentClass]:
    return parse_library(library_source())


def component_class(path: str) -> ComponentClass:
    try:
        return catalogue()[path]
    except KeyError as error:
        raise KeyError(f"Unknown FastModel component {path}. Available: {sorted(catalogue())}") from error


def package_source() -> str:
    """The ``FastModel`` Modelica package: the input connector of the generated models and the components."""
    return (
        'package FastModel "Connectable components for model predictive control: translate into an explicit '
        'CasADi ODE"\n'
        '  connector RealInput = input Real "Real input signal (library independent, for CasADi export)";\n\n'
        f"{library_source()}\n"
        "end FastModel;"
    )
