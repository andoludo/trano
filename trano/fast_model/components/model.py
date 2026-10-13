"""A building model assembled from ``FastModel.Components``: instances, connections and top-level inputs.

:class:`ConnectedModel` renders the ``building_mpc`` model of the ``FastModel`` library and
describes its CasADi vectors without any compiler: the states and parameters of the translated
model are those of the instances, in declaration order, and within an instance those of its class
(:mod:`trano.fast_model.components.catalogue`). The inputs are the top-level inputs, in declaration order.
"""

from typing import Self

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from trano.fast_model.components.catalogue import ComponentClass, ComponentState, component_class
from trano.mpc.parameters import validate_identifier


class TopInput(BaseModel):
    """A top-level input of the model (weather, gains, controls)."""

    model_config = ConfigDict(frozen=True)

    name: str
    unit: str = ""
    description: str = ""
    control: bool = Field(False, description="Set by the controller (MPC decision), not a disturbance")
    zone: str | None = Field(None, description="Zone the input heats, if any (interface metadata)")
    system: str | None = Field(None, description="System the input acts on, if any (interface metadata)")
    minimum: float | None = Field(None, description="Lower bound of a control")
    maximum: float | None = Field(None, description="Upper bound of a control")

    _name_validator = field_validator("name")(validate_identifier)


class Instance(BaseModel):
    """An instance of a library component with literal parameter values and input bindings."""

    model_config = ConfigDict(frozen=True)

    name: str
    component: str = Field(description="Class path relative to FastModel.Components, e.g. Thermal.HeatCapacitor")
    parameters: dict[str, int | float | list[float]] = Field(
        default_factory=dict, description="Literal values; a list for an array parameter"
    )
    inputs: dict[str, str] = Field(default_factory=dict, description="Input bindings: input name -> expression")
    description: str = ""
    zone: str | None = Field(None, description="Zone the instance belongs to (interface metadata, not rendered)")
    system: str | None = Field(None, description="System the instance belongs to (interface metadata, not rendered)")

    _name_validator = field_validator("name")(validate_identifier)

    @property
    def component_class(self) -> ComponentClass:
        return component_class(self.component)

    @model_validator(mode="after")
    def _check_against_class(self) -> Self:
        component = self.component_class
        for parameter in component.parameters:
            value = self.parameters.get(parameter.name)
            if parameter.integer and value is not None and not isinstance(value, int):
                raise ValueError(f"{self.name}: parameter {parameter.name} of {self.component} is an Integer.")
            if parameter.dimension and value is not None:
                size = self.parameters.get(parameter.dimension)
                if not isinstance(value, list) or len(value) != size:
                    raise ValueError(f"{self.name}: {parameter.name} must be a list of {parameter.dimension} values.")
        known_parameters = {parameter.name for parameter in component.parameters}
        if unknown := set(self.parameters) - known_parameters:
            raise ValueError(f"{self.name}: {self.component} has no parameters {sorted(unknown)}.")
        missing = [p.name for p in component.parameters if p.default is None and p.name not in self.parameters]
        if missing:
            raise ValueError(f"{self.name}: parameters {missing} of {self.component} need a value.")
        known_inputs = {input_.name for input_ in component.inputs}
        if unknown := set(self.inputs) - known_inputs:
            raise ValueError(f"{self.name}: {self.component} has no inputs {sorted(unknown)}.")
        if unbound := known_inputs - set(self.inputs):
            raise ValueError(f"{self.name}: inputs {sorted(unbound)} of {self.component} must be bound.")
        return self

    def parameter_values(self) -> list[tuple[str, float]]:
        """(name, value) of every parameter of the class, in declaration order; arrays element by element."""
        values: list[tuple[str, float]] = []
        for parameter in self.component_class.parameters:
            value = self.parameters.get(parameter.name, parameter.default)
            assert value is not None  # checked by the validator
            if isinstance(value, list):
                values += [(f"{self.name}.{parameter.name}[{i}]", float(v)) for i, v in enumerate(value, start=1)]
            else:
                values.append((f"{self.name}.{parameter.name}", float(value)))
        return values

    def size(self, dimension: str) -> int:
        """Value of the Integer parameter sizing an array."""
        value = self.parameters.get(dimension, self.component_class.parameter(dimension).default)
        return int(value)  # type: ignore[arg-type]

    def states(self) -> list[tuple[str, ComponentState]]:
        """(name, class state) of each CasADi state of the instance, arrays element by element."""
        states: list[tuple[str, ComponentState]] = []
        for state in self.component_class.states:
            if state.dimension:
                states += [(f"{self.name}.{state.name}[{i}]", state) for i in range(1, self.size(state.dimension) + 1)]
            else:
                states.append((f"{self.name}.{state.name}", state))
        return states

    def state_names(self) -> list[str]:
        return [name for name, _ in self.states()]

    def render(self) -> str:
        modifiers = [f"{name}={_literal(value)}" for name, value in self.parameters.items()]
        modifiers += [f"{name}={expression}" for name, expression in self.inputs.items()]
        arguments = f"({', '.join(modifiers)})" if modifiers else ""
        description = f' "{self.description}"' if self.description else ""
        return f"  {self.component_class.qualified_name} {self.name}{arguments}{description};"


def _literal(value: float | list[float]) -> str:
    """Modelica literal of a parameter value (``repr`` keeps every digit); an array as ``{a, b}``."""
    if isinstance(value, list):
        return "{" + ", ".join(repr(float(item)) for item in value) + "}"
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    return repr(float(value))


class ConnectedModel(BaseModel):
    """A model made of component instances joined by ``connect()`` statements."""

    model_config = ConfigDict(frozen=True)

    name: str
    description: str = ""
    inputs: list[TopInput] = Field(default_factory=list)
    instances: list[Instance] = Field(default_factory=list)
    connections: list[tuple[str, str]] = Field(default_factory=list)
    input_connector: str = Field("FastModel.RealInput", description="Declaration type of the top-level inputs")

    _name_validator = field_validator("name")(validate_identifier)

    @model_validator(mode="after")
    def _check_names(self) -> Self:
        names = [instance.name for instance in self.instances] + [input_.name for input_ in self.inputs]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            raise ValueError(f"Duplicate names in {self.name}: {duplicates}.")
        instances = {instance.name for instance in self.instances}
        for a, b in self.connections:
            for port in (a, b):
                if port.split(".", 1)[0] not in instances:
                    raise ValueError(f"Connection {a} - {b} refers to an unknown instance ({port}).")
        return self

    @property
    def state_names(self) -> list[str]:
        """Names of the CasADi states, in order."""
        return [name for instance in self.instances for name in instance.state_names()]

    @property
    def input_names(self) -> list[str]:
        return [input_.name for input_ in self.inputs]

    @property
    def parameters(self) -> list[tuple[str, float]]:
        """(name, value) of the CasADi parameters, in order."""
        return [item for instance in self.instances for item in instance.parameter_values()]

    @property
    def parameter_names(self) -> list[str]:
        return [name for name, _ in self.parameters]

    def initial_state(self) -> list[float]:
        """Start values of the states (the ``start`` parameter of each state)."""
        values = []
        for instance in self.instances:
            parameters = dict(instance.parameter_values())
            for name, state in instance.states():
                if state.start_parameter is None:
                    raise ValueError(f"State {name} has no start parameter.")
                values.append(parameters[f"{instance.name}.{state.start_parameter}"])
        return values

    def render(self) -> str:
        lines = [f'model {self.name} "{self.description}"' if self.description else f"model {self.name}"]
        for input_ in self.inputs:
            unit = f'(unit="{input_.unit}")' if input_.unit else ""
            description = f' "{input_.description}"' if input_.description else ""
            lines.append(f"  {self.input_connector} {input_.name}{unit}{description};")
        lines += [instance.render() for instance in self.instances]
        lines.append("equation")
        lines += [f"  connect({a}, {b});" for a, b in self.connections]
        lines.append(f"end {self.name};")
        return "\n".join(lines) + "\n"
