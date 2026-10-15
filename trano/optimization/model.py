"""A Modelica model translated into a CasADi ODE ``xdot = rhs(t, x, u, p)`` with rumoca, and its description.

Everything the optimization needs is read from the Modelica model itself: the names of the states, inputs and
parameters in the order of the CasADi vectors, their units, bounds (``min``/``max``) and descriptions, the start
values of the states and the values of the parameters.
"""

import itertools
import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Self

import casadi as ca
import numpy as np
from pydantic import BaseModel, ConfigDict


class Variable(BaseModel):
    """A state, input or parameter of the model."""

    model_config = ConfigDict(frozen=True)

    name: str
    unit: str = ""
    quantity: str = ""
    description: str = ""
    minimum: float | None = None
    maximum: float | None = None
    value: float | None = None  # start value of a state, value of a parameter


class ModelDescription(BaseModel):
    """States, inputs and parameters of a model, in the order of its CasADi vectors."""

    model_config = ConfigDict(frozen=True)

    name: str
    states: tuple[Variable, ...]
    inputs: tuple[Variable, ...]
    parameters: tuple[Variable, ...]

    @property
    def state_names(self) -> list[str]:
        return [variable.name for variable in self.states]

    @property
    def input_names(self) -> list[str]:
        return [variable.name for variable in self.inputs]

    @property
    def parameter_names(self) -> list[str]:
        return [variable.name for variable in self.parameters]

    @property
    def initial_state(self) -> np.ndarray:
        return np.array([variable.value for variable in self.states], dtype=float)

    @property
    def parameter_values(self) -> np.ndarray:
        return np.array([variable.value for variable in self.parameters], dtype=float)

    def state(self, name: str) -> Variable:
        return self.states[self.state_names.index(name)]

    def input(self, name: str) -> Variable:
        return self.inputs[self.input_names.index(name)]


def _bound(value: Any) -> float | None:  # noqa: ANN401 - rumoca attribute
    return None if value is None or not np.isfinite(float(value)) else float(value)


def _variables(names: Sequence[str], infos: Any, values: Sequence[float | None] | np.ndarray) -> tuple[Variable, ...]:  # noqa: ANN401
    """The variables named in the order of the CasADi vector, described by rumoca ``VariableInfo`` objects."""
    by_name = {info.name: info for info in infos}
    variables = []
    for name, value in zip(names, values, strict=True):
        info = by_name.get(name)
        variables.append(
            Variable(
                name=name,
                unit=(info.unit or "") if info else "",
                quantity=(info.quantity or "") if info else "",
                description=(info.description or "") if info else "",
                minimum=_bound(info.min) if info else None,
                maximum=_bound(info.max) if info else None,
                value=None if value is None else float(value),
            )
        )
    return tuple(variables)


def _names(names: Sequence[Any]) -> list[str]:
    """Plain variable names: rumoca main names its inputs as JSON (``{"dimensions": [], "name": "u"}``), its
    releases as plain strings; an array input is expanded into its elements."""
    result = []
    for raw in names:
        name = str(raw)
        if not name.startswith("{"):
            result.append(name)
            continue
        variable = json.loads(name)
        dimensions = [int(size) for size in variable.get("dimensions") or []]
        indices = itertools.product(*(range(1, size + 1) for size in dimensions))
        result += [variable["name"] + (f"[{','.join(map(str, index))}]" if index else "") for index in indices]
    return result


@dataclass(frozen=True)
class CasadiModel:
    """The CasADi right-hand side of a model and its description."""

    rhs: ca.Function
    description: ModelDescription

    @classmethod
    def from_modelica(cls, source: str, model: str) -> Self:
        """Translate ``model`` of the Modelica ``source`` text with rumoca."""
        import rumoca

        translated = rumoca.Session().loads(source, model=model)
        export = translated.to_casadi()
        inputs = _names(export.input_names)
        description = ModelDescription(
            name=model,
            states=_variables(_names(export.state_names), translated.states, np.asarray(export.default_states).ravel()),
            inputs=_variables(inputs, translated.inputs, [None] * len(inputs)),
            parameters=_variables(
                _names(export.parameter_names), translated.parameters, np.asarray(export.default_parameters).ravel()
            ),
        )
        return cls(rhs=export.rhs, description=description)

    @classmethod
    def from_file(cls, path: Path | str, model: str) -> Self:
        return cls.from_modelica(Path(path).read_text(), model)

    def derivatives(
        self, time: float, state: np.ndarray, inputs: np.ndarray, parameters: np.ndarray | None = None
    ) -> np.ndarray:
        """``xdot`` at one point, with the parameter values of the model by default."""
        values = self.description.parameter_values if parameters is None else parameters
        return np.asarray(self.rhs(time, state, inputs, values), dtype=float).ravel()
