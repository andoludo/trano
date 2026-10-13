"""What each variable of a model is for in an optimal control problem.

The roles are given explicitly, or inferred from the metadata of the Modelica model with simple conventions:

* a **control** is an input with both a ``min`` and a ``max`` (a decision of the controller within its range);
  every other input is a **disturbance** (weather, gains), to be forecast;
* a **comfort temperature** is a state whose name matches ``comfort_pattern`` (``TAir`` of a zone by default);
* a **meter** is a state in ``kWh``: the energy a system drew (or produced) since the start. Its name gives its
  carrier: ``EFue`` fuel, ``EPro`` electricity production (PV), any other name electricity.
* an input in ``W`` without a meter (an ideal heater) is heat bought at the electricity price.
"""

import re
from typing import Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .model import ModelDescription

Carrier = Literal["electricity", "fuel", "electricity_production"]
DEFAULT_COMFORT_PATTERN = r"(^|\.)TAir$"
_CARRIERS: dict[str, Carrier] = {"EFue": "fuel", "EPro": "electricity_production"}


class Roles(BaseModel):
    """Names of the controls, disturbances, comfort temperatures and meters of a model."""

    model_config = ConfigDict(frozen=True)

    controls: tuple[str, ...]
    disturbances: tuple[str, ...]
    comfort: tuple[str, ...] = Field(description="States kept above the comfort bound (zone air temperatures)")
    meters: dict[str, Carrier] = Field(default_factory=dict, description="Energy meter states [kWh] and carriers")

    @model_validator(mode="after")
    def _disjoint(self) -> Self:
        if set(self.controls) & set(self.disturbances):
            raise ValueError(f"Inputs both controls and disturbances: {set(self.controls) & set(self.disturbances)}")
        return self

    @classmethod
    def infer(cls, description: ModelDescription, comfort_pattern: str = DEFAULT_COMFORT_PATTERN) -> Self:
        controls = tuple(
            variable.name
            for variable in description.inputs
            if variable.minimum is not None and variable.maximum is not None
        )
        disturbances = tuple(name for name in description.input_names if name not in controls)
        pattern = re.compile(comfort_pattern)
        comfort = tuple(name for name in description.state_names if pattern.search(name))
        meters = {
            variable.name: _CARRIERS.get(variable.name.rsplit(".", 1)[-1], "electricity")
            for variable in description.states
            if variable.unit == "kWh"
        }
        return cls(controls=controls, disturbances=disturbances, comfort=comfort, meters=meters)

    def check(self, description: ModelDescription) -> None:
        """Every name exists in the model and every input has a role."""
        inputs = set(description.input_names)
        missing = (set(self.controls) | set(self.disturbances)) - inputs
        missing |= (set(self.comfort) | set(self.meters)) - set(description.state_names)
        if missing:
            raise ValueError(f"Unknown variables of {description.name}: {sorted(missing)}")
        unassigned = inputs - set(self.controls) - set(self.disturbances)
        if unassigned:
            raise ValueError(f"Inputs of {description.name} without a role: {sorted(unassigned)}")
