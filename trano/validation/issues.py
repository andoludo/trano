"""The findings of the model checks."""

from dataclasses import dataclass, field
from typing import Literal

Level = Literal["error", "warning"]


class ModelWarning(UserWarning):
    """A configuration that translates and simulates but is likely not what the user intended."""


class HydronicLayoutWarning(ModelWarning):
    """A water circuit wired in a way that cannot do what its controls ask (e.g. a diverting three-way valve).

    Turn it into an error with ``warnings.simplefilter("error", HydronicLayoutWarning)``.
    """


@dataclass(frozen=True)
class Issue:
    """A finding on one component of the model, with how to fix it."""

    level: Level
    component: str
    message: str
    category: type[ModelWarning] = field(default=ModelWarning, compare=False)

    def __str__(self) -> str:
        return f"{self.component}: {self.message}"
