"""The findings of the model checks."""

from dataclasses import dataclass
from typing import Literal

Level = Literal["error", "warning"]


class ModelWarning(UserWarning):
    """A configuration that translates and simulates but is likely not what the user intended."""


@dataclass(frozen=True)
class Issue:
    """A finding on one component of the model, with how to fix it."""

    level: Level
    component: str
    message: str

    def __str__(self) -> str:
        return f"{self.component}: {self.message}"
