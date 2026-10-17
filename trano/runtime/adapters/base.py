"""What a library adapter tells the run-time workflow about the models trano writes for a library.

The run-time names of the parameters a YAML specification sets depend on the library templates. An
adapter lists them as a *snapshot*: every scalar parameter of the compiled model whose value trano
writes from the specification, with that value. The snapshot of two specifications differing by one
field gives the run-time parameters the field reaches and their new values (:mod:`trano.runtime.resolution`),
whatever Python transformation sits in between, because the values come from the same objects the
templates render.

The part shared by every library is here: the parameters of the elements, which are rendered from
``processed_parameters`` (the source of truth of ``data_models/parameters.yaml``). The adapter of a
library adds what its templates write outside of the element parameters (records, arrays of the zones).
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import cache
from pathlib import Path
from typing import ClassVar

import yaml

from trano.elements.base import BaseElement
from trano.runtime.expressions import scalarize
from trano.topology import Network


@dataclass(frozen=True)
class SnapshotEntry:
    """A scalar run-time parameter written from the specification."""

    value: float
    modelica_path: str  # the parameter as the generated model writes it (an array before scalarisation)
    formula: str | None = None  # how trano computes it from other fields, when it does


@cache
def parameter_formulas() -> dict[tuple[str, str], str]:
    """The Python expressions of the computed parameters of parameters.yaml, by (element class, Modelica name).

    These are the YAML fields transformed in Python before they reach Modelica (``func`` and
    ``default_from`` of the schema).
    """
    path = Path(__file__).parents[2].joinpath("data_models", "parameters.yaml")
    formulas: dict[tuple[str, str], str] = {}
    for definition in yaml.safe_load(path.read_text()).values():
        for name, attribute in definition.get("attributes", {}).items():
            expression = attribute.get("func") or attribute.get("default_from")
            if not expression:
                continue
            alias = attribute.get("alias")
            modelica_name = name if alias in (None, "None", "null") else str(alias)
            for element_class in definition.get("classes") or []:
                formulas[(element_class, modelica_name)] = " ".join(str(expression).split())
    return formulas


Snapshot = dict[str, SnapshotEntry]


class LibraryAdapter(ABC):
    """Run-time names of a library's models."""

    library: ClassVar[str]
    top_component: ClassVar[str] = "building"  # the container component of the generated `<name>.building`
    not_used_fields: ClassVar[dict[str, str]] = {}
    """Regular expressions of YAML paths the library does not take, with the reason."""
    frozen_mechanisms: ClassVar[dict[str, str]] = {}
    """Regular expressions of run-time names the translation evaluates, with the reason found in the sources."""

    def model_name(self, network: Network) -> str:
        """The class compiled: the top model of the generated package."""
        return f"{network.name}.building"

    def element_prefix(self, element: BaseElement) -> str:
        return f"{self.top_component}.{element.container_type}1.{element.name}"

    def element_parameters(self, network: Network) -> Snapshot:
        """The numeric parameters of every element, as its template renders them."""
        snapshot: Snapshot = {}
        for element in network.graph.nodes:
            if not isinstance(element, BaseElement) or element.container_type is None:
                continue
            prefix = self.element_prefix(element)
            for name, value in element.processed_parameters(network.library).items():
                scalars = scalarize(f"{prefix}.{name}", value) if value is not None else None
                formula = parameter_formulas().get((type(element).__name__, name))
                for runtime_name, scalar in (scalars or {}).items():
                    snapshot[runtime_name] = SnapshotEntry(scalar, f"{prefix}.{name}", formula)
        return snapshot

    @abstractmethod
    def parameter_snapshot(self, network: Network) -> Snapshot:
        """Every scalar run-time parameter written from the specification; ``network.model()`` was called."""

    def admissible_range(self, network: Network, path: str) -> tuple[float, float] | None:
        """The range of a YAML field in which the compiled structure stays valid; None when unbounded.

        A field the translation uses for sizes (a discretisation) can change at run time only as long as the
        sizes it gives do not change.
        """
        return None

    @abstractmethod
    def impact_outputs(self, network: Network) -> dict[str, str]:
        """The outputs the impact of a parameter is measured on, by label: run-time variable names."""


__all__ = ["LibraryAdapter", "Snapshot", "SnapshotEntry"]
