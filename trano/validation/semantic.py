"""Checks of a model that need more than one object at a time: references, ids, topology and parameters.

They run on the model as written by the user, after the schema validation and before any element is
built, so that every message names the ids of the YAML file and says how to fix the model. Errors are
collected and raised together; warnings are emitted with `ModelWarning`.
"""

import warnings
from collections.abc import Iterable, Iterator
from pathlib import Path
from typing import Any

from trano.exceptions import InvalidModelError
from trano.validation.graph import build_graph
from trano.validation.hydronics import check_hydronics, derive_defaults
from trano.validation.issues import Issue, ModelWarning
from trano.validation.parameters import check_parameters
from trano.validation.references import check_references
from trano.validation.topology import check_controls, check_edges, check_spaces, check_topology


def check_weather(data: dict[str, Any]) -> Iterator[Issue]:
    """A weather file that cannot be found: the model translates but its simulation will fail."""
    path = ((data.get("weather") or {}).get("parameters") or {}).get("path")
    if not isinstance(path, str) or "modelica://" in path or "(" in path:
        return  # a Modelica URI or expression, resolved by the simulation tool
    file = Path(path.strip('"'))
    if not file.exists() and next(Path.cwd().rglob(file.name), None) is None:
        yield Issue(
            "warning",
            "weather",
            f"the weather file '{path}' is not found from {Path.cwd()}: the model "
            "translates, but simulating it will fail. Give the path of a .mos file.",
        )


def check(data: dict[str, Any], is_rc: bool = False) -> list[Issue]:
    """Every issue of the model; `is_rc` for the flat RC (mpc) models, which abstract the hydraulics."""
    graph, issues = build_graph(data)
    checks: list[Iterable[Issue]] = [
        check_references(data, graph),
        check_edges(graph),
        check_spaces(data),
        check_controls(graph, is_rc),
        check_parameters(data),
        check_weather(data),
    ]
    if not is_rc:
        checks += [check_topology(graph), check_hydronics(graph)]
    for found in checks:
        issues += found
    return list(dict.fromkeys(issues))


def validate_semantics(data: dict[str, Any], is_rc: bool = False) -> None:
    """Raise every error of the model at once and emit its warnings.

    Raises:
        InvalidModelError: when the model has at least one error.
    """
    issues = check(data, is_rc=is_rc)
    for issue in issues:
        if issue.level == "warning":
            warnings.warn(str(issue), ModelWarning, stacklevel=2)
    errors = [issue for issue in issues if issue.level == "error"]
    if errors:
        raise InvalidModelError("Invalid model:\n" + "\n".join(f"- {error}" for error in errors))


def apply_derived_defaults(data: dict[str, Any], is_rc: bool = False) -> None:
    """Complete a valid model with the defaults that depend on its topology, and report each one."""
    if is_rc:
        return
    graph, _ = build_graph(data)
    for issue in derive_defaults(data, graph):
        warnings.warn(str(issue), ModelWarning, stacklevel=2)
