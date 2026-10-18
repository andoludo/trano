"""Numeric fields of a YAML building specification, addressed by readable paths.

A path names the keys from the top of the description; list items are selected by their ``id`` (or the
``id`` of their single child, as in ``systems``) or by their index otherwise:

    material[EPS:001].thermal_conductivity
    spaces[LIVING:001].external_boundaries.windows[0].surface
    systems[GENERATOR:001].boiler.parameters.nominal_heating_power
    glass_material[GLASS:001].solar_transmittance

Numeric fields are numbers, lists of numbers and the Modelica array strings of schedules
(``3600*{7, 23}``); identifiers, variants, tilts and the weather path are not.
"""

import copy
import re
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import yaml

from trano.runtime.expressions import NotNumericError, evaluate, modelica_array

PathKey = str | int
FieldValue = float | int | list[float] | str
_SEGMENT = re.compile(r"(?P<key>[^.\[\]]+)(?:\[(?P<selector>[^\]]+)\])?")
_NUMERIC_STRING_KEYS = ("schedule", "occupancy")
DAY = 86400.0  # [s] period of the day schedules  # Modelica arrays of times given as strings


@dataclass(frozen=True)
class YamlField:
    """A numeric field of the specification."""

    path: str
    keys: tuple[PathKey, ...]
    value: FieldValue

    @property
    def name(self) -> str:
        """The last key: the field name (``thermal_conductivity``)."""
        return str(self.keys[-1])


def load_description(path: Path) -> dict[str, Any]:
    description = yaml.safe_load(path.read_text())
    if not isinstance(description, dict):
        raise ValueError(f"{path} does not hold a YAML mapping")
    return description


def _item_id(item: Any) -> str | None:  # noqa: ANN401
    if isinstance(item, dict):
        if "id" in item:
            return str(item["id"])
        if len(item) == 1:
            (child,) = item.values()
            if isinstance(child, dict) and "id" in child:
                return str(child["id"])
    return None


def _is_numeric_string(key: PathKey, value: Any) -> bool:  # noqa: ANN401
    if not (isinstance(value, str) and isinstance(key, str) and key in _NUMERIC_STRING_KEYS):
        return False
    try:
        return isinstance(evaluate(value), list)
    except NotNumericError:
        return False


def _is_number(value: Any) -> bool:  # noqa: ANN401
    return isinstance(value, int | float) and not isinstance(value, bool)


def iter_numeric_fields(node: Any, keys: tuple[PathKey, ...] = (), path: str = "") -> Iterator[YamlField]:  # noqa: ANN401
    """Every numeric field below ``node``, in document order."""
    if isinstance(node, dict):
        for key, value in node.items():
            child_path = f"{path}.{key}" if path else str(key)
            if _is_number(value) or _is_numeric_string(key, value):
                yield YamlField(child_path, (*keys, key), value)
            elif isinstance(value, list) and value and all(_is_number(item) for item in value):
                yield YamlField(child_path, (*keys, key), [float(item) for item in value])
            else:
                yield from iter_numeric_fields(value, (*keys, key), child_path)
    elif isinstance(node, list):
        for index, item in enumerate(node):
            selector = _item_id(item) or str(index)
            yield from iter_numeric_fields(item, (*keys, index), f"{path}[{selector}]")


def numeric_fields(description: dict[str, Any]) -> list[YamlField]:
    return list(iter_numeric_fields(description))


def resolve_keys(description: dict[str, Any], path: str) -> tuple[PathKey, ...]:
    """The keys of a readable path in the description; raises KeyError when it names nothing."""
    keys: list[PathKey] = []
    node: Any = description
    for segment in path.split("."):
        match = _SEGMENT.fullmatch(segment)
        if match is None or not isinstance(node, dict) or match.group("key") not in node:
            raise KeyError(f"{path}: no {segment!r}")
        keys.append(match.group("key"))
        node = node[match.group("key")]
        if (selector := match.group("selector")) is not None:
            if not isinstance(node, list):
                raise KeyError(f"{path}: {match.group('key')} is not a list")
            index = next((i for i, item in enumerate(node) if _item_id(item) == selector), None)
            if index is None and selector.isdigit() and int(selector) < len(node):
                index = int(selector)
            if index is None:
                raise KeyError(f"{path}: no item {selector!r} in {match.group('key')}")
            keys.append(index)
            node = node[index]
    return tuple(keys)


def get_value(description: dict[str, Any], path: str) -> Any:  # noqa: ANN401
    node: Any = description
    for key in resolve_keys(description, path):
        node = node[key]
    return node


def with_values(description: dict[str, Any], changes: dict[str, FieldValue]) -> dict[str, Any]:
    """A copy of the description with the fields at the given paths set to new values."""
    changed = copy.deepcopy(description)
    for path, value in changes.items():
        *parents, last = resolve_keys(changed, path)
        node: Any = changed
        for key in parents:
            node = node[key]
        node[last] = value
    return changed


def perturbed(value: FieldValue, relative: float, shift: float = 1800.0) -> FieldValue:
    """The value scaled by ``1 + relative``; schedules (arrays of times) shifted by ``shift`` seconds, but
    for their ends at midnight."""
    if isinstance(value, str):
        evaluated = evaluate(value)
        if not isinstance(evaluated, list) or not all(isinstance(time, float) for time in evaluated):
            raise ValueError(f"Not a schedule: {value!r}")
        times = cast(list[float], evaluated)
        # the ends of a day schedule (0 and 24 h) stay: a schedule may not leave its period
        return modelica_array([time if time <= 0 or time >= DAY else time + shift for time in times])
    if isinstance(value, list):
        return [item * (1 + relative) for item in value]
    if value == 0:
        return relative  # an absolute change around zero (an azimuth)
    return value * (1 + relative)


def write_description(description: dict[str, Any], path: Path) -> Path:
    path.write_text(yaml.safe_dump(description, sort_keys=False, allow_unicode=True))
    return path


__all__ = [
    "YamlField",
    "get_value",
    "iter_numeric_fields",
    "load_description",
    "numeric_fields",
    "perturbed",
    "resolve_keys",
    "with_values",
    "write_description",
]
