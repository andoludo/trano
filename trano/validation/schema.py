"""Validation of a model, as written by the user, against the Trano LinkML schema."""

from collections.abc import Iterable
from functools import cache
from pathlib import Path
from typing import Any

from jsonschema.exceptions import ValidationError  # type: ignore[import-untyped]
from jsonschema.protocols import Validator  # type: ignore[import-untyped]
from linkml.validator.validation_context import ValidationContext  # type: ignore
from linkml_runtime import SchemaView  # type: ignore

from trano.exceptions import InvalidModelError

DATA_MODEL_PATH = Path(__file__).parents[1].joinpath("data_models", "trano_final.yaml")
TARGET_CLASS = "Building"


@cache
def _json_schema_validator(schema_path: Path = DATA_MODEL_PATH) -> Validator:
    """JSON Schema validator generated from the LinkML schema, closed (cached: generating it is expensive)."""
    context = ValidationContext(SchemaView(str(schema_path)).schema, TARGET_CLASS)
    return context.json_schema_validator(closed=True, include_range_class_descendants=False)


def _expects_string(error: ValidationError) -> bool:
    expected = error.validator_value
    return "string" in (expected if isinstance(expected, list) else [expected])


def _is_scalar_for_string(error: ValidationError) -> bool:
    """A number or a boolean given for a string slot, which the schema's Python classes convert."""
    return (
        error.validator == "type"
        and _expects_string(error)
        and isinstance(error.instance, (bool, int, float))
        and len(error.absolute_path) > 0
    )


def _coerce(data: Any, error: ValidationError) -> None:  # noqa: ANN401
    *parents, key = error.absolute_path
    container = data
    for part in parents:
        container = container[part]
    container[key] = str(error.instance)


def _is_null_branch(errors: list[ValidationError]) -> bool:
    return all(error.validator == "type" and error.validator_value == "null" for error in errors)


def _leaf_errors(error: ValidationError) -> list[ValidationError]:
    """The errors that explain `error`, looking through the `anyOf: [<class>, null]` LinkML generates.

    Without this, a single wrong parameter deep in a component is reported as the whole component
    being "not valid under any of the given schemas".
    """
    if error.validator not in {"anyOf", "oneOf"} or not error.context:
        return [error]
    branches: dict[Any, list[ValidationError]] = {}
    for sub_error in error.context:
        branches.setdefault(sub_error.relative_schema_path[0], []).append(sub_error)
    candidates = [branch for branch in branches.values() if not _is_null_branch(branch)]
    if len(candidates) != 1:
        return [error]
    return [leaf for sub_error in candidates[0] for leaf in _leaf_errors(sub_error)]


KELVIN_MINIMUM = 223.15


def _message(error: ValidationError) -> str:
    """The validator's message, made explicit for the two cases its wording leaves obscure."""
    if (
        error.validator == "minimum"
        and error.validator_value == KELVIN_MINIMUM
        and isinstance(error.instance, (int, float))
    ):
        celsius = error.instance
        return (
            f"{celsius} is not a temperature in kelvin; temperatures are in kelvin "
            f"({celsius} degC is {celsius + 273.15:.2f} K)"
        )
    if error.validator == "oneOf" and isinstance(error.instance, dict):
        keys = [key for branch in error.validator_value for key in branch.get("required", [])]
        if keys:
            given = [key for key in keys if key in error.instance]
            return f"exactly one of {', '.join(keys)} is expected, {len(given)} given ({', '.join(given) or 'none'})"
    return str(error.message)


JSON_TYPES = {
    "number": "a number",
    "integer": "an integer",
    "string": "a string",
    "boolean": "a boolean",
    "array": "a list",
    "object": "a mapping",
}
BOUNDS = {"minimum": ">=", "maximum": "<=", "exclusiveMinimum": ">", "exclusiveMaximum": "<"}
MAXIMUM_LISTED_FIELDS = 20


def _location(path: Iterable[Any]) -> str:
    """The YAML path of a value: ``spaces[0].parameters.floor_area``."""
    location = ""
    for part in path:
        location += f"[{part}]" if isinstance(part, int) else f".{part}" if location else str(part)
    return location or "(model)"


def _expected_type(error: ValidationError) -> str:
    expected = error.validator_value if isinstance(error.validator_value, list) else [error.validator_value]
    names = [JSON_TYPES.get(str(name), str(name)) for name in expected if name != "null"]
    return " or ".join(names)


def _readable(error: ValidationError) -> list[str]:
    """One line per violation: the YAML path, what is expected and the description of the field."""
    path = list(error.absolute_path)
    description = error.schema.get("description") if isinstance(error.schema, dict) else None
    properties = error.schema.get("properties", {}) if isinstance(error.schema, dict) else {}
    if error.validator == "required":
        name = str(error.message).split("'")[1] if "'" in str(error.message) else ""
        return [_line([*path, name], "is required", properties.get(name, {}).get("description"))]
    if error.validator == "additionalProperties" and isinstance(error.instance, dict):
        allowed = list(properties)
        listed = ", ".join(allowed[:MAXIMUM_LISTED_FIELDS]) + (", ..." if len(allowed) > MAXIMUM_LISTED_FIELDS else "")
        hint = f"unknown field; the fields of {_location(path)} are {listed}" if allowed else "unknown field"
        return [_line([*path, name], hint, None) for name in error.instance if name not in properties]
    if error.validator in BOUNDS and not (error.validator == "minimum" and error.validator_value == KELVIN_MINIMUM):
        message = f"must be {BOUNDS[str(error.validator)]} {error.validator_value} (got {error.instance!r})"
    elif error.validator == "type":
        message = f"must be {_expected_type(error)} (got {error.instance!r})"
    elif error.validator == "enum":
        message = f"must be one of {', '.join(map(str, error.validator_value))} (got {error.instance!r})"
    else:
        message = _message(error)
    return [_line(path, message, description)]


def _line(path: Iterable[Any], message: str, description: str | None) -> str:
    return f"{_location(path)}: {message}" + (f" ({description})" if description else "")


def validate_schema(data: dict[str, Any], schema_path: Path = DATA_MODEL_PATH) -> None:
    """Validate the model against the schema and raise every violation at once.

    String slots that hold a Modelica expression also take plain numbers and booleans (``dp_nominal: 100``
    for ``"100"``), as the schema's Python classes always did: such values are converted to strings in place.
    Keys left empty (None) must have been removed beforehand: they count as absent.

    Raises:
        InvalidModelError: listing each violation, one line per violation (also in its ``errors``):
            ``spaces[0].parameters.floor_area: must be >= 0 (got -5) (Floor area [m2])``, the YAML path,
            what is expected and the description of the field in the schema.
    """
    validator = _json_schema_validator(schema_path)
    while True:
        errors = sorted(
            (leaf for error in validator.iter_errors(data) for leaf in _leaf_errors(error)),
            key=lambda error: list(map(str, error.absolute_path)),
        )
        coercible = [error for error in errors if _is_scalar_for_string(error)]
        if not coercible:
            break
        for error in coercible:
            _coerce(data, error)
    if errors:
        lines = [line for error in errors for line in _readable(error)]
        messages = "\n".join(f"- {line}" for line in lines)
        raise InvalidModelError(f"Invalid model:\n{messages}", errors=lines)
