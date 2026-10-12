"""Validation of a model, as written by the user, against the Trano LinkML schema."""

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


def _location(error: ValidationError) -> str:
    return "/" + "/".join(str(part) for part in error.absolute_path)


def validate_schema(data: dict[str, Any], schema_path: Path = DATA_MODEL_PATH) -> None:
    """Validate the model against the schema and raise every violation at once.

    String slots that hold a Modelica expression also take plain numbers and booleans (``dp_nominal: 100``
    for ``"100"``), as the schema's Python classes always did: such values are converted to strings in place.
    Keys left empty (None) must have been removed beforehand: they count as absent.

    Raises:
        InvalidModelError: listing each violation with its location in the model.
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
        messages = "\n".join(f"- {_message(error)} in {_location(error)}" for error in errors)
        raise InvalidModelError(f"Invalid model:\n{messages}")
