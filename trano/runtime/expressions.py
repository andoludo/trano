"""Numeric values of the Modelica expressions trano writes as parameter values.

Parameters reach the model as numbers or as Modelica text: arrays (``3600*{7, 23}``, ``{2000,0}``,
``0.05/1000*{0.5,1}``), column matrices (``[35; 70; 30]``) or scalar arithmetic (``1/6/4``). The run-time
names of an array are its scalarised elements (``schedule[1]``, ``gain[2,1]``), which
:func:`scalarize` gives with their values. Anything else (a boolean, an enumeration, a record) is not a
numeric parameter and gives None.
"""

import ast
import operator
import re
from collections.abc import Callable
from typing import Any, cast

Number = float
Values = list[float] | list[list[float]] | float

_OPERATORS: dict[type[ast.operator], Callable[[float, float], float]] = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.Pow: operator.pow,
}
_IDENTIFIER = re.compile(r"(?<![\d.])[A-Za-z_]")  # a name, not the exponent of a number


class NotNumericError(ValueError):
    """The expression is not numeric Modelica arithmetic."""


def _to_python(expression: str) -> str:
    """Modelica arrays and matrices as Python lists: ``{a, b}`` -> ``[a, b]``, ``[a; b]`` -> ``[[a], [b]]``."""
    expression = expression.strip()
    if expression.startswith("[") and ";" in expression:
        rows = expression.strip("[]").split(";")
        return "[" + ", ".join(f"[{row}]" for row in rows) + "]"
    return expression.replace("{", "[").replace("}", "]")


def _evaluate(node: ast.AST) -> Values:  # noqa: PLR0911
    if isinstance(node, ast.Expression):
        return _evaluate(node.body)
    if isinstance(node, ast.Constant) and isinstance(node.value, int | float) and not isinstance(node.value, bool):
        return float(node.value)
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub | ast.UAdd):
        value = _evaluate(node.operand)
        return _elementwise(value, -1.0 if isinstance(node.op, ast.USub) else 1.0, operator.mul)
    if isinstance(node, ast.List):
        items = [_evaluate(item) for item in node.elts]
        if all(isinstance(item, float) for item in items):
            return [cast(float, item) for item in items]
        if all(isinstance(item, list) and all(isinstance(value, float) for value in item) for item in items):
            return [[float(value) for value in cast(list[float], item)] for item in items]
        raise NotNumericError("Nested arrays of unequal depth")
    if isinstance(node, ast.BinOp) and type(node.op) in _OPERATORS:
        left, right = _evaluate(node.left), _evaluate(node.right)
        operation = _OPERATORS[type(node.op)]
        if isinstance(left, float):
            return _elementwise(right, left, lambda value, scalar: operation(scalar, value))
        if isinstance(right, float):
            return _elementwise(left, right, operation)
        raise NotNumericError("Array by array arithmetic is not supported")
    raise NotNumericError(f"Not numeric: {ast.dump(node)}")


def _elementwise(values: Values, scalar: float, operation: Callable[[float, float], float]) -> Values:
    if isinstance(values, float):
        return operation(values, scalar)
    return cast(Values, [_elementwise(cast(Values, value), scalar, operation) for value in values])


def evaluate(expression: str | float | int) -> Values:
    """The numeric value of a Modelica expression; raises :class:`NotNumericError` otherwise."""
    if isinstance(expression, bool):
        raise NotNumericError("A boolean is not numeric")
    if isinstance(expression, int | float):
        return float(expression)
    text = expression.strip()
    if not text or _IDENTIFIER.search(text):
        raise NotNumericError(f"Not numeric: {expression!r}")
    try:
        tree = ast.parse(_to_python(text), mode="eval")
    except SyntaxError as error:
        raise NotNumericError(f"Not numeric: {expression!r}") from error
    return _evaluate(tree)


def scalarize(name: str, expression: str | float | int) -> dict[str, float] | None:
    """The scalar run-time names of a parameter and their values; None when the value is not numeric."""
    try:
        value = evaluate(expression)
    except (NotNumericError, ZeroDivisionError):
        return None
    if isinstance(value, float):
        return {name: value}
    scalars: dict[str, float] = {}
    items: list[Any] = list(value)
    for i, item in enumerate(items, start=1):
        if isinstance(item, list):
            scalars.update({f"{name}[{i},{j}]": element for j, element in enumerate(item, start=1)})
        else:
            scalars[f"{name}[{i}]"] = item
    return scalars


def modelica_array(values: list[float]) -> str:
    """A Modelica array literal: ``{25200, 82800}``."""
    return "{" + ", ".join(f"{value:g}" if float(value).is_integer() else repr(value) for value in values) + "}"


__all__ = ["NotNumericError", "evaluate", "modelica_array", "scalarize"]
