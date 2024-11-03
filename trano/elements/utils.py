import re
from typing import TYPE_CHECKING, Any, Callable, List

from trano import elements

if TYPE_CHECKING:

    from trano.elements import Port


def to_snake_case(name: str) -> str:
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def compose_func(ports_: List["Port"]) -> Callable[[], List["Port"]]:
    return lambda: ports_


def _get_type(_type: Any) -> Any:  # noqa: ANN401
    if _type == "string":
        return str
    elif _type == "float":
        return float
    elif _type == "integer":
        return int
    elif _type == "boolean":
        return bool
    else:
        raise Exception("Unknown type")


def _get_default(v: Any) -> Any:  # noqa: ANN401
    if "ifabsent" not in v:
        return None
    tag = v["range"]
    if tag == "integer":
        tag = "int"
    value = v["ifabsent"].replace(tag, "")[1:-1]
    if value == "None":
        return None

    try:
        return _get_type(v["range"])(value)
    except Exception as e:

        raise e


# TODO: class names should be standardized!!
def import_element_function(function_name: str) -> Any:  # noqa: ANN401
    attribute = [
        attribute
        for attribute in elements.__all__
        if attribute.lower() == function_name.lower()
    ]
    if len(attribute) > 1:
        raise Exception(f"Element {function_name} has more than one match")
    if len(attribute) == 0:
        raise Exception(f"Element {function_name} not found")
    return getattr(elements, attribute[0])
