import importlib
import re
from typing import List, Callable, Any

from mypy.semanal_main import TYPE_CHECKING

if TYPE_CHECKING:

    from trano.elements.base import Port


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

def dynamic_import_function(module_name: str, function_name: str)-> Callable[[Any], Any]:
    module = importlib.import_module(module_name)


    function = getattr(module, function_name)

    return function