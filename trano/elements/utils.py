import re
from typing import TYPE_CHECKING, Any, Callable, List, Tuple, Dict, Optional

import networkx as nx

from trano import elements
from trano.elements.common_base import Point

if TYPE_CHECKING:

    from trano.elements import Port
    from trano.topology import Network


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


def generate_normalized_layout(
    network: "Network",
    scale: int = 200,
    origin: Optional[Point] = None,
    container_type: Optional[str] = None,
) -> Dict[str, Tuple[float, float]]:
    def normalize(value, min_val, max_val, scale=200):
        return ((value - min_val) / (max_val - min_val)) * scale if max_val != min_val else scale/2

    origin = origin or Point(x=0, y=0)
    new_graph = nx.DiGraph()
    edges = [
        (e[0].name, e[1].name)
        for e in network.graph.edges
        if (
            e[0].container_type == container_type
            and e[1].container_type == container_type
        )
        or (container_type is None)
    ]
    nodes = [
        n.name
        for n in network.graph.nodes
        if (n.container_type == container_type) or (container_type is None)
    ]
    if not nodes:
        return {}
    new_graph.add_nodes_from(nodes)
    new_graph.add_edges_from(edges)
    pos = nx.nx_pydot.pydot_layout(new_graph, prog="sfdp")
    x_values, y_values = zip(*pos.values())
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    try:
        return {
            node: (
                origin.x + normalize(x, x_min, x_max, scale),
                origin.y + normalize(y, y_min, y_max, scale),
            )
            for node, (x, y) in pos.items()
        }
    except:
        raise
