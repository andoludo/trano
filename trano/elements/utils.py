import logging
import re
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, List, Tuple, Dict, Optional

import networkx as nx
import yaml

from trano import elements
from trano.elements.common_base import Point

if TYPE_CHECKING:

    from trano.elements import Port
    from trano.topology import Network

logger = logging.getLogger(__name__)


class BlockStyleDumper(yaml.Dumper):
    def represent_scalar(
        self, tag: Any, value: Any, style: Optional[str] = None  # noqa: ANN401
    ) -> Any:  # noqa: ANN401
        if isinstance(value, str) and ("\n" in value):
            style = "|"
        return super().represent_scalar(tag, value, style)


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
    def normalize(
        value: float, min_val: float, max_val: float, scale: int = 200
    ) -> float:
        return (
            ((value - min_val) / (max_val - min_val)) * scale
            if max_val != min_val
            else scale / 2
        )

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
    try:
        pos = nx.nx_pydot.pydot_layout(new_graph, prog="sfdp")
    except Exception as e:
        logger.warning(
            f"Error generating layout using graphviz. {e}. Graphviz is probably not installed."
        )
        pos = nx.planar_layout(new_graph)
    x_values, y_values = zip(*pos.values())
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    return {
        node: (
            origin.c_.x + normalize(x, x_min, x_max, scale),
            origin.c_.y + normalize(y, y_min, y_max, scale),
        )
        for node, (x, y) in pos.items()
    }


def wrap_with_raw(template: str) -> str:
    pattern = (
        r"(\{\s*-?\d+\s*,\s*-?\d+\s*(?:,\s*-?\d+\s*)?\})|"
        r"(\{\{\s*-?\d+\s*,\s*-?\d+\s*\},\{\s*-?\d+\s*,\s*-?\d+\s*\}\})"
    )
    return re.sub(
        pattern, lambda m: f"{{% raw %}} {m.group(0)} {{% endraw %}}", template
    )


def camel_to_snake(camel_str: str) -> str:
    snake_str = re.sub(r"(?<!^)(?=[A-Z])", "_", camel_str).lower()
    return snake_str


def json_component_name(classes: List[str]) -> str:
    return "_".join([camel_to_snake(c_) for c_ in sorted(c for c in classes)])


def dump_components(
    library_path: Path, library_components: List[Dict[str, Any]]
) -> None:
    library_path.mkdir(exist_ok=True)
    components_final = {
        json_component_name(component["classes"]): [
            cp
            for cp in library_components
            if json_component_name(component["classes"])
            == json_component_name(cp["classes"])
        ]
        for component in library_components
    }
    for file_name, components_ in components_final.items():
        file = library_path / f"{file_name}.yaml"
        with file.open("w") as file_:
            file_.write(
                yaml.dump(
                    components_, Dumper=BlockStyleDumper, default_flow_style=False
                )
            )
