"""Checks of the ids of a model and of the references between its objects."""

from collections import Counter, defaultdict
from collections.abc import Iterator
from typing import Any

from trano.validation.graph import ModelGraph
from trano.validation.issues import Issue

OPAQUE_BOUNDARY_KEYS = ("external_walls", "floor_on_grounds", "adiabatic_walls")
MATERIAL_KEYS = ("material", "gas", "glass_material")


def clean_name(value: str) -> str:
    """The Modelica name trano gives an id (`trano.elements.base.BaseElement.clean_name`)."""
    return value.lower().replace(":", "_") if ":" in value else value


def _sources(data: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """The model and its `default` block, both holding materials and constructions."""
    return data, data.get("default") or {}


def _ids(data: dict[str, Any], key: str) -> list[str]:
    return [item["id"] for source in _sources(data) for item in source.get(key) or [] if "id" in item]


def _internal_wall(wall: dict[str, Any]) -> str:
    return f"internal wall {wall.get('space_1')}-{wall.get('space_2')}"


def check_ids(data: dict[str, Any], graph: ModelGraph) -> Iterator[Issue]:
    """Ids shared by several spaces, materials or constructions, or giving the same Modelica name."""
    controls = [component.control["id"] for component in graph.components.values() if component.control.get("id")]
    by_name: dict[str, set[str]] = defaultdict(set)
    for id_ in [*graph.components, *graph.spaces, *controls]:
        by_name[clean_name(id_)].add(id_)
    for name, same in by_name.items():
        if len(same) > 1:
            yield Issue(
                "error",
                min(same),
                f"the ids {', '.join(sorted(same))} all become the Modelica name "
                f"'{name}'. Rename them so that they differ in more than case and ':'/'_'.",
            )
    groups = {
        "space": [space["id"] for space in data.get("spaces") or []],
        "material": [id_ for key in MATERIAL_KEYS for id_ in _ids(data, key)],
        "construction or glazing": [*_ids(data, "constructions"), *_ids(data, "glazings")],
    }
    for label, ids in groups.items():
        for id_, count in Counter(ids).items():
            if count > 1:
                yield Issue(
                    "error",
                    id_,
                    f"{count} {label}s have this id (the 'default' block included). Give each one its own id.",
                )


def check_flow_references(graph: ModelGraph) -> Iterator[Issue]:
    known = set(graph.components) | set(graph.spaces)
    for source, target in graph.declared_edges:
        for id_, owner in ((source, target), (target, source)):
            if id_ not in known:
                yield Issue(
                    "error",
                    owner,
                    f"refers to '{id_}' in its inlets or outlets, but no component has "
                    f"this id. Declare '{id_}' or correct the reference.",
                )


def check_boundary_references(data: dict[str, Any]) -> Iterator[Issue]:
    """Walls and floors take a construction, windows a glazing."""
    constructions, glazings = set(_ids(data, "constructions")), set(_ids(data, "glazings"))
    for space in data.get("spaces") or []:
        boundaries = space.get("external_boundaries") or {}
        for key in (*OPAQUE_BOUNDARY_KEYS, "windows"):
            expected, other = ("glazing", "construction") if key == "windows" else ("construction", "glazing")
            expected_ids, other_ids = (glazings, constructions) if key == "windows" else (constructions, glazings)
            for index, boundary in enumerate(boundaries.get(key) or []):
                reference = boundary.get("construction")
                if reference is None or reference in expected_ids:
                    continue
                if reference in other_ids:
                    message = f"uses '{reference}', which is a {other}; {key.replace('_', ' ')} take a {expected}."
                else:
                    message = f"uses the {expected} '{reference}', which is not defined. Add it or correct the id."
                yield Issue("error", space["id"], f"{key}[{index}] {message}")
    spaces = {space["id"] for space in data.get("spaces") or []}
    for wall in data.get("internal_walls") or []:
        for key in ("space_1", "space_2"):
            if wall.get(key) is not None and wall[key] not in spaces:
                yield Issue("error", _internal_wall(wall), f"{key} '{wall[key]}' is not a space of the model.")
        if wall.get("construction") is not None and wall["construction"] not in constructions:
            yield Issue("error", _internal_wall(wall), f"the construction '{wall['construction']}' is not defined.")


def check_layer_references(data: dict[str, Any]) -> Iterator[Issue]:
    """Construction layers take a material, glazing layers a gas or a glass."""
    materials = {id_ for key in MATERIAL_KEYS for id_ in _ids(data, key)}
    known = {"material": materials, "gas": set(_ids(data, "gas")), "glass": set(_ids(data, "glass_material"))}
    for source in _sources(data):
        for key in ("constructions", "glazings"):
            for construction in source.get(key) or []:
                for index, layer in enumerate(construction.get("layers") or []):
                    for layer_key, ids in known.items():
                        if layer.get(layer_key) is not None and layer[layer_key] not in ids:
                            yield Issue(
                                "error",
                                construction["id"],
                                f"layer {index} uses the {layer_key} '{layer[layer_key]}', which is not defined.",
                            )


def check_references(data: dict[str, Any], graph: ModelGraph) -> Iterator[Issue]:
    yield from check_ids(data, graph)
    yield from check_flow_references(graph)
    yield from check_boundary_references(data)
    yield from check_layer_references(data)
