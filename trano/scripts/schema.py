from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml

TRANO_KEYS = ("alias", "libraries", "short_name", "render", "numerical", "deprecated", "default_from", "positive")


def _linkml_attributes(attributes: dict[str, Any]) -> dict[str, Any]:
    """The attributes as LinkML slots: trano's own keys dropped, the short names added as slots."""
    slots: dict[str, Any] = {}
    for name, attribute in attributes.items():
        if "func" in attribute:
            continue
        slot = {key: value for key, value in attribute.items() if key not in TRANO_KEYS}
        slots[name] = slot
        if attribute.get("short_name"):
            slots[attribute["short_name"]] = {
                "description": f"Same as {name}",
                "range": attribute["range"],
                **({"multivalued": True} if attribute.get("multivalued") else {}),
            }
    return slots


def _hoist_shared_attributes(schema: dict[str, Any]) -> None:
    """Declare once, as a schema slot, every attribute name used by several classes.

    LinkML gives an attribute the URI of its name, so the same name declared with different ranges or
    descriptions in several classes is one ambiguous property (gen-owl warns about each). The shared name
    becomes a top-level slot and each class narrows it with `slot_usage`, which keeps its range, default
    and bounds per class.
    """
    slots = schema.setdefault("slots", {})
    owners: dict[str, list[str]] = defaultdict(list)
    for class_name, definition in schema["classes"].items():
        for name in definition.get("attributes") or {}:
            owners[name].append(class_name)
    for name, classes in owners.items():
        if len(classes) < 2 and name not in slots:
            continue
        slots.setdefault(
            name,
            {
                "description": f"{name.replace('_', ' ').capitalize()}; its meaning, range and "
                f"default depend on the class (see slot_usage)."
            },
        )
        for class_name in classes:
            definition = schema["classes"][class_name]
            usage = definition["attributes"].pop(name)
            definition.setdefault("slots", []).append(name)
            definition.setdefault("slot_usage", {})[name] = usage
            if not definition["attributes"]:
                definition.pop("attributes")


def create_final_schema(parameters_path: Path, trano_final_path: Path, trano_path: Path) -> None:
    trano = yaml.safe_load(trano_path.read_text())
    parameters = yaml.safe_load(parameters_path.read_text())
    for name, parameter in parameters.items():
        parameter.pop("classes", None)
        parameter.pop("libraries", None)
        parameter["attributes"] = _linkml_attributes(parameter["attributes"])
        trano["classes"][name] = parameter
    _hoist_shared_attributes(trano)
    with trano_final_path.open("w") as file:
        yaml.dump(trano, file)
