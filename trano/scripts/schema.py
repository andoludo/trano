from pathlib import Path
from typing import Any

import yaml

TRANO_KEYS = ("alias", "libraries", "short_name", "render", "numerical", "deprecated", "default_from")


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


def create_final_schema(parameters_path: Path, trano_final_path: Path, trano_path: Path) -> None:
    trano = yaml.safe_load(trano_path.read_text())
    parameters = yaml.safe_load(parameters_path.read_text())
    for name, parameter in parameters.items():
        parameter.pop("classes", None)
        parameter.pop("libraries", None)
        parameter["attributes"] = _linkml_attributes(parameter["attributes"])
        trano["classes"][name] = parameter
    yaml.dump(trano, trano_final_path.open("w"))
