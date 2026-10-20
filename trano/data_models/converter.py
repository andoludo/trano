import json
from functools import cache
from pathlib import Path
from typing import Any

from linkml.generators.pythongen import PythonGenerator  # type: ignore
from linkml_runtime.loaders import json_loader, yaml_loader  # type: ignore
from linkml_runtime.dumpers import json_dumper  # type: ignore


def delete_none(_dict: dict[str, Any]) -> dict[str, Any]:
    # TODO: this function needs to be reviewed
    """Delete None values recursively from all of the dictionaries"""
    for key, value in list(_dict.items()):
        if isinstance(value, dict):
            delete_none(value)
        elif value is None:
            del _dict[key]
        elif isinstance(value, list):
            for v_i in value:
                if isinstance(v_i, dict):
                    delete_none(v_i)

    return _dict


@cache
def _compile_schema_module(schema: str) -> Any:  # noqa: ANN401
    """Compile the LinkML schema into a Python module (cached: it is expensive)."""
    return PythonGenerator(schema).compile_module()


def _extend_variant_enums(python_module: Any) -> None:  # noqa: ANN401
    """Add the externally registered variants to the schema's variant enums, which only know
    the built-in ones. The precise check, variant by component, already ran in
    ``trano.validation.schema.validate_schema``."""
    from linkml_runtime.linkml_model.meta import PermissibleValue  # type: ignore

    from trano.elements.library.components import all_external_variant_names

    names = all_external_variant_names()
    if not names:
        return
    for attribute, value in vars(python_module).items():
        if attribute.endswith("Variant") and isinstance(value, type):
            for name in names:
                if name not in vars(value):
                    setattr(value, name, PermissibleValue(text=name))


def converter(
    input: str,
    target_class: str,
    schema: str | Path,
    output_format: str | None = None,
) -> dict[str, Any]:
    """Load YAML/JSON instance data conforming to a LinkML schema as a dictionary.

    The data is loaded through the schema's compiled Python model, which validates
    and normalizes it, and is returned as a plain dictionary without None values.
    """
    python_module = _compile_schema_module(str(schema))
    _extend_variant_enums(python_module)
    py_target_class = python_module.__dict__[target_class]
    loader = yaml_loader if Path(input).suffix in {".yaml", ".yml"} else json_loader
    obj = loader.load(source=input, target_class=py_target_class)
    return json.loads(json_dumper.dumps(obj, inject_type=False))  # type: ignore
