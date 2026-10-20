import os
from functools import cache
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel


from trano.elements.library.base import LibraryData

_BUILTIN_MODELS_PATH = Path(__file__).parent.joinpath("models")
_EXTRA_VARIANT_PATHS: list[Path] = []
_TRANO_VARIANTS_PATH_ENV = "TRANO_VARIANTS_PATH"


def _read_variants_from_folder(folder: Path) -> list[dict[str, Any]]:
    components: list[dict[str, Any]] = []
    for file in sorted(folder.rglob("*.yaml")):
        file_data = yaml.safe_load(file.read_text())
        if not file_data:
            continue
        if not isinstance(file_data, list):
            raise ValueError(f"Variant file {file} must contain a list of variant definitions at the top level.")
        components += file_data
    return components


def _check_variant_folder(path: Path | str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"External variant folder does not exist: {resolved}")
    if not resolved.is_dir():
        raise NotADirectoryError(f"External variant folder must be a directory: {resolved}")
    return resolved


def _environment_variant_paths() -> list[Path]:
    raw = os.environ.get(_TRANO_VARIANTS_PATH_ENV)
    if not raw:
        return []
    return [_check_variant_folder(p) for p in raw.split(os.pathsep) if p]


def _external_variant_paths() -> list[Path]:
    paths = _environment_variant_paths()
    return paths + [path for path in _EXTRA_VARIANT_PATHS if path not in paths]


def _external_component_definitions() -> list[dict[str, Any]]:
    definitions: list[dict[str, Any]] = []
    for path in _external_variant_paths():
        definitions += _read_variants_from_folder(_check_variant_folder(path))
    return definitions


def _class_names(component_key: str) -> set[str]:
    class_name = "".join(part.capitalize() for part in component_key.lower().split("_"))
    return {class_name, class_name.removesuffix("s")}  # a `spaces[i].variant` path names the list


def _external_definitions_of(component_key: str) -> list[dict[str, Any]]:
    class_names = _class_names(component_key)
    return [
        definition
        for definition in _external_component_definitions()
        if class_names & set(definition.get("classes", []))
    ]


def external_variant_names(component_key: str) -> set[str]:
    """The variant names that external folders define for one component.

    :param component_key: The YAML key of the component (``radiator``, ``emission_control``, ...).
    """
    return {str(definition.get("variant", "default")) for definition in _external_definitions_of(component_key)}


def all_external_variant_names() -> set[str]:
    """The variant names that external folders define, for any component."""
    return {str(definition.get("variant", "default")) for definition in _external_component_definitions()}


def external_variant_has_fluid_ports(component_key: str, variant: str) -> bool | None:
    """Whether an externally defined variant of a component declares fluid ports.

    None when no external folder defines this variant for this component: the caller falls back
    to what it knows of the built-in variants.
    """
    definitions = [
        definition
        for definition in _external_definitions_of(component_key)
        if str(definition.get("variant", "default")) == variant
    ]
    if not definitions:
        return None
    return any(
        port.get("medium") == "fluid" for definition in definitions for port in definition.get("ports") or []
    )


class Components(BaseModel):
    components: list[dict[str, Any]]

    @classmethod
    def load(cls) -> "Components":
        components = _read_variants_from_folder(_BUILTIN_MODELS_PATH)
        for extra_path in _external_variant_paths():
            components += _read_variants_from_folder(_check_variant_folder(extra_path))
        return cls(components=components)

    def get_components(self, component_name: str) -> list[LibraryData]:
        libraries_data = [LibraryData.model_validate(c) for c in self.components if component_name in c["classes"]]
        variants = [(ld.variant, library) for ld in libraries_data for library in ld.library]
        if len(set(variants)) != len(variants):
            raise ValueError(f"Duplicate variant for {component_name}")
        return libraries_data


@cache
def get_components_registry() -> Components:
    """Load the component library lazily: parsing every YAML file is slow."""
    return Components.load()


def register_variants_folder(path: str | Path) -> None:
    """Register a folder outside of trano holding additional component variant YAML files.

    The folder is scanned recursively for ``*.yaml`` files, each one a list of variant
    definitions with the same schema as the built-in ones under
    ``trano/elements/library/models``. Once registered, the variants are available to any
    element created afterwards, exactly like built-in variants. Folders can also be
    registered without code through the ``TRANO_VARIANTS_PATH`` environment variable, an
    ``os.pathsep``-separated list of folders.

    :param path: A folder containing ``*.yaml`` variant definitions.
    :raises FileNotFoundError: If the folder does not exist.
    :raises NotADirectoryError: If the path is not a directory.
    """
    resolved = _check_variant_folder(path)
    if resolved not in _EXTRA_VARIANT_PATHS:
        _EXTRA_VARIANT_PATHS.append(resolved)
    get_components_registry.cache_clear()


def registered_variants_folders() -> list[Path]:
    """The external variant folders registered with :func:`register_variants_folder`."""
    return list(_EXTRA_VARIANT_PATHS)


def clear_variants_folders() -> None:
    """Forget every registered external variant folder, keeping only the built-in variants."""
    _EXTRA_VARIANT_PATHS.clear()
    get_components_registry.cache_clear()
