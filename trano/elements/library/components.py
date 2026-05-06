import os
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
    for file in folder.rglob("*.yaml"):
        file_data = yaml.safe_load(file.read_text())
        if not file_data:
            continue
        if not isinstance(file_data, list):
            raise ValueError(
                f"Variant file {file} must contain a list of variant definitions at the top level."
            )
        components += file_data
    return components


def _initial_extra_paths() -> list[Path]:
    raw = os.environ.get(_TRANO_VARIANTS_PATH_ENV)
    if not raw:
        return []
    return [Path(p).expanduser().resolve() for p in raw.split(os.pathsep) if p]


class Components(BaseModel):
    components: list[dict[str, Any]]

    @classmethod
    def load(cls, extra_paths: list[Path] | None = None) -> "Components":
        _BUILTIN_MODELS_PATH.mkdir(exist_ok=True)
        components = _read_variants_from_folder(_BUILTIN_MODELS_PATH)

        all_extra_paths = list(extra_paths or []) + list(_EXTRA_VARIANT_PATHS)
        for extra_path in all_extra_paths:
            extra = Path(extra_path).expanduser().resolve()
            if not extra.exists():
                raise FileNotFoundError(
                    f"External variant folder does not exist: {extra}"
                )
            if not extra.is_dir():
                raise NotADirectoryError(
                    f"External variant folder must be a directory: {extra}"
                )
            components += _read_variants_from_folder(extra)

        return cls(components=components)

    def reload(self) -> None:
        refreshed = type(self).load()
        self.components = refreshed.components

    def get_components(self, component_name: str) -> list[LibraryData]:
        libraries_data = [LibraryData.model_validate(c) for c in self.components if component_name in c["classes"]]
        if len({(ld.variant, ld.library) for ld in libraries_data}) != len(libraries_data):
            raise ValueError(f"Duplicate variant for {component_name}")
        return libraries_data


def register_variants_folder(path: str | Path) -> None:
    """Register a folder outside of trano that contains additional variant
    YAML definitions. Once registered, the variants are immediately available
    to :class:`trano.elements.base.BaseElement` subclasses through the global
    ``COMPONENTS`` registry. The folder is scanned recursively for ``*.yaml``
    files, each file containing a list of variant definitions with the same
    schema as the built-in variants under ``trano/elements/library/models``.

    A folder can also be registered before importing trano by setting the
    ``TRANO_VARIANTS_PATH`` environment variable to a ``os.pathsep`` separated
    list of folders.

    :param path: A path to a folder containing ``*.yaml`` variant definitions.
    :raises FileNotFoundError: If the folder does not exist.
    :raises NotADirectoryError: If the path is not a directory.
    """
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"External variant folder does not exist: {resolved}")
    if not resolved.is_dir():
        raise NotADirectoryError(f"External variant folder must be a directory: {resolved}")
    if resolved not in _EXTRA_VARIANT_PATHS:
        _EXTRA_VARIANT_PATHS.append(resolved)
    COMPONENTS.reload()


def registered_variants_folders() -> list[Path]:
    """Return the list of currently registered external variant folders."""
    return list(_EXTRA_VARIANT_PATHS)


def clear_variants_folders() -> None:
    """Remove all externally registered variant folders and reload the
    built-in variants. Useful for tests."""
    _EXTRA_VARIANT_PATHS.clear()
    COMPONENTS.reload()


_EXTRA_VARIANT_PATHS.extend(_initial_extra_paths())
COMPONENTS = Components.load()
