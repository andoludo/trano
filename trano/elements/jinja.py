"""Shared Jinja environment for Modelica template rendering.

Creating a Jinja Environment (and re-parsing template strings) for every render
dominates model-generation time, so a single module-level environment plus a
compiled-template cache is shared by all renderers.
"""

from functools import cache
from pathlib import Path
from types import ModuleType

from jinja2 import Environment, FileSystemLoader, Template
from markupsafe import Markup

ENVIRONMENT = Environment(
    trim_blocks=True,
    lstrip_blocks=True,
    loader=FileSystemLoader(str(Path(__file__).parents[1].joinpath("templates"))),
    autoescape=True,
)
ENVIRONMENT.filters["enumerate"] = enumerate
ENVIRONMENT.filters["frozenset"] = frozenset


def modelica_number(value: float) -> str:
    return f"{float(value):.10g}"


def _modelica_package(name: str) -> ModuleType:
    """The Python package that carries a Modelica library trano generates models with: FastSmoothBuildings."""
    if name != "FastSmoothBuildings":
        raise ValueError(f"trano uses no Modelica library {name} from a Python package")
    import fastsmoothbuildings

    return fastsmoothbuildings


@cache
def modelica_library(name: str) -> Markup:
    """The Modelica source of a library that a generated package embeds: FastSmoothBuildings, from its Python package,
    without its examples, validation models, guide and documentation (the library itself carries them)."""
    bundle = _modelica_package(name).bundle(documentation=False, examples=False)
    return Markup(bundle)  # noqa: S704 - the library, not user input


def modelica_library_path(name: str) -> Path:
    """The folder of a library that a generated package uses without embedding it: where tools load it from."""
    return Path(_modelica_package(name).LIBRARY)


def modelica_library_version(name: str) -> str:
    """The version of a library, for the ``uses`` annotation of a package that does not embed it."""
    return str(_modelica_package(name).__version__)


class UnsupportedElementError(NotImplementedError):
    """An element of the model that the library does not model."""


def unsupported(message: str) -> str:
    """Stop the generation of a model: called by the template of an element the library does not model."""
    raise UnsupportedElementError(message)


ENVIRONMENT.filters["mo"] = modelica_number
ENVIRONMENT.globals["modelica_library"] = modelica_library
ENVIRONMENT.globals["modelica_library_version"] = modelica_library_version
ENVIRONMENT.globals["unsupported"] = unsupported

# For small inline templates that don't use the macros/templates directory.
STRING_ENVIRONMENT = Environment(autoescape=True)


@cache
def compile_template(source: str) -> Template:
    return ENVIRONMENT.from_string(source)


@cache
def compile_string_template(source: str) -> Template:
    return STRING_ENVIRONMENT.from_string(source)
