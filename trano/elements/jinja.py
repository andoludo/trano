"""Shared Jinja environment for Modelica template rendering.

Creating a Jinja Environment (and re-parsing template strings) for every render
dominates model-generation time, so a single module-level environment plus a
compiled-template cache is shared by all renderers.
"""

from functools import cache
from pathlib import Path

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


@cache
def modelica_library(name: str) -> Markup:
    """The Modelica source of a library that a generated package embeds: FastSmoothBuildings, from its Python package,
    without its examples, validation models, guide and documentation (the library itself carries them)."""
    if name != "FastSmoothBuildings":
        raise ValueError(f"trano embeds no Modelica library {name}")
    import fastsmoothbuildings

    return Markup(fastsmoothbuildings.bundle(documentation=False, examples=False))  # noqa: S704 - the library, not user input


class UnsupportedElementError(NotImplementedError):
    """An element of the model that the library does not model."""


def unsupported(message: str) -> str:
    """Stop the generation of a model: called by the template of an element the library does not model."""
    raise UnsupportedElementError(message)


ENVIRONMENT.filters["mo"] = modelica_number
ENVIRONMENT.globals["modelica_library"] = modelica_library
ENVIRONMENT.globals["unsupported"] = unsupported

# For small inline templates that don't use the macros/templates directory.
STRING_ENVIRONMENT = Environment(autoescape=True)


@cache
def compile_template(source: str) -> Template:
    return ENVIRONMENT.from_string(source)


@cache
def compile_string_template(source: str) -> Template:
    return STRING_ENVIRONMENT.from_string(source)
