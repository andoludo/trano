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


def _mpc_library() -> Markup:
    # Imported lazily: trano.mpc depends on the elements that use this environment.
    from trano.mpc.modelica import render_mpc_library

    return Markup(render_mpc_library())  # noqa: S704 - Modelica source rendered by Trano itself


ENVIRONMENT.filters["mo"] = modelica_number
ENVIRONMENT.globals["mpc_library"] = _mpc_library

# For small inline templates that don't use the macros/templates directory.
STRING_ENVIRONMENT = Environment(autoescape=True)


@cache
def compile_template(source: str) -> Template:
    return ENVIRONMENT.from_string(source)


@cache
def compile_string_template(source: str) -> Template:
    return STRING_ENVIRONMENT.from_string(source)
