"""Shared jinja2 Environment used by container templates."""

from pathlib import Path

from jinja2 import Environment, FileSystemLoader

ENVIRONMENT = Environment(
    trim_blocks=True,
    lstrip_blocks=True,
    loader=FileSystemLoader(str(Path(__file__).parents[2].joinpath("templates"))),
    autoescape=True,
)
ENVIRONMENT.filters["frozenset"] = frozenset
ENVIRONMENT.filters["enumerate"] = enumerate
