"""Smoke tests that verify every public symbol in trano can be imported.

These guard against future move-and-forget regressions in the package
__init__ files. They are deliberately cheap so they run in a few ms.
"""

import importlib

import pytest


def test_top_level_import() -> None:
    importlib.import_module("trano")


def test_elements_public_surface() -> None:
    from trano import elements

    for name in elements.__all__:
        assert hasattr(elements, name), f"{name} declared in __all__ but not exported"


@pytest.mark.parametrize(
    "module",
    [
        "trano",
        "trano.cli.app",
        "trano.cli.parse",
        "trano.cli.schema",
        "trano.core.topology",
        "trano.core.utils",
        "trano.data.include",
        "trano.data_models.conversion",
        "trano.data_models.converter",
        "trano.elements",
        "trano.elements.base",
        "trano.elements.boundary",
        "trano.elements.bus",
        "trano.elements.common_base",
        "trano.elements.connection",
        "trano.elements.connection.connect",
        "trano.elements.connection.connection",
        "trano.elements.connection.port",
        "trano.elements.construction",
        "trano.elements.containers",
        "trano.elements.containers.container",
        "trano.elements.containers.containers",
        "trano.elements.containers.models",
        "trano.elements.control",
        "trano.elements.data_bus.controller_bus",
        "trano.elements.data_bus.inputs",
        "trano.elements.envelope",
        "trano.elements.figure",
        "trano.elements.library.base",
        "trano.elements.library.components",
        "trano.elements.library.library",
        "trano.elements.library.parameters",
        "trano.elements.solar",
        "trano.elements.space",
        "trano.elements.system",
        "trano.elements.types",
        "trano.elements.utils",
        "trano.exceptions",
        "trano.plot.plot",
        "trano.reporting.docx",
        "trano.reporting.html",
        "trano.reporting.reporting",
        "trano.reporting.types",
        "trano.reporting.utils",
        "trano.simulate.simulate",
    ],
)
def test_module_importable(module: str) -> None:
    importlib.import_module(module)


def test_module_docstrings_present() -> None:
    """Every module under trano/ should declare a one-line docstring."""
    import pkgutil

    import trano

    missing: list[str] = []
    for module_info in pkgutil.walk_packages(trano.__path__, prefix="trano."):
        if "__pycache__" in module_info.name:
            continue
        module = importlib.import_module(module_info.name)
        if not (module.__doc__ or "").strip():
            missing.append(module_info.name)
    assert not missing, f"missing module docstrings: {missing}"
