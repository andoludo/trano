"""The description of a model for its reports: what trano-simulate needs to know of the network.

``model_description`` turns the elements of a network into the records of the report tables (spaces,
constructions, systems), the figures of the result variables of each element, and the descriptions of the
parameters. It is written next to the model (``<name>.description.json``) and read by
``trano_simulate.reporting``, which needs nothing else from trano.
"""

from pathlib import Path
from typing import TYPE_CHECKING, Any

from trano_simulate.plot.figure import NamedFigure as ResultFigure
from trano_simulate.reporting.description import DESCRIPTION_SUFFIX, ModelDescription

from trano.elements import BaseElement, BaseParameter, BaseSimpleWall, BaseWall
from trano.elements.envelope import MergedBaseWall
from trano.elements.space import Space
from trano.elements.system import Boiler, System

if TYPE_CHECKING:
    from trano.topology import Network

BOUNDARY_PARAMETERS = {
    "name": True,
    "surface": True,
    "type": True,
    "azimuth": True,
    "tilt": True,
    "construction": {"name"},
}
SYSTEM_PARAMETERS = {"name": True, "parameters": True, "type": True}
SPACE_LISTS = {
    "emissions": SYSTEM_PARAMETERS,
    "ventilation_inlets": SYSTEM_PARAMETERS,
    "ventilation_outlets": SYSTEM_PARAMETERS,
    "external_boundaries": BOUNDARY_PARAMETERS,
    "internal_elements": BOUNDARY_PARAMETERS,
}
SUBSYSTEMS = ["control", "emissions", "ventilation_inlets", "ventilation_outlets"]


def documented_elements(network: "Network") -> list[BaseElement]:
    """The elements the reports describe: the spaces, their walls and the boilers."""
    return [x for x in list(network.graph.nodes) if isinstance(x, (Boiler, Space, BaseSimpleWall, MergedBaseWall))]


def _dump_list_attributes(element: BaseElement, attribute_name: str, include_mapping: object) -> list[dict[str, Any]]:
    records = []
    for item in getattr(element, attribute_name):
        record = item.model_dump(by_alias=True, exclude_none=True, mode="json", include=include_mapping)
        if item.parameters:
            record["parameters"] = item.parameters.model_dump(by_alias=True, exclude_none=True)
        records.append(record)
    return records


def space_records(elements: list[BaseElement]) -> list[dict[str, Any]]:
    records = []
    for space in [element for element in elements if isinstance(element, Space)]:
        record = space.model_dump(
            mode="json", include={"name": True, "occupancy": {"name": True}}, exclude_none=True, by_alias=True
        )
        if space.parameters:
            record["parameters"] = space.parameters.model_dump(by_alias=True, exclude_none=True)
        if record.get("occupancy") and space.occupancy and space.occupancy.parameters:
            record["occupancy"]["parameters"] = space.occupancy.parameters.model_dump(by_alias=True, exclude_none=True)
        for key, include in SPACE_LISTS.items():
            if values := _dump_list_attributes(space, key, include):
                record[key] = values
        records.append(record)
    return records


def construction_records(elements: list[BaseElement]) -> list[dict[str, Any]]:
    constructions = {
        construction
        for wall in elements
        if isinstance(wall, BaseWall) and isinstance(wall, (BaseSimpleWall, MergedBaseWall))
        for construction in ([wall.construction] if hasattr(wall, "construction") else wall.constructions)
    }
    return [
        construction.model_dump(
            by_alias=True,
            exclude_none=True,
            exclude={
                "total_thermal_resistance",
                "total_thermal_capacitance",
                "u_value",
                "resistance_external",
                "resistance_external_remaining",
            },
        )
        for construction in constructions
    ]


def system_records(elements: list[BaseElement]) -> list[dict[str, Any]]:
    spaces = [element for element in elements if isinstance(element, Space)]
    in_spaces = {
        system for space in spaces for system in space.emissions + space.ventilation_inlets + space.ventilation_outlets
    }
    records = []
    for system in [element for element in elements if isinstance(element, System)]:
        if system in in_spaces:
            continue
        record = system.model_dump(
            by_alias=True, exclude_none=True, include={"name": True, "parameters": True, "type": True}
        )
        if system.parameters:
            record["parameters"] = system.parameters.model_dump()
        records.append(record)
    return records


def parameter_descriptions() -> dict[str, str]:
    """The description (or the title) of every parameter, by its YAML name."""
    return {
        field.alias: (field.description or field.title or "")
        for cls in BaseParameter.__subclasses__()
        for field in cls.model_fields.values()
        if field.alias
    }


def _figures(element: BaseElement) -> list[ResultFigure]:
    return [ResultFigure.model_validate(figure.model_dump()) for figure in element.figures]


def element_figures(element: BaseElement) -> list[ResultFigure]:
    """The figures plotted for an element: each of its figures followed by the figures of its subsystems."""
    figures = []
    for figure in _figures(element):
        figures.append(figure)
        for subsystem in SUBSYSTEMS:
            sub_element = getattr(element, subsystem, None)
            if sub_element is None:
                continue
            for sub in sub_element if isinstance(sub_element, list) else [sub_element]:
                figures += _figures(sub)
    return figures


def model_description(network: "Network") -> ModelDescription:
    elements = documented_elements(network)
    figures: dict[str, list[ResultFigure]] = {}
    for element in elements:  # the first element of a name, as the reports looked elements up
        if element.name:
            figures.setdefault(element.name, element_figures(element))
    return ModelDescription(
        name=network.name,
        spaces=space_records(elements),
        constructions=construction_records(elements),
        systems=system_records(elements),
        figures=figures,
        space_figures={element.name: _figures(element) for element in elements if isinstance(element, Space)},
        parameter_descriptions=parameter_descriptions(),
    )


def write_model_description(network: "Network", directory: Path) -> Path:
    """Write ``<directory>/<network name>.description.json`` and return its path."""
    return model_description(network).write(directory / f"{network.name}{DESCRIPTION_SUFFIX}")
