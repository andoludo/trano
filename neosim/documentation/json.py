from typing import List, Type, Dict, Any, TYPE_CHECKING

from json2html import json2html

from neosim.models.elements.base import BaseElement
from neosim.models.elements.envelope.base import BaseSimpleWall
from neosim.models.elements.space import Space
from neosim.models.elements.system import System

if TYPE_CHECKING:
    from neosim.topology import Network


def _get_elements(
    elements: List[BaseElement], element_type: Type[Space | System | BaseSimpleWall]
) -> List[Space | System | BaseSimpleWall]:
    return [element for element in elements if isinstance(element, element_type)]


def _dump_list_attributes(
    element: BaseElement, attribute_name: str, include_mapping: Dict[str, Any] | set
) -> List[Dict[int, Any]]:
    values = []
    for el in getattr(element, attribute_name):
        values.append(
            el.model_dump(
                by_alias=True,
                exclude_none=True,
                mode="json",
                include=include_mapping,
            )
        )
    return values


def _get_space_documentation(elements: List[BaseElement]) -> List[Dict[str, Any]]:
    spaces = _get_elements(elements, Space)
    spaces_ = []
    boundary_parameters = {
        "name": True,
        "surface": True,
        "type": True,
        "azimuth": True,
        "tilt": True,
        "construction": {"name"},
    }
    system_parameters = {"name": True, "parameters": True, "type": True}
    document_mapping = {
        "emissions": system_parameters,
        "ventilation_inlets": system_parameters,
        "ventilation_outlets": system_parameters,
        "external_boundaries": boundary_parameters,
        "internal_elements": boundary_parameters,
    }
    for space in spaces:
        main_space = space.model_dump(
            mode="json",
            include={
                "name": True,
                "parameters": True,
                "occupancy": {"name": True, "parameters": True},
            },
            exclude_none=True,
            by_alias=True,
        )
        for key, value in document_mapping.items():
            values = _dump_list_attributes(space, key, value)
            if values:
                main_space[key] = _dump_list_attributes(space, key, value)
        spaces_.append(main_space)
    return spaces_


def _get_constructions(elements: List[BaseElement]) -> List[Dict[str, Any]]:
    return [
        c.model_dump(by_alias=True, exclude_none=True)
        for c in {w.construction for w in _get_elements(elements, BaseSimpleWall)}
    ]


def _get_systems(elements: List[BaseElement]) -> List[Dict[str, Any]]:
    spaces = _get_elements(elements, Space)

    systems_to_exclude = {
        system
        for space in spaces
        for system in space.emissions
        + space.ventilation_inlets
        + space.ventilation_outlets
    }
    systems = [
        system.model_dump(
            by_alias=True,
            exclude_none=True,
            include={"name": True, "parameters": True, "type": True},
        )
        for system in _get_elements(elements, System)
        if system not in systems_to_exclude
    ]
    return systems


def generate_html_documentation(network: "Network") -> Any:
    nodes = network.graph.nodes
    _get_constructions(nodes)
    _get_systems(nodes)
    spaces = _get_space_documentation(nodes)
    return json2html.convert(
        json=spaces,
        table_attributes="border='1'  align='center' bgcolor='#f0f0f0' style='border-collapse: collapse;'",
    )
