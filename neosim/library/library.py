from typing import Any, Callable, Dict, List, Literal, Union

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel

from neosim.models.elements.constants.buildings import BUILDINGS_CONSTANTS
from neosim.models.elements.constants.ideas import CONSTANTS
from neosim.models.elements.materials.base import MaterialProperties
from neosim.models.elements.materials.properties import (
    extract_buildings_data,
    extract_ideas_data,
)


def tilts_processing_ideas(element) -> List[str]:
    return [f"IDEAS.Types.Tilt.{tilt.value.capitalize()}" for tilt in element.tilts]


class Library(BaseModel):
    name: str
    merged_external_boundaries: bool = False
    functions: Dict[str, Callable[[Any], Any]] = {
        "tilts_processing_ideas": tilts_processing_ideas
    }
    constants: str = ""
    extract_properties: Callable[[str, Any], Any]


LibraryNames = Literal["IDEAS", "Buildings"]


class Ideas(Library):
    name: str = "IDEAS"
    merged_external_boundaries: bool = True
    constants: str = CONSTANTS
    extract_properties: Callable[
        [str, NodeView], MaterialProperties
    ] = extract_ideas_data


class Buildings(Library):
    name: str = "Buildings"
    merged_external_boundaries: bool = False
    constants: str = BUILDINGS_CONSTANTS
    extract_properties: Callable[
        [str, NodeView], MaterialProperties
    ] = extract_buildings_data


Libraries = Union[Ideas, Buildings]
