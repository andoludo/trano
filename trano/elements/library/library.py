import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast
from collections.abc import Callable

from pydantic import BaseModel, Field

from trano.elements.common_base import MediumTemplate
from trano.elements.types import TILT_MAPPING, DEFAULT_TILT
from trano.exceptions import UnknownLibraryError

if TYPE_CHECKING:
    from trano.elements import WallParameters


# TODO: this must go!!!
def tilts_processing_ideas(element: "WallParameters") -> list[str | int]:
    return [
        (f"IDEAS.Types.Tilt.{tilt.value.capitalize()}" if tilt.value in DEFAULT_TILT else TILT_MAPPING[tilt.value])
        for tilt in element.tilts
    ]


class Templates(BaseModel):
    is_package: bool = False
    construction: str
    glazing: str
    material: str | None = None
    main: str


def read_libraries() -> dict[str, dict[str, Any]]:
    library_json_path = Path(__file__).parent.joinpath("library.json")
    return cast(dict[str, dict[str, Any]], json.loads(library_json_path.read_text()))


class Library(BaseModel):
    name: str
    merged_external_boundaries: bool = False
    functions: dict[str, Callable[[Any], Any]] = {"tilts_processing_ideas": tilts_processing_ideas}
    core_library: str | None = None
    medium: MediumTemplate
    constants: str = ""
    templates: Templates
    default: bool = False
    default_parameters: dict[str, Any] = Field(default_factory=dict)  # TODO: this should be baseparameters

    def base_library(self) -> str:
        return self.core_library or self.name

    @classmethod
    def from_configuration(cls, name: str) -> "Library":
        libraries = read_libraries()

        if name not in libraries:
            raise UnknownLibraryError(f"Library {name} not found. Available libraries: {list(libraries)}")
        library_data = libraries[name]
        return cls(**library_data)

    @classmethod
    def load_default(cls) -> "Library":
        libraries = read_libraries()
        default_library = [library_data for _, library_data in libraries.items() if library_data.get("default")]
        if not default_library:
            raise ValueError("No default library found")
        return cls(**default_library[0])
