import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, cast

from pydantic import BaseModel, Field

from trano.elements.common_base import MediumTemplate
from trano.exceptions import UnknownLibraryError

if TYPE_CHECKING:
    from trano.elements import WallParameters


# TODO: this must go!!!
def tilts_processing_ideas(element: "WallParameters") -> List[str]:
    return [f"IDEAS.Types.Tilt.{tilt.value.capitalize()}" for tilt in element.tilts]


class Templates(BaseModel):
    is_package: bool = False
    construction: str
    glazing: str
    material: Optional[str] = None
    main: str


def read_libraries() -> Dict[str, Dict[str, Any]]:
    library_json_path = Path(__file__).parent.joinpath("library.json")
    return cast(Dict[str, Dict[str, Any]], json.loads(library_json_path.read_text()))


class Library(BaseModel):
    name: str
    merged_external_boundaries: bool = False
    functions: Dict[str, Callable[[Any], Any]] = {
        "tilts_processing_ideas": tilts_processing_ideas
    }
    core_library: Optional[str] = None
    medium: MediumTemplate
    constants: str = ""
    templates: Templates
    default: bool = False
    default_parameters: Dict[str, Any] = Field(
        default_factory=dict
    )  # TODO: this should be baseparameters

    def base_library(self) -> str:
        return self.core_library or self.name

    @classmethod
    def from_configuration(cls, name: str) -> "Library":
        libraries = read_libraries()

        if name not in libraries:
            raise UnknownLibraryError(
                f"Library {name} not found. Available libraries: {list(libraries)}"
            )
        library_data = libraries[name]
        return cls(**library_data)

    @classmethod
    def load_default(cls) -> "Library":
        libraries = read_libraries()
        default_library = [
            library_data
            for _, library_data in libraries.items()
            if library_data.get("default")
        ]
        if not default_library:
            raise ValueError("No default library found")
        return cls(**default_library[0])
