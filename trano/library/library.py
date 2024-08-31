from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import yaml
from pydantic import BaseModel

from trano.elements import WallParameters


def tilts_processing_ideas(element: WallParameters) -> List[str]:
    return [f"IDEAS.Types.Tilt.{tilt.value.capitalize()}" for tilt in element.tilts]


class Templates(BaseModel):
    is_package: bool = False
    construction: str
    glazing: str
    material: Optional[str] = None
    main: str


def read_libraries() -> Dict[str, Dict[str, Any]]:
    library_path = Path(__file__).parent.joinpath("library.yaml")
    data: Dict[str, Dict[str, Any]] = yaml.safe_load(library_path.read_text())
    return data


class Library(BaseModel):
    name: str
    merged_external_boundaries: bool = False
    functions: Dict[str, Callable[[Any], Any]] = {
        "tilts_processing_ideas": tilts_processing_ideas
    }
    constants: str = ""
    templates: Templates
    default: bool = False

    @classmethod
    def from_configuration(cls, name: str) -> "Library":
        libraries = read_libraries()

        if name not in libraries:
            raise ValueError(f"Library {name} not found")
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
