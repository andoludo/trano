import json
from pathlib import Path
from typing import Any, cast

from pydantic import BaseModel, Field, model_validator

from trano.elements.common_base import MediumTemplate
from trano.elements.jinja import compile_template
from trano.exceptions import UnknownLibraryError


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
    core_library: str | None = None
    medium: MediumTemplate
    constants: str = ""
    templates: Templates
    default: bool = False
    default_parameters: dict[str, Any] = Field(default_factory=dict)  # TODO: this should be baseparameters

    @model_validator(mode="after")
    def _render_constants(self) -> "Library":
        """Render the constants template once: it may refer to the medium and include shared blocks."""
        self.constants = compile_template(self.constants).render(library=self)
        return self

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
