import json
import warnings
from pathlib import Path
from typing import Any, Final, cast

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


class Connectors(BaseModel):
    """Connector types of the ports of the containers, and of the data bus of the elements."""

    heat: str
    fluid_a: str
    fluid_b: str
    fluids_a: str = Field(description="Vector of fluid ports, inlet side")
    fluids_b: str = Field(description="Vector of fluid ports, outlet side")
    real_input: str
    real_output: str
    data_bus: str
    electrical: str | None = Field(description="Electrical terminal; none: no electrical grid")
    medium: bool = Field(description="The fluid ports take the medium as a replaceable package")


DEPRECATED_NAMES: Final = {"FastModel": "FastSmoothBuildings", "FastModelMPC": "FastSmoothBuildingsMPC"}


def library_name(name: str) -> str:
    """The name of a library, its former name replaced (with a deprecation warning)."""
    if name not in DEPRECATED_NAMES:
        return name
    warnings.warn(
        f"The library {name} is now {DEPRECATED_NAMES[name]} (the Modelica library FastSmoothBuildings); the name "
        f"{name} will be removed in trano 1.0.",
        DeprecationWarning,
        stacklevel=3,
    )
    return DEPRECATED_NAMES[name]


def _read_library_json() -> dict[str, dict[str, Any]]:
    return cast(dict[str, dict[str, Any]], json.loads(Path(__file__).parent.joinpath("library.json").read_text()))


def _base(key: str) -> Any:  # noqa: ANN401
    """A value of the abstract ``base`` entry of library.json: the default of the libraries built in code."""
    return _read_library_json()["base"][key]


def read_libraries() -> dict[str, dict[str, Any]]:
    """The libraries of library.json; a library with ``extends`` takes the entry of that library, its own keys
    replacing the inherited ones, and the keys of an inherited object merged with its own (every library extends
    the abstract ``base``; FastSmoothBuildingsMPC is FastSmoothBuildings under another name). Abstract entries are not
    libraries."""
    libraries = _read_library_json()
    return {name: _resolve(name, libraries) for name in libraries if not libraries[name].get("abstract")}


def _resolve(name: str, libraries: dict[str, dict[str, Any]]) -> dict[str, Any]:
    library = dict(libraries[name])
    parent = library.pop("extends", None)
    if not parent:
        return library
    inherited = _resolve(parent, libraries)
    inherited.pop("abstract", None)
    return inherited | {
        key: inherited[key] | value if isinstance(value, dict) and isinstance(inherited.get(key), dict) else value
        for key, value in library.items()
    }


class Library(BaseModel):
    name: str
    merged_external_boundaries: bool = False
    core_library: str | None = None
    medium: MediumTemplate
    constants: str = ""
    templates: Templates
    default: bool = False
    default_parameters: dict[str, Any] = Field(default_factory=dict)  # TODO: this should be baseparameters
    parameters_from: str | None = Field(
        default=None,
        description="Library whose parameter names the elements take (FastSmoothBuildings: the names of Buildings).",
    )
    package_template: str = Field(
        default="base.jinja2", description="Template of the Modelica package generated for a network."
    )
    use_containers: bool = Field(default=True, description="Group the components in containers (sub-models).")
    connectors: Connectors = Field(default_factory=lambda: Connectors(**_base("connectors")))
    data_bus_template: str = Field(
        default_factory=lambda: _base("data_bus_template"),
        description="Template declaring the connector of the data bus: an expandable connector, empty, or with the "
        "signals of the network declared (``bus_members``) for the tools that do not add members to it (rumoca).",
    )

    @property
    def parameter_library(self) -> str:
        """Library whose parameter mappings process the parameters of the elements."""
        return self.parameters_from or self.name

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
        name = library_name(name)

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
