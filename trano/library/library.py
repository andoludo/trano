from copy import deepcopy
from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional

import yaml
from pydantic import BaseModel, ConfigDict, Field, create_model

from trano.elements.components import COMPONENTS, DynamicComponentTemplate
from trano.elements.parameters import default_parameters
from trano.elements.types import BaseVariant
from trano.elements.utils import compose_func

if TYPE_CHECKING:
    from trano.elements import BaseParameter, Figure, Port, WallParameters


def tilts_processing_ideas(element: "WallParameters") -> List[str]:
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


class AvailableLibraries(BaseModel):
    ideas: List[Callable[[], "LibraryData"]] = Field(default=[lambda: None])
    buildings: List[Callable[[], "LibraryData"]] = Field(default=[lambda: None])

    def get_library_data(self, library: "Library", variant: str) -> Any:  # noqa: ANN401
        selected_variant = [
            variant_
            for variant_ in getattr(self, library.name.lower())
            if variant_().variant == variant
        ]
        if not selected_variant:
            return
        # TODO: to be more strict
        return selected_variant[0]()

    # TODO: this code should be library independent
    @classmethod
    def from_config(cls, name: str) -> Optional["AvailableLibraries"]:
        from trano.elements import BaseParameter, Figure, Port, parameters

        data = deepcopy(COMPONENTS)
        components_data__ = [
            component
            for component in data["components"]
            for classes_ in component["classes"]
            if name == classes_
        ]
        if not components_data__:
            return None
        components: Dict[str, Any] = {"ideas": [], "buildings": []}
        for component in components_data__:
            dynamic_component = None
            if component.get("component_template"):
                dynamic_component = DynamicComponentTemplate(
                    **component["component_template"]
                )
            if component["parameter_processing"].get("parameter", None):
                function_name = component["parameter_processing"]["function"]
                parameter_processing = partial(
                    getattr(parameters, function_name),
                    **{
                        function_name: component["parameter_processing"].get(
                            "parameter", {}
                        )
                    },
                )
            else:
                parameter_processing = getattr(
                    parameters, component["parameter_processing"]["function"]
                )

            component_ = create_model(
                f"Base{component['library'].capitalize() }{name.capitalize()}",
                __base__=LibraryData,
                template=(str, f"{component['template']}"),
                ports_factory=(
                    Callable[[], List[Port]],
                    compose_func([Port(**port) for port in component["ports"]]),
                ),
                component_template=(DynamicComponentTemplate, dynamic_component),
                variant=(str, component["variant"]),
                figures=(
                    List[Figure],
                    Field(
                        default_factory=lambda: [
                            Figure(**fig)
                            for fig in component.get("figures", [])  # noqa: B023
                        ]
                    ),
                ),
                parameter_processing=(
                    Callable[[BaseParameter], Dict[str, Any]],
                    parameter_processing,
                ),
            )

            if component["library"] == "default":
                components["ideas"].append(component_)
                components["buildings"].append(component_)
            else:
                components[component["library"]].append(component_)
        return cls(**components)


class LibraryData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    template: str = ""
    component_template: Optional[DynamicComponentTemplate] = None
    ports_factory: Callable[[], List["Port"]]
    variant: str = BaseVariant.default
    parameter_processing: Callable[
        ["BaseParameter"], Dict[str, Any]
    ] = default_parameters
    figures: List["Figure"] = Field(default=[])
