from copy import deepcopy
from functools import partial
from typing import (
    TYPE_CHECKING,
    Any,
    Callable,
    ClassVar,
    Dict,
    List,
    Optional,
)

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    create_model,
    field_validator,
    model_validator,
)

from trano.elements import parameters
from trano.elements.connection import Port
from trano.elements.components import COMPONENTS, DynamicComponentTemplate
from trano.elements.connection import _has_inlet_or_outlet, _is_inlet_or_outlet
from trano.elements.parameters import BaseParameter, param_from_config
from trano.elements.figure import Figure
from trano.elements.parameters import default_parameters
from trano.elements.types import (
    Flow,
    BaseVariant,
)
from trano.elements.utils import compose_func

if TYPE_CHECKING:
    from trano.library.library import Library


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

    @classmethod
    def from_config(cls, name: str) -> Optional["AvailableLibraries"]:
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
                    getattr(
                        parameters, function_name),
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


class BaseElement(BaseModel):
    name_counter: ClassVar[
        int
    ] = 0  # TODO: this needs to be removed and replaced with a proper solution.
    name: Optional[str] = Field(default=None)
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    model_config = ConfigDict(arbitrary_types_allowed=True, extra="allow")
    parameters: Optional[BaseParameter] = None
    position: Optional[List[float]] = None
    ports: list[Port] = Field(default=[], validate_default=True)
    template: Optional[str] = None
    component_template: Optional[DynamicComponentTemplate] = None
    variant: str = BaseVariant.default
    libraries_data: Optional[AvailableLibraries] = None
    figures: List[Figure] = Field(default=[])

    @model_validator(mode="before")
    @classmethod
    def validate_libraries_data(cls, value: Dict[str, Any]) -> Dict[str, Any]:
        libraries_data = AvailableLibraries.from_config(cls.__name__)
        if libraries_data:
            value["libraries_data"] = libraries_data
        parameter_class = param_from_config(cls.__name__)
        if parameter_class:
            value["parameters"] = parameter_class()
        return value

    @model_validator(mode="after")
    def assign_default_name(self) -> "BaseElement":
        if self.name is None:
            self.name = f"{type(self).__name__.lower()}_{type(self).name_counter}"
            type(self).name_counter += 1
        return self

    @field_validator("name")
    @classmethod
    def clean_name(cls, value: str) -> str:
        if ":" in value:
            return value.lower().replace(":", "_")
        return value

    def assign_library_property(self, library: "Library") -> bool:
        if not self.libraries_data:
            return False
        library_data = self.libraries_data.get_library_data(library, self.variant)
        if not library_data:
            return False
        if not self.ports:
            self.ports = library_data.ports_factory()
        if not self.template:
            self.template = library_data.template
        if not self.component_template:
            self.component_template = library_data.component_template
        if not self.figures and library_data.figures:
            self.figures = [fig.render_key(self) for fig in library_data.figures]

        return True

    def processed_parameters(self, library: "Library") -> Any:  # noqa: ANN401
        if self.libraries_data:
            library_data = self.libraries_data.get_library_data(library, self.variant)
            if library_data and self.parameters:
                return library_data.parameter_processing(self.parameters)
        return {}

    def get_position(self, layout: Dict["BaseElement", Any]) -> None:
        if not self.position:
            self.position = list(layout.get(self))  # type: ignore

    def get_controllable_ports(self) -> List[Port]:
        return [port for port in self.ports if port.is_controllable()]

    @property
    def type(self) -> str:
        return type(self).__name__

    def _get_target_compatible_port(  # noqa: C901, PLR0911
        self, target: "BaseElement", flow: Flow, ports_to_skip: List[Port]
    ) -> Optional["Port"]:
        # TODO: function too complex. To be refactored
        self_ports = [port for port in self.ports if port not in ports_to_skip]
        ports = [
            port
            for port in self_ports
            if port.targets
            and any(isinstance(target, target_) for target_ in port.targets)
            and port.is_available()
            and port.flow == Flow.interchangeable_port
        ]
        if ports:
            return ports[0]
        ports = [
            port
            for port in self_ports
            if port.targets
            and any(isinstance(target, target_) for target_ in port.targets)
            and port.is_available()
            and port.flow == Flow.undirected
        ]
        if ports and len(ports) != 1:
            raise NotImplementedError
        if ports:
            return ports[0]
        ports = [
            port
            for port in self_ports
            if port.targets
               and any(isinstance(target, target_) for target_ in port.targets)
               and port.is_available()
               and port.flow == Flow.inlet_or_outlet
               and _is_inlet_or_outlet(target)
        ]
        if ports:
            return ports[0]
        ports = [
            port
            for port in self_ports
            if port.targets
            and any(isinstance(target, target_) for target_ in port.targets)
            and port.is_available()
            and port.flow == flow
        ]
        if ports and len(ports) != 1:
            raise NotImplementedError
        if ports:
            return ports[0]
        ports = [
            port
            for port in self_ports
            if not port.targets
               and port.is_available()
               and port.flow == Flow.inlet_or_outlet
               and _has_inlet_or_outlet(target)
        ]
        if ports:
            if len(ports) > 1:
                raise NotImplementedError
            return ports[0]
        ports = [
            port
            for port in self_ports
            if not port.targets and port.is_available() and port.flow == flow
        ]
        if ports:
            if len(ports) > 1:
                raise NotImplementedError
            return ports[0]

        return None

    def __hash__(self) -> int:
        return hash(f"{self.name}-{type(self).__name__}")


class LibraryData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    template: str = ""
    component_template: Optional[DynamicComponentTemplate] = None
    ports_factory: Callable[[], List[Port]]
    variant: str = BaseVariant.default
    parameter_processing: Callable[[BaseParameter], Dict[str, Any]] = default_parameters
    figures: List[Figure] = Field(default=[])


class Control(BaseElement):
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None
