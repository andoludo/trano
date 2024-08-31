from typing import (
    Any,
    ClassVar,
    Dict,
    List,
    Optional,
    Callable,
)

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    field_validator,
    model_validator,
)

from trano.elements.components import DynamicComponentTemplate
from trano.elements.connection import Port
from trano.elements.connection import _has_inlet_or_outlet, _is_inlet_or_outlet
from trano.elements.figure import Figure
from trano.elements.parameters import BaseParameter, param_from_config
from trano.elements.types import (
    Flow,
    BaseVariant,
)
from trano.library.library import Library, AvailableLibraries

class PortFound(Exception):
    def __init__(self, message: str, port: Port):
        super().__init__(message)
        self.port = port

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

    def _ports(self, ports_to_skip: List[Port]) -> List[Port]:
        return [port for port in self.ports if port not in ports_to_skip]

    def _available_ports_with_targets(
        self, self_ports: List[Port], target: "BaseElement"
    ) -> List[Port]:
        return [
            port
            for port in self_ports
            if port.targets
            and any(isinstance(target, target_) for target_ in port.targets)
            and port.is_available()
        ]

    def _available_ports_without_targets(self, self_ports: List[Port]) -> List[Port]:
        return [port for port in self_ports if not port.targets and port.is_available()]

    def _ports_with_specified_flow(
        self, _available_ports: List[Port], flow: Flow
    ) -> List[Port]:
        return [port for port in _available_ports if port.flow == flow]

    def _apply_function_to_inlet_outlet_ports(
        self,
        _available_ports: List[Port],
        target: "BaseElement",
        function: Callable[["BaseElement"], bool],
    ) -> List[Port]:
        return [
            port
            for port in self._ports_with_specified_flow(
                _available_ports, Flow.inlet_or_outlet
            )
            if function(target)
        ]

    def _unique_port(self, ports: List[Port]) -> None:
        if ports and len(ports) != 1:
            raise NotImplementedError

        if ports:
            raise PortFound("Port found",port=ports[0])

    def _one_port_max(self, ports: List[Port]) -> None:
        if ports:
            if len(ports) > 1:
                raise NotImplementedError
            raise PortFound("Port found",port=ports[0])
    def _ports_exist(self, ports: List[Port]) -> None:
        if ports:
            raise PortFound("Port found",port=ports[0])


    def _get_target_compatible_port(  # noqa: C901, PLR0911
        self, target: "BaseElement", flow: Flow, ports_to_skip: List[Port]
    ) -> Optional["Port"]:
        self_ports = self._ports(ports_to_skip)
        available_ports = self._available_ports_with_targets(self_ports, target)
        available_ports_without_target = self._available_ports_without_targets(
            self_ports
        )
        available_ports_with_interchangeable_flow = self._ports_with_specified_flow(
            available_ports, Flow.interchangeable_port
        )
        available_ports_with_undirected_flow = self._ports_with_specified_flow(
            available_ports, Flow.undirected
        )
        available_ports_with_directed_flow = self._ports_with_specified_flow(
            available_ports, flow
        )
        available_ports_with_inlet_or_outlet = (
            self._apply_function_to_inlet_outlet_ports(
                available_ports, target, _is_inlet_or_outlet
            )
        )
        available_ports_without_target_with_inlet_outlet = (
            self._apply_function_to_inlet_outlet_ports(
                available_ports_without_target, target, _has_inlet_or_outlet
            )
        )
        available_ports_without_target_with_directed_flow = (
            self._ports_with_specified_flow(available_ports_without_target, flow)
        )
        try:
            self._ports_exist(available_ports_with_interchangeable_flow)
            self._unique_port(available_ports_with_undirected_flow)
            self._ports_exist(available_ports_with_inlet_or_outlet)
            self._unique_port(available_ports_with_directed_flow)
            self._one_port_max(available_ports_without_target_with_inlet_outlet)
            self._one_port_max(available_ports_without_target_with_directed_flow)
        except PortFound as port:
            return port.port
        except NotImplementedError:
            raise NotImplementedError("Port configuration not supported!!")
        except Exception:
            raise
        return None

    def __hash__(self) -> int:
        return hash(f"{self.name}-{type(self).__name__}")


# This has to be here for now!!!
class Control(BaseElement):
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None
