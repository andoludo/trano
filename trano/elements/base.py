from typing import Any, Callable, ClassVar, Dict, List, Optional

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from trano.elements.components import DynamicComponentTemplate
from trano.elements.connection import Port, _has_inlet_or_outlet, _is_inlet_or_outlet
from trano.elements.figure import NamedFigure
from trano.elements.parameters import BaseParameter, param_from_config
from trano.elements.types import BaseVariant, Flow, ContainerTypes
from trano.library.library import AvailableLibraries, Library


class PortFound(Exception):  # noqa: N818
    def __init__(self, message: str, port: Port) -> None:
        super().__init__(message)
        self.port = port


def _unique_port(ports: List[Port]) -> None:
    if ports and len(ports) != 1:
        raise NotImplementedError

    if ports:
        raise PortFound("Port found", port=ports[0])


def _one_port_max(ports: List[Port]) -> None:
    if ports:
        if len(ports) > 1:
            raise NotImplementedError
        raise PortFound("Port found", port=ports[0])


def _ports_exist(ports: List[Port]) -> None:
    if ports:
        raise PortFound("Port found", port=ports[0])


def _available_ports_with_targets(
    self_ports: List[Port], target: "BaseElement"
) -> List[Port]:
    return [
        port
        for port in self_ports
        if port.targets
        and any(isinstance(target, target_) for target_ in port.targets)
        and port.is_available()
    ]


def _available_ports_without_targets(self_ports: List[Port]) -> List[Port]:
    return [port for port in self_ports if not port.targets and port.is_available()]


def _ports_with_specified_flow(_available_ports: List[Port], flow: Flow) -> List[Port]:
    return [port for port in _available_ports if port.flow == flow]


def _apply_function_to_inlet_outlet_ports(
    _available_ports: List[Port],
    target_ports: List[Port],
    function: Callable[[List[Port]], bool],
) -> List[Port]:
    return [
        port
        for port in _ports_with_specified_flow(_available_ports, Flow.inlet_or_outlet)
        if function(target_ports)
    ]

def find_port(
    flow: Flow,
    available_ports: List[Port],
    available_ports_without_target: List[Port],
    target_ports: List[Port],
) -> Optional[Port]:

    available_ports_with_interchangeable_flow = _ports_with_specified_flow(
        available_ports, Flow.interchangeable_port
    )
    available_ports_with_undirected_flow = _ports_with_specified_flow(
        available_ports, Flow.undirected
    )
    available_ports_with_directed_flow = _ports_with_specified_flow(
        available_ports, flow
    )
    available_ports_with_inlet_or_outlet = _apply_function_to_inlet_outlet_ports(
        available_ports, target_ports, _is_inlet_or_outlet
    )
    available_ports_without_target_with_inlet_outlet = (
        _apply_function_to_inlet_outlet_ports(
            available_ports_without_target, target_ports, _has_inlet_or_outlet
        )
    )
    available_ports_without_target_with_directed_flow = _ports_with_specified_flow(
        available_ports_without_target, flow
    )
    try:
        _ports_exist(available_ports_with_interchangeable_flow)
        _unique_port(available_ports_with_undirected_flow)
        _ports_exist(available_ports_with_inlet_or_outlet)
        _unique_port(available_ports_with_directed_flow)
        _one_port_max(available_ports_without_target_with_inlet_outlet)
        _one_port_max(available_ports_without_target_with_directed_flow)
    except PortFound as port:
        return port.port
    except NotImplementedError:
        raise
    except Exception:
        raise
    return None

class BaseElement(BaseModel):
    name_counter: ClassVar[
        int
    ] = 0  # TODO: this needs to be removed and replaced with a proper solution.
    name: Optional[str] = Field(default=None)
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    container_annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.container_position) }},
    extent = {% raw %}{{10, -10}, {-10, 10}}
    {% endraw %})));"""
    model_config = ConfigDict(arbitrary_types_allowed=True, extra="allow")
    parameters: Optional[BaseParameter] = None
    position: Optional[List[float]] = None
    container_position: Optional[List[float]] = None
    ports: list[Port] = Field(default=[], validate_default=True)
    template: Optional[str] = None
    component_template: Optional[DynamicComponentTemplate] = None
    variant: str = BaseVariant.default
    libraries_data: Optional[AvailableLibraries] = None
    figures: List[NamedFigure] = Field(default=[])
    container_type: Optional[ContainerTypes] = None

    @model_validator(mode="before")
    @classmethod
    def validate_libraries_data(cls, value: Dict[str, Any]) -> Dict[str, Any]:
        libraries_data = AvailableLibraries.from_config(cls.__name__)
        if libraries_data:
            value["libraries_data"] = libraries_data

        parameter_class = param_from_config(cls.__name__)
        if parameter_class and isinstance(value, dict) and not value.get("parameters"):
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
            self.figures = [
                NamedFigure(**(fig.render_key(self).model_dump() | {"name": self.name}))
                for fig in library_data.figures
            ]

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

    def _get_target_compatible_port(
        self, target: "BaseElement", flow: Flow, ports_to_skip: List[Port]
    ) -> Optional["Port"]:

        target_ports = target.ports
        self_ports = self._ports(ports_to_skip)
        available_ports = _available_ports_with_targets(self_ports, target)
        available_ports_without_target = _available_ports_without_targets(self_ports)
        return find_port(
            flow, available_ports, available_ports_without_target, target_ports
        )



    def __hash__(self) -> int:
        return hash(f"{self.name}-{type(self).__name__}")


# This has to be here for now!!!
class Control(BaseElement):
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None
    container_annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.container_position) }},
    extent = {% raw %}{{5, -5}, {-5, 5}}
    {% endraw %})));"""
