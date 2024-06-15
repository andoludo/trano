from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Literal, Optional, Tuple

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, ConfigDict, Field, computed_field, create_model

from neosim.controller.parser import ControllerBus
from neosim.models.constants import Flow

if TYPE_CHECKING:
    from neosim.library.library import Libraries

Boolean = Literal["true", "false"]


class PartialConnection(BaseModel):
    equation: str
    position: List[float]


class AvailableLibraries(BaseModel):
    ideas: List[Callable[[], "LibraryData"]] = Field(default=[lambda: None])
    buildings: List[Callable[[], "LibraryData"]] = Field(default=[lambda: None])

    def get_library_data(
        self, library: "Libraries", variant: str
    ) -> Any:  # noqa: ANN401
        if variant == BaseVariant.default:
            return getattr(self, library.name.lower())[0]()
        selected_variant = [
            variant_
            for variant_ in getattr(self, library.name.lower())
            if variant_().variant == variant
        ]
        if not selected_variant:
            raise ValueError(f"Variant {variant} not found in library {library.name}")
        return selected_variant[0]()


class ConnectionView(BaseModel):
    color: str = "{255,204,51}"
    thickness: float = 0.5


class Connection(BaseModel):
    right: PartialConnection
    left: PartialConnection
    connection_view: ConnectionView = Field(default=ConnectionView())

    @property
    def path(self) -> List[List[float] | Tuple[float, float]]:
        if self.left.position[0] < self.right.position[0]:
            mid_path = (self.right.position[0] - self.left.position[0]) / 2
            return [
                self.left.position,
                (self.left.position[0] + mid_path, self.left.position[1]),
                (self.right.position[0] - mid_path, self.right.position[1]),
                self.right.position,
            ]

        else:
            mid_path = (self.left.position[0] - self.right.position[0]) / 2
            return [
                self.left.position,
                (self.left.position[0] - mid_path, self.left.position[1]),
                (self.right.position[0] + mid_path, self.right.position[1]),
                self.right.position,
            ]


class Port(BaseModel):
    names: list[str]
    targets: Optional[List[Any]] = None
    available: bool = True
    flow: Flow = Field(default=Flow.undirected)
    multi_connection: bool = False
    multi_object: bool = False
    bus_connection: bool = False
    use_counter: bool = True
    counter: int = Field(default=1)

    def is_available(self) -> bool:
        return self.multi_connection or self.available

    def is_controllable(self) -> bool:
        from neosim.models.elements.controls.base import Control

        return self.targets is not None and any(
            target == Control for target in self.targets
        )

    def link(
        self, node: "BaseElement", connected_node: "BaseElement"
    ) -> list[PartialConnection]:

        from neosim.models.elements.envelope.base import MergedBaseWall

        self.available = False
        partial_connections = []
        merged_number = 1
        if isinstance(node, MergedBaseWall):
            merged_number = len(node.surfaces)

        if isinstance(connected_node, MergedBaseWall):
            merged_number = len(connected_node.surfaces)
        for name in self.names:
            if self.multi_connection and self.bus_connection:
                first_counter = self.counter
                last_counter = self.counter + merged_number - 1
                if first_counter == last_counter:
                    counter = f"{first_counter}"
                else:
                    counter = f"{first_counter}:{last_counter}"
                if self.multi_object:
                    equation = f"{node.name}[{counter}].{name}"
                else:
                    equation = f"{node.name}.{name}[{counter}]"
                self.counter = last_counter + 1
            elif self.multi_connection and self.use_counter:
                equation = f"{node.name}.{name}[{self.counter}]"
                self.counter += 1
            else:
                equation = f"{node.name}.{name}"

            partial_connections.append(
                PartialConnection(equation=equation, position=node.position)
            )

        return partial_connections


class BaseVariant:
    default: str = "default"


DynamicTemplateCategories = Literal["ventilation", "control", "fluid", "boiler"]


class DynamicComponentTemplate(BaseModel):

    template: str
    category: Optional[DynamicTemplateCategories] = None
    function: Callable[[Any], Any] = Field(default=lambda _: {})
    bus: ControllerBus

    def render(
        self, package_name: str, element: "BaseElement", parameters: Dict[str, Any]
    ) -> str:
        ports = list(self.bus.bus_ports(element))
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(
                str(Path(__file__).parents[2].joinpath("templates"))
            ),
            autoescape=True,
        )
        environment.filters["enumerate"] = enumerate
        rtemplate = environment.from_string(
            "{% import 'macros.jinja2' as macros %}" + self.template
        )
        component = rtemplate.render(
            element=element,
            package_name=package_name,
            bus_template=self.bus.template,
            bus_ports="\n".join(ports),
            parameters=parameters,
            **self.function(element),
        )

        return component


class BaseParameter(BaseModel):
    model_config = ConfigDict(populate_by_name=True, extra="forbid")


class BaseElement(BaseModel):
    name: str
    annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.position) }},
    extent = {% raw %}{{-10, -10}, {10, 10}}
    {% endraw %})));"""
    model_config = ConfigDict(arbitrary_types_allowed=True, extra="forbid")
    parameters: Optional[BaseParameter] = None
    position: Optional[List[float]] = None
    ports: list[Port] = Field(default=[], validate_default=True)
    template: Optional[str] = None
    component_template: Optional[DynamicComponentTemplate] = None
    variant: str = BaseVariant.default
    libraries_data: Optional[AvailableLibraries] = None

    def assign_library_property(self, library: "Libraries") -> bool:
        if self.libraries_data is None:
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
        return True

    def processed_parameters(self, library: "Libraries") -> Any:  # noqa: ANN401
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

    @computed_field
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


def connection_color(edge: Tuple["BaseElement", "BaseElement"]) -> ConnectionView:
    from neosim.models.elements.envelope.base import BaseSimpleWall

    if any(isinstance(e, BaseSimpleWall) for e in edge):
        return ConnectionView(color="{191,0,0}", thickness=0.2)
    return ConnectionView()


def connect(edge: Tuple["BaseElement", "BaseElement"]) -> list[Connection]:
    connections = []
    edge_first = edge[0]
    edge_second = edge[1]
    edge_first_ports_to_skip: List[Port] = []
    edge_second_ports_to_skip: List[Port] = []
    while True:
        current_port = edge_first._get_target_compatible_port(
            edge_second, Flow.outlet, ports_to_skip=edge_first_ports_to_skip
        )
        other_port = edge_second._get_target_compatible_port(
            edge_first, Flow.inlet, ports_to_skip=edge_second_ports_to_skip
        )
        if any(port is None for port in [current_port, other_port]):
            break
        for left, right in zip(
            current_port.link(edge_first, edge_second),  # type: ignore
            other_port.link(edge_second, edge_first),  # type: ignore
        ):
            connections.append(
                Connection(
                    left=left, right=right, connection_view=connection_color(edge)
                )
            )
        edge_first_ports_to_skip.append(current_port)  # type: ignore
        edge_second_ports_to_skip.append(other_port)  # type: ignore
    return connections


def _has_inlet_or_outlet(target: "BaseElement") -> bool:
    return bool([port for port in target.ports if port.flow == Flow.inlet_or_outlet])


def _is_inlet_or_outlet(target: "BaseElement") -> bool:
    return bool(
        [port for port in target.ports if port.flow in [Flow.inlet, Flow.outlet]]
    )


def change_alias(
    parameter: BaseParameter, mapping: Optional[Dict[str, str]] = None
) -> Any:  # noqa: ANN401
    mapping = mapping or {}
    new_param = {}
    for name, field in parameter.model_fields.items():
        if mapping.get(name):
            field.alias = mapping[name]
        new_param[name] = (
            field.annotation,
            Field(field.default, alias=field.alias, description=field.description),
        )

    for name, field in parameter.model_computed_fields.items():  # type: ignore
        if mapping.get(name):
            new_param[name] = (
                Optional[field.return_type],  # type: ignore
                Field(None, alias=mapping[name], description=field.description),
            )
    return create_model(  # type: ignore
        __model_name="new_model",
        **new_param,
        __config__=ConfigDict(populate_by_name=True),
    )


def modify_alias(
    parameter: BaseParameter, mapping: Dict[str, str]
) -> Any:  # noqa: ANN401
    return change_alias(parameter, mapping)(**parameter.model_dump()).model_dump(
        by_alias=True, include=set(mapping)
    )


def exclude_parameters(
    parameters: BaseParameter, exclude_parameters: Optional[set[str]] = None
) -> Dict[str, Any]:
    return parameters.model_dump(by_alias=True, exclude=exclude_parameters)


def default_parameters(parameters: BaseParameter) -> Dict[str, Any]:
    if not parameters:
        return {}
    return parameters.model_dump(by_alias=True, exclude_none=True)


class LibraryData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    template: str = ""
    component_template: Optional[DynamicComponentTemplate] = None
    ports_factory: Callable[[], List[Port]]
    variant: str = BaseVariant.default
    parameter_processing: Callable[[BaseParameter], Dict[str, Any]] = default_parameters


class BaseBoundary(BaseElement):
    ...
