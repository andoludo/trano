import os
import re
import subprocess
from copy import deepcopy
from functools import partial
from pathlib import Path
from typing import (
    TYPE_CHECKING,
    Any,
    Callable,
    ClassVar,
    Dict,
    List,
    Literal,
    Optional,
    Tuple,
    Type,
)

import jinja2.exceptions
import yaml
from jinja2 import Environment, FileSystemLoader
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    create_model,
    field_validator,
    model_validator,
)
from pydantic.fields import computed_field

from trano.controller.parser import ControllerBus
from trano.models.elements.types import Flow

if TYPE_CHECKING:
    from trano.library.library import Libraries

Boolean = Literal["true", "false"]


class Line(BaseModel):
    template: str
    key: Optional[str] = None
    color: str = "grey"
    label: str
    line_style: str = "solid"
    line_width: float = 1.5


class Axis(BaseModel):
    lines: List[Line] = Field(default=[])
    label: str


class Figure(BaseModel):
    right_axis: Axis = Field(default=Axis(lines=[], label=""))
    left_axis: Axis = Field(default=Axis(lines=[], label=""))

    def render_key(self, element: "BaseElement") -> "Figure":
        environment = Environment(autoescape=True)
        for axis in self.right_axis.lines + self.left_axis.lines:
            template = environment.from_string(axis.template)
            try:
                axis.key = template.render(element=element)
            except jinja2.exceptions.UndefinedError:
                continue

        return self


class PartialConnection(BaseModel):
    equation: str
    position: List[float]


def convert_copy(schema: Path, input_file: Path, target: str, output: Path) -> bool:
    root_path = Path(__file__).parents[3]
    os.chdir(root_path)
    command = [
        "poetry",
        "run",
        "linkml-convert",
        "-o",
        f"{output}",
        "-t",
        target,
        "-C",
        "Building",
        "-s",
        str(schema),
        f"{input_file}",
    ]

    process = subprocess.run(
        command, check=True, capture_output=True, text=True  # noqa: S603
    )
    return process.returncode == 0


def to_snake_case(name: str) -> str:
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def compose_func(ports_: List["Port"]) -> Callable[[], List["Port"]]:
    return lambda: ports_


def _load_components() -> Dict[str, Any]:
    libraries_path = Path(__file__).parent.joinpath("models")
    data: Dict[str, Any] = {"components": []}
    for file in libraries_path.glob("*.yaml"):
        data["components"] += yaml.safe_load(file.read_text()).get("components", [])
    return data


COMPONENTS = _load_components()


class AvailableLibraries(BaseModel):
    ideas: List[Callable[[], "LibraryData"]] = Field(default=[lambda: None])
    buildings: List[Callable[[], "LibraryData"]] = Field(default=[lambda: None])

    def get_library_data(
        self, library: "Libraries", variant: str
    ) -> Any:  # noqa: ANN401
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
                    globals()[function_name],
                    **{
                        function_name: component["parameter_processing"].get(
                            "parameter", {}
                        )
                    },
                )
            else:
                parameter_processing = globals()[
                    component["parameter_processing"]["function"]
                ]
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


class ConnectionView(BaseModel):
    color: Optional[str] = "{255,204,51}"
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

    @field_validator("targets")
    @classmethod
    def validate_targets(  # noqa: PLR0915, PLR0912, C901
        cls, values: List[str]
    ) -> List[Type["BaseElement"]]:  # TODO: reduce complexity
        # TODO: this function to be refactored!!!
        targets: List[Type[BaseElement]] = []
        for value in values:
            if isinstance(value, str):
                if value == "DataBus":
                    from trano.models.elements.bus import DataBus

                    targets.append(DataBus)
                elif value == "Control":

                    targets.append(Control)
                elif value == "Space":
                    from trano.models.elements.space import Space

                    targets.append(Space)
                elif value == "BaseBoundary":
                    targets.append(BaseBoundary)
                elif value == "System":
                    from trano.models.elements.system import System

                    targets.append(System)
                elif value == "AhuControl":
                    from trano.models.elements.control import AhuControl

                    targets.append(AhuControl)
                elif value == "BaseWeather":
                    from trano.models.elements.system import BaseWeather

                    targets.append(BaseWeather)
                elif value == "AirHandlingUnit":
                    from trano.models.elements.system import AirHandlingUnit

                    targets.append(AirHandlingUnit)
                elif value == "Ventilation":
                    from trano.models.elements.system import Ventilation

                    targets.append(Ventilation)
                elif value == "BaseInternalElement":
                    from trano.models.elements.envelope import BaseInternalElement

                    targets.append(BaseInternalElement)
                elif value == "BaseOccupancy":
                    from trano.models.elements.system import BaseOccupancy

                    targets.append(BaseOccupancy)
                elif value == "Emission":
                    from trano.models.elements.system import Emission

                    targets.append(Emission)
                elif value == "VAVControl":
                    from trano.models.elements.control import VAVControl

                    targets.append(VAVControl)
                elif value == "BaseWall":
                    from trano.models.elements.envelope import BaseWall

                    targets.append(BaseWall)
                elif value == "ThreeWayValve":
                    from trano.models.elements.system import ThreeWayValve

                    targets.append(ThreeWayValve)
                elif value == "TemperatureSensor":
                    from trano.models.elements.system import TemperatureSensor

                    targets.append(TemperatureSensor)
                elif value == "Boundary":
                    from trano.models.elements.boundary import Boundary

                    targets.append(Boundary)
                else:
                    raise ValueError(f"Target {value} not found")
            else:
                targets.append(value)
        return targets

    def is_available(self) -> bool:
        return self.multi_connection or self.available

    def is_controllable(self) -> bool:

        return self.targets is not None and any(
            target == Control for target in self.targets
        )

    def link(
        self, node: "BaseElement", connected_node: "BaseElement"
    ) -> list[PartialConnection]:

        from trano.models.elements.envelope import MergedBaseWall

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


def _get_type(_type: Any) -> Any:  # noqa: ANN401
    if _type == "string":
        return str
    elif _type == "float":
        return float
    elif _type == "integer":
        return int
    elif _type == "boolean":
        return bool
    else:
        raise Exception("Unknown type")


class BaseParameter(BaseModel):
    model_config = ConfigDict(populate_by_name=True, extra="allow")


def _get_default(v: Any) -> Any:  # noqa: ANN401
    if "ifabsent" not in v:
        return None
    tag = v["range"]
    if tag == "integer":
        tag = "int"
    value = v["ifabsent"].replace(tag, "")[1:-1]
    if value == "None":
        return None

    try:
        return _get_type(v["range"])(value)
    except Exception as e:

        raise e


def load_parameters() -> Dict[str, Type["BaseParameter"]]:
    # TODO: remove absoluth path reference
    parameter_path = (
        Path(__file__).parents[2].joinpath("data_models", "parameters.yaml")
    )
    data = yaml.safe_load(parameter_path.read_text())
    classes = {}

    for name, parameter in data.items():
        attrib_ = {}
        for k, v in parameter["attributes"].items():
            alias = v.get("alias", None)
            alias = alias if alias != "None" else None
            if v.get("range"):
                attrib_[k] = (
                    _get_type(v["range"]),
                    Field(
                        default=_get_default(v),
                        alias=alias,
                        description=v.get("description", None),
                    ),
                )
            else:
                attrib_[k] = computed_field(  # type: ignore # TODO: why?
                    eval(v["func"]),  # noqa: S307
                    return_type=eval(v["type"]),  # noqa: S307
                    alias=alias,  # TODO: avoid using eval
                )
        model = create_model(f"{name}_", __base__=BaseParameter, **attrib_)  # type: ignore # TODO: why?
        for class_ in parameter["classes"]:
            classes[class_] = model
    return classes


PARAMETERS = load_parameters()


def param_from_config(name: str) -> Optional[Type[BaseParameter]]:
    return PARAMETERS.get(name)


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

    def assign_library_property(self, library: "Libraries") -> bool:
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


def connection_color(edge: Tuple["BaseElement", "BaseElement"]) -> ConnectionView:
    from trano.models.elements.bus import DataBus
    from trano.models.elements.envelope import BaseSimpleWall
    from trano.models.elements.system import Weather

    if any(isinstance(e, BaseSimpleWall) for e in edge):
        return ConnectionView(color="{191,0,0}", thickness=0.1)
    if any(isinstance(e, DataBus) for e in edge):
        return ConnectionView(color=None, thickness=0.05)
    if any(isinstance(e, Weather) for e in edge):
        return ConnectionView(color=None, thickness=0.05)
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
        "new_model",
        **new_param,
        __config__=ConfigDict(populate_by_name=True),
    )


def modify_alias(
    parameter: BaseParameter, modify_alias: Dict[str, str]
) -> Any:  # noqa: ANN401
    return change_alias(parameter, modify_alias)(**parameter.model_dump()).model_dump(
        by_alias=True, include=set(modify_alias)
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
    figures: List[Figure] = Field(default=[])


class BaseBoundary(BaseElement):
    ...


class Control(BaseElement):
    position: Optional[List[float]] = None
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None
