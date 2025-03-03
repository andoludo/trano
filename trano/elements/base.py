from functools import cache
from pathlib import Path
from typing import Any, ClassVar, Dict, List, Optional, Type, TYPE_CHECKING, get_args

from jinja2 import Environment, FileSystemLoader
from pydantic import (
    ConfigDict,
    Field,
    field_validator,
    model_validator,
    PrivateAttr,
)

from trano.elements.common_base import (
    BaseElementPosition,
    ComponentModel,
    MediumTemplate,
)
from trano.elements.common_base import BaseParameter
from trano.elements.connection import Port
from trano.elements.figure import NamedFigure
from trano.elements.library.base import DynamicComponentTemplate, LibraryData
from trano.elements.library.parameters import param_from_config
from trano.elements.types import BaseVariant, ContainerTypes
from trano.exceptions import UnknownComponentVariantError

if TYPE_CHECKING:
    from trano.topology import Network
    from trano.elements.library.library import Library


class BaseElementPort(BaseElementPosition):
    name: Optional[str] = Field(default=None)
    ports: list[Port] = Field(default=[], validate_default=True)
    container_type: Optional[ContainerTypes] = None

    def available_ports(self) -> List[Port]:
        return [port for port in self.ports if not port.connected]

    def reset_port_counters(self) -> None:
        for port in self.ports:
            port.reset_counter()


def default_environment() -> Environment:
    environment = Environment(
        trim_blocks=True,
        lstrip_blocks=True,
        loader=FileSystemLoader(str(Path(__file__).parents[1].joinpath("templates"))),
        autoescape=True,
    )
    environment.filters["enumerate"] = enumerate
    return environment


class BaseElement(BaseElementPort):
    name_counter: ClassVar[int] = (
        0  # TODO: this needs to be removed and replaced with a proper solution.
    )

    model_config = ConfigDict(arbitrary_types_allowed=True, extra="allow")
    medium: MediumTemplate = Field(default_factory=MediumTemplate)
    parameters: Optional[BaseParameter] = None
    template: Optional[str] = None
    component_template: Optional[DynamicComponentTemplate] = None
    variant: str = BaseVariant.default
    libraries_data: List[LibraryData] = Field(default=[])
    figures: List[NamedFigure] = Field(default=[])
    _environment: Environment = PrivateAttr(default_factory=default_environment)
    component_model: Optional[ComponentModel] = None

    @model_validator(mode="before")
    @classmethod
    def validate_libraries_data(cls, value: Dict[str, Any]) -> Dict[str, Any]:
        if isinstance(value, dict):
            if "libraries_data" not in value:
                from trano.elements.library.components import COMPONENTS

                value["libraries_data"] = COMPONENTS.get_components(cls.__name__)
            parameter_class = param_from_config(cls.__name__)
            if (
                parameter_class
                and isinstance(value, dict)
                and not value.get("parameters")
            ):
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

    def get_library_data(self, library: "Library") -> Optional[LibraryData]:
        libraries_data_variants = [
            library_
            for library_ in self.libraries_data
            if library_.variant == self.variant
        ]
        if not libraries_data_variants:
            raise UnknownComponentVariantError(
                f"Library data not found for {self.name} in {library.name} for variant {self.variant}"
            )
        libraries_data = [
            library_
            for library_ in libraries_data_variants
            if (library_.library == library.name.lower())
        ]
        if not libraries_data:
            libraries_data = [
                library_
                for library_ in libraries_data_variants
                if (library_.library == "default")
            ]
        if libraries_data:
            return libraries_data[0]
        return None

    def assign_library_property(self, library: "Library") -> bool:
        if library.medium.is_empty():
            raise ValueError("Library medium is empty")
        if not self.libraries_data:
            return False
        library_data = self.get_library_data(library)
        if self.medium.is_empty():
            self.medium = library.medium
        if not library_data:
            return False
        if not self.ports:
            self.ports = library_data.ports()
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
            library_data = self.get_library_data(library)
            if library_data and self.parameters:
                return library_data.parameter_processing(self.parameters)
        return {}

    def set_position(self, layout: Dict[str, Any], global_: bool = True) -> None:
        if (
            self.position.is_global_empty()
            if global_
            else self.position.is_container_empty()
        ):
            x, y = list(layout.get(self.name))  # type: ignore
            (
                self.position.set_global(x, y)
                if global_
                else self.position.set_container(x, y)
            )

    def get_controllable_ports(self) -> List[Port]:
        return [port for port in self.ports if port.is_controllable()]

    @property
    def type(self) -> str:
        return type(self).__name__

    @cache  # noqa: B019
    def model(self, network: "Network") -> Optional[ComponentModel]:
        if not self.template:
            return None
        self._environment.globals.update(network.library.functions)
        if self.component_template:
            component = self.component_template.render(
                network.name, self, self.processed_parameters(network.library)
            )
            if self.component_template.category:
                network.dynamic_components[self.component_template.category].append(
                    component
                )
        package_name = network.name
        library_name = network.library.base_library()
        parameters = self.processed_parameters(network.library)
        component_model: Dict[str, Any] = {"id": hash(self)}
        for model_type, annotation in {
            "model": self.position.global_.annotation,
            "container": self.position.container.annotation,
        }.items():
            template = self._environment.from_string(
                "{% import 'macros.jinja2' as macros %}"
                + self.template
                + " "
                + annotation
            )
            component_model.update(
                {
                    model_type: template.render(
                        element=self,
                        package_name=package_name,
                        library_name=library_name,
                        parameters=parameters,
                    )
                }
            )
        component_model_ = ComponentModel.model_validate(component_model)
        self.component_model = component_model_
        return component_model_

    def __hash__(self) -> int:
        return hash(f"{self.name}-{type(self).__name__}")

    def add_to_network(self, network: "Network") -> None:
        network.add_node(self)

    def processing(self, network: "Network") -> None: ...

    def configure(self, network: "Network") -> None: ...
    def system_ports_connected(self) -> bool:
        return True

    def non_connected_ports_name(self) -> List[str]:
        return [
            f"{self.name}.{n}"
            for port in self.ports
            if not port.connected
            for n in port.names
        ]

    def assign_container_type(self, network: "Network") -> None:
        if self.container_type is None:
            network.graph.neighbors(self)
            container_types = {
                s.container_type
                for s in list(network.graph.predecessors(self))  # type: ignore
                + list(network.graph.successors(self))  # type: ignore
                if s.container_type
            }
            self.container_type = next(
                iter(
                    sorted(
                        container_types,
                        key=lambda type_: {v: i for i, v in enumerate(get_args(ContainerTypes))}.get(type_),  # type: ignore
                    )
                )
            )


class ElementPort(BaseElementPort):
    element_type: Optional[Type[BaseElement]] = None
    merged_number: int = 1

    def has_target(self, targets: List[Type[BaseElement]]) -> bool:
        return (not targets) or bool(
            self.element_type is not None
            and targets
            and any(issubclass(self.element_type, t) for t in targets)
        )

    def get_connection_per_target(
        self, target: Optional[Type[BaseElement]] = None
    ) -> int:
        if target is None:
            return 0
        return len(
            [
                target_
                for port in self.ports
                for target_ in port.targets
                if issubclass(target, target_)
            ]
        )

    @classmethod
    def from_element_without_ports(cls, element: BaseElement) -> "ElementPort":
        return cls.from_element(element, use_original_ports=False)

    @classmethod
    def from_element(
        cls, element: BaseElement, use_original_ports: bool = True
    ) -> "ElementPort":
        from trano.elements.envelope import MergedBaseWall

        merged_number = 1
        if isinstance(element, MergedBaseWall):
            merged_number = len(element.surfaces)
        if element.position is not None:
            element_port = cls(
                **(
                    element.model_dump()
                    | {"element_type": type(element), "merged_number": merged_number}
                )
            )
        else:
            element_port = cls(
                **(
                    element.model_dump(exclude={"position"})
                    | {"element_type": type(element), "merged_number": merged_number}
                )
            )
        if use_original_ports:
            element_port.ports = element.ports
        return element_port


# This has to be here for now!!!
class Control(BaseElement):
    controllable_element: Optional[BaseElement] = None
    space_name: Optional[str] = None
    container_annotation_template: str = """annotation (
    Placement(transformation(origin = {{ macros.join_list(element.container_position) }},
    extent = {% raw %}{{5, -5}, {-5, 5}}
    {% endraw %})));"""
