from typing import Any, ClassVar, Dict, List, Optional, Type

from pydantic import ConfigDict, Field, field_validator, model_validator

from trano.elements.common_base import BaseElementPosition
from trano.elements.components import DynamicComponentTemplate
from trano.elements.connection import Port
from trano.elements.figure import NamedFigure
from trano.elements.parameters import BaseParameter, param_from_config
from trano.elements.types import BaseVariant, Flow, ContainerTypes, Medium
from trano.library.library import AvailableLibraries, Library


class BaseElementPort(BaseElementPosition):
    name: Optional[str] = Field(default=None)
    ports: list[Port] = Field(default=[], validate_default=True)
    container_type: Optional[ContainerTypes] = None
    def available_ports(self) -> List[Port]:
        return [port for port in self.ports if not port.connected]

class BaseElement(BaseElementPort):
    name_counter: ClassVar[
        int
    ] = 0  # TODO: this needs to be removed and replaced with a proper solution.

    model_config = ConfigDict(arbitrary_types_allowed=True, extra="allow")
    parameters: Optional[BaseParameter] = None
    template: Optional[str] = None
    component_template: Optional[DynamicComponentTemplate] = None
    variant: str = BaseVariant.default
    libraries_data: Optional[AvailableLibraries] = None
    figures: List[NamedFigure] = Field(default=[])



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
        if self.position.is_empty():
            x, y= list(layout.get(self))  # type: ignore
            self.position.set_global(x, y)

    def get_controllable_ports(self) -> List[Port]:
        return [port for port in self.ports if port.is_controllable()]

    @property
    def type(self) -> str:
        return type(self).__name__

    def __hash__(self) -> int:
        return hash(f"{self.name}-{type(self).__name__}")


class ElementPort(BaseElementPort):
    element_type: Optional[Type[BaseElement]] = None
    merged_number: int = 1
    def has_target(self, targets: List[BaseElement]) -> bool:
        return (not targets) or (self.element_type is not None and targets and any(issubclass(self.element_type, t) for t in targets))
    def get_connection_per_target(self, target: Type[BaseElement]) -> int:
        return len([target_ for port in self.ports for target_ in port.targets if issubclass(target, target_)])







    @classmethod
    def from_element_without_ports(cls, element: BaseElement) -> "ElementPort":
        return cls.from_element(element, use_original_ports=False)


    @classmethod
    def from_element(cls, element: BaseElement, use_original_ports: bool = True) -> "ElementPort":
        from trano.elements.envelope import MergedBaseWall
        merged_number = 1
        if isinstance(element, MergedBaseWall):
            merged_number = len(element.surfaces)
        if element.position is not None:
            element_port = cls(**(element.model_dump() | {"element_type": type(element), "merged_number": merged_number}))
        else:
            element_port =  cls(**(element.model_dump(exclude={"position"}) | {"element_type": type(element), "merged_number": merged_number}))
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


