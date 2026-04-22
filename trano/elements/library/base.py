from functools import partial
from pathlib import Path
from typing import Any, TYPE_CHECKING
from collections.abc import Callable

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, Field, ConfigDict, field_validator

from trano.elements.common_base import BaseParameter
from trano.elements.connection import Port
from trano.elements.data_bus.controller_bus import ControllerBus
from trano.elements.figure import Figure
from trano.elements.library import parameters
from trano.elements.library.parameters import default_parameters
from trano.elements.types import BaseVariant, DynamicTemplateCategories
from trano.elements.utils import compose_func, json_component_name

if TYPE_CHECKING:
    from trano.elements.base import BaseElement


class DynamicComponentTemplate(BaseModel):
    template: str
    category: DynamicTemplateCategories | None = None
    function: Callable[[Any], Any] = Field(default=lambda _: {})
    bus: ControllerBus

    def _has_required_attributes(self, element: "BaseElement") -> None:
        for target in self.bus.main_targets():
            if "element" not in target:
                raise ValueError(f"Target {target} should start with the word 'element'")
            attributes = target.split(".")[1:]
            for attr in attributes:
                if not hasattr(element, attr):
                    raise ValueError(f"Element {element} does not have attribute {attr}")
                element = getattr(element, attr)

    def render(self, package_name: str, element: "BaseElement", parameters: dict[str, Any]) -> str:
        ports = list(self.bus.bus_ports(element))
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(str(Path(__file__).parents[2].joinpath("templates"))),
            autoescape=True,
        )
        environment.filters["enumerate"] = enumerate
        rtemplate = environment.from_string("{% import 'macros.jinja2' as macros %}" + self.template)
        component = rtemplate.render(
            element=element,
            package_name=package_name,
            bus_template=self.bus.template,
            bus_ports="\n".join(ports),
            parameters=parameters,
            **self.function(element),
        )

        return component


class LibraryData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    template: str = ""
    component_template: DynamicComponentTemplate | None = None
    variant: str = BaseVariant.default
    figures: list[Figure] = Field(default=[])
    ports: Callable[[], list[Port]]
    parameter_processing: Callable[[BaseParameter], dict[str, Any]] = default_parameters
    library: str
    classes: list[str]

    @field_validator("parameter_processing", mode="before")
    @classmethod
    def _parameters_processing_validator(cls, value: dict[str, Any]) -> Callable[[BaseParameter], dict[str, Any]]:
        if value.get("parameter"):
            function_name = value["function"]
            parameter_processing = partial(
                getattr(parameters, function_name),
                **{function_name: value.get("parameter", {})},
            )
        else:
            parameter_processing = getattr(parameters, value["function"])
        return parameter_processing

    @field_validator("ports", mode="before")
    @classmethod
    def _ports_factory_validator(cls, value: list[dict[str, Any]]) -> Callable[[], list[Port]]:
        return compose_func([Port(**port) for port in value])

    def json_file_name(self) -> str:
        return json_component_name(self.classes)
