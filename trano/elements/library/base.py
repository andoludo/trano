from functools import partial
from pathlib import Path
from typing import Optional, Callable, Any, Dict, List, TYPE_CHECKING

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
    category: Optional[DynamicTemplateCategories] = None
    function: Callable[[Any], Any] = Field(default=lambda _: {})
    bus: ControllerBus

    def _has_required_attributes(self, element: "BaseElement") -> None:
        for target in self.bus.main_targets():
            if "element" not in target:
                raise ValueError(
                    f"Target {target} should start with the word 'element'"
                )
            attributes = target.split(".")[1:]
            for attr in attributes:
                if not hasattr(element, attr):
                    raise ValueError(
                        f"Element {element} does not have attribute {attr}"
                    )
                element = getattr(element, attr)

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


class LibraryData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    template: str = ""
    component_template: Optional[DynamicComponentTemplate] = None
    variant: str = BaseVariant.default
    figures: List[Figure] = Field(default=[])
    ports: Callable[[], List[Port]]
    parameter_processing: Callable[[BaseParameter], Dict[str, Any]] = default_parameters
    library: str
    classes: List[str]

    @field_validator("parameter_processing", mode="before")
    @classmethod
    def _parameters_processing_validator(
        cls, value: Dict[str, Any]
    ) -> Callable[[BaseParameter], Dict[str, Any]]:

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
    def _ports_factory_validator(
        cls, value: List[Dict[str, Any]]
    ) -> Callable[[], List[Port]]:
        return compose_func([Port(**port) for port in value])

    def json_file_name(self) -> str:
        return json_component_name(self.classes)
