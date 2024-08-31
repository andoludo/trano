from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, Optional

import yaml
from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, Field

from trano.elements.controller_bus import ControllerBus
from trano.elements.types import DynamicTemplateCategories

if TYPE_CHECKING:
    from trano.elements.base import BaseElement


def _load_components() -> Dict[str, Any]:
    libraries_path = Path(__file__).parent.joinpath("models")
    data: Dict[str, Any] = {"components": []}
    for file in libraries_path.glob("*.yaml"):
        data["components"] += yaml.safe_load(file.read_text()).get("components", [])
    return data


COMPONENTS = _load_components()


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
                str(Path(__file__).parents[1].joinpath("templates"))
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
