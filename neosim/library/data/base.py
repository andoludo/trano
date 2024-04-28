from pathlib import Path
from typing import Dict, List, Optional, Union

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel

from neosim.construction import Construction
from neosim.glass import Glass
from neosim.material import Material


class BaseData(BaseModel):
    template: Optional[str] = None
    constructions: List[Union[Construction, Material, Glass]]


class BaseConstructionData(BaseModel):
    template: str
    construction: BaseData
    material: BaseData
    glazing: BaseData

    def generate_data(self, package_name: str) -> str:
        environment = Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            loader=FileSystemLoader(
                str(Path(__file__).parents[2].joinpath("templates"))
            ),
            autoescape=True,
        )
        models: Dict[str, List[str]] = {
            "material": [],
            "construction": [],
            "glazing": [],
        }
        for construction_type_name in models:
            construction_type = getattr(self, construction_type_name)
            for construction in construction_type.constructions:
                template = environment.from_string(
                    "{% import 'macros.jinja2' as macros %}"
                    + construction_type.template
                )
                model = template.render(
                    construction=construction, package_name=package_name
                )
                models[construction_type_name].append(model)
        template = environment.from_string(
            "{% import 'macros.jinja2' as macros %}" + self.template
        )
        model = template.render(**models, package_name=package_name)
        return model
