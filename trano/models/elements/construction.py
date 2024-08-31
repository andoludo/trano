from pathlib import Path
from typing import Optional, List, Union, Dict

from jinja2 import Environment, FileSystemLoader
from pydantic import BaseModel, field_validator, ConfigDict, Field



class Material(BaseModel):
    model_config = ConfigDict(populate_by_name=True)
    name: str
    thermal_conductivity: float = Field(
        ..., title="Thermal conductivity [W/(m.K)]", alias="k"
    )
    specific_heat_capacity: float = Field(
        ..., title="Specific thermal capacity [J/(kg.K)]", alias="c"
    )
    density: float = Field(..., title="Density [kg/m3]", alias="rho")
    longwave_emissivity: float = Field(
        0.85, title="Longwave emissivity [1]", alias="epsLw"
    )
    shortwave_emissivity: float = Field(
        0.65, title="Shortwave emissivity [1]", alias="epsSw"
    )

    def __hash__(self) -> int:
        return hash(self.name)

    @field_validator("name")
    @classmethod
    def clean_name(cls, value: str) -> str:
        if ":" in value:
            return value.lower().replace(":", "_")
        return value


class GlassMaterial(Material):
    solar_transmittance: list[float]
    solar_reflectance_outside_facing: list[float]
    solar_reflectance_room_facing: list[float]
    infrared_transmissivity: float
    infrared_absorptivity_outside_facing: float
    infrared_absorptivity_room_facing: float


class Gas(Material):
    ...


class Layer(BaseModel):
    material: Material
    thickness: float

class Construction(BaseModel):
    name: str
    layers: list[Layer]

    def __hash__(self) -> int:
        return hash(self.name)

    @field_validator("name")
    @classmethod
    def clean_name(cls, value: str) -> str:
        if ":" in value:
            return value.lower().replace(":", "_")
        return value


class GlassLayer(BaseModel):
    thickness: float
    material: GlassMaterial
    layer_type: str = "glass"


class GasLayer(BaseModel):
    model_config = ConfigDict(use_enum_values=True)
    thickness: float
    material: Gas
    layer_type: str = "gas"


class Glass(BaseModel):
    name: str
    layers: list[GlassLayer | GasLayer]
    u_value_frame: float

    def __hash__(self) -> int:
        return hash(self.name)

    @field_validator("name")
    @classmethod
    def clean_name(cls, value: str) -> str:
        if ":" in value:
            return value.lower().replace(":", "_")
        return value


class BaseData(BaseModel):
    template: Optional[str] = None
    constructions: List[Union[Construction, Material, Glass]]


class ConstructionData(BaseModel):
    constructions: List[Construction]
    materials: List[Material]
    glazing: List[Glass]


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


class MaterialProperties(BaseModel):
    data: str
    is_package: bool
