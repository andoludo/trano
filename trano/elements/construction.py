from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Union

from jinja2 import Environment, FileSystemLoader
from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, ConfigDict, Field, field_validator

if TYPE_CHECKING:

    from trano.library.library import Library


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
                str(Path(__file__).parents[1].joinpath("templates"))
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


class BaseTemplateData(BaseModel):
    template: Optional[str] = None
    constructions: List[Union[Construction, Material, Glass]]


def default_construction(nodes: NodeView) -> ConstructionData:
    from trano.elements.envelope import BaseSimpleWall

    constructions = {
        node.construction
        for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
    }
    wall_constructions = sorted(
        [c for c in constructions if isinstance(c, Construction)], key=lambda x: x.name
    )
    glazing = sorted(
        [c for c in constructions if isinstance(c, Glass)], key=lambda x: x.name
    )
    return ConstructionData(
        constructions=wall_constructions, materials=[], glazing=glazing
    )


def merged_construction(nodes: NodeView) -> ConstructionData:
    # TODO: Fix the import
    from trano.elements.envelope import BaseSimpleWall, MergedBaseWall

    merged_constructions = {
        construction
        for node in [node_ for node_ in nodes if isinstance(node_, MergedBaseWall)]
        for construction in node.constructions
    }
    constructions = {
        node.construction
        for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]
    }
    merged_constructions.update(constructions)
    wall_constructions = [
        c for c in merged_constructions if isinstance(c, Construction)
    ]
    glazing = [c for c in merged_constructions if isinstance(c, Glass)]
    materials = {
        layer.material
        for construction in merged_constructions
        for layer in construction.layers
    }
    return ConstructionData(
        constructions=wall_constructions, materials=list(materials), glazing=glazing
    )


def extract_data(
    package_name: str, nodes: NodeView, library: "Library"
) -> MaterialProperties:

    if library.merged_external_boundaries:
        data = merged_construction(nodes)
    else:
        data = default_construction(nodes)
    data_ = BaseConstructionData(
        template=library.templates.main,
        construction=BaseData(
            constructions=data.constructions, template=library.templates.construction
        ),
        glazing=BaseData(
            constructions=data.glazing, template=library.templates.glazing
        ),
        material=BaseData(
            constructions=data.materials, template=library.templates.material
        ),
    )
    return MaterialProperties(
        data=data_.generate_data(package_name), is_package=library.templates.is_package
    )


def extract_properties(
    library: "Library", package_name: str, nodes: NodeView
) -> MaterialProperties:
    return extract_data(package_name, nodes, library)