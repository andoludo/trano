from typing import TYPE_CHECKING, NamedTuple

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, ConfigDict, Field, field_validator, computed_field

from trano.elements.common_base import BaseProperties
from trano.elements.jinja import compile_template
from trano.elements.types import ContainerTypes

if TYPE_CHECKING:
    from trano.elements.library.library import Library


class Material(BaseModel):
    model_config = ConfigDict(populate_by_name=True)
    name: str
    thermal_conductivity: float = Field(..., title="Thermal conductivity [W/(m.K)]", alias="k")
    specific_heat_capacity: float = Field(..., title="Specific thermal capacity [J/(kg.K)]", alias="c")
    density: float = Field(..., title="Density [kg/m3]", alias="rho")
    longwave_emissivity: float = Field(0.85, title="Longwave emissivity [1]", alias="epsLw")
    shortwave_emissivity: float = Field(0.65, title="Shortwave emissivity [1]", alias="epsSw")

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


class StandardGas(NamedTuple):
    molar_mass: float  # [kg/mol]
    a_mu: float  # [N.s/m2]
    b_mu: float  # [N.s/(m2.K)]


# Viscosity coefficients and molar masses of Buildings.HeatTransfer.Data.Gases (ISO 15099).
STANDARD_GASES = (
    StandardGas(molar_mass=28.97e-3, a_mu=3.723e-6, b_mu=4.940e-8),  # Air
    StandardGas(molar_mass=39.948e-3, a_mu=3.379e-6, b_mu=6.451e-8),  # Argon
    StandardGas(molar_mass=83.80e-3, a_mu=2.213e-6, b_mu=7.777e-8),  # Krypton
    StandardGas(molar_mass=131.3e-3, a_mu=1.069e-6, b_mu=7.414e-8),  # Xenon
)
GAS_REFERENCE_TEMPERATURE = 293.15  # [K]
GAS_REFERENCE_PRESSURE = 101325.0  # [Pa]
UNIVERSAL_GAS_CONSTANT = 8.314462618  # [J/(mol.K)]


class Gas(Material):
    @property
    def molar_mass(self) -> float:
        """Molar mass [kg/mol] giving the gas density at 20 °C and 1 atm (ideal gas law)."""
        return self.density * UNIVERSAL_GAS_CONSTANT * GAS_REFERENCE_TEMPERATURE / GAS_REFERENCE_PRESSURE

    @property
    def viscosity_coefficients(self) -> tuple[float, float]:
        """Viscosity coefficients (a_mu, b_mu) of the standard gas with the closest molar mass.

        The viscosity is not part of the gas description, so it is taken from the standard gas it resembles most.
        """
        gas = min(STANDARD_GASES, key=lambda standard_gas: abs(standard_gas.molar_mass - self.molar_mass))
        return gas.a_mu, gas.b_mu


class Layer(BaseModel):
    material: Material
    thickness: float

    @computed_field  # type: ignore
    @property
    def thermal_resistance(self) -> float:
        return self.thickness / self.material.thermal_conductivity

    @computed_field  # type: ignore
    @property
    def thermal_capacitance(self) -> float:
        return self.thickness * self.material.specific_heat_capacity * self.material.density


# TODO: Add units
class BaseConstruction(BaseModel):
    layers: list[Layer]

    @computed_field  # type: ignore
    @property
    def total_thermal_resistance(self) -> float:
        return sum([layer.thermal_resistance for layer in self.layers])

    @computed_field
    def total_thermal_capacitance(self) -> float:
        return sum([layer.thermal_capacitance for layer in self.layers])

    @computed_field
    def u_value(self) -> float:
        if not self.total_thermal_resistance:
            return 0.0
        return 1.0 / self.total_thermal_resistance

    @computed_field
    def resistance_external(self) -> float:
        return self.total_thermal_resistance / 2.0

    @computed_field
    def resistance_external_remaining(self) -> float:
        return self.total_thermal_resistance / 2.0


class Construction(BaseConstruction):
    name: str

    def __hash__(self) -> int:
        return hash(self.name)

    @field_validator("name")
    @classmethod
    def clean_name(cls, value: str) -> str:
        if ":" in value:
            return value.lower().replace(":", "_")
        return value


class GlassLayer(Layer):
    thickness: float
    material: GlassMaterial
    layer_type: str = "glass"


class GasLayer(Layer):
    model_config = ConfigDict(use_enum_values=True)
    thickness: float
    material: Gas
    layer_type: str = "gas"


class Glass(BaseConstruction):
    name: str
    layers: list[GlassLayer | GasLayer]  # type: ignore
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
    template: str | None = None
    constructions: list[Construction | Material | Glass]


class ConstructionData(BaseModel):
    constructions: list[Construction]
    materials: list[Material]
    glazing: list[Glass]


class BaseConstructionData(BaseModel):
    template: str
    construction: BaseData
    material: BaseData
    glazing: BaseData

    def generate_data(self, package_name: str) -> str:
        models: dict[str, list[str]] = {
            "material": [],
            "construction": [],
            "glazing": [],
        }
        for construction_type_name, rendered_models in models.items():
            construction_type = getattr(self, construction_type_name)
            for construction in construction_type.constructions:
                template = compile_template("{% import 'macros.jinja2' as macros %}" + construction_type.template)
                model = template.render(construction=construction, package_name=package_name)
                rendered_models.append(model)
        template = compile_template("{% import 'macros.jinja2' as macros %}" + self.template)
        model = template.render(**models, package_name=package_name)
        return model


class MaterialProperties(BaseProperties):
    container_type: ContainerTypes = "envelope"


class BaseTemplateData(BaseModel):
    template: str | None = None
    constructions: list[Construction | Material | Glass]


def default_construction(nodes: NodeView) -> ConstructionData:
    from trano.elements.envelope import BaseSimpleWall

    constructions = {node.construction for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]}
    wall_constructions = sorted([c for c in constructions if isinstance(c, Construction)], key=lambda x: x.name)
    glazing = sorted([c for c in constructions if isinstance(c, Glass)], key=lambda x: x.name)
    return ConstructionData(constructions=wall_constructions, materials=[], glazing=glazing)


def merged_construction(nodes: NodeView) -> ConstructionData:
    # TODO: Fix the import
    from trano.elements.envelope import BaseSimpleWall, MergedBaseWall

    merged_constructions = {
        construction
        for node in [node_ for node_ in nodes if isinstance(node_, MergedBaseWall)]
        for construction in node.constructions
    }
    constructions = {node.construction for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]}
    merged_constructions.update(constructions)
    wall_constructions = [c for c in merged_constructions if isinstance(c, Construction)]
    glazing = [c for c in merged_constructions if isinstance(c, Glass)]
    materials = {layer.material for construction in merged_constructions for layer in construction.layers}
    return ConstructionData(constructions=wall_constructions, materials=list(materials), glazing=glazing)


def extract_data(package_name: str, nodes: NodeView, library: "Library") -> MaterialProperties:
    data = merged_construction(nodes) if library.merged_external_boundaries else default_construction(nodes)
    data_ = BaseConstructionData(
        template=library.templates.main,
        construction=BaseData(constructions=data.constructions, template=library.templates.construction),
        glazing=BaseData(constructions=data.glazing, template=library.templates.glazing),
        material=BaseData(constructions=data.materials, template=library.templates.material),
    )
    return MaterialProperties(data=data_.generate_data(package_name), is_package=library.templates.is_package)


def extract_properties(library: "Library", package_name: str, nodes: NodeView) -> MaterialProperties:
    return extract_data(package_name, nodes, library)
