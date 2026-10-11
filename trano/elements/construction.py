from dataclasses import replace
from functools import cached_property
from typing import TYPE_CHECKING, NamedTuple

from networkx.classes.reportviews import NodeView
from pydantic import BaseModel, ConfigDict, Field, computed_field, field_validator, model_validator

from trano.elements.common_base import BaseProperties
from trano.elements.glazing import GlazingProperties
from trano.elements.jinja import compile_template
from trano.elements.types import ContainerTypes

if TYPE_CHECKING:
    from trano.elements.library.library import Library


EFFECTIVE_DEPTH = 0.1  # [m] depth of the layers taking part in the daily heat storage (ISO 13786)


class Material(BaseModel):
    model_config = ConfigDict(populate_by_name=True)
    name: str
    thermal_conductivity: float = Field(..., title="Thermal conductivity [W/(m.K)]", alias="k")
    specific_heat_capacity: float = Field(..., title="Specific thermal capacity [J/(kg.K)]", alias="c")
    density: float = Field(..., title="Density [kg/m3]", alias="rho")
    longwave_emissivity: float = Field(0.85, title="Longwave emissivity [1]", alias="epsLw")
    shortwave_emissivity: float = Field(0.65, title="Shortwave emissivity [1]", alias="epsSw")
    number_of_states: int = Field(
        3,
        ge=1,
        title="Number of states of a 0.2 m reference layer [1]",
        description="Spatial discretization of the layers (Buildings): states of a 0.2 m concrete layer, "
        "scaled with the thickness and diffusivity of each layer; 3 is the library default.",
        alias="nStaRef",
    )

    def __hash__(self) -> int:
        return hash(self.name)

    @field_validator("name")
    @classmethod
    def clean_name(cls, value: str) -> str:
        if ":" in value:
            return value.lower().replace(":", "_")
        return value

    @property
    def kind(self) -> str:
        """Solid, glass or gas: libraries model gas layers as cavities and glass panes without thermal mass."""
        return "solid"


class GlassMaterial(Material):
    solar_transmittance: list[float]
    solar_reflectance_outside_facing: list[float]
    solar_reflectance_room_facing: list[float]
    infrared_transmissivity: float
    infrared_absorptivity_outside_facing: float
    infrared_absorptivity_room_facing: float

    @property
    def kind(self) -> str:
        return "glass"


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
    def kind(self) -> str:
        return "gas"

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

    @computed_field  # type: ignore
    @property
    def total_thermal_capacitance(self) -> float:
        return sum([layer.thermal_capacitance for layer in self.layers])

    @property
    def internal_heat_capacity(self) -> float:
        """Areal heat capacity [J/(m2.K)] of the layers reached from the room within the effective depth.

        The simplified effective thickness of ISO 13786: the layers from the inside (the last layer)
        down to 0.1 m, the last one counted in proportion. Used to class a zone's thermal mass.
        """
        remaining = EFFECTIVE_DEPTH
        capacity = 0.0
        for layer in reversed(self.layers):
            thickness = min(layer.thickness, remaining)
            capacity += thickness * layer.material.density * layer.material.specific_heat_capacity
            remaining -= thickness
            if remaining <= 0:
                break
        return capacity

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
    u_value_given: float | None = None  # [W/(m2.K)] given in the description instead of computed from the layers
    g_value_given: float | None = None  # [1]

    def __hash__(self) -> int:
        return hash(self.name)

    @model_validator(mode="after")
    def _check_layer_sequence(self) -> "Glass":
        """Glass panes and gas gaps alternate, as every library's glazing model expects."""
        kinds = [layer.layer_type for layer in self.layers]
        if (
            not kinds
            or kinds != ["glass" if index % 2 == 0 else "gas" for index in range(len(kinds))]
            or (kinds[-1] != "glass")
        ):
            raise ValueError(
                f"Glazing {self.name} must alternate glass panes and gas gaps, starting and ending with glass, "
                f"got {kinds}."
            )
        return self

    @cached_property
    def properties(self) -> GlazingProperties:
        """Solar-optical (Buildings algorithm) and thermal (EN 673, EN 410) properties of the glazing."""
        properties = GlazingProperties.from_layers(
            panes=[layer for layer in self.layers if isinstance(layer, GlassLayer)],
            gaps=[layer for layer in self.layers if isinstance(layer, GasLayer)],
        )
        return replace(properties, u_value_override=self.u_value_given, g_value_override=self.g_value_given)

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


def _space_constructions(nodes: NodeView) -> set[Construction | Glass]:
    from trano.elements.space import BaseSpace

    return {
        construction for node in nodes if isinstance(node, BaseSpace) for construction in node.template_constructions()
    }


def default_construction(nodes: NodeView) -> ConstructionData:
    from trano.elements.envelope import BaseSimpleWall

    constructions = {node.construction for node in [node_ for node_ in nodes if isinstance(node_, BaseSimpleWall)]}
    constructions |= _space_constructions(nodes)
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
    merged_constructions.update(_space_constructions(nodes))
    # Sorted by name: sets iterate in an order that differs between processes, the model must not.
    by_name = lambda item: item.name  # noqa: E731
    wall_constructions = sorted((c for c in merged_constructions if isinstance(c, Construction)), key=by_name)
    glazing = sorted((c for c in merged_constructions if isinstance(c, Glass)), key=by_name)
    materials = {layer.material for construction in merged_constructions for layer in construction.layers}
    return ConstructionData(constructions=wall_constructions, materials=sorted(materials, key=by_name), glazing=glazing)


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
