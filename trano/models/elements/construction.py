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
