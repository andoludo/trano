from pydantic import BaseModel, ConfigDict, Field


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


class GlassMaterial(Material):
    solar_transmittance: list[float]
    solar_reflectance_outside_facing: list[float]
    solar_reflectance_room_facing: list[float]
    infrared_transmissivity: float
    infrared_absorptivity_outside_facing: float
    infrared_absorptivity_room_facing: float


class Gas(Material):
    ...
