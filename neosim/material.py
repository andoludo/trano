from pydantic import BaseModel


class Material(BaseModel):
    name: str
    thermal_conductivity: float
    specific_heat_capacity: float
    density: float

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
