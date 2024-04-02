from enum import Enum

from pydantic import BaseModel, ConfigDict


class GlassMaterial(BaseModel):
    thermal_conductivity: float
    solar_transmittance: list[float]
    solar_reflectance_outside_facing: list[float]
    solar_reflectance_room_facing: list[float]
    infrared_transmissivity: float
    infrared_absorptivity_outside_facing: float
    infrared_absorptivity_room_facing: float


class Gas(Enum):
    air = "Air"


class GlassMaterials:
    id_100: GlassMaterial = GlassMaterial(
        thermal_conductivity=1,
        solar_transmittance=[0.646],
        solar_reflectance_outside_facing=[0.062],
        solar_reflectance_room_facing=[0.063],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    id_101: GlassMaterial = GlassMaterial(
        thermal_conductivity=1,
        solar_transmittance=[0.486],
        solar_reflectance_outside_facing=[0.053],
        solar_reflectance_room_facing=[0.053],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    id_102: GlassMaterial = GlassMaterial(
        thermal_conductivity=1,
        solar_transmittance=[0.834],
        solar_reflectance_outside_facing=[0.075],
        solar_reflectance_room_facing=[0.075],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    id_103: GlassMaterial = GlassMaterial(
        thermal_conductivity=1,
        solar_transmittance=[0.771],
        solar_reflectance_outside_facing=[0.070],
        solar_reflectance_room_facing=[0.070],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    electro_chromic: GlassMaterial = GlassMaterial(
        thermal_conductivity=0.9,
        solar_transmittance=[0.814, 0.111],
        solar_reflectance_outside_facing=[0.086, 0.179],
        solar_reflectance_room_facing=[0.086, 0.179],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )


class GlassLayer(BaseModel):
    thickness: float
    material: GlassMaterial
    layer_type: str = "glass"


class GasLayer(BaseModel):
    model_config = ConfigDict(use_enum_values=True)
    thickness: float
    gas: Gas
    layer_type: str = "gas"


class Glass(BaseModel):
    name: str
    layers: list[GlassLayer | GasLayer]
    u_value_frame: float


class Glasses:
    double_glazing: Glass = Glass(
        name="double_glazing",
        u_value_frame=1.4,
        layers=[
            GlassLayer(thickness=0.003, material=GlassMaterials.id_100),
            GasLayer(thickness=0.0127, gas=Gas.air),
            GlassLayer(thickness=0.003, material=GlassMaterials.id_100),
        ],
    )
