from pydantic import BaseModel, ConfigDict, field_validator

from trano.material import Gas, GlassMaterial


class GlassMaterials:
    id_100: GlassMaterial = GlassMaterial(
        name="id_100",
        thermal_conductivity=1,
        density=2500,
        specific_heat_capacity=840,
        solar_transmittance=[0.646],
        solar_reflectance_outside_facing=[0.062],
        solar_reflectance_room_facing=[0.063],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    id_101: GlassMaterial = GlassMaterial(
        name="id_101",
        thermal_conductivity=1,
        density=2500,
        specific_heat_capacity=840,
        solar_transmittance=[0.486],
        solar_reflectance_outside_facing=[0.053],
        solar_reflectance_room_facing=[0.053],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    id_102: GlassMaterial = GlassMaterial(
        name="id_102",
        thermal_conductivity=1,
        density=2500,
        specific_heat_capacity=840,
        solar_transmittance=[0.834],
        solar_reflectance_outside_facing=[0.075],
        solar_reflectance_room_facing=[0.075],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    id_103: GlassMaterial = GlassMaterial(
        name="id_103",
        thermal_conductivity=1,
        density=2500,
        specific_heat_capacity=840,
        solar_transmittance=[0.771],
        solar_reflectance_outside_facing=[0.070],
        solar_reflectance_room_facing=[0.070],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )
    electro_chromic: GlassMaterial = GlassMaterial(
        name="electro_chromic",
        thermal_conductivity=0.9,
        density=2500,
        specific_heat_capacity=840,
        solar_transmittance=[0.814, 0.111],
        solar_reflectance_outside_facing=[0.086, 0.179],
        solar_reflectance_room_facing=[0.086, 0.179],
        infrared_transmissivity=0,
        infrared_absorptivity_outside_facing=0.84,
        infrared_absorptivity_room_facing=0.84,
    )


class GasMaterials:
    air: Gas = Gas(
        name="Air",
        thermal_conductivity=0.025,
        density=1.2,
        specific_heat_capacity=1005,
    )
    argon: Gas = Gas(
        name="Argon",
        thermal_conductivity=0.016,
        density=1.784,
        specific_heat_capacity=520,
    )
    krypton: Gas = Gas(
        name="Krypton",
        thermal_conductivity=0.008,
        density=3.749,
        specific_heat_capacity=248,
    )
    xenon: Gas = Gas(
        name="Xenon",
        thermal_conductivity=0.005,
        density=5.9,
        specific_heat_capacity=158,
    )


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


class Glasses:
    double_glazing: Glass = Glass(
        name="double_glazing",
        u_value_frame=1.4,
        layers=[
            GlassLayer(thickness=0.003, material=GlassMaterials.id_100),
            GasLayer(thickness=0.0127, material=GasMaterials.air),
            GlassLayer(thickness=0.003, material=GlassMaterials.id_100),
        ],
    )
    simple_glazing: Glass = Glass(
        name="simple_glazing",
        u_value_frame=1.4,
        layers=[
            GlassLayer(thickness=0.003, material=GlassMaterials.id_100),
            GasLayer(thickness=0.0127, material=GasMaterials.air),
            GlassLayer(thickness=0.003, material=GlassMaterials.id_100),
        ],
    )
