from trano.models.elements.construction import Construction, Layer, GlassLayer, GasLayer, Glass, Material, GlassMaterial, Gas


class Materials:
    brick: Material = Material(
        name="brick",
        thermal_conductivity=0.89,
        density=1920,
        specific_heat_capacity=790,
    )

    concrete: Material = Material(
        name="concrete",
        thermal_conductivity=1.4,
        density=2240,
        specific_heat_capacity=840,
    )

    insulation_board: Material = Material(
        name="insulation_board",
        thermal_conductivity=0.03,
        density=40,
        specific_heat_capacity=1200,
    )

    gypsum_board: Material = Material(
        name="gypsum_board",
        thermal_conductivity=0.16,
        density=800,
        specific_heat_capacity=1200,
    )
    plywood: Material = Material(
        name="plywood",
        thermal_conductivity=0.12,
        density=540,
        specific_heat_capacity=1210,
    )

    steel: Material = Material(
        name="steel",
        thermal_conductivity=50.2,
        density=7850,
        specific_heat_capacity=450,
    )
    wood: Material = Material(
        name="wood",
        thermal_conductivity=0.131,
        specific_heat_capacity=1000,
        density=600,
    )


class Constructions:
    external_wall: Construction = Construction(
        name="external_wall",
        layers=[
            Layer(material=Materials.concrete, thickness=0.2),
            Layer(material=Materials.insulation_board, thickness=0.02),
            Layer(material=Materials.plywood, thickness=0.1),
        ],
    )
    test_wall: Construction = Construction(
        name="test_wall",
        layers=[
            Layer(material=Materials.concrete, thickness=0.4),
            Layer(material=Materials.insulation_board, thickness=0.2),
            Layer(material=Materials.plywood, thickness=0.4),
        ],
    )
    internal_wall: Construction = Construction(
        name="internal_wall",
        layers=[
            Layer(material=Materials.brick, thickness=0.2),
        ],
    )
    door: Construction = Construction(
        name="Door",
        layers=[
            Layer(material=Materials.wood, thickness=0.04),
        ],
    )


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
