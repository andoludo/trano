from pydantic import BaseModel


class Material(BaseModel):
    name: str
    thermal_conductivity: float
    specific_heat_capacity: float
    density: float


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


class Layer(BaseModel):
    material: Material
    thickness: float


class Construction(BaseModel):
    name: str
    layers: list[Layer]


class Constructions:
    external_wall: Construction = Construction(
        name="external_wall",
        layers=[
            Layer(material=Materials.concrete, thickness=0.2),
            Layer(material=Materials.insulation_board, thickness=0.02),
            Layer(material=Materials.plywood, thickness=0.1),
        ],
    )
    internal_wall: Construction = Construction(
        name="internal_wall",
        layers=[
            Layer(material=Materials.brick, thickness=0.2),
        ],
    )
