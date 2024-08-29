from pathlib import Path

from trano.construction import Construction, Layer
from trano.glass import GasMaterials, Glasses
from trano.material import Material
from trano.models.constants import Azimuth, Tilt
from trano.models.elements.base import param_from_config
from trano.models.elements.controls.collector import CollectorControl
from trano.models.elements.controls.emission import EmissionControl
from trano.models.elements.controls.three_way_valve import ThreeWayValveControl
from trano.models.elements.envelope.envelope import ExternalDoor, ExternalWall, FloorOnGround, InternalElement, Window
from trano.models.elements.space import Space
from trano.models.elements.system import Weather, Valve, ThreeWayValve, TemperatureSensor, SplitValve, Radiator, Pump, \
    Occupancy, Boiler
from trano.topology import Network

BoilerParameters = param_from_config("Boiler")
OccupancyParameters = param_from_config("Occupancy")
PumpParameters = param_from_config("Pump")
RadiatorParameter = param_from_config("Radiator")
SpaceParameter = param_from_config("Space")
SplitValveParameters = param_from_config("SplitValve")
material_1 = Material(
    name="material_1",
    thermal_conductivity=0.046,
    specific_heat_capacity=940,
    density=80,
)

material_2 = Material(
    name="material_2",
    thermal_conductivity=0.9,
    specific_heat_capacity=840,
    density=975,
    epsLw=0.85,
    epsSw=0.65,
)
material_3 = Material(
    name="material_3",
    thermal_conductivity=0.036,
    specific_heat_capacity=1470,
    density=26,
    epsLw=0.8,
    epsSw=0.8,
)
material_4 = Material(
    name="material_4",
    thermal_conductivity=0.84,
    specific_heat_capacity=840,
    density=1400,
    epsLw=0.88,
    epsSw=0.55,
)
material_5 = Material(
    name="material_5",
    thermal_conductivity=0.9,
    specific_heat_capacity=840,
    density=1100,
    epsLw=0.88,
    epsSw=0.55,
)
material_6 = Material(
    name="material_6",
    thermal_conductivity=0.11,
    specific_heat_capacity=1880,
    density=550,
    epsLw=0.86,
    epsSw=0.44,
)
material_7 = Material(
    name="material_7",
    thermal_conductivity=0.131,
    specific_heat_capacity=1000,
    density=600,
)
material_8 = Material(
    name="material_8",
    thermal_conductivity=1.4,
    specific_heat_capacity=840,
    density=2100,
    epsLw=0.88,
    epsSw=0.55,
)
material_9 = Material(
    name="material_9",
    thermal_conductivity=0.6,
    specific_heat_capacity=840,
    density=975,
    epsLw=0.85,
    epsSw=0.65,
)
material_10 = Material(
    name="material_10", k=0.3, c=880, rho=850, epsLw=0.88, epsSw=0.55
)
material_11 = Material(
    name="material_11", k=0.89, c=800, rho=1920, epsLw=0.88, epsSw=0.55
)
material_12 = Material(
    name="material_12", k=1.4, c=840, rho=2100, epsLw=0.88, epsSw=0.55
)
Construction_1 = Construction(
    name="Construction_1",
    layers=[
        Layer(material=material_12, thickness=0.025),
        Layer(material=material_1, thickness=0.05),
        Layer(material=material_1, thickness=0.24),
        Layer(material=material_2, thickness=0.02),
    ],
)


construction_2 = Construction(
    name="construction_2",
    layers=[
        Layer(material=material_2, thickness=0.02),
        Layer(material=material_4, thickness=0.14),
        Layer(material=material_2, thickness=0.02),
    ],
)

construction_3 = Construction(
    name="construction_3",
    layers=[
        Layer(material=material_7, thickness=0.04),
    ],
)


construction_4 = Construction(
    name="construction_4",
    layers=[
        Layer(material=material_1, thickness=0.18),
        Layer(material=material_10, thickness=0.09),
        Layer(material=GasMaterials.air, thickness=0.02),
        Layer(material=material_1, thickness=0.05),
        Layer(material=material_11, thickness=0.14),
        Layer(material=material_9, thickness=0.01),
    ],
)

construction_5 = Construction(
    name="construction_5",
    layers=[
        Layer(material=material_2, thickness=0.02),
        Layer(material=material_4, thickness=0.14),
        Layer(material=material_2, thickness=0.02),
    ],
)

construction_6 = Construction(
    name="construction_6",
    layers=[
        Layer(material=material_8, thickness=0.2),
        Layer(material=material_3, thickness=0.2),
        Layer(material=material_3, thickness=0.02),
        Layer(material=material_5, thickness=0.05),
        Layer(material=material_6, thickness=0.1),
    ],
)


space_1 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(),
    ],
    name="space_1",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=7.07, average_room_height=3.7),
    external_boundaries=[
        ExternalDoor(
            surface=1.5,
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=construction_3,
        ),
        Window(
            surface=2 * (1.162 + 0.521),
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=6.44,
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
        Window(
            surface=(3.31 + 0.978),
            azimuth=Azimuth.east,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=7.224,
            azimuth=Azimuth.east,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
        FloorOnGround(
            surface=7.07,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        ),
    ],
)


space_2 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(),
    ],
    name="space_2",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=2.7, average_room_height=3.8),
    external_boundaries=[
        ExternalWall(
            surface=2.37,
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
        FloorOnGround(
            surface=2.7,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        ),
    ],
)

space_3 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(
            parameters=RadiatorParameter(
                nominal_heating_power_positive_for_heating=4000
            )
        ),
    ],
    name="space_3",
    occupancy=Occupancy(
        parameters=OccupancyParameters(
            occupancy="3600*{0.1,2,15,24}", heat_gain_if_occupied="0.15"
        )
    ),
    parameters=SpaceParameter(floor_area=11.3, average_room_height=2.4),
    external_boundaries=[
        FloorOnGround(
            surface=11.3,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        ),
        Window(
            surface=(0.904 + 0.4),
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=9.024,
            azimuth=Azimuth.south,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
    ],
)


space_4 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(
            parameters=RadiatorParameter(
                nominal_heating_power_positive_for_heating=4000
            )
        ),
    ],
    name="space_4",
    occupancy=Occupancy(
        parameters=OccupancyParameters(occupancy="3600*{9, 12, 17,22}")
    ),
    parameters=SpaceParameter(
        floor_area=27.11, average_room_height=3.7, temperature_initial=273.15 + 26
    ),
    external_boundaries=[
        FloorOnGround(
            surface=27.11,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        ),
        ExternalWall(
            surface=13,
            azimuth=Azimuth.north,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
        Window(
            surface=3 * (1.227 + 0.4),
            azimuth=Azimuth.north,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=8.664,
            azimuth=Azimuth.north,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
        Window(
            surface=(3.31 + 0.978),
            azimuth=Azimuth.east,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=18.024,
            azimuth=Azimuth.east,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
    ],
)
space_5 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(
            parameters=RadiatorParameter(
                nominal_heating_power_positive_for_heating=4000
            )
        ),
    ],
    name="space_5",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=11.34, average_room_height=3.7),
    external_boundaries=[
        FloorOnGround(
            surface=11.34,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        ),
        ExternalWall(
            surface=13,
            azimuth=Azimuth.north,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
        Window(
            surface=(0.693 + 1.109 + +0.769 + 0.26),
            azimuth=Azimuth.north,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=11.34,  # - (0.693 + 1.109 + +0.769 + 0.26
            azimuth=Azimuth.north,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
    ],
)


space_6 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(
            parameters=RadiatorParameter(
                nominal_heating_power_positive_for_heating=4000
            )
        ),
    ],
    name="space_6",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=5.7, average_room_height=2.4),
    external_boundaries=[
        FloorOnGround(
            surface=5.7,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        )
    ],
)

space_7 = Space(
    name="space_7",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=5.7, average_room_height=2.4),
    external_boundaries=[
        FloorOnGround(
            surface=5.7,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        )
    ],
)

space_8 = Space(
    name="space_8",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=3.139, average_room_height=6.5),
    external_boundaries=[
        FloorOnGround(
            surface=3.19,
            azimuth=Azimuth.south,
            tilt=Tilt.floor,
            construction=construction_6,
        )
    ],
)
space_9 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(),
    ],
    name="space_9",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=15.7, average_room_height=3.7),
    external_boundaries=[
        ExternalWall(
            surface=9,
            azimuth=Azimuth.north,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
        ExternalWall(
            surface=9,
            azimuth=Azimuth.south,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
        Window(
            surface=2 * (0.67 + 0.314),
            azimuth=Azimuth.east,
            tilt=Tilt.wall,
            construction=Glasses.simple_glazing,
        ),
        ExternalWall(
            surface=12.938,  # - 2 * (0.67 + 0.314)
            azimuth=Azimuth.east,
            tilt=Tilt.wall,
            construction=construction_4,
        ),
    ],
)

space_10 = Space(
    emissions=[
        Valve(control=EmissionControl()),
        Radiator(),
    ],
    name="space_10",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=11.8116, average_room_height=3.7),
    external_boundaries=[
        ExternalWall(
            surface=9,
            azimuth=Azimuth.north,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
        ExternalWall(
            surface=9,
            azimuth=Azimuth.south,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
    ],
)
space_11 = Space(
    name="space_11",
    occupancy=Occupancy(),
    parameters=SpaceParameter(floor_area=6.6, average_room_height=1.6),
    external_boundaries=[
        ExternalWall(
            surface=9,
            azimuth=Azimuth.south,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
        ExternalWall(
            surface=3.16,
            azimuth=Azimuth.east,
            tilt=Tilt.ceiling,
            construction=Construction_1,
        ),
    ],
)

internal_construction = InternalElement(
    surface=7.43,
    azimuth=10,
    construction=construction_5,
    tilt=Tilt.wall,
)


def house_model_fixture() -> Network:
    network = Network(
        name="house_model",
    )
    network.add_boiler_plate_spaces(
        [
            space_3,
            space_2,
            space_1,
            space_6,
            space_9,
            space_7,
            space_8,
            space_10,
            space_11,
            space_5,
            space_4,
        ],
        create_internal=False,
        weather=Weather(
            parameters=param_from_config("Weather")(
                path=str(
                    Path(__file__).parents[1]
                    / "resources"
                    / "BEL_VLG_Uccle.064470_TMYx.2007-2021.mos"
                )
            )
        ),
    )
    network.connect_spaces(
        space_4,
        space_5,
        InternalElement(
            surface=11.368,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_4,
        space_2,
        InternalElement(
            surface=3.64,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_4,
        space_1,
        InternalElement(
            surface=6.44,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_4,
        space_8,
        InternalElement(
            surface=8,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_1,
        space_2,
        InternalElement(
            surface=7.22,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_3,
        space_2,
        InternalElement(
            surface=7.22,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_3,
        space_6,
        InternalElement(
            surface=6.5,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_3,
        space_8,
        InternalElement(
            surface=1.5,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_6,
        space_7,
        InternalElement(
            surface=6.5,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_6,
        space_8,
        InternalElement(
            surface=5,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_7,
        space_8,
        InternalElement(
            surface=5,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_7,
        space_5,
        InternalElement(
            surface=6.5,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_10,
        space_8,
        InternalElement(
            surface=12.321,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.wall,
        ),
    )
    network.connect_spaces(
        space_10,
        space_5,
        InternalElement(
            surface=3.33,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.floor,
        ),
    )
    network.connect_spaces(
        space_9,
        space_4,
        InternalElement(
            surface=15.7,
            azimuth=10,
            construction=construction_5,
            tilt=Tilt.floor,
        ),
    )

    pump = Pump(
        control=CollectorControl(),
        parameters=PumpParameters(),
    )
    boiler = Boiler(
        control=CollectorControl(),
        parameters=BoilerParameters(nominal_heating_power=6000),
    )
    split_valve = SplitValve(
        parameters=SplitValveParameters(),
    )
    three_way_valve_control = ThreeWayValveControl()
    three_way_valve = ThreeWayValve(
        control=three_way_valve_control,
    )
    temperature_sensor = TemperatureSensor()
    for s in [
        space_3,
        space_2,
        space_1,
        space_6,
        space_9,
        space_10,
        space_5,
        space_4,
    ]:
        network.connect_systems(temperature_sensor, s.first_emission())
        network.connect_systems(s.last_emission(), split_valve)
    network.connect_systems(pump, temperature_sensor)
    network.connect_systems(three_way_valve, pump)
    network.connect_systems(boiler, three_way_valve)
    network.connect_systems(three_way_valve, split_valve)
    network.connect_systems(split_valve, boiler)
    network.connect_systems(three_way_valve_control, temperature_sensor)
    return network
