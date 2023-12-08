import pytest

from neosim.construction import Constructions
from neosim.glass import Glasses
from neosim.model import Space, ExternalWall, Window, Azimuth, Tilt, Occupancy, Weather, FloorOnGround
from neosim.topology import Network


@pytest.fixture
def network():
    space_1 = Space(
        name="space_1",
        volume=10,
        floor_area = 10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction= Constructions.external_wall
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width= 1,
                height=1,
                construction=Glasses.double_glazing
            ),
            Window(
                name="win2_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            Window(
                name="win1_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
            Window(
                name="win2_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
        ],
    )
    space_3 = Space(
        name="space_3",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_3",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_3",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_3",
                surface=10,
                azimuth=Azimuth.east,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            Window(
                name="w4_3",
                surface=10,
                azimuth=Azimuth.east,
                construction=Glasses.double_glazing,
                tilt=Tilt.wall,
                width=1,
                height=1,
            ),
        ],
    )
    space_4 = Space(
        name="space_4",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_4", surface=10, azimuth=10, construction=Constructions.external_wall, tilt=Tilt.wall
            ),
            ExternalWall(
                name="w2_4", surface=10, azimuth=10, construction=Constructions.external_wall, tilt=Tilt.wall
            ),
            ExternalWall(
                name="w3_4", surface=10, azimuth=10, construction=Constructions.external_wall, tilt=Tilt.wall
            ),
            ExternalWall(
                name="w4_4", surface=10, azimuth=10, construction=Constructions.external_wall, tilt=Tilt.wall
            ),
        ],
    )

    network = Network()
    network.add_space(space_1)
    network.add_space(space_2)
    network.add_space(space_3)
    network.add_space(space_4)
    network.connect_spaces(space_1, space_2)
    network.connect_spaces(space_1, space_3)
    network.connect_spaces(space_1, space_4)
    network.connect_spaces(space_2, space_4)
    weather = Weather(name=f"weather")
    network.graph.add_node(weather)
    for i, space in enumerate([space_1, space_2, space_3, space_4]):
        occupancy = Occupancy(name=f"occupancy_{i}")
        network.graph.add_node(occupancy)
        network.connect_system(space, occupancy)
        network.connect_system(space, weather)
    return network


@pytest.fixture
def building_1():
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area = 50,
        height=2,
        elevation=2,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=Tilt.wall,
                construction= Constructions.external_wall
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.external_wall
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall
            ),
            FloorOnGround(
                name="floor_2",
                surface=10,
                construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width= 1,
                height=1,
                construction=Glasses.double_glazing
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=100,
        floor_area=50,
        height=2,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.north,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.south,
                construction=Constructions.external_wall,
                tilt=Tilt.wall,
            ),
            FloorOnGround(
                name="floor_1",
                surface=10,
                construction=Constructions.external_wall
            ),
        ],
    )


    network = Network()
    network.add_space(space_1)
    network.add_space(space_2)
    network.connect_spaces(space_1, space_2)

    weather = Weather(name=f"weather")
    network.graph.add_node(weather)
    for i, space in enumerate([space_1, space_2]):
        occupancy = Occupancy(name=f"occupancy_{i}")
        network.graph.add_node(occupancy)
        network.connect_system(space, occupancy)
        network.connect_system(space, weather)
    return network