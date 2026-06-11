from tests.constructions.constructions import Constructions, Glasses
from trano.elements import ExternalDoor, ExternalWall, FloorOnGround, Window, param_from_config
from trano.elements.control import EmissionControl
from trano.elements.space import Space
from trano.elements.system import Occupancy, Radiator
from trano.elements.types import Azimuth, Tilt

SpaceParameter = param_from_config("Space")


def space_1_different_construction_types_fixture() -> Space:
    return Space(
        name="space_1",
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                tilt=Tilt.wall,
                construction=Constructions.internal_wall,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.test_wall,
            ),
            FloorOnGround(name="floor_2", surface=10, construction=Constructions.external_wall),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            Window(
                name="win1_2",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.simple_glazing,
            ),
        ],
        emissions=[
            Radiator(
                name="emission",
                variant="ideal",
                control=EmissionControl(name="emission_control"),
            )
        ],
    )


def space_with_same_properties_fixture() -> Space:
    space = Space(
        name="bed",
        parameters=SpaceParameter(floor_area=11.3, average_room_height=3.75),
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalWall(
                name="bw",
                surface=13,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            ExternalWall(
                name="bw2",
                surface=9.29,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            Window(
                name="window",
                surface=1.3,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1.3,
                construction=Glasses.double_glazing,
            ),
        ],
    )

    return space


def space_with_door_fixture() -> Space:
    return Space(
        name="door",
        parameters=SpaceParameter(floor_area=11.3, average_room_height=3.75),
        occupancy=Occupancy(name="occupancy_0"),
        external_boundaries=[
            ExternalDoor(
                name="door",
                surface=13,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.door,
            ),
            ExternalWall(
                name="wall",
                surface=9.29,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                construction=Constructions.external_wall,
            ),
            Window(
                name="window",
                surface=1.3,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1.3,
                construction=Glasses.double_glazing,
            ),
        ],
    )
