from tests.constructions.constructions import Constructions, Glasses
from trano.elements import ExternalDoor, ExternalWall, Window, param_from_config
from trano.elements.space import Space
from trano.elements.system import Occupancy
from trano.elements.types import Azimuth, Tilt

SpaceParameter = param_from_config("Space")


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
