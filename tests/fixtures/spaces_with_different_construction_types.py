from trano.construction import Constructions
from trano.glass import Glasses
from trano.models.constants import Azimuth, Tilt
from trano.models.elements.envelope.external_wall import ExternalDoor, ExternalWall
from trano.models.elements.envelope.window import Window
from trano.models.elements.occupancy import Occupancy
from trano.models.elements.space import Space, SpaceParameter


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
