from typing import List

from trano.construction import Constructions
from trano.glass import Glasses
from trano.models.constants import Azimuth, Tilt
from trano.models.elements.envelope import ExternalWall, FloorOnGround, Window
from trano.models.elements.space import Space
from trano.models.elements.system import Occupancy


def three_spaces(occupancy: bool = True) -> List[Space]:
    space_1 = Space(
        name="space_1",
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
                construction=Constructions.external_wall,
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
                construction=Constructions.external_wall,
            ),
            Window(
                name="win1_1",
                surface=10,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            Window(
                name="win2_1",
                surface=10,
                azimuth=Azimuth.south,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
            FloorOnGround(
                name="floor_1", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
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
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    space_3 = Space(
        name="space_3",
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
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    if occupancy:
        space_1.occupancy = Occupancy(name="occupancy_0", space_name="space_1")
        space_2.occupancy = Occupancy(name="occupancy_1", space_name="space_2")
        space_3.occupancy = Occupancy(name="occupancy_2", space_name="space_3")

    return [space_1, space_2, space_3]
