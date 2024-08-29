from typing import List

from trano.construction import Constructions
from trano.glass import Glasses
from trano.models.constants import Azimuth, Tilt
from trano.models.elements.envelope.envelope import ExternalWall, FloorOnGround, Window
from trano.models.elements.system import Occupancy
from trano.models.elements.space import Space


def two_spaces() -> List[Space]:
    space_1 = Space(
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
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
            ),
            Window(
                name="win1_1",
                surface=1,
                azimuth=Azimuth.east,
                tilt=Tilt.wall,
                width=1,
                height=1,
                construction=Glasses.double_glazing,
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        occupancy=Occupancy(name="occupancy_1"),
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
                name="floor_1", surface=10, construction=Constructions.external_wall
            ),
        ],
    )
    return [space_1, space_2]
