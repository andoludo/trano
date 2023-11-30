import pytest

from neosim.model import Space, ExternalWall, Window, Azimuth
from neosim.topology import Network


@pytest.fixture
def network():
    space_1 = Space(
        name="space_1",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w2_1",
                surface=10,
                azimuth=Azimuth.north,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w3_1",
                surface=10,
                azimuth=Azimuth.east,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w4_1",
                surface=10,
                azimuth=Azimuth.south,
                layer_name="layer",
                tilt=90,
            ),
            Window(
                name="win1_1",
                surface=10,
                azimuth=Azimuth.east,
                layer_name="glass",
                tilt=90,
                width= 1,
                height=1,
            ),
            Window(
                name="win2_1",
                surface=10,
                azimuth=Azimuth.south,
                layer_name="glass",
                tilt=90,
                width=1,
                height=1,
            ),
        ],
    )
    space_2 = Space(
        name="space_2",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_2",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w2_2",
                surface=10,
                azimuth=Azimuth.south,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w3_2",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=90,
            ),
            Window(
                name="win1_2",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="glass",
                tilt=90,
                width=1,
                height=1,
            ),
            Window(
                name="win2_2",
                surface=10,
                azimuth=Azimuth.south,
                layer_name="glass",
                tilt=90,
                width=1,
                height=1,
            ),
        ],
    )
    space_3 = Space(
        name="space_3",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_3",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w2_3",
                surface=10,
                azimuth=Azimuth.north,
                layer_name="layer",
                tilt=90,
            ),
            ExternalWall(
                name="w3_3",
                surface=10,
                azimuth=Azimuth.south,
                layer_name="layer",
                tilt=90,
            ),
            Window(
                name="w4_3",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="glass",
                tilt=90,
                width=1,
                height=1,
            ),
        ],
    )
    space_4 = Space(
        name="space_4",
        volume=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_4", surface=10, azimuth=10, layer_name="layer", tilt=90
            ),
            ExternalWall(
                name="w2_4", surface=10, azimuth=10, layer_name="layer", tilt=90
            ),
            ExternalWall(
                name="w3_4", surface=10, azimuth=10, layer_name="layer", tilt=90
            ),
            ExternalWall(
                name="w4_4", surface=10, azimuth=10, layer_name="layer", tilt=90
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
    return network

