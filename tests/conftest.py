import tempfile
from contextlib import contextmanager
from pathlib import Path

import docker
import jinja2
import pytest

from neosim.construction import Constructions
from neosim.glass import Glasses
from neosim.model import Azimuth, ExternalWall, FloorOnGround, Space, Tilt, Window
from neosim.topology import Network


@contextmanager
def create_mos_file(network: Network) -> str:
    model = network.model()
    with tempfile.NamedTemporaryFile(
        mode="w", dir=Path(__file__).parent, suffix=".mo"
    ) as temp_model_file, tempfile.NamedTemporaryFile(
        mode="w", dir=Path(__file__).parent, suffix=".mos"
    ) as temp_mos_file:
        Path(temp_model_file.name).write_text(model)
        environment = jinja2.Environment()
        template = environment.from_string(
            """
getVersion();
loadFile("/neosim/neosim/library/Neosim.mo");
loadFile("/neosim/tests/{{model_file}}");
checkModel({{model_name}});
simulate({{model_name}},startTime = 0, stopTime = 3600);
"""
        )
        mos_file = template.render(
            model_file=Path(temp_model_file.name).name, model_name=network.name
        )
        Path(temp_mos_file.name).write_text(mos_file)
        yield Path(temp_mos_file.name).name


def is_success(results: docker.models.containers.ExecResult) -> bool:
    return "The simulation finished successfully" in results.output.decode()


@pytest.fixture
def buildings_free_float_single_zone() -> Network:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
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
    network = Network(name="buildings_free_float_single_zone")
    network.add_boiler_plate_spaces([space_1])
    return network


@pytest.fixture
def buildings_free_float_two_zones() -> Network:
    space_1 = Space(
        name="space_1",
        volume=100,
        floor_area=50,
        height=2,
        elevation=2,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
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
                name="floor_1", surface=10, construction=Constructions.external_wall
            ),
        ],
    )

    network = Network(name="buildings_free_float_two_zones")
    network.add_boiler_plate_spaces([space_1, space_2])
    return network


@pytest.fixture
def buildings_free_float_three_zones() -> Network:
    space_1 = Space(
        name="space_1",
        volume=10,
        floor_area=10,
        height=10,
        elevation=10,
        external_boundaries=[
            ExternalWall(
                name="w1_1",
                surface=10,
                azimuth=Azimuth.west,
                layer_name="layer",
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
            FloorOnGround(
                name="floor_2", surface=10, construction=Constructions.external_wall
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
            FloorOnGround(
                name="floor_3", surface=10, construction=Constructions.external_wall
            ),
        ],
    )

    network = Network(name="buildings_free_float_three_zones")
    network.add_boiler_plate_spaces([space_1, space_2, space_3])
    return network
