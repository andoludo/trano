from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from tests.constructions.constructions import Constructions
from trano.elements import ExternalWall
from trano.elements.types import Tilt
from trano.exceptions import (
    IncompatiblePortsError,
    InvalidBuildingStructureError,
    InvalidModelError,
    WrongSystemFlowError,
    SystemsNotConnectedError,
    UnknownLibraryError,
)
from trano.main import create_model


def get_path(file_name: str) -> Path:
    return Path(__file__).parent.joinpath("models", file_name)


def test_single_zone_air_handling_unit_wrong_flow(schema: Path) -> None:
    house = get_path("single_zone_air_handling_unit_wrong_flow.yaml")
    network = convert_network("single_zone_air_handling_unit_wrong_flow", house)
    with pytest.raises(WrongSystemFlowError):
        network.model()


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_unidentified_paramer",
        "single_zone_hydronic_unknown_id",
        "single_zone_hydronic_unknown_system",
        "single_zone_air_handling_unit_without_vav",
        "single_zone_air_handling_unit",
        "hello_world_missing_space_parameters",
    ],
)
def test_unexpected_configuration(schema: Path, file_name: str) -> None:
    house = get_path(f"{file_name}.yaml")
    with pytest.raises((ValueError, KeyError, IncompatiblePortsError, TypeError)):
        network = convert_network(file_name, house)
        network.model()


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_random_id",
    ],
)
def test_unexpected_configuration_should_fail_but_pass_(schema: Path, file_name: str) -> None:
    # TODO: this is to be checked
    house = get_path(f"{file_name}.yaml")
    network = convert_network(file_name, house)
    network.model()


def test_single_zone_hydronic_incomplete_system(schema: Path) -> None:
    house = get_path("single_zone_hydronic_incomplete_system.yaml")
    network = convert_network("single_zone_hydronic_incomplete_system", house)
    with pytest.raises(SystemsNotConnectedError):
        network.model()


def test_unknown_library() -> None:
    house = get_path("single_zone_hydronic.yaml")
    with pytest.raises(UnknownLibraryError):
        create_model(
            house,
            library="unknown",
        )


@pytest.mark.parametrize("azimuth", [90, 180.0, -270, 360.5])
def test_azimuth_in_degrees_is_rejected(azimuth: float) -> None:
    with pytest.raises(InvalidBuildingStructureError, match="radians"):
        ExternalWall(name="w", surface=10, azimuth=azimuth, tilt=Tilt.wall, construction=Constructions.external_wall)


@pytest.mark.parametrize("azimuth", [0, 1.57, -1.57, 3.14, 4.71, -6.2831, 6.2831])
def test_azimuth_in_radians_is_accepted(azimuth: float) -> None:
    wall = ExternalWall(name="w", surface=10, azimuth=azimuth, tilt=Tilt.wall, construction=Constructions.external_wall)
    assert wall.azimuth == azimuth


def test_yaml_azimuth_in_degrees_is_rejected(schema: Path, tmp_path: Path) -> None:
    model = get_path("single_zone_hydronic.yaml").read_text().replace("azimuth: 1.57", "azimuth: 90", 1)
    house = tmp_path / "single_zone_hydronic_degrees.yaml"
    house.write_text(model)
    with pytest.raises(InvalidModelError, match=r"(?i)azimuth"):
        convert_network("single_zone_hydronic_degrees", house)
