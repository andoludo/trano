from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from trano.exceptions import (
    IncompatiblePortsError,
    WrongSystemFlowError,
    SystemsNotConnectedError,
    UnknownLibraryError,
    UnknownComponentVariantError,
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
    with pytest.raises((ValueError, KeyError, IncompatiblePortsError,TypeError)):
        network = convert_network(file_name, house)
        network.model()


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_random_id",
    ],
)
def test_unexpected_configuration_should_fail_but_pass_(
    schema: Path, file_name: str
) -> None:
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


def test_unknown_variant() -> None:
    house = get_path("single_zone_hydronic_unknown_variant.yaml")
    with pytest.raises(UnknownComponentVariantError):
        create_model(
            house,
        )
