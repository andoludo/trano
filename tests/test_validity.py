from pathlib import Path

import pytest

from trano.data_models.conversion import convert_network
from trano.exceptions import IncompatiblePortsError, WrongSystemFlowError


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
    with pytest.raises((ValueError, KeyError, IncompatiblePortsError)):
        network = convert_network(file_name, house)
        network.model()


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_wrong_flow",
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
