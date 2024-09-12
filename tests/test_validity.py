import tempfile
from pathlib import Path

import pytest

from tests.conftest import is_success
from trano.data_models.conversion import convert_network
from trano.exceptions import WrongSystemFlowError
from trano.library.library import Library
from trano.simulate.simulate import SimulationOptions, simulate


@pytest.mark.parametrize("library_name", ["IDEAS", "Buildings"])
def test_three_zones_hydronic(schema: Path, library_name: str) -> None:
    house = Path(__file__).parent.joinpath("three_zones_hydronic.yaml")
    network = convert_network(
        "three_zones_hydronic", house, library=Library.from_configuration(library_name)
    )
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_hydronic(schema: Path) -> None:
    house = Path(__file__).parent.joinpath("single_zone_hydronic.yaml")
    network = convert_network("single_zone_hydronic", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_hydronic_weather(schema: Path) -> None:
    house = Path(__file__).parent.joinpath("single_zone_hydronic_weather.yaml")
    network = convert_network("single_zone_hydronic_weather", house)

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_air_handling_unit(schema: Path) -> None:
    house = Path(__file__).parent.joinpath("single_zone_air_handling_unit.yaml")
    network = convert_network("single_zone_air_handling_unit", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_air_handling_unit_complex_vav(schema: Path) -> None:
    house = Path(__file__).parent.joinpath(
        "single_zone_air_handling_unit_complex_vav.yaml"
    )
    network = convert_network("single_zone_air_handling_unit_complex_vav", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_air_handling_unit_wrong_flow(schema: Path) -> None:
    house = Path(__file__).parent.joinpath(
        "single_zone_air_handling_unit_wrong_flow.yaml"
    )
    network = convert_network("single_zone_air_handling_unit_wrong_flow", house)
    with pytest.raises(WrongSystemFlowError):
        network.model()


def test_single_zone_air_handling_unit_without_vav(schema: Path) -> None:
    house = Path(__file__).parent.joinpath(
        "single_zone_air_handling_unit_without_vav.yaml"
    )
    network = convert_network("single_zone_air_handling_unit_without_vav", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone_air_handling_unit_without_vav_duct_only(schema: Path) -> None:
    house = Path(__file__).parent.joinpath(
        "single_zone_air_handling_unit_without_vav_duct_only.yaml"
    )
    network = convert_network(
        "single_zone_air_handling_unit_without_vav_duct_only", house
    )
    model = Path(
        "/home/aan/Documents/trano/tests/single_zone_air_handling_unit_without_vav_duct_only.mo"
    )
    with model.open("w+") as f:
        f.write(network.model())
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


@pytest.mark.parametrize(
    "file_name",
    [
        "single_zone_hydronic_unidentified_paramer",
        "single_zone_hydronic_wrong_flow",
        "single_zone_hydronic_random_id",
        "single_zone_hydronic_unknown_id",
        "single_zone_hydronic_unknown_system",
    ],
)
def test_unexpected_configuration(schema: Path, file_name: str) -> None:
    house = Path(__file__).parent.joinpath(f"{file_name}.yaml")
    network = convert_network(file_name, house)
    network.model()
