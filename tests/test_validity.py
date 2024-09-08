import tempfile
from pathlib import Path

from tests.conftest import is_success
from trano.data_models.conversion import convert_network
from trano.simulate.simulate import SimulationOptions, simulate


def test_three_zones_hydronic(schema: Path) -> None:
    house = Path(__file__).parent.joinpath("three_zones_hydronic.yaml")
    network = convert_network("three_zones_hydronic", house)
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
