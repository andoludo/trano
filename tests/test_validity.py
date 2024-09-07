import tempfile
from pathlib import Path

from tests.conftest import is_success
from trano.data_models.conversion import convert_network
from trano.simulate.simulate import SimulationOptions, simulate


def test_three_zones(schema: Path) -> None:
    house = Path(__file__).parent.joinpath("three_zones.yaml")
    network = convert_network("three_zones", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_single_zone(schema: Path) -> None:
    house = Path(__file__).parent.joinpath("single_zone.yaml")
    network = convert_network("single_zone", house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)
