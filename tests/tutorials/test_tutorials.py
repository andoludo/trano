from pathlib import Path

from dotenv import load_dotenv

from trano.main import report
import pytest

load_dotenv()


path_to_yaml_configuration_folder = Path(__file__).parent


@pytest.mark.simulate
def test_first_simulation() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        path_to_yaml_configuration_folder / "first_simulation.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
        ),
    )


@pytest.mark.simulate
def test_first_model() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "first_model.yaml",
    )


@pytest.mark.simulate
def test_two_zones() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        path_to_yaml_configuration_folder / "two_zones.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="Buildings",
        ),
    )


@pytest.mark.simulate
def test_three_zones_ideal_heating() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        path_to_yaml_configuration_folder / "three_zones_ideal_heating.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="Buildings",
        ),
    )


@pytest.mark.simulate
def test_three_zones_hydronic_heating() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        path_to_yaml_configuration_folder / "three_zones_hydronic_heating.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="Buildings",
        ),
    )


@pytest.mark.simulate
def test_two_zones_ideas() -> None:
    from trano.main import simulate_model
    from trano.simulate.simulate import SimulationLibraryOptions

    simulate_model(
        path_to_yaml_configuration_folder / "two_zones_ideas.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
            library_name="IDEAS",
        ),
    )


def test_report() -> None:
    from trano.simulate.simulate import SimulationLibraryOptions

    report(
        path_to_yaml_configuration_folder / "three_zones_hydronic_heating.yaml",
        SimulationLibraryOptions(
            start_time=0,
            end_time=2 * 3600 * 24 * 7,
            tolerance=1e-4,
        ),
    )
