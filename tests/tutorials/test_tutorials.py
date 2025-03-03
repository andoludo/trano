from pathlib import Path

from dotenv import load_dotenv

import pytest

load_dotenv()


path_to_yaml_configuration_folder = Path(__file__).parent


@pytest.mark.simulate
def test_first_simulation() -> None:
    from trano.main import simulate_model

    simulate_model(
        path_to_yaml_configuration_folder / "first_simulation.yaml",
        library="Buildings",
        start=0,
        end=2 * 3600 * 24 * 7,
    )


def test_first_model() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "first_model.yaml",
    )


def test_first_model_other_library() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "first_model.yaml",
        library="IDEAS",
    )


def test_multi_zones() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "multi_zones.yaml",
        library="reduced_order",
    )


def test_three_zones_ideal_heating() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "three_zones_ideal_heating.yaml",
    )


def test_three_zones_hydronic_heating() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "three_zones_hydronic_heating.yaml",
        library="IDEAS",
    )


def test_ventilation() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "zone_with_ventilation.yaml",
        library="iso_13790",
    )


def test_multizones_with_pv() -> None:
    from trano.main import create_model

    create_model(
        path_to_yaml_configuration_folder / "multizones_with_pv.yaml",
        library="iso_13790",
    )
