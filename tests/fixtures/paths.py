from pathlib import Path

import pytest

TESTS_DIRECTORY = Path(__file__).parents[1]


@pytest.fixture
def result_data_path() -> Path:
    return TESTS_DIRECTORY.joinpath("resources", "data.mat")


@pytest.fixture
def result_data_container_path() -> Path:
    return TESTS_DIRECTORY.joinpath("resources", "multiple_internal_walls_buildings.building_res.mat")


@pytest.fixture
def schema() -> Path:
    return TESTS_DIRECTORY.parent.joinpath("trano", "data_models", "trano_final.yaml")


@pytest.fixture
def schema_original() -> Path:
    return TESTS_DIRECTORY.parent.joinpath("trano", "data_models", "trano.yaml")


@pytest.fixture
def parameters_path() -> Path:
    return TESTS_DIRECTORY.parent.joinpath("trano", "data_models", "parameters.yaml")
