import tempfile
from pathlib import Path

import pytest
import yaml
from linkml.validator import validate_file

from tests.conftest import _read, clean_model, is_success
from trano.data_models.conversion import convert, convert_model, convert_network
from trano.simulate.simulate import SimulationOptions, simulate


@pytest.fixture
def schema() -> Path:
    return (
        Path(__file__).parents[1].joinpath("trano", "data_models", "trano_final.yaml")
    )


@pytest.fixture
def schema_original() -> Path:
    return Path(__file__).parents[1].joinpath("trano", "data_models", "trano.yaml")


@pytest.fixture
def parameters_path() -> Path:
    return Path(__file__).parents[1].joinpath("trano", "data_models", "parameters.yaml")


@pytest.fixture
def house() -> Path:
    return Path(__file__).parents[1].joinpath("tests", "house.yaml")


def test_validate_schema() -> None:
    house = Path(__file__).parents[1].joinpath("tests", "house.yaml")
    data_model_path = (
        Path(__file__).parents[1].joinpath("trano", "data_models", "trano.yaml")
    )
    report = validate_file(house, data_model_path, "Building")
    assert report.results == []


def test_create_new_schema(
    schema: Path, schema_original: Path, parameters_path: Path
) -> None:
    trano_path = schema_original
    trano_final_path = schema
    trano = yaml.safe_load(trano_path.read_text())
    parameters = yaml.safe_load(parameters_path.read_text())
    for name, parameter in parameters.items():
        parameter.pop("classes")
        parameter__ = {}
        for k, v in parameter["attributes"].items():
            if "func" not in v:
                v.pop("alias", None)
                parameter__[k] = v
        parameter["attributes"] = parameter__
        trano["classes"][name] = parameter
    yaml.dump(trano, trano_final_path.open("w"))


def test_convert_to_json(schema: Path, house: Path) -> None:
    for target in ["ttl", "json", "rdf", "json-ld"]:
        with tempfile.NamedTemporaryFile() as temp:
            assert convert(schema, house, target, Path(temp.name))


def test_create_model_json(schema: Path, house: Path) -> None:
    model_name = "house"
    with tempfile.NamedTemporaryFile(suffix=".json") as temp:
        assert convert(schema, house, "json", Path(temp.name))
        model_ = convert_model(model_name, Path(temp.name))
        assert clean_model(model_, model_name) == set(_read(model_name))


def test_create_model_yaml(schema: Path, house: Path) -> None:
    model_name = "house"
    model_ = convert_model(model_name, house)
    assert clean_model(model_, model_name) == set(_read(model_name))


def test_simulate_model_yaml(house: Path) -> None:
    model_name = "house"
    network = convert_network(model_name, house)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)


def test_simulate_simplified_yaml() -> None:
    model_path = Path(__file__).parents[1].joinpath("tests", "simplified_house.yaml")
    network = convert_network("simplified_yaml", model_path)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)