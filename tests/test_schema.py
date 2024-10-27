import json
import tempfile
from pathlib import Path

import pytest
from linkml.validator import validate_file

from tests.conftest import _read, clean_model, is_success
from trano.data_models.conversion import convert_model, convert_network
from trano.data_models.converter import converter
from trano.scripts.schema import create_final_schema
from trano.simulate.simulate import SimulationOptions, simulate


@pytest.fixture
def house() -> Path:
    return Path(__file__).parents[1].joinpath("tests", "models", "house.yaml")


def test_validate_schema() -> None:
    house = Path(__file__).parents[1].joinpath("tests", "models", "house.yaml")
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
    create_final_schema(parameters_path, trano_final_path, trano_path)
    assert trano_final_path.exists()


def test_convert_to_json(schema: Path, house: Path) -> None:
    for target in ["ttl", "json", "rdf"]:
        # TODO: why doesn't this work with json-ld
        assert converter(
            input=str(house),
            target_class="Building",
            schema=schema,
            output_format=target,
        )


def test_create_model_json(schema: Path, house: Path) -> None:
    model_name = "house"
    with tempfile.NamedTemporaryFile(suffix=".json") as temp:
        data = converter(
            input=str(house),
            target_class="Building",
            schema=schema,
            output_format="json",
        )
        Path(temp.name).write_text(json.dumps(data))
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
    model_path = (
        Path(__file__).parents[1].joinpath("tests", "models", "simplified_house.yaml")
    )
    network = convert_network("simplified_yaml", model_path)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as project_path:
        results = simulate(
            Path(project_path),
            network,
            options=SimulationOptions(end_time=3600),
        )
        assert is_success(results)
